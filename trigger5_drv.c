// SPDX-License-Identifier: GPL-2.0-only

#include <linux/iosys-map.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_modeset_helper_vtables.h>

#include "trigger5.h"

static int trigger5_usb_suspend(struct usb_interface *interface,
				pm_message_t message)
{
	struct trigger5_device *trigger5 = usb_get_intfdata(interface);

	return drm_mode_config_helper_suspend(&trigger5->drm);
}

static int trigger5_usb_resume(struct usb_interface *interface)
{
	struct trigger5_device *trigger5 = usb_get_intfdata(interface);

	return drm_mode_config_helper_resume(&trigger5->drm);
}

DEFINE_DRM_GEM_FOPS(trigger5_driver_fops);

static const struct drm_driver trigger5_drm_driver = {
	.driver_features = DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,

	/* GEM hooks */
	.fops = &trigger5_driver_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	DRM_FBDEV_SHMEM_DRIVER_OPS,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static const struct drm_mode_config_funcs trigger5_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static u64 trigger5_calculate_pll(struct trigger5_pll *pll, int clock)
{
	u64 ref_clock = 10000000;
	u64 target_clock = (u64)clock * 1000;
	u64 calculated_clock, calculated_err, best_err = U64_MAX;
	int prediv, mul1, mul2, div1, div2;

	/* Use values found in the capture */
	for (prediv = 1; prediv <= 0x10; prediv <<= 1) {
		for (mul1 = 1; mul1 <= 0x32; mul1++) {
			for (mul2 = 1; mul2 <= 0x32; mul2++) {
				for (div1 = 1; div1 <= 0x32; div1++) {
					for (div2 = 0x02; div2 <= 0x10;
					     div2 <<= 1) {
						calculated_clock =
							div_u64(ref_clock * mul1 * mul2,
								prediv * div1 * div2);
						calculated_err =
							abs_diff(calculated_clock,
								 target_clock);
						if (calculated_err < best_err) {
							best_err =
								calculated_err;
							pll->mul1 = mul1;
							pll->mul2 = mul2;
							pll->div1 = div1;
							pll->div2 = div2;
							pll->prediv = prediv;
						}
					}
				}
			}
		}
	}
	return best_err;
}

static void trigger5_bulk_timeout(struct timer_list *t)
{
	struct trigger5_transfer *transfer =
		timer_container_of(transfer, t, timer);

	usb_sg_cancel(&transfer->sgr);
}

static void trigger5_transfer_work(struct work_struct *work)
{
	struct trigger5_transfer *transfer =
		container_of(work, struct trigger5_transfer, transfer_work);
	struct trigger5_device *trigger5 = transfer->trigger5;
	struct usb_device *usbdev;
	int idx, ret;

	if (!drm_dev_enter(&trigger5->drm, &idx))
		goto complete;

	usbdev = interface_to_usbdev(trigger5->intf);

	/* Submit bulk transfer with a five-second timeout. */
	ret = usb_sg_init(&transfer->sgr, usbdev, trigger5->bulk_pipe, 0,
			  transfer->transfer_sgt.sgl,
			  transfer->transfer_sgt.nents, transfer->frame_len,
			  GFP_KERNEL);
	if (ret) {
		drm_err_ratelimited(&trigger5->drm,
				    "failed to initialize USB transfer: %d\n",
				    ret);
		goto exit;
	}

	mod_timer(&transfer->timer, jiffies + msecs_to_jiffies(5000));
	usb_sg_wait(&transfer->sgr);
	timer_delete_sync(&transfer->timer);

	if (transfer->sgr.status)
		drm_err_ratelimited(&trigger5->drm,
				    "USB transfer failed: %d\n",
				    transfer->sgr.status);
	else if (transfer->sgr.bytes != transfer->frame_len)
		drm_err_ratelimited(&trigger5->drm,
				    "short USB transfer: %zu/%zu bytes\n",
				    transfer->sgr.bytes, transfer->frame_len);

exit:
	drm_dev_exit(idx);
complete:
	complete(&transfer->frame_complete);
}

static void trigger5_free_bulk_buffer(struct trigger5_transfer *transfer)
{
	if (!transfer->frame_data)
		return;
	sg_free_table(&transfer->transfer_sgt);
	vfree(transfer->frame_data);
	transfer->frame_data = NULL;
	transfer->frame_len = 0;
	transfer->frame_alloc_len = 0;
}

static int trigger5_resize_bulk_buffer(struct trigger5_transfer *transfer,
				       size_t len)
{
	unsigned int num_pages;
	struct sg_table transfer_sgt;
	int ret, i;
	struct page **pages;
	u8 *data;
	void *ptr;

	/* Large transfer buffer requires vmalloc and a scatterlist. */
	data = vmalloc_32(len);
	if (!data)
		return -ENOMEM;

	num_pages = DIV_ROUND_UP(len, PAGE_SIZE);
	pages = kmalloc_array(num_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto err_vfree;
	}
	for (i = 0, ptr = data; i < num_pages; i++, ptr += PAGE_SIZE)
		pages[i] = vmalloc_to_page(ptr);
	ret = sg_alloc_table_from_pages(&transfer_sgt, pages,
					num_pages, 0, len, GFP_KERNEL);
	kfree(pages);
	if (ret)
		goto err_vfree;

	/* Allocate a replacement before releasing the current buffer. */
	sg_free_table(&transfer->transfer_sgt);
	transfer->transfer_sgt = transfer_sgt;
	vfree(transfer->frame_data);
	transfer->frame_data = data;
	transfer->frame_alloc_len = len;

	return 0;
err_vfree:
	vfree(data);
	return ret;
}

static void trigger5_init_transfer(struct trigger5_device *trigger5,
				   struct trigger5_transfer *transfer)
{
	init_completion(&transfer->frame_complete);
	complete(&transfer->frame_complete);
	timer_setup(&transfer->timer, trigger5_bulk_timeout, 0);
	INIT_WORK(&transfer->transfer_work, trigger5_transfer_work);
	transfer->trigger5 = trigger5;
}

static void trigger5_crtc_atomic_enable(struct drm_crtc *crtc,
					struct drm_atomic_commit *state)
{
	struct trigger5_device *trigger5 = to_trigger5(crtc->dev);
	struct usb_device *udev;
	struct drm_crtc_state *crtc_state =
		drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_display_mode *mode = &crtc_state->mode;
	struct trigger5_mode_request request = {};
	u8 data[4];
	u64 clk;
	int idx, ret;

	if (!drm_dev_enter(crtc->dev, &idx))
		return;

	udev = interface_to_usbdev(trigger5->intf);

	/* Sequence recovered from USB captures. */
	ret = usb_control_msg_recv(udev, 0,
				   TRIGGER5_REQUEST_FIRMWARE_RESET,
				   USB_DIR_IN | USB_TYPE_VENDOR |
					   USB_RECIP_DEVICE,
				   0x0000, 0x0000, data, 1,
				   USB_CTRL_GET_TIMEOUT, GFP_KERNEL);
	if (ret)
		goto err;

	request.height = cpu_to_be16(mode->vdisplay);
	request.height_minus_one = cpu_to_be16(mode->vdisplay - 1);
	request.width = cpu_to_be16(mode->hdisplay);
	request.width_minus_one = cpu_to_be16(mode->hdisplay - 1);

	request.line_total_pixels = cpu_to_be16(mode->htotal - 1);
	request.line_sync_pulse =
		cpu_to_be16(mode->hsync_end - mode->hsync_start - 1);
	request.line_back_porch =
		cpu_to_be16(mode->htotal - mode->hsync_end - 1);

	request.frame_total_lines = cpu_to_be16(mode->vtotal - 1);
	request.frame_sync_pulse =
		cpu_to_be16(mode->vsync_end - mode->vsync_start - 1);
	request.frame_back_porch =
		cpu_to_be16(mode->vtotal - mode->vsync_end - 1);
	request.unknown1 = cpu_to_be16(0xff);
	request.unknown2 = cpu_to_be16(0xff);
	request.unknown3 = cpu_to_be16(0xff);
	request.unknown4 = cpu_to_be16(0xff);

	request.hsync_polarity = (mode->flags & DRM_MODE_FLAG_PHSYNC) ? 0 : 1;
	request.vsync_polarity = (mode->flags & DRM_MODE_FLAG_PVSYNC) ? 0 : 1;

	trigger5_calculate_pll(&request.pll, mode->clock);
	clk = div_u64(10000000ULL * request.pll.mul1 * request.pll.mul2,
		      (u32)request.pll.prediv * request.pll.div1 *
			      request.pll.div2 * 1000);
	drm_dbg_kms(&trigger5->drm,
		    "pll: %02x %02x %02x %02x %02x -> %llu kHz (want %d kHz)\n",
		    request.pll.prediv, request.pll.mul1, request.pll.mul2,
		    request.pll.div1, request.pll.div2, clk, mode->clock);

	/* wValue can be any value since we are sending a custom mode */
	ret = usb_control_msg_send(udev, 0, TRIGGER5_REQUEST_SET_MODE,
				   USB_DIR_OUT | USB_TYPE_VENDOR |
					   USB_RECIP_DEVICE,
				   0, 0, &request, sizeof(request),
				   USB_CTRL_SET_TIMEOUT, GFP_KERNEL);
	if (ret)
		goto err;

	ret = usb_control_msg_recv(udev, 0,
				   TRIGGER5_REQUEST_FIRMWARE_RESET,
				   USB_DIR_IN | USB_TYPE_VENDOR |
					   USB_RECIP_DEVICE,
				   0x0201, 0x0000, data, 1,
				   USB_CTRL_GET_TIMEOUT, GFP_KERNEL);
	if (ret)
		goto err;

	ret = usb_control_msg_recv(udev, 0,
				   TRIGGER5_REQUEST_GET_REGISTER,
				   USB_DIR_IN | USB_TYPE_VENDOR |
					   USB_RECIP_DEVICE,
				   0x0000, 0xec34, data, sizeof(data),
				   USB_CTRL_GET_TIMEOUT, GFP_KERNEL);
	if (ret)
		goto err;

	data[0] = 0x60;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x10;
	ret = usb_control_msg_send(udev, 0,
				   TRIGGER5_REQUEST_SET_REGISTER,
				   USB_DIR_OUT | USB_TYPE_VENDOR |
					   USB_RECIP_DEVICE,
				   0x0000, 0xec34, data, sizeof(data),
				   USB_CTRL_SET_TIMEOUT, GFP_KERNEL);
	if (ret)
		goto err;

	data[0] = 0x01;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	ret = usb_control_msg_send(udev, 0,
				   TRIGGER5_REQUEST_SET_CURSOR_POSITION,
				   USB_DIR_OUT | USB_TYPE_VENDOR |
					   USB_RECIP_DEVICE,
				   0x0000, 0xe868, data, sizeof(data),
				   USB_CTRL_SET_TIMEOUT, GFP_KERNEL);
	if (ret)
		goto err;

	goto exit;

err:
	drm_err_ratelimited(&trigger5->drm,
			    "failed to configure display mode: %d\n", ret);
exit:
	drm_dev_exit(idx);
}

static enum drm_mode_status
trigger5_crtc_mode_valid(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode)
{
	struct trigger5_pll pll;
	size_t frame_len, payload_len;
	u64 err, ppm;

	payload_len = array3_size(mode->hdisplay, mode->vdisplay, 3);
	frame_len = size_add(payload_len, sizeof(struct trigger5_bulk_header));
	if (frame_len > SZ_16M)
		return MODE_MEM;

	if (!mode->clock)
		return MODE_CLOCK_LOW;

	err = trigger5_calculate_pll(&pll, mode->clock);
	ppm = div64_u64(err * 1000, mode->clock);
	if (ppm > 10000)
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

static int trigger5_plane_atomic_check(struct drm_plane *plane,
				       struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state =
		drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc *crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state;

	if (!new_plane_state->fb)
		return 0;
	if (!crtc)
		return -EINVAL;

	new_crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	return drm_atomic_helper_check_plane_state(new_plane_state,
						   new_crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, false);
}

static u8 trigger5_bulk_header_checksum(struct trigger5_bulk_header *header)
{
	u16 checksum = 0;
	u8 *data = (u8 *)header;
	size_t i;

	for (i = 0; i < sizeof(struct trigger5_bulk_header) - 1; i++)
		checksum += data[i];
	checksum &= 0xff;
	checksum = 0x100 - checksum;
	return checksum & 0xff;
}

static void trigger5_merge_rect(struct drm_rect *r1, struct drm_rect *r2)
{
	r1->x1 = min(r1->x1, r2->x1);
	r1->y1 = min(r1->y1, r2->y1);
	r1->x2 = max(r1->x2, r2->x2);
	r1->y2 = max(r1->y2, r2->y2);
}

static void trigger5_plane_atomic_update(struct drm_plane *plane,
					 struct drm_atomic_commit *atomic_state)
{
	struct drm_plane_state *old_state =
		drm_atomic_get_old_plane_state(atomic_state, plane);
	struct drm_plane_state *state =
		drm_atomic_get_new_plane_state(atomic_state, plane);
	struct drm_shadow_plane_state *shadow_plane_state =
		to_drm_shadow_plane_state(state);
	struct trigger5_device *trigger5 = to_trigger5(plane->dev);
	struct drm_format_conv_state fmtcnv_state = DRM_FORMAT_CONV_STATE_INIT;
	struct trigger5_transfer *current_transfer, *previous_transfer;
	struct trigger5_bulk_header *header;
	struct drm_rect current_rect, src_rect;
	struct iosys_map data_map;
	size_t frame_len, payload_len, max_len;
	int width, height;
	int idx, ret;

	if (!drm_atomic_helper_damage_merged(old_state, state, &current_rect))
		return;

	if (!drm_dev_enter(plane->dev, &idx))
		return;

	current_transfer =
		&trigger5->transfers[trigger5->current_transfer];
	previous_transfer = &trigger5->transfers[1 - trigger5->current_transfer];

	src_rect = drm_plane_state_src(state);

	/* Match drm_atomic_helper_damage_iter_init() rounding. */
	src_rect.x1 >>= 16;
	src_rect.y1 >>= 16;
	src_rect.x2 = (src_rect.x2 >> 16) + !!(src_rect.x2 & 0xffff);
	src_rect.y2 = (src_rect.y2 >> 16) + !!(src_rect.y2 & 0xffff);

	/* Latency reduction: requeue with the latest frame data. */
	if (cancel_work(&previous_transfer->transfer_work)) {
		complete(&previous_transfer->frame_complete);

		trigger5_merge_rect(&current_rect, &previous_transfer->transfer_rect);

		/* Clip merged damage to the new resolution. */
		if (!drm_rect_intersect(&current_rect, &src_rect))
			goto exit;

		current_transfer = previous_transfer;
		trigger5->current_transfer = !trigger5->current_transfer;
	}

	width = drm_rect_width(&current_rect);
	height = drm_rect_height(&current_rect);
	payload_len = array3_size(width, height, 3);
	frame_len = size_add(payload_len, sizeof(*header));
	current_transfer->transfer_rect = current_rect;

	if (!wait_for_completion_timeout(&current_transfer->frame_complete,
					 msecs_to_jiffies(10)))
		goto exit;

	/* Resize buffer to the current resolution. */
	max_len = array3_size(drm_rect_width(&src_rect),
			      drm_rect_height(&src_rect), 3);
	max_len = size_add(max_len, sizeof(*header));
	/*
	 * Allocation failure leaves the old buffer available for smaller
	 * partial updates.
	 */
	if (max_len != current_transfer->frame_alloc_len)
		trigger5_resize_bulk_buffer(current_transfer, max_len);

	if (frame_len > current_transfer->frame_alloc_len)
		goto exit;

	current_transfer->frame_len = frame_len;
	header = current_transfer->frame_data;
	header->magic = 0xfb;
	header->length = 0x14;
	/* flags 0: uncompressed 24-bit RGB888. */
	header->counter =
		cpu_to_le16((trigger5->frame_counter++) & 0xfff);
	header->horizontal_offset = cpu_to_le16(current_rect.x1);
	header->vertical_offset = cpu_to_le16(current_rect.y1);
	header->width = cpu_to_le16(width);
	header->height = cpu_to_le16(height);
	header->payload_length = cpu_to_le32((u32)payload_len);
	header->flags = 0x1;
	header->unknown1 = 0;
	header->unknown2 = 0;
	header->checksum = trigger5_bulk_header_checksum(header);

	iosys_map_set_vaddr(&data_map,
			    current_transfer->frame_data + sizeof(*header));

	ret = drm_gem_fb_begin_cpu_access(state->fb, DMA_FROM_DEVICE);
	if (ret < 0) {
		complete(&current_transfer->frame_complete);
		goto exit;
	}

	drm_fb_xrgb8888_to_rgb888(&data_map, NULL,
				  &shadow_plane_state->data[0],
				  state->fb, &current_rect, &fmtcnv_state);
	drm_format_conv_state_release(&fmtcnv_state);

	drm_gem_fb_end_cpu_access(state->fb, DMA_FROM_DEVICE);

	queue_work(trigger5->transfer_wq, &current_transfer->transfer_work);
	trigger5->current_transfer = !trigger5->current_transfer;

exit:
	drm_dev_exit(idx);
}

static const struct drm_crtc_helper_funcs trigger5_crtc_helper_funcs = {
	.mode_valid = trigger5_crtc_mode_valid,
	.atomic_check = drm_crtc_helper_atomic_check,
	.atomic_enable = trigger5_crtc_atomic_enable,
};

static const struct drm_crtc_funcs trigger5_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_plane_helper_funcs trigger5_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = trigger5_plane_atomic_check,
	.atomic_update = trigger5_plane_atomic_update,
};

static const struct drm_plane_funcs trigger5_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static const struct drm_encoder_funcs trigger5_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const u32 trigger5_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static int trigger5_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	int ret;
	struct trigger5_device *trigger5;
	struct usb_endpoint_descriptor *bulk_out;
	struct drm_device *dev;
	struct device *dma_dev;
	struct usb_device *udev = interface_to_usbdev(interface);
	/* Presence of audio interfaces indicates HDMI. */
	bool is_hdmi = udev->config->desc.bNumInterfaces > 1;

	trigger5 = devm_drm_dev_alloc(&interface->dev, &trigger5_drm_driver,
				      struct trigger5_device, drm);
	if (IS_ERR(trigger5))
		return PTR_ERR(trigger5);

	trigger5->intf = interface;

	ret = usb_find_bulk_out_endpoint(interface->cur_altsetting, &bulk_out);
	if (ret)
		return ret;
	trigger5->bulk_pipe =
		usb_sndbulkpipe(udev, usb_endpoint_num(bulk_out));

	dev = &trigger5->drm;

	dma_dev = usb_intf_get_dma_device(interface);
	if (dma_dev) {
		drm_dev_set_dma_dev(dev, dma_dev);
		put_device(dma_dev);
	} else {
		drm_warn(dev,
			 "buffer sharing not supported"); /* not an error */
	}

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	/*
	 * The device has a built-in mode list, however we ignore
	 * the mode list because the device accepts custom modes
	 */
	dev->mode_config.min_width = 0;
	dev->mode_config.max_width = 8191;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_height = 8191;

	dev->mode_config.funcs = &trigger5_mode_config_funcs;

	trigger5_init_transfer(trigger5, &trigger5->transfers[0]);
	trigger5_init_transfer(trigger5, &trigger5->transfers[1]);

	/* The first transfer resizes them for the active mode. */
	ret = trigger5_resize_bulk_buffer(&trigger5->transfers[0], SZ_64K);
	if (ret)
		return ret;

	ret = trigger5_resize_bulk_buffer(&trigger5->transfers[1], SZ_64K);
	if (ret)
		goto err_alloc_0;

	ret = drm_universal_plane_init(dev, &trigger5->plane, 0,
				       &trigger5_plane_funcs,
				       trigger5_plane_formats,
				       ARRAY_SIZE(trigger5_plane_formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		goto err_alloc_1;

	drm_plane_helper_add(&trigger5->plane, &trigger5_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&trigger5->plane);

	ret = drm_crtc_init_with_planes(dev, &trigger5->crtc, &trigger5->plane,
					NULL, &trigger5_crtc_funcs, NULL);
	if (ret)
		goto err_alloc_1;

	drm_crtc_helper_add(&trigger5->crtc, &trigger5_crtc_helper_funcs);

	ret = trigger5_connector_init(trigger5, is_hdmi ?
					      DRM_MODE_CONNECTOR_HDMIA :
					      DRM_MODE_CONNECTOR_VGA);
	if (ret)
		goto err_alloc_1;

	trigger5->encoder.possible_crtcs = drm_crtc_mask(&trigger5->crtc);
	ret = drm_encoder_init(dev, &trigger5->encoder, &trigger5_encoder_funcs,
			       is_hdmi ? DRM_MODE_ENCODER_TMDS :
					 DRM_MODE_ENCODER_DAC, NULL);
	if (ret)
		goto err_alloc_1;

	ret = drm_connector_attach_encoder(&trigger5->connector,
					   &trigger5->encoder);
	if (ret)
		goto err_alloc_1;

	trigger5->transfer_wq = alloc_ordered_workqueue(DRIVER_NAME, 0);
	if (!trigger5->transfer_wq) {
		ret = -ENOMEM;
		goto err_alloc_1;
	}

	drm_mode_config_reset(dev);

	usb_set_intfdata(interface, trigger5);

	drm_kms_helper_poll_init(dev);

	ret = drm_dev_register(dev, 0);
	if (ret)
		goto err_poll_fini;

	drm_client_setup(dev, NULL);

	return 0;

err_poll_fini:
	drm_kms_helper_poll_fini(dev);
	usb_set_intfdata(interface, NULL);
	destroy_workqueue(trigger5->transfer_wq);
err_alloc_1:
	trigger5_free_bulk_buffer(&trigger5->transfers[1]);
err_alloc_0:
	trigger5_free_bulk_buffer(&trigger5->transfers[0]);
	return ret;
}

static void trigger5_usb_disconnect(struct usb_interface *interface)
{
	struct trigger5_device *trigger5 = usb_get_intfdata(interface);
	struct drm_device *dev = &trigger5->drm;

	drm_kms_helper_poll_fini(dev);
	drm_dev_unplug(dev);
	drm_atomic_helper_shutdown(dev);
	destroy_workqueue(trigger5->transfer_wq);
	trigger5_free_bulk_buffer(&trigger5->transfers[0]);
	trigger5_free_bulk_buffer(&trigger5->transfers[1]);
}

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5800, 0) }, /* HDMI */
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5801, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5802, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5803, 0) },
	{ USB_DEVICE(0x0711, 0x5804) }, /* VGA */
	{ USB_DEVICE(0x0711, 0x5805) },
	{ USB_DEVICE(0x0711, 0x5806) },
	{ USB_DEVICE(0x0711, 0x5807) },
	{ USB_DEVICE(0x0711, 0x5808) },
	{ USB_DEVICE(0x0711, 0x5809) },
	{ USB_DEVICE(0x0711, 0x580A) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x580B, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x580C, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x580D, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x580E, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x580F, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5810, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5811, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5812, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5813, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5814, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5815, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5816, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5817, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5818, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5819, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x581A, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x581B, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x581C, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x581D, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x581E, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x581F, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5820, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5821, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5822, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5823, 0) },
	{ USB_DEVICE(0x0711, 0x5824) },
	{ USB_DEVICE(0x0711, 0x5825) },
	{ USB_DEVICE(0x0711, 0x5826) },
	{ USB_DEVICE(0x0711, 0x5827) },
	{ USB_DEVICE(0x0711, 0x5828) },
	{ USB_DEVICE(0x0711, 0x5829) },
	{ USB_DEVICE(0x0711, 0x582A) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x582B, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x582C, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x582D, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x582E, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x582F, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5830, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5831, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5832, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x5833, 0) },
	{ USB_DEVICE(0x0711, 0x5834) },
	{ USB_DEVICE(0x0711, 0x5835) },
	{ USB_DEVICE(0x0711, 0x5836) },
	{ USB_DEVICE(0x0711, 0x5837) },
	{ USB_DEVICE(0x0711, 0x5838) },
	{ USB_DEVICE(0x0711, 0x5839) },
	{ USB_DEVICE(0x0711, 0x583A) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x583B, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x583C, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x583D, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x583E, 0) },
	{ USB_DEVICE_INTERFACE_NUMBER(0x0711, 0x583F, 0) },
	{},
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver trigger5_driver = {
	.name = "trigger5",
	.probe = trigger5_usb_probe,
	.disconnect = trigger5_usb_disconnect,
	.suspend = trigger5_usb_suspend,
	.resume = trigger5_usb_resume,
	.reset_resume = trigger5_usb_resume,
	.id_table = id_table,
};
module_usb_driver(trigger5_driver);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
