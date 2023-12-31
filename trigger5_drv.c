// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_file.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>

#include "trigger5.h"

static int trigger5_usb_suspend(struct usb_interface *interface,
				pm_message_t message)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	return drm_mode_config_helper_suspend(dev);
}

static int trigger5_usb_resume(struct usb_interface *interface)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	return drm_mode_config_helper_resume(dev);
}

/*
 * FIXME: Dma-buf sharing requires DMA support by the importing device.
 *        This function is a workaround to make USB devices work as well.
 *        See todo.rst for how to fix the issue in the dma-buf framework.
 */
static struct drm_gem_object *
trigger5_driver_gem_prime_import(struct drm_device *dev,
				 struct dma_buf *dma_buf)
{
	struct trigger5_device *trigger5 = to_trigger5(dev);

	if (!trigger5->dmadev)
		return ERR_PTR(-ENODEV);

	return drm_gem_prime_import_dev(dev, dma_buf, trigger5->dmadev);
}

DEFINE_DRM_GEM_FOPS(trigger5_driver_fops);

static const struct drm_driver driver = {
	.driver_features = DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,

	/* GEM hooks */
	.fops = &trigger5_driver_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	.gem_prime_import = trigger5_driver_gem_prime_import,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static const struct drm_mode_config_funcs trigger5_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const int trigger5_get_mode(struct trigger5_device *trigger5,
				   const struct drm_display_mode *mode)
{
	unsigned int num_modes =
		min((u16)52, be16_to_cpu(trigger5->mode_list.count));
	unsigned int i;

	for (i = 0; i < num_modes; i++) {
		const struct trigger5_mode *trigger5_mode =
			&trigger5->mode_list.modes[i];

		if (le16_to_cpu(trigger5_mode->width) == mode->hdisplay &&
		    le16_to_cpu(trigger5_mode->height) == mode->vdisplay &&
		    trigger5_mode->hz == drm_mode_vrefresh(mode))
			return trigger5_mode->mode_number;
	}

	// Any mode is supported by setting the right parameters
	// Return the last mode number
	return trigger5->mode_list.modes[num_modes - 1].mode_number;
}

static u64 trigger5_calculate_pll(struct trigger5_pll *pll, int clock)
{
	u64 ref_clock = 10000000;
	u64 target_clock = clock * 1000;
	u64 calculated_clock, calculated_err, best_err = U64_MAX;
	int prediv, mul1, mul2, div1, div2;

	// Use values found in the capture
	for (prediv = 1; prediv <= 0x10; prediv <<= 1) {
		for (mul1 = 1; mul1 <= 0x32; mul1++) {
			for (mul2 = 1; mul2 <= 0x32; mul2++) {
				for (div1 = 1; div1 <= 0x32; div1++) {
					for (div2 = 0x02; div2 <= 0x10;
					     div2 <<= 1) {
						calculated_clock = ref_clock *
								   mul1 * mul2 /
								   prediv /
								   div1 / div2;
						calculated_err =
							abs(calculated_clock -
							    target_clock);
						if (calculated_err < best_err) {
							best_err =
								calculated_err;
							pll->mul1 = mul1;
							pll->mul2 = mul2;
							pll->div1 = div1;
							pll->div2 = div2;
							pll->unknown = prediv;
						}
					}
				}
			}
		}
	}
	return best_err;
}

static void trigger5_pipe_enable(struct drm_simple_display_pipe *pipe,
				 struct drm_crtc_state *crtc_state,
				 struct drm_plane_state *plane_state)
{
	struct trigger5_device *trigger5 = to_trigger5(pipe->crtc.dev);
	struct drm_display_mode *mode = &crtc_state->mode;
	struct trigger6_mode_request *request;
	u8 mode_number;
	u8 *data;

	if (crtc_state->mode_changed) {
		// Sequence cloned from captures
		data = kmalloc(4, GFP_KERNEL);
		usb_control_msg(
			interface_to_usbdev(trigger5->intf),
			usb_rcvctrlpipe(interface_to_usbdev(trigger5->intf), 0),
			0xd1, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0000, 0x0000, data, 1, USB_CTRL_GET_TIMEOUT);

		request = kmalloc(sizeof(struct trigger6_mode_request),
				  GFP_KERNEL);
		mode_number = trigger5_get_mode(trigger5, mode);

		request->height = cpu_to_be16(mode->vdisplay);
		request->height_minus_one = cpu_to_be16(mode->vdisplay - 1);
		request->width = cpu_to_be16(mode->hdisplay);
		request->width_minus_one = cpu_to_be16(mode->hdisplay - 1);

		request->line_total_pixels = cpu_to_be16(mode->htotal - 1);
		request->line_sync_pulse =
			cpu_to_be16(mode->hsync_end - mode->hsync_start - 1);
		request->line_back_porch =
			cpu_to_be16(mode->htotal - mode->hsync_end - 1);

		request->frame_total_lines = cpu_to_be16(mode->vtotal - 1);
		request->frame_sync_pulse =
			cpu_to_be16(mode->vsync_end - mode->vsync_start - 1);
		request->frame_back_porch =
			cpu_to_be16(mode->vtotal - mode->vsync_end - 1);
		request->unknown1 = 0xff;
		request->unknown2 = 0xff;
		request->unknown3 = 0xff;
		request->unknown4 = 0xff;

		request->hsync_polarity =
			(mode->flags & DRM_MODE_FLAG_PHSYNC) ? 0 : 1;
		request->vsync_polarity =
			(mode->flags & DRM_MODE_FLAG_PVSYNC) ? 0 : 1;

		trigger5_calculate_pll(&request->pll, mode->clock);
		long long int clk = 10000000LL * request->pll.mul1 *
				    request->pll.mul2 / request->pll.unknown /
				    request->pll.div1 / request->pll.div2 /
				    1000;
		drm_info(&trigger5->drm,
			 "pll: %02x %02x %02x %02x %02x %d %d\n",
			 request->pll.unknown, request->pll.mul1,
			 request->pll.mul2, request->pll.div1,
			 request->pll.div2, (int)clk, mode->clock);

		usb_control_msg(
			interface_to_usbdev(trigger5->intf),
			usb_sndctrlpipe(interface_to_usbdev(trigger5->intf), 0),
			TRIGGER5_REQUEST_SET_MODE,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			mode_number, 0, request,
			sizeof(struct trigger6_mode_request),
			USB_CTRL_SET_TIMEOUT);

		kfree(request);

		usb_control_msg(
			interface_to_usbdev(trigger5->intf),
			usb_rcvctrlpipe(interface_to_usbdev(trigger5->intf), 0),
			0xd1, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0201, 0x0000, data, 1, USB_CTRL_GET_TIMEOUT);

		usb_control_msg(
			interface_to_usbdev(trigger5->intf),
			usb_rcvctrlpipe(interface_to_usbdev(trigger5->intf), 0),
			0xa5, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0000, 0xec34, data, 4, USB_CTRL_GET_TIMEOUT);

		data[0] = 0x60;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x10;
		usb_control_msg(
			interface_to_usbdev(trigger5->intf),
			usb_sndctrlpipe(interface_to_usbdev(trigger5->intf), 0),
			0xc4, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0000, 0xec34, data, 4, USB_CTRL_SET_TIMEOUT);

		usb_control_msg(
			interface_to_usbdev(trigger5->intf),
			usb_sndctrlpipe(interface_to_usbdev(trigger5->intf), 0),
			0xc8, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0000, 0xec34, data, 4, USB_CTRL_SET_TIMEOUT);

		kfree(data);
	}
}

static void trigger5_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	//struct trigger5_device *trigger5 = to_trigger5(pipe->crtc.dev);
}

enum drm_mode_status
trigger5_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			 const struct drm_display_mode *mode)
{
	struct trigger5_pll pll;
	u64 err = trigger5_calculate_pll(&pll, mode->clock);
	u64 ppm = err * 1000000 / mode->clock;
	if (ppm > 10000) {
		return MODE_CLOCK_RANGE;
	}
	return MODE_OK;
}

int trigger5_pipe_check(struct drm_simple_display_pipe *pipe,
			struct drm_plane_state *new_plane_state,
			struct drm_crtc_state *new_crtc_state)
{
	return 0;
}

static u8 trigger5_bulk_header_checksum(struct trigger5_bulk_header *header)
{
	u16 checksum = 0;
	u8 *data = (u8 *)header;
	int i;

	for (i = 0; i < sizeof(struct trigger5_bulk_header) - 1; i++) {
		checksum += data[i];
	}
	checksum &= 0xff;
	checksum = 0x100 - checksum;
	return checksum & 0xff;
}

static void trigger5_bulk_timeout(struct timer_list *t)
{
	struct trigger5_device *trigger5 = from_timer(trigger5, t, timer);

	usb_sg_cancel(&trigger5->sgr);
}

static void trigger5_free_bulk_buffer(struct trigger5_device *trigger5)
{
	if (!trigger5->frame_data)
		return;
	sg_free_table(&trigger5->transfer_sgt);
	vfree(trigger5->frame_data);
	trigger5->frame_data = NULL;
	trigger5->frame_len = -1;
}
static int trigger5_alloc_bulk_buffer(struct trigger5_device *trigger5,
				      unsigned int len)
{
	unsigned int num_pages;
	int ret, i;
	struct page **pages;
	u8 *data;
	void *ptr;

	if (trigger5->frame_len == len) {
		return 0;
	}
	trigger5_free_bulk_buffer(trigger5);

	// Allocate buffer for bulk transfer
	// Buffer may be very large so use vmalloc and scatterlist
	data = vmalloc_32(len);
	if (!data) {
		return -ENOMEM;
	}

	num_pages = DIV_ROUND_UP(len, PAGE_SIZE);
	pages = kmalloc_array(num_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto err_vfree;
	}
	for (i = 0, ptr = data; i < num_pages; i++, ptr += PAGE_SIZE)
		pages[i] = vmalloc_to_page(ptr);
	ret = sg_alloc_table_from_pages(&trigger5->transfer_sgt, pages,
					num_pages, 0, len, GFP_KERNEL);
	kfree(pages);
	if (ret) {
		goto err_vfree;
	}

	trigger5->frame_len = len;
	trigger5->frame_data = data;

	return 0;
	sg_free_table(&trigger5->transfer_sgt);
err_vfree:
	vfree(data);
	return ret;
}

static void trigger5_transfer_work(struct work_struct *work)
{
	struct trigger5_device *trigger5 =
		container_of(work, struct trigger5_device, transfer_work);
	struct usb_device *usbdev = interface_to_usbdev(trigger5->intf);

	// Submit bulk transfer with timeout of 5 seconds
	usb_sg_init(&trigger5->sgr, usbdev, usb_sndbulkpipe(usbdev, 0x01), 0,
		    trigger5->transfer_sgt.sgl, trigger5->transfer_sgt.nents,
		    trigger5->frame_len, GFP_KERNEL);
	mod_timer(&trigger5->timer, jiffies + msecs_to_jiffies(5000));
	usb_sg_wait(&trigger5->sgr);
	del_timer_sync(&trigger5->timer);
	complete(&trigger5->frame_complete);
}

static void trigger5_pipe_update(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_shadow_plane_state *shadow_plane_state =
		to_drm_shadow_plane_state(state);
	struct drm_rect current_rect;
	struct trigger5_device *trigger5 = to_trigger5(pipe->crtc.dev);
	struct trigger5_bulk_header *header;
	int width, height, ret;
	struct iosys_map data_map;

	if (drm_atomic_helper_damage_merged(old_state, state, &current_rect)) {
		// Wait for previous frame to finish

		wait_for_completion_timeout(
			&trigger5->frame_complete, msecs_to_jiffies(1000));
		
		width = drm_rect_width(&current_rect);
		height = drm_rect_height(&current_rect);

		ret = trigger5_alloc_bulk_buffer(
			trigger5, width * height * 3 +
					  sizeof(struct trigger5_bulk_header));
		if (ret) {
			return;
		}

		// Only full screen updates work
		header = (struct trigger5_bulk_header *)trigger5->frame_data;
		header->magic = 0xfb;
		header->length = 0x14;
		header->counter = (trigger5->frame_counter++) & 0xfff;
		header->horizontal_offset = cpu_to_le16(current_rect.x1);
		header->vertical_offset = cpu_to_le16(current_rect.y1);
		header->width = cpu_to_le16(width);
		header->height = cpu_to_le16(height);
		header->payload_length = cpu_to_le32(width * height * 3);
		header->flags = 0x1;
		header->unknown1 = 0;
		header->unknown2 = 0;
		header->checksum = trigger5_bulk_header_checksum(header);

		iosys_map_set_vaddr(
			&data_map, trigger5->frame_data +
					   sizeof(struct trigger5_bulk_header));

		ret = drm_gem_fb_begin_cpu_access(state->fb, DMA_FROM_DEVICE);
		if (ret < 0) {
			return;
		}

		drm_fb_xrgb8888_to_rgb888(&data_map, NULL,
					  &shadow_plane_state->data[0],
					  state->fb, &current_rect);

		drm_gem_fb_end_cpu_access(state->fb, DMA_FROM_DEVICE);

		queue_work(system_highpri_wq, &trigger5->transfer_work);

		/*usb_control_msg(
			interface_to_usbdev(trigger5->intf),
			usb_rcvctrlpipe(interface_to_usbdev(trigger5->intf), 0),
			0x91, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0002, 0x0000, data, 1, USB_CTRL_SET_TIMEOUT);*/
	}
}

static const struct drm_simple_display_pipe_funcs trigger5_pipe_funcs = {
	.enable = trigger5_pipe_enable,
	.disable = trigger5_pipe_disable,
	.check = trigger5_pipe_check,
	.mode_valid = trigger5_pipe_mode_valid,
	.update = trigger5_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static const uint32_t trigger5_pipe_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static int trigger5_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	int ret;
	unsigned int i;
	struct trigger5_device *trigger5;
	struct drm_device *dev;
	struct usb_device *udev = interface_to_usbdev(interface);
	int min_width = 20000, min_height = 20000;
	int max_width = 0, max_height = 0;
	int cur_width, cur_height;

	trigger5 = devm_drm_dev_alloc(&interface->dev, &driver,
				      struct trigger5_device, drm);
	if (IS_ERR(trigger5))
		return PTR_ERR(trigger5);

	trigger5->intf = interface;
	dev = &trigger5->drm;

	trigger5->dmadev = usb_intf_get_dma_device(interface);
	if (!trigger5->dmadev)
		drm_warn(dev,
			 "buffer sharing not supported"); /* not an error */

	ret = drmm_mode_config_init(dev);
	if (ret)
		goto err_put_device;

	// Obtain array of supported modes
	usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			TRIGGER5_REQUEST_GET_MODE,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE, 0, 0,
			&trigger5->mode_list, sizeof(trigger5->mode_list),
			USB_CTRL_GET_TIMEOUT);

	for (i = 0; i < be16_to_cpu(trigger5->mode_list.count); i++) {
		cur_width = le16_to_cpu(trigger5->mode_list.modes[i].width);
		cur_height = le16_to_cpu(trigger5->mode_list.modes[i].height);
		min_width = min(min_width, cur_width);
		min_height = min(min_height, cur_height);
		max_width = max(max_width, cur_width);
		max_height = max(max_height, cur_height);
	}

	dev->mode_config.min_width = min_width;
	dev->mode_config.max_width = max_width;
	dev->mode_config.min_height = min_height;
	dev->mode_config.max_height = max_height;
	dev->mode_config.funcs = &trigger5_mode_config_funcs;

	trigger5->frame_counter = 0;
	trigger5->frame_len = 0;

	init_completion(&trigger5->frame_complete);
	complete(&trigger5->frame_complete);

	timer_setup(&trigger5->timer, trigger5_bulk_timeout, 0);
	INIT_WORK(&trigger5->transfer_work, trigger5_transfer_work);

	// Presence of audio interfaces = HDMI
	ret = trigger5_connector_init(trigger5,
				      udev->config->desc.bNumInterfaces > 1 ?
					      DRM_MODE_CONNECTOR_HDMIA :
					      DRM_MODE_CONNECTOR_VGA);
	if (ret)
		goto err_put_device;

	ret = drm_simple_display_pipe_init(
		&trigger5->drm, &trigger5->display_pipe, &trigger5_pipe_funcs,
		trigger5_pipe_formats, ARRAY_SIZE(trigger5_pipe_formats), NULL,
		&trigger5->connector);
	if (ret)
		goto err_put_device;

	drm_plane_enable_fb_damage_clips(&trigger5->display_pipe.plane);

	drm_mode_config_reset(dev);

	usb_set_intfdata(interface, trigger5);

	drm_kms_helper_poll_init(dev);

	ret = drm_dev_register(dev, 0);
	if (ret)
		goto err_put_device;

	drm_fbdev_generic_setup(dev, 0);

	return 0;

err_put_device:
	put_device(trigger5->dmadev);
	return ret;
}

static void trigger5_usb_disconnect(struct usb_interface *interface)
{
	struct trigger5_device *trigger5 = usb_get_intfdata(interface);
	struct drm_device *dev = &trigger5->drm;

	drm_kms_helper_poll_fini(dev);
	drm_dev_unplug(dev);
	drm_atomic_helper_shutdown(dev);
	put_device(trigger5->dmadev);
	trigger5->dmadev = NULL;
	trigger5_free_bulk_buffer(trigger5);
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
	.id_table = id_table,
};
module_usb_driver(trigger5_driver);
MODULE_LICENSE("GPL");
