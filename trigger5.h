
#ifndef trigger5_H
#define trigger5_H

#include <linux/mm_types.h>
#include <linux/scatterlist.h>
#include <linux/usb.h>

#include <drm/drm_device.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_simple_kms_helper.h>

#define DRIVER_NAME "trigger5"
#define DRIVER_DESC "Trigger 5"
#define DRIVER_DATE "20220101"

#define DRIVER_MAJOR 0
#define DRIVER_MINOR 0
#define DRIVER_PATCHLEVEL 1

struct trigger5_mode {
	u8 hz;
	u8 clock_mhz;
	u8 bpp;
	u8 mode_number;
	__le16 height;
	__le16 width;
} __attribute__((packed));

struct trigger5_mode_list {
	__be16 count;
	u8 padding[2];
	struct trigger5_mode modes[52];
} __attribute__((packed));

struct trigger5_sg_request {
	struct usb_device * usbdev;
	u8 *frame_data;
	size_t frame_len;
	struct sg_table transfer_sgt;
	struct timer_list timer;
	struct usb_sg_request sgr;
	struct work_struct transfer_work;
	struct trigger5_device *trigger5;
};

struct trigger5_device {
	struct drm_device drm;
	struct usb_interface *intf;
	struct device *dmadev;

	struct drm_connector connector;
	struct drm_simple_display_pipe display_pipe;

	struct trigger5_mode_list mode_list;

	struct trigger5_sg_request sg_requests[2];

	u16 frame_counter;

	struct completion frame_complete;
};

struct trigger5_pll {
	u8 unknown;
	u8 mul1;
	u8 mul2;
	u8 div1;
	u8 div2;
} __attribute__((packed));

struct trigger6_mode_request {
	__be16 height;
	__be16 width;
	__be16 line_total_pixels; // minus one
	__be16 line_sync_pulse; // minus one
	__be16 line_back_porch; // minus one
	__be16 unknown1;
	__be16 unknown2;
	__be16 width_minus_one;
	__be16 frame_total_lines; // minus one
	__be16 frame_sync_pulse; // minus one
	__be16 frame_back_porch; // minus one
	__be16 unknown3;
	__be16 unknown4;
	__be16 height_minus_one;
	struct trigger5_pll pll;
	u8 hsync_polarity;
	u8 vsync_polarity;
} __attribute__((packed));

struct trigger5_bulk_header {
	u8 magic; //0xfb
	u8 length; //0x14
	__le16 counter; //12 bits
	__le16 horizontal_offset;
	__le16 vertical_offset;
	__le16 width;
	__le16 height;
	__le32 payload_length; //upper 4 bits = 0x3
	u8 flags; //0
	u8 unknown1; //0
	u8 unknown2; //0
	u8 checksum;
} __attribute__((packed));

#define TRIGGER5_REQUEST_GET_MODE 0xA4
#define TRIGGER5_REQUEST_GET_STATUS 0xA6
#define TRIGGER5_REQUEST_GET_EDID 0xA8
#define TRIGGER5_REQUEST_SET_MODE 0xC3

#define to_trigger5(x) container_of(x, struct trigger5_device, drm)

int trigger5_connector_init(struct trigger5_device *trigger5,
			    int connector_type);
#endif