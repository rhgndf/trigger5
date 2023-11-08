
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>

#include "trigger5.h"

static int trigger5_read_edid(void *data, u8 *buf, unsigned int block,
			      size_t len)
{
	struct trigger5_device *trigger5 = data;
	struct usb_device *udev = interface_to_usbdev(trigger5->intf);
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			      TRIGGER5_REQUEST_GET_EDID,
			      USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      block, 0, buf, len, USB_CTRL_GET_TIMEOUT);

	if (ret < 0)
		return ret;

	return 0;
}

static int trigger5_connector_get_modes(struct drm_connector *connector)
{
	int ret;
	struct trigger5_device *trigger5 = to_trigger5(connector->dev);
	struct edid *edid;
	edid = drm_do_get_edid(connector, trigger5_read_edid, trigger5);
	drm_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);
	kfree(edid);
	return ret;
}

static enum drm_connector_status
trigger5_detect(struct drm_connector *connector, bool force)
{
	struct trigger5_device *trigger5 = to_trigger5(connector->dev);
	struct usb_device *udev = interface_to_usbdev(trigger5->intf);
	u8 *status_buf = kmalloc(2, GFP_KERNEL);
	int ret;
	u8 status;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			      TRIGGER5_REQUEST_GET_STATUS,
			      USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      0xff, 0x3, status_buf, 2, USB_CTRL_GET_TIMEOUT);
	status = status_buf[1];
	kfree(status_buf);

	if (ret < 0)
		return connector_status_unknown;

	return status == 1 ? connector_status_connected :
			     connector_status_disconnected;
}
static const struct drm_connector_helper_funcs trigger5_connector_helper_funcs = {
	.get_modes = trigger5_connector_get_modes,
};

static const struct drm_connector_funcs trigger5_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.detect = trigger5_detect,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

int trigger5_connector_init(struct trigger5_device *trigger5,
			    int connector_type)
{
	int ret;
	drm_connector_helper_add(&trigger5->connector,
				 &trigger5_connector_helper_funcs);
	ret = drm_connector_init(&trigger5->drm, &trigger5->connector,
				 &trigger5_connector_funcs, connector_type);
	trigger5->connector.polled =
		DRM_CONNECTOR_POLL_CONNECT | DRM_CONNECTOR_POLL_DISCONNECT;
	return ret;
}