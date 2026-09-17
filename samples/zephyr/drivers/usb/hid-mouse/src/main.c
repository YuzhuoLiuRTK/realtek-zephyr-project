/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* USB HID mouse demo combining boot-report support and remote wakeup.
 *
 * The device presents a HID mouse that supports the HID *boot interface*
 * (bInterfaceSubClass=1, bInterfaceProtocol=2) *and* is armed for remote
 * wakeup (remote-wakeup attribute in the configuration descriptor).
 *
 * Boot/report protocol switch:
 *   - Report Protocol (HID_PROTOCOL_REPORT, value 1): the 4-byte layout from
 *     the report descriptor.
 *   - Boot Protocol   (HID_PROTOCOL_BOOT, value 0):  the fixed 3-byte boot
 *     mouse report (buttons | X | Y) used by BIOS/UEFI. The host selects the
 *     protocol via SET_PROTOCOL; get_report() answers a host GET_REPORT with
 *     the matching layout.
 *
 * Remote wakeup:
 *   When the host suspends the bus, mouse activity triggers
 *   usbd_wakeup_request(), which drives a bus resume (K state). The suspend
 *   state is polled periodically (PROBE) so a lost USBSUSP interrupt can be
 *   told apart from a host that never suspended the bus.
 */

#include <sample_usbd.h>

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/*
 * Custom report descriptor. This is a deliberately non-standard mouse layout
 * to demonstrate that the interrupt report is whatever the descriptor says:
 *
 *   o  buttons : 2 bits (left/right) + 6 bits padding
 *   o  X       : 16-bit signed relative delta
 *   o  Y       : 16-bit signed relative delta
 *   o  wheel   : 8-bit signed
 *
 * Total interrupt report size: 1 + 2 + 2 + 1 = 6 bytes.
 */
static const uint8_t hid_report_desc[] = {
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_MOUSE),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
		/* 2 button bits */
		HID_USAGE_PAGE(HID_USAGE_GEN_BUTTON),
		HID_USAGE_MIN8(1),
		HID_USAGE_MAX8(2),
		HID_LOGICAL_MIN8(0),
		HID_LOGICAL_MAX8(1),
		HID_REPORT_SIZE(1),
		HID_REPORT_COUNT(2),
		HID_INPUT(0x02),
		/* 6 pad bits to align to a byte */
		HID_REPORT_SIZE(6),
		HID_REPORT_COUNT(1),
		HID_INPUT(0x03),
		/* X: 16-bit signed relative (LE) */
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
		HID_USAGE(HID_USAGE_GEN_DESKTOP_X),
		HID_LOGICAL_MIN16(0x00, 0x80),
		HID_LOGICAL_MAX16(0xFF, 0x7F),
		HID_REPORT_SIZE(16),
		HID_REPORT_COUNT(1),
		HID_INPUT(0x06),
		/* Y: 16-bit signed relative (LE) */
		HID_USAGE(HID_USAGE_GEN_DESKTOP_Y),
		HID_LOGICAL_MIN16(0x00, 0x80),
		HID_LOGICAL_MAX16(0xFF, 0x7F),
		HID_REPORT_SIZE(16),
		HID_REPORT_COUNT(1),
		HID_INPUT(0x06),
		/* wheel: 8-bit signed relative */
		HID_USAGE(HID_USAGE_GEN_DESKTOP_WHEEL),
		HID_LOGICAL_MIN8(-127),
		HID_LOGICAL_MAX8(127),
		HID_REPORT_SIZE(8),
		HID_REPORT_COUNT(1),
		HID_INPUT(0x06),
	HID_END_COLLECTION,
};

#define MOUSE_BTN_LEFT	0
#define MOUSE_BTN_RIGHT	1

enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,	/* byte 0 : 2 button bits + 6 pad bits */
	MOUSE_X_REPORT_IDX = 1,		/* bytes 1-2 : 16-bit X (LE) */
	MOUSE_Y_REPORT_IDX = 3,		/* bytes 3-4 : 16-bit Y (LE) */
	MOUSE_WHEEL_REPORT_IDX = 5,	/* byte 5 : 8-bit wheel */
	MOUSE_REPORT_COUNT = 6,
};

/* Boot mouse report is fixed at 3 bytes by the HID spec. */
#define BOOT_MOUSE_REPORT_SIZE	3

K_MSGQ_DEFINE(mouse_msgq, MOUSE_REPORT_COUNT, 4, 1);
static bool mouse_ready;

/* Current HID protocol: HID_PROTOCOL_REPORT (1) or HID_PROTOCOL_BOOT (0). */
static uint8_t protocol = HID_PROTOCOL_REPORT;

/* Last generated report, kept so a GET_REPORT from the host can return the
 * current mouse state. Byte layout matches enum mouse_report_idx.
 */
static uint8_t cur_report[MOUSE_REPORT_COUNT];

/* USB context used by the suspend-state probe and the wake-up path. */
static struct usbd_context *g_usbd;

static void input_cb(struct input_event *evt, void *user_data)
{
	static uint8_t tmp[MOUSE_REPORT_COUNT];
	static int16_t delta_x;
	static int16_t delta_y;

	ARG_UNUSED(user_data);

	switch (evt->code) {
	case INPUT_KEY_0:
		WRITE_BIT(tmp[MOUSE_BTN_REPORT_IDX], MOUSE_BTN_LEFT, evt->value);
		break;
	case INPUT_KEY_1:
		WRITE_BIT(tmp[MOUSE_BTN_REPORT_IDX], MOUSE_BTN_RIGHT, evt->value);
		break;
	case INPUT_KEY_2:
		if (evt->value) {
			delta_x += 10;
		}

		break;
	case INPUT_KEY_3:
		if (evt->value) {
			delta_y += 10;
		}

		break;
	default:
		LOG_INF("Unrecognized input code %u value %d",
			evt->code, evt->value);
		return;
	}

	/* Encode the 16-bit deltas little-endian. */
	sys_put_le16((uint16_t)delta_x, &tmp[MOUSE_X_REPORT_IDX]);
	sys_put_le16((uint16_t)delta_y, &tmp[MOUSE_Y_REPORT_IDX]);

	if (k_msgq_put(&mouse_msgq, tmp, K_NO_WAIT) != 0) {
		LOG_ERR("Failed to put new input event");
	}

	/* Keep the current state for GET_REPORT. Buttons persist; X/Y are
	 * deltas that reset to zero after each event.
	 */
	memcpy(cur_report, tmp, sizeof(cur_report));
	sys_put_le16(0U, &cur_report[MOUSE_X_REPORT_IDX]);
	sys_put_le16(0U, &cur_report[MOUSE_Y_REPORT_IDX]);

	delta_x = 0;
	delta_y = 0;
}

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

static void mouse_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("HID device %s interface is %s",
		dev->name, ready ? "ready" : "not ready");
	mouse_ready = ready;
}

static int mouse_get_report(const struct device *dev,
			    const uint8_t type, const uint8_t id,
			    const uint16_t len, uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	if (type != HID_REPORT_TYPE_INPUT) {
		LOG_WRN("Get Report: unsupported type %u", type);
		return -ENOTSUP;
	}

	/* Under the boot protocol the report must use the fixed 3-byte layout
	 * from the HID spec (X/Y are 8-bit there); otherwise the 6-byte custom
	 * layout described by the report descriptor (X/Y 16-bit).
	 */
	if (protocol == HID_PROTOCOL_BOOT) {
		uint8_t boot[BOOT_MOUSE_REPORT_SIZE];

		boot[0] = cur_report[MOUSE_BTN_REPORT_IDX];
		boot[1] = cur_report[MOUSE_X_REPORT_IDX];	/* X low byte */
		boot[2] = cur_report[MOUSE_Y_REPORT_IDX];	/* Y low byte */

		if (len < sizeof(boot)) {
			return -ENOMEM;
		}
		memcpy(buf, boot, sizeof(boot));

		LOG_INF("Get Report (Input, boot): %02x %02x %02x",
			boot[0], boot[1], boot[2]);
		return sizeof(boot);
	}

	if (len < MOUSE_REPORT_COUNT) {
		return -ENOMEM;
	}
	memcpy(buf, cur_report, MOUSE_REPORT_COUNT);

	LOG_INF("Get Report (Input, report, %u bytes): "
		"%02x %02x %02x %02x %02x %02x",
		MOUSE_REPORT_COUNT, cur_report[0], cur_report[1],
		cur_report[2], cur_report[3], cur_report[4], cur_report[5]);
	return MOUSE_REPORT_COUNT;
}

/* Called by the usbd_hid class when the host switches between Boot Protocol
 * and Report Protocol via SET_PROTOCOL.
 */
static void mouse_set_protocol(const struct device *dev, const uint8_t proto)
{
	ARG_UNUSED(dev);

	protocol = proto;
	LOG_INF("HID protocol set to %s",
		proto == HID_PROTOCOL_BOOT ? "Boot" : "Report");
}

struct hid_device_ops mouse_ops = {
	.iface_ready = mouse_iface_ready,
	.get_report = mouse_get_report,
	.set_protocol = mouse_set_protocol,
};

/* Periodically print the bus suspend state. This polls the stack flags rather
 * than relying on suspend/resume events, so that a missing UDC interrupt can
 * be told apart from a host that never suspended the bus.
 */
static void bus_state_probe(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	static bool last_suspended;
	bool suspended;

	if (g_usbd == NULL) {
		goto reschedule;
	}

	suspended = usbd_is_suspended(g_usbd);

	if (suspended != last_suspended) {
		LOG_INF("PROBE: bus state changed -> %s [usbd=%d udc=%d rwup=%d]",
			suspended ? "SUSPENDED" : "ACTIVE",
			suspended, udc_is_suspended(g_usbd->dev),
			g_usbd->status.rwup);
		last_suspended = suspended;
	} else {
		LOG_DBG("PROBE: %s [usbd=%d udc=%d rwup=%d]",
			suspended ? "SUSPENDED" : "ACTIVE",
			suspended, udc_is_suspended(g_usbd->dev),
			g_usbd->status.rwup);
	}

reschedule:
	(void)k_work_reschedule(dwork, K_SECONDS(1));
}

static K_WORK_DELAYABLE_DEFINE(probe_work, bus_state_probe);

static void msg_cb(struct usbd_context *const usbd_ctx,
		   const struct usbd_msg *const msg)
{
	LOG_INF("USBD message: %s [usbd=%d udc=%d]",
		usbd_msg_type_string(msg->type),
		usbd_is_suspended(usbd_ctx), udc_is_suspended(usbd_ctx->dev));

	if (usbd_can_detect_vbus(usbd_ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(usbd_ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(usbd_ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}
}

int main(void)
{
	struct usbd_context *sample_usbd;
	const struct device *hid_dev;
	int ret;

	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
	if (!device_is_ready(hid_dev)) {
		LOG_ERR("HID Device is not ready");
		return -EIO;
	}

	ret = hid_device_register(hid_dev, hid_report_desc,
				  sizeof(hid_report_desc), &mouse_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID Device, %d", ret);
		return ret;
	}

	sample_usbd = sample_usbd_init_device(msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(sample_usbd)) {
		ret = usbd_enable(sample_usbd);
		if (ret != 0) {
			LOG_ERR("Failed to enable device support");
			return ret;
		}
	}

	LOG_INF("HID mouse sample started (protocol=%s, rwup_armed=%d)",
		protocol == HID_PROTOCOL_BOOT ? "Boot" : "Report",
		sample_usbd->status.rwup);

	g_usbd = sample_usbd;
	(void)k_work_reschedule(&probe_work, K_SECONDS(1));

	while (true) {
		static uint8_t boot_report[BOOT_MOUSE_REPORT_SIZE];
		UDC_STATIC_BUF_DEFINE(report, MOUSE_REPORT_COUNT);

		k_msgq_get(&mouse_msgq, &report, K_FOREVER);

		if (!mouse_ready) {
			LOG_INF("USB HID device is not ready");
			continue;
		}

		if (usbd_is_suspended(sample_usbd)) {
			/* Bus suspended: wake it with K state instead of
			 * sending the report blindly.
			 */
			ret = usbd_wakeup_request(sample_usbd);
			if (ret) {
				LOG_ERR("Remote wakeup error, %d", ret);
			} else {
				LOG_INF("Remote wakeup requested");
			}
			continue;
		}

		/* Under the boot protocol the report must use the fixed 3-byte
		 * layout from the HID spec instead of the 6-byte custom
		 * Report-Protocol layout defined by the report descriptor.
		 */
		if (protocol == HID_PROTOCOL_BOOT) {
			boot_report[0] = report[MOUSE_BTN_REPORT_IDX];
			boot_report[1] = report[MOUSE_X_REPORT_IDX];
			boot_report[2] = report[MOUSE_Y_REPORT_IDX];

			ret = hid_device_submit_report(hid_dev,
					BOOT_MOUSE_REPORT_SIZE, boot_report);
		} else {
			ret = hid_device_submit_report(hid_dev,
					MOUSE_REPORT_COUNT, report);
		}

		if (ret) {
			LOG_ERR("HID submit report error, %d", ret);
		}
	}

	return 0;
}
