// SPDX-License-Identifier: GPL-2.0+
/*
 *  HID driver for HangWang devices not fully compliant with HID standard
 *
 *  Copyright (c) 2019 Volodymyr Puiul
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/usb.h>

#include "hid-ids.h"

/* Size of the original descriptors of Parblo A609 tablet */
#define HANWANG_RDESC_PARBLO_A609_ORIG_SIZE 151

/* Fixed report descriptor for Parblo A609 tablet */
__u8 hanwang_rdesc_parblo_a609_fixed[] = {
	0x05, 0x01,         /*  Usage Page (Desktop),               */
	0x09, 0x02,         /*  Usage (Mouse),                      */
	0xA1, 0x01,         /*  Collection (Application),           */
	0x85, 0x01,         /*      Report ID (1),                  */
	0x09, 0x01,         /*      Usage (Pointer),                */
	0xA1, 0x00,         /*      Collection (Physical),          */
	0x05, 0x09,         /*          Usage Page (Button),        */
	0x19, 0x01,         /*          Usage Minimum (01h),        */
	0x29, 0x03,         /*          Usage Maximum (03h),        */
	0x15, 0x00,         /*          Logical Minimum (0),        */
	0x25, 0x01,         /*          Logical Maximum (1),        */
	0x75, 0x01,         /*          Report Size (1),            */
	0x95, 0x03,         /*          Report Count (3),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0x75, 0x05,         /*          Report Size (5),            */
	0x95, 0x01,         /*          Report Count (1),           */
	0x81, 0x01,         /*          Input (Constant),           */
	0x05, 0x01,         /*          Usage Page (Desktop),       */
	0x09, 0x30,         /*          Usage (X),                  */
	0x16, 0x00, 0x00,   /*          Logical Minimum (0),        */
	0x26, 0x50, 0x57,   /*          Logical Maximum (22352),    */
	0x36, 0x00, 0x00,   /*          Physical Minimum (0),       */
	0x46, 0x50, 0x57,   /*          Physical Maximum (22352),   */
	0x75, 0x10,         /*          Report Size (16),           */
	0x95, 0x01,         /*          Report Count (1),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0x09, 0x31,         /*          Usage (Y),                  */
	0x16, 0x00, 0x00,   /*          Logical Minimum (0),        */
	0x26, 0x92, 0x36,   /*          Logical Maximum (13970),    */
	0x36, 0x00, 0x00,   /*          Physical Minimum (0),       */
	0x46, 0x92, 0x36,   /*          Physical Maximum (13970),   */
	0x75, 0x10,         /*          Report Size (16),           */
	0x95, 0x01,         /*          Report Count (1),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0xC0,               /*      End Collection,                 */
	0xC0,               /*  End Collection,                     */

	0x05, 0x0D,         /*  Usage Page (Digitizer),             */
	0x09, 0x02,         /*  Usage (Pen),                        */
	0xA1, 0x01,         /*  Collection (Application),           */
	0x85, 0x02,         /*      Report ID (2),                  */
	0x09, 0x20,         /*      Usage (Stylus),                 */
	0xA0,               /*      Collection (Physical),          */
	0x14,               /*          Logical Minimum (0),        */
	0x25, 0x01,         /*          Logical Maximum (1),        */
	0x75, 0x01,         /*          Report Size (1),            */
	0x09, 0x42,         /*          Usage (Tip Switch),         */
	0x09, 0x00,         /*          Usage (Undefined)           */
	0x09, 0x44,         /*          Usage (Barrel Switch),      */
	0x09, 0x00,         /*          Usage (Undefined)           */
	0x09, 0x00,         /*          Usage (Undefined)           */
	0x09, 0x00,         /*          Usage (Undefined)           */
	0x09, 0x32,         /*          Usage (In Range),           */
	0x09, 0x00,         /*          Usage (Undefined)           */
	0x14,               /*          Logical Minimum (0),        */
	0x25, 0x01,         /*          Logical Maximum (1),        */
	0x95, 0x08,         /*          Report Count (8),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0x95, 0x01,         /*          Report Count (1),           */
	0x75, 0x10,         /*          Report Size (16),           */
	0xA4,               /*          Push,                       */
	0x05, 0x01,         /*          Usage Page (Desktop),       */
	0x65, 0x33,         /*          Unit (Inch),                */
	0x55, 0xFD,         /*          Unit Exponent (-3),         */
	0x34,               /*          Physical Minimum (0),       */
	0x09, 0x30,         /*          Usage (X),                  */
	0x46, 0x28, 0x23,   /*          Physical Maximum (9000),    */
	0x26, 0x50, 0x57,   /*          Logical Maximum (22352),    */
	0x81, 0x02,         /*          Input (Variable),           */
	0x09, 0x31,         /*          Usage (Y),                  */
	0x46, 0x70, 0x17,   /*          Physical Maximum (6000),    */
	0x26, 0x92, 0x36,   /*          Logical Maximum (13970),    */
	0x81, 0x02,         /*          Input (Variable),           */
	0xB4,               /*          Pop,                        */
	0x09, 0x30,         /*          Usage (Tip Pressure),       */
	0x26, 0xFF, 0x03,   /*          Logical Maximum (1024),     */
	0x75, 0x10,         /*          Report Size (16),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0x55, 0xFF,         /*          Unit Exponent (-1),         */
	0x65, 0x44,         /*          Unit (Degrees),             */
	0x26, 0x3F, 0x00,   /*          Logical Maximum (64),       */
	0x36, 0xA8, 0xFD,   /*          Physical Minimum (-600),    */
	0x46, 0x58, 0x02,   /*          Physical Maximum (600),     */
	0x75, 0x08,         /*          Report Size (8),            */
	0x09, 0x3D,         /*          Usage (X Tilt),             */
	0x09, 0x3E,         /*          Usage (Y Tilt),             */
	0x95, 0x02,         /*          Report Count (2),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0xC0,               /*      End Collection,                 */
	0xC0,               /*  End Collection                      */

	0x05, 0x01,         /*  Usage Page (Desktop),               */
	0x09, 0x07,         /*  Usage (Keypad),                     */
	0xA1, 0x01,         /*  Collection (Application),           */
	0x85, 0x0C,         /*      Report ID (12),                 */
	0x05, 0x0D,         /*      Usage Page (Digitizer),         */
	0x09, 0x39,         /*      Usage (Tablet Function Keys),   */
	0xA0,               /*      Collection (Physical),          */
	0x14,               /*          Logical Minimum (0),        */
	0x25, 0x01,         /*          Logical Maximum (1),        */
	0x05, 0x09,         /*          Usage Page (Button),        */
	0x75, 0x01,         /*          Report Size (1),            */
	0x95, 0x10,         /*          Report Count (10),          */
	0x81, 0x03,         /*          Input (Constant, Variable), */
	0x19, 0x10,         /*          Usage Minimum (10h),        */
	0x29, 0x13,         /*          Usage Maximum (13h),        */
	0x95, 0x04,         /*          Report Count (4),           */
	0x81, 0x02,         /*          Input (Variable),           */
	0x95, 0x34,         /*          Report Count (52),          */
	0x81, 0x03,         /*          Input (Constant, Variable), */
	0xC0,               /*      End Collection,                 */
	0xC0                /*  End Collection                      */
};

static __u8 *hanwang_report_fixup(struct hid_device *hdev, __u8 *rdesc,
								  unsigned int *rsize)
{
	switch (hdev->product)
	{
	case USB_DEVICE_ID_PARBLO_A609:
	case USB_DEVICE_ID_SIGNOTEC_VIEWSONIC_PD1011:
		if (*rsize == HANWANG_RDESC_PARBLO_A609_ORIG_SIZE)
		{
			rdesc = hanwang_rdesc_parblo_a609_fixed;
			*rsize = sizeof(hanwang_rdesc_parblo_a609_fixed);
		}
		break;
	}

	return rdesc;
}

/**
 * hanwang_init() - initialize a HanWang tablets interface
 * By default device are initialized in mouse mode and send only
 * reportd with ID #1. To switch device into tablet mode we need to
 * send feature request. Only after this device starts sending reports
 * which contains information about pressure and pen inclination.
 *
 * @hdev:	The HID device of the tablet interface to initialize.
 * 		Cannot be NULL.
 *
 * Returns:
 *	Zero, if successful. A negative errno code on error.
 */
static int hanwang_init(struct hid_device *hdev)
{
	// send feature request to enable digitizer functionality
	// after this device switch to tablet mode and sends
	// reports with ID 2
	u8 buf[0x2] = {0x02, 0x02};
	int ret;
	ret = hid_hw_raw_request(hdev,
							 0x02,
							 buf,
							 2,
							 HID_FEATURE_REPORT,
							 HID_REQ_SET_REPORT);
	if (ret < 0)
	{
		hid_err(hdev, "Can't set operational mode\n");
		return ret;
	}
	return 0;
}

static int hanwang_raw_event(struct hid_device *hdev,
							 struct hid_report *report,
							 u8 *data, int size)
{
	u16 pos_x;
	u16 pos_y;
	u16 pressure;
	u8 tiltX;
	u8 tiltY;

	/* Tweak pen reports*/
	if (report->type == HID_INPUT_REPORT &&
		report->id == 2)
	{

		pos_x = data[3] | data[2] << 8; // swapped bytes
		pos_y = data[5] | data[4] << 8; // swapped bytes

		// 10 bits for pressure
		pressure = ((data[6] << 2) | ((data[7] & 0xC0) >> 6));

		// 6 bits for tiltX
		tiltX = (data[7] & 0x3F);

		// 6 bits of tiltY
		tiltY = (data[8] & 0x7F) >> 1;

		// data[9] contains some counter

		// override tip button value with pressure treshold
		// because some jitter present
		if (pressure > 0x20)
		{
			data[1] |= 0x1;
		}

		// refill report with corrected values in accordance to
		// fixed HID report descriptor

		// pos X - 16 bits
		data[2] = pos_x & 0xFF;
		data[3] = pos_x >> 8;

		// pos Y - 16 bits
		data[4] = pos_y & 0xFF;
		data[5] = pos_y >> 8;

		// pressure - 10 bits
		// 8 lsb in data[6]
		// 2 msb in data[7]
		data[6] = pressure & 0xFF;
		data[7] = (pressure >> 8);

		// tiltX - 6 bits
		data[8] = tiltX;

		// tiltY - 6 bits
		data[9] = tiltY;
		// hid_err(hdev, "fixed data: X=>%d; Y=>%d; pressure=>%04d; tiltX=>%03d; tiltY=>%03d\n", pos_x, pos_y, pressure, tiltX, tiltY);
	}
	return 0;
}

static int hanwang_probe(struct hid_device *hdev,
						 const struct hid_device_id *id)
{
	int rc;

	rc = hid_parse(hdev);
	if (rc)
	{
		hid_err(hdev, "parse failed\n");
		return rc;
	}

	rc = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (rc)
	{
		hid_err(hdev, "hw start failed\n");
		return rc;
	}

	rc = hanwang_init(hdev);
	if (rc)
	{
		hid_err(hdev, "switch to full mode failed\n");
		return rc;
	}

	return 0;
}

static void hanwang_remove(struct hid_device *hdev)
{
	hid_hw_stop(hdev);
}

#ifdef CONFIG_PM
static int hanwang_resume(struct hid_device *hdev)
{
	int rc;

	/* Re-initialize the device*/
	rc = hanwang_init(hdev);
	if (rc != 0)
		hid_err(hdev, "failed to re-initialize the device\n");

	return rc;
}
#endif

static const struct hid_device_id hanwang_devices[] = {
	{HID_USB_DEVICE(USB_VENDOR_ID_HANWANG,
					USB_DEVICE_ID_PARBLO_A609)},
	{HID_USB_DEVICE(USB_VENDOR_ID_HANWANG,
					USB_DEVICE_ID_INTEY_NY_BG14)},
	{}};
MODULE_DEVICE_TABLE(hid, hanwang_devices);

static struct hid_driver hanwang_driver = {
	.name = "hanwang",
	.id_table = hanwang_devices,
	.probe = hanwang_probe,
	.remove = hanwang_remove,
	.raw_event = hanwang_raw_event,
	.report_fixup = hanwang_report_fixup,
#ifdef CONFIG_PM
	.resume = hanwang_resume,
	.reset_resume = hanwang_resume,
#endif
};
module_hid_driver(hanwang_driver);

MODULE_LICENSE("GPL");
MODULE_VERSION("10");
