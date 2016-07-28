/*
 * RTL8821AU quirks
 *
 * stupipity found around the USB3 issue
 *
 * So USB3 on RTL8821AU  is only for *power* ??
 *
 */

/*
 * Found on a MT7612U device
 * this is a 'true' USB3 wifi device
 * plugged in a USB2 HUB/Port

Binary Object Store Descriptor:
  bLength                 5
  bDescriptorType        15
  wTotalLength           22
  bNumDeviceCaps          2
  USB 2.0 Extension Device Capability:
    bLength                 7
    bDescriptorType        16
    bDevCapabilityType      2
    bmAttributes   0x0000f41e
      BESL Link Power Management (LPM) Supported
    BESL value     1024 us 
    Deep BESL value    61440 us 
  SuperSpeed USB Device Capability:
    bLength                10
    bDescriptorType        16
    bDevCapabilityType      3
    bmAttributes         0x00
    wSpeedsSupported   0x000e
      Device can operate at Full Speed (12Mbps)
      Device can operate at High Speed (480Mbps)
      Device can operate at SuperSpeed (5Gbps)
    bFunctionalitySupport   1
      Lowest fully-functional device speed is Full Speed (12Mbps)
    bU1DevExitLat          10 micro seconds
    bU2DevExitLat         180 micro seconds
can't get debug descriptor: Resource temporarily unavailable
 */
 
/*
 * This is the *only* thing I get from the RTL8821AU devices
 * which are supposed to be USB3

Device Qualifier (for other device speed):
  bLength                10
  bDescriptorType         6
  bcdUSB               2.00
  bDeviceClass            0 
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0        64
  bNumConfigurations      1

*/

#ifdef CONFIG_RTLWIFI

#include <../drivers/net/wireless/realtek/rtlwifi/wifi.h>
#include <../drivers/net/wireless/realtek/rtlwifi/base.h>

#else

#include <drv_types.h>
#include "dm.h"
#include "phy.h"
#include "reg.h"
#include "fw.h"
#include "quirks.h"

#endif

#define IS_HIGH_SPEED_USB(udev) \
		((USB_SPEED_HIGH == (udev)->usb_speed) ? true : false)

#define IS_SUPER_SPEED_USB(udev) \
		((USB_SPEED_SUPER == (udev)->usb_speed) ? true : false)


bool usb_reprobe_to_usb3(struct rtl_priv *rtlpriv)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	int ret = false;

	if (IS_HIGH_SPEED_USB(rtlusb)) {
		if ((rtl_read_byte(rtlpriv, 0x74) & (BIT(2)|BIT(3))) != BIT(3)) {
			rtl_write_byte(rtlpriv, 0x74, 0x8);
			rtl_write_byte(rtlpriv, 0x70, 0x2);
			rtl_write_byte(rtlpriv, 0x3e, 0x1);
			rtl_write_byte(rtlpriv, 0x3d, 0x3);
			/* usb disconnect */
			rtl_write_byte(rtlpriv, 0x5, 0x80);
			ret = true;
		}
	} else if (IS_SUPER_SPEED_USB(rtlusb))	{
		rtl_write_byte(rtlpriv, 0x70, rtl_read_byte(rtlpriv, 0x70) & (~BIT(1)));
		rtl_write_byte(rtlpriv, 0x3e, rtl_read_byte(rtlpriv, 0x3e) & (~BIT(0)));
	}

	return ret;
}
