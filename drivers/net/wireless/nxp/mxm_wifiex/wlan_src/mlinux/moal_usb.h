/** @file moal_usb.h
 *
 * @brief This file contains definitions for USB interface.
 * driver.
 *
 *
 * Copyright 2014-2020 NXP
 *
 * This software file (the File) is distributed by NXP
 * under the terms of the GNU General Public License Version 2, June 1991
 * (the License).  You may use, redistribute and/or modify the File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 *
 */
/*************************************************************
Change Log:
    10/21/2008: initial version
************************************************************/

#ifndef _MOAL_USB_H
#define _MOAL_USB_H

#ifdef USB8997
/** USB VID 1 */
#define USB8997_VID_1 0x1286
/** USB PID 1 */
#define USB8997_PID_1 0x204D
/** USB PID 2 */
#define USB8997_PID_2 0x204E
#define USB8997_PID_3 0x2047
#define USB8997_PID_4 0x2048
#define USB8997_PID_5 0x2050
#define USB8997_PID_6 0x2051
#define USB8997V2_PID_1 0x2052
#endif /* USB8997 */

#ifdef USB8978
/** USB VID 1 */
#define USB8978_VID_1 0x1286
/** USB PID 1 */
#define USB8978_PID_1 0x2062
/** USB PID 2 */
#define USB8978_PID_2 0x2063
/* BT downloaded combo  firmware; register for WLAN enumeration */
#define USB8978_PID_1_BT 0x2064
#define USB8978_PID_2_BT 0x2065
#endif

#ifdef USB8897
/** USB VID 1 */
#define USB8897_VID_1 0x1286
/** USB PID 1 */
#define USB8897_PID_1 0x2045
/** USB PID 2 */
#define USB8897_PID_2 0x2046
#endif /* USB8897 */

#ifdef USB9098
/** USB VID 1 */
#define USB9098_VID_1 0x1286
/** USB PID 1 */
#define USB9098_PID_1 0x2056
/** USB PID 2 */
#define USB9098_PID_2 0x2057
#endif /* USB9098 */

#ifdef USB9097
/** USB VID 1 */
#define USB9097_VID_1 0x1286
/** USB PID 1 */
#define USB9097_PID_1 0x2060
/** USB PID 2 */
#define USB9097_PID_2 0x2061
#endif /* USB9097 */

/** Boot state: FW download */
#define USB_FW_DNLD 1
/** Boot state: FW ready */
#define USB_FW_READY 2

/** High watermark for Tx data */
#define MVUSB_TX_HIGH_WMARK 12

/** Number of Rx data URB */
#define MVUSB_RX_DATA_URB 6

#if defined(USB8997) || defined(USB9098) || defined(USB9097) || defined(USB8978)
/* Transmit buffer size for chip revision check */
#define CHIP_REV_TX_BUF_SIZE 16
/* Receive buffer size for chip revision check */
#define CHIP_REV_RX_BUF_SIZE 2048

/* Extensions */
#define EXTEND_HDR (0xAB950000)
#define EXTEND_V1 (0x00000001)
#define EXTEND_V2 (0x00000002)

#endif

/** Default firmaware name */
#ifdef USB8997
#define USB8997_DEFAULT_COMBO_FW_NAME "nxp/usbusb8997_combo_v4.bin"
#define USB8997_DEFAULT_WLAN_FW_NAME "nxp/usb8997_wlan_v4.bin"
#define USBUART8997_DEFAULT_COMBO_FW_NAME "nxp/usbuart8997_combo_v4.bin"
#define USBUSB8997_DEFAULT_COMBO_FW_NAME "nxp/usbusb8997_combo_v4.bin"

#endif /* USB8997 */

#ifdef USB8978
#define USB8978_DEFAULT_COMBO_FW_NAME "nxp/usbusb8978_combo.bin"
#define USB8978_DEFAULT_WLAN_FW_NAME "nxp/usb8978_wlan.bin"
#define USBUART8978_DEFAULT_COMBO_FW_NAME "nxp/usbuart8978_combo.bin"
#define USBUSB8978_DEFAULT_COMBO_FW_NAME "nxp/usbusb8978_combo.bin"
#endif /* USB8978 */

#ifdef USB8897
#define USB8897_DEFAULT_COMBO_FW_NAME "nxp/usb8897_uapsta.bin"
#define USB8897_DEFAULT_WLAN_FW_NAME "nxp/usb8897_wlan.bin"
#endif /* USB8897 */

#ifdef USB9098
#define USB9098_Z1Z2 0x00
#define USB9098_A0 0x01
#define USB9098_A1 0x02
#define USB9098_A2 0x03
#define USB9098_DEFAULT_COMBO_FW_NAME "nxp/usbusb9098_combo.bin"
#define USB9098_DEFAULT_WLAN_FW_NAME "nxp/usb9098_wlan.bin"
#define USBUART9098_DEFAULT_COMBO_FW_NAME "nxp/usbuart9098_combo.bin"
#define USBUSB9098_DEFAULT_COMBO_FW_NAME "nxp/usbusb9098_combo.bin"
#define USB9098_WLAN_V1_FW_NAME "nxp/usb9098_wlan_v1.bin"
#define USBUART9098_COMBO_V1_FW_NAME "nxp/usbuart9098_combo_v1.bin"
#define USBUSB9098_COMBO_V1_FW_NAME "nxp/usbusb9098_combo_v1.bin"
#endif /* USB9098 */

#ifdef USB9097
#define USB9097_B0 0x01
#define USB9097_B1 0x02
#define USB9097_DEFAULT_COMBO_FW_NAME "nxp/usbusb9097_combo_v1.bin"
#define USB9097_DEFAULT_WLAN_FW_NAME "nxp/usb9097_wlan_v1.bin"
#define USB9097_WLAN_V1_FW_NAME "nxp/usb9097_wlan_v1.bin"
#define USBUART9097_COMBO_V1_FW_NAME "nxp/usbuart9097_combo_v1.bin"
#define USBUSB9097_COMBO_V1_FW_NAME "nxp/usbusb9097_combo_v1.bin"
#endif /* USB9097 */

/** urb context */
typedef struct _urb_context {
	/** Pointer to moal_handle structure */
	moal_handle *handle;
	/** Pointer to mlan buffer */
	mlan_buffer *pmbuf;
	/** URB */
	struct urb *urb;
	/** EP */
	t_u8 ep;
} urb_context;

/** USB card description structure*/
struct usb_card_rec {
	/** USB device */
	struct usb_device *udev;
	/** MOAL handle */
	moal_handle *phandle;
	/** USB interface */
	struct usb_interface *intf;
	/** Rx command endpoint type */
	int rx_cmd_ep_type;
	/** Rx command interval for INTR type */
	t_u8 rx_cmd_interval;
	/** Rx data endpoint address */
	t_u8 rx_cmd_ep;
	/** Rx cmd contxt */
	urb_context rx_cmd;
	/** Rx command URB pending count */
	atomic_t rx_cmd_urb_pending;
	/** Rx data context list */
	urb_context rx_data_list[MVUSB_RX_DATA_URB];
	/** Flag to indicate boot state */
	t_u8 boot_state;
	/** Rx data endpoint address */
	t_u8 rx_data_ep;
	/** Rx data URB pending count */
	atomic_t rx_data_urb_pending;
	/** Tx data endpoint address */
	t_u8 tx_data_ep;
	/** Tx command endpoint type */
	int tx_cmd_ep_type;
	/** Tx command interval for INTR type */
	t_u8 tx_cmd_interval;
	/** Tx command endpoint address */
	t_u8 tx_cmd_ep;
	/** Tx data URB pending count */
	atomic_t tx_data_urb_pending;
	/** Tx command URB pending count */
	atomic_t tx_cmd_urb_pending;
	/** Tx data endpoint max pkt size */
	int tx_data_maxpktsize;
	/** Tx cmd endpoint max pkt size */
	int tx_cmd_maxpktsize;
	/** Pre-allocated urb for command */
	urb_context tx_cmd;
	/** Index to point to next data urb to use */
	int tx_data_ix;
	/** Pre-allocated urb for data */
	urb_context tx_data_list[MVUSB_TX_HIGH_WMARK];
	usb_aggr_ctrl tx_aggr_ctrl;
	usb_aggr_ctrl rx_deaggr_ctrl;
	t_u8 resubmit_urbs;
	/** USB card type */
	t_u16 card_type;
	t_u8 second_mac;
};

void woal_kill_urbs(moal_handle *handle);
void woal_resubmit_urbs(moal_handle *handle);

mlan_status woal_write_data_async(moal_handle *handle, mlan_buffer *pmbuf,
				  t_u8 ep);
mlan_status woal_usb_submit_rx_data_urbs(moal_handle *handle);
mlan_status woal_usb_rx_init(moal_handle *handle);
mlan_status woal_usb_tx_init(moal_handle *handle);
mlan_status woal_usb_aggr_init(moal_handle *handle);
void woal_submit_rx_urb(moal_handle *handle, t_u8 ep);
void woal_usb_bus_unregister(void);
mlan_status woal_usb_bus_register(void);
void woal_usb_free(struct usb_card_rec *cardp);
#endif /*_MOAL_USB_H */
