/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
/*-------------------------------------------------------------------------------

	For type defines and data structure defines

--------------------------------------------------------------------------------*/


#ifndef __DRV_TYPES_H__
#define __DRV_TYPES_H__

#include <drv_conf.h>
#include <basic_types.h>
#include <osdep_service.h>
#include <wlan_bssdef.h>
#include <wifi.h>
#include <ieee80211.h>

struct rtl_priv;

#include <rtw_recv.h>
#include <rtw_security.h>
#include <rtw_xmit.h>
#include <recv_osdep.h>
#include <../rtl8821au/led.h>			/* Cureently here needed */
#include <hal_intf.h>
#include <hal_com.h>
#include <rtw_qos.h>
#include <rtw_pwrctrl.h>
#include <rtw_mlme.h>
#include <mlme_osdep.h>
#include <rtw_ioctl.h>
#include <rtw_ioctl_set.h>
#include <osdep_intf.h>
#include <rtw_eeprom.h>
#include <sta_info.h>
#include <rtw_event.h>
#include <rtw_mlme_ext.h>
// #include <rtw_efuse.h>

#include <ethernet.h>

#define SPEC_DEV_ID_NONE BIT(0)
#define SPEC_DEV_ID_DISABLE_HT BIT(1)
#define SPEC_DEV_ID_ENABLE_PS BIT(2)
#define SPEC_DEV_ID_RF_CONFIG_1T1R BIT(3)
#define SPEC_DEV_ID_RF_CONFIG_2T2R BIT(4)
#define SPEC_DEV_ID_ASSIGN_IFNAME BIT(5)

struct specific_device_id{

	u32		flags;

	u16		idVendor;
	u16		idProduct;

};

struct registry_priv
{
	NDIS_802_11_SSID	ssid;
	uint8_t	channel;//ad-hoc support requirement
	uint8_t	preamble;//long, short, auto
	u16	rts_thresh;
	uint8_t	adhoc_tx_pwr;
	uint8_t	soft_ap;
	uint8_t	power_mgnt;
	uint8_t	smart_ps;
	uint8_t	long_retry_lmt;
	uint8_t	short_retry_lmt;
	u16	busy_thresh;
	uint8_t	ack_policy;
	uint8_t	acm_method;
	  //UAPSD
	uint8_t	wmm_enable;
	uint8_t	uapsd_enable;

	WLAN_BSSID_EX    dev_network;

	// 0: 20 MHz, 1: 40 MHz, 2: 80 MHz, 3: 160MHz
	// 2.4G use bit 0 ~ 3, 5G use bit 4 ~ 7
	// 0x21 means enable 2.4G 40MHz & 5G 80MHz
	uint8_t 	rx_stbc;
	uint8_t	ampdu_amsdu;//A-MPDU Supports A-MSDU is permitted
	// Short GI support Bit Map
	// BIT(0) - 20MHz, 1: support, 0: non-support
	// BIT(1) - 40MHz, 1: support, 0: non-support
	// BIT(2) - 80MHz, 1: support, 0: non-support
	// BIT(3) - 160MHz, 1: support, 0: non-support

	// BIT(0): Enable VHT LDPC Rx, BIT(1): Enable VHT LDPC Tx, BIT(4): Enable HT LDPC Rx, BIT(5): Enable HT LDPC Tx
	// BIT(0): Enable VHT STBC Rx, BIT(1): Enable VHT STBC Tx, BIT(4): Enable HT STBC Rx, BIT(5): Enable HT STBC Tx
	// BIT(0): Enable VHT Beamformer, BIT(1): Enable VHT Beamformee, BIT(4): Enable HT Beamformer, BIT(5): Enable HT Beamformee
	uint8_t	beamform_cap;

	uint8_t	low_power ;

	bool	bAcceptAddbaReq;

	uint8_t	usbss_enable;//0:disable,1:enable
	uint8_t	hwpdn_mode;//0:disable,1:enable,2:decide by EFUSE config
	uint8_t	hwpwrp_detect;//0:disable,1:enable

	uint8_t	hw_wps_pbc;//0:disable,1:enable
};


//For registry parameters
#define MAX_CONTINUAL_URB_ERR 4


#include "../wifi.h"

#endif //__DRV_TYPES_H__

