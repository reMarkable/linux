/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
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
#ifndef __RTW_IOCTL_SET_H_
#define __RTW_IOCTL_SET_H_


typedef uint8_t NDIS_802_11_PMKID_VALUE[16];

typedef struct _BSSIDInfo {
	NDIS_802_11_MAC_ADDRESS  BSSID;
	NDIS_802_11_PMKID_VALUE  PMKID;
} BSSIDInfo, *PBSSIDInfo;




uint8_t rtw_set_802_11_add_key(struct rtl_priv * rtlpriv, NDIS_802_11_KEY * key);
uint8_t rtw_set_802_11_authentication_mode(struct rtl_priv *pdapter, NDIS_802_11_AUTHENTICATION_MODE authmode);
uint8_t rtw_set_802_11_bssid(struct rtl_priv* rtlpriv, uint8_t *bssid);
uint8_t rtw_set_802_11_add_wep(struct rtl_priv * rtlpriv, NDIS_802_11_WEP * wep);
uint8_t rtw_set_802_11_disassociate(struct rtl_priv * rtlpriv);
uint8_t rtw_set_802_11_bssid_list_scan(struct rtl_priv* rtlpriv, NDIS_802_11_SSID *pssid, int ssid_max_num);
uint8_t rtw_set_802_11_infrastructure_mode(struct rtl_priv * rtlpriv, NDIS_802_11_NETWORK_INFRASTRUCTURE networktype);
uint8_t rtw_set_802_11_remove_wep(struct rtl_priv * rtlpriv, u32 keyindex);
uint8_t rtw_set_802_11_ssid(struct rtl_priv * rtlpriv, NDIS_802_11_SSID * ssid);
uint8_t rtw_set_802_11_remove_key(struct rtl_priv * rtlpriv, NDIS_802_11_REMOVE_KEY * key);


uint8_t rtw_validate_ssid(NDIS_802_11_SSID *ssid);

u16 rtw_get_cur_max_rate(struct rtl_priv *rtlpriv);
int rtw_set_scan_mode(struct rtl_priv *rtlpriv, RT_SCAN_TYPE scan_mode);

#endif

