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
#ifndef __RTW_AP_H_
#define __RTW_AP_H_


#ifdef CONFIG_AP_MODE

//external function
extern void rtw_indicate_sta_assoc_event(struct rtl_priv *rtlpriv, struct sta_info *psta);
extern void rtw_indicate_sta_disassoc_event(struct rtl_priv *rtlpriv, struct sta_info *psta);


void init_mlme_ap_info(struct rtl_priv *rtlpriv);
void free_mlme_ap_info(struct rtl_priv *rtlpriv);
//void update_BCNTIM(struct rtl_priv *rtlpriv);
void rtw_add_bcn_ie(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork, uint8_t index, uint8_t *data, uint8_t len);
void rtw_remove_bcn_ie(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork, uint8_t index);
void update_beacon(struct rtl_priv *rtlpriv, uint8_t ie_id, uint8_t *oui, uint8_t tx);
void add_RATid(struct rtl_priv *rtlpriv, struct sta_info *psta, uint8_t rssi_level);
void expire_timeout_chk(struct rtl_priv *rtlpriv);
void update_sta_info_apmode(struct rtl_priv *rtlpriv, struct sta_info *psta);
int rtw_check_beacon_data(struct rtl_priv *rtlpriv, uint8_t *pbuf,  int len);
void rtw_ap_restore_network(struct rtl_priv *rtlpriv);
void rtw_set_macaddr_acl(struct rtl_priv *rtlpriv, int mode);
int rtw_acl_add_sta(struct rtl_priv *rtlpriv, uint8_t *addr);
int rtw_acl_remove_sta(struct rtl_priv *rtlpriv, uint8_t *addr);

uint8_t rtw_ap_set_pairwise_key(struct rtl_priv *rtlpriv, struct sta_info *psta);
int rtw_ap_set_group_key(struct rtl_priv *rtlpriv, uint8_t *key, uint8_t alg, int keyid);
int rtw_ap_set_wep_key(struct rtl_priv *rtlpriv, uint8_t *key, uint8_t keylen, int keyid, uint8_t set_tx);

void associated_clients_update(struct rtl_priv *rtlpriv, uint8_t updated);
void bss_cap_update_on_sta_join(struct rtl_priv *rtlpriv, struct sta_info *psta);
uint8_t bss_cap_update_on_sta_leave(struct rtl_priv *rtlpriv, struct sta_info *psta);
void sta_info_update(struct rtl_priv *rtlpriv, struct sta_info *psta);
void ap_sta_info_defer_update(struct rtl_priv *rtlpriv, struct sta_info *psta);
uint8_t ap_free_sta(struct rtl_priv *rtlpriv, struct sta_info *psta, bool active, u16 reason);
int rtw_sta_flush(struct rtl_priv *rtlpriv);
int rtw_ap_inform_ch_switch(struct rtl_priv *rtlpriv, uint8_t new_ch, uint8_t ch_offset);
void start_ap_mode(struct rtl_priv *rtlpriv);
void stop_ap_mode(struct rtl_priv *rtlpriv);

#endif //end of CONFIG_AP_MODE

#endif

