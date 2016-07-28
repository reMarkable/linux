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
#define _RTW_AP_C_

#include <drv_types.h>
#include <rtw_ap.h>
#include <../rtl8821au/hw.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

#ifdef CONFIG_AP_MODE

static struct list_head *get_next(struct list_head	*list)
{
	return list->next;
}

extern unsigned char	RTW_WPA_OUI[];
extern unsigned char	WMM_OUI[];
extern unsigned char	WPS_OUI[];
extern unsigned char	P2P_OUI[];
extern unsigned char	WFD_OUI[];

void init_mlme_ap_info(struct rtl_priv *rtlpriv)
{
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct wlan_acl_pool *pacl_list = &pstapriv->acl_list;


	spin_lock_init(&pmlmepriv->bcn_update_lock);

	/*
	 * for ACL
	 */
	_rtw_init_queue(&pacl_list->acl_node_q);

	/*
	 * pmlmeext->bstart_bss = false;
	 */

	start_ap_mode(rtlpriv);
}

void free_mlme_ap_info(struct rtl_priv *rtlpriv)
{
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	/*
	 * stop_ap_mode(rtlpriv);
	 */

	pmlmepriv->update_bcn = false;
	pmlmeext->bstart_bss = false;

	rtw_sta_flush(rtlpriv);

	pmlmeinfo->state = _HW_STATE_NOLINK_;

	/*
	 * ree_assoc_sta_resources
	 */
	rtw_free_all_stainfo(rtlpriv);

	/*
	 * free bc/mc sta_info
	 */

	psta = rtw_get_bcmc_stainfo(rtlpriv);
	spin_lock_bh(&(pstapriv->sta_hash_lock));
	rtw_free_stainfo(rtlpriv, psta);
	spin_unlock_bh(&(pstapriv->sta_hash_lock));
}

static void update_BCNTIM(struct rtl_priv *rtlpriv)
{
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct mlme_ext_priv *pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX *pnetwork_mlmeext = &(pmlmeinfo->network);
	unsigned char *pie = pnetwork_mlmeext->IEs;

	/*
	 * DBG_871X("%s\n", __FUNCTION__);
	 */

	/*
	 * pdate TIM IE
	 */
	/* if (pstapriv->tim_bitmap) */
	if (true) {
		uint8_t *p, *dst_ie, *premainder_ie = NULL, *pbackup_remainder_ie = NULL;
		u16 tim_bitmap_le;
		uint offset, tmp_len, tim_ielen, tim_ie_offset, remainder_ielen;

		tim_bitmap_le = cpu_to_le16(pstapriv->tim_bitmap);

		p = rtw_get_ie(pie + _FIXED_IE_LENGTH_, _TIM_IE_, &tim_ielen, pnetwork_mlmeext->IELength - _FIXED_IE_LENGTH_);
		if (p != NULL && tim_ielen > 0) {
			tim_ielen += 2;

			premainder_ie = p+tim_ielen;

			tim_ie_offset = (int)(p - pie);

			remainder_ielen = pnetwork_mlmeext->IELength - tim_ie_offset - tim_ielen;

			/*
			 * append TIM IE from dst_ie offset
			 */
			dst_ie = p;
		} else {
			tim_ielen = 0;

			/* calucate head_len */

			offset = _FIXED_IE_LENGTH_;

			/* get ssid_ie len */
			p = rtw_get_ie(pie + _BEACON_IE_OFFSET_, _SSID_IE_, &tmp_len, (pnetwork_mlmeext->IELength - _BEACON_IE_OFFSET_));
			if (p != NULL)
				offset += tmp_len+2;

			/* get supported rates len */
			p = rtw_get_ie(pie + _BEACON_IE_OFFSET_, _SUPPORTEDRATES_IE_, &tmp_len, (pnetwork_mlmeext->IELength - _BEACON_IE_OFFSET_));
			if (p !=  NULL) {
				offset += tmp_len+2;
			}

			/* DS Parameter Set IE, len=3 */
			offset += 3;

			premainder_ie = pie + offset;

			remainder_ielen = pnetwork_mlmeext->IELength - offset - tim_ielen;

			/* append TIM IE from offset */
			dst_ie = pie + offset;
		}


		if (remainder_ielen > 0) {
			pbackup_remainder_ie = rtw_malloc(remainder_ielen);
			if (pbackup_remainder_ie && premainder_ie)
				memcpy(pbackup_remainder_ie, premainder_ie, remainder_ielen);
		}

		*dst_ie++ = _TIM_IE_;

		if ((pstapriv->tim_bitmap&0xff00) && (pstapriv->tim_bitmap&0x00fe))
			tim_ielen = 5;
		else
			tim_ielen = 4;

		*dst_ie++ = tim_ielen;

		*dst_ie++ = 0;	/*	DTIM count */
		*dst_ie++ = 1;	/*	DTIM peroid */

		if (pstapriv->tim_bitmap&BIT(0))	/* for bc/mc frames */
			*dst_ie++ = BIT(0);	/*bitmap ctrl */
		else
			*dst_ie++ = 0;

		if (tim_ielen == 4) {
			uint8_t pvb = 0;

			if (pstapriv->tim_bitmap&0x00fe)
				pvb = (uint8_t)tim_bitmap_le;
			else if (pstapriv->tim_bitmap&0xff00)
				pvb = (uint8_t)(tim_bitmap_le>>8);
			else
				pvb = (uint8_t)tim_bitmap_le;

			*dst_ie++ = pvb;

		} else if (tim_ielen == 5) {
			memcpy(dst_ie, &tim_bitmap_le, 2);
			dst_ie += 2;
		}

		/* copy remainder IE */
		if (pbackup_remainder_ie) {
			memcpy(dst_ie, pbackup_remainder_ie, remainder_ielen);

			/* ULLI check usage of remainder_ielen */
			rtw_mfree(pbackup_remainder_ie);
		}

		offset =  (uint)(dst_ie - pie);
		pnetwork_mlmeext->IELength = offset + remainder_ielen;

	}

	set_tx_beacon_cmd(rtlpriv);


}

void rtw_add_bcn_ie(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork, uint8_t index, uint8_t *data, uint8_t len)
{
	PNDIS_802_11_VARIABLE_IEs	pIE;
	uint8_t	bmatch = false;
	uint8_t	*pie = pnetwork->IEs;
	uint8_t	*p, *dst_ie, *premainder_ie = NULL, *pbackup_remainder_ie = NULL;
	uint32_t	i, offset, ielen, ie_offset, remainder_ielen = 0;

	for (i = sizeof(NDIS_802_11_FIXED_IEs); i < pnetwork->IELength;) {
		pIE = (PNDIS_802_11_VARIABLE_IEs)(pnetwork->IEs + i);

		if (pIE->ElementID > index) {
			break;
		} else if (pIE->ElementID == index) {	/* already exist the same IE */
			p = (uint8_t *)pIE;
			ielen = pIE->Length;
			bmatch = true;
			break;
		}

		p = (uint8_t *)pIE;
		ielen = pIE->Length;
		i += (pIE->Length + 2);
	}

	if (p != NULL && ielen > 0) {
		ielen += 2;

		premainder_ie = p+ielen;

		ie_offset = (int)(p -pie);

		remainder_ielen = pnetwork->IELength - ie_offset - ielen;

		if (bmatch)
			dst_ie = p;
		else
			dst_ie = (p+ielen);
	}

	if (remainder_ielen > 0) {
		pbackup_remainder_ie = rtw_malloc(remainder_ielen);
		if (pbackup_remainder_ie && premainder_ie)
			memcpy(pbackup_remainder_ie, premainder_ie, remainder_ielen);
	}

	*dst_ie++ = index;
	*dst_ie++ = len;

	memcpy(dst_ie, data, len);
	dst_ie += len;

	/* copy remainder IE */
	if (pbackup_remainder_ie) {
		memcpy(dst_ie, pbackup_remainder_ie, remainder_ielen);
		/* ULLI check usage of remainder_ielen  */
		rtw_mfree(pbackup_remainder_ie);
	}

	offset =  (uint)(dst_ie - pie);
	pnetwork->IELength = offset + remainder_ielen;
}

void rtw_remove_bcn_ie(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork, uint8_t index)
{
	uint8_t *p, *dst_ie, *premainder_ie = NULL, *pbackup_remainder_ie = NULL;
	uint offset, ielen, ie_offset, remainder_ielen = 0;
	uint8_t	*pie = pnetwork->IEs;

	p = rtw_get_ie(pie + _FIXED_IE_LENGTH_, index, &ielen, pnetwork->IELength - _FIXED_IE_LENGTH_);
	if (p != NULL && ielen > 0) {
		ielen += 2;

		premainder_ie = p+ielen;

		ie_offset = (int)(p -pie);

		remainder_ielen = pnetwork->IELength - ie_offset - ielen;

		dst_ie = p;
	}

	if (remainder_ielen > 0) {
		pbackup_remainder_ie = rtw_malloc(remainder_ielen);
		if (pbackup_remainder_ie && premainder_ie)
			memcpy(pbackup_remainder_ie, premainder_ie, remainder_ielen);
	}

	/* copy remainder IE */
	if (pbackup_remainder_ie) {
		memcpy(dst_ie, pbackup_remainder_ie, remainder_ielen);
		/* ULLI check usage of remainder_ielen */
		rtw_mfree(pbackup_remainder_ie);
	}

	offset =  (uint)(dst_ie - pie);
	pnetwork->IELength = offset + remainder_ielen;
}


uint8_t chk_sta_is_alive(struct sta_info *psta);
uint8_t chk_sta_is_alive(struct sta_info *psta)
{
	uint8_t ret = false;
	#ifdef DBG_EXPIRATION_CHK
	DBG_871X("sta:"MAC_FMT", rssi:%d, rx:"STA_PKTS_FMT", expire_to:%u, %s%ssq_len:%u\n"
		, MAC_ARG(psta->hwaddr)
		, psta->rssi_stat.UndecoratedSmoothedPWDB
		//, STA_RX_PKTS_ARG(psta)
		, STA_RX_PKTS_DIFF_ARG(psta)
		, psta->expire_to
		, psta->state&WIFI_SLEEP_STATE?"PS, ":""
		, psta->state&WIFI_STA_ALIVE_CHK_STATE?"SAC, ":""
		, psta->sleepq_len
	);
	#endif

	/* if (sta_last_rx_pkts(psta) == sta_rx_pkts(psta)) */
	if ((psta->sta_stats.last_rx_data_pkts + psta->sta_stats.last_rx_ctrl_pkts) == (psta->sta_stats.rx_data_pkts + psta->sta_stats.rx_ctrl_pkts))
	{
		#if 0
		if (psta->state&WIFI_SLEEP_STATE)
			ret = true;
		#endif
	} else {
		ret = true;
	}

	sta_update_last_rx_pkts(psta);

	return ret;
}

void	expire_timeout_chk(struct rtl_priv *rtlpriv)
{
	struct list_head	*phead, *plist;
	uint8_t updated;
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	uint8_t chk_alive_num = 0;
	char chk_alive_list[NUM_STA];
	int i;

	spin_lock_bh(&pstapriv->auth_list_lock);

	phead = &pstapriv->auth_list;
	plist = get_next(phead);

	/* check auth_queue */
#ifdef DBG_EXPIRATION_CHK
	if (rtw_end_of_queue_search(phead, plist) == false) {
		DBG_871X(FUNC_NDEV_FMT" auth_list, cnt:%u\n"
			, FUNC_NDEV_ARG(rtlpriv->ndev), pstapriv->auth_list_cnt);
	}
#endif
	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		psta = container_of(plist, struct sta_info, auth_list);
		plist = get_next(plist);

		if (psta->expire_to > 0) {
			psta->expire_to--;
			if (psta->expire_to == 0) {
				list_del_init(&psta->auth_list);
				pstapriv->auth_list_cnt--;

				DBG_871X("auth expire %02X%02X%02X%02X%02X%02X\n",
					psta->hwaddr[0], psta->hwaddr[1], psta->hwaddr[2],
					psta->hwaddr[3], psta->hwaddr[4], psta->hwaddr[5]);

				spin_unlock_bh(&pstapriv->auth_list_lock);

				spin_lock_bh(&(pstapriv->sta_hash_lock));
				rtw_free_stainfo(rtlpriv, psta);
				spin_unlock_bh(&(pstapriv->sta_hash_lock));

				spin_lock_bh(&pstapriv->auth_list_lock);
			}
		}

	}

	spin_unlock_bh(&pstapriv->auth_list_lock);

	psta = NULL;

	spin_lock_bh(&pstapriv->asoc_list_lock);

	phead = &pstapriv->asoc_list;
	plist = get_next(phead);

	/* check asoc_queue */
#ifdef DBG_EXPIRATION_CHK
	if (rtw_end_of_queue_search(phead, plist) == false) {
		DBG_871X(FUNC_NDEV_FMT" asoc_list, cnt:%u\n"
			, FUNC_NDEV_ARG(rtlpriv->ndev), pstapriv->asoc_list_cnt);
	}
#endif
	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		psta = container_of(plist, struct sta_info, asoc_list);
		plist = get_next(plist);

		if (chk_sta_is_alive(psta) || !psta->expire_to) {
			psta->expire_to = pstapriv->expire_to;
			psta->keep_alive_trycnt = 0;
			#ifdef CONFIG_TX_MCAST2UNI
			psta->under_exist_checking = 0;
			#endif	/* CONFIG_TX_MCAST2UNI */
		} else {
			psta->expire_to--;
		}

#ifdef CONFIG_TX_MCAST2UNI
		if ((psta->flags & WLAN_STA_HT) && (psta->htpriv.agg_enable_bitmap || psta->under_exist_checking)) {
			/*
			 * check sta by delba(addba) for 11n STA
			 * ToDo: use CCX report to check for all STAs
			 */

			/*
			 * DBG_871X("asoc check by DELBA/ADDBA! (pstapriv->expire_to=%d s)(psta->expire_to=%d s), [%02x, %d]\n", pstapriv->expire_to*2, psta->expire_to*2, psta->htpriv.agg_enable_bitmap, psta->under_exist_checking);
			 */

				if ( psta->expire_to <= (pstapriv->expire_to - 50 ) ) {
				DBG_871X("asoc expire by DELBA/ADDBA! (%d s)\n", (pstapriv->expire_to-psta->expire_to)*2);
				psta->under_exist_checking = 0;
				psta->expire_to = 0;
			} else if ( psta->expire_to <= (pstapriv->expire_to - 3) && (psta->under_exist_checking == 0)) {
				DBG_871X("asoc check by DELBA/ADDBA! (%d s)\n", (pstapriv->expire_to-psta->expire_to)*2);
				psta->under_exist_checking = 1;
				/* tear down TX AMPDU */
				send_delba(rtlpriv, 1, psta->hwaddr);// // originator
				psta->htpriv.agg_enable_bitmap = 0x0;//reset
				psta->htpriv.candidate_tid_bitmap = 0x0;//reset
			}
		}
#endif // CONFIG_TX_MCAST2UNI

		if (psta->expire_to <= 0) {
			list_del_init(&psta->asoc_list);
			pstapriv->asoc_list_cnt--;

			DBG_871X("asoc expire "MAC_FMT", state=0x%x\n", MAC_ARG(psta->hwaddr), psta->state);
			updated = ap_free_sta(rtlpriv, psta, false, WLAN_REASON_DEAUTH_LEAVING);
		} else {
			/* TODO: Aging mechanism to digest frames in sleep_q to avoid running out of xmitframe */
			if (psta->sleepq_len > (NR_XMITFRAME/pstapriv->asoc_list_cnt)
				&& rtlpriv->xmitpriv.free_xmitframe_cnt < ((NR_XMITFRAME/pstapriv->asoc_list_cnt)/2)
			){
				DBG_871X("%s sta:"MAC_FMT", sleepq_len:%u, free_xmitframe_cnt:%u, asoc_list_cnt:%u, clear sleep_q\n", __func__
					, MAC_ARG(psta->hwaddr)
					, psta->sleepq_len, rtlpriv->xmitpriv.free_xmitframe_cnt, pstapriv->asoc_list_cnt);
				wakeup_sta_to_xmit(rtlpriv, psta);
			}
		}
	}

	spin_unlock_bh(&pstapriv->asoc_list_lock);

	associated_clients_update(rtlpriv, updated);
}

void add_RATid(struct rtl_priv *rtlpriv, struct sta_info *psta, uint8_t rssi_level)
{
	int i;
	uint8_t rf_type;
	uint32_t	 init_rate = 0;
	unsigned char sta_band = 0, raid, shortGIrate = false;
	unsigned char limit;
	unsigned int tx_ra_bitmap=0;
	struct ht_priv	*psta_ht = NULL;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	WLAN_BSSID_EX *pcur_network = (WLAN_BSSID_EX *)&pmlmepriv->cur_network.network;

	if (psta)
		psta_ht = &psta->htpriv;
	else
		return;

	if (!(psta->state & _FW_LINKED))
		return;

	//b/g mode ra_bitmap
	for (i = 0; i < sizeof(psta->bssrateset); i++) {
		if (psta->bssrateset[i])
			tx_ra_bitmap |= rtw_get_bit_value_from_ieee_value(psta->bssrateset[i]&0x7f);
	}
	//AC mode ra_bitmap
	if (psta->vhtpriv.vht_option) {
		uint32_t	vht_bitmap = 0;

		vht_bitmap = rtw_vht_rate_to_bitmap(psta->vhtpriv.vht_mcs_map);
		tx_ra_bitmap |= (vht_bitmap << 12);

		//max short GI rate
		shortGIrate = psta->vhtpriv.sgi;
	} else
		{
		//n mode ra_bitmap
		if (psta_ht->ht_option)	{
			rf_type = rtlpriv->phy.rf_type;

			if (rf_type == RF_2T2R)
				limit = 16;	// 2R
			else
				limit = 8;	//  1R

			for (i = 0; i < limit; i++) {
				if (psta_ht->ht_cap.supp_mcs_set[i/8] & BIT(i%8))
					tx_ra_bitmap |= BIT(i+12);
			}

			//max short GI rate
			shortGIrate = psta_ht->sgi;
		}
	}

	if ( pcur_network->Configuration.DSConfig > 14 ) {
		// 5G band
		if (psta->vhtpriv.vht_option)  {
			sta_band = WIRELESS_11_5AC;
		} else
		{
			if (tx_ra_bitmap & 0xffff000)
				sta_band |= WIRELESS_11_5N | WIRELESS_11A;
			else
				sta_band |= WIRELESS_11A;
		}
	} else {
		if (tx_ra_bitmap & 0xffff000)
			sta_band |= WIRELESS_11_24N | WIRELESS_11G | WIRELESS_11B;
		else if (tx_ra_bitmap & 0xff0)
			sta_band |= WIRELESS_11G |WIRELESS_11B;
		else
			sta_band |= WIRELESS_11B;
	}

	psta->wireless_mode = sta_band;

	//raid = networktype_to_raid(sta_band);
	raid = rtw_hal_networktype_to_raid(rtlpriv,sta_band);

	init_rate = get_highest_rate_idx(tx_ra_bitmap)&0x3f;

	if (psta->aid < NUM_STA) {
		uint8_t	arg[4] = {0};

		//arg[0] = macid
		//arg[1] = raid
		//arg[2] = shortGIrate
		//arg[3] = init_rate

		arg[0] = psta->mac_id;
		arg[1] = raid;
		arg[2] = shortGIrate;
		arg[3] = init_rate;

		DBG_871X("%s=> mac_id:%d , raid:%d , shortGIrate=%d, bitmap=0x%x\n",
			__FUNCTION__ , psta->mac_id, raid ,shortGIrate, tx_ra_bitmap);

		rtlpriv->cfg->ops->Add_RateATid(rtlpriv, tx_ra_bitmap, arg, rssi_level);

		if (shortGIrate == true)
			init_rate |= BIT(6);

		//set ra_id, init_rate
		psta->raid = raid;
		psta->init_rate = init_rate;
	} else {
		DBG_871X("station aid %d exceed the max number\n", psta->aid);
	}

}

static void update_bmc_sta(struct rtl_priv *rtlpriv)
{
	uint32_t	 init_rate=0;
	unsigned char	network_type, raid;
	int i, supportRateNum = 0;
	unsigned int tx_ra_bitmap = 0;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX *pcur_network = (WLAN_BSSID_EX *)&pmlmepriv->cur_network.network;
	struct sta_info *psta = rtw_get_bcmc_stainfo(rtlpriv);

	if (psta) {
		psta->aid = 0;//default set to 0
		//psta->mac_id = psta->aid+4;
		psta->mac_id = psta->aid + 1;//mac_id=1 for bc/mc stainfo

		pmlmeinfo->FW_sta_info[psta->mac_id].psta = psta;

		psta->qos_option = 0;
		psta->htpriv.ht_option = false;

		psta->ieee8021x_blocked = 0;

		memset((void*)&psta->sta_stats, 0, sizeof(struct stainfo_stats));

		//psta->dot118021XPrivacy = NO_ENCRYPTION;//!!! remove it, because it has been set before this.



		//prepare for add_RATid
		supportRateNum = rtw_get_rateset_len((uint8_t *)&pcur_network->SupportedRates);
		network_type = rtw_check_network_type((uint8_t *)&pcur_network->SupportedRates, supportRateNum, 1);

		memcpy(psta->bssrateset, &pcur_network->SupportedRates, supportRateNum);
		psta->bssratelen = supportRateNum;

		//b/g mode ra_bitmap
		for (i=0; i<supportRateNum; i++) {
			if (psta->bssrateset[i])
				tx_ra_bitmap |= rtw_get_bit_value_from_ieee_value(psta->bssrateset[i]&0x7f);
		}

		if ( pcur_network->Configuration.DSConfig > 14 ) {
			//force to A mode. 5G doesn't support CCK rates
			network_type = WIRELESS_11A;
			tx_ra_bitmap = 0x150; // 6, 12, 24 Mbps
		} else {
			//force to b mode
			network_type = WIRELESS_11B;
			tx_ra_bitmap = 0xf;
		}

		//tx_ra_bitmap = update_basic_rate(pcur_network->SupportedRates, supportRateNum);

		//raid = networktype_to_raid(network_type);
		raid = rtw_hal_networktype_to_raid(rtlpriv,network_type);

		init_rate = get_highest_rate_idx(tx_ra_bitmap&0x0fffffff)&0x3f;

		//DBG_871X("Add id %d val %08x to ratr for bmc sta\n", psta->aid, tx_ra_bitmap);
		//ap mode
		rtw_set_sta_info(rtlpriv, psta, true);

		//if (pHalData->fw_ractrl == true)
		{
			uint8_t	arg[4] = {0};

			//arg[0] = macid
			//arg[1] = raid
			//arg[2] = shortGIrate
			//arg[3] = init_rate

			arg[0] = psta->mac_id;
			arg[1] = raid;
			arg[2] = 0;
			arg[3] = init_rate;

			DBG_871X("%s=> mac_id:%d , raid:%d , bitmap=0x%x\n",
				__FUNCTION__ , psta->mac_id, raid , tx_ra_bitmap);

			rtlpriv->cfg->ops->Add_RateATid(rtlpriv, tx_ra_bitmap, arg, 0);
		}

		//set ra_id, init_rate
		psta->raid = raid;
		psta->init_rate = init_rate;

		rtw_stassoc_hw_rpt(rtlpriv, psta);

		spin_lock_bh(&psta->lock);
		psta->state = _FW_LINKED;
		spin_unlock_bh(&psta->lock);

	} else {
		DBG_871X("add_RATid_bmc_sta error!\n");
	}

}

//notes:
//AID: 1~MAX for sta and 0 for bc/mc in ap/adhoc mode
//MAC_ID = AID+1 for sta in ap/adhoc mode
//MAC_ID = 1 for bc/mc for sta/ap/adhoc
//MAC_ID = 0 for bssid for sta/ap/adhoc
//CAM_ID = //0~3 for default key, cmd_id=macid + 3, macid=aid+1;

void update_sta_info_apmode(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct ht_priv	*phtpriv_ap = &pmlmepriv->htpriv;
	struct ht_priv	*phtpriv_sta = &psta->htpriv;
	//set intf_tag to if1
	//psta->intf_tag = 0;

        DBG_871X("%s\n",__FUNCTION__);

	//psta->mac_id = psta->aid+4;
	//psta->mac_id = psta->aid+1;//alloc macid when call rtw_alloc_stainfo(),
		                                       //release macid when call rtw_free_stainfo()

	//ap mode
	rtw_set_sta_info(rtlpriv, psta, true);

	if (psecuritypriv->dot11AuthAlgrthm == dot11AuthAlgrthm_8021X)
		psta->ieee8021x_blocked = true;
	else
		psta->ieee8021x_blocked = false;


	//update sta's cap

	//ERP
	VCS_update(rtlpriv, psta);
	//HT related cap
	if (phtpriv_sta->ht_option) {
		//check if sta supports rx ampdu
		phtpriv_sta->ampdu_enable = phtpriv_ap->ampdu_enable;

		//check if sta support s Short GI
		if ((phtpriv_sta->ht_cap.cap_info & phtpriv_ap->ht_cap.cap_info) & cpu_to_le16(IEEE80211_HT_CAP_SGI_20|IEEE80211_HT_CAP_SGI_40)) {
			phtpriv_sta->sgi = true;
		}

		// bwmode
		if ((phtpriv_sta->ht_cap.cap_info & phtpriv_ap->ht_cap.cap_info) & cpu_to_le16(IEEE80211_HT_CAP_SUP_WIDTH)) {
			//phtpriv_sta->bwmode = CHANNEL_WIDTH_40;
			phtpriv_sta->bwmode = pmlmeext->cur_bwmode;
			phtpriv_sta->ch_offset = pmlmeext->cur_ch_offset;
		}

		psta->qos_option = true;

	} else {
		phtpriv_sta->ampdu_enable = false;

		phtpriv_sta->sgi = false;
		phtpriv_sta->bwmode = CHANNEL_WIDTH_20;
		phtpriv_sta->ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
	}

	//Rx AMPDU
	send_delba(rtlpriv, 0, psta->hwaddr);// recipient

	//TX AMPDU
	send_delba(rtlpriv, 1, psta->hwaddr);// // originator
	phtpriv_sta->agg_enable_bitmap = 0x0;//reset
	phtpriv_sta->candidate_tid_bitmap = 0x0;//reset

	update_sta_vht_info_apmode(rtlpriv, psta);

	//todo: init other variables

	memset((void*)&psta->sta_stats, 0, sizeof(struct stainfo_stats));


	//add ratid
	//add_RATid(rtlpriv, psta);//move to ap_sta_info_defer_update()


	spin_lock_bh(&psta->lock);
	psta->state |= _FW_LINKED;
	spin_unlock_bh(&psta->lock);


}

static void update_hw_ht_param(struct rtl_priv *rtlpriv)
{
	unsigned char		max_AMPDU_len;
	unsigned char		min_MPDU_spacing;
	struct registry_priv	 *pregpriv = &rtlpriv->registrypriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	DBG_871X("%s\n", __FUNCTION__);


	//handle A-MPDU parameter field
	/*
		AMPDU_para [1:0]:Max AMPDU Len => 0:8k , 1:16k, 2:32k, 3:64k
		AMPDU_para [4:2]:Min MPDU Start Spacing
	*/
	max_AMPDU_len = pmlmeinfo->HT_caps.u.HT_cap_element.AMPDU_para & 0x03;

	min_MPDU_spacing = (pmlmeinfo->HT_caps.u.HT_cap_element.AMPDU_para & 0x1c) >> 2;

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AMPDU_MIN_SPACE, (uint8_t *)(&min_MPDU_spacing));

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AMPDU_FACTOR, (uint8_t *)(&max_AMPDU_len));

	//
	// Config SM Power Save setting
	//
	pmlmeinfo->SM_PS = (pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info & 0x0C) >> 2;
	if (pmlmeinfo->SM_PS == WLAN_HT_CAP_SM_PS_STATIC) {
		/*uint8_t i;
		//update the MCS rates
		for (i = 0; i < 16; i++)
		{
			pmlmeinfo->HT_caps.HT_cap_element.MCS_rate[i] &= MCS_rate_1R[i];
		}*/
		DBG_871X("%s(): WLAN_HT_CAP_SM_PS_STATIC\n",__FUNCTION__);
	}

	//
	// Config current HT Protection mode.
	//
	//pmlmeinfo->HT_protection = pmlmeinfo->HT_info.infos[1] & 0x3;

}

static void start_bss_network(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	uint8_t *p;
	uint8_t val8, cur_channel, cur_bwmode, cur_ch_offset;
	u16 bcn_interval;
	uint32_t	acparm;
	int	ie_len;
	struct registry_priv	 *pregpriv = &rtlpriv->registrypriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct security_priv* psecuritypriv= &(rtlpriv->securitypriv);
	WLAN_BSSID_EX *pnetwork = (WLAN_BSSID_EX *)&pmlmepriv->cur_network.network;
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX *pnetwork_mlmeext = &(pmlmeinfo->network);
	struct HT_info_element *pht_info=NULL;
	uint8_t	cbw40_enable=0;
	uint8_t	change_band = false;

	//DBG_871X("%s\n", __FUNCTION__);

	bcn_interval = (u16)pnetwork->Configuration.BeaconPeriod;
	cur_channel = pnetwork->Configuration.DSConfig;
	cur_bwmode = CHANNEL_WIDTH_20;
	cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;


	//check if there is wps ie,
	//if there is wpsie in beacon, the hostapd will update beacon twice when stating hostapd,
	//and at first time the security ie ( RSN/WPA IE) will not include in beacon.
	if (NULL == rtw_get_wps_ie(pnetwork->IEs+_FIXED_IE_LENGTH_, pnetwork->IELength-_FIXED_IE_LENGTH_, NULL, NULL))
	{
		pmlmeext->bstart_bss = true;
	}

	//todo: update wmm, ht cap
	//pmlmeinfo->WMM_enable;
	//pmlmeinfo->HT_enable;
	if (pmlmepriv->qospriv.qos_option)
		pmlmeinfo->WMM_enable = true;
	if (pmlmepriv->htpriv.ht_option) {
		pmlmeinfo->WMM_enable = true;
		pmlmeinfo->HT_enable = true;
		//pmlmeinfo->HT_info_enable = true;
		//pmlmeinfo->HT_caps_enable = true;

		update_hw_ht_param(rtlpriv);
	}

	if (pmlmepriv->vhtpriv.vht_option) {
		pmlmeinfo->VHT_enable = true;
		update_hw_vht_param(rtlpriv);
	}

	if (pmlmepriv->cur_network.join_res != true) { //setting only at  first time
		//WEP Key will be set before this function, do not clear CAM.
		if ((psecuritypriv->dot11PrivacyAlgrthm != WEP40_ENCRYPTION) && (psecuritypriv->dot11PrivacyAlgrthm != WEP104_ENCRYPTION))
			flush_all_cam_entry(rtlpriv);	//clear CAM
	}

	//set MSR to AP_Mode
	Set_MSR(rtlpriv, _HW_STATE_AP_);

	//Set BSSID REG
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BSSID, pnetwork->MacAddress);

	//Set EDCA param reg
	acparm = 0x002F3217; // VO
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AC_PARAM_VO, (uint8_t *)(&acparm));
	acparm = 0x005E4317; // VI
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AC_PARAM_VI, (uint8_t *)(&acparm));
	//acparm = 0x00105320; // BE
	acparm = 0x005ea42b;
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AC_PARAM_BE, (uint8_t *)(&acparm));
	acparm = 0x0000A444; // BK
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AC_PARAM_BK, (uint8_t *)(&acparm));

	//Set Security
	val8 = (psecuritypriv->dot11AuthAlgrthm == dot11AuthAlgrthm_8021X)? 0xcc: 0xcf;
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_SEC_CFG, (uint8_t *)(&val8));

	//Beacon Control related register
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BEACON_INTERVAL, (uint8_t *)(&bcn_interval));

	if (pmlmepriv->cur_network.join_res != true) { //setting only at  first time
		//uint32_t	 initialgain;

		//initialgain = 0x1e;


		//disable dynamic functions, such as high power, DIG
		//Save_DM_Func_Flag(rtlpriv);
		//Switch_DM_Func(rtlpriv, DYNAMIC_FUNC_DISABLE, false);

			{

			//rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_INITIAL_GAIN, (uint8_t *)(&initialgain));
		}

	}
	//set channel, bwmode
	p = rtw_get_ie((pnetwork->IEs + sizeof(NDIS_802_11_FIXED_IEs)), _HT_ADD_INFO_IE_, &ie_len, (pnetwork->IELength - sizeof(NDIS_802_11_FIXED_IEs)));
	if ( p && ie_len) {
		pht_info = (struct HT_info_element *)(p+2);

		if (cur_channel > 14) {
			if ((0x21 & 0xf0) > 0)
				cbw40_enable = 1;
		} else {
			if ((0x21 & 0x0f) > 0)
				cbw40_enable = 1;
		}

		if ((cbw40_enable) && (pht_info->infos[0] & BIT(2))) {
			//switch to the 40M Hz mode
			//pmlmeext->cur_bwmode = CHANNEL_WIDTH_40;
			cur_bwmode = CHANNEL_WIDTH_40;
			switch (pht_info->infos[0] & 0x3) {
			case 1:
				//pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_LOWER;
				cur_ch_offset = HAL_PRIME_CHNL_OFFSET_LOWER;
				break;

			case 3:
				//pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_UPPER;
				cur_ch_offset = HAL_PRIME_CHNL_OFFSET_UPPER;
				break;

			default:
				//pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
				cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
				break;
			}

		}

	}

	p = rtw_get_ie((pnetwork->IEs + sizeof(NDIS_802_11_FIXED_IEs)), EID_VHTOperation, &ie_len, (pnetwork->IELength - sizeof(NDIS_802_11_FIXED_IEs)));
	if ( p && ie_len) {
		if (GET_VHT_OPERATION_ELE_CHL_WIDTH(p+2) >= 1) {
			cur_bwmode = CHANNEL_WIDTH_80;
		}
	}

	//TODO: need to judge the phy parameters on concurrent mode for single phy
	//set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);
	set_channel_bwmode(rtlpriv, cur_channel, cur_ch_offset, cur_bwmode);

	DBG_871X("CH=%d, BW=%d, offset=%d\n", cur_channel, cur_bwmode, cur_ch_offset);

	pmlmeext->cur_channel = cur_channel;
	pmlmeext->cur_bwmode = cur_bwmode;
	pmlmeext->cur_ch_offset = cur_ch_offset;

	//buddy interface band is different from current interface, update ERP, support rate, ext support rate IE
	if (change_band == true)
		change_band_update_ie(rtlpriv, pnetwork);

	pmlmeext->cur_wireless_mode = pmlmepriv->cur_network.network_type;

	//update cur_wireless_mode
	update_wireless_mode(rtlpriv);

	//update RRSR after set channel and bandwidth
	UpdateBrateTbl(rtlpriv, pnetwork->SupportedRates);
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BASIC_RATE, pnetwork->SupportedRates);

	//udpate capability after cur_wireless_mode updated
	update_capinfo(rtlpriv, rtw_get_capability((WLAN_BSSID_EX *)pnetwork));

	//let pnetwork_mlmeext == pnetwork_mlme.
	memcpy(pnetwork_mlmeext, pnetwork, pnetwork->Length);

	if (true == pmlmeext->bstart_bss) {
		update_beacon(rtlpriv, _TIM_IE_, NULL, false);

		//issue beacon frame
		if (send_beacon(rtlpriv) == _FAIL)
		{
			DBG_871X("issue_beacon, fail!\n");
		}

	}


	//update bc/mc sta_info
	update_bmc_sta(rtlpriv);

	//pmlmeext->bstart_bss = true;

}

int rtw_check_beacon_data(struct rtl_priv *rtlpriv, uint8_t *pbuf,  int len)
{
	int ret=_SUCCESS;
	uint8_t *p;
	uint8_t *pHT_caps_ie=NULL;
	uint8_t *pHT_info_ie=NULL;
	struct sta_info *psta = NULL;
	u16 cap, ht_cap=false;
	uint ie_len = 0;
	int group_cipher, pairwise_cipher;
	uint8_t	channel, network_type, supportRate[NDIS_802_11_LENGTH_RATES_EX];
	int supportRateNum = 0;
	uint8_t OUI1[] = {0x00, 0x50, 0xf2,0x01};
	uint8_t wps_oui[4]={0x0,0x50,0xf2,0x04};
	uint8_t WMM_PARA_IE[] = {0x00, 0x50, 0xf2, 0x02, 0x01, 0x01};
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	WLAN_BSSID_EX *pbss_network = (WLAN_BSSID_EX *)&pmlmepriv->cur_network.network;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	uint8_t *ie = pbss_network->IEs;


	/* SSID */
	/* Supported rates */
	/* DS Params */
	/* WLAN_EID_COUNTRY */
	/* ERP Information element */
	/* Extended supported rates */
	/* WPA/WPA2 */
	/* Wi-Fi Wireless Multimedia Extensions */
	/* ht_capab, ht_oper */
	/* WPS IE */

	DBG_871X("%s, len=%d\n", __FUNCTION__, len);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) != true)
		return _FAIL;


	if (len>MAX_IE_SZ)
		return _FAIL;

	pbss_network->IELength = len;

	memset(ie, 0, MAX_IE_SZ);

	memcpy(ie, pbuf, pbss_network->IELength);


	if (pbss_network->InfrastructureMode!=Ndis802_11APMode)
		return _FAIL;

	pbss_network->Rssi = 0;

	memcpy(pbss_network->MacAddress, rtlpriv->mac80211.mac_addr, ETH_ALEN);

	//beacon interval
	p = rtw_get_beacon_interval_from_ie(ie);//ie + 8;	// 8: TimeStamp, 2: Beacon Interval 2:Capability
	//pbss_network->Configuration.BeaconPeriod = le16_to_cpu(*(unsigned short*)p);
	pbss_network->Configuration.BeaconPeriod = RTW_GET_LE16(p);

	//capability
	//cap = *(unsigned short *)rtw_get_capability_from_ie(ie);
	//cap = le16_to_cpu(cap);
	cap = RTW_GET_LE16(ie);

	//SSID
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _SSID_IE_, &ie_len, (pbss_network->IELength -_BEACON_IE_OFFSET_));
	if (p && ie_len>0) {
		memset(&pbss_network->Ssid, 0, sizeof(NDIS_802_11_SSID));
		memcpy(pbss_network->Ssid.Ssid, (p + 2), ie_len);
		pbss_network->Ssid.SsidLength = ie_len;
	}

	//chnnel
	channel = 0;
	pbss_network->Configuration.Length = 0;
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _DSSET_IE_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_));
	if (p && ie_len>0)
		channel = *(p + 2);

	pbss_network->Configuration.DSConfig = channel;


	memset(supportRate, 0, NDIS_802_11_LENGTH_RATES_EX);
	// get supported rates
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _SUPPORTEDRATES_IE_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_));
	if (p !=  NULL) {
		memcpy(supportRate, p+2, ie_len);
		supportRateNum = ie_len;
	}

	//get ext_supported rates
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _EXT_SUPPORTEDRATES_IE_, &ie_len, pbss_network->IELength - _BEACON_IE_OFFSET_);
	if (p !=  NULL) {
		memcpy(supportRate+supportRateNum, p+2, ie_len);
		supportRateNum += ie_len;

	}

	network_type = rtw_check_network_type(supportRate, supportRateNum, channel);

	rtw_set_supported_rate(pbss_network->SupportedRates, network_type);


	//parsing ERP_IE
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _ERPINFO_IE_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_));
	if (p && ie_len>0) {
		ERP_IE_handler(rtlpriv, (PNDIS_802_11_VARIABLE_IEs)p);
	}

	//update privacy/security
	if (cap & BIT(4))
		pbss_network->Privacy = 1;
	else
		pbss_network->Privacy = 0;

	psecuritypriv->wpa_psk = 0;

	//wpa2
	group_cipher = 0; pairwise_cipher = 0;
	psecuritypriv->wpa2_group_cipher = NO_ENCRYPTION;
	psecuritypriv->wpa2_pairwise_cipher = NO_ENCRYPTION;
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _RSN_IE_2_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_));
	if (p && ie_len>0) {
		if (rtw_parse_wpa2_ie(p, ie_len+2, &group_cipher, &pairwise_cipher, NULL) == _SUCCESS) {
			psecuritypriv->dot11AuthAlgrthm= dot11AuthAlgrthm_8021X;

			psecuritypriv->dot8021xalg = 1;//psk,  todo:802.1x
			psecuritypriv->wpa_psk |= BIT(1);

			psecuritypriv->wpa2_group_cipher = group_cipher;
			psecuritypriv->wpa2_pairwise_cipher = pairwise_cipher;
		}

	}

	//wpa
	ie_len = 0;
	group_cipher = 0; pairwise_cipher = 0;
	psecuritypriv->wpa_group_cipher = NO_ENCRYPTION;
	psecuritypriv->wpa_pairwise_cipher = NO_ENCRYPTION;
	for (p = ie + _BEACON_IE_OFFSET_; ;p += (ie_len + 2)) {
		p = rtw_get_ie(p, _SSN_IE_1_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_ - (ie_len + 2)));
		if ((p) && (_rtw_memcmp(p+2, OUI1, 4))) {
			if (rtw_parse_wpa_ie(p, ie_len+2, &group_cipher, &pairwise_cipher, NULL) == _SUCCESS) {
				psecuritypriv->dot11AuthAlgrthm= dot11AuthAlgrthm_8021X;

				psecuritypriv->dot8021xalg = 1;//psk,  todo:802.1x

				psecuritypriv->wpa_psk |= BIT(0);

				psecuritypriv->wpa_group_cipher = group_cipher;
				psecuritypriv->wpa_pairwise_cipher = pairwise_cipher;
			}

			break;

		}

		if ((p == NULL) || (ie_len == 0)) {
				break;
		}

	}

	//wmm
	ie_len = 0;
	pmlmepriv->qospriv.qos_option = 0;
	if (pregistrypriv->wmm_enable) {
		for (p = ie + _BEACON_IE_OFFSET_; ;p += (ie_len + 2)) {
			p = rtw_get_ie(p, _VENDOR_SPECIFIC_IE_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_ - (ie_len + 2)));
			if ((p) && _rtw_memcmp(p+2, WMM_PARA_IE, 6)) {
				pmlmepriv->qospriv.qos_option = 1;

				*(p+8) |= BIT(7);//QoS Info, support U-APSD

				/* disable all ACM bits since the WMM admission control is not supported */
				*(p + 10) &= ~BIT(4); /* BE */
				*(p + 14) &= ~BIT(4); /* BK */
				*(p + 18) &= ~BIT(4); /* VI */
				*(p + 22) &= ~BIT(4); /* VO */

				break;
			}

			if ((p == NULL) || (ie_len == 0)) {
				break;
			}
		}
	}
	//parsing HT_CAP_IE
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _HT_CAPABILITY_IE_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_));
	if (p && ie_len>0) {
		uint8_t rf_type;

		struct rtw_ieee80211_ht_cap *pht_cap = (struct rtw_ieee80211_ht_cap *)(p+2);

		pHT_caps_ie=p;


		ht_cap = true;
		network_type |= WIRELESS_11_24N;

		rf_type = rtlpriv->phy.rf_type;

		if ((psecuritypriv->wpa_pairwise_cipher & WPA_CIPHER_CCMP) ||
			(psecuritypriv->wpa2_pairwise_cipher & WPA_CIPHER_CCMP)) {
			pht_cap->ampdu_params_info |= (IEEE80211_HT_CAP_AMPDU_DENSITY&(0x07<<2));
		} else 	{
			pht_cap->ampdu_params_info |= (IEEE80211_HT_CAP_AMPDU_DENSITY&0x00);
		}

		pht_cap->ampdu_params_info |= (IEEE80211_HT_CAP_AMPDU_FACTOR & 0x03); //set  Max Rx AMPDU size  to 64K

		if (rf_type == RF_1T1R) {
			pht_cap->supp_mcs_set[0] = 0xff;
			pht_cap->supp_mcs_set[1] = 0x0;
		}

		memcpy(&pmlmepriv->htpriv.ht_cap, p+2, ie_len);

	}

	//parsing HT_INFO_IE
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _HT_ADD_INFO_IE_, &ie_len, (pbss_network->IELength - _BEACON_IE_OFFSET_));
	if (p && ie_len>0) {
		pHT_info_ie=p;
	}
	switch(network_type) {
	case WIRELESS_11B:
		pbss_network->NetworkTypeInUse = Ndis802_11DS;
		break;
	case WIRELESS_11G:
	case WIRELESS_11BG:
	case WIRELESS_11G_24N:
	case WIRELESS_11BG_24N:
		pbss_network->NetworkTypeInUse = Ndis802_11OFDM24;
		break;
	case WIRELESS_11A:
		pbss_network->NetworkTypeInUse = Ndis802_11OFDM5;
		break;
	default :
		pbss_network->NetworkTypeInUse = Ndis802_11OFDM24;
	break;
	}

	pmlmepriv->cur_network.network_type = network_type;

	pmlmepriv->htpriv.ht_option = false;

	if ( (psecuritypriv->wpa2_pairwise_cipher&WPA_CIPHER_TKIP) ||
		      (psecuritypriv->wpa_pairwise_cipher&WPA_CIPHER_TKIP)) {
		//todo:
		//ht_cap = false;
	}

	//ht_cap
	if (ht_cap == true) {
		pmlmepriv->htpriv.ht_option = true;
		pmlmepriv->qospriv.qos_option = 1;

		pmlmepriv->htpriv.ampdu_enable = true;

		HT_caps_handler(rtlpriv, (PNDIS_802_11_VARIABLE_IEs)pHT_caps_ie);

		HT_info_handler(rtlpriv, (PNDIS_802_11_VARIABLE_IEs)pHT_info_ie);
	}
	pbss_network->Length = get_WLAN_BSSID_EX_sz((WLAN_BSSID_EX  *)pbss_network);

	//issue beacon to start bss network
	start_bss_network(rtlpriv, (uint8_t *)pbss_network);


	//alloc sta_info for ap itself
	psta = rtw_get_stainfo(&rtlpriv->stapriv, pbss_network->MacAddress);
	if (!psta) {
		psta = rtw_alloc_stainfo(&rtlpriv->stapriv, pbss_network->MacAddress);
		if (psta == NULL) {
			return _FAIL;
		}
	}
	psta->state |= WIFI_AP_STATE;		//Aries, add,fix bug of flush_cam_entry at STOP AP mode , 0724
	rtw_indicate_connect( rtlpriv);

	pmlmepriv->cur_network.join_res = true;//for check if already set beacon

	//update bc/mc sta_info
	//update_bmc_sta(rtlpriv);

	return ret;

}

void rtw_set_macaddr_acl(struct rtl_priv *rtlpriv, int mode)
{
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct wlan_acl_pool *pacl_list = &pstapriv->acl_list;

	DBG_871X("%s, mode=%d\n", __func__, mode);

	pacl_list->mode = mode;
}

int rtw_acl_add_sta(struct rtl_priv *rtlpriv, uint8_t *addr)
{
	struct list_head	*plist, *phead;
	uint8_t added = false;
	int i, ret=0;
	struct rtw_wlan_acl_node *paclnode;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct wlan_acl_pool *pacl_list = &pstapriv->acl_list;
	struct __queue	*pacl_node_q = &pacl_list->acl_node_q;

	DBG_871X("%s(acl_num=%d)=" MAC_FMT "\n", __func__, pacl_list->num, MAC_ARG(addr));

	if ((NUM_ACL-1) < pacl_list->num)
		return (-1);

	spin_lock_bh(&(pacl_node_q->lock));

	phead = get_list_head(pacl_node_q);
	plist = get_next(phead);

	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		paclnode = container_of(plist, struct rtw_wlan_acl_node, list);
		plist = get_next(plist);

		if (_rtw_memcmp(paclnode->addr, addr, ETH_ALEN)) {
			if (paclnode->valid == true) {
				added = true;
				DBG_871X("%s, sta has been added\n", __func__);
				break;
			}
		}
	}

	spin_unlock_bh(&(pacl_node_q->lock));


	if (added == true)
		return ret;


	spin_lock_bh(&(pacl_node_q->lock));

	for(i=0; i< NUM_ACL; i++) {
		paclnode = &pacl_list->aclnode[i];

		if (paclnode->valid == false) {
			INIT_LIST_HEAD(&paclnode->list);

			memcpy(paclnode->addr, addr, ETH_ALEN);

			paclnode->valid = true;

			list_add_tail(&paclnode->list, get_list_head(pacl_node_q));

			pacl_list->num++;

			break;
		}
	}

	DBG_871X("%s, acl_num=%d\n", __func__, pacl_list->num);

	spin_unlock_bh(&(pacl_node_q->lock));

	return ret;
}

int rtw_acl_remove_sta(struct rtl_priv *rtlpriv, uint8_t *addr)
{
	struct list_head	*plist, *phead;
	int i, ret=0;
	struct rtw_wlan_acl_node *paclnode;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct wlan_acl_pool *pacl_list = &pstapriv->acl_list;
	struct __queue	*pacl_node_q = &pacl_list->acl_node_q;

	DBG_871X("%s(acl_num=%d)=" MAC_FMT "\n", __func__, pacl_list->num, MAC_ARG(addr));

	spin_lock_bh(&(pacl_node_q->lock));

	phead = get_list_head(pacl_node_q);
	plist = get_next(phead);

	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		paclnode = container_of(plist, struct rtw_wlan_acl_node, list);
		plist = get_next(plist);

		if (_rtw_memcmp(paclnode->addr, addr, ETH_ALEN)) {
			if (paclnode->valid == true) {
				paclnode->valid = false;

				list_del_init(&paclnode->list);

				pacl_list->num--;
			}
		}
	}

	spin_unlock_bh(&(pacl_node_q->lock));

	DBG_871X("%s, acl_num=%d\n", __func__, pacl_list->num);

	return ret;

}

uint8_t rtw_ap_set_pairwise_key(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	struct cmd_obj*			ph2c;
	struct set_stakey_parm	*psetstakey_para;
	struct cmd_priv 			*pcmdpriv= &rtlpriv->cmdpriv;
	uint8_t	res=_SUCCESS;

	ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if ( ph2c == NULL) {
		res= _FAIL;
		goto exit;
	}

	psetstakey_para = (struct set_stakey_parm*)rtw_zmalloc(sizeof(struct set_stakey_parm));
	if (psetstakey_para == NULL) {
		rtw_mfree(ph2c);
		res=_FAIL;
		goto exit;
	}

	init_h2fwcmd_w_parm_no_rsp(ph2c, psetstakey_para, _SetStaKey_CMD_);


	psetstakey_para->algorithm = (uint8_t)psta->dot118021XPrivacy;

	memcpy(psetstakey_para->addr, psta->hwaddr, ETH_ALEN);

	memcpy(psetstakey_para->key, &psta->dot118021x_UncstKey, 16);


	res = rtw_enqueue_cmd(pcmdpriv, ph2c);

exit:

	return res;

}

static int rtw_ap_set_key(struct rtl_priv *rtlpriv, uint8_t *key, uint8_t alg, int keyid, uint8_t set_tx)
{
	uint8_t keylen;
	struct cmd_obj* pcmd;
	struct setkey_parm *psetkeyparm;
	struct cmd_priv	*pcmdpriv= &(rtlpriv->cmdpriv);
	int res=_SUCCESS;

	//DBG_871X("%s\n", __FUNCTION__);

	pcmd = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj));
	if (pcmd == NULL) {
		res= _FAIL;
		goto exit;
	}
	psetkeyparm=(struct setkey_parm*)rtw_zmalloc(sizeof(struct setkey_parm));
	if (psetkeyparm == NULL) {
		rtw_mfree(pcmd);
		res= _FAIL;
		goto exit;
	}

	memset(psetkeyparm, 0, sizeof(struct setkey_parm));

	psetkeyparm->keyid=(uint8_t)keyid;
	if (is_wep_enc(alg))
		rtlpriv->mlmepriv.key_mask |= BIT(psetkeyparm->keyid);

	psetkeyparm->algorithm = alg;

	psetkeyparm->set_tx = set_tx;

	switch(alg) {
	case WEP40_ENCRYPTION:
		keylen = 5;
		break;
	case WEP104_ENCRYPTION:
		keylen = 13;
		break;
	case TKIP_ENCRYPTION:
	case RSERVED_ENCRYPTION:
	case AESCCMP_ENCRYPTION:
		keylen = 16;
	default:
		keylen = 16;
	}

	memcpy(&(psetkeyparm->key[0]), key, keylen);

	pcmd->cmdcode = _SetKey_CMD_;
	pcmd->parmbuf = (uint8_t *)psetkeyparm;
	pcmd->cmdsz =  (sizeof(struct setkey_parm));
	pcmd->rsp = NULL;
	pcmd->rspsz = 0;


	INIT_LIST_HEAD(&pcmd->list);

	res = rtw_enqueue_cmd(pcmdpriv, pcmd);

exit:

	return res;
}

int rtw_ap_set_group_key(struct rtl_priv *rtlpriv, uint8_t *key, uint8_t alg, int keyid)
{
	DBG_871X("%s\n", __FUNCTION__);

	return rtw_ap_set_key(rtlpriv, key, alg, keyid, 1);
}

int rtw_ap_set_wep_key(struct rtl_priv *rtlpriv, uint8_t *key, uint8_t keylen, int keyid, uint8_t set_tx)
{
	uint8_t alg;

	switch(keylen) {
	case 5:
		alg =WEP40_ENCRYPTION;
		break;
	case 13:
		alg =WEP104_ENCRYPTION;
		break;
	default:
		alg =NO_ENCRYPTION;
	}

	DBG_871X("%s\n", __FUNCTION__);

	return rtw_ap_set_key(rtlpriv, key, alg, keyid, set_tx);
}

static void update_bcn_erpinfo_ie(struct rtl_priv *rtlpriv)
{
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX *pnetwork = &(pmlmeinfo->network);
	unsigned char *p, *ie = pnetwork->IEs;
	uint32_t	 len = 0;

	DBG_871X("%s, ERP_enable=%d\n", __FUNCTION__, pmlmeinfo->ERP_enable);

	if (!pmlmeinfo->ERP_enable)
		return;

	//parsing ERP_IE
	p = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _ERPINFO_IE_, &len, (pnetwork->IELength - _BEACON_IE_OFFSET_));
	if (p && len>0) {
		PNDIS_802_11_VARIABLE_IEs pIE = (PNDIS_802_11_VARIABLE_IEs)p;

		if (pmlmepriv->num_sta_non_erp == 1)
			pIE->data[0] |= RTW_ERP_INFO_NON_ERP_PRESENT|RTW_ERP_INFO_USE_PROTECTION;
		else
			pIE->data[0] &= ~(RTW_ERP_INFO_NON_ERP_PRESENT|RTW_ERP_INFO_USE_PROTECTION);

		if (pmlmepriv->num_sta_no_short_preamble > 0)
			pIE->data[0] |= RTW_ERP_INFO_BARKER_PREAMBLE_MODE;
		else
			pIE->data[0] &= ~(RTW_ERP_INFO_BARKER_PREAMBLE_MODE);

		ERP_IE_handler(rtlpriv, pIE);
	}

}

static void update_bcn_wps_ie(struct rtl_priv *rtlpriv)
{
	uint8_t *pwps_ie=NULL, *pwps_ie_src, *premainder_ie, *pbackup_remainder_ie=NULL;
	uint wps_ielen=0, wps_offset, remainder_ielen;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX *pnetwork = &(pmlmeinfo->network);
	unsigned char *ie = pnetwork->IEs;
	uint32_t	 ielen = pnetwork->IELength;


	DBG_871X("%s\n", __FUNCTION__);

	pwps_ie = rtw_get_wps_ie(ie+_FIXED_IE_LENGTH_, ielen-_FIXED_IE_LENGTH_, NULL, &wps_ielen);

	if (pwps_ie == NULL || wps_ielen == 0)
		return;

	wps_offset = (uint)(pwps_ie-ie);

	premainder_ie = pwps_ie + wps_ielen;

	remainder_ielen = ielen - wps_offset - wps_ielen;

	if (remainder_ielen>0) {
		pbackup_remainder_ie = rtw_malloc(remainder_ielen);
		if (pbackup_remainder_ie)
			memcpy(pbackup_remainder_ie, premainder_ie, remainder_ielen);
	}


	pwps_ie_src = pmlmepriv->wps_beacon_ie;
	if (pwps_ie_src == NULL)
		return;


	wps_ielen = (uint)pwps_ie_src[1];//to get ie data len
	if ((wps_offset+wps_ielen+2+remainder_ielen)<=MAX_IE_SZ) {
		memcpy(pwps_ie, pwps_ie_src, wps_ielen+2);
		pwps_ie += (wps_ielen+2);

		if (pbackup_remainder_ie)
			memcpy(pwps_ie, pbackup_remainder_ie, remainder_ielen);

		//update IELength
		pnetwork->IELength = wps_offset + (wps_ielen+2) + remainder_ielen;
	}

	if (pbackup_remainder_ie) {
		/* ULLI check usage of remainder_ielen */
		rtw_mfree(pbackup_remainder_ie);
	}

}

static void update_bcn_vendor_spec_ie(struct rtl_priv *rtlpriv, uint8_t *oui)
{
	DBG_871X("%s\n", __FUNCTION__);

	if (_rtw_memcmp(WPS_OUI, oui, 4)) {
		update_bcn_wps_ie(rtlpriv);
	} else {
		DBG_871X("unknown OUI type!\n");
 	}


}

void update_beacon(struct rtl_priv *rtlpriv, uint8_t ie_id, uint8_t *oui, uint8_t tx)
{
	struct mlme_priv *pmlmepriv;
	struct mlme_ext_priv	*pmlmeext;
	//struct mlme_ext_info	*pmlmeinfo;

	//DBG_871X("%s\n", __FUNCTION__);

	if (!rtlpriv)
		return;

	pmlmepriv = &(rtlpriv->mlmepriv);
	pmlmeext = &(rtlpriv->mlmeextpriv);
	//pmlmeinfo = &(pmlmeext->mlmext_info);

	if (false == pmlmeext->bstart_bss)
		return;

	spin_lock_bh(&pmlmepriv->bcn_update_lock);

	switch(ie_id) {
	case _TIM_IE_:
		update_BCNTIM(rtlpriv);
		break;

	case _ERPINFO_IE_:
		update_bcn_erpinfo_ie(rtlpriv);
		break;

	case _VENDOR_SPECIFIC_IE_:
		update_bcn_vendor_spec_ie(rtlpriv, oui);
		break;

	default:
		break;
	}

	pmlmepriv->update_bcn = true;

	spin_unlock_bh(&pmlmepriv->bcn_update_lock);

	if (tx) {
		//send_beacon(rtlpriv);//send_beacon must execute on TSR level
		set_tx_beacon_cmd(rtlpriv);
	}

}

/*
op_mode
Set to 0 (HT pure) under the followign conditions
	- all STAs in the BSS are 20/40 MHz HT in 20/40 MHz BSS or
	- all STAs in the BSS are 20 MHz HT in 20 MHz BSS
Set to 1 (HT non-member protection) if there may be non-HT STAs
	in both the primary and the secondary channel
Set to 2 if only HT STAs are associated in BSS,
	however and at least one 20 MHz HT STA is associated
Set to 3 (HT mixed mode) when one or more non-HT STAs are associated
	(currently non-GF HT station is considered as non-HT STA also)
*/
static int rtw_ht_operation_update(struct rtl_priv *rtlpriv)
{
	u16 cur_op_mode, new_op_mode;
	int op_mode_changes = 0;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct ht_priv	*phtpriv_ap = &pmlmepriv->htpriv;

	if (pmlmepriv->htpriv.ht_option == true)
		return 0;

	//if (!iface->conf->ieee80211n || iface->conf->ht_op_mode_fixed)
	//	return 0;

	DBG_871X("%s current operation mode=0x%X\n",
		   __FUNCTION__, pmlmepriv->ht_op_mode);

	if (!(pmlmepriv->ht_op_mode & HT_INFO_OPERATION_MODE_NON_GF_DEVS_PRESENT)
	    && pmlmepriv->num_sta_ht_no_gf) {
		pmlmepriv->ht_op_mode |=
			HT_INFO_OPERATION_MODE_NON_GF_DEVS_PRESENT;
		op_mode_changes++;
	} else if ((pmlmepriv->ht_op_mode &
		    HT_INFO_OPERATION_MODE_NON_GF_DEVS_PRESENT) &&
		   pmlmepriv->num_sta_ht_no_gf == 0) {
		pmlmepriv->ht_op_mode &=
			~HT_INFO_OPERATION_MODE_NON_GF_DEVS_PRESENT;
		op_mode_changes++;
	}

	if (!(pmlmepriv->ht_op_mode & HT_INFO_OPERATION_MODE_NON_HT_STA_PRESENT) &&
	    (pmlmepriv->num_sta_no_ht || pmlmepriv->olbc_ht)) {
		pmlmepriv->ht_op_mode |= HT_INFO_OPERATION_MODE_NON_HT_STA_PRESENT;
		op_mode_changes++;
	} else if ((pmlmepriv->ht_op_mode &
		    HT_INFO_OPERATION_MODE_NON_HT_STA_PRESENT) &&
		   (pmlmepriv->num_sta_no_ht == 0 && !pmlmepriv->olbc_ht)) {
		pmlmepriv->ht_op_mode &=
			~HT_INFO_OPERATION_MODE_NON_HT_STA_PRESENT;
		op_mode_changes++;
	}

	/* Note: currently we switch to the MIXED op mode if HT non-greenfield
	 * station is associated. Probably it's a theoretical case, since
	 * it looks like all known HT STAs support greenfield.
	 */
	new_op_mode = 0;
	if (pmlmepriv->num_sta_no_ht ||
	    (pmlmepriv->ht_op_mode & HT_INFO_OPERATION_MODE_NON_GF_DEVS_PRESENT))
		new_op_mode = OP_MODE_MIXED;
	else if ((phtpriv_ap->ht_cap.cap_info & IEEE80211_HT_CAP_SUP_WIDTH)
		 && pmlmepriv->num_sta_ht_20mhz)
		new_op_mode = OP_MODE_20MHZ_HT_STA_ASSOCED;
	else if (pmlmepriv->olbc_ht)
		new_op_mode = OP_MODE_MAY_BE_LEGACY_STAS;
	else
		new_op_mode = OP_MODE_PURE;

	cur_op_mode = pmlmepriv->ht_op_mode & HT_INFO_OPERATION_MODE_OP_MODE_MASK;
	if (cur_op_mode != new_op_mode) {
		pmlmepriv->ht_op_mode &= ~HT_INFO_OPERATION_MODE_OP_MODE_MASK;
		pmlmepriv->ht_op_mode |= new_op_mode;
		op_mode_changes++;
	}

	DBG_871X("%s new operation mode=0x%X changes=%d\n",
		   __FUNCTION__, pmlmepriv->ht_op_mode, op_mode_changes);

	return op_mode_changes;

}

void associated_clients_update(struct rtl_priv *rtlpriv, uint8_t updated)
{
	//update associcated stations cap.
	if (updated == true) {
		struct list_head	*phead, *plist;
		struct sta_info *psta=NULL;
		struct sta_priv *pstapriv = &rtlpriv->stapriv;

		spin_lock_bh(&pstapriv->asoc_list_lock);

		phead = &pstapriv->asoc_list;
		plist = get_next(phead);

		//check asoc_queue
		while ((rtw_end_of_queue_search(phead, plist)) == false) {
			psta = container_of(plist, struct sta_info, asoc_list);

			plist = get_next(plist);

			VCS_update(rtlpriv, psta);
		}

		spin_unlock_bh(&pstapriv->asoc_list_lock);

	}

}

/* called > TSR LEVEL for USB or SDIO Interface*/
void bss_cap_update_on_sta_join(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	uint8_t beacon_updated = false;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv *pmlmeext = &(rtlpriv->mlmeextpriv);

	if (!(psta->flags & WLAN_STA_SHORT_PREAMBLE)) {
		if (!psta->no_short_preamble_set) {
			psta->no_short_preamble_set = 1;

			pmlmepriv->num_sta_no_short_preamble++;

			if ((pmlmeext->cur_wireless_mode > WIRELESS_11B) &&
		     		(pmlmepriv->num_sta_no_short_preamble == 1)) {
				beacon_updated = true;
				update_beacon(rtlpriv, 0xFF, NULL, true);
			}

		}
	} else {
		if (psta->no_short_preamble_set) {
			psta->no_short_preamble_set = 0;

			pmlmepriv->num_sta_no_short_preamble--;

			if ((pmlmeext->cur_wireless_mode > WIRELESS_11B) &&
		     		(pmlmepriv->num_sta_no_short_preamble == 0)) {
				beacon_updated = true;
				update_beacon(rtlpriv, 0xFF, NULL, true);
			}

		}
	}

	if (psta->flags & WLAN_STA_NONERP) {
		if (!psta->nonerp_set) {
			psta->nonerp_set = 1;

			pmlmepriv->num_sta_non_erp++;

			if (pmlmepriv->num_sta_non_erp == 1) {
				beacon_updated = true;
				update_beacon(rtlpriv, _ERPINFO_IE_, NULL, true);
			}
		}

	} else {
		if (psta->nonerp_set) {
			psta->nonerp_set = 0;

			pmlmepriv->num_sta_non_erp--;

			if (pmlmepriv->num_sta_non_erp == 0) {
				beacon_updated = true;
				update_beacon(rtlpriv, _ERPINFO_IE_, NULL, true);
			}
		}

	}

	if (!(psta->capability & WLAN_CAPABILITY_SHORT_SLOT)) {
		if (!psta->no_short_slot_time_set) {
			psta->no_short_slot_time_set = 1;

			pmlmepriv->num_sta_no_short_slot_time++;

			if ((pmlmeext->cur_wireless_mode > WIRELESS_11B) &&
		   		 (pmlmepriv->num_sta_no_short_slot_time == 1)) {
				beacon_updated = true;
				update_beacon(rtlpriv, 0xFF, NULL, true);
			}

		}
	} else {
		if (psta->no_short_slot_time_set) {
			psta->no_short_slot_time_set = 0;

			pmlmepriv->num_sta_no_short_slot_time--;

			if ((pmlmeext->cur_wireless_mode > WIRELESS_11B) &&
		   		 (pmlmepriv->num_sta_no_short_slot_time == 0)) {
				beacon_updated = true;
				update_beacon(rtlpriv, 0xFF, NULL, true);
			}
		}
	}

	if (psta->flags & WLAN_STA_HT) {
		u16 ht_capab = le16_to_cpu(psta->htpriv.ht_cap.cap_info);

		DBG_871X("HT: STA " MAC_FMT " HT Capabilities "
			   "Info: 0x%04x\n", MAC_ARG(psta->hwaddr), ht_capab);

		if (psta->no_ht_set) {
			psta->no_ht_set = 0;
			pmlmepriv->num_sta_no_ht--;
		}

		if ((ht_capab & IEEE80211_HT_CAP_GRN_FLD) == 0) {
			if (!psta->no_ht_gf_set) {
				psta->no_ht_gf_set = 1;
				pmlmepriv->num_sta_ht_no_gf++;
			}
			DBG_871X("%s STA " MAC_FMT " - no "
				   "greenfield, num of non-gf stations %d\n",
				   __FUNCTION__, MAC_ARG(psta->hwaddr),
				   pmlmepriv->num_sta_ht_no_gf);
		}

		if ((ht_capab & IEEE80211_HT_CAP_SUP_WIDTH) == 0) {
			if (!psta->ht_20mhz_set) {
				psta->ht_20mhz_set = 1;
				pmlmepriv->num_sta_ht_20mhz++;
			}
			DBG_871X("%s STA " MAC_FMT " - 20 MHz HT, "
				   "num of 20MHz HT STAs %d\n",
				   __FUNCTION__, MAC_ARG(psta->hwaddr),
				   pmlmepriv->num_sta_ht_20mhz);
		}

	} else {
		if (!psta->no_ht_set) {
			psta->no_ht_set = 1;
			pmlmepriv->num_sta_no_ht++;
		}
		if (pmlmepriv->htpriv.ht_option == true) {
			DBG_871X("%s STA " MAC_FMT
				   " - no HT, num of non-HT stations %d\n",
				   __FUNCTION__, MAC_ARG(psta->hwaddr),
				   pmlmepriv->num_sta_no_ht);
		}
	}

	if (rtw_ht_operation_update(rtlpriv) > 0) {
		update_beacon(rtlpriv, _HT_CAPABILITY_IE_, NULL, false);
		update_beacon(rtlpriv, _HT_ADD_INFO_IE_, NULL, true);
	}

	//update associcated stations cap.
	associated_clients_update(rtlpriv,  beacon_updated);

	DBG_871X("%s, updated=%d\n", __func__, beacon_updated);

}

uint8_t bss_cap_update_on_sta_leave(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	uint8_t beacon_updated = false;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv *pmlmeext = &(rtlpriv->mlmeextpriv);

	if (!psta)
		return beacon_updated;

	if (psta->no_short_preamble_set) {
		psta->no_short_preamble_set = 0;
		pmlmepriv->num_sta_no_short_preamble--;
		if (pmlmeext->cur_wireless_mode > WIRELESS_11B
		    && pmlmepriv->num_sta_no_short_preamble == 0) {
			beacon_updated = true;
			update_beacon(rtlpriv, 0xFF, NULL, true);
		}
	}

	if (psta->nonerp_set) {
		psta->nonerp_set = 0;
		pmlmepriv->num_sta_non_erp--;
		if (pmlmepriv->num_sta_non_erp == 0) {
			beacon_updated = true;
			update_beacon(rtlpriv, _ERPINFO_IE_, NULL, true);
		}
	}

	if (psta->no_short_slot_time_set) {
		psta->no_short_slot_time_set = 0;
		pmlmepriv->num_sta_no_short_slot_time--;
		if (pmlmeext->cur_wireless_mode > WIRELESS_11B
		    && pmlmepriv->num_sta_no_short_slot_time == 0) {
			beacon_updated = true;
			update_beacon(rtlpriv, 0xFF, NULL, true);
		}
	}

	if (psta->no_ht_gf_set) {
		psta->no_ht_gf_set = 0;
		pmlmepriv->num_sta_ht_no_gf--;
	}

	if (psta->no_ht_set) {
		psta->no_ht_set = 0;
		pmlmepriv->num_sta_no_ht--;
	}

	if (psta->ht_20mhz_set) {
		psta->ht_20mhz_set = 0;
		pmlmepriv->num_sta_ht_20mhz--;
	}

	if (rtw_ht_operation_update(rtlpriv) > 0)
	{
		update_beacon(rtlpriv, _HT_CAPABILITY_IE_, NULL, false);
		update_beacon(rtlpriv, _HT_ADD_INFO_IE_, NULL, true);
	}

	//update associcated stations cap.
	//associated_clients_update(rtlpriv,  beacon_updated); //move it to avoid deadlock

	DBG_871X("%s, updated=%d\n", __func__, beacon_updated);

	return beacon_updated;

}

uint8_t ap_free_sta(struct rtl_priv *rtlpriv, struct sta_info *psta, bool active, u16 reason)
{
	uint8_t beacon_updated = false;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;

	if (!psta)
		return beacon_updated;

	if (active == true) {
		//tear down Rx AMPDU
		send_delba(rtlpriv, 0, psta->hwaddr);// recipient

		//tear down TX AMPDU
		send_delba(rtlpriv, 1, psta->hwaddr);// // originator

		issue_deauth(rtlpriv, psta->hwaddr, reason);
	}

	psta->htpriv.agg_enable_bitmap = 0x0;//reset
	psta->htpriv.candidate_tid_bitmap = 0x0;//reset


	//report_del_sta_event(rtlpriv, psta->hwaddr, reason);

	//clear cam entry / key
	//clear_cam_entry(rtlpriv, (psta->mac_id + 3));
	rtw_clearstakey_cmd(rtlpriv, (uint8_t *)psta, (uint8_t)rtw_get_camid(psta->mac_id), true);


	spin_lock_bh(&psta->lock);
	psta->state &= ~_FW_LINKED;
	spin_unlock_bh(&psta->lock);

	{
		rtw_indicate_sta_disassoc_event(rtlpriv, psta);
	}

	report_del_sta_event(rtlpriv, psta->hwaddr, reason);

	beacon_updated = bss_cap_update_on_sta_leave(rtlpriv, psta);

	spin_lock_bh(&(pstapriv->sta_hash_lock));
	rtw_free_stainfo(rtlpriv, psta);
	spin_unlock_bh(&(pstapriv->sta_hash_lock));


	return beacon_updated;

}

int rtw_ap_inform_ch_switch(struct rtl_priv *rtlpriv, uint8_t new_ch, uint8_t ch_offset)
{
	struct list_head	*phead, *plist;
	int ret=0;
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t bc_addr[ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};

	if ((pmlmeinfo->state&0x03) != WIFI_FW_AP_STATE)
		return ret;

	DBG_871X(FUNC_NDEV_FMT" with ch:%u, offset:%u\n",
		FUNC_NDEV_ARG(rtlpriv->ndev), new_ch, ch_offset);

	spin_lock_bh(&pstapriv->asoc_list_lock);
	phead = &pstapriv->asoc_list;
	plist = get_next(phead);

	/* for each sta in asoc_queue */
	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		psta = container_of(plist, struct sta_info, asoc_list);
		plist = get_next(plist);

		issue_action_spct_ch_switch(rtlpriv, psta->hwaddr, new_ch, ch_offset);
		psta->expire_to = ((pstapriv->expire_to * 2) > 5) ? 5 : (pstapriv->expire_to * 2);
	}
	spin_unlock_bh(&pstapriv->asoc_list_lock);

	issue_action_spct_ch_switch(rtlpriv, bc_addr, new_ch, ch_offset);

	return ret;
}

int rtw_sta_flush(struct rtl_priv *rtlpriv)
{
	struct list_head	*phead, *plist;
	int ret=0;
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t bc_addr[ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};

	DBG_871X(FUNC_NDEV_FMT"\n", FUNC_NDEV_ARG(rtlpriv->ndev));

	if ((pmlmeinfo->state&0x03) != WIFI_FW_AP_STATE)
		return ret;


	spin_lock_bh(&pstapriv->asoc_list_lock);
	phead = &pstapriv->asoc_list;
	plist = get_next(phead);

	//free sta asoc_queue
	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		psta = container_of(plist, struct sta_info, asoc_list);

		plist = get_next(plist);

		list_del_init(&psta->asoc_list);
		pstapriv->asoc_list_cnt--;

		//spin_unlock_bh(&pstapriv->asoc_list_lock, &irqL);
		ap_free_sta(rtlpriv, psta, true, WLAN_REASON_DEAUTH_LEAVING);
		//spin_lock_bh(&pstapriv->asoc_list_lock, &irqL);
	}
	spin_unlock_bh(&pstapriv->asoc_list_lock);


	issue_deauth(rtlpriv, bc_addr, WLAN_REASON_DEAUTH_LEAVING);

	associated_clients_update(rtlpriv, true);

	return ret;

}

/* called > TSR LEVEL for USB or SDIO Interface*/
void sta_info_update(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	int flags = psta->flags;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);


	//update wmm cap.
	if (WLAN_STA_WME&flags)
		psta->qos_option = 1;
	else
		psta->qos_option = 0;

	if (pmlmepriv->qospriv.qos_option == 0)
		psta->qos_option = 0;


	//update 802.11n ht cap.
	if (WLAN_STA_HT&flags) {
		psta->htpriv.ht_option = true;
		psta->qos_option = 1;
	} else {
		psta->htpriv.ht_option = false;
	}

	if (pmlmepriv->htpriv.ht_option == false)
		psta->htpriv.ht_option = false;

	//update 802.11AC vht cap.
	if (WLAN_STA_VHT&flags) {
		psta->vhtpriv.vht_option = true;
	} else {
		psta->vhtpriv.vht_option = false;
	}

	if (pmlmepriv->vhtpriv.vht_option == false)
		psta->vhtpriv.vht_option = false;


	update_sta_info_apmode(rtlpriv, psta);


}

/* called >= TSR LEVEL for USB or SDIO Interface*/
void ap_sta_info_defer_update(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	if (psta->state & _FW_LINKED) {
		pmlmeinfo->FW_sta_info[psta->mac_id].psta = psta;

		//add ratid
		add_RATid(rtlpriv, psta, 0);//DM_RATR_STA_INIT
	}
}

/* restore hw setting from sw data structures */
void rtw_ap_restore_network(struct rtl_priv *rtlpriv)
{
	struct mlme_priv *mlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct sta_priv * pstapriv = &rtlpriv->stapriv;
	struct sta_info *psta;
	struct security_priv* psecuritypriv= &(rtlpriv->securitypriv);
	struct list_head	*phead, *plist;
	uint8_t chk_alive_num = 0;
	char chk_alive_list[NUM_STA];
	int i;

	rtw_setopmode_cmd(rtlpriv, Ndis802_11APMode);

	set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);

	start_bss_network(rtlpriv, (uint8_t *)&mlmepriv->cur_network.network);

	if ((rtlpriv->securitypriv.dot11PrivacyAlgrthm == TKIP_ENCRYPTION) ||
		(rtlpriv->securitypriv.dot11PrivacyAlgrthm == AESCCMP_ENCRYPTION)) {
		/* restore group key, WEP keys is restored in ips_leave() */
		rtw_set_key(rtlpriv, psecuritypriv, psecuritypriv->dot118021XGrpKeyid, 0);
	}

	/* per sta pairwise key and settings */
	if ((rtlpriv->securitypriv.dot11PrivacyAlgrthm != TKIP_ENCRYPTION) &&
		(rtlpriv->securitypriv.dot11PrivacyAlgrthm != AESCCMP_ENCRYPTION)) {
		return;
	}

	spin_lock_bh(&pstapriv->asoc_list_lock);

	phead = &pstapriv->asoc_list;
	plist = get_next(phead);

	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		int stainfo_offset;

		psta = container_of(plist, struct sta_info, asoc_list);
		plist = get_next(plist);

		stainfo_offset = rtw_stainfo_offset(pstapriv, psta);
		if (stainfo_offset_valid(stainfo_offset)) {
			chk_alive_list[chk_alive_num++] = stainfo_offset;
		}
	}

	spin_unlock_bh(&pstapriv->asoc_list_lock);

	for (i = 0; i < chk_alive_num; i++) {
		psta = rtw_get_stainfo_by_offset(pstapriv, chk_alive_list[i]);

		if (psta == NULL) {
			DBG_871X(FUNC_ADPT_FMT" sta_info is null\n", FUNC_ADPT_ARG(rtlpriv));
		} else if (psta->state &_FW_LINKED) {
			Update_RA_Entry(rtlpriv, psta);
			//pairwise key
			rtw_setstakey_cmd(rtlpriv, (unsigned char *)psta, true);
		}
	}

}

void start_ap_mode(struct rtl_priv *rtlpriv)
{
	int i;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct wlan_acl_pool *pacl_list = &pstapriv->acl_list;

	pmlmepriv->update_bcn = false;

	//init_mlme_ap_info(rtlpriv);
	pmlmeext->bstart_bss = false;

	pmlmepriv->num_sta_non_erp = 0;

	pmlmepriv->num_sta_no_short_slot_time = 0;

	pmlmepriv->num_sta_no_short_preamble = 0;

	pmlmepriv->num_sta_ht_no_gf = 0;
	pmlmepriv->num_sta_no_ht = 0;
	pmlmepriv->num_sta_ht_20mhz = 0;

	pmlmepriv->olbc = false;

	pmlmepriv->olbc_ht = false;

	pmlmepriv->ht_op_mode = 0;

	for(i=0; i<NUM_STA; i++)
		pstapriv->sta_aid[i] = NULL;

	pmlmepriv->wps_beacon_ie = NULL;
	pmlmepriv->wps_probe_resp_ie = NULL;
	pmlmepriv->wps_assoc_resp_ie = NULL;

	pmlmepriv->p2p_beacon_ie = NULL;
	pmlmepriv->p2p_probe_resp_ie = NULL;


	//for ACL
	INIT_LIST_HEAD(&(pacl_list->acl_node_q.list));
	pacl_list->num = 0;
	pacl_list->mode = 0;
	for(i = 0; i < NUM_ACL; i++) {
		INIT_LIST_HEAD(&pacl_list->aclnode[i].list);
		pacl_list->aclnode[i].valid = false;
	}

}

void stop_ap_mode(struct rtl_priv *rtlpriv)
{
	struct list_head	*phead, *plist;
	struct rtw_wlan_acl_node *paclnode;
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct wlan_acl_pool *pacl_list = &pstapriv->acl_list;
	struct __queue	*pacl_node_q = &pacl_list->acl_node_q;

	pmlmepriv->update_bcn = false;
	pmlmeext->bstart_bss = false;

	//reset and init security priv , this can refine with rtw_reset_securitypriv
	memset((unsigned char *)&rtlpriv->securitypriv, 0, sizeof (struct security_priv));
	rtlpriv->securitypriv.ndisauthtype = Ndis802_11AuthModeOpen;
	rtlpriv->securitypriv.ndisencryptstatus = Ndis802_11WEPDisabled;

	//for ACL
	spin_lock_bh(&(pacl_node_q->lock));
	phead = get_list_head(pacl_node_q);
	plist = get_next(phead);
	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		paclnode = container_of(plist, struct rtw_wlan_acl_node, list);
		plist = get_next(plist);

		if (paclnode->valid == true) {
			paclnode->valid = false;

			list_del_init(&paclnode->list);

			pacl_list->num--;
		}
	}
	spin_unlock_bh(&(pacl_node_q->lock));

	DBG_871X("%s, free acl_node_queue, num=%d\n", __func__, pacl_list->num);

	rtw_sta_flush(rtlpriv);

	//free_assoc_sta_resources
	rtw_free_all_stainfo(rtlpriv);

	psta = rtw_get_bcmc_stainfo(rtlpriv);
	spin_lock_bh(&(pstapriv->sta_hash_lock));
	rtw_free_stainfo(rtlpriv, psta);
	spin_unlock_bh(&(pstapriv->sta_hash_lock));

	rtw_init_bcmc_stainfo(rtlpriv);

	rtw_free_mlme_priv_ie_data(pmlmepriv);

}

#endif //CONFIG_AP_MODE

