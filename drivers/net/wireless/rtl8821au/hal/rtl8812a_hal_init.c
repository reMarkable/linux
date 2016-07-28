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
#define _RTL8812A_HAL_INIT_C_
#include <drv_types.h>
#include <rtl8812a_hal.h>

/*
 *-------------------------------------------------------------------------
 *
 * LLT R/W/Init function
 *
 *-------------------------------------------------------------------------
 */

/*====================================================================================
 *
 * 20100209 Joseph:
 * This function is used only for 92C to set REG_BCN_CTRL(0x550) register.
 * We just reserve the value of the register in variable pHalData->RegBcnCtrlVal and then operate
 * the value of the register via atomic operation.
 * This prevents from race condition when setting this register.
 * The value of pHalData->RegBcnCtrlVal is initialized in HwConfigureRTL8192CE() function.
 *
 */

void InitializeFirmwareVars8812(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	/* Init Fw LPS related. */
	rtlpriv->pwrctrlpriv.fw_current_inpsmode = false;
	/* Init H2C counter. by tynli. 2009.12.09. */
	rtlhal->last_hmeboxnum = 0;
}

void rtl8812_free_hal_data(struct rtl_priv *rtlpriv)
{
	if (rtlpriv->HalData) {
			rtw_mfree(rtlpriv->HalData);
		rtlpriv->HalData = NULL;
	}
}

/*
 * ===========================================================
 * 				Efuse related code
 * ===========================================================
 */


/*
 * ULLI need to refactoring for rtlwifi-lib
 * static bool _rt8812au_phy_set_rf_power_state(struct ieee80211_hw *hw,
 * 					    enum rf_pwrstate rfpwr_state)
 *
 */

void rtw_set_sta_info(struct rtl_priv *rtlpriv, struct sta_info *psta, bool bSet)
{
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);
	/* _irqL irqL; */

	if (bSet) {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "### Set STA_(%d) info\n", psta->mac_id);
		rtldm->pODM_StaInfo[psta->mac_id] = psta;
#if (RATE_ADAPTIVE_SUPPORT == 1)
		ODM_RAInfo_Init(podmpriv, psta->mac_id);
#endif
	} else {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "### Clean STA_(%d) info\n", psta->mac_id);
		/* spin_lock_bh(&pHalData->odm_stainfo_lock, &irqL); */
		rtldm->pODM_StaInfo[psta->mac_id] = NULL;
		/* spin_unlock_bh(&pHalData->odm_stainfo_lock, &irqL); */
	}
}

void UpdateHalRAMask8812A(struct rtl_priv *rtlpriv, uint32_t mac_id, uint8_t rssi_level)
{
	/* volatile unsigned int result; */
	uint8_t	init_rate = 0;
	uint8_t	networkType, raid;
	uint32_t	mask, rate_bitmap;
	uint8_t	shortGIrate = false;
	int	supportRateNum = 0;
	uint8_t	arg[4] = {0};
	struct sta_info	*psta;
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);
	/* struct dm_priv	*pdmpriv = &pHalData->dmpriv; */
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);

	if (mac_id >= NUM_STA) {	/* CAM_SIZE */
		return;
	}

	psta = pmlmeinfo->FW_sta_info[mac_id].psta;
	if (psta == NULL) {
		return;
	}

	switch (mac_id) {
	case 0:
		/* for infra mode */
		supportRateNum = rtw_get_rateset_len(cur_network->SupportedRates);
		networkType = judge_network_type(rtlpriv, cur_network->SupportedRates, supportRateNum);
		/* pmlmeext->cur_wireless_mode = networkType; */
		/* raid = networktype_to_raid(networkType); */
		raid = rtw_hal_networktype_to_raid(rtlpriv, networkType);

		mask = update_supported_rate(cur_network->SupportedRates, supportRateNum);
		if (pmlmeinfo->VHT_enable) {
			mask |= (rtw_vht_rate_to_bitmap(psta->vhtpriv.vht_mcs_map) << 12);
			shortGIrate = psta->vhtpriv.sgi;
		} else
			{
				mask |= (pmlmeinfo->HT_enable) ? update_MCS_rate(&(pmlmeinfo->HT_caps)) : 0;
				if (support_short_GI(rtlpriv, &(pmlmeinfo->HT_caps)))
					shortGIrate = true;
			}

		break;

	case 1:
		/* for broadcast/multicast */
		supportRateNum = rtw_get_rateset_len(pmlmeinfo->FW_sta_info[mac_id].SupportedRates);
		if (pmlmeext->cur_wireless_mode & WIRELESS_11B)
			networkType = WIRELESS_11B;
		else
			networkType = WIRELESS_11G;

		/* raid = networktype_to_raid(networkType); */
		raid = rtw_hal_networktype_to_raid(rtlpriv, networkType);
		mask = update_basic_rate(cur_network->SupportedRates, supportRateNum);

		break;

	default:
		/*for each sta in IBSS */
		supportRateNum = rtw_get_rateset_len(pmlmeinfo->FW_sta_info[mac_id].SupportedRates);
		networkType = judge_network_type(rtlpriv, pmlmeinfo->FW_sta_info[mac_id].SupportedRates, supportRateNum) & 0xf;
		/*
		 * pmlmeext->cur_wireless_mode = networkType;
		 * raid = networktype_to_raid(networkType);
		 */
		raid = rtw_hal_networktype_to_raid(rtlpriv, networkType);
		mask = update_supported_rate(cur_network->SupportedRates, supportRateNum);

		/* todo: support HT in IBSS */

		break;
	}

	/* mask &=0x0fffffff; */
	rate_bitmap = ODM_Get_Rate_Bitmap(rtlpriv, mac_id, mask, rssi_level);
		RT_TRACE(rtlpriv, COMP_RATE, DBG_LOUD, "%s => mac_id:%d, networkType:0x%02x, mask:0x%08x\n\t ==> rssi_level:%d, rate_bitmap:0x%08x\n",
			__FUNCTION__, mac_id, networkType, mask, rssi_level, rate_bitmap);

	mask &= rate_bitmap;
	init_rate = get_highest_rate_idx(mask)&0x3f;

	/*
	 * arg[0] = macid
	 * arg[1] = raid
	 * arg[2] = shortGIrate
	 * arg[3] = init_rate
	 */

	arg[0] = mac_id;
	arg[1] = raid;
	arg[2] = shortGIrate;
	arg[3] = init_rate;

	rtl8812_set_raid_cmd(rtlpriv, mask, arg);

	/* set ra_id */
	psta->raid = raid;
	psta->init_rate = init_rate;
}
