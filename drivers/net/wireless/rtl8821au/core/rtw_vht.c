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
#define _RTW_VHT_C

#include <drv_types.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

/*
 * 20/40/80,	ShortGI,	MCS Rate
 */

const u16 VHT_MCS_DATA_RATE[3][2][20] = {
	{ {		/* Long GI, 20MHz */
			13, 26, 39, 52, 78, 104, 117, 130, 156, 156,
			26, 52, 78, 104, 156, 208, 234, 260, 312, 312
		}, {	/* Short GI, 20MHz */
			14, 29, 43, 58, 87, 116, 130, 144, 173, 173,
			29, 58, 87, 116, 173, 231, 260, 289, 347, 347
		}
	}, {
		{	/* Long GI, 40MHz */
			27, 54, 81, 108, 162, 216, 243, 270, 324, 360,
			54, 108, 162, 216, 324, 432, 486, 540, 648, 720
		}, {	/* Short GI, 40MHz */
			30, 60, 90, 120, 180, 240, 270, 300, 360, 400,
			60, 120, 180, 240, 360, 480, 540, 600, 720, 800
		}
	}, {
		{	/* Long GI, 80MHz */
			59, 117,  176, 234, 351, 468, 527, 585, 702, 780,
			117, 234, 351, 468, 702, 936, 1053, 1170, 1404, 1560
		},  {	/* Short GI, 80MHz */
			65, 130, 195, 260, 390, 520, 585, 650, 780, 867,
			130, 260, 390, 520, 780, 1040, 1170, 1300, 1560, 1734
		}
	} };

uint8_t	rtw_get_vht_highest_rate(struct rtl_priv *rtlpriv, uint8_t *pvht_mcs_map)
{
	uint8_t	i, j;
	uint8_t	bit_map;
	uint8_t	vht_mcs_rate = 0;

	for (i = 0; i < 2; i++) {
		if (pvht_mcs_map[i] != 0xff) {
			for (j = 0; j < 8; j += 2) {
				bit_map = (pvht_mcs_map[i] >> j) & 3;

				if (bit_map != 3)
					/*
					 * VHT rate indications begin from 0x90
					 */
					vht_mcs_rate = MGN_VHT1SS_MCS7 + 10*j/2 + i*40 + bit_map;
			}
		}
	}

	/*
	 * DBG_871X("HighestVHTMCSRate is %x\n", vht_mcs_rate);
	 */
	return vht_mcs_rate;
}

u16 rtw_vht_data_rate(uint8_t bw, uint8_t short_GI, uint8_t vht_mcs_rate)
{
	if (vht_mcs_rate > MGN_VHT2SS_MCS9)
		vht_mcs_rate = MGN_VHT2SS_MCS9;

	return VHT_MCS_DATA_RATE[bw][short_GI][((vht_mcs_rate - MGN_VHT1SS_MCS0)&0x3f)];
}


uint32_t	rtw_vht_rate_to_bitmap(uint8_t *pVHTRate)
{

	uint8_t	i, j, tmpRate;
	uint32_t RateBitmap = 0;

	for (i = j = 0; i < 4; i += 2, j += 10) {
		tmpRate = (pVHTRate[0] >> i) & 3;

		switch (tmpRate) {
		case 2:
			RateBitmap = RateBitmap | (0x03ff << j);
			break;
		case 1:
			RateBitmap = RateBitmap | (0x01ff << j);
		break;

		case 0:
			RateBitmap = RateBitmap | (0x00ff << j);
		break;

		default:
			break;
		}
	}

	return RateBitmap;
}

void update_sta_vht_info_apmode(struct rtl_priv *rtlpriv, void *psta)
{
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct vht_priv *pvhtpriv_ap = &pmlmepriv->vhtpriv;
	struct vht_priv *pvhtpriv_sta = &((struct sta_info *)psta)->vhtpriv;
	struct ht_priv *phtpriv_sta = &((struct sta_info *)psta)->htpriv;
	uint8_t	cur_ldpc_cap = 0,
		cur_stbc_cap = 0,
		cur_beamform_cap = 0;
	uint8_t	*pcap_mcs;

	if (pvhtpriv_sta->vht_option == false)
		return;

	/* B4 Rx LDPC */
	if (TEST_FLAG(pvhtpriv_ap->ldpc_cap, LDPC_VHT_ENABLE_TX))
		SET_FLAG(cur_ldpc_cap, GET_VHT_CAPABILITY_ELE_RX_LDPC(pvhtpriv_sta->vht_cap) ? (LDPC_VHT_ENABLE_TX | LDPC_VHT_CAP_TX) : 0);

	pvhtpriv_sta->ldpc_cap = cur_ldpc_cap;
	DBG_871X("Current STA VHT LDPC = %02X\n", cur_ldpc_cap);

	if (pvhtpriv_sta->vht_bwmode > pvhtpriv_ap->vht_bwmode)
		pvhtpriv_sta->vht_bwmode = pvhtpriv_ap->vht_bwmode;

	if (pvhtpriv_sta->vht_bwmode == CHANNEL_WIDTH_80) {
		/* B5 Short GI for 80 MHz */
		pvhtpriv_sta->sgi = (GET_VHT_CAPABILITY_ELE_SHORT_GI80M(pvhtpriv_sta->vht_cap) & pvhtpriv_ap->sgi) ? true : false;
		DBG_871X("Current STA ShortGI80MHz = %d\n", pvhtpriv_sta->sgi);
	} else if (pvhtpriv_sta->vht_bwmode >= CHANNEL_WIDTH_160) {
		/* B5 Short GI for 80 MHz */
		pvhtpriv_sta->sgi = (GET_VHT_CAPABILITY_ELE_SHORT_GI160M(pvhtpriv_sta->vht_cap) & pvhtpriv_ap->sgi) ? true : false;
		DBG_871X("Current STA ShortGI160MHz = %d\n", pvhtpriv_sta->sgi);
	} else {
		pvhtpriv_sta->sgi = phtpriv_sta->sgi;
	}

	/* B8 B9 B10 Rx STBC */
	if (TEST_FLAG(pvhtpriv_ap->stbc_cap, STBC_VHT_ENABLE_TX)
	   && GET_VHT_CAPABILITY_ELE_RX_STBC(pvhtpriv_sta->vht_cap)) {
		SET_FLAG(cur_stbc_cap, (STBC_VHT_ENABLE_TX | STBC_VHT_CAP_TX));
	}
	pvhtpriv_sta->stbc_cap = cur_stbc_cap;
	DBG_871X("Current STA VHT STBC = %02X\n", cur_stbc_cap);

	/*
	 * B11
	 * SU Beamformer Capable, the target supports
	 * Beamformer and we are Beamformee
	 */
	if (TEST_FLAG(pvhtpriv_ap->beamform_cap, BEAMFORMING_VHT_BEAMFORMER_ENABLE)
	   && GET_VHT_CAPABILITY_ELE_SU_BFEE(pvhtpriv_sta->vht_cap)) {
		SET_FLAG(cur_beamform_cap, BEAMFORMING_VHT_BEAMFORMEE_ENABLE);
	}

	/* B12
	 * SU Beamformee Capable, the target supports
	 * Beamformee and we are Beamformer
	 */
	if (TEST_FLAG(pvhtpriv_ap->beamform_cap, BEAMFORMING_VHT_BEAMFORMEE_ENABLE)
	   && GET_VHT_CAPABILITY_ELE_SU_BFER(pvhtpriv_sta->vht_cap)) {
		SET_FLAG(cur_beamform_cap, BEAMFORMING_VHT_BEAMFORMER_ENABLE);
	}
	pvhtpriv_sta->beamform_cap = cur_beamform_cap;
	DBG_871X("Current VHT Beamforming Setting = %02X\n", cur_beamform_cap);

	/*
	 * B23 B24 B25
	 * Maximum A-MPDU Length Exponent
	 */
	pvhtpriv_sta->ampdu_len = GET_VHT_CAPABILITY_ELE_MAX_RXAMPDU_FACTOR(pvhtpriv_sta->vht_cap);

	pcap_mcs = GET_VHT_CAPABILITY_ELE_RX_MCS(pvhtpriv_sta->vht_cap);
	memcpy(pvhtpriv_sta->vht_mcs_map, pcap_mcs, 2);

	pvhtpriv_sta->vht_highest_rate = rtw_get_vht_highest_rate(rtlpriv, pvhtpriv_sta->vht_mcs_map);

}

void update_hw_vht_param(struct rtl_priv *rtlpriv)
{
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv *pvhtpriv = &pmlmepriv->vhtpriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t	ht_AMPDU_len;

	ht_AMPDU_len = pmlmeinfo->HT_caps.u.HT_cap_element.AMPDU_para & 0x03;

	if (pvhtpriv->ampdu_len > ht_AMPDU_len)
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AMPDU_FACTOR, (uint8_t *)(&pvhtpriv->ampdu_len));
}

void VHT_caps_handler(struct rtl_priv *rtlpriv, PNDIS_802_11_VARIABLE_IEs pIE)
{
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv *pvhtpriv = &pmlmepriv->vhtpriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t	cur_ldpc_cap = 0,
		cur_stbc_cap = 0,
		cur_beamform_cap = 0,
		rf_type = RF_1T1R;
	uint8_t	*pcap_mcs;
	uint8_t	vht_mcs[2];

	if (pIE == NULL)
		return;

	if (pvhtpriv->vht_option == false)
		return;

	pmlmeinfo->VHT_enable = 1;

	/*
	 * B4
	 * Rx LDPC
	 */
	if (TEST_FLAG(pvhtpriv->ldpc_cap, LDPC_VHT_ENABLE_TX))
		SET_FLAG(cur_ldpc_cap, GET_VHT_CAPABILITY_ELE_RX_LDPC(pIE->data) ? (LDPC_VHT_ENABLE_TX | LDPC_VHT_CAP_TX) : 0);

	pvhtpriv->ldpc_cap = cur_ldpc_cap;
	DBG_871X("Current VHT LDPC Setting = %02X\n", cur_ldpc_cap);

	/*
	 * B5
	 * Short GI for 80 MHz
	 */
	pvhtpriv->sgi = (GET_VHT_CAPABILITY_ELE_SHORT_GI80M(pIE->data) & pvhtpriv->sgi) ? true : false;
	DBG_871X("Current ShortGI80MHz = %d\n", pvhtpriv->sgi);

	/*
	 * B8 B9 B10
	 * Rx STBC
	 */
	if (TEST_FLAG(pvhtpriv->stbc_cap, STBC_VHT_ENABLE_TX)
	   && GET_VHT_CAPABILITY_ELE_RX_STBC(pIE->data)) {
		SET_FLAG(cur_stbc_cap, (STBC_VHT_ENABLE_TX | STBC_VHT_CAP_TX));
	}

	pvhtpriv->stbc_cap = cur_stbc_cap;
	DBG_871X("Current VHT STBC Setting = %02X\n", cur_stbc_cap);

	/* B11
	 * SU Beamformer Capable, the target
	 * supports Beamformer and we are Beamformee
	 */
	if (TEST_FLAG(pvhtpriv->beamform_cap, BEAMFORMING_VHT_BEAMFORMER_ENABLE)
	   && GET_VHT_CAPABILITY_ELE_SU_BFEE(pIE->data)) {
		SET_FLAG(cur_beamform_cap, BEAMFORMING_VHT_BEAMFORMEE_ENABLE);
	}

	/* B12
	 * SU Beamformee Capable, the target
	 * supports Beamformee and we are Beamformer
	 */
	if (TEST_FLAG(pvhtpriv->beamform_cap, BEAMFORMING_VHT_BEAMFORMEE_ENABLE)
	   && GET_VHT_CAPABILITY_ELE_SU_BFER(pIE->data)) {
		SET_FLAG(cur_beamform_cap, BEAMFORMING_VHT_BEAMFORMER_ENABLE);
	}

	pvhtpriv->beamform_cap = cur_beamform_cap;
	DBG_871X("Current VHT Beamforming Setting = %02X\n", cur_beamform_cap);

	/*
	 * B23 B24 B25
	 * Maximum A-MPDU Length Exponent
	 */

	pvhtpriv->ampdu_len = GET_VHT_CAPABILITY_ELE_MAX_RXAMPDU_FACTOR(pIE->data);

	pcap_mcs = GET_VHT_CAPABILITY_ELE_RX_MCS(pIE->data);
	memcpy(vht_mcs, pcap_mcs, 2);

	rf_type = rtlpriv->phy.rf_type;
	
	if ((rf_type == RF_1T1R) || (rf_type == RF_1T2R))
		vht_mcs[0] |= 0xfc;
	else if (rf_type == RF_2T2R)
		vht_mcs[0] |= 0xf0;

	memcpy(pvhtpriv->vht_mcs_map, vht_mcs, 2);

	pvhtpriv->vht_highest_rate = rtw_get_vht_highest_rate(rtlpriv, pvhtpriv->vht_mcs_map);
}

void VHT_operation_handler(struct rtl_priv *rtlpriv, PNDIS_802_11_VARIABLE_IEs pIE)
{
	struct mlme_priv  *pmlmepriv = &rtlpriv->mlmepriv;
	struct ht_priv *phtpriv = &pmlmepriv->htpriv;
	struct vht_priv *pvhtpriv = &pmlmepriv->vhtpriv;
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;

	if (pIE == NULL)
		return;

	if (pvhtpriv->vht_option == false)
		return;

	if ((GET_VHT_OPERATION_ELE_CHL_WIDTH(pIE->data) >= 1)
	   && ((0x21 & 0xf0) >= CHANNEL_WIDTH_80)) {
		pvhtpriv->vht_bwmode = CHANNEL_WIDTH_80;
	} else {
		pvhtpriv->vht_bwmode = phtpriv->bwmode;
	}
}

uint32_t rtw_build_vht_operation_ie(struct rtl_priv *rtlpriv, uint8_t *pbuf, uint8_t channel)
{
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv *pvhtpriv = &pmlmepriv->vhtpriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	uint8_t	ChnlWidth, center_freq;
	uint32_t	len = 0;
	uint8_t	operation[5];

	if (pvhtpriv->vht_bwmode >= CHANNEL_WIDTH_80)
		ChnlWidth = 1;
	else
		ChnlWidth = 0;

	center_freq = rtw_get_center_ch(channel, pvhtpriv->vht_bwmode, HAL_PRIME_CHNL_OFFSET_LOWER);

	SET_VHT_OPERATION_ELE_CHL_WIDTH(operation, ChnlWidth);
	/*
	 * center frequency
	 */
	SET_VHT_OPERATION_ELE_CHL_CENTER_FREQ1(operation, center_freq); /* Todo: need to set correct center channel */
	SET_VHT_OPERATION_ELE_CHL_CENTER_FREQ2(operation, 0);
	SET_VHT_OPERATION_ELE_BASIC_MCS_SET(operation, 0xFFFF);

	pbuf = rtw_set_ie(pbuf, EID_VHTOperation, 12, operation, &len);

	return len;
}

uint32_t rtw_build_vht_op_mode_notify_ie(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv *pvhtpriv = &pmlmepriv->vhtpriv;
	uint32_t len = 0;
	uint8_t	opmode = 0,
		rf_type = 0;
	uint8_t	chnl_width, rx_nss;

	chnl_width = pvhtpriv->vht_bwmode;

	rf_type = rtlpriv->phy.rf_type;

	if (rf_type == RF_1T1R)
		rx_nss = 1;
	else
		rx_nss = 2;

	SET_VHT_OPERATING_MODE_FIELD_CHNL_WIDTH(&opmode, chnl_width);
	SET_VHT_OPERATING_MODE_FIELD_RX_NSS(&opmode, (rx_nss-1));
	SET_VHT_OPERATING_MODE_FIELD_RX_NSS_TYPE(&opmode, 0); /* Todo */

	pbuf = rtw_set_ie(pbuf, EID_OpModeNotification, 1, &opmode, &len);

	return len;
}

uint32_t rtw_build_vht_cap_ie(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	uint8_t	bw, rf_type;
	u16 HighestRate;
	uint8_t	*pcap, *pcap_mcs;
	uint32_t len = 0;
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv	*pvhtpriv = &pmlmepriv->vhtpriv;

	pcap = pvhtpriv->vht_cap;
	memset(pcap, 0, 32);

	/*
	 * B2 B3
	 * Supported Channel Width Set
	 */
	SET_VHT_CAPABILITY_ELE_CHL_WIDTH(pcap, 0);  /* indicate we don't support neither 160M nor 80+80M bandwidth. */

	/*
	 * B4
	 * Rx LDPC
	 */
	if (TEST_FLAG(pvhtpriv->ldpc_cap, LDPC_VHT_ENABLE_RX)) {
		SET_VHT_CAPABILITY_ELE_RX_LDPC(pcap, 1);
	}

	/*
	 * B5
	 * ShortGI for 80MHz
	 */
	SET_VHT_CAPABILITY_ELE_SHORT_GI80M(pcap, pvhtpriv->sgi ? 1 : 0); /* We can receive Short GI of 80M */

	/*
	 * B6
	 * ShortGI for 160MHz
	 */

	if (pvhtpriv->vht_bwmode > CHANNEL_WIDTH_80) {
		SET_VHT_CAPABILITY_ELE_SHORT_GI160M(pcap, pvhtpriv->sgi ? 1 : 0);
	}

	/*
	 * B7
	 * Tx STBC
	 */
	if (TEST_FLAG(pvhtpriv->stbc_cap, STBC_VHT_ENABLE_TX)) {
		SET_VHT_CAPABILITY_ELE_TX_STBC(pcap, 1);
	}

	/*
	 * B8 B9 B10
	 * Rx STBC
	 */

	if (TEST_FLAG(pvhtpriv->stbc_cap, STBC_VHT_ENABLE_RX)) {
		rf_type = rtlpriv->phy.rf_type;

		if ((rf_type == RF_2T2R) || (rf_type == RF_1T2R)) {
			SET_VHT_CAPABILITY_ELE_RX_STBC(pcap, 2);
		} else if (rf_type == RF_1T1R) {
			SET_VHT_CAPABILITY_ELE_RX_STBC(pcap, 1);
		}
	}

	/*
	 * B11
	 * SU Beamformer Capable
	 */
	if (TEST_FLAG(pvhtpriv->beamform_cap, BEAMFORMING_VHT_BEAMFORMER_ENABLE)) {
		SET_VHT_CAPABILITY_ELE_SU_BFER(pcap, 1);
	}

	/*
	 * B12
	 * SU Beamformee Capable
	 */
	if (TEST_FLAG(pvhtpriv->beamform_cap, BEAMFORMING_VHT_BEAMFORMEE_ENABLE)) {
		SET_VHT_CAPABILITY_ELE_SU_BFEE(pcap, 1);
	}

	/*
	 * B13 14 15
	 * Compressed Steering Number of Beamformer Antennas Supported
	 */

	SET_VHT_CAPABILITY_ELE_BFER_ANT_SUPP(pcap, 1);	/* TODO */
	/*
	 * B16 17 18
	 * Number of Sounding Dimensions
	 */
	SET_VHT_CAPABILITY_ELE_SOUNDING_DIMENSIONS(pcap, 1);

	/*
	 * B19 MU Beamformer Capable
	 */
	SET_VHT_CAPABILITY_ELE_MU_BFER(pcap, 0);  /* HW don't support mu bfee/bfer */
	/*
	 * B20
	 * MU Beamformee Capable
	 */
	SET_VHT_CAPABILITY_ELE_MU_BFEE(pcap, 0);
	/*
	 * B21
	 * VHT TXOP PS
	 */
	SET_VHT_CAPABILITY_ELE_TXOP_PS(pcap, 0);
	/*
	 * B22
	 * +HTC-VHT Capable
	 */
	SET_VHT_CAPABILITY_ELE_HTC_VHT(pcap, 1);
	/*
	 * B23 24 25
	 * Maximum A-MPDU Length Exponent
	 */
	if (7 != 0xFE) {
		SET_VHT_CAPABILITY_ELE_MAX_RXAMPDU_FACTOR(pcap, 7);
	} else {
		SET_VHT_CAPABILITY_ELE_MAX_RXAMPDU_FACTOR(pcap, 7);
	}
	/*
	 * B26 27
	 * VHT Link Adaptation Capable
	 */
	SET_VHT_CAPABILITY_ELE_LINK_ADAPTION(pcap, 0);

	pcap_mcs = GET_VHT_CAPABILITY_ELE_RX_MCS(pcap);
	memcpy(pcap_mcs, pvhtpriv->vht_mcs_map, 2);

	pcap_mcs = GET_VHT_CAPABILITY_ELE_TX_MCS(pcap);
	memcpy(pcap_mcs, pvhtpriv->vht_mcs_map, 2);

	bw = (0x21 >> 4) & 0xf;
	HighestRate = VHT_MCS_DATA_RATE[bw][pvhtpriv->sgi][((pvhtpriv->vht_highest_rate - MGN_VHT1SS_MCS0)&0x3f)];
	HighestRate = (HighestRate+1) >> 1;

	SET_VHT_CAPABILITY_ELE_MCS_RX_HIGHEST_RATE(pcap, HighestRate); /* indicate we support highest rx rate is 600Mbps. */
	SET_VHT_CAPABILITY_ELE_MCS_TX_HIGHEST_RATE(pcap, HighestRate); /* indicate we support highest tx rate is 600Mbps. */

	pbuf = rtw_set_ie(pbuf, EID_VHTCapability, 12, pcap, &len);

	return len;
}

uint32_t rtw_restructure_vht_ie(struct rtl_priv *rtlpriv, uint8_t *in_ie, uint8_t *out_ie, uint in_len, uint *pout_len)
{
	uint32_t ielen, out_len;
	uint8_t	cap_len, notify_len, operation_bw, supported_chnl_width;
	uint8_t	*p, *pframe;
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv	*pvhtpriv = &pmlmepriv->vhtpriv;

	rtw_vht_use_default_setting(rtlpriv);

	p = rtw_get_ie(in_ie+12, EID_VHTCapability, &ielen, in_len-12);
	if (p && ielen > 0) {
		supported_chnl_width = GET_VHT_CAPABILITY_ELE_CHL_WIDTH(p+2);

		/*
		 *  VHT Capabilities element
		 */
		cap_len = rtw_build_vht_cap_ie(rtlpriv, out_ie + *pout_len);
		*pout_len += cap_len;

		/*
		 * Get HT BW
		 */
		p = rtw_get_ie(in_ie+12, _HT_EXTRA_INFO_IE_, &ielen, in_len-12);
		if (p && ielen > 0) {
			struct HT_info_element *pht_info = (struct HT_info_element *)(p+2);
			if (pht_info->infos[0] & BIT(2))
				operation_bw = CHANNEL_WIDTH_40;
			else
				operation_bw = CHANNEL_WIDTH_20;
		}

		/*
		 *  VHT Operation element
		 */
		p = rtw_get_ie(in_ie+12, EID_VHTOperation, &ielen, in_len-12);
		if (p && ielen > 0) {
			out_len = *pout_len;
			if (GET_VHT_OPERATION_ELE_CHL_WIDTH(p+2) >= 1) {
				if (supported_chnl_width == 2)
					operation_bw = CHANNEL_WIDTH_80_80;
				else if (supported_chnl_width == 1)
					operation_bw = CHANNEL_WIDTH_160;
				else
					operation_bw = CHANNEL_WIDTH_80;
			}
			pframe = rtw_set_ie(out_ie+out_len, EID_VHTOperation, ielen, p+2 , pout_len);
		}

		if (pvhtpriv->vht_bwmode > operation_bw)
			pvhtpriv->vht_bwmode = operation_bw;

		/*
		 * Operating Mode Notification element
		 */
		notify_len = rtw_build_vht_op_mode_notify_ie(rtlpriv, out_ie + *pout_len);
		*pout_len += notify_len;

		pvhtpriv->vht_option = true;
	}

	return pvhtpriv->vht_option;

}

void VHTOnAssocRsp(struct rtl_priv *rtlpriv)
{
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv *pvhtpriv = &pmlmepriv->vhtpriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t	ht_AMPDU_len;

	DBG_871X("%s\n", __FUNCTION__);

	if (!pmlmeinfo->HT_enable)
		return;

	if (!pmlmeinfo->VHT_enable)
		return;

	if (pvhtpriv->vht_bwmode >= CHANNEL_WIDTH_80)
		pmlmeext->cur_bwmode = pvhtpriv->vht_bwmode;

	ht_AMPDU_len = pmlmeinfo->HT_caps.u.HT_cap_element.AMPDU_para & 0x03;

	if (pvhtpriv->ampdu_len > ht_AMPDU_len)
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AMPDU_FACTOR, (uint8_t *)(&pvhtpriv->ampdu_len));

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_AMPDU_MAX_TIME, (uint8_t *)(&pvhtpriv->vht_highest_rate));
}


