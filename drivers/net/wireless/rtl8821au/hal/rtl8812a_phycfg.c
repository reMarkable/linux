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
#define _RTL8812A_PHYCFG_C_

#include <drv_types.h>
#include <rtl8812a_hal.h>
#include <../rtl8821au/phy.h>
#include <../rtl8821au/dm.h>
#include <../rtl8821au/reg.h>
#include <../rtl8821au/rf.h>
#include <../wifi.h>

const char *const GLBwSrc[]={
	"CHANNEL_WIDTH_20",
	"CHANNEL_WIDTH_40",
	"CHANNEL_WIDTH_80",
	"CHANNEL_WIDTH_160",
	"CHANNEL_WIDTH_80_80"
};
#define		ENABLE_POWER_BY_RATE		1
#define		POWERINDEX_ARRAY_SIZE		48 //= cckRatesSize + ofdmRatesSize + htRates1TSize + htRates2TSize + vhtRates1TSize + vhtRates1TSize;

/* ---------------------Define local function prototype----------------------- */

/* ----------------------------Function Body---------------------------------- */

/*
 * 2. RF register R/W API
 */
/*
 * 3. Initial MAC/BB/RF config by reading MAC/BB/RF txt.
 */

/* Ulli called in odm_RegConfig8812A.c and odm_RegConfig8821A.c */

/**************************************************************************************************************
 *   Description:
 *       The low-level interface to get the FINAL Tx Power Index , called  by both MP and Normal Driver.
 *
 *                                                                                    <20120830, Kordan>
 **************************************************************************************************************/

/**************************************************************************************************************
 *   Description:
 *       The low-level interface to set TxAGC , called by both MP and Normal Driver.
 *
 *                                                                                    <20120830, Kordan>
 **************************************************************************************************************/


/*
 * create new definition of PHY_SetTxPowerLevel8812 by YP.
 * Page revised on 20121106
 * the new way to set tx power by rate, NByte access, here N byte shall be 4 byte(DWord) or NByte(N>4) access. by page/YP, 20121106
 */

/* ULLI called in HalPhyRf8812A.c and HalPhyRf21A.c */

/* ULLI used in rtl8821au/dm.c */




/*
 * Prototypes needed here, because functions are moved to rtl8821au/phy.c
 */

static void PHY_HandleSwChnlAndSetBW8812(struct rtl_priv *rtlpriv,
	bool	bSwitchChannel, bool	bSetBandWidth,
	uint8_t	ChannelNum, enum CHANNEL_WIDTH ChnlWidth,
	uint8_t	ChnlOffsetOf40MHz, uint8_t ChnlOffsetOf80MHz,
	uint8_t	CenterFrequencyIndex1
)
{
	struct rtl_mac	*mac = &(rtlpriv->mac80211);
	struct _rtw_hal *	pHalData = GET_HAL_DATA(rtlpriv);
	uint8_t			tmpChannel = rtlpriv->phy.current_channel;
	enum CHANNEL_WIDTH	tmpBW= rtlpriv->phy.current_chan_bw;
	uint8_t			tmpnCur40MhzPrimeSC = mac->cur_40_prime_sc;
	uint8_t			tmpnCur80MhzPrimeSC = mac->cur_80_prime_sc;

	bool bSwChnl = false, bSetChnlBW = false;


	/* DBG_871X("=> PHY_HandleSwChnlAndSetBW8812: bSwitchChannel %d, bSetBandWidth %d \n",bSwitchChannel,bSetBandWidth); */

	/* check is swchnl or setbw */
	if(!bSwitchChannel && !bSetBandWidth) {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "PHY_HandleSwChnlAndSetBW8812:  not switch channel and not set bandwidth \n");
		return;
	}

	/* skip change for channel or bandwidth is the same */
	if(bSwitchChannel) {
		if(rtlpriv->phy.current_channel != ChannelNum) {
			if (HAL_IsLegalChannel(rtlpriv, ChannelNum))
				bSwChnl = true;
			else
				return;
		}
	}

	if(bSetBandWidth) {
		if(pHalData->bChnlBWInitialzed == false) {
			pHalData->bChnlBWInitialzed = true;
			bSetChnlBW = true;
		} else if((rtlpriv->phy.current_chan_bw != ChnlWidth) ||
			(mac->cur_40_prime_sc != ChnlOffsetOf40MHz) ||
			(mac->cur_80_prime_sc != ChnlOffsetOf80MHz) ||
			(rtlpriv->phy.current_channel != CenterFrequencyIndex1)) {

			bSetChnlBW = true;
		}
	}

	if(!bSetChnlBW && !bSwChnl) {
		/* DBG_871X("<= PHY_HandleSwChnlAndSetBW8812: bSwChnl %d, bSetChnlBW %d \n",bSwChnl,bSetChnlBW); */
		return;
	}


	if(bSwChnl) {
		rtlpriv->phy.current_channel = ChannelNum;
	}


	if(bSetChnlBW) {
		rtlpriv->phy.current_chan_bw = ChnlWidth;
		mac->cur_40_prime_sc = ChnlOffsetOf40MHz;
		mac->cur_80_prime_sc = ChnlOffsetOf80MHz;
	}

	/* Switch workitem or set timer to do switch channel or setbandwidth operation */
	if((!rtlpriv->bDriverStopped) && (!rtlpriv->bSurpriseRemoved)) {
		struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

		/* DBG_871X("phy_SwChnlAndSetBwMode8812(): bSwChnl %d, bSetChnlBW %d \n", bSwChnl, bSetChnlBW); */

		if ((rtlpriv->bDriverStopped) || (rtlpriv->bSurpriseRemoved)) {
			return;
		}

		if (bSwChnl) {
			rtl8821au_phy_sw_chnl_callback(rtlpriv);
			bSwChnl = false;
		}

		if (bSetChnlBW) {
			rtlpriv->cfg->ops->phy_set_bw_mode_callback(rtlpriv);
			bSetChnlBW = false;
		}

		rtl8821au_dm_clean_txpower_tracking_state(rtlpriv);
		PHY_SetTxPowerLevel8812(rtlpriv, rtlpriv->phy.current_channel);

		if ((rtlpriv->phy.need_iqk = false == true)) {
			if(IS_HARDWARE_TYPE_8812(rtlhal))
				rtl8812au_phy_iq_calibrate(rtlpriv, false);
			else if(IS_HARDWARE_TYPE_8821(rtlhal))
				rtl8821au_phy_iq_calibrate(rtlpriv, false);

			rtlpriv->phy.need_iqk = false;
		}
	} else {
		if(bSwChnl) {
			rtlpriv->phy.current_channel = tmpChannel;
		}
		if(bSetChnlBW) {
			rtlpriv->phy.current_chan_bw = tmpBW;
			mac->cur_40_prime_sc = tmpnCur40MhzPrimeSC;
			mac->cur_80_prime_sc = tmpnCur80MhzPrimeSC;
		}
	}

	/*
	 * DBG_871X("Channel %d ChannelBW %d ",pHalData->CurrentChannel, pHalData->CurrentChannelBW);
	 * DBG_871X("40MhzPrimeSC %d 80MhzPrimeSC %d ",pHalData->nCur40MhzPrimeSC, pHalData->nCur80MhzPrimeSC);
	 * DBG_871X("CenterFrequencyIndex1 %d \n",pHalData->CurrentCenterFrequencyIndex1);
	 */

	/*
	 * DBG_871X("<= PHY_HandleSwChnlAndSetBW8812: bSwChnl %d, bSetChnlBW %d \n",bSwChnl,bSetChnlBW);
	 */

}

void PHY_SetBWMode8812(struct rtl_priv *rtlpriv,
	enum CHANNEL_WIDTH	Bandwidth,	/* 20M or 40M */
	uint8_t	Offset)		/* Upper, Lower, or Don't care */
{
	/* DBG_871X("%s()===>\n",__FUNCTION__); */

	PHY_HandleSwChnlAndSetBW8812(rtlpriv, false, true, rtlpriv->phy.current_channel, Bandwidth, Offset, Offset, rtlpriv->phy.current_channel);

	//DBG_871X("<==%s()\n",__FUNCTION__);
}

void PHY_SwChnl8812(struct rtl_priv *rtlpriv, uint8_t channel)
{
	/* DBG_871X("%s()===>\n",__FUNCTION__); */

	PHY_HandleSwChnlAndSetBW8812(rtlpriv, true, false, channel, 0, 0, 0, channel);

	/* DBG_871X("<==%s()\n",__FUNCTION__); */
}

void PHY_SetSwChnlBWMode8812(struct rtl_priv *rtlpriv, uint8_t channel,
	enum CHANNEL_WIDTH Bandwidth, uint8_t Offset40, uint8_t Offset80)
{
	/* DBG_871X("%s()===>\n",__FUNCTION__); */

	PHY_HandleSwChnlAndSetBW8812(rtlpriv, true, true, channel, Bandwidth, Offset40, Offset80, channel);

	/* DBG_871X("<==%s()\n",__FUNCTION__); */
}


