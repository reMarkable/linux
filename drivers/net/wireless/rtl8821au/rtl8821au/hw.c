#ifdef CONFIG_RTLWIFI

#include <../drivers/net/wireless/realtek/rtlwifi/wifi.h>
#include <../drivers/net/wireless/realtek/rtlwifi/core.h>
#include <../drivers/net/wireless/realtek/rtlwifi/use.h>
#include <../drivers/net/wireless/realtek/rtlwifi/efuse.h>
#include <../drivers/net/wireless/realtek/rtlwifi/base.h>

#else

#include <drv_types.h>
#include "hw.h"
#include "reg.h"
#include "fw.h"
#include "rf.h"
#include "phy.h"
#include "def.h"
#include "dm.h"
#include "../cam.h"
#include <usb_ops.h>

#endif

void rtl8821au_init_beacon_parameters(struct rtl_priv *rtlpriv)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);

	rtl_write_word(rtlpriv, REG_BCN_CTRL, 0x1010);

	/* TODO: Remove these magic number */
	rtl_write_word(rtlpriv, REG_TBTT_PROHIBIT, 0x6404);		/* ms */
	rtl_write_byte(rtlpriv, REG_DRVERLYINT, DRIVER_EARLY_INT_TIME_8812);	/* 5ms */
	rtl_write_byte(rtlpriv, REG_BCNDMATIM, BCN_DMA_ATIME_INT_TIME_8812); 	/* 2ms */

	/*
	 *  Suggested by designer timchen. Change beacon AIFS to the largest number
	 *  beacause test chip does not contension before sending beacon. by tynli. 2009.11.03
	 */
	rtl_write_word(rtlpriv, REG_BCNTCFG, 0x660F);

	rtlusb->reg_bcn_ctrl_val = rtl_read_byte(rtlpriv, REG_BCN_CTRL);
	pHalData->RegTxPause = rtl_read_byte(rtlpriv, REG_TXPAUSE);
	pHalData->RegFwHwTxQCtrl = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL+2);
	pHalData->RegReg542 = rtl_read_byte(rtlpriv, REG_TBTT_PROHIBIT+2);
	pHalData->RegCR_1 = rtl_read_byte(rtlpriv, REG_CR+1);
}

static void _rtl8821au_stop_tx_beacon(struct rtl_priv *rtlpriv)
{
	 struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);

	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL+2, (pHalData->RegFwHwTxQCtrl) & (~BIT(6)));
	pHalData->RegFwHwTxQCtrl &= (~BIT(6));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+1, 0x64);
	pHalData->RegReg542 &= ~(BIT(0));
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+2, pHalData->RegReg542);

	 /* todo: CheckFwRsvdPageContent(rtlpriv);  // 2010.06.23. Added by tynli. */
}

static void  _rtl8821au_resume_tx_beacon(struct rtl_priv *rtlpriv)
{
	 struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);

	/*
	 * 2010.03.01. Marked by tynli. No need to call workitem beacause we record the value
	 * which should be read from register to a global variable.
	 */

	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL+2, (pHalData->RegFwHwTxQCtrl) | BIT(6));
	pHalData->RegFwHwTxQCtrl |= BIT(6);
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+1, 0xff);
	pHalData->RegReg542 |= BIT(0);
	rtl_write_byte(rtlpriv, REG_TBTT_PROHIBIT+2, pHalData->RegReg542);
}

static void _BeaconFunctionEnable(struct rtl_priv *rtlpriv, bool Enable,
	bool	Linked)
{
	rtl_write_byte(rtlpriv, REG_BCN_CTRL, (BIT(4) | BIT(3) | BIT(1)));
	/*
	 * SetBcnCtrlReg(rtlpriv, (BIT(4) | BIT(3) | BIT(1)), 0x00);
	 * RT_TRACE(COMP_BEACON, DBG_LOUD, ("_BeaconFunctionEnable 0x550 0x%x\n", rtl_read_byte(rtlpriv, 0x550)));
	 */

	rtl_write_byte(rtlpriv, REG_RD_CTRL+1, 0x6F);
}

void rtl8821au_set_beacon_related_registers(struct rtl_priv *rtlpriv)
{
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	uint32_t	value32;
	/* reset TSF, enable update TSF, correcting TSF On Beacon */

	/*
	 * REG_BCN_INTERVAL
	 * REG_BCNDMATIM
	 * REG_ATIMWND
	 * REG_TBTT_PROHIBIT
	 * REG_DRVERLYINT
	 * REG_BCN_MAX_ERR
	 * REG_BCNTCFG //(0x510)
	 * REG_DUAL_TSF_RST
	 * REG_BCN_CTRL //(0x550)
	 */

	/* BCN interval */
	rtl_write_word(rtlpriv, REG_BCN_INTERVAL, mac->beacon_interval);
	rtl_write_byte(rtlpriv, REG_ATIMWND, 0x02);	/* 2ms */

	rtl8821au_init_beacon_parameters(rtlpriv);

	rtl_write_byte(rtlpriv, REG_SLOT, 0x09);

	value32 = rtl_read_dword(rtlpriv, REG_TCR);
	value32 &= ~TSFRST;
	rtl_write_dword(rtlpriv,  REG_TCR, value32);

	value32 |= TSFRST;
	rtl_write_dword(rtlpriv, REG_TCR, value32);

	/* NOTE: Fix test chip's bug (about contention windows's randomness) */
	rtl_write_byte(rtlpriv,  REG_RXTSF_OFFSET_CCK, 0x50);
	rtl_write_byte(rtlpriv, REG_RXTSF_OFFSET_OFDM, 0x50);

	_BeaconFunctionEnable(rtlpriv, true, true);

	_rtl8821au_resume_tx_beacon(rtlpriv);

	/* rtl_write_byte(rtlpriv, 0x422, rtl_read_byte(rtlpriv, 0x422)|BIT(6)); */

	/* rtl_write_byte(rtlpriv, 0x541, 0xff); */

	/* rtl_write_byte(rtlpriv, 0x542, rtl_read_byte(rtlpriv, 0x541)|BIT(0)); */

	rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)|BIT(1));

}

int rtl8821au_set_network_type(struct rtl_priv *rtlpriv, uint8_t mode)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	uint8_t	val8;

	{
		/* disable Port0 TSF update */
		rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)|DIS_TSF_UDT);

		/*  set net_type */
		val8 = rtl_read_byte(rtlpriv, MSR)&0x0c;
		val8 |= mode;
		rtl_write_byte(rtlpriv, MSR, val8);

		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s()-%d mode = %d\n", __FUNCTION__, __LINE__, mode);

		if ((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_)) {
			_rtl8821au_stop_tx_beacon(rtlpriv);

			rtl_write_byte(rtlpriv, REG_BCN_CTRL, 0x19);		/* disable atim wnd */
			/* rtl_write_byte(rtlpriv,REG_BCN_CTRL, 0x18); */
		} else if ((mode == _HW_STATE_ADHOC_) /*|| (mode == _HW_STATE_AP_)*/ ) {
			_rtl8821au_resume_tx_beacon(rtlpriv);
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, 0x1a);
		} else if (mode == _HW_STATE_AP_) {
			_rtl8821au_resume_tx_beacon(rtlpriv);

			rtl_write_byte(rtlpriv, REG_BCN_CTRL, 0x12);

			/* Set RCR */
			rtl_write_dword(rtlpriv, REG_RCR, 0x7000208e);	/* CBSSID_DATA must set to 0,reject ICV_ERR packet */
			/* enable to rx data frame */
			rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xFFFF);
			/* enable to rx ps-poll */
			rtl_write_word(rtlpriv, REG_RXFLTMAP1, 0x0400);

			/* Beacon Control related register for first time */
			rtl_write_byte(rtlpriv, REG_BCNDMATIM, 0x02); /* 2ms */

			/* rtl_write_byte(rtlpriv, REG_BCN_MAX_ERR, 0xFF); */
			rtl_write_byte(rtlpriv, REG_ATIMWND, 0x0a); 	/* 10ms */
			rtl_write_word(rtlpriv, REG_BCNTCFG, 0x00);
			rtl_write_word(rtlpriv, REG_TBTT_PROHIBIT, 0xff04);
			rtl_write_word(rtlpriv, REG_TSFTR_SYN_OFFSET, 0x7fff);	/* +32767 (~32ms) */

			/* reset TSF */
			rtl_write_byte(rtlpriv, REG_DUAL_TSF_RST, BIT(0));

			/*
			 * enable BCN0 Function for if1
			 * don't enable update TSF0 for if1 (due to TSF update when beacon/probe rsp are received)
			 */
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, (DIS_TSF_UDT|EN_BCN_FUNCTION | EN_TXBCN_RPT|DIS_BCNQ_SUB));

			if (IS_HARDWARE_TYPE_8821(rtlhal)) {
				/*  select BCN on port 0 */
				rtl_write_byte(rtlpriv, REG_CCK_CHECK_8812,	rtl_read_byte(rtlpriv, REG_CCK_CHECK_8812)&(~BIT(5)));
			}


			/* dis BCN1 ATIM  WND if if2 is station */
			rtl_write_byte(rtlpriv, REG_BCN_CTRL_1, rtl_read_byte(rtlpriv, REG_BCN_CTRL_1)|DIS_ATIM);
		}
	}
	return 0;
}

static void hw_var_set_bcn_func(struct rtl_priv *rtlpriv, uint8_t variable, uint8_t *val)
{
	if (*((uint8_t *) val))
		rtl_write_byte(rtlpriv, REG_BCN_CTRL, (EN_BCN_FUNCTION | EN_TXBCN_RPT));
	else {
		u8 tmp;

		tmp = rtl_read_byte(rtlpriv, REG_BCN_CTRL);
		tmp &= (~(EN_BCN_FUNCTION | EN_TXBCN_RPT));
		rtl_write_byte(rtlpriv, REG_BCN_CTRL, tmp);
	}
}

static void hw_var_set_mlme_sitesurvey(struct rtl_priv *rtlpriv, uint8_t variable, uint8_t *val)
{
	uint32_t value_rcr, rcr_clear_bit, reg_bcn_ctl;
	u16 value_rxfltmap2;
	 struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);

		reg_bcn_ctl = REG_BCN_CTRL;

	rcr_clear_bit = RCR_CBSSID_BCN;

	/* config RCR to receive different BSSID & not to receive data frame */
	value_rxfltmap2 = 0;

	if ((check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)) {
		rcr_clear_bit = RCR_CBSSID_BCN;
	}

	value_rcr = rtl_read_dword(rtlpriv, REG_RCR);

	if (*((uint8_t *) val)) {
		/* under sitesurvey */

		value_rcr &= ~(rcr_clear_bit);
		rtl_write_dword(rtlpriv, REG_RCR, value_rcr);

		rtl_write_word(rtlpriv, REG_RXFLTMAP2, value_rxfltmap2);

		if (check_fwstate(pmlmepriv, WIFI_STATION_STATE | WIFI_ADHOC_STATE | WIFI_ADHOC_MASTER_STATE)) {
			/* disable update TSF */
			rtl_write_byte(rtlpriv, reg_bcn_ctl, rtl_read_byte(rtlpriv, reg_bcn_ctl)|DIS_TSF_UDT);
		}

		/* Save orignal RRSR setting. */
		pHalData->RegRRSR = rtl_read_word(rtlpriv, REG_RRSR);

	} else {
		/* sitesurvey done */

		if (check_fwstate(pmlmepriv, (_FW_LINKED | WIFI_AP_STATE))) {
			/* enable to rx data frame */
			rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xFFFF);
		}

		if (check_fwstate(pmlmepriv, WIFI_STATION_STATE | WIFI_ADHOC_STATE | WIFI_ADHOC_MASTER_STATE)) {
			/* enable update TSF */
			rtl_write_byte(rtlpriv, reg_bcn_ctl, rtl_read_byte(rtlpriv, reg_bcn_ctl)&(~(DIS_TSF_UDT)));
		}

		value_rcr |= rcr_clear_bit;
		rtl_write_dword(rtlpriv, REG_RCR, value_rcr);

		/* Restore orignal RRSR setting. */
		rtl_write_word(rtlpriv, REG_RRSR, pHalData->RegRRSR);

	}
}

static void Hal_PatchwithJaguar_8812(struct rtl_priv *rtlpriv, RT_MEDIA_STATUS	MediaStatus)
{
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	if ((MediaStatus == RT_MEDIA_CONNECT)
	  && (pmlmeinfo->assoc_AP_vendor == HT_IOT_PEER_REALTEK_JAGUAR_BCUTAP)) {
		rtl_write_byte(rtlpriv, rVhtlen_Use_Lsig_Jaguar, 0x1);
		rtl_write_byte(rtlpriv, REG_TCR+3, BIT(2));
	} else {
		rtl_write_byte(rtlpriv, rVhtlen_Use_Lsig_Jaguar, 0x3F);
		rtl_write_byte(rtlpriv, REG_TCR+3, BIT(0)|BIT(1)|BIT(2));
	}


	if ((MediaStatus == RT_MEDIA_CONNECT)
	   && ((pmlmeinfo->assoc_AP_vendor == HT_IOT_PEER_REALTEK_JAGUAR_BCUTAP)
	      || (pmlmeinfo->assoc_AP_vendor == HT_IOT_PEER_REALTEK_JAGUAR_CCUTAP))) {
		rtlpriv->phy.reg_837 |= BIT(2);
		rtl_write_byte(rtlpriv, rBWIndication_Jaguar+3, rtlpriv->phy.reg_837);
	} else {
		rtlpriv->phy.reg_837 &= (~BIT(2));
		rtl_write_byte(rtlpriv, rBWIndication_Jaguar+3, rtlpriv->phy.reg_837);
	}
}

void rtl8821au_set_hw_reg(struct rtl_priv *rtlpriv, u8 variable, u8 *pval)
{
	struct rtl_phy *rtlphy = &(rtlpriv->phy);
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct rtl_efuse *rtlefuse =  rtl_efuse(rtlpriv);
	struct _rtw_hal *pHalData;
	struct dm_priv *pdmpriv;
	struct _rtw_dm *podmpriv;
	uint8_t val8;
	u16 val16;
	uint32_t val32;
	u8 idx;

	pHalData = GET_HAL_DATA(rtlpriv);
	pdmpriv = &pHalData->dmpriv;
	podmpriv = &pHalData->odmpriv;

	switch (variable) {
	case HW_VAR_ETHER_ADDR:
		for (idx = 0 ; idx < 6; idx++) {
			rtl_write_byte(rtlpriv, (REG_MACID + idx), pval[idx]);
		}
		break;

	/* ULLI : old Hw vars */



	case HW_VAR_RXDMA_AGG_PG_TH:
		{
			/*uint8_t	threshold = *((uint8_t *)val);
			if ( threshold == 0)
			{
				threshold = pHalData->UsbRxAggPageCount;
			}
			rtl_write_byte(rtlpriv, REG_RXDMA_AGG_PG_TH, threshold);*/
		}
		break;
	case HW_VAR_MEDIA_STATUS:
		val8 = rtl_read_byte(rtlpriv, MSR) & 0x0c;
		val8 |= *pval;
		rtl_write_byte(rtlpriv, MSR, val8);
		break;

	case HW_VAR_MEDIA_STATUS1:
		val8 = rtl_read_byte(rtlpriv, MSR) & 0x03;
		val8 |= *pval << 2;
		rtl_write_byte(rtlpriv, MSR, val8);
		break;

	case HW_VAR_BSSID:
		for (idx = 0 ; idx < 6; idx++) {
			rtl_write_byte(rtlpriv, (REG_BSSID + idx), pval[idx]);
		}
		break;

	case HW_VAR_BASIC_RATE:
		{
			u16 BrateCfg = 0;
			uint8_t RateIndex = 0;

			/*
			 * 2007.01.16, by Emily
			 * Select RRSR (in Legacy-OFDM and CCK)
			 * For 8190, we select only 24M, 12M, 6M, 11M, 5.5M, 2M, and 1M from the Basic rate.
			 * We do not use other rates.
			 */
			HalSetBrateCfg(rtlpriv, pval, &BrateCfg);

			if (rtlhal->current_bandtype == BAND_ON_2_4G) {
				/*
				 * CCK 2M ACK should be disabled for some BCM and Atheros AP IOT
				 * because CCK 2M has poor TXEVM
				 * CCK 5.5M & 11M ACK should be enabled for better performance
				 */
				BrateCfg = (BrateCfg | 0xd) & 0x15d;
				BrateCfg |= 0x01; /* default enable 1M ACK rate */
			} else { /* 5G */
				BrateCfg |= 0x10; /* default enable 6M ACK rate */
			}
			/*
			 * DBG_8192C("HW_VAR_BASIC_RATE: BrateCfg(%#x)\n", BrateCfg);
			 */

			/* Set RRSR rate table. */
			rtl_write_byte(rtlpriv, REG_RRSR, BrateCfg&0xff);
			rtl_write_byte(rtlpriv, REG_RRSR+1, (BrateCfg>>8)&0xff);
			rtl_write_byte(rtlpriv, REG_RRSR+2, rtl_read_byte(rtlpriv, REG_RRSR+2)&0xf0);
		}
		break;

	case HW_VAR_TXPAUSE:
		rtl_write_byte(rtlpriv, REG_TXPAUSE, *pval);
		break;

	case HW_VAR_BCN_FUNC:
		hw_var_set_bcn_func(rtlpriv, variable, pval);
		break;

	case HW_VAR_CORRECT_TSF:
		{
			u64	tsf;
			struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
			struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

			/*
			 * tsf = pmlmeext->TSFValue - ((u32)pmlmeext->TSFValue % (pmlmeinfo->bcn_interval*1024)) -1024; //us
			 */
			tsf = pmlmeext->TSFValue - rtw_modular64(pmlmeext->TSFValue, (mac->beacon_interval*1024)) - 1024; /* us */

			if (((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)
			   || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)) {
				/*
				 * pHalData->RegTxPause |= STOP_BCNQ;BIT(6)
				 * rtl_write_byte(rtlpriv, REG_TXPAUSE, (rtl_read_byte(rtlpriv, REG_TXPAUSE)|BIT(6)));
				 */
				_rtl8821au_stop_tx_beacon(rtlpriv);
			}

			/* disable related TSF function */
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)&(~BIT(3)));

			rtl_write_dword(rtlpriv, REG_TSFTR, tsf);
			rtl_write_dword(rtlpriv, REG_TSFTR+4, tsf>>32);

			/* enable related TSF function */
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, rtl_read_byte(rtlpriv, REG_BCN_CTRL)|BIT(3));


			if (((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)
			   || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)) {
				/*
				 * pHalData->RegTxPause  &= (~STOP_BCNQ);
				 * rtl_write_byte(rtlpriv, REG_TXPAUSE, (rtl_read_byte(rtlpriv, REG_TXPAUSE)&(~BIT(6))));
				 */
				_rtl8821au_resume_tx_beacon(rtlpriv);
			}
		}
		break;

	case HW_VAR_CHECK_BSSID:
		val32 = rtl_read_dword(rtlpriv, REG_RCR);
		if (*pval)
			val32 |= RCR_CBSSID_DATA|RCR_CBSSID_BCN;
		else
			val32 &= ~(RCR_CBSSID_DATA|RCR_CBSSID_BCN);
		rtl_write_dword(rtlpriv, REG_RCR, val32);
		break;

	case HW_VAR_MLME_DISCONNECT:
		{
			/* Set RCR to not to receive data frame when NO LINK state
			 * val32 = rtl_read_dword(rtlpriv, REG_RCR);
			 * val32 &= ~RCR_ADF;
			 * rtl_write_dword(rtlpriv, REG_RCR, val32);
			 */

			 /* reject all data frames */
			rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0x00);

			/* reset TSF */
			val8 = BIT(0) | BIT(1);
			rtl_write_byte(rtlpriv, REG_DUAL_TSF_RST, val8);

			/* disable update TSF */
			val8 = rtl_read_byte(rtlpriv, REG_BCN_CTRL);
			val8 |= BIT(4);
			rtl_write_byte(rtlpriv, REG_BCN_CTRL, val8);
		}
		break;

	case HW_VAR_MLME_SITESURVEY:
		hw_var_set_mlme_sitesurvey(rtlpriv, variable,  pval);

		break;

	case HW_VAR_MLME_JOIN:
		{
			uint8_t RetryLimit = 0x30;
			uint8_t type = *(uint8_t *)pval;

			struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;

			if (type == 0) { 	/* prepare to join  */
				/*
				 * enable to rx data frame.Accept all data frame
				 * rtl_write_dword(rtlpriv, REG_RCR, rtl_read_dword(rtlpriv, REG_RCR)|RCR_ADF);
				 */
				rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0xFFFF);

				val32 = rtl_read_dword(rtlpriv, REG_RCR);
				if (rtlpriv->in_cta_test)
					val32 &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);/* | RCR_ADF */
				else
					val32 |= RCR_CBSSID_DATA|RCR_CBSSID_BCN;
				rtl_write_dword(rtlpriv, REG_RCR, val32);

				if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true) {
					/* ULLI removed
					 * RetryLimit = (pEEPROM->CustomerID == RT_CID_CCX) ? 7 : 48;
					 */

					RetryLimit = 48;
				} else { /* Ad-hoc Mode */
					RetryLimit = 0x7;
				}

				rtlphy->need_iqk = true;
			} else if (type == 1) { /* joinbss_event call back when join res < 0  */

				rtl_write_word(rtlpriv, REG_RXFLTMAP2, 0x00);
			} else if (type == 2) { /* sta add event call back */
				/* enable update TSF */
				val8 = rtl_read_byte(rtlpriv, REG_BCN_CTRL);
				val8 &= ~BIT(4);
				rtl_write_byte(rtlpriv, REG_BCN_CTRL, val8);

				if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE)) {
					RetryLimit = 0x7;
				}
			}

			val16 = RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT;
			rtl_write_word(rtlpriv, REG_RL, val16);
		}

		break;

	case HW_VAR_ON_RCR_AM:
		val32 = rtl_read_dword(rtlpriv, REG_RCR);
		val32 |= RCR_AM;
		rtl_write_dword(rtlpriv, REG_RCR, val32);
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s, %d, RCR= %x\n", __FUNCTION__, __LINE__, rtl_read_dword(rtlpriv, REG_RCR));
		break;

	case HW_VAR_OFF_RCR_AM:
		val32 = rtl_read_dword(rtlpriv, REG_RCR);
		val32 &= ~RCR_AM;
		rtl_write_dword(rtlpriv, REG_RCR, val32);
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,  "%s, %d, RCR= %x\n", __FUNCTION__, __LINE__, rtl_read_dword(rtlpriv, REG_RCR));
		break;

	case HW_VAR_BEACON_INTERVAL:
		rtl_write_word(rtlpriv, REG_BCN_INTERVAL, *(u16 *)pval);
		break;

	case HW_VAR_SLOT_TIME:
		rtl_write_byte(rtlpriv, REG_SLOT, *pval);
		break;

	case HW_VAR_RESP_SIFS:
		/*
		 * SIFS_Timer = 0x0a0a0808;
		 * RESP_SIFS for CCK
		 */
		rtl_write_byte(rtlpriv, REG_RESP_SIFS_CCK, pval[0]); 	/* SIFS_T2T_CCK (0x08) */
		rtl_write_byte(rtlpriv, REG_RESP_SIFS_CCK+1, pval[1]); 	/* SIFS_R2T_CCK(0x08) */
		/*  RESP_SIFS for OFDM */
		rtl_write_byte(rtlpriv, REG_RESP_SIFS_OFDM, pval[2]); 	/* SIFS_T2T_OFDM (0x0a) */
		rtl_write_byte(rtlpriv, REG_RESP_SIFS_OFDM+1, pval[3]); 	/* SIFS_R2T_OFDM(0x0a) */
		break;

	case HW_VAR_ACK_PREAMBLE:
		{
			uint8_t bShortPreamble = *pval;

			/*  Joseph marked out for Netgear 3500 TKIP channel 7 issue.(Temporarily) */
			val8 = (rtlpriv->mac80211.cur_40_prime_sc) << 5;
			if (bShortPreamble)
				val8 |= 0x80;
			rtl_write_byte(rtlpriv, REG_RRSR+2, val8);
		}
		break;

	case HW_VAR_SEC_CFG:
		val8 = *pval;
		rtl_write_byte(rtlpriv, REG_SECCFG, val8);
		break;

	case HW_VAR_CAM_WRITE:
		{
			uint32_t cmd;
			uint32_t *cam_val = (u32 *)pval;

			rtl_write_dword(rtlpriv, WCAMI, cam_val[0]);

			cmd = CAM_POLLINIG | CAM_WRITE | cam_val[1];
			rtl_write_dword(rtlpriv, RWCAM, cmd);
		}
		break;

	case HW_VAR_CAM_READ:
		break;

	case HW_VAR_AC_PARAM_VO:
		rtl_write_dword(rtlpriv, REG_EDCA_VO_PARAM, *(u32 *)pval);
		break;

	case HW_VAR_AC_PARAM_VI:
		rtl_write_dword(rtlpriv, REG_EDCA_VI_PARAM, *(u32 *)pval);
		break;

	case HW_VAR_AC_PARAM_BE:
		pHalData->AcParam_BE = *(u32 *)pval;
		rtl_write_dword(rtlpriv, REG_EDCA_BE_PARAM, *(u32 *)pval);
		break;

	case HW_VAR_AC_PARAM_BK:
		rtl_write_dword(rtlpriv, REG_EDCA_BK_PARAM, *(u32 *)pval);
		break;

	case HW_VAR_ACM_CTRL:
		{
			uint8_t acm_ctrl;
			uint8_t AcmCtrl;

			acm_ctrl = *(uint8_t *)pval;
			AcmCtrl = rtl_read_byte(rtlpriv, REG_ACMHWCTRL);

			if (acm_ctrl > 1)
				AcmCtrl = AcmCtrl | 0x1;

			if (acm_ctrl & BIT(3))
				AcmCtrl |= AcmHw_VoqEn;
			else
				AcmCtrl &= (~AcmHw_VoqEn);

			if (acm_ctrl & BIT(2))
				AcmCtrl |= AcmHw_ViqEn;
			else
				AcmCtrl &= (~AcmHw_ViqEn);

			if (acm_ctrl & BIT(1))
				AcmCtrl |= AcmHw_BeqEn;
			else
				AcmCtrl &= (~AcmHw_BeqEn);

			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "[HW_VAR_ACM_CTRL] Write 0x%X\n", AcmCtrl);
			rtl_write_byte(rtlpriv, REG_ACMHWCTRL, AcmCtrl);
		}
		break;

	case HW_VAR_AMPDU_MIN_SPACE:
		pHalData->AMPDUDensity = *(uint8_t *)pval;
		break;

	case HW_VAR_AMPDU_FACTOR:
		{
			uint32_t	AMPDULen = *(uint8_t *)pval;

			if (IS_HARDWARE_TYPE_8812(rtlhal)) {
				if (AMPDULen < VHT_AGG_SIZE_128K)
					AMPDULen = (0x2000 << *(uint8_t *)pval) - 1;
				else
					AMPDULen = 0x1ffff;
			} else if (IS_HARDWARE_TYPE_8821(rtlhal)) {
				if (AMPDULen < HT_AGG_SIZE_64K)
					AMPDULen = (0x2000 << *(uint8_t *)pval) - 1;
				else
					AMPDULen = 0xffff;
			}
			AMPDULen |= BIT(31);
			rtl_write_dword(rtlpriv, REG_AMPDU_MAX_LENGTH_8812, AMPDULen);
		}
		break;
	case HW_VAR_H2C_FW_PWRMODE:
		{
			uint8_t psmode = *pval;

			/*
			 * Forece leave RF low power mode for 1T1R to prevent conficting setting in Fw power
			 * saving sequence. 2010.06.07. Added by tynli. Suggested by SD3 yschang.
			 */
			rtl8812au_set_fw_pwrmode_cmd(rtlpriv, psmode);
		}
		break;

	case HW_VAR_H2C_FW_JOINBSSRPT:
		rtl8821au_set_fw_joinbss_report_cmd(rtlpriv, *pval);
		break;

	case HW_VAR_INITIAL_GAIN:	/* ULLI not in rtlwifi */
		{
			struct dig_t *dm_digtable = &(rtlpriv->dm_digtable);
			uint32_t rx_gain = *(u32 *)pval;

			if (rx_gain == 0xff) {		/* restore rx gain */
				rtl8821au_dm_write_dig(rtlpriv, dm_digtable->BackupIGValue);
			} else {
				dm_digtable->BackupIGValue = dm_digtable->cur_igvalue;
				rtl8821au_dm_write_dig(rtlpriv, rx_gain);
			}
		}
		break;


#if (RATE_ADAPTIVE_SUPPORT == 1)
	case HW_VAR_RPT_TIMER_SETTING:
		{
			val16 = *(u16 *)pval;
			ODM_RA_Set_TxRPT_Time(podmpriv, val16);
		}
		break;
#endif

	case HW_VAR_EFUSE_USAGE:
		rtlefuse->efuse_usedpercentage = *pval;
		break;

	case HW_VAR_EFUSE_BYTES:
		rtlefuse->efuse_usedbytes = *(u16 *)pval;
		break;
	case HW_VAR_FIFO_CLEARN_UP:
		{
			struct pwrctrl_priv *pwrpriv;
			uint8_t trycnt = 100;

			pwrpriv = &rtlpriv->pwrctrlpriv;

			/* pause tx */
			rtl_write_byte(rtlpriv, REG_TXPAUSE, 0xff);

			/* keep sn */
			rtlpriv->xmitpriv.nqos_ssn = rtl_read_word(rtlpriv, REG_NQOS_SEQ);

			{
				/* RX DMA stop */
				val32 = rtl_read_dword(rtlpriv, REG_RXPKT_NUM);
				val32 |= RW_RELEASE_EN;
				rtl_write_dword(rtlpriv, REG_RXPKT_NUM, val32);
				do {
					val32 = rtl_read_dword(rtlpriv, REG_RXPKT_NUM);
					val32 &= RXDMA_IDLE;
					if (!val32)
						break;
				} while (trycnt--);
				if (trycnt == 0) {
					RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "[HW_VAR_FIFO_CLEARN_UP] Stop RX DMA failed......\n");
				}

				/* RQPN Load 0 */
				rtl_write_word(rtlpriv, REG_RQPN_NPQ, 0x0);
				rtl_write_dword(rtlpriv, REG_RQPN, 0x80000000);
				mdelay(10);
			}
		}
		break;


#if (RATE_ADAPTIVE_SUPPORT == 1)
	case HW_VAR_TX_RPT_MAX_MACID:
		{
			uint8_t maxMacid = *pval;
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "### MacID(%d),Set Max Tx RPT MID(%d)\n", maxMacid, maxMacid+1);
			rtl_write_byte(rtlpriv, REG_TX_RPT_CTRL+1, maxMacid+1);
		}
		break;
#endif

	case HW_VAR_H2C_MEDIA_STATUS_RPT:
		{
			struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
			RT_MEDIA_STATUS	mstatus = *(u16 *)pval & 0xFF;

			rtl8812_set_FwMediaStatus_cmd(rtlpriv, *(u16 *)pval);

			if (check_fwstate(pmlmepriv, WIFI_STATION_STATE))
				Hal_PatchwithJaguar_8812(rtlpriv, mstatus);
		}
		break;

	case HW_VAR_NAV_UPPER:
		{
			uint32_t usNavUpper = *((u32 *)pval);

			if (usNavUpper > HAL_NAV_UPPER_UNIT * 0xFF) {
				RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s: [HW_VAR_NAV_UPPER] set value(0x%08X us) is larger than (%d * 0xFF)!\n",
					__FUNCTION__, usNavUpper, HAL_NAV_UPPER_UNIT);
				break;
			}

			/*
			 *  The value of ((usNavUpper + HAL_NAV_UPPER_UNIT - 1) / HAL_NAV_UPPER_UNIT)
			 * is getting the upper integer.
			 */
			usNavUpper = (usNavUpper + HAL_NAV_UPPER_UNIT - 1) / HAL_NAV_UPPER_UNIT;
			rtl_write_byte(rtlpriv, REG_NAV_UPPER, (uint8_t)usNavUpper);
		}
		break;

	case HW_VAR_BCN_VALID:
		{
			/*
			 * BCN_VALID, BIT(16) of REG_TDECTRL = BIT(0) of REG_TDECTRL+2, write 1 to clear, Clear by sw
			 */
			val8 = rtl_read_byte(rtlpriv, REG_TDECTRL+2);
			val8 |= BIT(0);
			rtl_write_byte(rtlpriv, REG_TDECTRL+2, val8);
		}
		break;

	case HW_VAR_DL_BCN_SEL:
		{
			/* SW_BCN_SEL - Port0 */
			val8 = rtl_read_byte(rtlpriv, REG_TDECTRL1_8812+2);
			val8 &= ~BIT(4);
			rtl_write_byte(rtlpriv, REG_TDECTRL1_8812+2, val8);
		}
		break;

	case HW_VAR_WIRELESS_MODE:
		{
			uint8_t	R2T_SIFS = 0, SIFS_Timer = 0;
			uint8_t	wireless_mode = *pval;

			if ((wireless_mode == WIRELESS_11BG) || (wireless_mode == WIRELESS_11G))
				SIFS_Timer = 0xa;
			else
				SIFS_Timer = 0xe;

			/* SIFS for OFDM Data ACK */
			rtl_write_byte(rtlpriv, REG_SIFS_CTX+1, SIFS_Timer);
			/* SIFS for OFDM consecutive tx like CTS data! */
			rtl_write_byte(rtlpriv, REG_SIFS_TRX+1, SIFS_Timer);

			rtl_write_byte(rtlpriv, REG_SPEC_SIFS+1, SIFS_Timer);
			rtl_write_byte(rtlpriv, REG_MAC_SPEC_SIFS+1, SIFS_Timer);

			/* 20100719 Joseph: Revise SIFS setting due to Hardware register definition change. */
			rtl_write_byte(rtlpriv, REG_RESP_SIFS_OFDM+1, SIFS_Timer);
			rtl_write_byte(rtlpriv, REG_RESP_SIFS_OFDM, SIFS_Timer);

			/*
			 * Adjust R2T SIFS for IOT issue. Add by hpfan 2013.01.25
			 * Set R2T SIFS to 0x0a for Atheros IOT. Add by hpfan 2013.02.22
			 *
			 * Mac has 10 us delay so use 0xa value is enough.
			 */
			R2T_SIFS = 0xa;

			rtl_write_byte(rtlpriv, REG_RESP_SIFS_OFDM+1, R2T_SIFS);
		}
		break;

	default:
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s: [WARNNING] variable(%d) not defined!\n", __FUNCTION__, variable);
		break;
	}
}


void rtl8821au_get_hw_reg(struct rtl_priv *rtlpriv, u8 variable,u8 *pval)
{
	struct rtl_efuse *rtlefuse =  rtl_efuse(rtlpriv);
	struct _rtw_hal *pHalData;
	struct _rtw_dm *podmpriv;
	uint8_t val8;
	u16 val16;

	pHalData = GET_HAL_DATA(rtlpriv);
	podmpriv = &pHalData->odmpriv;

	switch (variable) {
	case HW_VAR_TXPAUSE:
		*pval = rtl_read_byte(rtlpriv, REG_TXPAUSE);
		break;

	case HW_VAR_BCN_VALID:
		{
			/* BCN_VALID, BIT(16) of REG_TDECTRL = BIT(0) of REG_TDECTRL+2 */
			val8 = rtl_read_byte(rtlpriv, REG_TDECTRL+2);
			*pval = (BIT(0) & val8) ? true:false;
		}
		break;

	case HW_VAR_FWLPS_RF_ON:
		/* When we halt NIC, we should check if FW LPS is leave. */
		if (rtlpriv->pwrctrlpriv.rf_pwrstate == ERFOFF) {
			/*
			 *  If it is in HW/SW Radio OFF or IPS state, we do not check Fw LPS Leave,
			 *  because Fw is unload.
			 */
			*pval = true;
		} else {
			uint32_t valRCR;
			valRCR = rtl_read_dword(rtlpriv, REG_RCR);
			valRCR &= 0x00070000;
			if (valRCR)
				*pval = false;
			else
				*pval = true;
		}

		break;

	case HW_VAR_EFUSE_BYTES: /*  To get EFUE total used bytes, added by Roger, 2008.12.22. */
		*(u16 *)pval = rtlefuse->efuse_usedbytes;
		break;

	case HW_VAR_CHK_HI_QUEUE_EMPTY:
		val16 = rtl_read_word(rtlpriv, REG_TXPKT_EMPTY);
		*pval = (val16 & BIT(10)) ? true:false;
		break;

	default:
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s: [WARNNING] variable(%d) not defined!\n", __FUNCTION__, variable);
		break;
	}
}

/*
 * These functions sets the media type register
 * AP, STA, ADHOC
 */

void Set_MSR(struct rtl_priv *rtlpriv, uint8_t type)
{
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MEDIA_STATUS, (uint8_t *)(&type));
}

void rtl8821au_read_chip_version(struct rtl_priv *rtlpriv)
{
	uint32_t	value32;
	enum version_8821au chip_version = VERSION_UNKNOWN;
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	value32 = rtl_read_dword(rtlpriv, REG_SYS_CFG);
	RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s SYS_CFG(0x%X)=0x%08x \n", __FUNCTION__, REG_SYS_CFG, value32);

	if (IS_HARDWARE_TYPE_8812(rtlhal))
		chip_version = CHIP_8812;
	else
		chip_version = CHIP_8821;

	chip_version |= ((value32 & RTL_ID) ? 0 : NORMAL_CHIP);

	if (IS_HARDWARE_TYPE_8812(rtlhal))
		chip_version |= RF_TYPE_2T2R;	/* RF_2T2R; */
	else
		chip_version |= RF_TYPE_1T1R;	/*RF_1T1R; */

	if (IS_HARDWARE_TYPE_8812(rtlhal))
		chip_version |= ((value32 & VENDOR_ID) ? CHIP_VENDOR_UMC : 0);
	else {
		uint32_t vendor;

		vendor = (value32 & EXT_VENDOR_ID) >> EXT_VENDOR_ID_SHIFT;
		switch (vendor) {
		case 2:
			chip_version |= CHIP_VENDOR_UMC;
			break;
		}
	}
	
	if (IS_HARDWARE_TYPE_8812(rtlhal)) {
		u32 rtl_id = ((value32 & CHIP_VER_RTL_MASK) >> CHIP_VER_RTL_SHIFT) + 1;
		
		chip_version = (enum version_8821au) (chip_version | (rtl_id << 12));
	} else {
		u32 rtl_id = ((value32 & CHIP_VER_RTL_MASK) >> CHIP_VER_RTL_SHIFT);
		
		chip_version = (enum version_8821au) (chip_version | (rtl_id << 12));
	}
	
#if 0	
	/* value32 = rtl_read_dword(rtlpriv, REG_GPIO_OUTSTS); */
	ChipVersion.ROMVer = 0;	/* ROM code version. */
#endif
	/* For multi-function consideration. Added by Roger, 2010.10.06. */
	value32 = rtl_read_dword(rtlpriv, REG_MULTI_FUNC_CTRL);
	rtlpriv->phy.polarity_ctl = ((value32 & WL_HWPDN_SL) ? RT_POLARITY_HIGH_ACT : RT_POLARITY_LOW_ACT);

	if (IS_1T2R(chip_version)) {
		rtlpriv->phy.rf_type = RF_1T2R;
		 rtlpriv->phy.num_total_rfpath = 2;
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "==> RF_Type : 1T2R\n");
	} else if (IS_2T2R(chip_version)) {
		rtlpriv->phy.rf_type = RF_2T2R;
		 rtlpriv->phy.num_total_rfpath = 2;
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "==> RF_Type : 2T2R\n");
	} else {
		rtlpriv->phy.rf_type = RF_1T1R;
		 rtlpriv->phy.num_total_rfpath = 1;
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "==> RF_Type 1T1R\n");
	}

	RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "RF_Type is %x!!\n", rtlpriv->phy.rf_type);
}


static int32_t _rtl8821au_llt_write(struct rtl_priv *rtlpriv, uint32_t address, uint32_t data)
{
	bool status = true;
	int32_t	count = 0;
	uint32_t value = _LLT_INIT_ADDR(address) | _LLT_INIT_DATA(data) |
			 _LLT_OP(_LLT_WRITE_ACCESS);

	rtl_write_dword(rtlpriv, REG_LLT_INIT, value);

	/* polling */
	do {
		value = rtl_read_dword(rtlpriv, REG_LLT_INIT);
		if (_LLT_NO_ACTIVE == _LLT_OP_VALUE(value)) {
			break;
		}

		if (count > POLLING_LLT_THRESHOLD) {
			status = false;
			break;
		}
	} while (count++);

	return status;
}

int32_t  _rtl8821au_llt_table_init(struct rtl_priv *rtlpriv, uint8_t txpktbuf_bndy)
{
	bool status;
	uint32_t i;
	uint32_t Last_Entry_Of_TxPktBuf = LAST_ENTRY_OF_TX_PKT_BUFFER_8812;

	for (i = 0; i < (txpktbuf_bndy - 1); i++) {
		status = _rtl8821au_llt_write(rtlpriv, i, i + 1);
		if (!status)
			return status;
	}

	/* end of list */
	status = _rtl8821au_llt_write(rtlpriv, (txpktbuf_bndy - 1), 0xFF);
	if (_SUCCESS != status) {
		return status;
	}

	/*
	 * Make the other pages as ring buffer
	 * This ring buffer is used as beacon buffer if we config this MAC as two MAC transfer.
	 * Otherwise used as local loopback buffer.
	 */
	for (i = txpktbuf_bndy; i < Last_Entry_Of_TxPktBuf; i++) {
		status = _rtl8821au_llt_write(rtlpriv, i, (i + 1));
		if (!status)
			return status;
	}

	/*  Let last entry point to the start entry of ring buffer */
	status = _rtl8821au_llt_write(rtlpriv, Last_Entry_Of_TxPktBuf, txpktbuf_bndy);
	if (!status)
		return status;

	return true;
}

static void _rtl8812au_read_rfe_type(struct rtl_priv *rtlpriv, u8 *hwinfo,
		bool autoload_fail)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	if (!autoload_fail) {
		if (hwinfo[EEPROM_RFE_OPTION_8812] & BIT(7)) {
			if (rtlhal->external_lna_5g) {
				if (rtlhal->external_pa_5g) {
					if (rtlhal->external_lna_2g && rtlhal->external_pa_2g)
						rtlhal->rfe_type = 3;
					else
						rtlhal->rfe_type = 0;
				} else
					rtlhal->rfe_type = 2;
			} else
				rtlhal->rfe_type = 4;
		} else {
			rtlhal->rfe_type= hwinfo[EEPROM_RFE_OPTION_8812]&0x3F;

			/*
			 * 2013/03/19 MH Due to othe customer already use incorrect EFUSE map
			 * to for their product. We need to add workaround to prevent to modify
			 * spec and notify all customer to revise the IC 0xca content. After
			 * discussing with Willis an YN, revise driver code to prevent.
			 */
			if (rtlhal->rfe_type == 4 &&
			   (rtlhal->external_pa_5g == true || rtlhal->external_pa_2g == true ||
			    rtlhal->external_lna_5g == true || rtlhal->external_lna_2g == true)) {
				if (IS_HARDWARE_TYPE_8812AU(rtlhal))
					rtlhal->rfe_type = 0;
			}
		}
	} else {
		rtlhal->rfe_type = EEPROM_DEFAULT_RFE_OPTION;
	}
#if 0
	DBG_871X("RFE Type: 0x%2x\n", rtlhal->rfe_type);
#endif	
}

static void _rtl8812au_read_pa_type(struct rtl_priv *rtlpriv, u8 *hwinfo,
				    bool autoload_fail);

static void _rtl8821au_read_pa_type(struct rtl_priv *rtlpriv, u8 *hwinfo,
				    bool autoload_fail);

static void _rtl88au_read_txpower_info_from_hwpg(struct rtl_priv *rtlpriv, u8 *hwinfo,
	bool autoload_fail);


/* ULLI : refractoring this into one function _read_adapter_info() */

static void Hal_ReadChannelPlan8812A(struct rtl_priv *rtlpriv, uint8_t *hwinfo,
	bool	AutoLoadFail)
{
	rtlpriv->mlmepriv.ChannelPlan = hal_com_get_channel_plan(
		rtlpriv
		, hwinfo?hwinfo[EEPROM_ChannelPlan_8812]:0xFF
		,  RT_CHANNEL_DOMAIN_MAX
		, RT_CHANNEL_DOMAIN_REALTEK_DEFINE
		, AutoLoadFail
	);

	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "mlmepriv.ChannelPlan = 0x%02x\n", rtlpriv->mlmepriv.ChannelPlan);
}

void _rtl8821au_read_adapter_info(struct rtl_priv *rtlpriv)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct rtl_usb_priv *usbpriv = rtl_usbpriv(rtlpriv);
	struct rtl_led_ctl *pledpriv = &(usbpriv->ledpriv);
	uint8_t	tmp_u1b;
	u8 hwinfo[HWSET_MAX_SIZE_JAGUAR];
	u16 EEPROMId;

	/* Read all content in Efuse/EEPROM. */

	/* check system boot selection */
	tmp_u1b = rtl_read_byte(rtlpriv, REG_9346CR);
	if (tmp_u1b & BIT(4)) {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_DMESG, "Boot from EEPROM\n");
		rtlefuse->epromtype = EEPROM_93C46;
	} else {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_DMESG, "Boot from EFUSE\n");
		rtlefuse->epromtype = EEPROM_BOOT_EFUSE;
	}

	if (tmp_u1b & EEPROM_EN) {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "Autoload OK\n");
		rtlefuse->autoload_failflag = false;
	} else {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_EMERG, "Autoload ERR!!\n");
		/* ULLI : not in rtlwifi, maybe autoload_failflag to set to true */
		rtlefuse->autoload_failflag = false;

	}

	/* pHalData->EEType = IS_BOOT_FROM_EEPROM(rtlpriv) ? EEPROM_93C46 : EEPROM_BOOT_EFUSE; */

	if (rtlefuse->autoload_failflag == false) { /* autoload OK. */
		rtw_efuse_shadow_map_update(rtlpriv);
		memcpy(hwinfo, &rtlefuse->efuse_map[EFUSE_INIT_MAP][0],
		       HWSET_MAX_SIZE_JAGUAR);
	} else {	/* autoload fail */
		/*
		 * pHalData->AutoloadFailFlag = true;
		 * update to default value 0xFF
		 */
		if (rtlefuse->epromtype == EEPROM_BOOT_EFUSE) {
			rtw_efuse_shadow_map_update(rtlpriv);
			memcpy(hwinfo, &rtlefuse->efuse_map[EFUSE_INIT_MAP][0],
			       HWSET_MAX_SIZE_JAGUAR);
		}

	}

	/*  Checl 0x8129 again for making sure autoload status!! */
	EEPROMId = le16_to_cpu(*((u16 *)hwinfo));
	if (EEPROMId != RTL_EEPROM_ID) {
		RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "EEPROM ID(%#x) is invalid!!\n", EEPROMId);
		rtlefuse->autoload_failflag = true;
	} else {
		rtlefuse->autoload_failflag = false;
	}

	if (rtlefuse->autoload_failflag) {
		rtlefuse->eeprom_version = EEPROM_Default_Version;
	} else{
		rtlefuse->eeprom_version = hwinfo[EEPROM_VERSION_8812];
		if (rtlefuse->eeprom_version == 0xFF)
			rtlefuse->eeprom_version = EEPROM_Default_Version;
	}

	if (!rtlefuse->autoload_failflag) {
		/* VID, PID */
		if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
			rtlefuse->eeprom_vid = *((u16 *) &hwinfo[EEPROM_VID_8812AU]);
			rtlefuse->eeprom_did = *((u16 *) &hwinfo[EEPROM_PID_8812AU]);
		} else if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
			rtlefuse->eeprom_vid = *((u16 *) &hwinfo[EEPROM_VID_8821AU]);
			rtlefuse->eeprom_did = *((u16 *) &hwinfo[EEPROM_PID_8821AU]);
		}

		/* Customer ID, 0x00 and 0xff are reserved for Realtek. */
		rtlefuse->eeprom_oemid = hwinfo[EEPROM_CustomID_8812];
/* ULLI : rot in rtlwifi
 *		efuse->EEPROMSubCustomerID = EEPROM_Default_SubCustomerID;
 */

	} else {
		rtlefuse->eeprom_vid = EEPROM_Default_VID;
		rtlefuse->eeprom_did = EEPROM_Default_PID;

		/* Customer ID, 0x00 and 0xff are reserved for Realtek. */
		rtlefuse->eeprom_oemid		= EEPROM_Default_CustomerID;
/* ULLI : rot in rtlwifi
 * 		efuse->EEPROMSubCustomerID	= EEPROM_Default_SubCustomerID;
 */
	}

	if (!rtlefuse->autoload_failflag) {
		if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
			/* Read Permanent MAC address and set value to hardware */
			memcpy(rtlpriv->mac80211.mac_addr, &hwinfo[EEPROM_MAC_ADDR_8812AU], ETH_ALEN);
		} else if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
			/*  Read Permanent MAC address and set value to hardware */
			memcpy(rtlpriv->mac80211.mac_addr, &hwinfo[EEPROM_MAC_ADDR_8821AU], ETH_ALEN);
		}
	} else {
		/* Random assigh MAC address */
		u8 sMacAddr[ETH_ALEN] = {0x00, 0xE0, 0x4C, 0x88, 0x12, 0x00};
		/* sMacAddr[5] = (u8)GetRandomNumber(1, 254); */
		memcpy(rtlpriv->mac80211.mac_addr, sMacAddr, ETH_ALEN);
	}

	_rtl88au_read_txpower_info_from_hwpg(rtlpriv, &rtlefuse->efuse_map[0][0], rtlefuse->autoload_failflag);
	

#if 0	/* ULLI check this in old source, may be vendor specific ?? */
	/* ULLI from Hal_ReadBoardType8812A() */
	if (!autoload_fail) {
		pHalData->InterfaceSel = (hwinfo[EEPROM_RF_BOARD_OPTION_8812]&0xE0)>>5;
		if (hwinfo[EEPROM_RF_BOARD_OPTION_8812] == 0xFF)
			pHalData->InterfaceSel = (EEPROM_DEFAULT_BOARD_OPTION&0xE0)>>5;
	} else {
		pHalData->InterfaceSel = 0;
	}
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "Board Type: 0x%2x\n", pHalData->InterfaceSel);
#endif

	/*
	 * Read Bluetooth co-exist and initialize
	 */

	Hal_ReadChannelPlan8812A(rtlpriv, &rtlefuse->efuse_map[0][0], rtlefuse->autoload_failflag);

	if (!rtlefuse->autoload_failflag) {
		rtlefuse->crystalcap = hwinfo[EEPROM_XTAL_8812];
		if (rtlefuse->crystalcap == 0xFF)
			rtlefuse->crystalcap = EEPROM_Default_CrystalCap_8812;	 /* what value should 8812 set? */
	} else {
		rtlefuse->crystalcap = EEPROM_Default_CrystalCap_8812;
	}


	if (!rtlefuse->autoload_failflag)
		rtlefuse->eeprom_thermalmeter = hwinfo[EEPROM_THERMAL_METER_8812];
	else
		rtlefuse->eeprom_thermalmeter = EEPROM_Default_ThermalMeter_8812;
	/* pHalData->EEPROMThermalMeter = (tempval&0x1f);	//[4:0] */

	if (rtlefuse->eeprom_thermalmeter == 0xff || rtlefuse->autoload_failflag) {
		rtlefuse->apk_thermalmeterignore = true;
		rtlefuse->eeprom_thermalmeter = 0xFF;
	}

	if (!rtlefuse->autoload_failflag) {
		u8 tmp;
		
		tmp = hwinfo[EEPROM_RF_BOARD_OPTION_8812];
		
		/*  Antenna Diversity setting. */
		rtlefuse->antenna_div_cfg = (tmp & 0x18) >>3;
		if (tmp == 0xFF)
			rtlefuse->antenna_div_cfg = (EEPROM_DEFAULT_BOARD_OPTION & 0x18) >> 3;;
	} else {
		rtlefuse->antenna_div_cfg = 0;
	}

	if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
		_rtl8821au_read_pa_type(rtlpriv, &rtlefuse->efuse_map[0][0], rtlefuse->autoload_failflag);
	} else {
		_rtl8812au_read_pa_type(rtlpriv, &rtlefuse->efuse_map[0][0], rtlefuse->autoload_failflag);
		_rtl8812au_read_rfe_type(rtlpriv, &rtlefuse->efuse_map[0][0], rtlefuse->autoload_failflag);
	}

	/* For customized behavior. */

	if ((rtlefuse->eeprom_vid == 0x050D) && (rtlefuse->eeprom_did == 0x1106))		/* SerComm for Belkin. */
		rtlhal->oem_id = RT_CID_Sercomm_Belkin;	/* ULLI : RTL8812 */
	else if ((rtlefuse->eeprom_vid == 0x0846) && (rtlefuse->eeprom_did == 0x9052))	/* SerComm for Netgear. */
		rtlhal->oem_id = RT_CID_Sercomm_Netgear;	/* ULLI :  posible typo for pid maybe 0x9052 */
	else if ((rtlefuse->eeprom_vid == 0x2001) && (rtlefuse->eeprom_did == 0x330e))	/* add by ylb 20121012 for customer led for alpha */
		rtlhal->oem_id = RT_CID_ALPHA_Dlink;	/* ULLI : RTL8812 */
	else if ((rtlefuse->eeprom_vid == 0x0B05) && (rtlefuse->eeprom_did == 0x17D2))	/* Edimax for ASUS */
		rtlhal->oem_id = RT_CID_Edimax_ASUS;	/* ULLI : RTL8812 */

	/* Decide CustomerID according to VID/DID or EEPROM */
	switch (rtlefuse->eeprom_oemid) {
	case EEPROM_CID_DEFAULT:
		if ((rtlefuse->eeprom_vid == 0x0846) && (rtlefuse->eeprom_did == 0x9052))
			rtlhal->oem_id = RT_CID_NETGEAR;		/* ULLI : RTL8821 */
		break;
	default:
		rtlhal->oem_id = RT_CID_DEFAULT;
		break;

	}

	pledpriv->led_opendrain = true;	/* Support Open-drain arrangement for controlling the LED. Added by Roger, 2009.10.16. */

#if 0	/* ULLI check this in old source, may be vendor specific ?? */
	if(pHalData->InterfaceSel == INTF_SEL1_USB_High_Power) 	{
		rtlhal->external_pa_2g = 1;
		rtlhal->external_lna_2g = 1;
	} else {
		rtlhal->external_lna_2g = 0;
	}
#endif
	rtlefuse->board_type = ODM_BOARD_DEFAULT;
	if (rtlhal->external_lna_2g != 0) {
		rtlefuse->board_type |= ODM_BOARD_EXT_LNA;
	}
	if (rtlhal->external_lna_5g != 0) {
		rtlefuse->board_type |= ODM_BOARD_EXT_LNA_5G;
	}
	if (rtlhal->external_pa_2g != 0) {
		rtlefuse->board_type |= ODM_BOARD_EXT_PA;
	}
	if (rtlhal->external_pa_5g != 0) {
		rtlefuse->board_type |= ODM_BOARD_EXT_PA_5G;
	}

	rtlhal->board_type = rtlefuse->board_type;

}

bool rtl8821au_gpio_radio_on_off_checking(struct rtl_priv *rtlpriv, u8 *valid)
{
	uint8_t	val8;
	enum rf_pwrstate rfpowerstate = ERFOFF;

	{ /* rf on/off */
		rtl_write_byte(rtlpriv, REG_MAC_PINMUX_CFG, rtl_read_byte(rtlpriv, REG_MAC_PINMUX_CFG)&~(BIT(3)));
		val8 = rtl_read_byte(rtlpriv, REG_GPIO_IO_SEL);
#if 0		
		DBG_8192C("GPIO_IN=%02x\n", val8);
#endif		
		rfpowerstate = (val8 & BIT(3)) ? ERFON : ERFOFF;
	}
	return rfpowerstate;
}


static void _InitBurstPktLen(struct rtl_priv *rtlpriv)
{
	struct rtl_usb	*rtlusb = rtl_usbdev(rtlpriv);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	u8 speedvalue, provalue, temp;
 	struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);

	/*
	 * rtl_write_word(rtlpriv, REG_TRXDMA_CTRL_8195, 0xf5b0);
	 * rtl_write_word(rtlpriv, REG_TRXDMA_CTRL_8812, 0xf5b4);
	 */
	rtl_write_byte(rtlpriv, 0xf050, 0x01);		/* usb3 rx interval */
	rtl_write_word(rtlpriv, REG_RXDMA_STATUS, 0x7400);	/* burset lenght=4, set 0x3400 for burset length=2 */
	rtl_write_byte(rtlpriv, 0x289, 0xf5);		/* for rxdma control */
	/* rtl_write_byte(rtlpriv, 0x3a, 0x46); */

	/*  0x456 = 0x70, sugguested by Zhilin */
	rtl_write_byte(rtlpriv, REG_AMPDU_MAX_TIME_8812, 0x70);

	rtl_write_dword(rtlpriv, 0x458, 0xffffffff);
	rtl_write_byte(rtlpriv, REG_USTIME_TSF, 0x50);
	rtl_write_byte(rtlpriv, REG_USTIME_EDCA, 0x50);

	if (IS_HARDWARE_TYPE_8821U(rtlhal))
		speedvalue = BIT(7);
	else
		speedvalue = rtl_read_byte(rtlpriv, 0xff); /* check device operation speed: SS 0xff BIT(7) */

	if (speedvalue & BIT(7)) {		/* USB2/1.1 Mode */
		temp = rtl_read_byte(rtlpriv, 0xfe17);
		if (((temp >> 4) & 0x03) == 0) {
			rtlusb->max_bulk_out_size = USB_HIGH_SPEED_BULK_SIZE;
			provalue = rtl_read_byte(rtlpriv, REG_RXDMA_PRO_8812);
			rtl_write_byte(rtlpriv, REG_RXDMA_PRO_8812, ((provalue|BIT(4))&(~BIT(5)))); /* set burst pkt len=512B */
			rtl_write_word(rtlpriv, REG_RXDMA_PRO_8812, 0x1e);
		} else {
			rtlusb->max_bulk_out_size = USB_FULL_SPEED_BULK_SIZE;
			provalue = rtl_read_byte(rtlpriv, REG_RXDMA_PRO_8812);
			rtl_write_byte(rtlpriv, REG_RXDMA_PRO_8812, ((provalue|BIT(5))&(~BIT(4)))); /* set burst pkt len=64B */
		}

		rtl_write_word(rtlpriv, REG_RXDMA_AGG_PG_TH, 0x2005); /* dmc agg th 20K */

		/*
		 * rtl_write_byte(rtlpriv, 0x10c, 0xb4);
		 * hal_UphyUpdate8812AU(rtlpriv);
		 */

		rtlpriv->rtlhal.version |= ~RTL8821AU_USB3_MODE;
	} else {		/* USB3 Mode */
		rtlusb->max_bulk_out_size = USB_SUPER_SPEED_BULK_SIZE;
		provalue = rtl_read_byte(rtlpriv, REG_RXDMA_PRO_8812);
		rtl_write_byte(rtlpriv, REG_RXDMA_PRO_8812, provalue&(~(BIT(5)|BIT(4)))); /* set burst pkt len=1k */
		rtl_write_word(rtlpriv, REG_RXDMA_PRO_8812, 0x0e);
		rtlpriv->rtlhal.version |= ~RTL8821AU_USB3_MODE;

		/*  set Reg 0xf008[3:4] to 2'00 to disable U1/U2 Mode to avoid 2.5G spur in USB3.0. added by page, 20120712 */
		rtl_write_byte(rtlpriv, 0xf008, rtl_read_byte(rtlpriv, 0xf008)&0xE7);
	}

#if 0
	/* ULLI disabled through CONFIG_USB_TX_AGGREGATION is in use */
	rtl_write_byte(rtlpriv, REG_TDECTRL, 0x10);
#endif
	temp = rtl_read_byte(rtlpriv, REG_SYS_FUNC_EN);
	rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN, temp&(~BIT(10))); 	/* reset 8051 */

	rtl_write_byte(rtlpriv, REG_HT_SINGLE_AMPDU_8812, rtl_read_byte(rtlpriv, REG_HT_SINGLE_AMPDU_8812)|BIT(7));	/* enable single pkt ampdu */
	rtl_write_byte(rtlpriv, REG_RX_PKT_LIMIT, 0x18);		/* for VHT packet length 11K */

	rtl_write_byte(rtlpriv, REG_PIFS, 0x00);

	/* Suggention by SD1 Jong and Pisa, by Maddest 20130107. */
	if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
		rtl_write_word(rtlpriv, REG_MAX_AGGR_NUM, 0x0a0a);
		rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL, 0x80);
		rtl_write_byte(rtlpriv, REG_AMPDU_MAX_TIME_8812, 0x5e);
		rtl_write_dword(rtlpriv, REG_FAST_EDCA_CTRL, 0x03087777);
	} else {
		rtl_write_byte(rtlpriv, REG_MAX_AGGR_NUM, 0x1f);
		rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL, rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL)&(~BIT(7)));
	}

	if (pHalData->AMPDUBurstMode)
		rtl_write_byte(rtlpriv, REG_AMPDU_BURST_MODE_8812, 0x5F);

	rtl_write_byte(rtlpriv, 0x1c, rtl_read_byte(rtlpriv, 0x1c) | BIT(5) | BIT(6));  /* to prevent mac is reseted by bus. 20111208, by Page */

	/* ARFB table 9 for 11ac 5G 2SS */
	rtl_write_dword(rtlpriv, REG_ARFR0, 0x00000010);
	if (IS_NORMAL_CHIP(rtlhal->version))
		rtl_write_dword(rtlpriv, REG_ARFR0+4, 0xfffff000);
	else
		rtl_write_dword(rtlpriv, REG_ARFR0+4, 0x3e0ff000);

	/* ARFB table 10 for 11ac 5G 1SS */
	rtl_write_dword(rtlpriv, REG_ARFR1, 0x00000010);
	if (IS_VENDOR_8812A_TEST_CHIP(rtlhal->version))
		rtl_write_dword(rtlpriv, REG_ARFR1_8812+4, 0x000ff000);
	else
		rtl_write_dword(rtlpriv, REG_ARFR1_8812+4, 0x003ff000);

}

static uint32_t _InitPowerOn8812AU(struct rtl_priv *rtlpriv)
{
	u16	u2btmp = 0;
	uint8_t	u1btmp = 0;
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	if (IS_VENDOR_8821A_MP_CHIP(rtlhal->version)) {
		/* HW Power on sequence */
		if (!rtw_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_A_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, Rtl8821A_NIC_ENABLE_FLOW)) {
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s: run power on flow fail\n", __func__);
			return _FAIL;
		}
	} else if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
		if (!rtw_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_TESTCHIP_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, Rtl8821A_NIC_ENABLE_FLOW)) {
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s: run power on flow fail\n", __func__);
			return _FAIL;
		}
	} else {
		if (!rtw_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, Rtl8812_NIC_ENABLE_FLOW)) {
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s: run power on flow fail\n", __func__);
			return _FAIL;
		}
	}

	/*
	 *  Enable MAC DMA/WMAC/SCHEDULE/SEC block
	 * Set CR BIT(10) to enable 32k calibration. Suggested by SD1 Gimmy. Added by tynli. 2011.08.31.
	 */
	rtl_write_word(rtlpriv, REG_CR, 0x00); 	/* suggseted by zhouzhou, by page, 20111230 */
	u2btmp = rtl_read_word(rtlpriv, REG_CR);
	u2btmp |= (HCI_TXDMA_EN | HCI_RXDMA_EN | TXDMA_EN | RXDMA_EN
				| PROTOCOL_EN | SCHEDULE_EN | ENSEC | CALTMR_EN);
	rtl_write_word(rtlpriv, REG_CR, u2btmp);

	/*
	 * Need remove below furture, suggest by Jackie.
	 * if 0xF0[24] =1 (LDO), need to set the 0x7C[6] to 1.
	 */
	if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
		u1btmp = rtl_read_byte(rtlpriv, REG_SYS_CFG+3);
		if (u1btmp & BIT(0)) { 	/* LDO mode. */
			u1btmp = rtl_read_byte(rtlpriv, 0x7c);
			/* ULLI unknown register */
			rtl_write_byte(rtlpriv, 0x7c, u1btmp | BIT(6));
		}
	}

	return _SUCCESS;
}

/* Shall USB interface init this? */
void rtl8821au_enable_interrupt(struct rtl_priv *rtlpriv)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);

	/* HIMR */
	rtl_write_dword(rtlpriv, REG_HIMR0_8812, rtlusb->irq_mask[0]&0xFFFFFFFF);
	rtl_write_dword(rtlpriv, REG_HIMR1_8812, rtlusb->irq_mask[1]&0xFFFFFFFF);
}

void rtl8821au_disable_interrupt(struct rtl_priv *rtlpriv)
{
	rtl_write_dword(rtlpriv, REG_HIMR0_8812, IMR_DISABLED_8812);
	rtl_write_dword(rtlpriv, REG_HIMR1_8812, IMR_DISABLED_8812);
}
static void _InitQueueReservedPage_8821AUsb(struct rtl_priv *rtlpriv)
{
	struct rtl_usb  *rtlusb = rtl_usbdev(rtlpriv);
	uint32_t numHQ = 0;
	uint32_t numLQ = 0;
	uint32_t numNQ = 0;
	uint32_t numPubQ = 0;
	uint32_t value32;
	uint8_t	 value8;

	numPubQ = NORMAL_PAGE_NUM_PUBQ_8821;

	if (rtlusb->out_queue_sel & TX_SELE_HQ)
		numHQ = NORMAL_PAGE_NUM_HPQ_8821;

	if (rtlusb->out_queue_sel & TX_SELE_LQ)
		numLQ = NORMAL_PAGE_NUM_LPQ_8821;

	/* NOTE: This step shall be proceed before writting REG_RQPN. */
	if (rtlusb->out_queue_sel & TX_SELE_NQ)
		numNQ = NORMAL_PAGE_NUM_NPQ_8821;


	value8 = (u8)_NPQ(numNQ);
	rtl_write_byte(rtlpriv, REG_RQPN_NPQ, value8);

	/* TX DMA */
	value32 = _HPQ(numHQ) | _LPQ(numLQ) | _PUBQ(numPubQ) | LD_RQPN;
	rtl_write_dword(rtlpriv, REG_RQPN, value32);
}

static void _InitQueueReservedPage_8812AUsb(struct rtl_priv *rtlpriv)
{
	struct rtl_usb  *rtlusb = rtl_usbdev(rtlpriv);
	uint32_t numHQ		= 0;
	uint32_t numLQ		= 0;
	uint32_t numNQ		= 0;
	uint32_t numPubQ	= 0;
	uint32_t value32;
	uint8_t	value8;

	numPubQ = NORMAL_PAGE_NUM_PUBQ_8812;

	if (rtlusb->out_queue_sel & TX_SELE_HQ)
		numHQ = NORMAL_PAGE_NUM_HPQ_8812;

	if (rtlusb->out_queue_sel & TX_SELE_LQ)
		numLQ = NORMAL_PAGE_NUM_LPQ_8812;

	/* NOTE: This step shall be proceed before writting REG_RQPN. */
	if (rtlusb->out_queue_sel & TX_SELE_NQ)
		numNQ = NORMAL_PAGE_NUM_NPQ_8812;

	value8 = (u8)_NPQ(numNQ);
	rtl_write_byte(rtlpriv, REG_RQPN_NPQ, value8);

	/* TX DMA */
	value32 = _HPQ(numHQ) | _LPQ(numLQ) | _PUBQ(numPubQ) | LD_RQPN;
	rtl_write_dword(rtlpriv, REG_RQPN, value32);
}

static void _InitTxBufferBoundary_8821AUsb(struct rtl_priv *rtlpriv)
{
	uint8_t	txpktbuf_bndy;

	txpktbuf_bndy = TX_PAGE_BOUNDARY_8821;

	rtl_write_byte(rtlpriv, REG_BCNQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_MGQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_WMAC_LBK_BF_HD, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TRXFF_BNDY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TDECTRL+1, txpktbuf_bndy);
}

static void _InitTxBufferBoundary_8812AUsb(struct rtl_priv *rtlpriv)
{
	uint8_t	txpktbuf_bndy;

	txpktbuf_bndy = TX_PAGE_BOUNDARY_8812;

	rtl_write_byte(rtlpriv, REG_BCNQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_MGQ_BDNY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_WMAC_LBK_BF_HD, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TRXFF_BNDY, txpktbuf_bndy);
	rtl_write_byte(rtlpriv, REG_TDECTRL+1, txpktbuf_bndy);
}

static void _InitPageBoundary_8812AUsb(struct rtl_priv *rtlpriv)
{
	/*
	 * u16 			rxff_bndy;
	 * u16			Offset;
	 * bool			bSupportRemoteWakeUp;
	 */

	/*
	 * rtlpriv->cfg->ops.GetHalDefVarHandler(rtlpriv, HAL_DEF_WOWLAN , &bSupportRemoteWakeUp);
	 * RX Page Boundary
	 * srand(static_cast<unsigned int>(time(NULL)) );
	 */

	/*
	 * Offset = MAX_RX_DMA_BUFFER_SIZE_8812/256;
	 * rxff_bndy = (Offset*256)-1;
	 */

	rtl_write_word(rtlpriv, (REG_TRXFF_BNDY + 2), MAX_RX_DMA_BUFFER_SIZE_8821-1);

}

static void _InitRDGSetting_8812A(struct rtl_priv *rtlpriv)
{
	rtl_write_byte(rtlpriv, REG_RD_CTRL, 0xFF);
	rtl_write_word(rtlpriv, REG_RD_NAV_NXT, 0x200);
	rtl_write_byte(rtlpriv, REG_RD_RESP_PKT_TH, 0x05);
}


static void _rtl8821au_init_chipN_reg_priority(struct rtl_priv *rtlpriv,
	u16 beQ, u16 bkQ, u16 viQ,
	u16 voQ, u16 mgtQ, u16 hiQ)
{
	u16 value16 = (rtl_read_word(rtlpriv, REG_TRXDMA_CTRL) & 0x7);

	value16 |= _TXDMA_BEQ_MAP(beQ) 	| _TXDMA_BKQ_MAP(bkQ) |
		   _TXDMA_VIQ_MAP(viQ) 	| _TXDMA_VOQ_MAP(voQ) |
		   _TXDMA_MGQ_MAP(mgtQ) | _TXDMA_HIQ_MAP(hiQ);

	rtl_write_word(rtlpriv, REG_TRXDMA_CTRL, value16);
}

static void _rtl8821au_init_chipN_two_ep_priority(struct rtl_priv *rtlpriv)
{
	struct rtl_usb  *rtlusb = rtl_usbdev(rtlpriv);
	u16	beQ, bkQ, viQ, voQ, mgtQ, hiQ;

	u16	valueHi = 0;
	u16	valueLow = 0;

	switch (rtlusb->out_queue_sel) {
	case (TX_SELE_HQ | TX_SELE_LQ):
		valueHi = QUEUE_HIGH;
		valueLow = QUEUE_LOW;
		break;
	case (TX_SELE_NQ | TX_SELE_LQ):
		valueHi = QUEUE_NORMAL;
		valueLow = QUEUE_LOW;
		break;
	case (TX_SELE_HQ | TX_SELE_NQ):
		valueHi = QUEUE_HIGH;
		valueLow = QUEUE_NORMAL;
		break;
	default:
		valueHi = QUEUE_HIGH;
		valueLow = QUEUE_NORMAL;
		break;
	}

		beQ	= valueLow;
		bkQ	= valueLow;
		viQ	= valueHi;
		voQ	= valueHi;
		mgtQ	= valueHi;
		hiQ	= valueHi;

	_rtl8821au_init_chipN_reg_priority(rtlpriv, beQ, bkQ, viQ, voQ, mgtQ, hiQ);

}

static void _rtl8821au_init_chipN_three_ep_priority(struct rtl_priv *rtlpriv)
{
	u16	beQ, bkQ, viQ, voQ, mgtQ, hiQ;

		beQ	= QUEUE_LOW;
		bkQ	= QUEUE_LOW;
		viQ	= QUEUE_NORMAL;
		voQ	= QUEUE_HIGH;
		mgtQ 	= QUEUE_HIGH;
		hiQ	= QUEUE_HIGH;

	_rtl8821au_init_chipN_reg_priority(rtlpriv, beQ, bkQ, viQ, voQ, mgtQ, hiQ);
}

static void _rtl8821au_init_chipN_queue_priority(struct rtl_priv *rtlpriv)
{
	struct rtl_usb	*rtlusb = rtl_usbdev(rtlpriv);

	switch (rtlusb->RtNumOutPipes) {
	case 2:
		_rtl8821au_init_chipN_two_ep_priority(rtlpriv);
		break;
	case 3:
	case 4:
		_rtl8821au_init_chipN_three_ep_priority(rtlpriv);
		break;
	default:
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "_InitQueuePriority_8812AUsb(): Shall not reach here!\n");
		break;
	}
}

static void _InitTransferPageSize_8812AUsb(struct rtl_priv *rtlpriv)
{
	uint8_t	value8;
	value8 = _PSTX(PBP_512);

	rtl_write_byte(rtlpriv, REG_PBP, value8);
}

static void _InitDriverInfoSize_8812A(struct rtl_priv *rtlpriv, uint8_t	drvInfoSize)
{
	rtl_write_byte(rtlpriv, REG_RX_DRVINFO_SZ, drvInfoSize);
}


static void _InitNetworkType_8812A(struct rtl_priv *rtlpriv)
{
	uint32_t	value32;

	value32 = rtl_read_dword(rtlpriv, REG_CR);
	/*  TODO: use the other function to set network type */
	value32 = (value32 & ~MASK_NETTYPE) | _NETTYPE(NT_LINK_AP);

	rtl_write_dword(rtlpriv, REG_CR, value32);
}


static void _InitEDCA_8812AUsb(struct rtl_priv *rtlpriv)
{
	/* Set Spec SIFS (used in NAV) */
	rtl_write_word(rtlpriv, REG_SPEC_SIFS, 0x100a);
	rtl_write_word(rtlpriv, REG_MAC_SPEC_SIFS, 0x100a);

	/* Set SIFS for CCK */
	rtl_write_word(rtlpriv, REG_SIFS_CTX, 0x100a);

	/* Set SIFS for OFDM */
	rtl_write_word(rtlpriv, REG_SIFS_TRX, 0x100a);

	/* TXOP */
	rtl_write_dword(rtlpriv, REG_EDCA_BE_PARAM, 0x005EA42B);
	rtl_write_dword(rtlpriv, REG_EDCA_BK_PARAM, 0x0000A44F);
	rtl_write_dword(rtlpriv, REG_EDCA_VI_PARAM, 0x005EA324);
	rtl_write_dword(rtlpriv, REG_EDCA_VO_PARAM, 0x002FA226);

	/* 0x50 for 80MHz clock */
	rtl_write_byte(rtlpriv, REG_USTIME_TSF, 0x50);
	rtl_write_byte(rtlpriv, REG_USTIME_EDCA, 0x50);
}

static void _InitRetryFunction_8812A(struct rtl_priv *rtlpriv)
{
	uint8_t	value8;

	value8 = rtl_read_byte(rtlpriv, REG_FWHW_TXQ_CTRL);
	value8 |= EN_AMPDU_RTY_NEW;
	rtl_write_byte(rtlpriv, REG_FWHW_TXQ_CTRL, value8);

	/*
	 * Set ACK timeout
	 * rtl_write_byte(rtlpriv, REG_ACKTO, 0x40);  //masked by page for BCM IOT issue temporally
	 */
	rtl_write_byte(rtlpriv, REG_ACKTO, 0x80);
}

static void _InitWMACSetting_8812A(struct rtl_priv *rtlpriv)
{
	/* uint32_t			value32; */
	/* u16			value16; */
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);

	/*
	 * pHalData->ReceiveConfig = AAP | APM | AM | AB | APP_ICV | ADF | AMF | APP_FCS | HTC_LOC_CTRL | APP_MIC | APP_PHYSTS;
	 */
	pHalData->ReceiveConfig = RCR_APM | RCR_AM | RCR_AB | RCR_CBSSID_DATA
				| RCR_CBSSID_BCN | RCR_APP_ICV | RCR_AMF
				| RCR_HTC_LOC_CTRL | RCR_APP_MIC
				| RCR_APP_PHYST_RXFF;

#if (1 == RTL8812A_RX_PACKET_INCLUDE_CRC)
	pHalData->ReceiveConfig |= ACRC32;
#endif

	pHalData->ReceiveConfig |= FORCEACK;

	/*
	 *  some REG_RCR will be modified later by phy_ConfigMACWithHeaderFile()
	 */
	rtl_write_dword(rtlpriv, REG_RCR, pHalData->ReceiveConfig);

	/* Accept all multicast address */
	rtl_write_dword(rtlpriv, REG_MAR, 0xFFFFFFFF);
	rtl_write_dword(rtlpriv, REG_MAR + 4, 0xFFFFFFFF);


	/*
	 *  Accept all data frames
	 * value16 = 0xFFFF;
	 * rtl_write_word(rtlpriv, REG_RXFLTMAP2, value16);
	 */

	/*
	 * 2010.09.08 hpfan
	 * Since ADF is removed from RCR, ps-poll will not be indicate to driver,
	 * RxFilterMap should mask ps-poll to gurantee AP mode can rx ps-poll.
	 * value16 = 0x400;
	 * rtl_write_word(rtlpriv, REG_RXFLTMAP1, value16);
	 */

	/*
	 *  Accept all management frames
	 * value16 = 0xFFFF;
	 * rtl_write_word(rtlpriv, REG_RXFLTMAP0, value16);
	 */

	/*
	 * enable RX_SHIFT bits
	 * rtl_write_byte(rtlpriv, REG_TRXDMA_CTRL, rtl_read_byte(rtlpriv, REG_TRXDMA_CTRL)|BIT(1));
	 */

}

static void _InitAdaptiveCtrl_8812AUsb(struct rtl_priv *rtlpriv)
{
	u16	value16;
	uint32_t	value32;

	/* Response Rate Set */
	value32 = rtl_read_dword(rtlpriv, REG_RRSR);
	value32 &= ~RATE_BITMAP_ALL;

	/* ULLI : maybe WIRELESS_MODE_MAX from ieee80211.h is meant ?? */

	if (WIRELESS_MODE_MAX & WIRELESS_11B)
		value32 |= RATE_RRSR_CCK_ONLY_1M;
	else
		value32 |= RATE_RRSR_WITHOUT_CCK;

	value32 |= RATE_RRSR_CCK_ONLY_1M;
	rtl_write_dword(rtlpriv, REG_RRSR, value32);

	/*
	 * CF-END Threshold
	 * m_spIoBase->rtl_write_byte(REG_CFEND_TH, 0x1);
	 */

	/* SIFS (used in NAV) */
	value16 = _SPEC_SIFS_CCK(0x10) | _SPEC_SIFS_OFDM(0x10);
	rtl_write_word(rtlpriv, REG_SPEC_SIFS, value16);

	/* Retry Limit */
	value16 = _LRL(0x30) | _SRL(0x30);
	rtl_write_word(rtlpriv, REG_RL, value16);

}


static void _InitBeaconMaxError_8812A(struct rtl_priv *rtlpriv, bool	InfraMode)
{
	/* ULLI: looks here is some hacking done, wrong nams ?? */
#ifdef RTL8192CU_ADHOC_WORKAROUND_SETTING
	rtl_write_byte(rtlpriv, REG_BCN_MAX_ERR, 0xFF);
#else
	/* rtl_write_byte(rtlpriv, REG_BCN_MAX_ERR, (InfraMode ? 0xFF : 0x10)); */
#endif
}

/*-----------------------------------------------------------------------------
 * Function:	usb_AggSettingTxUpdate()
 *
 * Overview:	Seperate TX/RX parameters update independent for TP detection and
 *			dynamic TX/RX aggreagtion parameters update.
 *
 * Input:			_ADAPTER
 *
 * Output/Return:	NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	12/10/2010	MHC		Seperate to smaller function.
 *
 *---------------------------------------------------------------------------*/
static void usb_AggSettingTxUpdate_8812A(struct rtl_priv *rtlpriv)
{
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);
	uint32_t			value32;

		value32 = rtl_read_dword(rtlpriv, REG_TDECTRL);
		value32 = value32 & ~(BLK_DESC_NUM_MASK << BLK_DESC_NUM_SHIFT);
		value32 |= ((pHalData->UsbTxAggDescNum & BLK_DESC_NUM_MASK) << BLK_DESC_NUM_SHIFT);

		rtl_write_dword(rtlpriv, REG_TDECTRL, value32);

}


/*-----------------------------------------------------------------------------
 * Function:	usb_AggSettingRxUpdate()
 *
 * Overview:	Seperate TX/RX parameters update independent for TP detection and
 *			dynamic TX/RX aggreagtion parameters update.
 *
 * Input:			_ADAPTER
 *
 * Output/Return:	NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	12/10/2010	MHC		Seperate to smaller function.
 *
 *---------------------------------------------------------------------------*/
static void usb_AggSettingRxUpdate_8812A(struct rtl_priv *rtlpriv)
{
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);
	uint8_t			valueDMA;

	valueDMA = rtl_read_byte(rtlpriv, REG_TRXDMA_CTRL);

	valueDMA |= RXDMA_AGG_EN;

	/* rtl_write_byte(rtlpriv, REG_RXDMA_AGG_PG_TH, 0x05); //dma agg mode, 20k
	 *
	 * 2012/10/26 MH For TX throught start rate temp fix.
	 */
	{
		u16			temp;

		/* ULLI DMA on USB Device WTF ??? */
		/* Adjust DMA page and thresh. */
		temp = pHalData->RegAcUsbDmaSize | (pHalData->RegAcUsbDmaTime<<8);
		rtl_write_word(rtlpriv, REG_RXDMA_AGG_PG_TH, temp);
	}

	rtl_write_byte(rtlpriv, REG_TRXDMA_CTRL, valueDMA);
}
static void init_UsbAggregationSetting_8812A(struct rtl_priv *rtlpriv)
{
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);

	/* Tx aggregation setting */
	usb_AggSettingTxUpdate_8812A(rtlpriv);

	/* Rx aggregation setting */
	usb_AggSettingRxUpdate_8812A(rtlpriv);

	/* 201/12/10 MH Add for USB agg mode dynamic switch. */
	pHalData->UsbRxHighSpeedMode = false;
}

static void _InitAntenna_Selection_8812A(struct rtl_priv *rtlpriv)
{
	 struct _rtw_hal	*pHalData	= GET_HAL_DATA(rtlpriv);

	if (pHalData->AntDivCfg == 0)
		return;

	rtl_write_byte(rtlpriv, REG_LEDCFG2, 0x82);

	rtl_set_bbreg(rtlpriv, RFPGA0_XAB_RFPARAMETER, BIT(13), 0x01);

	if (rtl_get_bbreg(rtlpriv, rFPGA0_XA_RFInterfaceOE, 0x300) == MAIN_ANT)
		pHalData->CurAntenna = MAIN_ANT;
	else
		pHalData->CurAntenna = AUX_ANT;
}

static void _rtl8812au_bb8812_config_1t(struct rtl_priv *rtlpriv)
{
	/* BB OFDM RX Path_A */
	rtl_set_bbreg(rtlpriv, rRxPath_Jaguar, bRxPath_Jaguar, 0x11);
	/* BB OFDM TX Path_A */
	rtl_set_bbreg(rtlpriv, rTxPath_Jaguar, bMaskLWord, 0x1111);
	/* BB CCK R/Rx Path_A */
	rtl_set_bbreg(rtlpriv, rCCK_RX_Jaguar, bCCK_RX_Jaguar, 0x0);
	/* MCS support */
	rtl_set_bbreg(rtlpriv, 0x8bc, 0xc0000060, 0x4);
	/* RF Path_B HSSI OFF */
	rtl_set_bbreg(rtlpriv, 0xe00, 0xf, 0x4);
	/* RF Path_B Power Down */
	rtl_set_bbreg(rtlpriv, 0xe90, bMaskDWord, 0);
	/* ADDA Path_B OFF */
	rtl_set_bbreg(rtlpriv, 0xe60, bMaskDWord, 0);
	rtl_set_bbreg(rtlpriv, 0xe64, bMaskDWord, 0);
}



static int PHY_RFConfig8812(struct rtl_priv *rtlpriv)
{
	int rtStatus = _SUCCESS;

	if (rtlpriv->bSurpriseRemoved)
		return _FAIL;

	rtStatus = rtl8821au_phy_rf6052_config(rtlpriv);

	return rtStatus;
}

uint32_t rtl8812au_hw_init(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = &(rtlpriv->phy);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	uint8_t	value8 = 0, u1bRegCR;
	uint8_t	txpktbuf_bndy;
	uint32_t	status = _SUCCESS;

	/* ULLI : for debuging USB3 issue, getting USB ID during hw init */
	struct rtl_usb_priv *usbpriv = rtl_usbpriv(rtlpriv);
	struct usb_device *udev = usbpriv->dev.udev;
	char *speed;

	 struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;
	enum rf_pwrstate eRfPowerStateToSet;
	uint32_t init_start_time = jiffies;

	switch (udev->speed) {
		case USB_SPEED_LOW :	speed = "LOW";
					break;
		case USB_SPEED_FULL :	speed = "FULL";
					break;
		case USB_SPEED_HIGH :	speed = "HIGH";
					break;
		case USB_SPEED_SUPER :	speed = "HIGH";
					break;
		default :		speed = "UNKNOWN";
					break;
	}

	dev_info(&udev->dev, "rtl8821au: hw_init USB-ID %04x:%04x %s %s %s-SPEED \n",
		udev->descriptor.idVendor, udev->descriptor.idProduct,
		udev->product, udev->manufacturer, speed);

	/* Check if MAC has already power on. by tynli. 2011.05.27. */
	value8 = rtl_read_byte(rtlpriv, REG_SYS_CLKR+1);
	u1bRegCR = rtl_read_byte(rtlpriv, REG_CR);
	RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, " power-on :REG_SYS_CLKR 0x09=0x%02x. REG_CR 0x100=0x%02x.\n", value8, u1bRegCR);
	if ((value8&BIT(3))  && (u1bRegCR != 0 && u1bRegCR != 0xEA)) {
		/* pHalData->bMACFuncEnable = true; */
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, " MAC has already power on.\n");
	} else {
		/*
		 * pHalData->bMACFuncEnable = false;
		 * Set FwPSState to ALL_ON mode to prevent from the I/O be return because of 32k
		 * state which is set before sleep under wowlan mode. 2012.01.04. by tynli.
		 * pHalData->FwPSState = FW_PS_STATE_ALL_ON_88E;
		 */
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, " MAC has not been powered on yet.\n");
	}

	/*
	 * 2012/11/13 MH Revise for U2/U3 switch we can not update RF-A/B reset.
	 * After discuss with BB team YN, reset after MAC power on to prevent RF
	 * R/W error. Is it a right method?
	 */
	if (!IS_HARDWARE_TYPE_8821(rtlhal)) {
		rtl_write_byte(rtlpriv, REG_RF_CTRL, 5);
		rtl_write_byte(rtlpriv, REG_RF_CTRL, 7);
		rtl_write_byte(rtlpriv, REG_RF_B_CTRL_8812, 5);
		rtl_write_byte(rtlpriv, REG_RF_B_CTRL_8812, 7);
	}

	status = _InitPowerOn8812AU(rtlpriv);
	if (status == _FAIL) {
		goto exit;
	}

		if (IS_HARDWARE_TYPE_8812(rtlhal))
			txpktbuf_bndy = TX_PAGE_BOUNDARY_8812;
		else
			txpktbuf_bndy = TX_PAGE_BOUNDARY_8821;

	status =  _rtl8821au_llt_table_init(rtlpriv, txpktbuf_bndy);
	if (status == _FAIL) {
		goto exit;
	}

#if ENABLE_USB_DROP_INCORRECT_OUT
	_InitHardwareDropIncorrectBulkOut_8812A(rtlpriv);
#endif

	if (pHalData->bRDGEnable)
		_InitRDGSetting_8812A(rtlpriv);

	status = rtl8821au_download_fw(rtlpriv, false);
	if (status != _SUCCESS) {
		RT_TRACE(rtlpriv, COMP_FW, DBG_LOUD, "%s: Download Firmware failed!!\n", __FUNCTION__);
		rtlhal->fw_ready = false;
		pHalData->fw_ractrl = false;
		/* return status; */
	} else {
		RT_TRACE(rtlpriv, COMP_FW, DBG_LOUD, "%s: Download Firmware Success!!\n", __FUNCTION__);
		rtlhal->fw_ready = true;
		pHalData->fw_ractrl = true;
	}

	InitializeFirmwareVars8812(rtlpriv);

	if (pwrctrlpriv->reg_rfoff == true)
		pwrctrlpriv->rf_pwrstate = ERFOFF;

	/*
	 * 2010/08/09 MH We need to check if we need to turnon or off RF after detecting
	 * HW GPIO pin. Before PHY_RFConfig8192C.
	 * HalDetectPwrDownMode(rtlpriv);
	 * 2010/08/26 MH If Efuse does not support sective suspend then disable the function.
	 * HalDetectSelectiveSuspendMode(rtlpriv);
	 */

	/*
	 * Save target channel
	 * <Roger_Notes> Current Channel will be updated again later.
	 */
	rtlpriv->phy.current_channel = 0;	/* set 0 to trigger switch correct channel */

	 _rtl8821au_phy_config_mac_with_headerfile(rtlpriv);

	if (IS_HARDWARE_TYPE_8812(rtlhal)) {
		_InitQueueReservedPage_8812AUsb(rtlpriv);
		_InitTxBufferBoundary_8812AUsb(rtlpriv);
	} else if (IS_HARDWARE_TYPE_8821(rtlhal)) {
		_InitQueueReservedPage_8821AUsb(rtlpriv);
		_InitTxBufferBoundary_8821AUsb(rtlpriv);
	}

	_rtl8821au_init_chipN_queue_priority(rtlpriv);
	_InitPageBoundary_8812AUsb(rtlpriv);

	if (IS_HARDWARE_TYPE_8812(rtlhal))
		_InitTransferPageSize_8812AUsb(rtlpriv);

	/* Get Rx PHY status in order to report RSSI and others. */
	_InitDriverInfoSize_8812A(rtlpriv, DRVINFO_SZ);

	rtlpriv->cfg->ops->enable_interrupt(rtlpriv);

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_ETHER_ADDR, mac->mac_addr);

	_InitNetworkType_8812A(rtlpriv);	/* set msr */
	_InitWMACSetting_8812A(rtlpriv);
	_InitAdaptiveCtrl_8812AUsb(rtlpriv);
	_InitEDCA_8812AUsb(rtlpriv);

	_InitRetryFunction_8812A(rtlpriv);
	init_UsbAggregationSetting_8812A(rtlpriv);
	rtl8821au_init_beacon_parameters(rtlpriv);
	_InitBeaconMaxError_8812A(rtlpriv, true);

	_InitBurstPktLen(rtlpriv);  /* added by page. 20110919 */

	/*
	 * Init CR MACTXEN, MACRXEN after setting RxFF boundary REG_TRXFF_BNDY to patch
	 * Hw bug which Hw initials RxFF boundry size to a value which is larger than the real Rx buffer size in 88E.
	 * 2011.08.05. by tynli.
	 */
	value8 = rtl_read_byte(rtlpriv, REG_CR);
	rtl_write_byte(rtlpriv, REG_CR, (value8|MACTXEN|MACRXEN));

#if defined(CONFIG_TX_MCAST2UNI)

#ifdef CONFIG_TX_MCAST2UNI
	rtl_write_word(rtlpriv, REG_PKT_VO_VI_LIFE_TIME, 0x0400);	/* unit: 256us. 256ms */
	rtl_write_word(rtlpriv, REG_PKT_BE_BK_LIFE_TIME, 0x0400);	/* unit: 256us. 256ms */
#else
	rtl_write_word(rtlpriv, REG_PKT_VO_VI_LIFE_TIME, 0x3000);	/* unit: 256us. 3s */
	rtl_write_word(rtlpriv, REG_PKT_BE_BK_LIFE_TIME, 0x3000);	/* unit: 256us. 3s */
#endif
#endif

	/*
	 * d. Initialize BB related configurations.
	 */


	status = rtl8821au_phy_bb_config(rtlpriv);

	if (status == _FAIL)
		goto exit;

	/*
	 * 92CU use 3-wire to r/w RF
	 * pHalData->Rf_Mode = RF_OP_By_SW_3wire;
	 */

	status = PHY_RFConfig8812(rtlpriv);
	if (status == _FAIL)
		goto exit;

	if (rtlpriv->phy.rf_type == RF_1T1R && IS_HARDWARE_TYPE_8812AU(rtlhal))
		_rtl8812au_bb8812_config_1t(rtlpriv);

	if (rtlpriv->registrypriv.channel <= 14)
		rtl8821au_phy_switch_wirelessband(rtlpriv, BAND_ON_2_4G);
	else
		rtl8821au_phy_switch_wirelessband(rtlpriv, BAND_ON_5G);

	rtlpriv->cfg->ops->set_chnl_bw_handler(rtlpriv, rtlpriv->registrypriv.channel,
					       CHANNEL_WIDTH_20, 
					       HAL_PRIME_CHNL_OFFSET_DONT_CARE, 
					       HAL_PRIME_CHNL_OFFSET_DONT_CARE);

	rtw_cam_reset_all_entry(rtlpriv);

	_InitAntenna_Selection_8812A(rtlpriv);

	/*
	 * HW SEQ CTRL
	 * set 0x0 to 0xFF by tynli. Default enable HW SEQ NUM.
	 */
	rtl_write_byte(rtlpriv, REG_HWSEQ_CTRL, 0xFF);

	/*
	 * Disable BAR, suggested by Scott
	 * 2010.04.09 add by hpfan
	 */
	rtl_write_dword(rtlpriv, REG_BAR_MODE_CTRL, 0x0201ffff);

	/* Nav limit , suggest by scott */
	rtl_write_byte(rtlpriv, 0x652, 0x0);

	rtl8812_dm_init(rtlpriv);

	/*
	 * 2010/08/11 MH Merge from 8192SE for Minicard init. We need to confirm current radio status
	 * and then decide to enable RF or not.!!!??? For Selective suspend mode. We may not
	 * call initstruct rtl_priv. May cause some problem??
	 *
	 * Fix the bug that Hw/Sw radio off before S3/S4, the RF off action will not be executed
	 * in MgntActSet_RF_State() after wake up, because the value of pHalData->eRFPowerState
	 * is the same as eRfOff, we should change it to eRfOn after we config RF parameters.
	 * Added by tynli. 2010.03.30.
	 */
	pwrctrlpriv->rf_pwrstate = ERFON;

	/*
	 * 0x4c6[3] 1: RTS BW = Data BW
	 * 0: RTS BW depends on CCA / secondary CCA result.
	 */
	rtl_write_byte(rtlpriv, REG_QUEUE_CTRL, rtl_read_byte(rtlpriv, REG_QUEUE_CTRL)&0xF7);

	/* enable Tx report. */
	rtl_write_byte(rtlpriv,  REG_FWHW_TXQ_CTRL+1, 0x0F);

	/* Suggested by SD1 pisa. Added by tynli. 2011.10.21. */
	rtl_write_byte(rtlpriv, REG_EARLY_MODE_CONTROL_8812+3, 0x01);/* Pretx_en, for WEP/TKIP SEC */

	/* tynli_test_tx_report. */
	rtl_write_word(rtlpriv, REG_TX_RPT_TIME, 0x3DF0);

	/* Reset USB mode switch setting */
	rtl_write_byte(rtlpriv, REG_SDIO_CTRL_8812, 0x0);
	rtl_write_byte(rtlpriv, REG_ACLK_MON, 0x0);

	/*
	 * RT_TRACE(COMP_INIT, DBG_TRACE, ("InitializeAdapter8188EUsb() <====\n"));
	 */

	/* 2010/08/26 MH Merge from 8192CE. */
	if (pwrctrlpriv->rf_pwrstate == ERFON) {
		if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
			rtlphy->need_iqk = true;
			if (rtlphy->iqk_initialized)
				rtl8812au_phy_iq_calibrate(rtlpriv, true);
			else {
				rtl8812au_phy_iq_calibrate(rtlpriv, false);
				rtlphy->iqk_initialized = true;
			}
		}


		/* rtl8812au_phy_lc_calibrate(rtlpriv); */
	}

	/* HAL_INIT_PROFILE_TAG(HAL_INIT_STAGES_INIT_PABIAS);
	 * _InitPABias(rtlpriv);
	 */

	/*
	 *  2010/08/23 MH According to Alfred's suggestion, we need to to prevent HW enter
	 *  suspend mode automatically.
	 * HwSuspendModeEnable92Cu(rtlpriv, false);
	 */


	rtl_write_byte(rtlpriv, REG_USB_HRPWM, 0);

	/* misc */
	{
		/* ULLI reading MAC address again ?? */
		int i;
		uint8_t mac_addr[6];
		for (i = 0; i < 6; i++) {
			mac_addr[i] = rtl_read_byte(rtlpriv, REG_MACID+i);
		}

		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "MAC Address from REG_MACID = "MAC_FMT"\n", MAC_ARG(mac_addr));
	}

exit:

	RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "%s in %dms\n", __FUNCTION__, rtw_get_passing_time_ms(init_start_time));

	return status;
}

static void _rtl8812au_read_pa_type(struct rtl_priv *rtlpriv, uint8_t *hwinfo,
			     bool autoload_fail)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	if (!autoload_fail) {
		rtlhal->pa_type_2g = hwinfo[EEPROM_PA_TYPE_8812AU];
		rtlhal->lna_type_2g = hwinfo[EEPROM_LNA_TYPE_2G_8812AU];

		if (rtlhal->pa_type_2g == 0xFF && rtlhal->lna_type_2g == 0xFF) {
			rtlhal->pa_type_2g = 0;
			rtlhal->lna_type_2g = 0;
		}

		rtlhal->external_pa_2g = ((rtlhal->pa_type_2g & BIT(5)) &&
					  (rtlhal->pa_type_2g & BIT(4))) ? 1 : 0;
		rtlhal->external_lna_2g = ((rtlhal->lna_type_2g & BIT(7)) &&
					   (rtlhal->lna_type_2g & BIT(3))) ? 1 : 0;

		rtlhal->pa_type_5g = hwinfo[EEPROM_PA_TYPE_8812AU];
		rtlhal->lna_type_5g = hwinfo[EEPROM_LNA_TYPE_5G_8812AU];

		if (rtlhal->pa_type_5g == 0xFF && rtlhal->lna_type_5g == 0xFF) {
			rtlhal->pa_type_5g = 0;
			rtlhal->lna_type_5g = 0;
		}

		rtlhal->external_pa_5g = ((rtlhal->pa_type_5g & BIT(1)) &&
					  (rtlhal->pa_type_5g & BIT(0))) ? 1 : 0;
		rtlhal->external_lna_5g = ((rtlhal->lna_type_5g & BIT(7)) &&
					   (rtlhal->lna_type_5g & BIT(3))) ? 1 : 0;

	} else {
		rtlhal->external_pa_2g  = EEPROM_Default_PAType;
		rtlhal->external_pa_5g  = 0xFF;
		rtlhal->external_lna_2g = EEPROM_Default_LNAType;
		rtlhal->external_lna_5g = 0xFF;

		rtlhal->external_pa_2g  = 0;
		rtlhal->external_lna_2g = 0;

		rtlhal->external_pa_5g  = 0;
		rtlhal->external_lna_5g = 0;
	}
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->PAType_2G is 0x%x, pHalData->ExternalPA_2G = %d\n", rtlhal->pa_type_2g, rtlhal->external_pa_2g);
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->PAType_5G is 0x%x, pHalData->ExternalPA_5G = %d\n", rtlhal->pa_type_5g, rtlhal->external_pa_5g);
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->LNAType_2G is 0x%x, pHalData->ExternalLNA_2G = %d\n", rtlhal->lna_type_2g, rtlhal->external_lna_2g);
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->LNAType_5G is 0x%x, pHalData->ExternalLNA_5G = %d\n", rtlhal->lna_type_5g, rtlhal->external_lna_5g);
}

static void _rtl8821au_read_pa_type(struct rtl_priv *rtlpriv, u8 *hwinfo,
				    bool autoload_fail)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	if (!autoload_fail) {
		rtlhal->pa_type_2g = hwinfo[EEPROM_PA_TYPE_8812AU];
		rtlhal->lna_type_2g = hwinfo[EEPROM_LNA_TYPE_2G_8812AU];
		if (rtlhal->pa_type_2g == 0xFF && rtlhal->lna_type_2g == 0xFF) {
			rtlhal->pa_type_2g = 0;
			rtlhal->lna_type_2g = 0;
		}
		rtlhal->external_pa_2g = (rtlhal->pa_type_2g & BIT(4)) ? 1 : 0;
		rtlhal->external_lna_2g = (rtlhal->lna_type_2g & BIT(3)) ? 1 : 0;

		rtlhal->pa_type_5g = hwinfo[EEPROM_PA_TYPE_8812AU];
		rtlhal->lna_type_5g = hwinfo[EEPROM_LNA_TYPE_5G_8812AU];
		if (rtlhal->pa_type_5g == 0xFF && rtlhal->lna_type_5g == 0xFF) {
			rtlhal->pa_type_5g = 0;
			rtlhal->lna_type_5g = 0;
		}
		rtlhal->external_pa_5g = (rtlhal->pa_type_5g & BIT(0)) ? 1 : 0;
		rtlhal->external_lna_5g = (rtlhal->lna_type_5g & BIT(3)) ? 1 : 0;
	} else {
		rtlhal->external_pa_2g  = EEPROM_Default_PAType;
		rtlhal->external_pa_5g  = 0xFF;
		rtlhal->external_lna_2g = EEPROM_Default_LNAType;
		rtlhal->external_lna_5g = 0xFF;

		rtlhal->external_pa_2g  = 0;
		rtlhal->external_lna_2g = 0;

		rtlhal->external_pa_5g  = 0;
		rtlhal->external_lna_5g = 0;
	}
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->PAType_2G is 0x%x, pHalData->ExternalPA_2G = %d\n", rtlhal->pa_type_2g, rtlhal->external_pa_2g);
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->PAType_5G is 0x%x, pHalData->ExternalPA_5G = %d\n", rtlhal->pa_type_5g, rtlhal->external_pa_5g);
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->LNAType_2G is 0x%x, pHalData->ExternalLNA_2G = %d\n", rtlhal->lna_type_2g, rtlhal->external_lna_2g);
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "pHalData->LNAType_5G is 0x%x, pHalData->ExternalLNA_5G = %d\n", rtlhal->lna_type_5g, rtlhal->external_lna_5g);
}

static void _rtl8821au_read_power_value_fromprom(struct rtl_priv *rtlpriv,
	struct txpower_info_2g *pwrinfo24g,
	struct txpower_info_5g *pwrinfo5g,
	u8 *hwinfo,
	bool autoload_fail)
{
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);
	uint32_t rfPath, eeAddr = EEPROM_TX_PWR_INX_8812, group, TxCount = 0;

	memset(pwrinfo24g, 0, sizeof(*pwrinfo24g));
	memset(pwrinfo5g, 0, sizeof(*pwrinfo5g));

	/* DBG_871X("hal_ReadPowerValueFromPROM8812A(): PROMContent[0x%x]=0x%x\n", (eeAddr+1), PROMContent[eeAddr+1]); */
	if (0xFF == hwinfo[eeAddr+1])  /* YJ,add,120316 */
		autoload_fail = true;

	if (autoload_fail) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "hal_ReadPowerValueFromPROM8812A(): Use Default value!\n");
		for (rfPath = 0 ; rfPath < MAX_RF_PATH ; rfPath++) {
			/*  2.4G default value */
			for (group = 0 ; group < MAX_CHNL_GROUP_24G; group++) {
				pwrinfo24g->index_cck_base[rfPath][group] = EEPROM_DEFAULT_24G_INDEX;
				pwrinfo24g->index_bw40_base[rfPath][group] = EEPROM_DEFAULT_24G_INDEX;
			}
			for (TxCount = 0; TxCount < MAX_TX_COUNT; TxCount++) {
				if (TxCount == 0) {
					pwrinfo24g->bw20_diff[rfPath][0] = EEPROM_DEFAULT_24G_HT20_DIFF;
					pwrinfo24g->ofdm_diff[rfPath][0] = EEPROM_DEFAULT_24G_OFDM_DIFF;
				} else {
					pwrinfo24g->bw20_diff[rfPath][TxCount] = EEPROM_DEFAULT_DIFF;
					pwrinfo24g->bw40_diff[rfPath][TxCount] = EEPROM_DEFAULT_DIFF;
					pwrinfo24g->cck_diff[rfPath][TxCount] =	EEPROM_DEFAULT_DIFF;
					pwrinfo24g->ofdm_diff[rfPath][TxCount] = EEPROM_DEFAULT_DIFF;
				}
			}

			/* 5G default value */
			for (group = 0 ; group < MAX_CHNL_GROUP_5G; group++)
				pwrinfo5g->index_bw40_base[rfPath][group] = EEPROM_DEFAULT_5G_INDEX;

			for (TxCount = 0; TxCount < MAX_TX_COUNT; TxCount++) {
				if (TxCount == 0) {
					pwrinfo5g->ofdm_diff[rfPath][0] = EEPROM_DEFAULT_5G_OFDM_DIFF;
					pwrinfo5g->bw20_diff[rfPath][0] = EEPROM_DEFAULT_5G_HT20_DIFF;
					pwrinfo5g->bw80_diff[rfPath][0] = EEPROM_DEFAULT_DIFF;
					pwrinfo5g->bw160_diff[rfPath][0] = EEPROM_DEFAULT_DIFF;
				} else {
					/* ULLI check for-loop in _rtl8821ae_read_power_fromprom() */
					pwrinfo5g->ofdm_diff[rfPath][TxCount] =	EEPROM_DEFAULT_DIFF;
					pwrinfo5g->bw20_diff[rfPath][TxCount] =	EEPROM_DEFAULT_DIFF;
					pwrinfo5g->bw40_diff[rfPath][TxCount] =	EEPROM_DEFAULT_DIFF;
					pwrinfo5g->bw80_diff[rfPath][TxCount] =	EEPROM_DEFAULT_DIFF;
					pwrinfo5g->bw160_diff[rfPath][TxCount] = EEPROM_DEFAULT_DIFF;

				}
			}

		}

		/* pHalData->bNOPG = true; */
		return;
	}

	pHalData->bTXPowerDataReadFromEEPORM = true;		/* YJ,move,120316 */

	for (rfPath = 0; rfPath < MAX_RF_PATH; rfPath++) {
		/*  2.4G default value */
		for (group = 0; group < MAX_CHNL_GROUP_24G; group++) {
			pwrinfo24g->index_cck_base[rfPath][group] = hwinfo[eeAddr++];
			if (pwrinfo24g->index_cck_base[rfPath][group] == 0xFF) {
				pwrinfo24g->index_cck_base[rfPath][group] = EEPROM_DEFAULT_24G_INDEX;
				/* pHalData->bNOPG = true; */
			}
			/*
			 * DBG_871X("8812-2G RF-%d-G-%d CCK-Addr-%x BASE=%x\n",
			 * rfPath, group, eeAddr-1, pwrInfo24G->IndexCCK_Base[rfPath][group]);
			 */
		}

		for (group = 0; group < MAX_CHNL_GROUP_24G-1; group++) {
			pwrinfo24g->index_bw40_base[rfPath][group] = hwinfo[eeAddr++];
			if (pwrinfo24g->index_bw40_base[rfPath][group] == 0xFF)
				pwrinfo24g->index_bw40_base[rfPath][group] = EEPROM_DEFAULT_24G_INDEX;
			/*
			 * DBG_871X("8812-2G RF-%d-G-%d BW40-Addr-%x BASE=%x\n",
			 * rfPath, group, eeAddr-1, pwrInfo24G->IndexBW40_Base[rfPath][group]);
			 */
		}

		for (TxCount = 0; TxCount < MAX_TX_COUNT; TxCount++) {
			if (TxCount == 0) {
				pwrinfo24g->bw40_diff[rfPath][TxCount] = 0;

				pwrinfo24g->bw20_diff[rfPath][TxCount] = (hwinfo[eeAddr] & 0xf0) >> 4;
				if (pwrinfo24g->bw20_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo24g->bw20_diff[rfPath][TxCount] |= 0xF0;

				/*
				 * DBG_871X("8812-2G RF-%d-SS-%d BW20-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo24G->BW20_Diff[rfPath][TxCount]);
				 */

				pwrinfo24g->ofdm_diff[rfPath][TxCount] =	(hwinfo[eeAddr]&0x0f);
				if (pwrinfo24g->ofdm_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo24g->ofdm_diff[rfPath][TxCount] |= 0xF0;

				/*
				 * DBG_871X("8812-2G RF-%d-SS-%d LGOD-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo24G->OFDM_Diff[rfPath][TxCount]);
				 */

				pwrinfo24g->cck_diff[rfPath][TxCount] = 0;
				eeAddr++;
			} else {
				pwrinfo24g->bw40_diff[rfPath][TxCount] =	(hwinfo[eeAddr]&0xf0)>>4;
				if (pwrinfo24g->bw40_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo24g->bw40_diff[rfPath][TxCount] |= 0xF0;

				/*
				 * DBG_871X("8812-2G RF-%d-SS-%d BW40-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo24G->BW40_Diff[rfPath][TxCount]);
				 */

				pwrinfo24g->bw20_diff[rfPath][TxCount] =	(hwinfo[eeAddr]&0x0f);
				if (pwrinfo24g->bw20_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo24g->bw20_diff[rfPath][TxCount] |= 0xF0;

				/*
				 * DBG_871X("8812-2G RF-%d-SS-%d BW20-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo24G->BW20_Diff[rfPath][TxCount]);
				 */

				eeAddr++;


				pwrinfo24g->ofdm_diff[rfPath][TxCount] =	(hwinfo[eeAddr]&0xf0)>>4;
				if (pwrinfo24g->ofdm_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo24g->ofdm_diff[rfPath][TxCount] |= 0xF0;

				/*
				 * DBG_871X("8812-2G RF-%d-SS-%d LGOD-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo24G->BW20_Diff[rfPath][TxCount]);
				 */

				pwrinfo24g->cck_diff[rfPath][TxCount] =	(hwinfo[eeAddr]&0x0f);
				if (pwrinfo24g->cck_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo24g->cck_diff[rfPath][TxCount] |= 0xF0;
				/*
				 * DBG_871X("8812-2G RF-%d-SS-%d CCK-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo24G->CCK_Diff[rfPath][TxCount]);
				 */

				eeAddr++;
			}
		}

		/* 5G default value */
		for (group = 0 ; group < MAX_CHNL_GROUP_5G; group++) {
			pwrinfo5g->index_bw40_base[rfPath][group] =		hwinfo[eeAddr++];
			if (pwrinfo5g->index_bw40_base[rfPath][group] == 0xFF)
				pwrinfo5g->index_bw40_base[rfPath][group] = EEPROM_DEFAULT_DIFF;

			/*
			 * DBG_871X("8812-5G RF-%d-G-%d BW40-Addr-%x BASE=%x\n",
			 * rfPath, TxCount, eeAddr-1, pwrInfo5G->IndexBW40_Base[rfPath][group]);
			 */
		}

		for (TxCount = 0; TxCount < MAX_TX_COUNT; TxCount++) {
			if (TxCount == 0) {
				pwrinfo5g->bw40_diff[rfPath][TxCount] = 0;
				pwrinfo5g->bw20_diff[rfPath][0] = (hwinfo[eeAddr] & 0xf0) >> 4;
				if (pwrinfo5g->bw20_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo5g->bw20_diff[rfPath][TxCount] |= 0xF0;
				/*
				 * DBG_871X("8812-5G RF-%d-SS-%d BW20-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo5G->BW20_Diff[rfPath][TxCount]);
				 */
				pwrinfo5g->ofdm_diff[rfPath][0] = (hwinfo[eeAddr] & 0x0f);
				if (pwrinfo5g->ofdm_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo5g->ofdm_diff[rfPath][TxCount] |= 0xF0;
				/*
				 * DBG_871X("8812-5G RF-%d-SS-%d LGOD-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo5G->OFDM_Diff[rfPath][TxCount]);
				 */

				eeAddr++;
			} else {
				pwrinfo5g->bw40_diff[rfPath][TxCount] = (hwinfo[eeAddr] & 0xf0) >> 4;
				if (pwrinfo5g->bw40_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo5g->bw40_diff[rfPath][TxCount] |= 0xF0;
				/*
				 * DBG_871X("8812-5G RF-%d-SS-%d BW40-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo5G->BW40_Diff[rfPath][TxCount]);
				 */

				pwrinfo5g->bw20_diff[rfPath][TxCount] = (hwinfo[eeAddr] & 0x0f);
				if (pwrinfo5g->bw20_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
					pwrinfo5g->bw20_diff[rfPath][TxCount] |= 0xF0;
				/*
				 * DBG_871X("8812-5G RF-%d-SS-%d BW20-Addr-%x DIFF=%d\n",
				 * rfPath, TxCount, eeAddr, pwrInfo5G->BW20_Diff[rfPath][TxCount]);
				 */

				eeAddr++;
			}
		}


		pwrinfo5g->ofdm_diff[rfPath][1] =	(hwinfo[eeAddr] & 0xf0) >> 4;
		pwrinfo5g->ofdm_diff[rfPath][2] =	(hwinfo[eeAddr] & 0x0f);
		/*
		 * DBG_871X("8812-5G RF-%d-SS-%d LGOD-Addr-%x DIFF=%d\n",
		 * rfPath, 2, eeAddr, pwrInfo5G->OFDM_Diff[rfPath][2]);
		 */
		eeAddr++;

		pwrinfo5g->ofdm_diff[rfPath][3] =	(hwinfo[eeAddr] & 0x0f);
		/*
		 * DBG_871X("8812-5G RF-%d-SS-%d LGOD-Addr-%x DIFF=%d\n",
		 * rfPath, 3, eeAddr, pwrInfo5G->OFDM_Diff[rfPath][3]);
		 */
		eeAddr++;

		for (TxCount = 1; TxCount < MAX_TX_COUNT; TxCount++) {
			if (pwrinfo5g->ofdm_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
				pwrinfo5g->ofdm_diff[rfPath][TxCount] |= 0xF0;

			/*
			 * DBG_871X("8812-5G RF-%d-SS-%d LGOD-Addr-%x DIFF=%d\n",
			 * rfPath, TxCount, eeAddr, pwrInfo5G->OFDM_Diff[rfPath][TxCount]);
			 */
		}

		for (TxCount = 0; TxCount < MAX_TX_COUNT; TxCount++) {
			pwrinfo5g->bw80_diff[rfPath][TxCount] =	(hwinfo[eeAddr] & 0xf0) >> 4;
			if (pwrinfo5g->bw80_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
				pwrinfo5g->bw80_diff[rfPath][TxCount] |= 0xF0;
			/*
			 * DBG_871X("8812-5G RF-%d-SS-%d BW80-Addr-%x DIFF=%d\n",
			 * rfPath, TxCount, eeAddr, pwrInfo5G->BW80_Diff[rfPath][TxCount]);
			 */
			pwrinfo5g->bw160_diff[rfPath][TxCount] =	(hwinfo[eeAddr] & 0x0f);
			if (pwrinfo5g->bw160_diff[rfPath][TxCount] & BIT(3))		/* 4bit sign number to 8 bit sign number */
				pwrinfo5g->bw160_diff[rfPath][TxCount] |= 0xF0;
			/*
			 * DBG_871X("8812-5G RF-%d-SS-%d BW160-Addr-%x DIFF=%d\n",
			 * rfPath, TxCount, eeAddr, pwrInfo5G->BW160_Diff[rfPath][TxCount]);
			 */
			eeAddr++;
		}
	}

}

static u8 _rtl8821au_get_chnl_group(u8 chnl)
{
	u8 group = 0;

	if (chnl <= 14) {
		if (1 <= chnl && chnl <= 2)
			group = 0;
		else if (3  <= chnl && chnl <= 5)
			group = 1;
		else if (6  <= chnl && chnl <= 8)
			group = 2;
		else if (9  <= chnl && chnl <= 11)
			group = 3;
		else if (12 <= chnl && chnl <= 14)
			group = 4;
#if 0
		else {
			DBG_871X("==>mpt_GetChnlGroup8812A in 2.4 G, but chnl %d in Group not found \n", chnl);
		}
#endif
	} else {
		if      (36   <= chnl && chnl <=  42)
			group = 0;
		else if (44   <= chnl && chnl <=  48)
			group = 1;
		else if (50   <= chnl && chnl <=  58)
			group = 2;
		else if (60   <= chnl && chnl <=  64)
			group = 3;
		else if (100  <= chnl && chnl <= 106)
			group = 4;
		else if (108  <= chnl && chnl <= 114)
			group = 5;
		else if (116  <= chnl && chnl <= 122)
			group = 6;
		else if (124  <= chnl && chnl <= 130)
			group = 7;
		else if (132  <= chnl && chnl <= 138)
			group = 8;
		else if (140  <= chnl && chnl <= 144)
			group = 9;
		else if (149  <= chnl && chnl <= 155)
			group = 10;
		else if (157  <= chnl && chnl <= 161)
			group = 11;
		else if (165  <= chnl && chnl <= 171)
			group = 12;
		else if (173  <= chnl && chnl <= 177)
			group = 13;
#if 0
		else {
			DBG_871X("==>mpt_GetChnlGroup8812A in 5G, but chnl %d in Group not found \n", chnl);
		}
#endif

	}
	/* DBG_871X("<==mpt_GetChnlGroup8812A,  (%s) Channel = %d, Group =%d,\n", (bIn24G) ? "2.4G" : "5G", Channel, *pGroup); */

	return group;
}

static void _rtl88au_read_txpower_info_from_hwpg(struct rtl_priv *rtlpriv, u8 *hwinfo,
	bool autoload_fail)
{
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);
	struct txpower_info_2g pwrInfo24G;
	struct txpower_info_5g pwrInfo5G;
	uint8_t	rfPath, ch, group, TxCount;
	uint8_t	channel5G[CHANNEL_MAX_NUMBER_5G] = {
		 36,  38,  40,  42,  44,  46,  48,  50,  52,  54,  56,  58,
		 60,  62,  64, 100, 102, 104, 106, 108, 110, 112, 114, 116,
		118, 120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140,
		142, 144, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167,
		168, 169, 171, 173, 175, 177 };

	uint8_t	channel5G_80M[CHANNEL_MAX_NUMBER_5G_80M] = {
		42, 58, 106, 122, 138, 155, 171};

	_rtl8821au_read_power_value_fromprom(rtlpriv, &pwrInfo24G, &pwrInfo5G, hwinfo, autoload_fail);

	/*
	 * if(!AutoLoadFail)
	 * 	pHalData->bTXPowerDataReadFromEEPORM = true;
	 */

	for (rfPath = 0; rfPath < MAX_RF_PATH; rfPath++) {
		for (ch = 0 ; ch < CHANNEL_MAX_NUMBER_2G; ch++) {
			group = _rtl8821au_get_chnl_group(ch+1);

			if (ch == (CHANNEL_MAX_NUMBER_2G-1)) {
				efuse->txpwrlevel_cck[rfPath][ch] =
					pwrInfo24G.index_cck_base[rfPath][5];
				efuse->txpwrlevel_ht40_1s[rfPath][ch] =
					pwrInfo24G.index_bw40_base[rfPath][group];
			} else {
				efuse->txpwrlevel_cck[rfPath][ch] =
					pwrInfo24G.index_cck_base[rfPath][group];
				efuse->txpwrlevel_ht40_1s[rfPath][ch] =
					pwrInfo24G.index_bw40_base[rfPath][group];
			}

			/*
			 * DBG_871X("======= Path %d, ChannelIndex %d, Group %d=======\n",rfPath,ch, group);
			 * DBG_871X("Index24G_CCK_Base[%d][%d] = 0x%x\n",rfPath,ch ,pHalData->Index24G_CCK_Base[rfPath][ch]);
			 * DBG_871X("Index24G_BW40_Base[%d][%d] = 0x%x\n",rfPath,ch,pHalData->Index24G_BW40_Base[rfPath][ch]);
			 */
		}

		for (ch = 0 ; ch < CHANNEL_MAX_NUMBER_5G; ch++) {
			group = _rtl8821au_get_chnl_group(channel5G[ch]);

			efuse->txpwr_5g_bw40base[rfPath][ch] = pwrInfo5G.index_bw40_base[rfPath][group];

			/*
			 * DBG_871X("======= Path %d, ChannelIndex %d, Group %d=======\n",rfPath,ch, group);
			 * DBG_871X("Index5G_BW40_Base[%d][%d] = 0x%x\n",rfPath,ch,pHalData->Index5G_BW40_Base[rfPath][ch]);
			 */
		}
		for (ch = 0 ; ch < CHANNEL_MAX_NUMBER_5G_80M; ch++) {
			uint8_t	upper, lower;

			group = _rtl8821au_get_chnl_group(channel5G_80M[ch]);
			upper = pwrInfo5G.index_bw40_base[rfPath][group];
			lower = pwrInfo5G.index_bw40_base[rfPath][group+1];

			efuse->txpwr_5g_bw80base[rfPath][ch] = (upper + lower) / 2;

			/*
			 * DBG_871X("======= Path %d, ChannelIndex %d, Group %d=======\n",rfPath,ch, group);
			 * DBG_871X("Index5G_BW80_Base[%d][%d] = 0x%x\n",rfPath,ch,pHalData->Index5G_BW80_Base[rfPath][ch]);
			 */
		}

		for (TxCount = 0; TxCount < MAX_TX_COUNT; TxCount++) {
			efuse->txpwr_cckdiff[rfPath][TxCount]  = pwrInfo24G.cck_diff[rfPath][TxCount];
			efuse->txpwr_legacyhtdiff[rfPath][TxCount] = pwrInfo24G.ofdm_diff[rfPath][TxCount];
			efuse->txpwr_ht20diff[rfPath][TxCount] = pwrInfo24G.bw20_diff[rfPath][TxCount];
			efuse->txpwr_ht40diff[rfPath][TxCount] = pwrInfo24G.bw40_diff[rfPath][TxCount];

			efuse->txpwr_5g_ofdmdiff[rfPath][TxCount] = pwrInfo5G.ofdm_diff[rfPath][TxCount];
			efuse->txpwr_5g_bw20diff[rfPath][TxCount] = pwrInfo5G.bw20_diff[rfPath][TxCount];
			efuse->txpwr_5g_bw40diff[rfPath][TxCount] = pwrInfo5G.bw40_diff[rfPath][TxCount];
			efuse->txpwr_5g_bw80diff[rfPath][TxCount] = pwrInfo5G.bw80_diff[rfPath][TxCount];
/* #if DBG */
		}
	}


	/* 2010/10/19 MH Add Regulator recognize for CU. */
	if (!autoload_fail) {
		if (hwinfo[EEPROM_RF_BOARD_OPTION_8812] == 0xFF)
			efuse->eeprom_regulatory = (EEPROM_DEFAULT_BOARD_OPTION&0x7);	/* BIT(0)~2 */
		else
			efuse->eeprom_regulatory = (hwinfo[EEPROM_RF_BOARD_OPTION_8812]&0x7);	/* BIT(0)~2 */

	} else {
		efuse->eeprom_regulatory = 0;
	}
	RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "EEPROMRegulatory = 0x%x\n", efuse->eeprom_regulatory);

}
