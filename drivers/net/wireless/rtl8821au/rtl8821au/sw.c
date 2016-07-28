#ifdef CONFIG_RTLWIFI

#include <../drivers/net/wireless/realtek/rtlwifi/wifi.h>
#include <../drivers/net/wireless/realtek/rtlwifi/core.h>
#include <../drivers/net/wireless/realtek/rtlwifi/use.h>
#include <../drivers/net/wireless/realtek/rtlwifi/efuse.h>
#include <../drivers/net/wireless/realtek/rtlwifi/base.h>

#else

#include <drv_types.h>
#include "sw.h"
#include "dm.h"
#include "phy.h"
#include "reg.h"
#include "rf.h"
#include "trx.h"
#include "hw.h"
#include "led.h"
#include "fw.h"

#include <rtl8812a_hal.h>

#include "../debug.h"

#endif

#include <linux/spinlock.h>
#include <linux/module.h>

#define RTL8821AU_DRIVER_NAME		"rtl8821au"

void rtl8812_free_hal_data(struct rtl_priv *rtlpriv);

static uint8_t rtw_init_default_value(struct rtl_priv *rtlpriv)
{
	struct rtl_hal_cfg *cfg = rtlpriv->cfg;
	uint8_t ret  = _SUCCESS;
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;

	/* xmit_priv */
	pxmitpriv->vcs_setting = AUTO_VCS;
	pxmitpriv->vcs = RTS_CTS;
	pxmitpriv->vcs_type = RTS_CTS;
	/* pxmitpriv->rts_thresh = pregistrypriv->rts_thresh; */
	pxmitpriv->frag_len = 2346;

	/* recv_priv */

	/* mlme_priv */
	pmlmepriv->scan_interval = SCAN_INTERVAL;	/* 30*2 sec = 60sec */
	pmlmepriv->scan_mode = SCAN_ACTIVE;

	/*
	 * qos_priv
	 * pmlmepriv->qospriv.qos_option = pregistrypriv->wmm_enable;
	 */

	/* ht_priv */
	pmlmepriv->htpriv.ampdu_enable = false; /* set to disabled */

	/* security_priv */
	/* rtw_get_encrypt_decrypt_from_registrypriv(rtlpriv); */
	psecuritypriv->binstallGrpkey = _FAIL;
	psecuritypriv->sw_encrypt = cfg->mod_params->sw_crypto;
	psecuritypriv->sw_decrypt = cfg->mod_params->sw_crypto;

	psecuritypriv->dot11AuthAlgrthm = dot11AuthAlgrthm_Open; /* open system */
	psecuritypriv->dot11PrivacyAlgrthm = NO_ENCRYPTION;

	psecuritypriv->dot11PrivacyKeyIndex = 0;

	psecuritypriv->dot118021XGrpPrivacy = NO_ENCRYPTION;
	psecuritypriv->dot118021XGrpKeyid = 1;

	psecuritypriv->ndisauthtype = Ndis802_11AuthModeOpen;
	psecuritypriv->ndisencryptstatus = Ndis802_11WEPDisabled;


	/* pwrctrl_priv */


	/* registry_priv */
	rtw_init_registrypriv_dev_network(rtlpriv);
	rtw_update_registrypriv_dev_network(rtlpriv);


	/* hal_priv */
	rtlpriv->cfg->ops->init_default_value(rtlpriv);

	/* misc. */
	rtlpriv->bReadPortCancel = false;
	rtlpriv->bWritePortCancel = false;
	rtlpriv->bNotifyChannelChange = 0;

	return ret;
}

/* ULLI : temporaly used here, to get right vars for init_sw_vars */

void rtw_vht_use_default_setting(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct vht_priv *pvhtpriv = &pmlmepriv->vhtpriv;
	struct registry_priv *pregistrypriv = &rtlpriv->registrypriv;
	bool	bHwLDPCSupport = false, bHwSTBCSupport = false;
	uint8_t	rf_type = 0;

	pvhtpriv->vht_bwmode = (0x21 & 0xF0) >> 4;
	if (pvhtpriv->vht_bwmode > CHANNEL_WIDTH_80)
		pvhtpriv->sgi = TEST_FLAG(0xf, BIT(3)) ? true : false;
	else
		pvhtpriv->sgi = TEST_FLAG(0xf, BIT(2)) ? true : false;

	/*
	 * LDPC support
	 */

	if (IS_VENDOR_8812A_C_CUT(rtlhal->version))
		bHwLDPCSupport = true;
	else if (IS_HARDWARE_TYPE_8821(rtlhal))
		bHwLDPCSupport = false;
	else
		bHwLDPCSupport = false;


	CLEAR_FLAGS(pvhtpriv->ldpc_cap);

	if (bHwLDPCSupport) {
		if (TEST_FLAG(0x33, BIT(0)))
			SET_FLAG(pvhtpriv->ldpc_cap, LDPC_VHT_ENABLE_RX);
		if (TEST_FLAG(0x33, BIT(1)))
			SET_FLAG(pvhtpriv->ldpc_cap, LDPC_VHT_ENABLE_TX);
#if 0
		DBG_871X("[VHT] Support LDPC = 0x%02X\n", pvhtpriv->ldpc_cap);
#endif
	}

	/*
	 *  STBC
	 */

	if (rtlpriv->phy.rf_type == RF_2T2R)
		bHwSTBCSupport = 1;
	else
		bHwSTBCSupport = 0;

	CLEAR_FLAGS(pvhtpriv->stbc_cap);

	if (bHwSTBCSupport) {
		if (TEST_FLAG(0x3, BIT(1)))
			SET_FLAG(pvhtpriv->stbc_cap, STBC_VHT_ENABLE_TX);
	}

	bHwSTBCSupport = 1;

	if (bHwSTBCSupport) {
		if (TEST_FLAG(0x3, BIT(0)))
			SET_FLAG(pvhtpriv->stbc_cap, STBC_VHT_ENABLE_RX);
	}

#if 0
	DBG_871X("[VHT] Support STBC = 0x%02X\n", pvhtpriv->stbc_cap);
#endif

	pvhtpriv->ampdu_len = 7;

	rf_type = rtlpriv->phy.rf_type;

	if (rf_type == RF_1T1R)
		pvhtpriv->vht_mcs_map[0] = 0xfe;	/* Only support 1SS MCS 0~9; */
	else
		pvhtpriv->vht_mcs_map[0] = 0xfa;	/* support 1SS MCS 0~9 2SS MCS 0~9 */
	pvhtpriv->vht_mcs_map[1] = 0xff;

	pvhtpriv->vht_highest_rate = rtw_get_vht_highest_rate(rtlpriv, pvhtpriv->vht_mcs_map);
}

#if 0
static int rtw_ampdu_factor = 7;
static int rtw_vht_rate_sel = 0;
/*
 *
 *  BIT(0): Enable VHT LDPC Rx,
 *  BIT(1): Enable VHT LDPC Tx,
 *  BIT(4): Enable HT LDPC Rx,
 *  BIT(5): Enable HT LDPC Tx
 */
static int rtw_ldpc_cap = 0x33;
/*  BIT(0): Enable VHT STBC Rx,
 *  BIT(1): Enable VHT STBC Tx,
 *  BIT(4): Enable HT STBC Rx,
 *  BIT(5): Enable HT STBC Tx
 */
static int rtw_stbc_cap = 0x3;
/*
 * BIT(0): Enable VHT Beamformer,
 * BIT(1): Enable VHT Beamformee,
 * BIT(4): Enable HT Beamformer,
 * BIT(5): Enable HT Beamformee
 */
static int rtw_beamform_cap = 0;
#endif



void rtl8812au_init_default_value(struct rtl_priv *rtlpriv)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);
	struct rtl_phy *rtlphy = &(rtlpriv->phy);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct _rtw_hal *pHalData;
	struct pwrctrl_priv *pwrctrlpriv;
	struct dm_priv *pdmpriv;

	pHalData = GET_HAL_DATA(rtlpriv);
	pwrctrlpriv = &rtlpriv->pwrctrlpriv;
	pdmpriv = &pHalData->dmpriv;


	/* init default value */
	pHalData->fw_ractrl = false;
	rtlhal->last_hmeboxnum = 0;

	/* init dm default value */
	pHalData->bChnlBWInitialzed = false;
	rtlphy->iqk_initialized = false;
	rtldm->tm_trigger = 0;/* for IQK */
	rtlphy->pwrgroup_cnt = 0;

	rtlusb->irq_mask[0]	= (u32)(	\
/*
					IMR_ROK 		|
					IMR_RDU		|
					IMR_VODOK		|
					IMR_VIDOK		|
					IMR_BEDOK		|
					IMR_BKDOK		|
					IMR_MGNTDOK		|
					IMR_HIGHDOK		|
					IMR_CPWM		|
					IMR_CPWM2		|
					IMR_C2HCMD		|
					IMR_HISR1_IND_INT	|
					IMR_ATIMEND		|
					IMR_BCNDMAINT_E	|
					IMR_HSISR_IND_ON_INT	|
					IMR_BCNDOK0		|
					IMR_BCNDMAINT0	|
					IMR_TSF_BIT32_TOGGLE	|
					IMR_TXBCN0OK	|
					IMR_TXBCN0ERR	|
					IMR_GTINT3		|
					IMR_GTINT4		|
					IMR_TXCCK		|
 */
					0);

	rtlusb->irq_mask[1] 	= (u32)(	\
/*
					IMR_RXFOVW		|
					IMR_TXFOVW		|
					IMR_RXERR		|
					IMR_TXERR		|
					IMR_ATIMEND_E	|
					IMR_BCNDOK1		|
					IMR_BCNDOK2		|
					IMR_BCNDOK3		|
					IMR_BCNDOK4		|
					IMR_BCNDOK5		|
					IMR_BCNDOK6		|
					IMR_BCNDOK7		|
					IMR_BCNDMAINT1	|
					IMR_BCNDMAINT2	|
					IMR_BCNDMAINT3	|
					IMR_BCNDMAINT4	|
					IMR_BCNDMAINT5	|
					IMR_BCNDMAINT6	|
					IMR_BCNDMAINT7	|
 */
					0);
}



static int rtl8821au_init_sw_vars(struct rtl_priv *rtlpriv)
{
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	uint8_t	ret8 = _SUCCESS;

	/* for debug level */
	rtlpriv->dbg.global_debuglevel = rtlpriv->cfg->mod_params->debug;

	/* ULLI : needed for rtlwifi-lib to init core , from rtl8821ae */

	mac->ht_enable = true;
	mac->ht_cur_stbc = 0;
	mac->ht_stbc_cap = 0;
	mac->vht_cur_ldpc = 0;
	mac->vht_ldpc_cap = 0;
	mac->vht_cur_stbc = 0;
	mac->vht_stbc_cap = 0;

	rtlpriv->rtlhal.current_bandtype = BAND_ON_2_4G;
	rtlpriv->rtlhal.bandset = BAND_ON_BOTH;

	/* ULLI : Border for old init_sw_vars */

	ret8 = rtw_init_default_value(rtlpriv);

	if ((rtw_init_cmd_priv(&rtlpriv->cmdpriv)) == _FAIL) {
		ret8 = _FAIL;
		goto exit;
	}

	rtlpriv->cmdpriv.rtlpriv = rtlpriv;

	if ((rtw_init_evt_priv(&rtlpriv->evtpriv)) == _FAIL) {
		ret8 = _FAIL;
		goto exit;
	}


	if (rtw_init_mlme_priv(rtlpriv) == _FAIL) {
		ret8 = _FAIL;
		goto exit;
	}

	if (init_mlme_ext_priv(rtlpriv) == _FAIL) {
		ret8 = _FAIL;
		goto exit;
	}

	if (_rtw_init_xmit_priv(&rtlpriv->xmitpriv, rtlpriv) == _FAIL) 	{
		printk("rtl8821au: Can't _rtw_init_xmit_priv\n");
		ret8 = _FAIL;
		goto exit;
	}

	if (_rtw_init_recv_priv(&rtlpriv->recvpriv, rtlpriv) == _FAIL) {
		printk("rtl8821au: Can't _rtw_init_recv_priv\n");
		ret8 = _FAIL;
		goto exit;
	}

	/*
	 * We don't need to memset rtlpriv->XXX to zero, because rtlpriv is allocated by rtw_zvmalloc().
	 * memset((unsigned char *)&rtlpriv->securitypriv, 0, sizeof (struct security_priv));
	 */

	/* _init_timer(&(rtlpriv->securitypriv.tkip_timer), rtlpriv->pifp, rtw_use_tkipkey_handler, rtlpriv); */

	if (_rtw_init_sta_priv(&rtlpriv->stapriv) == _FAIL) {
		printk("rtl8821au: Can't _rtw_init_sta_priv\n");
		ret8 = _FAIL;
		goto exit;
	}

	rtlpriv->stapriv.rtlpriv = rtlpriv;
	rtlpriv->setband = GHZ24_50;
	rtw_init_bcmc_stainfo(rtlpriv);

	rtw_init_pwrctrl_priv(rtlpriv);

	/* memset((uint8_t *)&rtlpriv->qospriv, 0, sizeof (struct qos_priv));//move to mlme_priv */


	rtlpriv->cfg->ops->dm_init(rtlpriv);
	rtlpriv->cfg->ops->init_sw_leds(rtlpriv);

exit:

	return ret8;

}

static void rtl8821au_deinit_sw_vars(struct rtl_priv *rtlpriv)
{
	kfree(rtlpriv->rtlhal.pfirmware);
	rtlpriv->rtlhal.pfirmware = NULL;
}

/* get bt coexist status */
static bool rtl8821au_get_btc_status(void)
{
	return false;
}

static struct rtl_hal_ops rtl8821au_hal_ops = {
	/*
	 * New HAL functions with struct net_device  as first param
	 * this can be (hopefully)switched to struct ieee80211_hw
	 */
	 
	 /* ULLI : API table stolen form rtl8192cu */

	.init_sw_vars	= rtl8821au_init_sw_vars,
	.deinit_sw_vars	= rtl8821au_deinit_sw_vars,

	/* ULLI : must check API here */
	.read_chip_version =	rtl8821au_read_chip_version,
	/* ULLI : somewhere in the source, nanme ?? ... */

#if 0	/* ULLI : _rtl8821au_read_adapter_info(), maybe */
	.read_eeprom_info = rtl8821au_read_eeprom_info,
#endif

	.enable_interrupt = rtl8821au_enable_interrupt,
	.disable_interrupt = rtl8821au_disable_interrupt,

	.hw_init =	 	rtl8812au_hw_init,

#if 0	
	.hw_disable = rtl8821au_card_disable,
#endif	

	.set_network_type = rtl8821au_set_network_type,

#if 0
	.set_chk_bssid = rtl8821au_set_check_bssid,
	.set_qos = rtl8821au_set_qos,
	.set_bcn_reg = rtl8821au_set_beacon_related_registers,
	.set_bcn_intv = rtl8821au_set_beacon_interval,
	.update_interrupt_mask = rtl8821_update_interrupt_mask,
#endif	

	/* ULLI : check API */
	.set_hw_reg =	 	rtl8821au_set_hw_reg,
	.get_hw_reg = 		rtl8821au_get_hw_reg,
	
#if 0	/* ULLI : currently not known */
	.update_rate_tbl = rtl8821_update_hal_rate_tbl,
#endif	

#if 0	/* ULLI : somewhere in the tx path mess */
	.fill_tx_desc = rtl8821au_tx_fill_desc,
#endif
	/* ULLI : maybe wrong API */
	.fill_fake_txdesc	= rtl8821au_fill_fake_txdesc,
	
#if 0	/* ULLI : somewhere in the tx path mess */
	.fill_tx_cmdesc = rtl8821au_tx_fill_cmddesc,
#endif

#if 0	/* ULLI : somewhere in the rx path mess */
	.query_rx_desc = rtl8821au_rx_query_desc,
#endif

#if 0	/* ULLI : currently not known */
	.set_channel_access = rtl8821au_update_channel_access_setting,
#endif
	/* ULLI : check params */
	.radio_onoff_checking = rtl8821au_gpio_radio_on_off_checking,

#if 0	/* ULLI : maybe in old HAL ops */
	.set_bw_mode = rtl8821au_phy_set_bw_mode,
	.switch_channel = rtl8821au_phy_sw_chanl,
#endif
	.dm_watchdog = rtl8821au_dm_watchdog,

#if 0
	/* ULLI : rtlwifi function here ? */
	.scan_operation_backup = rtl_phy_scan_operation_backup,
#endif

#if 0	/* ULLI : currently not known */
	.set_rf_power_state = rtl8821au_phy_set_rf_power_state,
#endif

	.led_control	= rtl8821au_led_control,

#if 0	/* ULLI : hmm, see in cam.c of rtlwifi ?? */
	.enable_hw_sec = rtl8821au_enable_hw_security_setup,
	.set_key = rtl8821au_set_key,
#endif

	.init_sw_leds = rtl8821au_init_sw_leds,
	.deinit_sw_leds = rtl8821au_deinit_sw_leds,

	.get_bbreg = 	rtl8821au_phy_query_bb_reg,
	.set_bbreg = 	rtl8821au_phy_set_bb_reg,
	.get_rfreg = 	rtl8821au_phy_query_rf_reg,
	.set_rfreg = 	rtl8821au_phy_set_rf_reg,

	/* ULLI : need to check params herer */
	.phy_rf6052_config = rtl8821au_phy_rf6052_config,
	.phy_rf6052_set_cck_txpower = rtl8821au_phy_rf6052_set_cck_txpower,
	.phy_rf6052_set_ofdm_txpower = rtl8821au_phy_rf6052_set_ofdm_txpower,
	.config_bb_with_headerfile = _rtl8821au_phy_config_bb_with_headerfile,
	.config_bb_with_pgheaderfile = _rtl8821au_phy_config_bb_with_pgheaderfile,
#if 0
	.phy_lc_calibrate = rtl8821au_phy_lc_calibrate,
#endif
	.phy_set_bw_mode_callback = rtl8821au_phy_set_bw_mode_callback,
#if 0
	.dm_dynamic_txpower = rtl8821au_dm_dynamic_txpower,
#endif
	.fill_h2c_cmd = rtl8821au_fill_h2c_cmd,
	.get_btc_status = rtl8821au_get_btc_status,
	.set_bcn_reg = 	rtl8821au_set_beacon_related_registers,

	/** ** */

	/* Old HAL functions */

	.hal_deinit = 		rtl8812au_hal_deinit,

	/* .free_hal_data = rtl8192c_free_hal_data, */

	.inirp_deinit =		rtl8812au_inirp_deinit,

	.init_recv_priv =	rtl8812au_init_recv_priv,
	.free_recv_priv =	rtl8812au_free_recv_priv,
	.init_default_value =	rtl8812au_init_default_value,
	.read_adapter_info =	_rtl8821au_read_adapter_info,

	/* .set_bwmode_handler = 	PHY_SetBWMode8192C; */
	/* .set_channel_handler = 	PHY_SwChnl8192C; */

	/* .Add_RateATid = &rtl8192c_Add_RateATid, */

	.hal_xmit = 		rtl8812au_hal_xmit,
	.mgnt_xmit = 		rtl8812au_mgnt_xmit,
	.hal_xmitframe_enqueue = 	rtl8812au_hal_xmitframe_enqueue,

	.free_hal_data =	rtl8812_free_hal_data,

	.dm_init =		rtl8812_init_dm_priv,


	.set_bwmode_handler =	PHY_SetBWMode8812,
	.set_channel_handler =	PHY_SwChnl8812,
	.set_chnl_bw_handler =	PHY_SetSwChnlBWMode8812,

	.Add_RateATid =		rtl8812_Add_RateATid,
};

static struct rtl_hal_usbint_cfg rtl8821au_interface_cfg = {
#if 0
	/* rx */
	.in_ep_num = RTL92C_USB_BULK_IN_NUM,
	.rx_urb_num = RTL92C_NUM_RX_URBS,
	.rx_max_size = RTL92C_SIZE_MAX_RX_BUFFER,
	.usb_rx_hdl = rtl8192cu_rx_hdl,
	.usb_rx_segregate_hdl = NULL, /* rtl8192c_rx_segregate_hdl; */
	/* tx */
	.usb_tx_cleanup = rtl8192c_tx_cleanup,
	.usb_tx_post_hdl = rtl8192c_tx_post_hdl,
	.usb_tx_aggregate_hdl = rtl8192c_tx_aggregate_hdl,
	/* endpoint mapping */
#endif
	.usb_endpoint_mapping = rtl8821au_endpoint_mapping,
#if 0
	.usb_mq_to_hwq = rtl8192cu_mq_to_hwq,
#endif	
};

/* ULLI : shamelessly copied from rtlwifi */

static struct rtl_mod_params rtl8821au_mod_params = {
	.sw_crypto = false,
	.inactiveps = true,
	.swctrl_lps = false,
	.fwctrl_lps = true,
	.msi_support = true,
	.debug = DBG_EMERG,
	.disable_watchdog = 0,
};

static struct rtl_hal_cfg rtl8821au_hal_cfg = {
	.name = "rtl8821au",
	.fw_name = "rtlwifi/rtl8821aufw.bin",	/* ULLI note two files */
	.ops = &rtl8821au_hal_ops,
	.mod_params = &rtl8821au_mod_params,
	.usb_interface_cfg = &rtl8821au_interface_cfg,

	.maps[SYS_ISO_CTRL] = REG_SYS_ISO_CTRL,
#if 0
	.maps[SYS_FUNC_EN] = REG_SYS_FUNC_EN,
	.maps[SYS_CLK] = REG_SYS_CLKR,
	.maps[MAC_RCR_AM] = AM,
	.maps[MAC_RCR_AB] = AB,
	.maps[MAC_RCR_ACRC32] = ACRC32,
	.maps[MAC_RCR_ACF] = ACF,
	.maps[MAC_RCR_AAP] = AAP,
#endif
	.maps[EFUSE_TEST] = REG_EFUSE_TEST,
	.maps[EFUSE_CTRL] = REG_EFUSE_CTRL,
	.maps[EFUSE_CLK] = 0,
	.maps[EFUSE_CLK_CTRL] = REG_EFUSE_CTRL,
	.maps[EFUSE_PWC_EV12V] = PWC_EV12V,
	.maps[EFUSE_FEN_ELDR] = FEN_ELDR,
	.maps[EFUSE_LOADER_CLK_EN] = LOADER_CLK_EN,
	.maps[EFUSE_ANA8M] = ANA8M,
	.maps[EFUSE_HWSET_MAX_SIZE] = EFUSE_MAP_LEN_JAGUAR,	/* ULLI : or HWSET_MAX_SIZE */
	.maps[EFUSE_MAX_SECTION_MAP] = EFUSE_MAX_SECTION_JAGUAR,	/* ULLI : or EFUSE_MAX_SECTION */
	.maps[EFUSE_REAL_CONTENT_SIZE] =  EFUSE_REAL_CONTENT_LEN_JAGUAR,	/* ULLI : EFUSE_REAL_CONTENT_LEN */
	.maps[EFUSE_OOB_PROTECT_BYTES_LEN] = EFUSE_OOB_PROTECT_BYTES_JAGUAR,	/* ULLI : EFUSE_OOB_PROTECT_BYTES */
	.maps[EFUSE_ACCESS] = REG_EFUSE_BURN_GNT_8812, /* ULLI : or REG_EFUSE_ACCESS as in rtlwifi */

	.maps[RWCAM] = REG_CAMCMD,
	.maps[WCAMI] = REG_CAMWRITE,
	.maps[RCAMO] = REG_CAMREAD,
	.maps[CAMDBG] = REG_CAMDBG,
	.maps[SECR] = REG_SECCFG,
	.maps[SEC_CAM_NONE] = CAM_NONE,
	.maps[SEC_CAM_WEP40] = CAM_WEP40,
	.maps[SEC_CAM_TKIP] = CAM_TKIP,
	.maps[SEC_CAM_AES] = CAM_AES,
	.maps[SEC_CAM_WEP104] = CAM_WEP104,
#if 0
	.maps[RTL_IMR_BCNDMAINT6] = IMR_BCNDMAINT6,
	.maps[RTL_IMR_BCNDMAINT5] = IMR_BCNDMAINT5,
	.maps[RTL_IMR_BCNDMAINT4] = IMR_BCNDMAINT4,
	.maps[RTL_IMR_BCNDMAINT3] = IMR_BCNDMAINT3,
	.maps[RTL_IMR_BCNDMAINT2] = IMR_BCNDMAINT2,
	.maps[RTL_IMR_BCNDMAINT1] = IMR_BCNDMAINT1,
/*	.maps[RTL_IMR_BCNDOK8] = IMR_BCNDOK8,     */   /*need check*/
	.maps[RTL_IMR_BCNDOK7] = IMR_BCNDOK7,
	.maps[RTL_IMR_BCNDOK6] = IMR_BCNDOK6,
	.maps[RTL_IMR_BCNDOK5] = IMR_BCNDOK5,
	.maps[RTL_IMR_BCNDOK4] = IMR_BCNDOK4,
	.maps[RTL_IMR_BCNDOK3] = IMR_BCNDOK3,
	.maps[RTL_IMR_BCNDOK2] = IMR_BCNDOK2,
	.maps[RTL_IMR_BCNDOK1] = IMR_BCNDOK1,
/*	.maps[RTL_IMR_TIMEOUT2] = IMR_TIMEOUT2,*/
/*	.maps[RTL_IMR_TIMEOUT1] = IMR_TIMEOUT1,*/

	.maps[RTL_IMR_TXFOVW] = IMR_TXFOVW,
	.maps[RTL_IMR_PSTIMEOUT] = IMR_PSTIMEOUT,
	.maps[RTL_IMR_BCNINT] = IMR_BCNDMAINT0,
	.maps[RTL_IMR_RXFOVW] = IMR_RXFOVW,
	.maps[RTL_IMR_RDU] = IMR_RDU,
	.maps[RTL_IMR_ATIMEND] = IMR_ATIMEND,
	.maps[RTL_IMR_BDOK] = IMR_BCNDOK0,
	.maps[RTL_IMR_MGNTDOK] = IMR_MGNTDOK,
	.maps[RTL_IMR_TBDER] = IMR_TBDER,
	.maps[RTL_IMR_HIGHDOK] = IMR_HIGHDOK,
	.maps[RTL_IMR_TBDOK] = IMR_TBDOK,
	.maps[RTL_IMR_BKDOK] = IMR_BKDOK,
	.maps[RTL_IMR_BEDOK] = IMR_BEDOK,
	.maps[RTL_IMR_VIDOK] = IMR_VIDOK,
	.maps[RTL_IMR_VODOK] = IMR_VODOK,
	.maps[RTL_IMR_ROK] = IMR_ROK,
	.maps[RTL_IBSS_INT_MASKS] = (IMR_BCNDMAINT0 | IMR_TBDOK | IMR_TBDER),
#endif
	/* Ths is for the core of rtlwifi */
	.maps[RTL_RC_CCK_RATE1M] = DESC_RATE1M,
	.maps[RTL_RC_CCK_RATE2M] =  DESC_RATE2M,
	.maps[RTL_RC_CCK_RATE5_5M] =  DESC_RATE5_5M,
	.maps[RTL_RC_CCK_RATE11M] =  DESC_RATE11M,
	.maps[RTL_RC_OFDM_RATE6M] =  DESC_RATE6M,
	.maps[RTL_RC_OFDM_RATE9M] =  DESC_RATE9M,
	.maps[RTL_RC_OFDM_RATE12M] =  DESC_RATE12M,
	.maps[RTL_RC_OFDM_RATE18M] =  DESC_RATE18M,
	.maps[RTL_RC_OFDM_RATE24M] =  DESC_RATE24M,
	.maps[RTL_RC_OFDM_RATE36M] =  DESC_RATE36M,
	.maps[RTL_RC_OFDM_RATE48M] =  DESC_RATE48M,
	.maps[RTL_RC_OFDM_RATE54M] =  DESC_RATE54M,

	.maps[RTL_RC_HT_RATEMCS7] =  DESC_RATEMCS7,
	.maps[RTL_RC_HT_RATEMCS15] =  DESC_RATEMCS15,

	/*VHT hightest rate*/
	.maps[RTL_RC_VHT_RATE_1SS_MCS7] = DESC_RATEVHT1SS_MCS7,
	.maps[RTL_RC_VHT_RATE_1SS_MCS8] = DESC_RATEVHT1SS_MCS8,
	.maps[RTL_RC_VHT_RATE_1SS_MCS9] = DESC_RATEVHT1SS_MCS9,
	
	/* ULLI : VHT rates for 2SS are decided in base.c of rtlwifi */
	
	.maps[RTL_RC_VHT_RATE_2SS_MCS7] = DESC_RATEVHT2SS_MCS7,
	.maps[RTL_RC_VHT_RATE_2SS_MCS8] = DESC_RATEVHT2SS_MCS8,
	.maps[RTL_RC_VHT_RATE_2SS_MCS9] = DESC_RATEVHT2SS_MCS9,
};



extern int rtw_bw_mode;

static int rtw_drv_init(struct usb_interface *pusb_intf,const struct usb_device_id *pdid);
static void rtw_dev_remove(struct usb_interface *pusb_intf);

#define USB_VENDER_ID_REALTEK		0x0BDA


/* DID_USB_v916_20130116 */
static struct usb_device_id rtw_usb_id_tbl[] ={
	/* RTL8812AU */
	/*=== Realtek demoboard ===*/
	{USB_DEVICE(USB_VENDER_ID_REALTEK, 0x8812),.driver_info = RTL8812},/* Default ID */
	{USB_DEVICE(USB_VENDER_ID_REALTEK, 0x881A),.driver_info = RTL8812},/* Default ID */
	{USB_DEVICE(USB_VENDER_ID_REALTEK, 0x881B),.driver_info = RTL8812},/* Default ID */
	{USB_DEVICE(USB_VENDER_ID_REALTEK, 0x881C),.driver_info = RTL8812},/* Default ID */
	/*=== Customer ID ===*/
	{USB_DEVICE(0x050D, 0x1109),.driver_info = RTL8812}, /* Belkin F9L1109 - SerComm */
	{USB_DEVICE(0x2001, 0x330E),.driver_info = RTL8812}, /* D-Link - ALPHA */
	{USB_DEVICE(0x7392, 0xA822),.driver_info = RTL8812}, /* Edimax - Edimax */
	{USB_DEVICE(0x0DF6, 0x0074),.driver_info = RTL8812}, /* Sitecom - Edimax */
	{USB_DEVICE(0x04BB, 0x0952),.driver_info = RTL8812}, /* I-O DATA - Edimax */
	{USB_DEVICE(0x0789, 0x016E),.driver_info = RTL8812}, /* Logitec - Edimax */
	{USB_DEVICE(0x0409, 0x0408),.driver_info = RTL8812}, /* NEC - */
	{USB_DEVICE(0x0B05, 0x17D2),.driver_info = RTL8812}, /* ASUS - Edimax */
	{USB_DEVICE(0x0E66, 0x0022),.driver_info = RTL8812}, /* HAWKING - Edimax */
	{USB_DEVICE(0x0586, 0x3426),.driver_info = RTL8812}, /* ZyXEL - */
	{USB_DEVICE(0x2001, 0x3313),.driver_info = RTL8812}, /* D-Link - ALPHA */
	{USB_DEVICE(0x1058, 0x0632),.driver_info = RTL8812}, /* WD - Cybertan*/
	{USB_DEVICE(0x1740, 0x0100),.driver_info = RTL8812}, /* EnGenius - EnGenius */
	{USB_DEVICE(0x2019, 0xAB30),.driver_info = RTL8812}, /* Planex - Abocom */
	{USB_DEVICE(0x07B8, 0x8812),.driver_info = RTL8812}, /* Abocom - Abocom */
	{USB_DEVICE(0x2001, 0x3315),.driver_info = RTL8812}, /* D-Link - Cameo */
	{USB_DEVICE(0x2001, 0x3316),.driver_info = RTL8812}, /* D-Link - Cameo */
	{USB_DEVICE(0x20F4, 0x805B),.driver_info = RTL8812}, /* TRENDnet - Cameo */
	{USB_DEVICE(0x13B1, 0x003F),.driver_info = RTL8812}, /* Linksys - SerComm */
	{USB_DEVICE(0x2357, 0x0101),.driver_info = RTL8812}, /* TP-Link - T4U */
	{USB_DEVICE(0x0BDA, 0x8812),.driver_info = RTL8812}, /* Alfa AWUS036AC */

	/* RTL8821AU */
        /*=== Realtek demoboard ===*/
	{USB_DEVICE(USB_VENDER_ID_REALTEK, 0x0811),.driver_info = RTL8821},/* Default ID */
	{USB_DEVICE(USB_VENDER_ID_REALTEK, 0x0821),.driver_info = RTL8821},/* Default ID */
	{USB_DEVICE(USB_VENDER_ID_REALTEK, 0x8822),.driver_info = RTL8821},/* Default ID */
	/*=== Customer ID ===*/
	{USB_DEVICE(0x7392, 0xA811),.driver_info = RTL8821}, /* Edimax - Edimax */
	{USB_DEVICE(0x0BDA, 0xA811),.driver_info = RTL8821}, /* OUTLINK - Edimax */
	{USB_DEVICE(0x7392, 0xA812),.driver_info = RTL8821}, /* Edimax - Edimax */
	{USB_DEVICE(0x2001, 0x3314),.driver_info = RTL8821}, /* D-Link - Cameo */
	{USB_DEVICE(0x0846, 0x9052),.driver_info = RTL8821}, /* Netgear - A6100 */
	{USB_DEVICE(0x0411, 0x0242),.driver_info = RTL8821}, /* BUFFALO - Edimax */
	{}	/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, rtw_usb_id_tbl);

/*
 *
 * ULLI : We can only track rtl8821au devices via MAC Addr.
 * ULLI : For every device we need an entry for MAC, USB Speed index and
 * ULLI : last seen
 *
 * ULLI : Note about this hack ...
 * ULLI : switch UBS3 capable device to superspeed mode
 * ULLI : Note about device behind a USB2 HUB and/or User (re)plugs device
 * ULLI : we need a simple byteswise read/write to track the device.
 * ULLI : This must be done before registering this device as ethernet or
 * ULLI : ieee80211 hw.
 *
 */

struct rtl8821au_probed_devices {
	struct list_head list;
	uint8_t mac[ETH_ALEN];
	int speed;
	unsigned long jiffies;
};

static LIST_HEAD(rtl8821_devices);
spinlock_t rtl8821au_devices_lock;

static int rtl8821au_probe(struct usb_interface *pusb_intf, const struct usb_device_id *pdid)
{
	struct usb_device *udev;
	char *speed;
	char *driver_info;

	udev = interface_to_usbdev(pusb_intf);
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

	switch (pdid->driver_info) {
		case RTL8812 :	driver_info ="RTL8812";
				break;

		case RTL8821 :	driver_info ="RTL8821";
				break;
	}

#if 0
	dev_info(&pusb_intf->dev, "Register : %s USB-ID %04x:%04x as %s Speed\n",
		driver_info, pdid->idVendor, pdid->idProduct, speed);
#endif
	return rtw_usb_probe(pusb_intf, pdid, &rtl8821au_hal_cfg);
}


static struct usb_driver rtl8821au_usb_drv = {
	.name = RTL8821AU_DRIVER_NAME,
	.probe = rtl8821au_probe,
	.disconnect = rtw_usb_disconnect,
	.id_table = rtw_usb_id_tbl,
#if 0
	.suspend =  rtl8821au_suspend,
	.resume = rtl8821au_resume,
  	.reset_resume   = rtl8821au_resume,
#endif
#ifdef CONFIG_AUTOSUSPEND
	.supports_autosuspend = 1,
#endif

};

static int __init rtl8821au_module_init(void)
{
	int res;

	res = usb_register(&rtl8821au_usb_drv);
	if (res < 0)
		pr_err(RTL8821AU_DRIVER_NAME ": usb_register() failed (%i)\n", res);

	spin_lock_init(&rtl8821au_devices_lock);

	return res;
}

static void __exit rtl8821au_module_exit(void)
{
	usb_deregister(&rtl8821au_usb_drv);
}

MODULE_DEVICE_TABLE(usb, rtw_usb_id_tbl);

module_init(rtl8821au_module_init);
module_exit(rtl8821au_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek Wireless Lan Driver");
MODULE_AUTHOR("Hans Ulli Kroll <ulli.kroll@googlemail.com>");
MODULE_VERSION("git. based on v4.2.2_7502.20130517");

module_param_named(swenc, rtl8821au_mod_params.sw_crypto, bool, 0444);
module_param_named(ips, rtl8821au_mod_params.inactiveps, bool, 0444);
module_param_named(debug, rtl8821au_mod_params.debug, int, 0444);

MODULE_PARM_DESC(swenc, "Set to 1 for software crypto (default 0)\n");
MODULE_PARM_DESC(ips, "Set to 0 to not use link power save (default 1)\n");
MODULE_PARM_DESC(debug, "Set debug level (0-5) (default 0)");
