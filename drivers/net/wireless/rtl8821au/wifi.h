#ifndef __WIFI_H__
#define __WIFI_H__

#include <linux/types.h>
#include "usb.h"

#include "efuse.h"	/* ULLI : Note this belongs to base.h we doesn't have */
#include "debug.h"

#define TOTAL_CAM_ENTRY		32
#define MAX_RF_PATH_NUM	2
#define MAX_CHNL_GROUP		3+9


enum intf_type {
	INTF_PCI = 0,
	INTF_USB = 1,
};

enum radio_path {
	RF90_PATH_A = 0,
	RF90_PATH_B = 1,
	RF90_PATH_C = 2,
	RF90_PATH_D = 3,
};

enum regulation_txpwr_lmt {
	TXPWR_LMT_FCC = 0,
	TXPWR_LMT_MKK = 1,
	TXPWR_LMT_ETSI = 2,
	TXPWR_LMT_WW = 3,

	TXPWR_LMT_MAX_REGULATION_NUM = 4
};

enum rate_section {
	CCK = 0,
	OFDM,
	HT_MCS0_MCS7,
	HT_MCS8_MCS15,
	HT_MCS16_MCS23,
	HT_MCS24_MCS31,
	VHT_1SSMCS0_1SSMCS9,
	VHT_2SSMCS0_2SSMCS9,
	VHT_3SSMCS0_3SSMCS9,
	VHT_4SSMCS0_4SSMCS9,
};

enum rf_tx_num {
	RF_1TX = 0,
	RF_2TX,
	RF_3TX,
	RF_4TX,
	RF_MAX_TX_NUM,
	RF_TX_NUM_NONIMPLEMENT,
};

enum rtl_desc92_rate {
	DESC_RATE1M = 0x00,
	DESC_RATE2M = 0x01,
	DESC_RATE5_5M = 0x02,
	DESC_RATE11M = 0x03,

	DESC_RATE6M = 0x04,
	DESC_RATE9M = 0x05,
	DESC_RATE12M = 0x06,
	DESC_RATE18M = 0x07,
	DESC_RATE24M = 0x08,
	DESC_RATE36M = 0x09,
	DESC_RATE48M = 0x0a,
	DESC_RATE54M = 0x0b,

	DESC_RATEMCS0 = 0x0c,
	DESC_RATEMCS1 = 0x0d,
	DESC_RATEMCS2 = 0x0e,
	DESC_RATEMCS3 = 0x0f,
	DESC_RATEMCS4 = 0x10,
	DESC_RATEMCS5 = 0x11,
	DESC_RATEMCS6 = 0x12,
	DESC_RATEMCS7 = 0x13,
	DESC_RATEMCS8 = 0x14,
	DESC_RATEMCS9 = 0x15,
	DESC_RATEMCS10 = 0x16,
	DESC_RATEMCS11 = 0x17,
	DESC_RATEMCS12 = 0x18,
	DESC_RATEMCS13 = 0x19,
	DESC_RATEMCS14 = 0x1a,
	DESC_RATEMCS15 = 0x1b,
	DESC_RATEMCS15_SG = 0x1c,
	DESC_RATEMCS32 = 0x20,

	DESC_RATEVHT1SS_MCS0 = 0x2c,
	DESC_RATEVHT1SS_MCS1 = 0x2d,
	DESC_RATEVHT1SS_MCS2 = 0x2e,
	DESC_RATEVHT1SS_MCS3 = 0x2f,
	DESC_RATEVHT1SS_MCS4 = 0x30,
	DESC_RATEVHT1SS_MCS5 = 0x31,
	DESC_RATEVHT1SS_MCS6 = 0x32,
	DESC_RATEVHT1SS_MCS7 = 0x33,
	DESC_RATEVHT1SS_MCS8 = 0x34,
	DESC_RATEVHT1SS_MCS9 = 0x35,
	DESC_RATEVHT2SS_MCS0 = 0x36,
	DESC_RATEVHT2SS_MCS1 = 0x37,
	DESC_RATEVHT2SS_MCS2 = 0x38,
	DESC_RATEVHT2SS_MCS3 = 0x39,
	DESC_RATEVHT2SS_MCS4 = 0x3a,
	DESC_RATEVHT2SS_MCS5 = 0x3b,
	DESC_RATEVHT2SS_MCS6 = 0x3c,
	DESC_RATEVHT2SS_MCS7 = 0x3d,
	DESC_RATEVHT2SS_MCS8 = 0x3e,
	DESC_RATEVHT2SS_MCS9 = 0x3f,
};

/* Ref: 802.11i sepc D10.0 7.3.2.25.1
Cipher Suites Encryption Algorithms */
enum rt_enc_alg {
	NO_ENCRYPTION = 0,
	WEP40_ENCRYPTION = 1,
	TKIP_ENCRYPTION = 2,
	RSERVED_ENCRYPTION = 3,
	AESCCMP_ENCRYPTION = 4,
	WEP104_ENCRYPTION = 5,
	AESCMAC_ENCRYPTION = 6,	/*IEEE802.11w */
};

enum rtl_var_map {
	/*reg map */
	SYS_ISO_CTRL = 0,
#if 0
	SYS_FUNC_EN,
	SYS_CLK,
	MAC_RCR_AM,
	MAC_RCR_AB,
	MAC_RCR_ACRC32,
	MAC_RCR_ACF,
	MAC_RCR_AAP,
	MAC_HIMR,
	MAC_HIMRE,
	MAC_HSISR,
#endif
	/*efuse map */
	EFUSE_TEST,
	EFUSE_CTRL,
	EFUSE_CLK,
	EFUSE_CLK_CTRL,
	EFUSE_PWC_EV12V,
	EFUSE_FEN_ELDR,
	EFUSE_LOADER_CLK_EN,
	EFUSE_ANA8M,
	EFUSE_HWSET_MAX_SIZE,
	EFUSE_MAX_SECTION_MAP,
	EFUSE_REAL_CONTENT_SIZE,
	EFUSE_OOB_PROTECT_BYTES_LEN,
	EFUSE_ACCESS,

	/*CAM map */
	RWCAM,
	WCAMI,
	RCAMO,
	CAMDBG,
	SECR,
	SEC_CAM_NONE,
	SEC_CAM_WEP40,
	SEC_CAM_TKIP,
	SEC_CAM_AES,
	SEC_CAM_WEP104,
#if 0
	/*IMR map */
	RTL_IMR_BCNDMAINT6,	/*Beacon DMA Interrupt 6 */
	RTL_IMR_BCNDMAINT5,	/*Beacon DMA Interrupt 5 */
	RTL_IMR_BCNDMAINT4,	/*Beacon DMA Interrupt 4 */
	RTL_IMR_BCNDMAINT3,	/*Beacon DMA Interrupt 3 */
	RTL_IMR_BCNDMAINT2,	/*Beacon DMA Interrupt 2 */
	RTL_IMR_BCNDMAINT1,	/*Beacon DMA Interrupt 1 */
	RTL_IMR_BCNDOK8,	/*Beacon Queue DMA OK Interrup 8 */
	RTL_IMR_BCNDOK7,	/*Beacon Queue DMA OK Interrup 7 */
	RTL_IMR_BCNDOK6,	/*Beacon Queue DMA OK Interrup 6 */
	RTL_IMR_BCNDOK5,	/*Beacon Queue DMA OK Interrup 5 */
	RTL_IMR_BCNDOK4,	/*Beacon Queue DMA OK Interrup 4 */
	RTL_IMR_BCNDOK3,	/*Beacon Queue DMA OK Interrup 3 */
	RTL_IMR_BCNDOK2,	/*Beacon Queue DMA OK Interrup 2 */
	RTL_IMR_BCNDOK1,	/*Beacon Queue DMA OK Interrup 1 */
	RTL_IMR_TIMEOUT2,	/*Timeout interrupt 2 */
	RTL_IMR_TIMEOUT1,	/*Timeout interrupt 1 */
	RTL_IMR_TXFOVW,		/*Transmit FIFO Overflow */
	RTL_IMR_PSTIMEOUT,	/*Power save time out interrupt */
	RTL_IMR_BCNINT,		/*Beacon DMA Interrupt 0 */
	RTL_IMR_RXFOVW,		/*Receive FIFO Overflow */
	RTL_IMR_RDU,		/*Receive Descriptor Unavailable */
	RTL_IMR_ATIMEND,	/*For 92C,ATIM Window End Interrupt */
	RTL_IMR_BDOK,		/*Beacon Queue DMA OK Interrup */
	RTL_IMR_HIGHDOK,	/*High Queue DMA OK Interrupt */
	RTL_IMR_COMDOK,		/*Command Queue DMA OK Interrupt*/
	RTL_IMR_TBDOK,		/*Transmit Beacon OK interrup */
	RTL_IMR_MGNTDOK,	/*Management Queue DMA OK Interrupt */
	RTL_IMR_TBDER,		/*For 92C,Transmit Beacon Error Interrupt */
	RTL_IMR_BKDOK,		/*AC_BK DMA OK Interrupt */
	RTL_IMR_BEDOK,		/*AC_BE DMA OK Interrupt */
	RTL_IMR_VIDOK,		/*AC_VI DMA OK Interrupt */
	RTL_IMR_VODOK,		/*AC_VO DMA Interrupt */
	RTL_IMR_ROK,		/*Receive DMA OK Interrupt */
	RTL_IMR_HSISR_IND,	/*HSISR Interrupt*/
	RTL_IBSS_INT_MASKS,	/*(RTL_IMR_BCNINT | RTL_IMR_TBDOK |
				 * RTL_IMR_TBDER) */
	RTL_IMR_C2HCMD,		/*fw interrupt*/
#endif

	/*CCK Rates, TxHT = 0 */
	RTL_RC_CCK_RATE1M,
	RTL_RC_CCK_RATE2M,
	RTL_RC_CCK_RATE5_5M,
	RTL_RC_CCK_RATE11M,

	/*OFDM Rates, TxHT = 0 */
	RTL_RC_OFDM_RATE6M,
	RTL_RC_OFDM_RATE9M,
	RTL_RC_OFDM_RATE12M,
	RTL_RC_OFDM_RATE18M,
	RTL_RC_OFDM_RATE24M,
	RTL_RC_OFDM_RATE36M,
	RTL_RC_OFDM_RATE48M,
	RTL_RC_OFDM_RATE54M,

	RTL_RC_HT_RATEMCS7,
	RTL_RC_HT_RATEMCS15,

	RTL_RC_VHT_RATE_1SS_MCS7,
	RTL_RC_VHT_RATE_1SS_MCS8,
	RTL_RC_VHT_RATE_1SS_MCS9,
	RTL_RC_VHT_RATE_2SS_MCS7,
	RTL_RC_VHT_RATE_2SS_MCS8,
	RTL_RC_VHT_RATE_2SS_MCS9,

	/*keep it last */
	RTL_VAR_MAP_MAX,
};

struct rtl_hal_ops;

#define CHANNEL_MAX_NUMBER	(14 + 24 + 21)	/* 14 is the max channel no */
#define CHANNEL_MAX_NUMBER_2G		14
#define CHANNEL_MAX_NUMBER_5G		54 /* Please refer to
					    *"phy_GetChnlGroup8812A" and
					    * "Hal_ReadTxPowerInfo8812A"
					    */
#define CHANNEL_MAX_NUMBER_5G_80M	7
#define CHANNEL_GROUP_MAX	(3 + 9)	/*  ch1~3, 4~9, 10~14 = three groups */
#define CHANNEL_MAX_NUMBER_5G		54 /* Please refer to
					    *"phy_GetChnlGroup8812A" and
					    * "Hal_ReadTxPowerInfo8812A"
					    */
#define CHANNEL_MAX_NUMBER_5G_80M	7
#define MAX_PG_GROUP			13
#define	CHANNEL_GROUP_MAX_2G		3
#define	CHANNEL_GROUP_IDX_5GL		3
#define	CHANNEL_GROUP_IDX_5GM		6
#define	CHANNEL_GROUP_IDX_5GH		9
#define	CHANNEL_GROUP_MAX_5G		9
#define CHANNEL_MAX_NUMBER_2G		14
#define AVG_THERMAL_NUM			8
#define AVG_THERMAL_NUM_88E		4
#define AVG_THERMAL_NUM_8723BE		4
#define AVG_THERMAL_NUM_8812A		4
#define MAX_TID_COUNT			9






#define MAX_REGULATION_NUM						3
#define MAX_RF_PATH_NUM_IN_POWER_LIMIT_TABLE	4
#define MAX_2_4G_BANDWITH_NUM					2
#define MAX_2_4G_RATE_SECTION_NUM				3
#define MAX_2_4G_CHANNEL_NUM						5 // adopt channel group instead of individual channel
#define MAX_5G_BANDWITH_NUM						4
#define MAX_5G_RATE_SECTION_NUM					4
#define MAX_5G_CHANNEL_NUM						14 // adopt channel group instead of individual channel

/*
 * ULLI : big remark
 * ULLI : in rtl8821ae there is no use of external_pa_2g or
 * ULLI : external_pa_5g in the phy.c file.
 * ULLI : But we need to configure the external_pa* here, because
 * ULLI : it is used here in the original source, and we don't want to
 * ULLI : drop this.
 * ULLI :
 * ULLI : Thus we do transmit some values use in rtl_dm (now _rtw_dm) and
 * ULLI : dm_priv into appreciate struct's (rtl_phy, rtl_hal, rtl_efuse)
 * ULLI : and use them, to select features.
 */
#define	MAX_RF_PATH				4
#define 	RF_PATH_MAX				MAX_RF_PATH
#define	MAX_CHNL_GROUP_24G		6
#define	MAX_CHNL_GROUP_5G		14

//It must always set to 4, otherwise read efuse table secquence will be wrong.
#define 	MAX_TX_COUNT				4

struct txpower_info_2g {
	u8 index_cck_base[MAX_RF_PATH][MAX_CHNL_GROUP_24G];
	u8 index_bw40_base[MAX_RF_PATH][MAX_CHNL_GROUP_24G];
	/*If only one tx, only BW20 and OFDM are used.*/
	u8 cck_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 ofdm_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw20_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw40_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw80_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw160_diff[MAX_RF_PATH][MAX_TX_COUNT];
};

struct txpower_info_5g {
	u8 index_bw40_base[MAX_RF_PATH][MAX_CHNL_GROUP_5G];
	/*If only one tx, only BW20, OFDM, BW80 and BW160 are used.*/
	u8 ofdm_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw20_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw40_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw80_diff[MAX_RF_PATH][MAX_TX_COUNT];
	u8 bw160_diff[MAX_RF_PATH][MAX_TX_COUNT];
};




#define	EFUSE_MAX_LOGICAL_SIZE			512

struct rtl_efuse {	bool autoLoad_ok;
	bool bootfromefuse;
	u16 max_physical_size;

	u8 efuse_map[2][EFUSE_MAX_LOGICAL_SIZE];
	u16 efuse_usedbytes;
	u8 efuse_usedpercentage;
#ifdef EFUSE_REPG_WORKAROUND
	bool efuse_re_pg_sec1flag;
	u8 efuse_re_pg_data[8];
#endif

	u8 autoload_failflag;
	u8 autoload_status;

	short epromtype;
	u16 eeprom_vid;
	u16 eeprom_did;
	u16 eeprom_svid;
	u16 eeprom_smid;
	u8 eeprom_oemid;
	u16 eeprom_channelplan;
	u8 eeprom_version;
	u8 board_type;
	u8 external_pa;

	u8 dev_addr[6];
	u8 wowlan_enable;
	u8 antenna_div_cfg;
	u8 antenna_div_type;

	bool txpwr_fromeprom;
	u8 eeprom_crystalcap;
	u8 eeprom_tssi[2];
	u8 eeprom_tssi_5g[3][2]; /* for 5GL/5GM/5GH band. */
	u8 eeprom_pwrlimit_ht20[CHANNEL_GROUP_MAX];
	u8 eeprom_pwrlimit_ht40[CHANNEL_GROUP_MAX];
	u8 eeprom_chnlarea_txpwr_cck[MAX_RF_PATH][CHANNEL_GROUP_MAX_2G];
	u8 eeprom_chnlarea_txpwr_ht40_1s[MAX_RF_PATH][CHANNEL_GROUP_MAX];
	u8 eprom_chnl_txpwr_ht40_2sdf[MAX_RF_PATH][CHANNEL_GROUP_MAX];

	u8 internal_pa_5g[2];	/* pathA / pathB */
	u8 eeprom_c9;
	u8 eeprom_cc;

	/*For power group */
	u8 eeprom_pwrgroup[2][3];
	u8 pwrgroup_ht20[2][CHANNEL_MAX_NUMBER];
	u8 pwrgroup_ht40[2][CHANNEL_MAX_NUMBER];

	u8 txpwrlevel_cck[MAX_RF_PATH][CHANNEL_MAX_NUMBER_2G];
	/*For HT 40MHZ pwr */
	u8 txpwrlevel_ht40_1s[MAX_RF_PATH][CHANNEL_MAX_NUMBER];
	/*For HT 40MHZ pwr */
	u8 txpwrlevel_ht40_2s[MAX_RF_PATH][CHANNEL_MAX_NUMBER];

	/*--------------------------------------------------------*
	 * 8192CE\8192SE\8192DE\8723AE use the following 4 arrays,
	 * other ICs (8188EE\8723BE\8192EE\8812AE...)
	 * define new arrays in Windows code.
	 * BUT, in linux code, we use the same array for all ICs.
	 *
	 * The Correspondance relation between two arrays is:
	 * txpwr_cckdiff[][] == CCK_24G_Diff[][]
	 * txpwr_ht20diff[][] == BW20_24G_Diff[][]
	 * txpwr_ht40diff[][] == BW40_24G_Diff[][]
	 * txpwr_legacyhtdiff[][] == OFDM_24G_Diff[][]
	 *
	 * Sizes of these arrays are decided by the larger ones.
	 */
	char txpwr_cckdiff[MAX_RF_PATH][CHANNEL_MAX_NUMBER];
	char txpwr_ht20diff[MAX_RF_PATH][CHANNEL_MAX_NUMBER];
	char txpwr_ht40diff[MAX_RF_PATH][CHANNEL_MAX_NUMBER];
	char txpwr_legacyhtdiff[MAX_RF_PATH][CHANNEL_MAX_NUMBER];

	u8 txpwr_5g_bw40base[MAX_RF_PATH][CHANNEL_MAX_NUMBER];
	u8 txpwr_5g_bw80base[MAX_RF_PATH][CHANNEL_MAX_NUMBER_5G_80M];
	char txpwr_5g_ofdmdiff[MAX_RF_PATH][MAX_TX_COUNT];
	char txpwr_5g_bw20diff[MAX_RF_PATH][MAX_TX_COUNT];
	char txpwr_5g_bw40diff[MAX_RF_PATH][MAX_TX_COUNT];
	char txpwr_5g_bw80diff[MAX_RF_PATH][MAX_TX_COUNT];

	u8 txpwr_safetyflag;			/* Band edge enable flag */
	u16 eeprom_txpowerdiff;
	u8 legacy_httxpowerdiff;	/* Legacy to HT rate power diff */
	u8 antenna_txpwdiff[3];

	u8 eeprom_regulatory;
	u8 eeprom_thermalmeter;
	u8 thermalmeter[2]; /*ThermalMeter, index 0 for RFIC0, 1 for RFIC1 */
	u16 tssi_13dbm;
	u8 crystalcap;		/* CrystalCap. */
	u8 delta_iqk;
	u8 delta_lck;

	u8 legacy_ht_txpowerdiff;	/*Legacy to HT rate power diff */
	bool apk_thermalmeterignore;

	bool b1x1_recvcombine;
	bool b1ss_support;

	/*channel plan */
	u8 channel_plan;

};

#define KEY_BUF_SIZE                             5
#define MAX_KEY_LEN                              61

struct rtl_security {
	/*default 0 */
	bool use_sw_sec;

	bool being_setkey;
	bool use_defaultkey;
	/*Encryption Algorithm for Unicast Packet */
	enum rt_enc_alg pairwise_enc_algorithm;
	/*Encryption Algorithm for Brocast/Multicast */
	enum rt_enc_alg group_enc_algorithm;
	/*Cam Entry Bitmap */
	u32 hwsec_cam_bitmap;
	u8 hwsec_cam_sta_addr[TOTAL_CAM_ENTRY][ETH_ALEN];
	/*local Key buffer, indx 0 is for
	   pairwise key 1-4 is for agoup key. */
	u8 key_buf[KEY_BUF_SIZE][MAX_KEY_LEN];
	u8 key_len[KEY_BUF_SIZE];

	/*The pointer of Pairwise Key,
	   it always points to KeyBuf[4] */
	u8 *pairwise_key;
};

enum hw_variables {
	HW_VAR_ETHER_ADDR,
	HW_VAR_MULTICAST_REG,
	HW_VAR_BASIC_RATE,
	HW_VAR_BSSID,
	HW_VAR_MEDIA_STATUS,
	HW_VAR_SECURITY_CONF,
	HW_VAR_BEACON_INTERVAL,
	HW_VAR_ATIM_WINDOW,
	HW_VAR_LISTEN_INTERVAL,
	HW_VAR_CS_COUNTER,
	HW_VAR_DEFAULTKEY0,
	HW_VAR_DEFAULTKEY1,
	HW_VAR_DEFAULTKEY2,
	HW_VAR_DEFAULTKEY3,
	HW_VAR_SIFS,
	HW_VAR_R2T_SIFS,
	HW_VAR_DIFS,
	HW_VAR_EIFS,
	HW_VAR_SLOT_TIME,
	HW_VAR_ACK_PREAMBLE,
	HW_VAR_CW_CONFIG,
	HW_VAR_CW_VALUES,
	HW_VAR_RATE_FALLBACK_CONTROL,
	HW_VAR_CONTENTION_WINDOW,
	HW_VAR_RETRY_COUNT,
	HW_VAR_TR_SWITCH,
	HW_VAR_COMMAND,
	HW_VAR_WPA_CONFIG,
	HW_VAR_AMPDU_MIN_SPACE,
	HW_VAR_SHORTGI_DENSITY,
	HW_VAR_AMPDU_FACTOR,
	HW_VAR_MCS_RATE_AVAILABLE,
	HW_VAR_AC_PARAM,
	HW_VAR_ACM_CTRL,
	HW_VAR_DIS_Req_Qsize,
	HW_VAR_CCX_CHNL_LOAD,
	HW_VAR_CCX_NOISE_HISTOGRAM,
	HW_VAR_CCX_CLM_NHM,
	HW_VAR_TxOPLimit,
	HW_VAR_TURBO_MODE,
	HW_VAR_RF_STATE,
	HW_VAR_RF_OFF_BY_HW,
	HW_VAR_BUS_SPEED,
	HW_VAR_SET_DEV_POWER,

	HW_VAR_RCR,
	HW_VAR_RATR_0,
	HW_VAR_RRSR,
	HW_VAR_CPU_RST,
	HW_VAR_CHECK_BSSID,
	HW_VAR_LBK_MODE,
	HW_VAR_AES_11N_FIX,
	HW_VAR_USB_RX_AGGR,
	HW_VAR_USER_CONTROL_TURBO_MODE,
	HW_VAR_RETRY_LIMIT,
	HW_VAR_INIT_TX_RATE,
	HW_VAR_TX_RATE_REG,
	HW_VAR_EFUSE_USAGE,
	HW_VAR_EFUSE_BYTES,
	HW_VAR_AUTOLOAD_STATUS,
	HW_VAR_RF_2R_DISABLE,
	HW_VAR_SET_RPWM,
	HW_VAR_H2C_FW_PWRMODE,
	HW_VAR_H2C_FW_JOINBSSRPT,
	HW_VAR_H2C_FW_MEDIASTATUSRPT,
	HW_VAR_H2C_FW_P2P_PS_OFFLOAD,
	HW_VAR_FW_PSMODE_STATUS,
	HW_VAR_INIT_RTS_RATE,
	HW_VAR_RESUME_CLK_ON,
	HW_VAR_FW_LPS_ACTION,
	HW_VAR_1X1_RECV_COMBINE,
	HW_VAR_STOP_SEND_BEACON,
	HW_VAR_TSF_TIMER,
	HW_VAR_IO_CMD,

	HW_VAR_RF_RECOVERY,
	HW_VAR_H2C_FW_UPDATE_GTK,
	HW_VAR_WF_MASK,
	HW_VAR_WF_CRC,
	HW_VAR_WF_IS_MAC_ADDR,
	HW_VAR_H2C_FW_OFFLOAD,
	HW_VAR_RESET_WFCRC,

	HW_VAR_HANDLE_FW_C2H,
	HW_VAR_DL_FW_RSVD_PAGE,
	HW_VAR_AID,
	HW_VAR_HW_SEQ_ENABLE,
	HW_VAR_CORRECT_TSF,
	HW_VAR_BCN_VALID,
	HW_VAR_FWLPS_RF_ON,
	HW_VAR_DUAL_TSF_RST,
	HW_VAR_SWITCH_EPHY_WoWLAN,
	HW_VAR_INT_MIGRATION,
	HW_VAR_INT_AC,
	HW_VAR_RF_TIMING,

	HAL_DEF_WOWLAN,
	HW_VAR_MRC,
	HW_VAR_KEEP_ALIVE,
	HW_VAR_NAV_UPPER,

	HW_VAR_MGT_FILTER,
	HW_VAR_CTRL_FILTER,
	HW_VAR_DATA_FILTER,

	/* ULLI : HW VARS rtlwifi end */

	HW_VAR_MEDIA_STATUS1,
	HW_VAR_TXPAUSE,
	HW_VAR_BCN_FUNC,
	HW_VAR_MLME_DISCONNECT,
	HW_VAR_MLME_SITESURVEY,
	HW_VAR_MLME_JOIN,
	HW_VAR_ON_RCR_AM,
	HW_VAR_OFF_RCR_AM,
	HW_VAR_RESP_SIFS,
	HW_VAR_SEC_CFG,
	HW_VAR_CAM_WRITE,
	HW_VAR_CAM_READ,
	HW_VAR_AC_PARAM_VO,
	HW_VAR_AC_PARAM_VI,
	HW_VAR_AC_PARAM_BE,
	HW_VAR_AC_PARAM_BK,
	HW_VAR_RXDMA_AGG_PG_TH,
	HW_VAR_H2C_PS_TUNE_PARAM,
	HW_VAR_TDLS_WRCR,
	HW_VAR_TDLS_INIT_CH_SEN,
	HW_VAR_TDLS_RS_RCR,
	HW_VAR_TDLS_DONE_CH_SEN,
	HW_VAR_INITIAL_GAIN,
	HW_VAR_TRIGGER_GPIO_0,
	HW_VAR_BT_SET_COEXIST,
	HW_VAR_BT_ISSUE_DELBA,
	HW_VAR_CURRENT_ANTENNA,
	HW_VAR_ANTENNA_DIVERSITY_LINK,
	HW_VAR_ANTENNA_DIVERSITY_SELECT,
	HW_VAR_EFUSE_BT_USAGE,
	HW_VAR_EFUSE_BT_BYTES,
	HW_VAR_FIFO_CLEARN_UP,
	HW_VAR_CHECK_TXBUF,
	HW_VAR_C2H_HANDLE,
	HW_VAR_RPT_TIMER_SETTING,
	HW_VAR_TX_RPT_MAX_MACID,
	HW_VAR_H2C_MEDIA_STATUS_RPT,
	HW_VAR_CHK_HI_QUEUE_EMPTY,
	HW_VAR_DL_BCN_SEL,
	HW_VAR_AMPDU_MAX_TIME,
	HW_VAR_WIRELESS_MODE,
};

typedef enum tag_Board_Definition
{
    ODM_BOARD_DEFAULT  	= 0, 	  // The DEFAULT case.
    ODM_BOARD_MINICARD  = BIT(0), // 0 = non-mini card, 1= mini card.
    ODM_BOARD_SLIM      = BIT(1), // 0 = non-slim card, 1 = slim card
    ODM_BOARD_BT        = BIT(2), // 0 = without BT card, 1 = with BT
    ODM_BOARD_EXT_PA    = BIT(3), // 0 = no 2G ext-PA, 1 = existing 2G ext-PA
    ODM_BOARD_EXT_LNA   = BIT(4), // 0 = no 2G ext-LNA, 1 = existing 2G ext-LNA
    ODM_BOARD_EXT_TRSW  = BIT(5), // 0 = no ext-TRSW, 1 = existing ext-TRSW
    ODM_BOARD_EXT_PA_5G    = BIT(6), // 0 = no 5G ext-PA, 1 = existing 5G ext-PA
    ODM_BOARD_EXT_LNA_5G   = BIT(7), // 0 = no 5G ext-LNA, 1 = existing 5G ext-LNA
}ODM_BOARD_TYPE_E;

enum band_type {
	BAND_ON_2_4G = 0,
	BAND_ON_5G,
	BAND_ON_BOTH,
	BANDMAX
};

/*mlme related.*/
enum wireless_mode {
	WIRELESS_MODE_UNKNOWN = 0x00,
	WIRELESS_MODE_A = 0x01,
	WIRELESS_MODE_B = 0x02,
	WIRELESS_MODE_G = 0x04,
	WIRELESS_MODE_AUTO = 0x08,
	WIRELESS_MODE_N_24G = 0x10,
	WIRELESS_MODE_N_5G = 0x20,
	WIRELESS_MODE_AC_5G = 0x40,
	WIRELESS_MODE_AC_24G  = 0x80,
	WIRELESS_MODE_AC_ONLY = 0x100,
#if 0	/* ULLI : we can't enable this,but this is not used in rtlwifi */
	WIRELESS_MODE_MAX = 0x800
#endif
};

struct rtl_hal {
#if 0
	struct ieee80211_hw *hw;
#endif
	bool driver_is_goingto_unload;
	bool up_first_time;
	bool first_init;
	bool being_init_adapter;
	bool bbrf_ready;
	bool mac_func_enable;
	bool pre_edcca_enable;
#if 0
	struct bt_coexist_8723 hal_coex_8723;
#endif

	enum intf_type interface;
	u16 hw_type;		/*92c or 92d or 92s and so on */
	u8 ic_class;
	u8 oem_id;
	u32 version;		/*version of chip */
	u8 state;		/*stop 0, start 1 */
	u8 board_type;
	u8 external_pa;

	u8 pa_mode;
	u8 pa_type_2g;
	u8 pa_type_5g;
	u8 lna_type_2g;
	u8 lna_type_5g;
	u8 external_pa_2g;
	u8 external_lna_2g;
	u8 external_pa_5g;
	u8 external_lna_5g;
	u8 rfe_type;

	/*firmware */
	u32 fwsize;
	u8 *pfirmware;
	u16 fw_version;
	u16 fw_subversion;
	bool h2c_setinprogress;
	u8 last_hmeboxnum;
	bool fw_ready;
	/*Reserve page start offset except beacon in TxQ. */
	u8 fw_rsvdpage_startoffset;
	u8 h2c_txcmd_seq;
	u8 current_ra_rate;

	/* FW Cmd IO related */
	u16 fwcmd_iomap;
	u32 fwcmd_ioparam;
	bool set_fwcmd_inprogress;
	u8 current_fwcmd_io;

#if 0
	struct p2p_ps_offload_t p2p_ps_offload;
#endif
	bool fw_clk_change_in_progress;
	bool allow_sw_to_change_hwclc;
	u8 fw_ps_state;
	/**/
	bool driver_going2unload;

	/*AMPDU init min space*/
	u8 minspace_cfg;	/*For Min spacing configurations */

	/* Dual mac */
#if 0
	enum macphy_mode macphymode;
#endif
	enum band_type current_bandtype;	/* 0:2.4G, 1:5G */
#if 0
	enum band_type current_bandtypebackup;
#endif
	enum band_type bandset;

	/* dual MAC 0--Mac0 1--Mac1 */
	u32 interfaceindex;
	/* just for DualMac S3S4 */
	u8 macphyctl_reg;
	bool earlymode_enable;
	u8 max_earlymode_num;
	/* Dual mac*/
	bool during_mac0init_radiob;
	bool during_mac1init_radioa;
	bool reloadtxpowerindex;
	/* true if IMR or IQK  have done
	for 2.4G in scan progress */
	bool load_imrandiqk_setting_for2g;

	bool disable_amsdu_8k;
	bool master_of_dmsp;
	bool slave_of_dmsp;

	u16 rx_tag;/*for 92ee*/
	u8 rts_en;

	/*for wowlan*/
	bool wow_enable;
	bool enter_pnp_sleep;
	bool wake_from_pnp_sleep;
	bool wow_enabled;
	__kernel_time_t last_suspend_sec;
	u32 wowlan_fwsize;
	u8 *wowlan_firmware;

	u8 hw_rof_enable; /*Enable GPIO[9] as WL RF HW PDn source*/

	bool real_wow_v2_enable;
	bool re_init_llt_table;
};


#define MAX_TX_COUNT			4
#define MAX_RATE_SECTION_NUM		6
#define MAX_5G_BANDWITH_NUM		4
#define	MAX_RF_PATH			4
#define	MAX_CHNL_GROUP_24G		6
#define	MAX_CHNL_GROUP_5G		14

#define TX_PWR_BY_RATE_NUM_BAND		2
#define TX_PWR_BY_RATE_NUM_RF		4
#define TX_PWR_BY_RATE_NUM_SECTION	12
#define MAX_BASE_NUM_IN_PHY_REG_PG_24G  6
#define MAX_BASE_NUM_IN_PHY_REG_PG_5G	5


//###### duplicate code,will move to ODM #########
#define IQK_MAC_REG_NUM		4
#define IQK_ADDA_REG_NUM		16

#define IQK_BB_REG_NUM			10
#define IQK_BB_REG_NUM_92C	9
#define IQK_BB_REG_NUM_92D	10
#define IQK_BB_REG_NUM_test	6


#define IQK_MATRIX_REG_NUM	8
#define IQK_MATRIX_SETTINGS_NUM	(1 + 24 + 21)

enum rt_polarity_ctl {
	RT_POLARITY_LOW_ACT = 0,
	RT_POLARITY_HIGH_ACT = 1,
};

struct iqk_matrix_regs {
	bool iqk_done;
	long value[1][IQK_MATRIX_REG_NUM];
};


struct phy_parameters {
	u16 length;
	u32 *pdata;
};

enum hw_param_tab_index {
	PHY_REG_2T,
	PHY_REG_1T,
	PHY_REG_PG,
	RADIOA_2T,
	RADIOB_2T,
	RADIOA_1T,
	RADIOB_1T,
	MAC_REG,
	AGCTAB_2T,
	AGCTAB_1T,
	MAX_TAB
};

enum io_type {
	IO_CMD_PAUSE_DM_BY_SCAN = 0,
	IO_CMD_PAUSE_BAND0_DM_BY_SCAN = 0,
	IO_CMD_PAUSE_BAND1_DM_BY_SCAN = 1,
	IO_CMD_RESUME_DM_BY_SCAN = 2,
};

struct bb_reg_def {
	u32 rfintfs;
	u32 rfintfi;
	u32 rfintfo;
	u32 rfintfe;
	u32 rf3wire_offset;
	u32 rflssi_select;
	u32 rftxgain_stage;
	u32 rfhssi_para1;
	u32 rfhssi_para2;
	u32 rfsw_ctrl;
	u32 rfagc_control1;
	u32 rfagc_control2;
	u32 rfrxiq_imbal;
	u32 rfrx_afe;
	u32 rftxiq_imbal;
	u32 rftx_afe;
	u32 rf_rb;		/* rflssi_readback */
	u32 rf_rbpi;		/* rflssi_readbackpi */
};

struct init_gain {
	u8 xaagccore1;
	u8 xbagccore1;
	u8 xcagccore1;
	u8 xdagccore1;
	u8 cca;

};

struct rtl_phy {
	struct bb_reg_def phyreg_def[4];	/*Radio A/B/C/D */
	struct init_gain initgain_backup;
	enum io_type current_io_type;

	u8 rf_mode;
	u8 rf_type;
	u8 current_chan_bw;
	u8 set_bwmode_inprogress;
	u8 sw_chnl_inprogress;
	u8 sw_chnl_stage;
	u8 sw_chnl_step;
	u8 current_channel;
	u8 h2c_box_num;
	u8 set_io_inprogress;
	u8 lck_inprogress;

	/* record for power tracking */
	s32 reg_e94;
	s32 reg_e9c;
	s32 reg_ea4;
	s32 reg_eac;
	s32 reg_eb4;
	s32 reg_ebc;
	s32 reg_ec4;
	s32 reg_ecc;
	u8 rfpienable;
	u8 reserve_0;
	u16 reserve_1;
	u32 reg_c04, reg_c08, reg_874;
	u32 adda_backup[16];
	u32 iqk_mac_backup[IQK_MAC_REG_NUM];
	u32 iqk_bb_backup[10];
	bool iqk_initialized;

	bool rfpath_rx_enable[MAX_RF_PATH];
	u8 reg_837;
	/* Dual mac */
	bool need_iqk;
	struct iqk_matrix_regs iqk_matrix[IQK_MATRIX_SETTINGS_NUM];

	bool rfpi_enable;
	bool iqk_in_progress;

	u8 pwrgroup_cnt;
	u8 cck_high_power;
	/* this is for 88E & 8723A */
	u32 mcs_txpwrlevel_origoffset[MAX_PG_GROUP][16];
	/* MAX_PG_GROUP groups of pwr diff by rates */
	u32 mcs_offset[MAX_PG_GROUP][16];

	u32 tx_power_by_rate_offset[TX_PWR_BY_RATE_NUM_BAND]
				   [TX_PWR_BY_RATE_NUM_RF]
				   [TX_PWR_BY_RATE_NUM_RF]
				   [TX_PWR_BY_RATE_NUM_SECTION];
	u8 txpwr_by_rate_base_24g[TX_PWR_BY_RATE_NUM_RF]
				 [TX_PWR_BY_RATE_NUM_RF]
				 [MAX_BASE_NUM_IN_PHY_REG_PG_24G];
	u8 txpwr_by_rate_base_5g[TX_PWR_BY_RATE_NUM_RF]
				[TX_PWR_BY_RATE_NUM_RF]
				[MAX_BASE_NUM_IN_PHY_REG_PG_5G];
	u8 default_initialgain[4];

	/* the current Tx power level */
	u8 cur_cck_txpwridx;
	u8 cur_ofdm24g_txpwridx;
	u8 cur_bw20_txpwridx;
	u8 cur_bw40_txpwridx;

	char txpwr_limit_2_4g[MAX_REGULATION_NUM]
			     [MAX_2_4G_BANDWITH_NUM]
			     [MAX_RATE_SECTION_NUM]
			     [CHANNEL_MAX_NUMBER_2G]
			     [MAX_RF_PATH_NUM];
	char txpwr_limit_5g[MAX_REGULATION_NUM]
			   [MAX_5G_BANDWITH_NUM]
			   [MAX_RATE_SECTION_NUM]
			   [CHANNEL_MAX_NUMBER_5G]
			   [MAX_RF_PATH_NUM];

	u32 rfreg_chnlval[2];
	bool apk_done;
	u32 reg_rf3c[2];	/* pathA / pathB  */

	u32 backup_rf_0x1a;/*92ee*/
	/* bfsync */
	u8 framesync;
	u32 framesync_c34;

	u8 num_total_rfpath;
	struct phy_parameters hwparam_tables[MAX_TAB];
	u16 rf_pathmap;

	u8 hw_rof_enable; /*Enable GPIO[9] as WL RF HW PDn source*/
	enum rt_polarity_ctl polarity_ctl;


	/* ULLI : Border from old (struct rtw_hal) */

#define MAX_BASE_NUM_IN_PHY_REG_PG_2_4G			4 //  CCK:1,OFDM:2, HT:2
#define MAX_BASE_NUM_IN_PHY_REG_PG_5G			5 // OFDM:1, HT:2, VHT:2

	/* ULLI: neded to convert this */

	//---------------------------------------------------------------------------------//

	// Power Limit Table for 2.4G
};


#define AVG_THERMAL_NUM		8
#define IQK_Matrix_REG_NUM		8
#define IQK_Matrix_Settings_NUM	14+24+21 // Channels_2_4G_NUM + Channels_5G_20M_NUM + Channels_5G

#define ASSOCIATE_ENTRY_NUM					32 // Max size of AsocEntry[].
#define	ODM_ASSOCIATE_ENTRY_NUM				ASSOCIATE_ENTRY_NUM

struct fast_ant_training {
	u8	bssid[6];
	u8	antsel_rx_keep_0;
	u8	antsel_rx_keep_1;
	u8	antsel_rx_keep_2;
	u32	ant_sum[7];
	u32	ant_cnt[7];
	u32	ant_ave[7];
	u8	fat_state;
	u32	train_idx;
	u8	antsel_a[ASSOCIATE_ENTRY_NUM];
	u8	antsel_b[ASSOCIATE_ENTRY_NUM];
	u8	antsel_c[ASSOCIATE_ENTRY_NUM];
	u32	main_ant_sum[ASSOCIATE_ENTRY_NUM];
	u32	aux_ant_sum[ASSOCIATE_ENTRY_NUM];
	u32	main_ant_cnt[ASSOCIATE_ENTRY_NUM];
	u32	aux_ant_cnt[ASSOCIATE_ENTRY_NUM];
	u8	rx_idle_ant;
	bool	becomelinked;
};


struct dm_phy_dbg_info {
	char rx_snrdb[4];
	u64 num_qry_phy_status;
	u64 num_qry_phy_status_cck;
	u64 num_qry_phy_status_ofdm;
	u16 num_qry_beacon_pkt;
	u16 num_non_be_pkt;
	s32 rx_evm[4];
};

#define DEL_SW_IDX_SZ		30
#define BAND_NUM			3

/* ULLI : Follwing needed for Rate Adaptive */

typedef struct _ODM_RA_Info_ {
	u8 RateID;
	uint32_t RateMask;
	uint32_t RAUseRate;
	u8 RateSGI;
	u8 RssiStaRA;
	u8 PreRssiStaRA;
	u8 SGIEnable;
	u8 DecisionRate;
	u8 PreRate;
	u8 HighestRate;
	u8 LowestRate;
	uint32_t NscUp;
	uint32_t NscDown;
	u16 RTY[5];
	uint32_t TOTAL;
	u16 DROP;
	u8 Active;
	u16 RptTime;
	u8 RAWaitingCounter;
	u8 RAPendingCounter;
} ODM_RA_INFO_T,*PODM_RA_INFO_T;

struct rtl_dm {
	/*PHY status for Dynamic Management */
	long entry_min_undec_sm_pwdb;
	long undec_sm_cck;
	long undec_sm_pwdb;	/*out dm */
	long entry_max_undec_sm_pwdb;
	s32 ofdm_pkt_cnt;
	bool dm_initialgain_enable;
	bool dynamic_txpower_enable;
	bool current_turbo_edca;
	bool is_any_nonbepkts;	/*out dm */
	bool is_cur_rdlstate;
	bool txpower_trackinginit;
	bool disable_framebursting;
	bool cck_inch14;
	bool txpower_tracking;
	bool useramask;
	bool rfpath_rxenable[4];
	bool inform_fw_driverctrldm;
	bool current_mrc_switch;
	u8 txpowercount;
	u8 powerindex_backup[6];

	u8 thermalvalue_rxgain;
	u8 thermalvalue_iqk;
	u8 thermalvalue_lck;
	u8 thermalvalue;		/* introduced in commit 1637c1b7e */
	u8 last_dtp_lvl;
	u8 thermalvalue_avg[AVG_THERMAL_NUM];
	u8 thermalvalue_avg_index;
	u8 tm_trigger;
	bool done_txpower;
	u8 dynamic_txhighpower_lvl;	/*Tx high power level */
	u8 dm_flag;		/*Indicate each dynamic mechanism's status. */
	u8 dm_flag_tmp;
	u8 dm_type;
	u8 dm_rssi_sel;
	u8 txpower_track_control;
	bool interrupt_migration;
	bool disable_tx_int;
	char ofdm_index[MAX_RF_PATH];
	u8 default_ofdm_index;
	u8 default_cck_index;
	char cck_index;
	char delta_power_index[MAX_RF_PATH];
	char delta_power_index_last[MAX_RF_PATH];
	char power_index_offset[MAX_RF_PATH];
	char absolute_ofdm_swing_idx[MAX_RF_PATH];
	char remnant_ofdm_swing_idx[MAX_RF_PATH];
	char remnant_cck_idx;
	bool modify_txagc_flag_path_a;
	bool modify_txagc_flag_path_b;

	bool one_entry_only;

	struct dm_phy_dbg_info dbginfo;

	/* Dynamic ATC switch */
	bool atc_status;
	bool large_cfo_hit;
	bool is_freeze;
	int cfo_tail[2];
	int cfo_ave_pre;
	int crystal_cap;
	u8 cfo_threshold;
	u32 packet_count;
	u32 packet_count_pre;
	u8 tx_rate;

	/*88e tx power tracking*/
	u8	swing_idx_ofdm[MAX_RF_PATH];
	u8	swing_idx_ofdm_cur;
	u8	swing_idx_ofdm_base[MAX_RF_PATH];
	bool	swing_flag_ofdm;
	u8	swing_idx_cck;
	u8	swing_idx_cck_cur;
	u8	swing_idx_cck_base;
	bool	swing_flag_cck;

	char	swing_diff_2g;
	char	swing_diff_5g;

	u8 delta_swing_table_idx_24gccka_p[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24gccka_n[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24gcckb_p[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24gcckb_n[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24ga_p[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24ga_n[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24gb_p[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24gb_n[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_5ga_p[BAND_NUM][DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_5ga_n[BAND_NUM][DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_5gb_p[BAND_NUM][DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_5gb_n[BAND_NUM][DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24ga_p_8188e[DEL_SW_IDX_SZ];
	u8 delta_swing_table_idx_24ga_n_8188e[DEL_SW_IDX_SZ];

	/* DMSP */
	bool supp_phymode_switch;

	/* DulMac */
	struct fast_ant_training fat_table;

	u8	resp_tx_path;
	u8	path_sel;
	u32	patha_sum;
	u32	pathb_sum;
	u32	patha_cnt;
	u32	pathb_cnt;

	u8 pre_channel;
	u8 *p_channel;
	u8 linked_interval;

	u64 last_tx_ok_cnt;
	u64 last_rx_ok_cnt;

/* ULLI : Form struct _rtw_dm, aka original sources */

	u16 			CurrminRptTime;
	ODM_RA_INFO_T   RAInfo[ODM_ASSOCIATE_ENTRY_NUM]; //See HalMacID support
	struct sta_info *pODM_StaInfo[ODM_ASSOCIATE_ENTRY_NUM];
	bool bOneEntryOnly;
};

#define rtl_hal(rtlpriv)	(&((rtlpriv)->rtlhal))
#define rtl_mac(rtlpriv)	(&((rtlpriv)->mac80211))
#define rtl_efuse(rtlpriv)	(&((rtlpriv)->efuse))
#define rtl_phy(rtlpriv)	(&((rtlpriv)->phy))
#define rtl_dm(rtlpriv)		(&((rtlpriv)->dm))
#define rtl_psc(rtlpriv)	(&((rtlpriv)->psc))
#define rtl_priv(ndev)		(netdev_priv(ndev))

struct rtl_mod_params {
	/* default: 0 = using hardware encryption */
	bool sw_crypto;

	/* default: 0 = DBG_EMERG (0)*/
	int debug;

	/* default: 1 = using no linked power save */
	bool inactiveps;

	/* default: 1 = using linked sw power save */
	bool swctrl_lps;

	/* default: 1 = using linked fw power save */
	bool fwctrl_lps;

	/* default: 0 = not using MSI interrupts mode
	 * submodules should set their own default value
	 */
	bool msi_support;

	/* default 0: 1 means disable */
	bool disable_watchdog;
};

struct rtl_hal_usbint_cfg {
	/* data - rx */
	u32 in_ep_num;
	u32 rx_urb_num;
	u32 rx_max_size;
#if 0	/* ULLI Currently disabled */
	/* op - rx */
	void (*usb_rx_hdl)(struct ieee80211_hw *, struct sk_buff *);
	void (*usb_rx_segregate_hdl)(struct ieee80211_hw *, struct sk_buff *,
				     struct sk_buff_head *);

	/* tx */
	void (*usb_tx_cleanup)(struct ieee80211_hw *, struct sk_buff *);
	int (*usb_tx_post_hdl)(struct ieee80211_hw *, struct urb *,
			       struct sk_buff *);
	struct sk_buff *(*usb_tx_aggregate_hdl)(struct ieee80211_hw *,
						struct sk_buff_head *);

	/* endpoint mapping */
#endif
	int (*usb_endpoint_mapping)(struct rtl_priv *rtlpriv);
#if 0
	u16 (*usb_mq_to_hwq)(__le16 fc, u16 mac80211_queue_index);
#endif
};

struct rtl_hal_cfg {
	u8 bar_id;
	bool write_readback;
	char *name;
	char *fw_name;
	char *alt_fw_name;
	struct rtl_hal_ops *ops;
	struct rtl_mod_params *mod_params;
	struct rtl_hal_usbint_cfg *usb_interface_cfg;
	/*this map used for some registers or vars
	   defined int HAL but used in MAIN */
	u32 maps[RTL_VAR_MAP_MAX];
};

struct rtl_priv;
struct rtl_io {
	struct device *dev;
	struct mutex bb_mutex;

	/*PCI MEM map */
	unsigned long pci_mem_end;	/*shared mem end        */
	unsigned long pci_mem_start;	/*shared mem start */

	/*PCI IO map */
	unsigned long pci_base_addr;	/*device I/O address */

	void (*write8_async) (struct rtl_priv *rtlpriv, u32 addr, u8 val);
	void (*write16_async) (struct rtl_priv *rtlpriv, u32 addr, u16 val);
	void (*write32_async) (struct rtl_priv *rtlpriv, u32 addr, u32 val);
	void (*writeN_sync) (struct rtl_priv *rtlpriv, u32 addr, void *buf,
			     u16 len);

	u8(*read8_sync) (struct rtl_priv *rtlpriv, u32 addr);
	u16(*read16_sync) (struct rtl_priv *rtlpriv, u32 addr);
	u32(*read32_sync) (struct rtl_priv *rtlpriv, u32 addr);
};

struct dig_t {
	u32 rssi_lowthresh;
	u32 rssi_highthresh;
	u32 fa_lowthresh;
	u32 fa_highthresh;
	long last_min_undec_pwdb_for_dm;
	long rssi_highpower_lowthresh;
	long rssi_highpower_highthresh;
	u32 recover_cnt;
	u32 pre_igvalue;
	u32 cur_igvalue;
	long rssi_val;
	u8 dig_enable_flag;
	u8 dig_ext_port_stage;
	u8 dig_algorithm;
	u8 dig_twoport_algorithm;
	u8 dig_dbgmode;
	u8 dig_slgorithm_switch;
	u8 cursta_cstate;
	u8 presta_cstate;
	u8 curmultista_cstate;
	u8 stop_dig;
	char back_val;
	char back_range_max;
	char back_range_min;
	u8 rx_gain_max;
	u8 rx_gain_min;
	u8 min_undec_pwdb_for_dm;
	u8 rssi_val_min;
	u8 pre_cck_cca_thres;
	u8 cur_cck_cca_thres;
	u8 pre_cck_pd_state;
	u8 cur_cck_pd_state;
	u8 pre_cck_fa_state;
	u8 cur_cck_fa_state;
	u8 pre_ccastate;
	u8 cur_ccasate;
	u8 large_fa_hit;
	u8 forbidden_igi;
	u8 dig_state;
	u8 dig_highpwrstate;
	u8 cur_sta_cstate;
	u8 pre_sta_cstate;
	u8 cur_ap_cstate;
	u8 pre_ap_cstate;
	u8 cur_pd_thstate;
	u8 pre_pd_thstate;
	u8 cur_cs_ratiostate;
	u8 pre_cs_ratiostate;
	u8 backoff_enable_flag;
	char backoffval_range_max;
	char backoffval_range_min;
	u8 dig_min_0;
	u8 dig_min_1;
	u8 bt30_cur_igi;
	bool media_connect_0;
	bool media_connect_1;

	u32 antdiv_rssi_max;
	u32 rssi_max;

/* ULLI : not in rtlwifi */

	u8 BackupIGValue;
};

struct rate_adaptive {
	u8 rate_adaptive_disabled;
	u8 ratr_state;
	u16 reserve;

	u32 high_rssi_thresh_for_ra;
	u32 high2low_rssi_thresh_for_ra;
	u8 low2high_rssi_thresh_for_ra40m;
	u32 low_rssi_thresh_for_ra40m;
	u8 low2high_rssi_thresh_for_ra20m;
	u32 low_rssi_thresh_for_ra20m;
	u32 upper_rssi_threshold_ratr;
	u32 middleupper_rssi_threshold_ratr;
	u32 middle_rssi_threshold_ratr;
	u32 middlelow_rssi_threshold_ratr;
	u32 low_rssi_threshold_ratr;
	u32 ultralow_rssi_threshold_ratr;
	u32 low_rssi_threshold_ratr_40m;
	u32 low_rssi_threshold_ratr_20m;
	u8 ping_rssi_enable;
	u32 ping_rssi_ratr;
	u32 ping_rssi_thresh_for_ra;
	u32 last_ratr;
	u8 pre_ratr_state;
	u8 ldpc_thres;
	bool use_ldpc;
	bool lower_rts_rate;
	bool is_special_data;
};

enum rtl_link_state {
	MAC80211_NOLINK = 0,
	MAC80211_LINKING = 1,
	MAC80211_LINKED = 2,
	MAC80211_LINKED_SCANNING = 3,
};

struct rtl_mac {
	u8 mac_addr[ETH_ALEN];
	u8 mac80211_registered;
	u8 beacon_enabled;

	u32 tx_ss_num;
	u32 rx_ss_num;

#if 0	/* ULLI : Currently we are using wireless ext */
	struct ieee80211_supported_band bands[IEEE80211_NUM_BANDS];
	struct ieee80211_hw *hw;
	struct ieee80211_vif *vif;
	enum nl80211_iftype opmode;
#endif

	/*Probe Beacon management */
#if 0	/* Ulli : currently we are using wireless-ext */
	struct rtl_tid_data tids[MAX_TID_COUNT];
#endif
	enum rtl_link_state link_state;
	int n_channels;
	int n_bitrates;

	bool offchan_delay;
	u8 p2p;	/*using p2p role*/
	bool p2p_in_use;

	/*filters */
	u32 rx_conf;
	u16 rx_mgt_filter;
	u16 rx_ctrl_filter;
	u16 rx_data_filter;

	bool act_scanning;
	u8 cnt_after_linked;
	bool skip_scan;

	/* early mode */
	/* skb wait queue */
	struct sk_buff_head skb_waitq[MAX_TID_COUNT];

	u8 ht_stbc_cap;
	u8 ht_cur_stbc;

	/*vht support*/
	u8 vht_enable;
	u8 bw_80;
	u8 vht_cur_ldpc;
	u8 vht_cur_stbc;
	u8 vht_stbc_cap;
	u8 vht_ldpc_cap;

	/*RDG*/
	bool rdg_en;

	/*AP*/
	u8 bssid[ETH_ALEN] __aligned(2);
	u32 vendor;
	u8 mcs[16];	/* 16 bytes mcs for HT rates. */
	u32 basic_rates; /* b/g rates */
	u8 ht_enable;
	u8 sgi_40;
	u8 sgi_20;
	u8 bw_40;
	u16 mode;		/* wireless mode */
	u8 slot_time;
	u8 short_preamble;
	u8 use_cts_protect;
	u8 cur_40_prime_sc;
	u8 cur_40_prime_sc_bk;
	u8 cur_80_prime_sc;
	u64 tsf;
	u8 retry_short;
	u8 retry_long;
	u16 assoc_id;
	bool hiddenssid;

	/*IBSS*/
	int beacon_interval;

	/*AMPDU*/
	u8 min_space_cfg;	/*For Min spacing configurations */
	u8 max_mss_density;
	u8 current_ampdu_factor;
	u8 current_ampdu_density;

	/*QOS & EDCA */
#if 0	/* Ulli : currently we are using wireless-ext */
	struct ieee80211_tx_queue_params edca_param[RTL_MAC80211_NUM_QUEUE];
	struct rtl_qos_parameters ac[AC_MAX];
#endif

	/* counters */
	u64 last_txok_cnt;
	u64 last_rxok_cnt;
	u32 last_bt_edca_ul;
	u32 last_bt_edca_dl;
};

struct false_alarm_statistics {
	u32 cnt_parity_fail;
	u32 cnt_rate_illegal;
	u32 cnt_crc8_fail;
	u32 cnt_mcs_fail;
	u32 cnt_fast_fsync_fail;
	u32 cnt_sb_search_fail;
	u32 cnt_ofdm_fail;
	u32 cnt_cck_fail;
	u32 cnt_all;
	u32 cnt_ofdm_cca;
	u32 cnt_cck_cca;
	u32 cnt_cca_all;
	u32 cnt_bw_usc;
	u32 cnt_bw_lsc;
};

enum rt_psmode {
	EACTIVE,		/*Active/Continuous access. */
	EMAXPS,			/*Max power save mode. */
	EFASTPS,		/*Fast power save mode. */
	EAUTOPS,		/*Auto power save mode. */
};

/*RF state.*/
enum rf_pwrstate {
	ERFON,
	ERFSLEEP,
	ERFOFF
};

enum p2p_ps_state {
	P2P_PS_DISABLE = 0,
	P2P_PS_ENABLE = 1,
	P2P_PS_SCAN = 2,
	P2P_PS_SCAN_DONE = 3,
	P2P_PS_ALLSTASLEEP = 4, /* for P2P GO */
};

enum p2p_ps_mode {
	P2P_PS_NONE = 0,
	P2P_PS_CTWINDOW = 1,
	P2P_PS_NOA	 = 2,
	P2P_PS_MIX = 3, /* CTWindow and NoA */
};

struct rtl_p2p_ps_info {
	enum p2p_ps_mode p2p_ps_mode; /* indicate p2p ps mode */
	enum p2p_ps_state p2p_ps_state; /*  indicate p2p ps state */
	u8 noa_index; /*  Identifies instance of Notice of Absence timing. */
	/*  Client traffic window. A period of time in TU after TBTT. */
	u8 ctwindow;
	u8 opp_ps; /*  opportunistic power save. */
	u8 noa_num; /*  number of NoA descriptor in P2P IE. */
	/*  Count for owner, Type of client. */
	u8 noa_count_type[P2P_MAX_NOA_NUM];
	/*  Max duration for owner, preferred or min acceptable duration
	 * for client.
	 */
	u32 noa_duration[P2P_MAX_NOA_NUM];
	/*  Length of interval for owner, preferred or max acceptable intervali
	 * of client.
	 */
	u32 noa_interval[P2P_MAX_NOA_NUM];
	/*  schedule in terms of the lower 4 bytes of the TSF timer. */
	u32 noa_start_time[P2P_MAX_NOA_NUM];
};

struct rtl_ps_ctl {
	bool pwrdomain_protect;
	bool in_powersavemode;
	bool rfchange_inprogress;
	bool swrf_processing;
	bool hwradiooff;
	/*
	 * just for PCIE ASPM
	 * If it supports ASPM, Offset[560h] = 0x40,
	 * otherwise Offset[560h] = 0x00.
	 * */
	bool support_aspm;
	bool support_backdoor;

	/*for LPS */
	enum rt_psmode dot11_psmode;	/*Power save mode configured. */
	bool swctrl_lps;
	bool leisure_ps;
	bool fwctrl_lps;
	u8 fwctrl_psmode;
	/*For Fw control LPS mode */
	u8 reg_fwctrl_lps;
	/*Record Fw PS mode status. */
	bool fw_current_inpsmode;
	u8 reg_max_lps_awakeintvl;
	bool report_linked;
	bool low_power_enable;/*for 32k*/

	/*for IPS */
	bool inactiveps;

	u32 rfoff_reason;

	/*RF OFF Level */
	u32 cur_ps_level;
	u32 reg_rfps_level;

	/*just for PCIE ASPM */
	u8 const_amdpci_aspm;
	bool pwrdown_mode;

	enum rf_pwrstate inactive_pwrstate;
	enum rf_pwrstate rfpwr_state;	/*cur power state */

	/* for SW LPS*/
	bool sw_ps_enabled;
	bool state;
	bool state_inap;
	bool multi_buffered;
	u16 nullfunc_seq;
	unsigned int dtim_counter;
	unsigned int sleep_ms;
	unsigned long last_sleep_jiffies;
	unsigned long last_awake_jiffies;
	unsigned long last_delaylps_stamp_jiffies;
	unsigned long last_dtim;
	unsigned long last_beacon;
	unsigned long last_action;
	unsigned long last_slept;

	/*For P2P PS */
	struct rtl_p2p_ps_info p2p_ps_info;
	u8 pwr_mode;
	u8 smart_ps;

	/* wake up on line */
	u8 wo_wlan_mode;
	u8 arp_offload_enable;
	u8 gtk_offload_enable;
	/* Used for WOL, indicates the reason for waking event.*/
	u32 wakeup_reason;
	/* Record the last waking time for comparison with setting key. */
	u64 last_wakeup_time;
};

struct pwrctrl_priv {
	struct semaphore lock;
	volatile uint8_t rpwm; // requested power state for fw
	volatile uint8_t cpwm; // fw current power state. updated when 1. read from HCPWM 2. driver lowers power level
	volatile uint8_t tog; // toggling
	volatile uint8_t cpwm_tog; // toggling

	uint8_t	pwr_mode;
	uint8_t	smart_ps;
	uint8_t	bcn_ant_mode;

	u32	alives;
	struct work_struct cpwm_event;
#ifdef CONFIG_LPS_RPWM_TIMER
	uint8_t brpwmtimeout;
	struct work_struct rpwmtimeoutwi;
	_timer pwr_rpwm_timer;
#endif // CONFIG_LPS_RPWM_TIMER
	uint8_t	bpower_saving;

	uint8_t	b_hw_radio_off;
	uint8_t	reg_rfoff;
	uint8_t	reg_pdnmode; //powerdown mode
	u32	rfoff_reason;

	//RF OFF Level
	u32	cur_ps_level;
	u32	reg_rfps_level;


	uint 	ips_enter_cnts;
	uint 	ips_leave_cnts;

	uint8_t	ips_mode;
	uint8_t	ips_mode_req; // used to accept the mode setting request, will update to ipsmode later
	uint bips_processing;
	u32 ips_deny_time; /* will deny IPS when system time is smaller than this */
	uint8_t ps_processing; /* temporarily used to mark whether in rtw_ps_processor */

	uint8_t	bLeisurePs;
	uint8_t	LpsIdleCount;
	uint8_t	power_mgnt;
	bool fw_current_inpsmode;
	u32	DelayLPSLastTimeStamp;
	uint8_t 	btcoex_rfon;
	int32_t		pnp_current_pwr_state;
	uint8_t		pnp_bstop_trx;


	uint8_t		bInternalAutoSuspend;
	uint8_t		bInSuspend;
	uint8_t		bSupportRemoteWakeup;
	struct timer_list	pwr_state_check_timer;
	int		pwr_state_check_interval;
	uint8_t		pwr_state_check_cnts;

	int 		ps_flag;

	enum rf_pwrstate	rf_pwrstate;//cur power state
	//rt_rf_power_state 	current_rfpwrstate;
	enum rf_pwrstate	change_rfpwrstate;

	uint8_t		wepkeymask;
	uint8_t		bHWPwrPindetect;
	uint8_t		brfoffbyhw;
	unsigned long PS_BBRegBackup[PSBBREG_TOTALCNT];
};

struct rtl_locks {
#if 0
	/* mutex */
	struct mutex conf_mutex;
	struct mutex ps_mutex;

	/*spin lock */
	spinlock_t ips_lock;
	spinlock_t irq_th_lock;
	spinlock_t irq_pci_lock;
	spinlock_t tx_lock;
	spinlock_t h2c_lock;
	spinlock_t rf_ps_lock;
	spinlock_t rf_lock;
	spinlock_t lps_lock;
	spinlock_t waitq_lock;
	spinlock_t entry_list_lock;
#endif
	spinlock_t usb_lock;
#if 0

	/*FW clock change */
	spinlock_t fw_ps_lock;

	/*Dual mac*/
	spinlock_t cck_and_rw_pagea_lock;

	/*Easy concurrent*/
	spinlock_t check_sendpkt_lock;

	spinlock_t iqk_lock;
#endif
};

struct rtl_works {
#if 0
	struct ieee80211_hw *hw;

	/*timer */
	struct timer_list watchdog_timer;
	struct timer_list dualmac_easyconcurrent_retrytimer;
	struct timer_list fw_clockoff_timer;
	struct timer_list fast_antenna_training_timer;
	/*task */
	struct tasklet_struct irq_tasklet;
	struct tasklet_struct irq_prepare_bcn_tasklet;

	/*work queue */
	struct workqueue_struct *rtl_wq;
	struct delayed_work watchdog_wq;
	struct delayed_work ips_nic_off_wq;

	/* For SW LPS */
	struct delayed_work ps_work;
	struct delayed_work ps_rfon_wq;
	struct delayed_work fwevt_wq;

	struct work_struct lps_change_work;
	struct work_struct fill_h2c_cmd;
#endif
};

struct rtl_debug {
	u32 dbgp_type[DBGP_TYPE_MAX];
	int global_debuglevel;
	u64 global_debugcomponents;

	/* add for proc debug */
	struct proc_dir_entry *proc_dir;
	char proc_name[20];
};

struct rtl_priv {
	struct net_device *ndev;

	struct rtl_hal rtlhal;		/* Caution new Hal data */
	struct rtl_phy phy;
	struct rtl_efuse efuse;
	struct rtl_ps_ctl psc;
	struct rtl_dm dm;		/* Caution new dm data */
	struct rtl_hal_cfg *cfg;
	struct rtl_io io;
	struct dig_t dm_digtable;
	struct rate_adaptive ra;
	struct rtl_mac mac80211;

	struct false_alarm_statistics falsealm_cnt;
	struct rtl_usb_priv priv;
	struct rtl_locks locks;
	struct rtl_debug dbg;
	struct rtl_security sec;
	struct rtl_works works;

	struct _rtw_hal *HalData;
	bool initialized;


	/* Border */

	struct	mlme_priv mlmepriv;
	struct	mlme_ext_priv mlmeextpriv;
	struct	cmd_priv	cmdpriv;
	struct	evt_priv	evtpriv;
	//struct	io_queue	*pio_queue;
	struct	xmit_priv	xmitpriv;
	struct	recv_priv	recvpriv;
	struct	sta_priv	stapriv;
	struct	security_priv	securitypriv;
	struct	registry_priv	registrypriv;
	struct	pwrctrl_priv	pwrctrlpriv;

#ifdef CONFIG_AP_MODE
	struct	hostapd_priv	*phostapdpriv;
#endif

	u32	setband;

	int32_t	bDriverStopped;
	int32_t	bSurpriseRemoved;
	int32_t  bCardDisableWOHSM;

	u32	IsrContent;
	u32	ImrContent;

	uint8_t	EepromAddressSize;
	uint8_t	hw_init_completed;
	uint8_t	bDriverIsGoingToUnload;
	uint8_t	init_adpt_in_progress;
	uint8_t	bHaltInProgress;

	void *cmdThread;
	void *evtThread;
	void *xmitThread;
	void *recvThread;

	struct net_device_stats stats;
	struct iw_statistics iwstats;

	int net_closed;

	uint8_t bBTFWReady;
	uint8_t bReadPortCancel;
	uint8_t bWritePortCancel;
	//	Added by Albert 2012/10/26
	//	The driver will show up the desired channel number when this flag is 1.
	uint8_t bNotifyChannelChange;
#ifdef CONFIG_AUTOSUSPEND
	uint8_t	bDisableAutosuspend;
#endif
        uint8_t    fix_rate;

	unsigned char     in_cta_test;

};


struct rtl_stats {
	u8 psaddr[ETH_ALEN];
	u32 mac_time[2];
	s8 rssi;
	u8 signal;
	u8 noise;
	u8 rate;		/* hw desc rate */
	u8 received_channel;
	u8 control;
	u8 mask;
	u8 freq;
	u16 len;
	u64 tsf;
	u32 beacon_time;
	u8 nic_type;
	u16 length;
	u8 signalquality;	/*in 0-100 index. */
	/*
	 * Real power in dBm for this packet,
	 * no beautification and aggregation.
	 * */
	s32 recvsignalpower;
	s8 rxpower;		/*in dBm Translate from PWdB */
	u8 signalstrength;	/*in 0-100 index. */
	u16 hwerror:1;
	u16 crc:1;
	u16 icv:1;
	u16 shortpreamble:1;
	u16 antenna:1;
	u16 decrypted:1;
	u16 wakeup:1;
	u32 timestamp_low;
	u32 timestamp_high;
	bool shift;

	u8 rx_drvinfo_size;
	u8 rx_bufshift;
	bool isampdu;
	bool isfirst_ampdu;
	bool rx_is40Mhzpacket;
	u8 rx_packet_bw;
	u32 rx_pwdb_all;
	u8 rx_mimo_signalstrength[4];	/*in 0~100 index */
	s8 rx_mimo_signalquality[4];
	u8 rx_mimo_evm_dbm[4];
	u16 cfo_short[4];		/* per-path's Cfo_short */
	u16 cfo_tail[4];

	s8 rx_mimo_sig_qual[4];
	u8 rx_pwr[4]; /* per-path's pwdb */
	u8 rx_snr[4]; /* per-path's SNR */
	u8 bandwidth;
	u8 bt_coex_pwr_adjust;
	bool packet_matchbssid;
	bool is_cck;
	bool is_ht;
	bool packet_toself;
	bool packet_beacon;	/*for rssi */
	char cck_adc_pwdb[4];	/*for rx path selection */

	bool is_vht;
	bool is_short_gi;
	u8 vht_nss;

	u8 packet_report_type;

	u32 macid;
	u8 wake_match;
	u32 bt_rx_rssi_percentage;
	u32 macid_valid_entry[2];
};



struct rtl_hal_ops {
	/*
	 * New HAL functions with struct net_device  as first param
	 * this can be (hopefully)switched to struct ieee80211_hw
	 */
	int (*init_sw_vars) (struct rtl_priv *rtlpriv);
	void (*deinit_sw_vars) (struct rtl_priv *rtlpriv);

	void	(*set_hw_reg)(struct rtl_priv *rtlpriv, u8 variable,u8 *val);
	void	(*get_hw_reg)(struct rtl_priv *rtlpriv, u8 variable,u8 *val);
/*
 * 	ULLI from original rtlwifi-lib in wifi.h
 *
 * 	void (*fill_fake_txdesc) (struct ieee80211_hw *hw, u8 *pDesc,
 *				  u32 buffer_len, bool bIsPsPoll);
*/

	void	(*fill_fake_txdesc) (struct rtl_priv *rtlpriv, u8 *pDesc,
				     u32 BufferLen, u8 IsPsPoll, u8 IsBTQosNull);

	u32	(*get_bbreg)(struct rtl_priv *rtlpriv, u32 RegAddr, u32 BitMask);
	void	(*set_bbreg)(struct rtl_priv *rtlpriv, u32 RegAddr, u32 BitMask, u32 Data);
	u32	(*get_rfreg)(struct rtl_priv *rtlpriv, u32 eRFPath, u32 RegAddr, u32 BitMask);
	void	(*set_rfreg)(struct rtl_priv *rtlpriv, u32 eRFPath, u32 RegAddr, u32 BitMask, u32 Data);

	void	(*init_sw_leds)(struct rtl_priv *rtlpriv);
	void	(*deinit_sw_leds)(struct rtl_priv *rtlpriv);
	void	(*led_control) (struct rtl_priv *rtlpriv, enum led_ctl_mode ledcation);
	bool	(*radio_onoff_checking) (struct rtl_priv *rtlpriv, u8 *valid);
	int 	(*set_network_type) (struct rtl_priv *prtpriv, uint8_t val);
	u32	(*hw_init)(struct rtl_priv *rtlpriv);

	bool	(*phy_rf6052_config) (struct rtl_priv *rtlpriv);
	void 	(*phy_rf6052_set_cck_txpower) (struct rtl_priv *rtlpriv,
					       u8 *powerlevel);
#if 0
	void 	(*phy_rf6052_set_ofdm_txpower) (struct rtl_priv *rtlpriv,
					        u8 *ppowerlevel, u8 channel);
#endif
	void 	(*phy_rf6052_set_ofdm_txpower)(struct rtl_priv *rtlpriv,
					       u8 *pPowerLevelOFDM,
					       u8 *pPowerLevelBW20,
					       u8 *pPowerLevelBW40,
					       u8 Channel);
	bool (*config_bb_with_headerfile) (struct rtl_priv *rtlpriv,
					   u8 configtype);
	bool (*config_bb_with_pgheaderfile) (struct rtl_priv *rtlpriv,
					     u8 configtype);

	void (*phy_set_bw_mode_callback) (struct rtl_priv *rtlpriv);
	void (*fill_h2c_cmd) (struct rtl_priv *rtlpriv, u8 element_id,
			      u32 cmd_len, u8 *p_cmdbuffer);


	bool (*get_btc_status) (void);
	void (*dm_watchdog)(struct rtl_priv *rtlpriv);
	void (*set_bcn_reg)(struct rtl_priv *rtlpriv);

	/* Old HAL functions */

	u32	(*hal_deinit)(struct rtl_priv *rtlpriv);

	void	(*free_hal_data)(struct rtl_priv *rtlpriv);

	u32	(*inirp_deinit)(struct rtl_priv *rtlpriv);

	int32_t	(*init_recv_priv)(struct rtl_priv *rtlpriv);
	void	(*free_recv_priv)(struct rtl_priv *rtlpriv);

	void	(*dm_init)(struct rtl_priv *rtlpriv);
	void	(*read_chip_version)(struct rtl_priv *rtlpriv);

	void	(*init_default_value)(struct rtl_priv *rtlpriv);

	void	(*read_adapter_info)(struct rtl_priv *rtlpriv);

	void	(*enable_interrupt)(struct rtl_priv *rtlpriv);
	void	(*disable_interrupt)(struct rtl_priv *rtlpriv);

	void	(*set_bwmode_handler)(struct rtl_priv *rtlpriv, enum CHANNEL_WIDTH Bandwidth, uint8_t Offset);
	void	(*set_channel_handler)(struct rtl_priv *rtlpriv, uint8_t channel);
	void	(*set_chnl_bw_handler)(struct rtl_priv *rtlpriv, uint8_t channel, enum CHANNEL_WIDTH Bandwidth, uint8_t Offset40, uint8_t Offset80);

	void	(*hal_dm_watchdog)(struct rtl_priv *rtlpriv);

	void	(*Add_RateATid)(struct rtl_priv *rtlpriv, u32 bitmap, u8* arg, uint8_t rssi_level);

	int32_t	(*hal_xmit)(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe);
	int32_t (*mgnt_xmit)(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe);
	int32_t	(*hal_xmitframe_enqueue)(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe);
};


static void rtw_hal_fill_fake_txdesc (struct rtl_priv *rtlpriv, u8 *pDesc,
	u32 BufferLen, u8 IsPsPoll, u8 IsBTQosNull)
{
	rtlpriv->cfg->ops->fill_fake_txdesc(rtlpriv, pDesc, BufferLen, IsPsPoll, IsBTQosNull);
}




/*Only for transition between old (RTW) and new (rtlwifi-lib) API */

uint32_t rtl8812au_hal_deinit(struct rtl_priv *rtlpriv);
unsigned int rtl8812au_inirp_deinit(struct rtl_priv *rtlpriv);
void rtl8812au_init_default_value(struct rtl_priv *rtlpriv);
void _rtl8821au_read_adapter_info(struct rtl_priv *rtlpriv);

enum hardware_type {
	HARDWARE_TYPE_RTL8812AU,
	HARDWARE_TYPE_RTL8811AU,
	HARDWARE_TYPE_RTL8821U,

	HARDWARE_TYPE_MAX,
};

// RTL8812 Series
#define IS_HARDWARE_TYPE_8812AU(rtlhal)	\
	(rtlhal->hw_type == HARDWARE_TYPE_RTL8812AU)
#define IS_HARDWARE_TYPE_8812(rtlhal)	\
	(IS_HARDWARE_TYPE_8812AU(rtlhal))

// RTL8821 Series
#define IS_HARDWARE_TYPE_8811AU(rtlhal)	\
	(rtlhal->hw_type == HARDWARE_TYPE_RTL8811AU)
#define IS_HARDWARE_TYPE_8821U(rtlhal)	\
	((rtlhal->hw_type == HARDWARE_TYPE_RTL8821U) || \
       	(rtlhal->hw_type == HARDWARE_TYPE_RTL8811AU))
#define IS_HARDWARE_TYPE_8821(rtlhal)		\
	(IS_HARDWARE_TYPE_8821U(rtlhal))

#define	LDPC_HT_ENABLE_RX		BIT(0)
#define	LDPC_HT_ENABLE_TX		BIT(1)
#define	LDPC_HT_TEST_TX_ENABLE		BIT(2)
#define	LDPC_HT_CAP_TX			BIT(3)

#define	STBC_HT_ENABLE_RX		BIT(0)
#define	STBC_HT_ENABLE_TX		BIT(1)
#define	STBC_HT_TEST_TX_ENABLE		BIT(2)
#define	STBC_HT_CAP_TX			BIT(3)

static inline u8 get_rf_type(struct rtl_phy *rtlphy)
{
	return rtlphy->rf_type;
}

/* Not fully compatible */
#define rtl_usbdev(rtlpriv) (&(rtlpriv->priv.dev))


static inline u8 rtl_read_byte(struct rtl_priv *rtlpriv, u32 addr)
{
	return rtlpriv->io.read8_sync(rtlpriv, addr);
}

static inline u16 rtl_read_word(struct rtl_priv *rtlpriv, u32 addr)
{
	return rtlpriv->io.read16_sync(rtlpriv, addr);
}

static inline u32 rtl_read_dword(struct rtl_priv *rtlpriv, u32 addr)
{
	return rtlpriv->io.read32_sync(rtlpriv, addr);
}

static inline void rtl_write_byte(struct rtl_priv *rtlpriv, u32 addr, u8 val8)
{
	rtlpriv->io.write8_async(rtlpriv, addr, val8);
}

static inline void rtl_write_word(struct rtl_priv *rtlpriv, u32 addr, u16 val16)
{
	rtlpriv->io.write16_async(rtlpriv, addr, val16);
}

static inline void rtl_write_dword(struct rtl_priv *rtlpriv,
				   u32 addr, u32 val32)
{
	rtlpriv->io.write32_async(rtlpriv, addr, val32);
}

static inline void rtl_writeN(struct rtl_priv *rtlpriv,
				   u32 addr, void *data, u16 len)
{
	rtlpriv->io.writeN_sync(rtlpriv, addr, data, len);
}

static u32 rtl_get_bbreg(struct rtl_priv *rtlpriv, uint32_t RegAddr,
		         uint32_t BitMask)
{
	return rtlpriv->cfg->ops->get_bbreg(rtlpriv, RegAddr, BitMask);
}

static void rtl_set_bbreg(struct rtl_priv *rtlpriv, uint32_t RegAddr,
			  uint32_t BitMask, uint32_t Data)
{
	rtlpriv->cfg->ops->set_bbreg(rtlpriv, RegAddr, BitMask, Data);
}

static inline void rtl_set_rfreg(struct rtl_priv *rtlpriv,
				 enum radio_path rfpath, u32 regaddr,
				 u32 bitmask, u32 data)
{
	rtlpriv->cfg->ops->set_rfreg(rtlpriv, rfpath, regaddr, bitmask, data);
}

static inline u32 rtl_get_rfreg(struct rtl_priv *rtlpriv, enum radio_path rfpath,
		       u32 regaddr, u32 bitmask)
{
	return rtlpriv->cfg->ops->get_rfreg(rtlpriv, rfpath, regaddr,
						    bitmask);
}

/****************************************
	mem access macro define start
	Call endian free function when
	1. Read/write packet content.
	2. Before write integer to IO.
	3. After read integer from IO.
****************************************/
/* Convert little data endian to host ordering */
#define EF1BYTE(_val)		\
	((u8)(_val))
#define EF2BYTE(_val)		\
	(le16_to_cpu(_val))
#define EF4BYTE(_val)		\
	(le32_to_cpu(_val))

/* Read data from memory */
#define READEF1BYTE(_ptr)	\
	EF1BYTE(*((u8 *)(_ptr)))
/* Read le16 data from memory and convert to host ordering */
#define READEF2BYTE(_ptr)	\
	EF2BYTE(*(_ptr))
#define READEF4BYTE(_ptr)	\
	EF4BYTE(*(_ptr))

/* Write data to memory */
#define WRITEEF1BYTE(_ptr, _val)	\
	(*((u8 *)(_ptr))) = EF1BYTE(_val)
/* Write le16 data to memory in host ordering */
#define WRITEEF2BYTE(_ptr, _val)	\
	(*((u16 *)(_ptr))) = EF2BYTE(_val)
#define WRITEEF4BYTE(_ptr, _val)	\
	(*((u32 *)(_ptr))) = EF2BYTE(_val)

/* Create a bit mask
 * Examples:
 * BIT_LEN_MASK_32(0) => 0x00000000
 * BIT_LEN_MASK_32(1) => 0x00000001
 * BIT_LEN_MASK_32(2) => 0x00000003
 * BIT_LEN_MASK_32(32) => 0xFFFFFFFF
 */
#define BIT_LEN_MASK_32(__bitlen)	 \
	(0xFFFFFFFF >> (32 - (__bitlen)))
#define BIT_LEN_MASK_16(__bitlen)	 \
	(0xFFFF >> (16 - (__bitlen)))
#define BIT_LEN_MASK_8(__bitlen) \
	(0xFF >> (8 - (__bitlen)))

/* Create an offset bit mask
 * Examples:
 * BIT_OFFSET_LEN_MASK_32(0, 2) => 0x00000003
 * BIT_OFFSET_LEN_MASK_32(16, 2) => 0x00030000
 */
#define BIT_OFFSET_LEN_MASK_32(__bitoffset, __bitlen) \
	(BIT_LEN_MASK_32(__bitlen) << (__bitoffset))
#define BIT_OFFSET_LEN_MASK_16(__bitoffset, __bitlen) \
	(BIT_LEN_MASK_16(__bitlen) << (__bitoffset))
#define BIT_OFFSET_LEN_MASK_8(__bitoffset, __bitlen) \
	(BIT_LEN_MASK_8(__bitlen) << (__bitoffset))

/*Description:
 * Return 4-byte value in host byte ordering from
 * 4-byte pointer in little-endian system.
 */
#define LE_P4BYTE_TO_HOST_4BYTE(__pstart) \
	(EF4BYTE(*((__le32 *)(__pstart))))
#define LE_P2BYTE_TO_HOST_2BYTE(__pstart) \
	(EF2BYTE(*((__le16 *)(__pstart))))
#define LE_P1BYTE_TO_HOST_1BYTE(__pstart) \
	(EF1BYTE(*((u8 *)(__pstart))))

/*Description:
Translate subfield (continuous bits in little-endian) of 4-byte
value to host byte ordering.*/
#define LE_BITS_TO_4BYTE(__pstart, __bitoffset, __bitlen) \
	( \
		(LE_P4BYTE_TO_HOST_4BYTE(__pstart) >> (__bitoffset))  & \
		BIT_LEN_MASK_32(__bitlen) \
	)
#define LE_BITS_TO_2BYTE(__pstart, __bitoffset, __bitlen) \
	( \
		(LE_P2BYTE_TO_HOST_2BYTE(__pstart) >> (__bitoffset)) & \
		BIT_LEN_MASK_16(__bitlen) \
	)
#define LE_BITS_TO_1BYTE(__pstart, __bitoffset, __bitlen) \
	( \
		(LE_P1BYTE_TO_HOST_1BYTE(__pstart) >> (__bitoffset)) & \
		BIT_LEN_MASK_8(__bitlen) \
	)

/* Description:
 * Mask subfield (continuous bits in little-endian) of 4-byte value
 * and return the result in 4-byte value in host byte ordering.
 */
#define LE_BITS_CLEARED_TO_4BYTE(__pstart, __bitoffset, __bitlen) \
	( \
		LE_P4BYTE_TO_HOST_4BYTE(__pstart)  & \
		(~BIT_OFFSET_LEN_MASK_32(__bitoffset, __bitlen)) \
	)
#define LE_BITS_CLEARED_TO_2BYTE(__pstart, __bitoffset, __bitlen) \
	( \
		LE_P2BYTE_TO_HOST_2BYTE(__pstart) & \
		(~BIT_OFFSET_LEN_MASK_16(__bitoffset, __bitlen)) \
	)
#define LE_BITS_CLEARED_TO_1BYTE(__pstart, __bitoffset, __bitlen) \
	( \
		LE_P1BYTE_TO_HOST_1BYTE(__pstart) & \
		(~BIT_OFFSET_LEN_MASK_8(__bitoffset, __bitlen)) \
	)

/* Description:
 * Set subfield of little-endian 4-byte value to specified value.
 */
#define SET_BITS_TO_LE_4BYTE(__pstart, __bitoffset, __bitlen, __val) \
	*((u32 *)(__pstart)) = \
	( \
		LE_BITS_CLEARED_TO_4BYTE(__pstart, __bitoffset, __bitlen) | \
		((((u32)__val) & BIT_LEN_MASK_32(__bitlen)) << (__bitoffset)) \
	);
#define SET_BITS_TO_LE_2BYTE(__pstart, __bitoffset, __bitlen, __val) \
	*((u16 *)(__pstart)) = \
	( \
		LE_BITS_CLEARED_TO_2BYTE(__pstart, __bitoffset, __bitlen) | \
		((((u16)__val) & BIT_LEN_MASK_16(__bitlen)) << (__bitoffset)) \
	);
#define SET_BITS_TO_LE_1BYTE(__pstart, __bitoffset, __bitlen, __val) \
	*((u8 *)(__pstart)) = EF1BYTE \
	( \
		LE_BITS_CLEARED_TO_1BYTE(__pstart, __bitoffset, __bitlen) | \
		((((u8)__val) & BIT_LEN_MASK_8(__bitlen)) << (__bitoffset)) \
	);


/* ULLI : Hope this is an border, for old code  */





#define IQK_Matrix_Settings_NUM_92D	1+24+21

#define HP_THERMAL_NUM		8

struct dm_priv {
	// Add for Reading Initial Data Rate SEL Register 0x484 during watchdog. Using for fill tx desc. 2011.3.21 by Thomas
	uint8_t	INIDATA_RATE[32];
};

//
// <Roger_Notes> For RTL8723 WiFi PDn/GPIO polarity control configuration. 2010.10.08.
//

// For RTL8723 regulator mode. by tynli. 2011.01.14.
typedef enum _RT_REGULATOR_MODE {
	RT_SWITCHING_REGULATOR 	= 0,
	RT_LDO_REGULATOR 			= 1,
} RT_REGULATOR_MODE, *PRT_REGULATOR_MODE;

//
// Interface type.
//

typedef enum _RT_AMPDU_BRUST_MODE{
	RT_AMPDU_BRUST_NONE 		= 0,
	RT_AMPDU_BRUST_92D 		= 1,
	RT_AMPDU_BRUST_88E 		= 2,
	RT_AMPDU_BRUST_8812_4 	= 3,
	RT_AMPDU_BRUST_8812_8 	= 4,
	RT_AMPDU_BRUST_8812_12 	= 5,
	RT_AMPDU_BRUST_8812_15	= 6,
	RT_AMPDU_BRUST_8723B	 	= 7,
}RT_AMPDU_BRUST,*PRT_AMPDU_BRUST_MODE;



//###### duplicate code,will move to ODM #########

typedef struct _EDCA_TURBO_
{
	uint32_t	prv_traffic_idx; // edca turbo
}EDCA_T,*pEDCA_T;


//#endif

//
// 2011/09/22 MH Copy from SD4 defined structure. We use to support PHY DM integration.
//
struct _rtw_dm {
	//
	//	Add for different team use temporarily
	//
	struct rtl_priv *	rtlpriv;		// For CE/NIC team
	// WHen you use rtlpriv or priv pointer, you must make sure the pointer is ready.
	bool			odm_ready;

	uint64_t			NumQryPhyStatusAll; 	//CCK + OFDM
	uint64_t			LastNumQryPhyStatusAll;
	uint64_t			RxPWDBAve;
	uint64_t			RxPWDBAve_final;
	bool			MPDIG_2G; 		//off MPDIG
	u8			Times_2G;

//------ ODM HANDLE, DRIVER NEEDS NOT TO HOOK------//
//------ ODM HANDLE, DRIVER NEEDS NOT TO HOOK------//

//--------REMOVED COMMON INFO----------//
	//u8				PseudoMacPhyMode;
	//bool			*BTCoexist;
	//bool			PseudoBtCoexist;
	//u8				OPMode;
	//bool			bAPMode;
	//bool			bClientMode;
	//bool			bAdHocMode;
	//bool			bSlaveOfDMSP;
//--------REMOVED COMMON INFO----------//


//1  COMMON INFORMATION

	//
	// Init Value
	//
//-----------HOOK BEFORE REG INIT-----------//
	// ODM Support Ability DIG/RATR/TX_PWR_TRACK/ ¡K¡K = 1/2/3/¡K
	// ODM PCIE/USB/SDIO = 1/2/3
	u8			SupportInterface;
	// Cut Version TestChip/A-cut/B-cut... = 0/1/2/3/...
	u8			CutVersion;

	// with external TRSW  NO/Yes = 0/1
	u8			PatchID; //Customer ID

//-----------HOOK BEFORE REG INIT-----------//

	//
	// Dynamic Value
	//
//--------- POINTER REFERENCE-----------//

	u8			u8_temp;

	// Security mode Open/WEP/AES/TKIP = 0/1/2/3
	u8			*pSecurity;
	// Common info for 92D DMSP

	//u8			*pAidMap;
//------------CALL BY VALUE-------------//
	u8          InterfaceIndex; // Add for 92D  dual MAC: 0--Mac0 1--Mac1
	bool			bOneEntryOnly;
	// Common info for BTDM
//------------CALL BY VALUE-------------//
	u8			RSSI_A;
	u8			RSSI_B;
	u8			RxRate;
	uint32_t			TH_H;
	uint32_t			TH_L;
	uint32_t			IGI_Base;
	uint32_t			IGI_target;
	bool			ForceEDCCA;
	u8			AdapEn_RSSI;

	//
	// 2012/02/14 MH Add to share 88E ra with other SW team.
	// We need to colelct all support abilit to a proper area.
	//
	bool				RaSupport88E;

	// Define ...........

	// Different Team independt structure??

	//
	//TX_RTP_CMN		TX_retrpo;
	//TX_RTP_88E		TX_retrpo;
	//TX_RTP_8195		TX_retrpo;

	//
	//ODM Structure
	//

	EDCA_T		DM_EDCA_Table;
	uint32_t		WMMEDCA_BE;
	// Copy from SD4 structure
	//
	// ==================================================
	//

	//common
	//u8    PSD_Report_RXHP[80];   // Add By Gary
	//u8    PSD_func_flag;               // Add By Gary
	//for DIG
	//u8		binitialized; // for dm_initial_gain_Multi_STA use.
	//for Antenna diversity
	//u8	AntDivCfg;// 0:OFF , 1:ON, 2:by efuse
	//struct sta_info *RSSI_target;

	//PSD

	//
	// TX power tracking
	//

	//
	// Dynamic ATC switch
	//
	int				CFO_tail[2];
	uint32_t			packetCount;
};

struct _rtw_hal {
	//current WIFI_PHY values
	enum wireless_mode CurrentWirelessMode;

	u16	CustomerID;
	u32	ReceiveConfig;

	//rf_ctrl
	uint8_t	DefaultInitialGain[4];

	uint8_t	bTXPowerDataReadFromEEPORM;
	//uint8_t				EfuseMap[2][HWSET_MAX_SIZE_JAGUAR];


//	uint8_t	Regulation2_4G;
//	uint8_t	Regulation5G;
	//
	// TX power by rate table at most 4RF path.
	// The register is
	//
	// VHT TX power by rate off setArray =
	// Band:-2G&5G = 0 / 1
	// RF: at most 4*4 = ABCD=0/1/2/3
	// CCK=0 OFDM=1/2 HT-MCS 0-15=3/4/56 VHT=7/8/9/10/11
	//
	uint8_t	TxPwrByRateTable;
 	uint8_t	TxPwrByRateBand;



	uint8_t	LegacyHTTxPowerDiff;// Legacy to HT rate power diff

	// Read/write are allow for following hardware information variables
	u32	CCKTxPowerLevelOriginalOffset;

	uint8_t	bIQKInitialized;
	bool		bLCKInProgress;

	bool		bChnlBWInitialzed;

	uint8_t	TxPowerTrackControl; //for mp mode, turn off txpwrtracking as default
	uint8_t	b1x1RecvCombine;	// for 1T1R receive combining

	u32	AcParam_BE; //Original parameter for BE, use for EDCA turbo.

	u32	RfRegChnlVal[2];

	//RDG enable
	bool	 bRDGEnable;

	uint8_t	fw_ractrl;
	uint8_t	RegTxPause;
	// Beacon function related global variable.
	uint8_t	RegFwHwTxQCtrl;
	uint8_t	RegReg542;
	uint8_t	RegCR_1;
	u16	RegRRSR;

	uint8_t	CurAntenna;
	uint8_t	AntDivCfg;

	uint8_t	FwRsvdPageStartOffset; //2010.06.23. Added by tynli. Reserve page start offset except beacon in TxQ.

	// 2010/12/10 MH Add for USB aggreation mode dynamic shceme.
	bool		UsbRxHighSpeedMode;

	uint8_t	AMPDUDensity;

	// Auto FSM to Turn On, include clock, isolation, power control for MAC only
	uint8_t	bMacPwrCtrlOn;

	RT_AMPDU_BRUST		AMPDUBurstMode; //92C maybe not use, but for compile successfully

	uint8_t	C2hArray[16];
	uint8_t	UsbTxAggDescNum;

	u16	HwRxPageSize;				// Hardware setting
	u32	MaxUsbRxAggBlock;

	uint8_t	UsbRxAggBlockCount;			// USB Block count. Block size is 512-byte in hight speed and 64-byte in full speed
	uint8_t	UsbRxAggBlockTimeout;
	uint8_t	UsbRxAggPageCount;			// 8192C DMA page count
	uint8_t	UsbRxAggPageTimeout;

	uint8_t	RegAcUsbDmaSize;
	uint8_t	RegAcUsbDmaTime;



	struct dm_priv	dmpriv;
	struct _rtw_dm odmpriv;
};

static inline struct _rtw_hal *GET_HAL_DATA(struct rtl_priv *priv)
{
	return priv->HalData;
}

/* ULLI : we must get rid of this studip code */

static inline uint8_t rtw_get_oper_ch(struct rtl_priv *rtlpriv)
{
	return rtl_usbdev(rtlpriv)->oper_channel;
}

static inline void rtw_set_oper_ch(struct rtl_priv *rtlpriv, uint8_t ch)
{
	rtl_usbdev(rtlpriv)->oper_channel = ch;
}

#endif
