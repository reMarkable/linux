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
#ifndef __RTW_MLME_EXT_H_
#define __RTW_MLME_EXT_H_

#include <rtw_cmd.h>

//	Commented by Albert 20101105
//	Increase the SURVEY_TO value from 100 to 150  ( 100ms to 150ms )
//	The Realtek 8188CE SoftAP will spend around 100ms to send the probe response after receiving the probe request.
//	So, this driver tried to extend the dwell time for each scanning channel.
//	This will increase the chance to receive the probe response from SoftAP.

#define SURVEY_TO		(100)
#define REAUTH_TO		(300) //(50)
#define REASSOC_TO		(300) //(50)
//#define DISCONNECT_TO	(3000)
#define ADDBA_TO			(2000)

#define LINKED_TO (1) //unit:2 sec, 1x2=2 sec

#define REAUTH_LIMIT	(4)
#define REASSOC_LIMIT	(4)
#define READDBA_LIMIT	(2)

	#define ROAMING_LIMIT	8
//#define	IOCMD_REG0		0x10250370
//#define	IOCMD_REG1		0x10250374
//#define	IOCMD_REG2		0x10250378

//#define	FW_DYNAMIC_FUN_SWITCH	0x10250364

//#define	WRITE_BB_CMD		0xF0000001
//#define	SET_CHANNEL_CMD	0xF3000000
//#define	UPDATE_RA_CMD	0xFD0000A2

// ====== ODM_ABILITY_E ========
// BB ODM section BIT 0-15
#define	DYNAMIC_BB_DIG				BIT(0)
#define	DYNAMIC_BB_RA_MASK			BIT(1)
#define	DYNAMIC_BB_DYNAMIC_TXPWR	BIT(2)
#define	DYNAMIC_BB_BB_FA_CNT			BIT(3)

#define 	DYNAMIC_BB_RSSI_MONITOR		BIT(4)
#define 	DYNAMIC_BB_CCK_PD			BIT(5)
#define 	DYNAMIC_BB_ANT_DIV			BIT(6)
#define 	DYNAMIC_BB_PWR_SAVE			BIT(7)
#define 	DYNAMIC_BB_PWR_TRAIN			BIT(8)
#define 	DYNAMIC_BB_RATE_ADAPTIVE		BIT(9)
#define 	DYNAMIC_BB_PATH_DIV			BIT(10)
#define 	DYNAMIC_BB_PSD				BIT(11)

// MAC DM section BIT 16-23
#define 	DYNAMIC_MAC_EDCA_TURBO		BIT(16)
#define 	DYNAMIC_MAC_EARLY_MODE		BIT(17)

// RF ODM section BIT 24-31
#define 	DYNAMIC_RF_TX_PWR_TRACK		BIT(24)
#define 	DYNAMIC_RF_RX_GAIN_TRACK		BIT(25)
#define 	DYNAMIC_RF_CALIBRATION		BIT(26)

#define 	DYNAMIC_ALL_FUNC_ENABLE		0xFFFFFFF

#define _HW_STATE_NOLINK_		0x00
#define _HW_STATE_ADHOC_		0x01
#define _HW_STATE_STATION_ 	0x02
#define _HW_STATE_AP_			0x03


#define		_1M_RATE_	0
#define		_2M_RATE_	1
#define		_5M_RATE_	2
#define		_11M_RATE_	3
#define		_6M_RATE_	4
#define		_9M_RATE_	5
#define		_12M_RATE_	6
#define		_18M_RATE_	7
#define		_24M_RATE_	8
#define		_36M_RATE_	9
#define		_48M_RATE_	10
#define		_54M_RATE_	11


extern unsigned char RTW_WPA_OUI[];
extern unsigned char WMM_OUI[];
extern unsigned char WPS_OUI[];
extern unsigned char WFD_OUI[];
extern unsigned char P2P_OUI[];

extern unsigned char WMM_INFO_OUI[];
extern unsigned char WMM_PARA_OUI[];


//
// Channel Plan Type.
// Note:
//	We just add new channel plan when the new channel plan is different from any of the following
//	channel plan.
//	If you just wnat to customize the acitions(scan period or join actions) about one of the channel plan,
//	customize them in RT_CHANNEL_INFO in the RT_CHANNEL_LIST.
//
typedef enum _RT_CHANNEL_DOMAIN
{
	//===== old channel plan mapping =====//
	RT_CHANNEL_DOMAIN_FCC = 0x00,
	RT_CHANNEL_DOMAIN_IC = 0x01,
	RT_CHANNEL_DOMAIN_ETSI = 0x02,
	RT_CHANNEL_DOMAIN_SPAIN = 0x03,
	RT_CHANNEL_DOMAIN_FRANCE = 0x04,
	RT_CHANNEL_DOMAIN_MKK = 0x05,
	RT_CHANNEL_DOMAIN_MKK1 = 0x06,
	RT_CHANNEL_DOMAIN_ISRAEL = 0x07,
	RT_CHANNEL_DOMAIN_TELEC = 0x08,
	RT_CHANNEL_DOMAIN_GLOBAL_DOAMIN = 0x09,
	RT_CHANNEL_DOMAIN_WORLD_WIDE_13 = 0x0A,
	RT_CHANNEL_DOMAIN_TAIWAN = 0x0B,
	RT_CHANNEL_DOMAIN_CHINA = 0x0C,
	RT_CHANNEL_DOMAIN_SINGAPORE_INDIA_MEXICO = 0x0D,
	RT_CHANNEL_DOMAIN_KOREA = 0x0E,
	RT_CHANNEL_DOMAIN_TURKEY = 0x0F,
	RT_CHANNEL_DOMAIN_JAPAN = 0x10,
	RT_CHANNEL_DOMAIN_FCC_NO_DFS = 0x11,
	RT_CHANNEL_DOMAIN_JAPAN_NO_DFS = 0x12,
	RT_CHANNEL_DOMAIN_WORLD_WIDE_5G = 0x13,
	RT_CHANNEL_DOMAIN_TAIWAN_NO_DFS = 0x14,

	//===== new channel plan mapping, (2GDOMAIN_5GDOMAIN) =====//
	RT_CHANNEL_DOMAIN_WORLD_NULL = 0x20,
	RT_CHANNEL_DOMAIN_ETSI1_NULL = 0x21,
	RT_CHANNEL_DOMAIN_FCC1_NULL = 0x22,
	RT_CHANNEL_DOMAIN_MKK1_NULL = 0x23,
	RT_CHANNEL_DOMAIN_ETSI2_NULL = 0x24,
	RT_CHANNEL_DOMAIN_FCC1_FCC1 = 0x25,
	RT_CHANNEL_DOMAIN_WORLD_ETSI1 = 0x26,
	RT_CHANNEL_DOMAIN_MKK1_MKK1 = 0x27,
	RT_CHANNEL_DOMAIN_WORLD_KCC1 = 0x28,
	RT_CHANNEL_DOMAIN_WORLD_FCC2 = 0x29,
	RT_CHANNEL_DOMAIN_WORLD_FCC3 = 0x30,
	RT_CHANNEL_DOMAIN_WORLD_FCC4 = 0x31,
	RT_CHANNEL_DOMAIN_WORLD_FCC5 = 0x32,
	RT_CHANNEL_DOMAIN_WORLD_FCC6 = 0x33,
	RT_CHANNEL_DOMAIN_FCC1_FCC7 = 0x34,
	RT_CHANNEL_DOMAIN_WORLD_ETSI2 = 0x35,
	RT_CHANNEL_DOMAIN_WORLD_ETSI3 = 0x36,
	RT_CHANNEL_DOMAIN_MKK1_MKK2 = 0x37,
	RT_CHANNEL_DOMAIN_MKK1_MKK3 = 0x38,
	RT_CHANNEL_DOMAIN_FCC1_NCC1 = 0x39,
	RT_CHANNEL_DOMAIN_FCC1_NCC2 = 0x40,
	RT_CHANNEL_DOMAIN_GLOBAL_DOAMIN_2G = 0x41,
	//===== Add new channel plan above this line===============//
	RT_CHANNEL_DOMAIN_MAX,
	RT_CHANNEL_DOMAIN_REALTEK_DEFINE = 0x7F,
}RT_CHANNEL_DOMAIN, *PRT_CHANNEL_DOMAIN;

typedef enum _RT_CHANNEL_DOMAIN_2G
{
	RT_CHANNEL_DOMAIN_2G_WORLD = 0x00,		//Worldwird 13
	RT_CHANNEL_DOMAIN_2G_ETSI1 = 0x01,		//Europe
	RT_CHANNEL_DOMAIN_2G_FCC1 = 0x02,		//US
	RT_CHANNEL_DOMAIN_2G_MKK1 = 0x03,		//Japan
	RT_CHANNEL_DOMAIN_2G_ETSI2 = 0x04,		//France
	RT_CHANNEL_DOMAIN_2G_NULL = 0x05,
	//===== Add new channel plan above this line===============//
	RT_CHANNEL_DOMAIN_2G_MAX,
}RT_CHANNEL_DOMAIN_2G, *PRT_CHANNEL_DOMAIN_2G;

typedef enum _RT_CHANNEL_DOMAIN_5G
{
	RT_CHANNEL_DOMAIN_5G_NULL = 0x00,
	RT_CHANNEL_DOMAIN_5G_ETSI1 = 0x01,		//Europe
	RT_CHANNEL_DOMAIN_5G_ETSI2 = 0x02,		//Australia, New Zealand
	RT_CHANNEL_DOMAIN_5G_ETSI3 = 0x03,		//Russia
	RT_CHANNEL_DOMAIN_5G_FCC1 = 0x04,		//US
	RT_CHANNEL_DOMAIN_5G_FCC2 = 0x05,		//FCC o/w DFS Channels
	RT_CHANNEL_DOMAIN_5G_FCC3 = 0x06,		//India, Mexico
	RT_CHANNEL_DOMAIN_5G_FCC4 = 0x07,		//Venezuela
	RT_CHANNEL_DOMAIN_5G_FCC5 = 0x08,		//China
	RT_CHANNEL_DOMAIN_5G_FCC6 = 0x09,		//Israel
	RT_CHANNEL_DOMAIN_5G_FCC7_IC1 = 0x0A,	//US, Canada
	RT_CHANNEL_DOMAIN_5G_KCC1 = 0x0B,		//Korea
	RT_CHANNEL_DOMAIN_5G_MKK1 = 0x0C,		//Japan
	RT_CHANNEL_DOMAIN_5G_MKK2 = 0x0D,		//Japan (W52, W53)
	RT_CHANNEL_DOMAIN_5G_MKK3 = 0x0E,		//Japan (W56)
	RT_CHANNEL_DOMAIN_5G_NCC1 = 0x0F,		//Taiwan
	RT_CHANNEL_DOMAIN_5G_NCC2 = 0x10,		//Taiwan o/w DFS
	//===== Add new channel plan above this line===============//
	//===== Driver Self Defined =====//
	RT_CHANNEL_DOMAIN_5G_FCC = 0x11,
	RT_CHANNEL_DOMAIN_5G_JAPAN_NO_DFS = 0x12,
	RT_CHANNEL_DOMAIN_5G_FCC4_NO_DFS = 0x13,
	RT_CHANNEL_DOMAIN_5G_MAX,
}RT_CHANNEL_DOMAIN_5G, *PRT_CHANNEL_DOMAIN_5G;

typedef struct _RT_CHANNEL_PLAN_MAP
{
	unsigned char	Index2G;
	unsigned char	Index5G;
}RT_CHANNEL_PLAN_MAP, *PRT_CHANNEL_PLAN_MAP;

enum Associated_AP
{
	atherosAP	= 0,
	broadcomAP	= 1,
	ciscoAP		= 2,
	marvellAP	= 3,
	ralinkAP	= 4,
	realtekAP	= 5,
	airgocapAP 	= 6,
	unknownAP	= 7,
	maxAP,
};

typedef enum _HT_IOT_PEER
{
	HT_IOT_PEER_UNKNOWN 			= 0,
	HT_IOT_PEER_REALTEK 			= 1,
	HT_IOT_PEER_REALTEK_92SE 		= 2,
	HT_IOT_PEER_BROADCOM 		= 3,
	HT_IOT_PEER_RALINK 			= 4,
	HT_IOT_PEER_ATHEROS 			= 5,
	HT_IOT_PEER_CISCO 				= 6,
	HT_IOT_PEER_MERU 				= 7,
	HT_IOT_PEER_MARVELL 			= 8,
	HT_IOT_PEER_REALTEK_SOFTAP 	= 9,// peer is RealTek SOFT_AP, by Bohn, 2009.12.17
	HT_IOT_PEER_SELF_SOFTAP 		= 10, // Self is SoftAP
	HT_IOT_PEER_AIRGO 				= 11,
	HT_IOT_PEER_INTEL 				= 12,
	HT_IOT_PEER_RTK_APCLIENT 		= 13,
	HT_IOT_PEER_REALTEK_81XX 		= 14,
	HT_IOT_PEER_REALTEK_WOW 		= 15,
	HT_IOT_PEER_REALTEK_JAGUAR_BCUTAP = 16,
	HT_IOT_PEER_REALTEK_JAGUAR_CCUTAP = 17,
	HT_IOT_PEER_MAX 				= 18
}HT_IOT_PEER_E, *PHTIOT_PEER_E;


enum SCAN_STATE
{
	SCAN_DISABLE = 0,
	SCAN_START = 1,
	SCAN_TXNULL = 2,
	SCAN_PROCESS = 3,
	SCAN_COMPLETE = 4,
	SCAN_STATE_MAX,
};

struct mlme_handler {
	unsigned int   num;
	char* str;
	unsigned int (*func)(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
};

struct action_handler {
	unsigned int   num;
	char* str;
	unsigned int (*func)(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
};

struct	ss_res
{
	int	state;
	int	bss_cnt;
	int	channel_idx;
	int	scan_mode;
	uint8_t ssid_num;
	uint8_t ch_num;
	NDIS_802_11_SSID ssid[RTW_SSID_SCAN_AMOUNT];
	struct rtw_ieee80211_channel ch[RTW_CHANNEL_SCAN_AMOUNT];
};

//#define AP_MODE				0x0C
//#define STATION_MODE	0x08
//#define AD_HOC_MODE		0x04
//#define NO_LINK_MODE	0x00

#define 	WIFI_FW_NULL_STATE			_HW_STATE_NOLINK_
#define	WIFI_FW_STATION_STATE		_HW_STATE_STATION_
#define	WIFI_FW_AP_STATE				_HW_STATE_AP_
#define	WIFI_FW_ADHOC_STATE			_HW_STATE_ADHOC_

#define	WIFI_FW_AUTH_NULL			0x00000100
#define	WIFI_FW_AUTH_STATE			0x00000200
#define	WIFI_FW_AUTH_SUCCESS			0x00000400

#define	WIFI_FW_ASSOC_STATE			0x00002000
#define	WIFI_FW_ASSOC_SUCCESS		0x00004000

#define	WIFI_FW_LINKING_STATE		(WIFI_FW_AUTH_NULL | WIFI_FW_AUTH_STATE | WIFI_FW_AUTH_SUCCESS |WIFI_FW_ASSOC_STATE)


struct FW_Sta_Info
{
	struct sta_info	*psta;
	u32	status;
	u32	rx_pkt;
	u32	retry;
	NDIS_802_11_RATES_EX  SupportedRates;
};

/*
 * Usage:
 * When one iface acted as AP mode and the other iface is STA mode and scanning,
 * it should switch back to AP's operating channel periodically.
 * Parameters info:
 * When the driver scanned RTW_SCAN_NUM_OF_CH channels, it would switch back to AP's operating channel for
 * RTW_STAY_AP_CH_MILLISECOND * SURVEY_TO milliseconds.
 * Example:
 * For chip supports 2.4G + 5GHz and AP mode is operating in channel 1,
 * RTW_SCAN_NUM_OF_CH is 8, RTW_STAY_AP_CH_MILLISECOND is 3 and SURVEY_TO is 100.
 * When it's STA mode gets set_scan command,
 * it would
 * 1. Doing the scan on channel 1.2.3.4.5.6.7.8
 * 2. Back to channel 1 for 300 milliseconds
 * 3. Go through doing site survey on channel 9.10.11.36.40.44.48.52
 * 4. Back to channel 1 for 300 milliseconds
 * 5. ... and so on, till survey done.
 */

struct mlme_ext_info
{
	u32	state;
	u32	reauth_count;
	u32	reassoc_count;
	u32	link_count;
	u32	auth_seq;
	u32	auth_algo;	// 802.11 auth, could be open, shared, auto
	u32	authModeToggle;
	u32	enc_algo;//encrypt algorithm;
	u32	key_index;	// this is only valid for legendary wep, 0~3 for key id.
	u32	iv;
	uint8_t	chg_txt[128];
	u16	aid;
	u16	capability;
	uint8_t	assoc_AP_vendor;
	uint8_t	slotTime;
	uint8_t	preamble_mode;
	uint8_t	WMM_enable;
	uint8_t	ERP_enable;
	uint8_t	ERP_IE;
	uint8_t	HT_enable;
	uint8_t	HT_caps_enable;
	uint8_t	HT_info_enable;
	uint8_t	HT_protection;
	uint8_t	turboMode_cts2self;
	uint8_t	turboMode_rtsen;
	uint8_t	SM_PS;
	uint8_t	agg_enable_bitmap;
	uint8_t	ADDBA_retry_count;
	uint8_t	candidate_tid_bitmap;
	uint8_t	dialogToken;
	// Accept ADDBA Request
	bool bAcceptAddbaReq;
	uint8_t	bwmode_updated;
	uint8_t	hidden_ssid_mode;
	uint8_t	VHT_enable;

	struct ADDBA_request		ADDBA_req;
	struct WMM_para_element	WMM_param;
	struct HT_caps_element	HT_caps;
	struct HT_info_element		HT_info;
	WLAN_BSSID_EX			network;//join network or bss_network, if in ap mode, it is the same to cur_network.network
	struct FW_Sta_Info		FW_sta_info[NUM_STA];
};

// The channel information about this channel including joining, scanning, and power constraints.
typedef struct _RT_CHANNEL_INFO
{
	uint8_t				ChannelNum;		// The channel number.
	RT_SCAN_TYPE	ScanType;		// Scan type such as passive or active scan.
	//u16				ScanPeriod;		// Listen time in millisecond in this channel.
	//int32_t				MaxTxPwrDbm;	// Max allowed tx power.
	//u32				ExInfo;			// Extended Information for this channel.
}RT_CHANNEL_INFO, *PRT_CHANNEL_INFO;

int rtw_ch_set_search_ch(RT_CHANNEL_INFO *ch_set, const u32 ch);

// P2P_MAX_REG_CLASSES - Maximum number of regulatory classes
#define P2P_MAX_REG_CLASSES 10

// P2P_MAX_REG_CLASS_CHANNELS - Maximum number of channels per regulatory class
#define P2P_MAX_REG_CLASS_CHANNELS 20

//  struct p2p_channels - List of supported channels
struct p2p_channels {
	// struct p2p_reg_class - Supported regulatory class
	struct p2p_reg_class {
		// reg_class - Regulatory class (IEEE 802.11-2007, Annex J)
		uint8_t reg_class;

		// channel - Supported channels
		uint8_t channel[P2P_MAX_REG_CLASS_CHANNELS];

		// channels - Number of channel entries in use
		size_t channels;
	} reg_class[P2P_MAX_REG_CLASSES];

	// reg_classes - Number of reg_class entries in use
	size_t reg_classes;
};

struct p2p_oper_class_map {
	enum hw_mode {IEEE80211G,IEEE80211A} mode;
	uint8_t op_class;
	uint8_t min_chan;
	uint8_t max_chan;
	uint8_t inc;
	enum { BW20, BW40PLUS, BW40MINUS } bw;
};

struct mlme_ext_priv
{
	struct rtl_priv	*rtlpriv;
	uint8_t	mlmeext_init;
	atomic_t event_seq;
	u16	mgnt_seq;

	//struct fw_priv 	fwpriv;

	unsigned char	cur_channel;
	unsigned char	cur_bwmode;
	unsigned char	cur_ch_offset;//PRIME_CHNL_OFFSET
	unsigned char	cur_wireless_mode;	// NETWORK_TYPE

	unsigned char	max_chan_nums;
	RT_CHANNEL_INFO		channel_set[MAX_CHANNEL_NUM];
	struct p2p_channels channel_list;
	unsigned char	basicrate[NumRates];
	unsigned char	datarate[NumRates];

	struct ss_res		sitesurvey_res;
	struct mlme_ext_info	mlmext_info;//for sta/adhoc mode, including current scanning/connecting/connected related info.
                                                     //for ap mode, network includes ap's cap_info
	struct timer_list	survey_timer;
	struct timer_list	link_timer;
	//_timer		ADDBA_timer;
	u16			chan_scan_time;

	uint8_t	scan_abort;
	uint8_t	tx_rate; // TXRATE when USERATE is set.

	u32	retry; //retry for issue probereq

	u64 TSFValue;

#ifdef CONFIG_AP_MODE
	unsigned char bstart_bss;
#endif

#ifdef CONFIG_80211D
	uint8_t update_channel_plan_by_ap_done;
#endif
	//recv_decache check for Action_public frame
	uint8_t action_public_dialog_token;
	u16 	 action_public_rxseq;

#ifdef DBG_FIXED_CHAN
	uint8_t fixed_chan;
#endif

};

int init_mlme_ext_priv(struct rtl_priv* rtlpriv);
int init_hw_mlme_ext(struct rtl_priv *rtlpriv);
void free_mlme_ext_priv (struct mlme_ext_priv *pmlmeext);
extern void init_mlme_ext_timer(struct rtl_priv *rtlpriv);
extern void init_addba_retry_timer(struct rtl_priv *rtlpriv, struct sta_info *psta);
extern struct xmit_frame *alloc_mgtxmitframe(struct xmit_priv *pxmitpriv);
struct xmit_frame *alloc_mgtxmitframe_once(struct xmit_priv *pxmitpriv);

//void fill_fwpriv(struct rtl_priv * rtlpriv, struct fw_priv *pfwpriv);

unsigned char networktype_to_raid(struct rtl_priv *rtlpriv,unsigned char network_type);
unsigned char networktype_to_raid_ex(struct rtl_priv *rtlpriv,unsigned char network_type);

uint8_t judge_network_type(struct rtl_priv *rtlpriv, unsigned char *rate, int ratelen);
void get_rate_set(struct rtl_priv *rtlpriv, unsigned char *pbssrate, int *bssrate_len);
void UpdateBrateTbl(struct rtl_priv *rtlpriv,uint8_t *mBratesOS);
void UpdateBrateTblForSoftAP(uint8_t *bssrateset, u32 bssratelen);
void change_band_update_ie(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork);

//void Set_NETYPE1_MSR(struct rtl_priv *rtlpriv, uint8_t type);
//void Set_NETYPE0_MSR(struct rtl_priv *rtlpriv, uint8_t type);

uint8_t	rtw_get_center_ch(uint8_t channel, uint8_t chnl_bw, uint8_t chnl_offset);
void set_channel_bwmode(struct rtl_priv *rtlpriv, unsigned char channel, unsigned char channel_offset, unsigned short bwmode);
void SelectChannel(struct rtl_priv *rtlpriv, unsigned char channel);
void SetBWMode(struct rtl_priv *rtlpriv, unsigned short bwmode, unsigned char channel_offset);

unsigned int decide_wait_for_beacon_timeout(unsigned int bcn_interval);

int allocate_fw_sta_entry(struct rtl_priv *rtlpriv);
void flush_all_cam_entry(struct rtl_priv *rtlpriv);

bool IsLegal5GChannel(struct rtl_priv *rtlpriv, uint8_t channel);

void site_survey(struct rtl_priv *rtlpriv);
uint8_t collect_bss_info(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame, WLAN_BSSID_EX *bssid);
void update_network(WLAN_BSSID_EX *dst, WLAN_BSSID_EX *src, struct rtl_priv * rtlpriv, bool update_ie);

int get_bsstype(unsigned short capability);
u8* get_my_bssid(WLAN_BSSID_EX *pnetwork);
u16 get_beacon_interval(WLAN_BSSID_EX *bss);

int is_client_associated_to_ap(struct rtl_priv *rtlpriv);
int is_client_associated_to_ibss(struct rtl_priv *rtlpriv);
int is_IBSS_empty(struct rtl_priv *rtlpriv);

unsigned char check_assoc_AP(uint8_t *pframe, uint len);

int WMM_param_handler(struct rtl_priv *rtlpriv, PNDIS_802_11_VARIABLE_IEs	pIE);
void WMMOnAssocRsp(struct rtl_priv *rtlpriv);

void HT_caps_handler(struct rtl_priv *rtlpriv, PNDIS_802_11_VARIABLE_IEs pIE);
void HT_info_handler(struct rtl_priv *rtlpriv, PNDIS_802_11_VARIABLE_IEs pIE);
void HTOnAssocRsp(struct rtl_priv *rtlpriv);

void ERP_IE_handler(struct rtl_priv *rtlpriv, PNDIS_802_11_VARIABLE_IEs pIE);
void VCS_update(struct rtl_priv *rtlpriv, struct sta_info *psta);

void update_beacon_info(struct rtl_priv *rtlpriv, uint8_t *pframe, uint len, struct sta_info *psta);
int rtw_check_bcn_info(struct rtl_priv *rtlpriv, uint8_t *pframe, u32 packet_len);
#ifdef CONFIG_DFS
void process_csa_ie(struct rtl_priv *rtlpriv, uint8_t *pframe, uint len);
#endif //CONFIG_DFS
void update_IOT_info(struct rtl_priv *rtlpriv);
void update_capinfo(struct rtl_priv *rtlpriv, u16 updateCap);
void update_wireless_mode(struct rtl_priv * rtlpriv);
void update_tx_basic_rate(struct rtl_priv *rtlpriv, uint8_t modulation);
void update_bmc_sta_support_rate(struct rtl_priv *rtlpriv, u32 mac_id);
int update_sta_support_rate(struct rtl_priv *rtlpriv, u8* pvar_ie, uint var_ie_len, int cam_idx);

//for sta/adhoc mode
void update_sta_info(struct rtl_priv *rtlpriv, struct sta_info *psta);
unsigned int update_basic_rate(unsigned char *ptn, unsigned int ptn_sz);
unsigned int update_supported_rate(unsigned char *ptn, unsigned int ptn_sz);
unsigned int update_MCS_rate(struct HT_caps_element *pHT_caps);
void Update_RA_Entry(struct rtl_priv *rtlpriv, struct sta_info *psta);
void set_sta_rate(struct rtl_priv *rtlpriv, struct sta_info *psta);

unsigned int receive_disconnect(struct rtl_priv *rtlpriv, unsigned char *MacAddr, unsigned short reason);

unsigned char get_highest_rate_idx(u32 mask);
int support_short_GI(struct rtl_priv *rtlpriv, struct HT_caps_element *pHT_caps);
unsigned int is_ap_in_tkip(struct rtl_priv *rtlpriv);
unsigned int is_ap_in_wep(struct rtl_priv *rtlpriv);
unsigned int should_forbid_n_rate(struct rtl_priv * rtlpriv);

extern uint rtw_get_camid(uint macid);
extern void rtw_alloc_macid(struct rtl_priv *rtlpriv, struct sta_info *psta);
extern void rtw_release_macid(struct rtl_priv *rtlpriv, struct sta_info *psta);

void report_join_res(struct rtl_priv *rtlpriv, int res);
void report_survey_event(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
void report_surveydone_event(struct rtl_priv *rtlpriv);
void report_del_sta_event(struct rtl_priv *rtlpriv, unsigned char* MacAddr, unsigned short reason);
void report_add_sta_event(struct rtl_priv *rtlpriv, unsigned char* MacAddr, int cam_idx);

void beacon_timing_control(struct rtl_priv *rtlpriv);
extern uint8_t set_tx_beacon_cmd(struct rtl_priv*rtlpriv);
unsigned int setup_beacon_frame(struct rtl_priv *rtlpriv, unsigned char *beacon_frame);
void update_mgnt_tx_rate(struct rtl_priv *rtlpriv, uint8_t rate);
void update_mgntframe_attrib(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib);
void update_mgntframe_attrib_addr(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe);
void dump_mgntframe(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe);
int32_t dump_mgntframe_and_wait(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe, int timeout_ms);
int32_t dump_mgntframe_and_wait_ack(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe);

void issue_beacon(struct rtl_priv *rtlpriv, int timeout_ms);
void issue_probersp(struct rtl_priv *rtlpriv, unsigned char *da, uint8_t is_valid_p2p_probereq);
void issue_assocreq(struct rtl_priv *rtlpriv);
void issue_asocrsp(struct rtl_priv *rtlpriv, unsigned short status, struct sta_info *pstat, int pkt_type);
void issue_auth(struct rtl_priv *rtlpriv, struct sta_info *psta, unsigned short status);
void issue_probereq(struct rtl_priv *rtlpriv, NDIS_802_11_SSID *pssid, uint8_t *da);
int32_t issue_probereq_ex(struct rtl_priv *rtlpriv, NDIS_802_11_SSID *pssid, u8* da, int try_cnt, int wait_ms);
int issue_nulldata(struct rtl_priv *rtlpriv, unsigned char *da, unsigned int power_mode, int try_cnt, int wait_ms);
int issue_qos_nulldata(struct rtl_priv *rtlpriv, unsigned char *da, u16 tid, int try_cnt, int wait_ms);
int issue_deauth(struct rtl_priv *rtlpriv, unsigned char *da, unsigned short reason);
int issue_deauth_ex(struct rtl_priv *rtlpriv, uint8_t *da, unsigned short reason, int try_cnt, int wait_ms);
void issue_action_spct_ch_switch(struct rtl_priv *rtlpriv, uint8_t *ra, uint8_t new_ch, uint8_t ch_offset);
void issue_action_BA(struct rtl_priv *rtlpriv, unsigned char *raddr, unsigned char action, unsigned short status);
unsigned int send_delba(struct rtl_priv *rtlpriv, uint8_t initiator, uint8_t *addr);
unsigned int send_beacon(struct rtl_priv *rtlpriv);

void start_clnt_assoc(struct rtl_priv *rtlpriv);
void start_clnt_auth(struct rtl_priv* rtlpriv);
void start_clnt_join(struct rtl_priv* rtlpriv);
void start_create_ibss(struct rtl_priv* rtlpriv);

unsigned int OnAssocReq(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAssocRsp(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnProbeReq(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnProbeRsp(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int DoReserved(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnBeacon(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAtim(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnDisassoc(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAuth(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAuthClient(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnDeAuth(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAction(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);

unsigned int on_action_spct(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAction_qos(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAction_dls(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAction_back(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int on_action_public(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAction_ht(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAction_wmm(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
unsigned int OnAction_p2p(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);


void mlmeext_joinbss_event_callback(struct rtl_priv *rtlpriv, int join_res);
void mlmeext_sta_del_event_callback(struct rtl_priv *rtlpriv);
void mlmeext_sta_add_event_callback(struct rtl_priv *rtlpriv, struct sta_info *psta);

void linked_status_chk(struct rtl_priv *rtlpriv);

void survey_timer_hdl (struct rtl_priv *rtlpriv);
void link_timer_hdl (struct rtl_priv *rtlpriv);
void addba_timer_hdl(struct sta_info *psta);
//void reauth_timer_hdl(struct rtl_priv *rtlpriv);
//void reassoc_timer_hdl(struct rtl_priv *rtlpriv);

#define set_survey_timer(mlmeext, ms) \
	do { \
		/*DBG_871X("%s set_survey_timer(%p, %d)\n", __FUNCTION__, (mlmeext), (ms));*/ \
		_set_timer(&(mlmeext)->survey_timer, (ms)); \
	} while(0)

#define set_link_timer(mlmeext, ms) \
	do { \
		/*DBG_871X("%s set_link_timer(%p, %d)\n", __FUNCTION__, (mlmeext), (ms));*/ \
		_set_timer(&(mlmeext)->link_timer, (ms)); \
	} while(0)

extern int cckrates_included(unsigned char *rate, int ratelen);
extern int cckratesonly_included(unsigned char *rate, int ratelen);

extern void process_addba_req(struct rtl_priv *rtlpriv, uint8_t *paddba_req, uint8_t *addr);

extern void update_TSF(struct mlme_ext_priv *pmlmeext, uint8_t *pframe, uint len);
extern void correct_TSF(struct rtl_priv *rtlpriv, struct mlme_ext_priv *pmlmeext);

struct cmd_hdl {
	uint	parmsize;
	uint8_t (*h2cfuns)(struct rtl_priv *rtlpriv, uint8_t *pbuf);
};


uint8_t read_macreg_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t write_macreg_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t read_bbreg_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t write_bbreg_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t read_rfreg_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t write_rfreg_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);


uint8_t NULL_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t join_cmd_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t disconnect_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t createbss_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t setopmode_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t sitesurvey_cmd_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t setauth_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t setkey_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t set_stakey_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t set_assocsta_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t del_assocsta_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t add_ba_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf);

uint8_t mlme_evt_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf);
uint8_t tx_beacon_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf);
uint8_t set_ch_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf);
uint8_t set_csa_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf);	//Kurt: Handling DFS channel switch announcement ie.
uint8_t tdls_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf);


#define GEN_DRV_CMD_HANDLER(size, cmd)	{size, &cmd ## _hdl},
#define GEN_MLME_EXT_HANDLER(size, cmd)	{size, cmd},

#ifdef _RTW_CMD_C_

struct cmd_hdl wlancmds[] = {
	GEN_MLME_EXT_HANDLER(sizeof (struct joinbss_parm), join_cmd_hdl)		/*  1 */
	GEN_MLME_EXT_HANDLER(sizeof (struct disconnect_parm), disconnect_hdl)		/*  2 */
	GEN_MLME_EXT_HANDLER(sizeof (struct createbss_parm), createbss_hdl)		/*  3 */
	GEN_MLME_EXT_HANDLER(sizeof (struct setopmode_parm), setopmode_hdl)		/*  4 */
	GEN_MLME_EXT_HANDLER(sizeof (struct sitesurvey_parm), sitesurvey_cmd_hdl)	/*  5 */
	GEN_MLME_EXT_HANDLER(sizeof (struct setauth_parm), setauth_hdl)			/*  6 */
	GEN_MLME_EXT_HANDLER(sizeof (struct setkey_parm), setkey_hdl)			/*  7 */
	GEN_MLME_EXT_HANDLER(sizeof (struct set_stakey_parm), set_stakey_hdl)		/*  8 */
	GEN_MLME_EXT_HANDLER(sizeof(struct addBaReq_parm), add_ba_hdl)			/*  9 */
	GEN_MLME_EXT_HANDLER(sizeof(struct set_ch_parm), set_ch_hdl)			/* 10 */
	GEN_MLME_EXT_HANDLER(sizeof(struct Tx_Beacon_param), tx_beacon_hdl)		/* 11 */
	GEN_MLME_EXT_HANDLER(0, mlme_evt_hdl)						/* 12 */
	GEN_MLME_EXT_HANDLER(0, rtw_drvextra_cmd_hdl)					/* 13 */
	GEN_MLME_EXT_HANDLER(sizeof(struct SetChannelSwitch_param), set_csa_hdl)	/* 15 */
};

#endif

struct C2HEvent_Header
{

#ifdef __LITTLE_ENDIAN

	unsigned int len:16;
	unsigned int ID:8;
	unsigned int seq:8;

#elif defined(__BIG_ENDIAN)

	unsigned int seq:8;
	unsigned int ID:8;
	unsigned int len:16;

#else

#  error "Must be LITTLE or BIG Endian"

#endif

	unsigned int rsvd;

};

void rtw_dummy_event_callback(struct rtl_priv *rtlpriv , uint8_t *pbuf);
void rtw_fwdbg_event_callback(struct rtl_priv *rtlpriv , uint8_t *pbuf);

enum rtw_c2h_event
{
	GEN_EVT_CODE(_Read_MACREG)=0, /*0*/
	GEN_EVT_CODE(_Read_BBREG),
 	GEN_EVT_CODE(_Read_RFREG),
 	GEN_EVT_CODE(_Read_EEPROM),
 	GEN_EVT_CODE(_Read_EFUSE),
	GEN_EVT_CODE(_Read_CAM),			/*5*/
 	GEN_EVT_CODE(_Get_BasicRate),
 	GEN_EVT_CODE(_Get_DataRate),
 	GEN_EVT_CODE(_Survey),	 /*8*/
 	GEN_EVT_CODE(_SurveyDone),	 /*9*/

 	GEN_EVT_CODE(_JoinBss) , /*10*/
 	GEN_EVT_CODE(_AddSTA),
 	GEN_EVT_CODE(_DelSTA),
 	GEN_EVT_CODE(_AtimDone) ,
 	GEN_EVT_CODE(_TX_Report),
	GEN_EVT_CODE(_CCX_Report),			/*15*/
 	GEN_EVT_CODE(_DTM_Report),
 	GEN_EVT_CODE(_TX_Rate_Statistics),
 	GEN_EVT_CODE(_C2HLBK),
 	GEN_EVT_CODE(_FWDBG),
	GEN_EVT_CODE(_C2HFEEDBACK),               /*20*/
	GEN_EVT_CODE(_ADDBA),
	GEN_EVT_CODE(_C2HBCN),
	GEN_EVT_CODE(_ReportPwrState),		//filen: only for PCIE, USB
	GEN_EVT_CODE(_CloseRF),				//filen: only for PCIE, work around ASPM
 	MAX_C2HEVT
};


#ifdef _RTW_MLME_EXT_C_

static struct fwevent wlanevents[] =
{
	{0, rtw_dummy_event_callback}, 	/*0*/
	{0, NULL},
	{0, NULL},
	{0, NULL},
	{0, NULL},
	{0, NULL},
	{0, NULL},
	{0, NULL},
	{0, &rtw_survey_event_callback},		/*8*/
	{sizeof (struct surveydone_event), &rtw_surveydone_event_callback},	/*9*/

	{0, &rtw_joinbss_event_callback},		/*10*/
	{sizeof(struct stassoc_event), &rtw_stassoc_event_callback},
	{sizeof(struct stadel_event), &rtw_stadel_event_callback},
	{0, &rtw_atimdone_event_callback},
	{0, rtw_dummy_event_callback},
	{0, NULL},	/*15*/
	{0, NULL},
	{0, NULL},
	{0, NULL},
	{0, rtw_fwdbg_event_callback},
	{0, NULL},	 /*20*/
	{0, NULL},
	{0, NULL},
	{0, &rtw_cpwm_event_callback},
};

#endif//_RTL8192C_CMD_C_

#endif

