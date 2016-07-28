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
#ifndef __RTW_MLME_H_
#define __RTW_MLME_H_

#include <rtw_rf.h>
#include <rtw_ht.h>
#include <rtw_vht.h>

#define	MAX_BSS_CNT	128
//#define   MAX_JOIN_TIMEOUT	2000
//#define   MAX_JOIN_TIMEOUT	2500
#define   MAX_JOIN_TIMEOUT	6500

//	Commented by Albert 20101105
//	Increase the scanning timeout because of increasing the SURVEY_TO value.

#define 	SCANNING_TIMEOUT 	8000

#define	SCAN_INTERVAL	(30) // unit:2sec, 30*2=60sec

#ifdef PALTFORM_OS_WINCE
#define	SCANQUEUE_LIFETIME 12000000 // unit:us
#else
#define	SCANQUEUE_LIFETIME 20000 // 20sec, unit:msec
#endif

#define WIFI_NULL_STATE		0x00000000
#define WIFI_ASOC_STATE		0x00000001		// Under Linked state...
#define WIFI_REASOC_STATE	0x00000002
#define WIFI_SLEEP_STATE	0x00000004
#define WIFI_STATION_STATE	0x00000008
#define	WIFI_AP_STATE			0x00000010
#define	WIFI_ADHOC_STATE		0x00000020
#define WIFI_ADHOC_MASTER_STATE	0x00000040
#define WIFI_UNDER_LINKING	0x00000080

#define WIFI_UNDER_WPS			0x00000100
//#define	WIFI_UNDER_CMD			0x00000200
//#define	WIFI_UNDER_P2P			0x00000400
#define	WIFI_STA_ALIVE_CHK_STATE	0x00000400
#define	WIFI_SITE_MONITOR			0x00000800		//to indicate the station is under site surveying
#ifdef WDS
#define	WIFI_WDS				0x00001000
#define	WIFI_WDS_RX_BEACON	0x00002000		// already rx WDS AP beacon
#endif
#ifdef AUTO_CONFIG
#define	WIFI_AUTOCONF			0x00004000
#define	WIFI_AUTOCONF_IND	0x00008000
#endif

/*
// ========== P2P Section Start ===============
#define	WIFI_P2P_LISTEN_STATE		0x00010000
#define	WIFI_P2P_GROUP_FORMATION_STATE		0x00020000
// ========== P2P Section End ===============
*/

//#ifdef UNDER_MPTEST
#define	WIFI_MP_STATE							0x00010000
#define	WIFI_MP_CTX_BACKGROUND				0x00020000	// in continous tx background
#define	WIFI_MP_CTX_ST						0x00040000	// in continous tx with single-tone
#define	WIFI_MP_CTX_BACKGROUND_PENDING	0x00080000	// pending in continous tx background due to out of skb
#define	WIFI_MP_CTX_CCK_HW					0x00100000	// in continous tx
#define	WIFI_MP_CTX_CCK_CS					0x00200000	// in continous tx with carrier suppression
#define   WIFI_MP_LPBK_STATE					0x00400000
//#endif

//#define _FW_UNDER_CMD		WIFI_UNDER_CMD
#define _FW_UNDER_LINKING	WIFI_UNDER_LINKING
#define _FW_LINKED			WIFI_ASOC_STATE
#define _FW_UNDER_SURVEY	WIFI_SITE_MONITOR


enum dot11AuthAlgrthmNum {
 dot11AuthAlgrthm_Open = 0,
 dot11AuthAlgrthm_Shared,
 dot11AuthAlgrthm_8021X,
 dot11AuthAlgrthm_Auto,
 dot11AuthAlgrthm_WAPI,
 dot11AuthAlgrthm_MaxNum
};

// Scan type including active and passive scan.
typedef enum _RT_SCAN_TYPE
{
	SCAN_PASSIVE,
	SCAN_ACTIVE,
	SCAN_MIX,
}RT_SCAN_TYPE, *PRT_SCAN_TYPE;

enum  _BAND
{
	GHZ24_50 = 0,
	GHZ_50,
	GHZ_24,
};

enum DriverInterface {
	DRIVER_WEXT =  1,
	DRIVER_CFG80211 = 2
};

enum SCAN_RESULT_TYPE
{
	SCAN_RESULT_P2P_ONLY = 0,		//	Will return all the P2P devices.
	SCAN_RESULT_ALL = 1,			//	Will return all the scanned device, include AP.
	SCAN_RESULT_WFD_TYPE = 2		//	Will just return the correct WFD device.
									//	If this device is Miracast sink device, it will just return all the Miracast source devices.
};

/*

there are several "locks" in mlme_priv,
since mlme_priv is a shared resource between many threads,
like ISR/Call-Back functions, the OID handlers, and even timer functions.


Each struct __queue has its own locks, already.
Other items are protected by mlme_priv.lock.

To avoid possible dead lock, any thread trying to modifiying mlme_priv
SHALL not lock up more than one locks at a time!

*/


#define traffic_threshold	10
#define	traffic_scan_period	500

struct sitesurvey_ctrl {
	u64	last_tx_pkts;
	uint	last_rx_pkts;
	int	traffic_busy;
	struct timer_list sitesurvey_ctrl_timer;
};

typedef struct _RT_LINK_DETECT_T{
	u32				NumTxOkInPeriod;
	u32				NumRxOkInPeriod;
	u32				NumRxUnicastOkInPeriod;
	bool			bBusyTraffic;
	bool			bTxBusyTraffic;
	bool			bRxBusyTraffic;
	bool			bHigherBusyTraffic; // For interrupt migration purpose.
	bool			bHigherBusyRxTraffic; // We may disable Tx interrupt according as Rx traffic.
	bool			bHigherBusyTxTraffic; // We may disable Tx interrupt according as Tx traffic.
}RT_LINK_DETECT_T, *PRT_LINK_DETECT_T;

struct profile_info {
	uint8_t	ssidlen;
	uint8_t	ssid[ WLAN_SSID_MAXLEN ];
	uint8_t	peermac[ ETH_ALEN ];
};

struct tx_invite_req_info{
	uint8_t					token;
	uint8_t					benable;
	uint8_t					go_ssid[ WLAN_SSID_MAXLEN ];
	uint8_t					ssidlen;
	uint8_t					go_bssid[ ETH_ALEN ];
	uint8_t					peer_macaddr[ ETH_ALEN ];
	uint8_t					operating_ch;	//	This information will be set by using the p2p_set op_ch=x
	uint8_t					peer_ch;		//	The listen channel for peer P2P device

};

struct tx_invite_resp_info{
	uint8_t					token;	//	Used to record the dialog token of p2p invitation request frame.
};

struct tx_provdisc_req_info{
	u16					wps_config_method_request;	//	Used when sending the provisioning request frame
	u16					peer_channel_num[2];		//	The channel number which the receiver stands.
	NDIS_802_11_SSID	ssid;
	uint8_t					peerDevAddr[ ETH_ALEN ];		//	Peer device address
	uint8_t					peerIFAddr[ ETH_ALEN ];		//	Peer interface address
	uint8_t					benable;					//	This provision discovery request frame is trigger to send or not
};

struct rx_provdisc_req_info{	//When peer device issue prov_disc_req first, we should store the following informations
	uint8_t					peerDevAddr[ ETH_ALEN ];		//	Peer device address
	uint8_t					strconfig_method_desc_of_prov_disc_req[4];	//	description for the config method located in the provisioning discovery request frame.
																	//	The UI must know this information to know which config method the remote p2p device is requiring.
};

struct tx_nego_req_info{
	u16					peer_channel_num[2];		//	The channel number which the receiver stands.
	uint8_t					peerDevAddr[ ETH_ALEN ];		//	Peer device address
	uint8_t					benable;					//	This negoitation request frame is trigger to send or not
};

struct group_id_info{
	uint8_t					go_device_addr[ ETH_ALEN ];	//	The GO's device address of this P2P group
	uint8_t					ssid[ WLAN_SSID_MAXLEN ];	//	The SSID of this P2P group
};

struct scan_limit_info{
	uint8_t					scan_op_ch_only;			//	When this flag is set, the driver should just scan the operation channel
	uint8_t					operation_ch[2];				//	Store the operation channel of invitation request frame
};

struct wifidirect_info{
	struct rtl_priv*				rtlpriv;
	struct timer_list				find_phase_timer;
	struct timer_list				restore_p2p_state_timer;

	//	Used to do the scanning. After confirming the peer is availalble, the driver transmits the P2P frame to peer.
	struct timer_list				pre_tx_scan_timer;
	struct timer_list				reset_ch_sitesurvey;
	struct timer_list				reset_ch_sitesurvey2;	//	Just for resetting the scan limit function by using p2p nego
	struct tx_provdisc_req_info	tx_prov_disc_info;
	struct rx_provdisc_req_info rx_prov_disc_info;
	struct tx_invite_req_info	invitereq_info;
	struct profile_info			profileinfo[ P2P_MAX_PERSISTENT_GROUP_NUM ];	//	Store the profile information of persistent group
	struct tx_invite_resp_info	inviteresp_info;
	struct tx_nego_req_info	nego_req_info;
	struct group_id_info		groupid_info;	//	Store the group id information when doing the group negotiation handshake.
	struct scan_limit_info		rx_invitereq_info;	//	Used for get the limit scan channel from the Invitation procedure
	struct scan_limit_info		p2p_info;		//	Used for get the limit scan channel from the P2P negotiation handshake
	enum P2P_ROLE			role;
	enum P2P_STATE			pre_p2p_state;
	enum P2P_STATE			p2p_state;
	uint8_t 						device_addr[ETH_ALEN];	//	The device address should be the mac address of this device.
	uint8_t						interface_addr[ETH_ALEN];
	uint8_t						social_chan[4];
	uint8_t						listen_channel;
	uint8_t						operating_channel;
	uint8_t						listen_dwell;		//	This value should be between 1 and 3
	uint8_t						support_rate[8];
	uint8_t						p2p_wildcard_ssid[P2P_WILDCARD_SSID_LEN];
	uint8_t						intent;		//	should only include the intent value.
	uint8_t						p2p_peer_interface_addr[ ETH_ALEN ];
	uint8_t						p2p_peer_device_addr[ ETH_ALEN ];
	uint8_t						peer_intent;	//	Included the intent value and tie breaker value.
	uint8_t						device_name[ WPS_MAX_DEVICE_NAME_LEN ];	//	Device name for displaying on searching device screen
	uint8_t						device_name_len;
	uint8_t						profileindex;	//	Used to point to the index of profileinfo array
	uint8_t						peer_operating_ch;
	uint8_t						find_phase_state_exchange_cnt;
	u16						device_password_id_for_nego;	//	The device password ID for group negotation
	uint8_t						negotiation_dialog_token;
	uint8_t						nego_ssid[ WLAN_SSID_MAXLEN ];	//	SSID information for group negotitation
	uint8_t						nego_ssidlen;
	uint8_t 						p2p_group_ssid[WLAN_SSID_MAXLEN];
	uint8_t 						p2p_group_ssid_len;
	uint8_t						persistent_supported;		//	Flag to know the persistent function should be supported or not.
														//	In the Sigma test, the Sigma will provide this enable from the sta_set_p2p CAPI.
														//	0: disable
														//	1: enable
	uint8_t						session_available;			//	Flag to set the WFD session available to enable or disable "by Sigma"
														//	In the Sigma test, the Sigma will disable the session available by using the sta_preset CAPI.
														//	0: disable
														//	1: enable

	uint8_t						wfd_tdls_enable;			//	Flag to enable or disable the TDLS by WFD Sigma
														//	0: disable
														//	1: enable
	uint8_t						wfd_tdls_weaksec;			//	Flag to enable or disable the weak security function for TDLS by WFD Sigma
														//	0: disable
														//	In this case, the driver can't issue the tdsl setup request frame.
														//	1: enable
														//	In this case, the driver can issue the tdls setup request frame
														//	even the current security is weak security.

	enum	P2P_WPSINFO		ui_got_wps_info;			//	This field will store the WPS value (PIN value or PBC) that UI had got from the user.
	u16						supported_wps_cm;			//	This field describes the WPS config method which this driver supported.
														//	The value should be the combination of config method defined in page104 of WPS v2.0 spec.
	uint						channel_list_attr_len;		//	This field will contain the length of body of P2P Channel List attribute of group negotitation response frame.
	uint8_t						channel_list_attr[100];		//	This field will contain the body of P2P Channel List attribute of group negotitation response frame.
														//	We will use the channel_cnt and channel_list fields when constructing the group negotitation confirm frame.
	uint8_t						driver_interface;			//	Indicate DRIVER_WEXT or DRIVER_CFG80211
};

struct tdls_ss_record{	//signal strength record
	uint8_t		macaddr[ETH_ALEN];
	uint8_t		RxPWDBAll;
	uint8_t		is_tdls_sta;	// true: direct link sta, false: else
};

struct tdls_info{
	uint8_t					ap_prohibited;
	uint					setup_state;
	uint8_t					sta_cnt;
	uint8_t					sta_maximum;	// 1:tdls sta is equal (NUM_STA-1), reach max direct link number; 0: else;
	struct tdls_ss_record	ss_record;
	uint8_t					macid_index;	//macid entry that is ready to write
	uint8_t					clear_cam;	//cam entry that is trying to clear, using it in direct link teardown
	uint8_t					ch_sensing;
	uint8_t					cur_channel;
	uint8_t					candidate_ch;
	uint8_t					collect_pkt_num[MAX_CHANNEL_NUM];
	spinlock_t				cmd_lock;
	spinlock_t				hdl_lock;
	uint8_t					watchdog_count;
	uint8_t					dev_discovered;		//WFD_TDLS: for sigma test
	uint8_t					enable;
};

struct mlme_priv {

	spinlock_t	lock;
	int	fw_state;	//shall we protect this variable? maybe not necessarily...
	uint8_t bScanInProcess;
	uint8_t	to_join; //flag

	uint8_t	*nic_hdl;

	uint8_t	not_indic_disco;
	struct list_head		*pscanned;
	struct __queue	free_bss_pool;
	struct __queue	scanned_queue;
	uint8_t		*free_bss_buf;
	u32	num_of_scanned;

	NDIS_802_11_SSID	assoc_ssid;
	uint8_t	assoc_bssid[6];

	struct wlan_network	cur_network;

	//uint wireless_mode; no used, remove it

	u32	scan_interval;

	struct timer_list assoc_timer;

	uint assoc_by_bssid;
	uint assoc_by_rssi;

	struct timer_list scan_to_timer; // driver itself handles scan_timeout status.
	u32 scan_start_time; // used to evaluate the time spent in scanning

	struct qos_priv qospriv;

	/* Number of non-HT AP/stations */
	int num_sta_no_ht;

	/* Number of HT AP/stations 20 MHz */
	//int num_sta_ht_20mhz;


	int num_FortyMHzIntolerant;

	struct ht_priv	htpriv;

	struct vht_priv	vhtpriv;

	RT_LINK_DETECT_T	LinkDetectInfo;
	struct timer_list	dynamic_chk_timer; //dynamic/periodic check timer

 	uint8_t 	key_mask; //use for ips to set wep key after ips_leave
	uint8_t	acm_mask; // for wmm acm mask
	uint8_t	ChannelPlan;
	RT_SCAN_TYPE 	scan_mode; // active: 1, passive: 0

	//uint8_t probereq_wpsie[MAX_WPS_IE_LEN];//added in probe req
	//int probereq_wpsie_len;
	uint8_t *wps_probe_req_ie;
	u32 wps_probe_req_ie_len;

#if defined (CONFIG_AP_MODE)
	/* Number of associated Non-ERP stations (i.e., stations using 802.11b
	 * in 802.11g BSS) */
	int num_sta_non_erp;

	/* Number of associated stations that do not support Short Slot Time */
	int num_sta_no_short_slot_time;

	/* Number of associated stations that do not support Short Preamble */
	int num_sta_no_short_preamble;

	int olbc; /* Overlapping Legacy BSS Condition */

	/* Number of HT associated stations that do not support greenfield */
	int num_sta_ht_no_gf;

	/* Number of associated non-HT stations */
	//int num_sta_no_ht;

	/* Number of HT associated stations 20 MHz */
	int num_sta_ht_20mhz;

	/* Overlapping BSS information */
	int olbc_ht;

	u16 ht_op_mode;

	uint8_t *assoc_req;
	u32 assoc_req_len;
	uint8_t *assoc_rsp;
	u32 assoc_rsp_len;

	uint8_t *wps_beacon_ie;
	//uint8_t *wps_probe_req_ie;
	uint8_t *wps_probe_resp_ie;
	uint8_t *wps_assoc_resp_ie; // for CONFIG_IOCTL_CFG80211, this IE could include p2p ie / wfd ie

	u32 wps_beacon_ie_len;
	//u32 wps_probe_req_ie_len;
	u32 wps_probe_resp_ie_len;
	u32 wps_assoc_resp_ie_len; // for CONFIG_IOCTL_CFG80211, this IE len could include p2p ie / wfd ie

	uint8_t *p2p_beacon_ie;
	uint8_t *p2p_probe_req_ie;
	uint8_t *p2p_probe_resp_ie;
	uint8_t *p2p_go_probe_resp_ie; //for GO
	uint8_t *p2p_assoc_req_ie;

	u32 p2p_beacon_ie_len;
	u32 p2p_probe_req_ie_len;
	u32 p2p_probe_resp_ie_len;
	u32 p2p_go_probe_resp_ie_len; //for GO
	u32 p2p_assoc_req_ie_len;

	spinlock_t	bcn_update_lock;
	uint8_t		update_bcn;


#endif //#if defined (CONFIG_AP_MODE)
};

#ifdef CONFIG_AP_MODE

struct hostapd_priv
{
	struct rtl_priv *rtlpriv;
};

extern int hostapd_mode_init(struct rtl_priv *rtlpriv);
extern void hostapd_mode_unload(struct rtl_priv *rtlpriv);
#endif


extern void rtw_joinbss_event_prehandle(struct rtl_priv *rtlpriv, uint8_t *pbuf);
extern void rtw_survey_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf);
extern void rtw_surveydone_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf);
extern void rtw_joinbss_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf);
extern void rtw_stassoc_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf);
extern void rtw_stadel_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf);
extern void rtw_atimdone_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf);
extern void rtw_cpwm_event_callback(struct rtl_priv *rtlpriv, uint8_t *pbuf);

extern void rtw_join_timeout_handler(RTW_TIMER_HDL_ARGS);
extern void _rtw_scan_timeout_handler(RTW_TIMER_HDL_ARGS);

int event_thread(void *context);

extern void rtw_free_network_queue(struct rtl_priv *rtlpriv,uint8_t isfreeall);
extern int rtw_init_mlme_priv(struct rtl_priv *rtlpriv);// (struct mlme_priv *pmlmepriv);

extern void rtw_free_mlme_priv (struct mlme_priv *pmlmepriv);


extern int rtw_select_and_join_from_scanned_queue(struct mlme_priv *pmlmepriv);
extern int rtw_set_key(struct rtl_priv *rtlpriv,struct security_priv *psecuritypriv,int keyid, uint8_t set_tx);
extern int rtw_set_auth(struct rtl_priv *rtlpriv,struct security_priv *psecuritypriv);

__inline static uint8_t *get_bssid(struct mlme_priv *pmlmepriv)
{	//if sta_mode:pmlmepriv->cur_network.network.MacAddress=> bssid
	// if adhoc_mode:pmlmepriv->cur_network.network.MacAddress=> ibss mac address
	return pmlmepriv->cur_network.network.MacAddress;
}

__inline static int check_fwstate(struct mlme_priv *pmlmepriv, int state)
{
	if (pmlmepriv->fw_state & state)
		return true;

	return false;
}

__inline static int get_fwstate(struct mlme_priv *pmlmepriv)
{
	return pmlmepriv->fw_state;
}

/*
 * No Limit on the calling context,
 * therefore set it to be the critical section...
 *
 * ### NOTE:#### (!!!!)
 * MUST TAKE CARE THAT BEFORE CALLING THIS FUNC, YOU SHOULD HAVE LOCKED pmlmepriv->lock
 */
__inline static void set_fwstate(struct mlme_priv *pmlmepriv, int state)
{
	pmlmepriv->fw_state |= state;
	//FOR HW integration
	if(_FW_UNDER_SURVEY==state){
		pmlmepriv->bScanInProcess = true;
	}
}

__inline static void _clr_fwstate_(struct mlme_priv *pmlmepriv, int state)
{
	pmlmepriv->fw_state &= ~state;
	//FOR HW integration
	if(_FW_UNDER_SURVEY==state){
		pmlmepriv->bScanInProcess = false;
	}
}

/*
 * No Limit on the calling context,
 * therefore set it to be the critical section...
 */
__inline static void clr_fwstate(struct mlme_priv *pmlmepriv, int state)
{
	spin_lock_bh(&pmlmepriv->lock);
	if (check_fwstate(pmlmepriv, state) == true)
		pmlmepriv->fw_state ^= state;
	spin_unlock_bh(&pmlmepriv->lock);
}

__inline static void clr_fwstate_ex(struct mlme_priv *pmlmepriv, int state)
{
	spin_lock_bh(&pmlmepriv->lock);
	_clr_fwstate_(pmlmepriv, state);
	spin_unlock_bh(&pmlmepriv->lock);
}

__inline static void up_scanned_network(struct mlme_priv *pmlmepriv)
{
	spin_lock_bh(&pmlmepriv->lock);
	pmlmepriv->num_of_scanned++;
	spin_unlock_bh(&pmlmepriv->lock);
}

__inline static void down_scanned_network(struct mlme_priv *pmlmepriv)
{
	spin_lock_bh(&pmlmepriv->lock);
	pmlmepriv->num_of_scanned--;
	spin_unlock_bh(&pmlmepriv->lock);
}

__inline static void set_scanned_network_val(struct mlme_priv *pmlmepriv, int val)
{
	spin_lock_bh(&pmlmepriv->lock);
	pmlmepriv->num_of_scanned = val;
	spin_unlock_bh(&pmlmepriv->lock);
}

extern u16 rtw_get_capability(WLAN_BSSID_EX *bss);
extern void rtw_update_scanned_network(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *target);
extern void rtw_disconnect_hdl_under_linked(struct rtl_priv* rtlpriv, struct sta_info *psta, uint8_t free_assoc);
extern void rtw_generate_random_ibss(uint8_t *pibss);
extern struct wlan_network* rtw_find_network(struct __queue *scanned_queue, uint8_t *addr);
extern struct wlan_network* rtw_get_oldest_wlan_network(struct __queue *scanned_queue);

extern void rtw_free_assoc_resources(struct rtl_priv* rtlpriv, int lock_scanned_queue);
extern void rtw_indicate_disconnect(struct rtl_priv* rtlpriv);
extern void rtw_indicate_connect(struct rtl_priv* rtlpriv);
void rtw_indicate_scan_done( struct rtl_priv *rtlpriv, bool aborted);
void rtw_scan_abort(struct rtl_priv *rtlpriv);

extern int rtw_restruct_sec_ie(struct rtl_priv *rtlpriv,uint8_t *in_ie,uint8_t *out_ie,uint in_len);
extern int rtw_restruct_wmm_ie(struct rtl_priv *rtlpriv, uint8_t *in_ie, uint8_t *out_ie, uint in_len, uint initial_out_len);
extern void rtw_init_registrypriv_dev_network(struct rtl_priv *rtlpriv);

extern void rtw_update_registrypriv_dev_network(struct rtl_priv *rtlpriv);

extern void rtw_get_encrypt_decrypt_from_registrypriv(struct rtl_priv *rtlpriv);

extern void _rtw_join_timeout_handler(struct rtl_priv *rtlpriv);
extern void rtw_scan_timeout_handler(struct rtl_priv *rtlpriv);

extern void rtw_dynamic_check_timer_handlder(struct rtl_priv *rtlpriv);

extern int _rtw_init_mlme_priv(struct rtl_priv *rtlpriv);

void rtw_free_mlme_priv_ie_data(struct mlme_priv *pmlmepriv);

extern void _rtw_free_mlme_priv(struct mlme_priv *pmlmepriv);

extern int _rtw_enqueue_network(struct __queue *queue, struct wlan_network *pnetwork);

//extern struct wlan_network* _rtw_dequeue_network(struct __queue *queue);

extern struct wlan_network* _rtw_alloc_network(struct mlme_priv *pmlmepriv);


extern void _rtw_free_network(struct mlme_priv *pmlmepriv, struct wlan_network *pnetwork, uint8_t isfreeall);
extern void _rtw_free_network_nolock(struct mlme_priv *pmlmepriv, struct wlan_network *pnetwork);


extern struct wlan_network* _rtw_find_network(struct __queue *scanned_queue, uint8_t *addr);

extern void _rtw_free_network_queue(struct rtl_priv* rtlpriv, uint8_t isfreeall);

extern int rtw_if_up(struct rtl_priv *rtlpriv);

int rtw_linked_check(struct rtl_priv *rtlpriv);

uint8_t *rtw_get_capability_from_ie(uint8_t *ie);
uint8_t *rtw_get_timestampe_from_ie(uint8_t *ie);
uint8_t *rtw_get_beacon_interval_from_ie(uint8_t *ie);


void rtw_joinbss_reset(struct rtl_priv *rtlpriv);

unsigned int rtw_restructure_ht_ie(struct rtl_priv *rtlpriv, uint8_t *in_ie, uint8_t *out_ie, uint in_len, uint *pout_len);
void rtw_update_ht_cap(struct rtl_priv *rtlpriv, uint8_t *pie, uint ie_len, uint8_t channel);
void rtw_issue_addbareq_cmd(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe);

int rtw_is_same_ibss(struct rtl_priv *rtlpriv, struct wlan_network *pnetwork);
int is_same_network(WLAN_BSSID_EX *src, WLAN_BSSID_EX *dst);

void rtw_stassoc_hw_rpt(struct rtl_priv *rtlpriv,struct sta_info *psta);

#endif //__RTL871X_MLME_H_

