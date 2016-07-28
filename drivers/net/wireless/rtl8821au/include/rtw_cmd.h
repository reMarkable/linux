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
#ifndef __RTW_CMD_H_
#define __RTW_CMD_H_

#include <rtw_rf.h>

#define C2H_MEM_SZ (16*1024)

	#define FREE_CMDOBJ_SZ	128

	#define MAX_CMDSZ	1024
	#define MAX_RSPSZ	512
	#define MAX_EVTSZ	1024

	#define CMDBUFF_ALIGN_SZ 512

	struct cmd_obj {
		struct rtl_priv *rtlpriv;
		u16	cmdcode;
		uint8_t	res;
		uint8_t	*parmbuf;
		u32	cmdsz;
		uint8_t	*rsp;
		u32	rspsz;
		//struct semaphore 	cmd_sem;
		struct list_head	list;
	};

	struct cmd_priv {
		struct semaphore	cmd_queue_sema;
		//struct semaphore	cmd_done_sema;
		struct semaphore	terminate_cmdthread_sema;
		struct __queue	cmd_queue;
		uint8_t	cmd_seq;
		uint8_t	*cmd_buf;	//shall be non-paged, and 4 bytes aligned
		uint8_t	*cmd_allocated_buf;
		uint8_t	*rsp_buf;	//shall be non-paged, and 4 bytes aligned
		uint8_t	*rsp_allocated_buf;
		u32	cmd_issued_cnt;
		u32	cmd_done_cnt;
		u32	rsp_cnt;
		uint8_t cmdthd_running;
		struct rtl_priv *rtlpriv;
	};

	struct	evt_priv {
		atomic_t event_seq;
		uint8_t	*evt_buf;	//shall be non-paged, and 4 bytes aligned
		uint8_t	*evt_allocated_buf;
		u32	evt_done_cnt;

	};

#define init_h2fwcmd_w_parm_no_rsp(pcmd, pparm, code) \
do {\
	INIT_LIST_HEAD(&pcmd->list);\
	pcmd->cmdcode = code;\
	pcmd->parmbuf = (uint8_t *)(pparm);\
	pcmd->cmdsz = sizeof (*pparm);\
	pcmd->rsp = NULL;\
	pcmd->rspsz = 0;\
} while(0)

struct c2h_evt_hdr {
	uint8_t id:4;
	uint8_t plen:4;
	uint8_t seq;
	uint8_t payload[0];
};

struct P2P_PS_Offload_t {
	uint8_t Offload_En:1;
	uint8_t role:1; // 1: Owner, 0: Client
	uint8_t CTWindow_En:1;
	uint8_t NoA0_En:1;
	uint8_t NoA1_En:1;
	uint8_t AllStaSleep:1; // Only valid in Owner
	uint8_t discovery:1;
	uint8_t rsvd:1;
};

struct P2P_PS_CTWPeriod_t {
	uint8_t CTWPeriod;	//TU
};

#define c2h_evt_exist(c2h_evt) ((c2h_evt)->id || (c2h_evt)->plen)

extern u32 rtw_enqueue_cmd(struct cmd_priv *pcmdpriv, struct cmd_obj *obj);
extern struct cmd_obj *rtw_dequeue_cmd(struct cmd_priv *pcmdpriv);
extern void rtw_free_cmd_obj(struct cmd_obj *pcmd);

int rtw_cmd_thread(void *context);

extern u32 rtw_init_cmd_priv (struct cmd_priv *pcmdpriv);
extern void rtw_free_cmd_priv (struct cmd_priv *pcmdpriv);

extern u32 rtw_init_evt_priv (struct evt_priv *pevtpriv);
extern void rtw_free_evt_priv (struct evt_priv *pevtpriv);
extern void rtw_cmd_clr_isr(struct cmd_priv *pcmdpriv);
extern void rtw_evt_notify_isr(struct evt_priv *pevtpriv);

enum rtw_drvextra_cmd_id
{
	NONE_WK_CID,
	DYNAMIC_CHK_WK_CID,
	DM_CTRL_WK_CID,
	PBC_POLLING_WK_CID,
	POWER_SAVING_CTRL_WK_CID,//IPS,AUTOSuspend
	LPS_CTRL_WK_CID,
	ANT_SELECT_WK_CID,
	P2P_PS_WK_CID,
	P2P_PROTO_WK_CID,
	CHECK_HIQ_WK_CID,//for softap mode, check hi queue if empty
	INTEl_WIDI_WK_CID,
	C2H_WK_CID,
	RTP_TIMER_CFG_WK_CID,
	MAX_WK_CID
};

enum LPS_CTRL_TYPE
{
	LPS_CTRL_SCAN=0,
	LPS_CTRL_JOINBSS=1,
	LPS_CTRL_CONNECT=2,
	LPS_CTRL_DISCONNECT=3,
	LPS_CTRL_SPECIAL_PACKET=4,
	LPS_CTRL_LEAVE=5,
};

enum RFINTFS {
	SWSI,
	HWSI,
	HWPI,
};

/*
Caller Mode: Infra, Ad-HoC(C)

Notes: To enter USB suspend mode

Command Mode

*/
struct usb_suspend_parm {
	u32 action;// 1: sleep, 0:resume
};

/*
Caller Mode: Infra, Ad-HoC

Notes: To join a known BSS.

Command-Event Mode

*/

/*
Caller Mode: Infra, Ad-Hoc

Notes: To join the specified bss

Command Event Mode

*/
struct joinbss_parm {
	WLAN_BSSID_EX network;
};

/*
Caller Mode: Infra, Ad-HoC(C)

Notes: To disconnect the current associated BSS

Command Mode

*/
struct disconnect_parm {
	u32 deauth_timeout_ms;
};

/*
Caller Mode: AP, Ad-HoC(M)

Notes: To create a BSS

Command Mode
*/
struct createbss_parm {
	WLAN_BSSID_EX network;
};

/*
Caller Mode: AP, Ad-HoC, Infra

Notes: To set the NIC mode of RTL8711

Command Mode

The definition of mode:

#define IW_MODE_AUTO	0	// Let the driver decides which AP to join
#define IW_MODE_ADHOC	1	// Single cell network (Ad-Hoc Clients)
#define IW_MODE_INFRA	2	// Multi cell network, roaming, ..
#define IW_MODE_MASTER	3	// Synchronisation master or Access Point
#define IW_MODE_REPEAT	4	// Wireless Repeater (forwarder)
#define IW_MODE_SECOND	5	// Secondary master/repeater (backup)
#define IW_MODE_MONITOR	6	// Passive monitor (listen only)

*/
struct	setopmode_parm {
	uint8_t	mode;
	uint8_t	rsvd[3];
};

/*
Caller Mode: AP, Ad-HoC, Infra

Notes: To ask RTL8711 performing site-survey

Command-Event Mode

*/

#define RTW_SSID_SCAN_AMOUNT 9 // for WEXT_CSCAN_AMOUNT 9
#define RTW_CHANNEL_SCAN_AMOUNT (14+37)
struct sitesurvey_parm {
	int scan_mode;	//active: 1, passive: 0
	/* int bsslimit;	// 1 ~ 48 */
	uint8_t ssid_num;
	uint8_t ch_num;
	NDIS_802_11_SSID ssid[RTW_SSID_SCAN_AMOUNT];
	struct rtw_ieee80211_channel ch[RTW_CHANNEL_SCAN_AMOUNT];
};

/*
Caller Mode: Any

Notes: To set the auth type of RTL8711. open/shared/802.1x

Command Mode

*/
struct setauth_parm {
	uint8_t mode;  //0: legacy open, 1: legacy shared 2: 802.1x
	uint8_t _1x;   //0: PSK, 1: TLS
	uint8_t rsvd[2];
};

/*
Caller Mode: Infra

a. algorithm: wep40, wep104, tkip & aes
b. keytype: grp key/unicast key
c. key contents

when shared key ==> keyid is the camid
when 802.1x ==> keyid [0:1] ==> grp key
when 802.1x ==> keyid > 2 ==> unicast key

*/
struct setkey_parm {
	uint8_t	algorithm;	// encryption algorithm, could be none, wep40, TKIP, CCMP, wep104
	uint8_t	keyid;
	uint8_t 	grpkey;		// 1: this is the grpkey for 802.1x. 0: this is the unicast key for 802.1x
	uint8_t 	set_tx;		// 1: main tx key for wep. 0: other key.
	uint8_t	key[16];	// this could be 40 or 104
};

/*
When in AP or Ad-Hoc mode, this is used to
allocate an sw/hw entry for a newly associated sta.

Command

when shared key ==> algorithm/keyid

*/
struct set_stakey_parm {
	uint8_t	addr[ETH_ALEN];
	uint8_t	algorithm;
	uint8_t 	id;// currently for erasing cam entry if algorithm == _NO_PRIVACY_
	uint8_t	key[16];
};

struct set_stakey_rsp {
	uint8_t	addr[ETH_ALEN];
	uint8_t	keyid;
	uint8_t	rsvd;
};

/*
Caller Ad-Hoc/AP

Command -Rsp(AID == CAMID) mode

This is to force fw to add an sta_data entry per driver's request.

FW will write an cam entry associated with it.

*/
struct set_assocsta_parm {
	uint8_t	addr[ETH_ALEN];
};

struct set_assocsta_rsp {
	uint8_t	cam_id;
	uint8_t	rsvd[3];
};

/*
	Caller Ad-Hoc/AP

	Command mode

	This is to force fw to del an sta_data entry per driver's request

	FW will invalidate the cam entry associated with it.

*/
struct del_assocsta_parm {
	uint8_t  	addr[ETH_ALEN];
};

/*
Caller Mode: AP/Ad-HoC(M)

Notes: To notify fw that given staid has changed its power state

Command Mode

*/
struct setstapwrstate_parm {
	uint8_t	staid;
	uint8_t	status;
	uint8_t	hwaddr[6];
};

/*
Caller Mode: Any

Notes: To setup the basic rate of RTL8711

Command Mode

*/
struct	setbasicrate_parm {
	uint8_t	basicrates[NumRates];
};

/*
Caller Mode: Any

Notes: To read the current basic rate

Command-Rsp Mode

*/
struct getbasicrate_parm {
	u32 rsvd;
};

struct getbasicrate_rsp {
	uint8_t basicrates[NumRates];
};

/*
Caller Mode: Any

Notes: To setup the data rate of RTL8711

Command Mode

*/
struct setdatarate_parm {
#ifdef MP_FIRMWARE_OFFLOAD
	u32	curr_rateidx;
#else
	uint8_t	mac_id;
	uint8_t	datarates[NumRates];
#endif
};

/*
Caller Mode: Any

Notes: To read the current data rate

Command-Rsp Mode

*/
struct getdatarate_parm {
	u32 rsvd;

};
struct getdatarate_rsp {
	uint8_t datarates[NumRates];
};


/*
Caller Mode: Any
AP: AP can use the info for the contents of beacon frame
Infra: STA can use the info when sitesurveying
Ad-HoC(M): Like AP
Ad-HoC(C): Like STA


Notes: To set the phy capability of the NIC

Command Mode

*/

struct	setphyinfo_parm {
	struct regulatory_class class_sets[NUM_REGULATORYS];
	uint8_t	status;
};

struct	getphyinfo_parm {
	u32 rsvd;
};

struct	getphyinfo_rsp {
	struct regulatory_class class_sets[NUM_REGULATORYS];
	uint8_t	status;
};

/*
Caller Mode: Any

Notes: To set the channel/modem/band
This command will be used when channel/modem/band is changed.

Command Mode

*/
struct	setphy_parm {
	uint8_t	rfchannel;
	uint8_t	modem;
};

/*
Caller Mode: Any

Notes: To get the current setting of channel/modem/band

Command-Rsp Mode

*/
struct	getphy_parm {
	u32 rsvd;

};
struct	getphy_rsp {
	uint8_t	rfchannel;
	uint8_t	modem;
};

struct readBB_parm {
	uint8_t	offset;
};
struct readBB_rsp {
	uint8_t	value;
};

struct readTSSI_parm {
	uint8_t	offset;
};
struct readTSSI_rsp {
	uint8_t	value;
};

struct writeBB_parm {
	uint8_t	offset;
	uint8_t	value;
};

struct readRF_parm {
	uint8_t	offset;
};
struct readRF_rsp {
	u32	value;
};

struct writeRF_parm {
	u32	offset;
	u32	value;
};

struct getrfintfs_parm {
	uint8_t	rfintfs;
};


struct Tx_Beacon_param
{
	WLAN_BSSID_EX network;
};

/*
	Notes: This command is used for H2C/C2H loopback testing

	mac[0] == 0
	==> CMD mode, return H2C_SUCCESS.
	The following condition must be ture under CMD mode
		mac[1] == mac[4], mac[2] == mac[3], mac[0]=mac[5]= 0;
		s0 == 0x1234, s1 == 0xabcd, w0 == 0x78563412, w1 == 0x5aa5def7;
		s2 == (b1 << 8 | b0);

	mac[0] == 1
	==> CMD_RSP mode, return H2C_SUCCESS_RSP

	The rsp layout shall be:
	rsp: 			parm:
		mac[0]  =   mac[5];
		mac[1]  =   mac[4];
		mac[2]  =   mac[3];
		mac[3]  =   mac[2];
		mac[4]  =   mac[1];
		mac[5]  =   mac[0];
		s0		=   s1;
		s1		=   swap16(s0);
		w0		=  	swap32(w1);
		b0		= 	b1
		s2		= 	s0 + s1
		b1		= 	b0
		w1		=	w0

	mac[0] == 	2
	==> CMD_EVENT mode, return 	H2C_SUCCESS
	The event layout shall be:
	event:			parm:
		mac[0]  =   mac[5];
		mac[1]  =   mac[4];
		mac[2]  =   event's sequence number, starting from 1 to parm's marc[3]
		mac[3]  =   mac[2];
		mac[4]  =   mac[1];
		mac[5]  =   mac[0];
		s0		=   swap16(s0) - event.mac[2];
		s1		=   s1 + event.mac[2];
		w0		=  	swap32(w0);
		b0		= 	b1
		s2		= 	s0 + event.mac[2]
		b1		= 	b0
		w1		=	swap32(w1) - event.mac[2];

		parm->mac[3] is the total event counts that host requested.


	event will be the same with the cmd's param.

*/

// CMD param Formart for driver extra cmd handler
struct drvextra_cmd_parm {
	int ec_id; //extra cmd id
	int type_size; // Can use this field as the type id or command size
	unsigned char *pbuf;
};

/*------------------- Below are used for RF/BB tunning ---------------------*/

struct	setantenna_parm {
	uint8_t	tx_antset;
	uint8_t	rx_antset;
	uint8_t	tx_antenna;
	uint8_t	rx_antenna;
};

struct	enrateadaptive_parm {
	u32	en;
};

struct settxagctbl_parm {
	u32	txagc[MAX_RATES_LENGTH];
};

struct gettxagctbl_parm {
	u32 rsvd;
};
struct gettxagctbl_rsp {
	u32	txagc[MAX_RATES_LENGTH];
};

struct setagcctrl_parm {
	u32	agcctrl;		// 0: pure hw, 1: fw
};


struct setssup_parm	{
	u32	ss_ForceUp[MAX_RATES_LENGTH];
};

struct getssup_parm	{
	u32 rsvd;
};
struct getssup_rsp	{
	uint8_t	ss_ForceUp[MAX_RATES_LENGTH];
};


struct setssdlevel_parm	{
	uint8_t	ss_DLevel[MAX_RATES_LENGTH];
};

struct getssdlevel_parm	{
	u32 rsvd;
};
struct getssdlevel_rsp	{
	uint8_t	ss_DLevel[MAX_RATES_LENGTH];
};

struct setssulevel_parm	{
	uint8_t	ss_ULevel[MAX_RATES_LENGTH];
};

struct getssulevel_parm	{
	u32 rsvd;
};
struct getssulevel_rsp	{
	uint8_t	ss_ULevel[MAX_RATES_LENGTH];
};


struct	setcountjudge_parm {
	uint8_t	count_judge[MAX_RATES_LENGTH];
};

struct	getcountjudge_parm {
	u32 rsvd;
};
struct	getcountjudge_rsp {
	uint8_t	count_judge[MAX_RATES_LENGTH];
};


struct setratable_parm {
	uint8_t ss_ForceUp[NumRates];
	uint8_t ss_ULevel[NumRates];
	uint8_t ss_DLevel[NumRates];
	uint8_t count_judge[NumRates];
};

struct getratable_parm {
                uint rsvd;
};
struct getratable_rsp {
        uint8_t ss_ForceUp[NumRates];
        uint8_t ss_ULevel[NumRates];
        uint8_t ss_DLevel[NumRates];
        uint8_t count_judge[NumRates];
};


//to get TX,RX retry count
struct gettxretrycnt_parm{
	unsigned int rsvd;
};
struct gettxretrycnt_rsp{
	unsigned long tx_retrycnt;
};

struct getrxretrycnt_parm{
	unsigned int rsvd;
};
struct getrxretrycnt_rsp{
	unsigned long rx_retrycnt;
};

//to get BCNOK,BCNERR count
struct getbcnokcnt_parm{
	unsigned int rsvd;
};
struct getbcnokcnt_rsp{
	unsigned long  bcnokcnt;
};

struct getbcnerrcnt_parm{
	unsigned int rsvd;
};
struct getbcnerrcnt_rsp{
	unsigned long bcnerrcnt;
};

// to get current TX power level
struct getcurtxpwrlevel_parm{
	unsigned int rsvd;
};
struct getcurtxpwrlevel_rsp{
	unsigned short tx_power;
};

struct setprobereqextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};

struct setassocreqextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};

struct setproberspextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};

struct setassocrspextraie_parm {
	unsigned char e_id;
	unsigned char ie_len;
	unsigned char ie[0];
};


struct addBaReq_parm
{
 	unsigned int tid;
	uint8_t	addr[ETH_ALEN];
};

/*H2C Handler index: 46 */
struct set_ch_parm {
	uint8_t ch;
	uint8_t bw;
	uint8_t ch_offset;
};

#ifdef MP_FIRMWARE_OFFLOAD
/*H2C Handler index: 47 */
struct SetTxPower_parm
{
	uint8_t TxPower;
};

/*H2C Handler index: 48 */
struct SwitchAntenna_parm
{
	u16 antenna_tx;
	u16 antenna_rx;
//	R_ANTENNA_SELECT_CCK cck_txrx;
	uint8_t cck_txrx;
};

/*H2C Handler index: 49 */
struct SetCrystalCap_parm
{
	u32 curr_crystalcap;
};

/*H2C Handler index: 50 */
struct SetSingleCarrierTx_parm
{
	uint8_t bStart;
};

/*H2C Handler index: 51 */
struct SetSingleToneTx_parm
{
	uint8_t bStart;
	uint8_t curr_rfpath;
};

/*H2C Handler index: 52 */
struct SetCarrierSuppressionTx_parm
{
	uint8_t bStart;
	u32 curr_rateidx;
};

/*H2C Handler index: 53 */
struct SetContinuousTx_parm
{
	uint8_t bStart;
	uint8_t CCK_flag; /*1:CCK 2:OFDM*/
	u32 curr_rateidx;
};

/*H2C Handler index: 54 */
struct SwitchBandwidth_parm
{
	uint8_t curr_bandwidth;
};

#endif	/* MP_FIRMWARE_OFFLOAD */

/*H2C Handler index: 59 */
struct SetChannelPlan_param
{
	uint8_t channel_plan;
};

/*H2C Handler index: 60 */
struct LedBlink_param
{
	void *pLed;
};

/*H2C Handler index: 61 */
struct SetChannelSwitch_param
{
	uint8_t new_ch_no;
};

/*H2C Handler index: 62 */
struct TDLSoption_param
{
	uint8_t addr[ETH_ALEN];
	uint8_t option;
};

#define GEN_CMD_CODE(cmd)	cmd ## _CMD_


/*

Result:
0x00: success
0x01: sucess, and check Response.
0x02: cmd ignored due to duplicated sequcne number
0x03: cmd dropped due to invalid cmd code
0x04: reserved.

*/

#define H2C_RSP_OFFSET			512

#define H2C_SUCCESS			0x00
#define H2C_SUCCESS_RSP			0x01
#define H2C_DUPLICATED			0x02
#define H2C_DROPPED			0x03
#define H2C_PARAMETERS_ERROR		0x04
#define H2C_REJECTED			0x05
#define H2C_CMD_OVERFLOW		0x06
#define H2C_RESERVED			0x07

uint8_t rtw_sitesurvey_cmd(struct rtl_priv  *rtlpriv, NDIS_802_11_SSID *ssid, int ssid_num, struct rtw_ieee80211_channel *ch, int ch_num);
extern uint8_t rtw_createbss_cmd(struct rtl_priv  *rtlpriv);
extern uint8_t rtw_createbss_cmd_ex(struct rtl_priv  *rtlpriv, unsigned char *pbss, unsigned int sz);
extern uint8_t rtw_setphy_cmd(struct rtl_priv  *rtlpriv, uint8_t modem, uint8_t ch);
extern uint8_t rtw_setstakey_cmd(struct rtl_priv  *rtlpriv, uint8_t *psta, uint8_t unicast_key);
extern uint8_t rtw_clearstakey_cmd(struct rtl_priv *rtlpriv, uint8_t *psta, uint8_t entry, uint8_t enqueue);
extern uint8_t rtw_joinbss_cmd(struct rtl_priv  *rtlpriv, struct wlan_network* pnetwork);
uint8_t rtw_disassoc_cmd(struct rtl_priv *rtlpriv, u32 deauth_timeout_ms, bool enqueue);
extern uint8_t rtw_setopmode_cmd(struct rtl_priv  *rtlpriv, NDIS_802_11_NETWORK_INFRASTRUCTURE networktype);
extern uint8_t rtw_setrfintfs_cmd(struct rtl_priv  *rtlpriv, uint8_t mode);

extern uint8_t rtw_gettssi_cmd(struct rtl_priv  *rtlpriv, uint8_t offset,uint8_t *pval);
extern uint8_t rtw_setfwdig_cmd(struct rtl_priv*rtlpriv, uint8_t type);
extern uint8_t rtw_setfwra_cmd(struct rtl_priv*rtlpriv, uint8_t type);

extern uint8_t rtw_addbareq_cmd(struct rtl_priv*rtlpriv, uint8_t tid, uint8_t *addr);

extern uint8_t rtw_dynamic_chk_wk_cmd(struct rtl_priv *rtlpriv);

uint8_t rtw_lps_ctrl_wk_cmd(struct rtl_priv*rtlpriv, uint8_t lps_ctrl_type, uint8_t enqueue);
#if (RATE_ADAPTIVE_SUPPORT==1)
uint8_t rtw_rpt_timer_cfg_cmd(struct rtl_priv*rtlpriv, u16 minRptTime);
#endif

extern uint8_t rtw_ps_cmd(struct rtl_priv*rtlpriv);

#ifdef CONFIG_AP_MODE
uint8_t rtw_chk_hi_queue_cmd(struct rtl_priv*rtlpriv);
#endif

uint8_t rtw_set_ch_cmd(struct rtl_priv*rtlpriv, uint8_t ch, uint8_t bw, uint8_t ch_offset, uint8_t enqueue);
extern uint8_t rtw_set_csa_cmd(struct rtl_priv*rtlpriv, uint8_t new_ch_no);

uint8_t rtw_drvextra_cmd_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf);

extern void rtw_survey_cmd_callback(struct rtl_priv  *rtlpriv, struct cmd_obj *pcmd);
extern void rtw_disassoc_cmd_callback(struct rtl_priv  *rtlpriv, struct cmd_obj *pcmd);
extern void rtw_joinbss_cmd_callback(struct rtl_priv  *rtlpriv, struct cmd_obj *pcmd);
extern void rtw_createbss_cmd_callback(struct rtl_priv  *rtlpriv, struct cmd_obj *pcmd);
extern void rtw_getbbrfreg_cmdrsp_callback(struct rtl_priv  *rtlpriv, struct cmd_obj *pcmd);
extern void rtw_readtssi_cmdrsp_callback(struct rtl_priv*	rtlpriv,  struct cmd_obj *pcmd);

extern void rtw_setstaKey_cmdrsp_callback(struct rtl_priv  *rtlpriv,  struct cmd_obj *pcmd);
extern void rtw_setassocsta_cmdrsp_callback(struct rtl_priv  *rtlpriv,  struct cmd_obj *pcmd);
extern void rtw_getrttbl_cmdrsp_callback(struct rtl_priv  *rtlpriv,  struct cmd_obj *pcmd);


struct _cmd_callback {
	u32	cmd_code;
	void (*callback)(struct rtl_priv  *rtlpriv, struct cmd_obj *cmd);
};

enum rtw_h2c_cmd {
 	GEN_CMD_CODE(_JoinBss),		/*  1 */
 	GEN_CMD_CODE(_DisConnect) ,	/*  2 */
 	GEN_CMD_CODE(_CreateBss) ,	/*  3 */
	GEN_CMD_CODE(_SetOpMode) ,	/*  4 */
	GEN_CMD_CODE(_SiteSurvey),	/*  5 */
 	GEN_CMD_CODE(_SetAuth) ,	/*  6 */
 	GEN_CMD_CODE(_SetKey) ,		/*  7 */
 	GEN_CMD_CODE(_SetStaKey) ,	/*  8 */
 	GEN_CMD_CODE(_AddBAReq) ,	/*  9 */
	GEN_CMD_CODE(_SetChannel),	/* 10 */
	GEN_CMD_CODE(_TX_Beacon),	/* 11 */
	GEN_CMD_CODE(_Set_MLME_EVT),	/* 12 */
	GEN_CMD_CODE(_Set_Drv_Extra),	/* 13 */
	GEN_CMD_CODE(_SetChannelSwitch),/* 15 */

	MAX_H2CCMD
};

#ifdef _RTW_CMD_C_
struct _cmd_callback 	rtw_cmd_callback[] = {
	{GEN_CMD_CODE(_JoinBss), &rtw_joinbss_cmd_callback},		/*  1 */
	{GEN_CMD_CODE(_DisConnect), &rtw_disassoc_cmd_callback},	/*  2 */
	{GEN_CMD_CODE(_CreateBss), &rtw_createbss_cmd_callback},	/*  3 */
	{GEN_CMD_CODE(_SetOpMode), NULL},				/*  4 */
	{GEN_CMD_CODE(_SiteSurvey), &rtw_survey_cmd_callback},		/*  5 */
	{GEN_CMD_CODE(_SetAuth), NULL},					/*  6 */
	{GEN_CMD_CODE(_SetKey), NULL},					/*  7 */
	{GEN_CMD_CODE(_SetStaKey), &rtw_setstaKey_cmdrsp_callback},	/*  8 */
 	{GEN_CMD_CODE(_AddBAReq), NULL},				/*  9 */
	{GEN_CMD_CODE(_SetChannel), NULL},				/* 10 */
	{GEN_CMD_CODE(_TX_Beacon), NULL},				/* 11 */
	{GEN_CMD_CODE(_Set_MLME_EVT), NULL},				/* 12 */
	{GEN_CMD_CODE(_Set_Drv_Extra), NULL},				/* 13 */
	{GEN_CMD_CODE(_SetChannelSwitch), NULL},			/* 15 */
};
#endif

#endif // _CMD_H_

