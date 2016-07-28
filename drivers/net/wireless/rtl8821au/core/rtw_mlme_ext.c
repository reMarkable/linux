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
#define _RTW_MLME_EXT_C_

#include <drv_types.h>
#include <rtw_ap.h>
#include <../rtl8821au/hw.h>

#include <../usb.h>
#include <../wifi.h>
#include <../cam.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

static struct list_head *get_next(struct list_head	*list)
{
	return list->next;
}

#define _drv_always_		1
#define	_drv_warning_		6
#undef DBG_871X_LEVEL
static inline void DBG_871X_LEVEL(const int level, const char *fmt, ...)
{
}

struct mlme_handler mlme_sta_tbl[]={
	{WIFI_ASSOCREQ,		"OnAssocReq",	&OnAssocReq},
	{WIFI_ASSOCRSP,		"OnAssocRsp",	&OnAssocRsp},
	{WIFI_REASSOCREQ,	"OnReAssocReq",	&OnAssocReq},
	{WIFI_REASSOCRSP,	"OnReAssocRsp",	&OnAssocRsp},
	{WIFI_PROBEREQ,		"OnProbeReq",	&OnProbeReq},
	{WIFI_PROBERSP,		"OnProbeRsp",		&OnProbeRsp},

	/*----------------------------------------------------------
					below 2 are reserved
	-----------------------------------------------------------*/
	{0,					"DoReserved",		&DoReserved},
	{0,					"DoReserved",		&DoReserved},
	{WIFI_BEACON,		"OnBeacon",		&OnBeacon},
	{WIFI_ATIM,			"OnATIM",		&OnAtim},
	{WIFI_DISASSOC,		"OnDisassoc",		&OnDisassoc},
	{WIFI_AUTH,			"OnAuth",		&OnAuthClient},
	{WIFI_DEAUTH,		"OnDeAuth",		&OnDeAuth},
	{WIFI_ACTION,		"OnAction",		&OnAction},
};

#ifdef _CONFIG_NATIVEAP_MLME_
struct mlme_handler mlme_ap_tbl[]={
	{WIFI_ASSOCREQ,		"OnAssocReq",	&OnAssocReq},
	{WIFI_ASSOCRSP,		"OnAssocRsp",	&OnAssocRsp},
	{WIFI_REASSOCREQ,	"OnReAssocReq",	&OnAssocReq},
	{WIFI_REASSOCRSP,	"OnReAssocRsp",	&OnAssocRsp},
	{WIFI_PROBEREQ,		"OnProbeReq",	&OnProbeReq},
	{WIFI_PROBERSP,		"OnProbeRsp",		&OnProbeRsp},

	/*----------------------------------------------------------
					below 2 are reserved
	-----------------------------------------------------------*/
	{0,					"DoReserved",		&DoReserved},
	{0,					"DoReserved",		&DoReserved},
	{WIFI_BEACON,		"OnBeacon",		&OnBeacon},
	{WIFI_ATIM,			"OnATIM",		&OnAtim},
	{WIFI_DISASSOC,		"OnDisassoc",		&OnDisassoc},
	{WIFI_AUTH,			"OnAuth",		&OnAuth},
	{WIFI_DEAUTH,		"OnDeAuth",		&OnDeAuth},
	{WIFI_ACTION,		"OnAction",		&OnAction},
};
#endif

struct action_handler OnAction_tbl[]={
	{RTW_WLAN_CATEGORY_SPECTRUM_MGMT,	 "ACTION_SPECTRUM_MGMT", on_action_spct},
	{RTW_WLAN_CATEGORY_QOS, "ACTION_QOS", &OnAction_qos},
	{RTW_WLAN_CATEGORY_DLS, "ACTION_DLS", &OnAction_dls},
	{RTW_WLAN_CATEGORY_BACK, "ACTION_BACK", &OnAction_back},
	{RTW_WLAN_CATEGORY_PUBLIC, "ACTION_PUBLIC", on_action_public},
	{RTW_WLAN_CATEGORY_RADIO_MEASUREMENT, "ACTION_RADIO_MEASUREMENT", &DoReserved},
	{RTW_WLAN_CATEGORY_FT, "ACTION_FT",	&DoReserved},
	{RTW_WLAN_CATEGORY_HT,	"ACTION_HT",	&OnAction_ht},
	{RTW_WLAN_CATEGORY_SA_QUERY, "ACTION_SA_QUERY", &DoReserved},
	{RTW_WLAN_CATEGORY_WMM, "ACTION_WMM", &OnAction_wmm},
	{RTW_WLAN_CATEGORY_P2P, "ACTION_P2P", &OnAction_p2p},
};


uint8_t	null_addr[ETH_ALEN]= {0,0,0,0,0,0};

/**************************************************
OUI definitions for the vendor specific IE
***************************************************/
unsigned char	RTW_WPA_OUI[] = {0x00, 0x50, 0xf2, 0x01};
unsigned char WMM_OUI[] = {0x00, 0x50, 0xf2, 0x02};
unsigned char	WPS_OUI[] = {0x00, 0x50, 0xf2, 0x04};
unsigned char	P2P_OUI[] = {0x50,0x6F,0x9A,0x09};
unsigned char	WFD_OUI[] = {0x50,0x6F,0x9A,0x0A};

unsigned char	WMM_INFO_OUI[] = {0x00, 0x50, 0xf2, 0x02, 0x00, 0x01};
unsigned char	WMM_PARA_OUI[] = {0x00, 0x50, 0xf2, 0x02, 0x01, 0x01};

unsigned char WPA_TKIP_CIPHER[4] = {0x00, 0x50, 0xf2, 0x02};
unsigned char RSN_TKIP_CIPHER[4] = {0x00, 0x0f, 0xac, 0x02};

extern unsigned char REALTEK_96B_IE[];

/********************************************************
MCS rate definitions
*********************************************************/
#ifdef CONFIG_DISABLE_MCS13TO15
unsigned char	MCS_rate_2R_MCS13TO15_OFF[16] = {0xff, 0x1f, 0x0, 0x0, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char	MCS_rate_2R[16] = {0xff, 0xff, 0x0, 0x0, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
#else //CONFIG_DISABLE_MCS13TO15
unsigned char	MCS_rate_2R[16] = {0xff, 0xff, 0x0, 0x0, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
#endif //CONFIG_DISABLE_MCS13TO15
unsigned char	MCS_rate_1R[16] = {0xff, 0x00, 0x0, 0x0, 0x01, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

/********************************************************
ChannelPlan definitions
*********************************************************/

static unsigned char RTW_ChannelPlan2G[MAX_CHANNEL_NUM_2G] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14		/* 0x03, RT_CHANNEL_DOMAIN_2G_MIKK1 */
};

static unsigned char RTW_ChannelPlan5G[MAX_CHANNEL_NUM_5G] = {
	 36,  40,  44,  48,  52,  56,  60,  64, 100, 104,	/* 0x02, RT_CHANNEL_DOMAIN_5G_ETSI2 */
	108, 112, 116, 120, 124, 128, 132, 136, 140, 149,
	153, 157, 161, 165
};

/*
 * Search the @param channel_num in given @param channel_set
 * @ch_set: the given channel set
 * @ch: the given channel number
 *
 * return the index of channel_num in channel_set, -1 if not found
 */
int rtw_ch_set_search_ch(RT_CHANNEL_INFO *ch_set, const uint32_t	 ch)
{
	int i;
	for(i=0;ch_set[i].ChannelNum!=0;i++){
		if(ch == ch_set[i].ChannelNum)
			break;
	}

	if(i >= ch_set[i].ChannelNum)
		return -1;
	return i;
}

/****************************************************************************

Following are the initialization functions for WiFi MLME

*****************************************************************************/

int init_hw_mlme_ext(struct rtl_priv *rtlpriv)
{
	struct	mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;

	//set_opmode_cmd(rtlpriv, infra_client_with_mlme);//removed

	set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);

	return _SUCCESS;
}

static void init_mlme_ext_priv_value(struct rtl_priv* rtlpriv)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);


	//unsigned char default_channel_set[MAX_CHANNEL_NUM] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 0};
	unsigned char	mixed_datarate[NumRates] = {_1M_RATE_, _2M_RATE_, _5M_RATE_, _11M_RATE_, _6M_RATE_,_9M_RATE_, _12M_RATE_, _18M_RATE_, _24M_RATE_, _36M_RATE_, _48M_RATE_, _54M_RATE_, 0xff};
	unsigned char	mixed_basicrate[NumRates] ={_1M_RATE_, _2M_RATE_, _5M_RATE_, _11M_RATE_, _6M_RATE_, _12M_RATE_, _24M_RATE_, 0xff,};

	atomic_set(&pmlmeext->event_seq, 0);
	pmlmeext->mgnt_seq = 0;//reset to zero when disconnect at client mode

	pmlmeext->cur_channel = rtlpriv->registrypriv.channel;
	pmlmeext->cur_bwmode = CHANNEL_WIDTH_20;
	pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;

	pmlmeext->retry = 0;

	pmlmeext->cur_wireless_mode = WIRELESS_MODE_MAX;

	//memcpy(pmlmeext->channel_set, DefaultChannelPlan[rtlpriv->mlmepriv.ChannelPlan].Channel, DefaultChannelPlan[rtlpriv->mlmepriv.ChannelPlan].Len);
	//memcpy(pmlmeext->channel_set, default_channel_set, MAX_CHANNEL_NUM);
	memcpy(pmlmeext->datarate, mixed_datarate, NumRates);
	memcpy(pmlmeext->basicrate, mixed_basicrate, NumRates);

	if(pmlmeext->cur_channel > 14)
		pmlmeext->tx_rate = IEEE80211_OFDM_RATE_6MB;
	else
		pmlmeext->tx_rate = IEEE80211_CCK_RATE_1MB;

	pmlmeext->sitesurvey_res.state = SCAN_DISABLE;
	pmlmeext->sitesurvey_res.channel_idx = 0;
	pmlmeext->sitesurvey_res.bss_cnt = 0;
	pmlmeext->scan_abort = false;

	pmlmeinfo->state = WIFI_FW_NULL_STATE;
	pmlmeinfo->reauth_count = 0;
	pmlmeinfo->reassoc_count = 0;
	pmlmeinfo->link_count = 0;
	pmlmeinfo->auth_seq = 0;
	pmlmeinfo->auth_algo = dot11AuthAlgrthm_Open;
	pmlmeinfo->key_index = 0;
	pmlmeinfo->iv = 0;

	pmlmeinfo->enc_algo = NO_ENCRYPTION;
	pmlmeinfo->authModeToggle = 0;

	memset(pmlmeinfo->chg_txt, 0, 128);

	pmlmeinfo->slotTime = SHORT_SLOT_TIME;
	pmlmeinfo->preamble_mode = PREAMBLE_AUTO;

	pmlmeinfo->dialogToken = 0;

	pmlmeext->action_public_rxseq = 0xffff;
	pmlmeext->action_public_dialog_token = 0xff;
}

static int has_channel(RT_CHANNEL_INFO *channel_set,
					   uint8_t chanset_size,
					   uint8_t chan) {
	int i;

	for (i = 0; i < chanset_size; i++) {
		if (channel_set[i].ChannelNum == chan) {
			return 1;
		}
	}

	return 0;
}

static void init_channel_list(struct rtl_priv *rtlpriv, RT_CHANNEL_INFO *channel_set,
							  uint8_t chanset_size,
							  struct p2p_channels *channel_list) {

	struct p2p_oper_class_map op_class[] = {
		{ IEEE80211G,  81,   1,  13,  1, BW20 },
		{ IEEE80211G,  82,  14,  14,  1, BW20 },
		{ IEEE80211A, 115,  36,  48,  4, BW20 },
		{ IEEE80211A, 116,  36,  44,  8, BW40PLUS },
		{ IEEE80211A, 117,  40,  48,  8, BW40MINUS },
		{ IEEE80211A, 124, 149, 161,  4, BW20 },
		{ IEEE80211A, 125, 149, 169,  4, BW20 },
		{ IEEE80211A, 126, 149, 157,  8, BW40PLUS },
		{ IEEE80211A, 127, 153, 161,  8, BW40MINUS },
		{ -1, 0, 0, 0, 0, BW20 }
	};

	int cla, op;

	cla = 0;

	for (op = 0; op_class[op].op_class; op++) {
		uint8_t ch;
		struct p2p_oper_class_map *o = &op_class[op];
		struct p2p_reg_class *reg = NULL;

		for (ch = o->min_chan; ch <= o->max_chan; ch += o->inc) {
			if (!has_channel(channel_set, chanset_size, ch)) {
				continue;
			}

			if ((0 < (0x21 & 0xf0)) &&
				((BW40MINUS == o->bw) || (BW40PLUS == o->bw)))
				continue;

			if (reg == NULL) {
				reg = &channel_list->reg_class[cla];
				cla++;
				reg->reg_class = o->op_class;
				reg->channels = 0;
			}
			reg->channel[reg->channels] = ch;
			reg->channels++;
		}
	}
	channel_list->reg_classes = cla;

}

static uint8_t init_channel_set(struct rtl_priv* rtlpriv, uint8_t ChannelPlan, RT_CHANNEL_INFO *channel_set)
{
	uint8_t	index,chanset_size = 0;
	uint8_t	b5GBand = false, b2_4GBand = false;
	uint8_t	Index2G = 0, Index5G=0;

	memset(channel_set, 0, sizeof(RT_CHANNEL_INFO)*MAX_CHANNEL_NUM);

	if (ChannelPlan >= RT_CHANNEL_DOMAIN_MAX &&
	    ChannelPlan != RT_CHANNEL_DOMAIN_REALTEK_DEFINE) {
		DBG_871X("ChannelPlan ID %x error !!!!!\n",ChannelPlan);
		return chanset_size;
	}

	if (IsSupported24G(WIRELESS_MODE_MAX)) {
		b2_4GBand = true;
		Index2G = 0x00;		/* ULLI now index */
	}

	if (IsSupported5G(WIRELESS_MODE_MAX)) {
		b5GBand = true;
		Index5G = 0x00;		/* ULLI now index */
	}

	if (b2_4GBand) {
		for (index = 0; index < 14; index++) {
			channel_set[chanset_size].ChannelNum = RTW_ChannelPlan2G[index];

			if ((RT_CHANNEL_DOMAIN_GLOBAL_DOAMIN == ChannelPlan) ||//Channel 1~11 is active, and 12~14 is passive
			    (RT_CHANNEL_DOMAIN_GLOBAL_DOAMIN_2G == ChannelPlan)) {
				if(channel_set[chanset_size].ChannelNum >= 1 && channel_set[chanset_size].ChannelNum <= 11)
					channel_set[chanset_size].ScanType = SCAN_ACTIVE;
				else if((channel_set[chanset_size].ChannelNum  >= 12 && channel_set[chanset_size].ChannelNum  <= 14))
					channel_set[chanset_size].ScanType  = SCAN_PASSIVE;
			} else if(RT_CHANNEL_DOMAIN_WORLD_WIDE_13 == ChannelPlan ||
				  RT_CHANNEL_DOMAIN_WORLD_WIDE_5G == ChannelPlan ||
				  RT_CHANNEL_DOMAIN_2G_WORLD == Index2G) { // channel 12~13, passive scan
					if (channel_set[chanset_size].ChannelNum <= 11)
						channel_set[chanset_size].ScanType = SCAN_ACTIVE;
					else
						channel_set[chanset_size].ScanType = SCAN_PASSIVE;
			} else {
				channel_set[chanset_size].ScanType = SCAN_ACTIVE;
			}

			chanset_size++;
		}
	}

	if (b5GBand) {
		for (index = 0;index < 24; index++) {
#ifdef CONFIG_DFS
			channel_set[chanset_size].ChannelNum = RTW_ChannelPlan5G[index];
			if (channel_set[chanset_size].ChannelNum <= 48 ||
			    channel_set[chanset_size].ChannelNum >= 149) {
				if(RT_CHANNEL_DOMAIN_WORLD_WIDE_5G == ChannelPlan)//passive scan for all 5G channels
					channel_set[chanset_size].ScanType = SCAN_PASSIVE;
				else
					channel_set[chanset_size].ScanType = SCAN_ACTIVE;
			} else {
				channel_set[chanset_size].ScanType = SCAN_PASSIVE;
			}
			chanset_size++;
#else /* CONFIG_DFS */
			if (RTW_ChannelPlan5G[Index5G].Channel[index] <= 48 ||
			    RTW_ChannelPlan5G[Index5G].Channel[index] >= 149 ) {
				channel_set[chanset_size].ChannelNum = RTW_ChannelPlan5G[Index5G].Channel[index];
				if(RT_CHANNEL_DOMAIN_WORLD_WIDE_5G == ChannelPlan)//passive scan for all 5G channels
					channel_set[chanset_size].ScanType = SCAN_PASSIVE;
				else
					channel_set[chanset_size].ScanType = SCAN_ACTIVE;
				DBG_871X("%s(): channel_set[%d].ChannelNum = %d\n", __FUNCTION__, chanset_size, channel_set[chanset_size].ChannelNum);
				chanset_size++;
			}
#endif /* CONFIG_DFS */
		}
	}

	DBG_871X("%s ChannelPlan ID %x Chan num:%d  \n",__FUNCTION__,ChannelPlan,chanset_size);
	return chanset_size;
}

int	init_mlme_ext_priv(struct rtl_priv* rtlpriv)
{
	int	res = _SUCCESS;
	struct registry_priv* pregistrypriv = &rtlpriv->registrypriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	// We don't need to memset rtlpriv->XXX to zero, because rtlpriv is allocated by rtw_zvmalloc().
	//memset((uint8_t *)pmlmeext, 0, sizeof(struct mlme_ext_priv));

	pmlmeext->rtlpriv = rtlpriv;

	//fill_fwpriv(rtlpriv, &(pmlmeext->fwpriv));

	init_mlme_ext_priv_value(rtlpriv);
	pmlmeinfo->bAcceptAddbaReq = pregistrypriv->bAcceptAddbaReq;

	init_mlme_ext_timer(rtlpriv);

#ifdef CONFIG_AP_MODE
	init_mlme_ap_info(rtlpriv);
#endif

	pmlmeext->max_chan_nums = init_channel_set(rtlpriv, pmlmepriv->ChannelPlan,pmlmeext->channel_set);
	init_channel_list(rtlpriv, pmlmeext->channel_set, pmlmeext->max_chan_nums, &pmlmeext->channel_list);

	pmlmeext->chan_scan_time = SURVEY_TO;
	pmlmeext->mlmeext_init = true;

#ifdef DBG_FIXED_CHAN
	pmlmeext->fixed_chan = 0xFF;
#endif

	return res;

}

void free_mlme_ext_priv (struct mlme_ext_priv *pmlmeext)
{
	struct rtl_priv *rtlpriv = pmlmeext->rtlpriv;

	if (!rtlpriv)
		return;

	if (rtlpriv->bDriverStopped == true)
	{
		del_timer_sync_ex(&pmlmeext->survey_timer);
		del_timer_sync_ex(&pmlmeext->link_timer);
		//del_timer_sync_ex(&pmlmeext->ADDBA_timer);
	}
}

static uint8_t cmp_pkt_chnl_diff(struct rtl_priv *rtlpriv,uint8_t * pframe,uint packet_len)
{	// if the channel is same, return 0. else return channel differential
	uint len;
	uint8_t channel;
	uint8_t *p;
	p = rtw_get_ie(pframe + WLAN_HDR_A3_LEN + _BEACON_IE_OFFSET_, _DSSET_IE_, &len, packet_len - _BEACON_IE_OFFSET_);
	if (p)
	{
		channel = *(p + 2);
		if(rtlpriv->mlmeextpriv.cur_channel >= channel)
		{
			return (rtlpriv->mlmeextpriv.cur_channel - channel);
		}
		else
		{
			return (channel-rtlpriv->mlmeextpriv.cur_channel);
		}
	}
	else
	{
		return 0;
	}
}

static void _mgt_dispatcher(struct rtl_priv *rtlpriv, struct mlme_handler *ptable, struct recv_frame *precv_frame)
{
	uint8_t bc_addr[ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};
	uint8_t *pframe = precv_frame->rx_data;

	  if(ptable->func)
        {
       	 //receive the frames that ra(a1) is my address or ra(a1) is bc address.
		if (!_rtw_memcmp(GetAddr1Ptr(pframe), rtlpriv->mac80211.mac_addr, ETH_ALEN) &&
			!_rtw_memcmp(GetAddr1Ptr(pframe), bc_addr, ETH_ALEN))
		{
			return;
		}

		ptable->func(rtlpriv, precv_frame);
        }

}

void mgt_dispatcher(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	int index;
	struct mlme_handler *ptable;
#ifdef CONFIG_AP_MODE
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
#endif //CONFIG_AP_MODE
	uint8_t bc_addr[ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};
	uint8_t *pframe = precv_frame->rx_data;
	struct sta_info *psta = rtw_get_stainfo(&rtlpriv->stapriv, GetAddr2Ptr(pframe));


	if (GetFrameType(pframe) != WIFI_MGT_TYPE)
	{
		return;
	}

	//receive the frames that ra(a1) is my address or ra(a1) is bc address.
	if (!_rtw_memcmp(GetAddr1Ptr(pframe), rtlpriv->mac80211.mac_addr, ETH_ALEN) &&
		!_rtw_memcmp(GetAddr1Ptr(pframe), bc_addr, ETH_ALEN))
	{
		return;
	}

	ptable = mlme_sta_tbl;

	index = GetFrameSubType(pframe) >> 4;


	if (index > 13)
	{
		return;
	}
	ptable += index;

#if 1
	if (psta != NULL)
	{
		if (GetRetry(pframe))
		{
			if (precv_frame->attrib.seq_num == psta->RxMgmtFrameSeqNum)
			{
				/* drop the duplicate management frame */
				DBG_871X("Drop duplicate management frame with seq_num = %d.\n", precv_frame->attrib.seq_num);
				return;
			}
		}
		psta->RxMgmtFrameSeqNum = precv_frame->attrib.seq_num;
	}
#else

	if(GetRetry(pframe))
	{
		//RT_TRACE(_module_rtl871x_mlme_c_,_drv_err_,("drop due to decache!\n"));
		//return;
	}
#endif

#ifdef CONFIG_AP_MODE
	switch (GetFrameSubType(pframe))
	{
		case WIFI_AUTH:
			if(check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)
				ptable->func = &OnAuth;
			else
				ptable->func = &OnAuthClient;
			//pass through
		case WIFI_ASSOCREQ:
		case WIFI_REASSOCREQ:
			_mgt_dispatcher(rtlpriv, ptable, precv_frame);
			break;
		case WIFI_PROBEREQ:
			if(check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)
			{
				_mgt_dispatcher(rtlpriv, ptable, precv_frame);
			}
			else
				_mgt_dispatcher(rtlpriv, ptable, precv_frame);
			break;
		case WIFI_BEACON:
			_mgt_dispatcher(rtlpriv, ptable, precv_frame);
			break;
		case WIFI_ACTION:
			//if(check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)
			_mgt_dispatcher(rtlpriv, ptable, precv_frame);
			break;
		default:
			_mgt_dispatcher(rtlpriv, ptable, precv_frame);
			break;
	}
#else

	_mgt_dispatcher(rtlpriv, ptable, precv_frame);

#endif

}

/****************************************************************************

Following are the callback functions for each subtype of the management frames

*****************************************************************************/

unsigned int OnProbeReq(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	unsigned int	ielen;
	unsigned char	*p;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 	*cur = &(pmlmeinfo->network);
	uint8_t *pframe = precv_frame->rx_data;
	uint len = precv_frame->len;
	uint8_t is_valid_p2p_probereq = false;


	if(check_fwstate(pmlmepriv, WIFI_STATION_STATE))
	{
		return _SUCCESS;
	}

	if(check_fwstate(pmlmepriv, _FW_LINKED) == false &&
		check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE|WIFI_AP_STATE)==false)
	{
		return _SUCCESS;
	}


	//DBG_871X("+OnProbeReq\n");


	p = rtw_get_ie(pframe + WLAN_HDR_A3_LEN + _PROBEREQ_IE_OFFSET_, _SSID_IE_, (int *)&ielen,
			len - WLAN_HDR_A3_LEN - _PROBEREQ_IE_OFFSET_);


	//check (wildcard) SSID
	if (p != NULL)
	{
		if(is_valid_p2p_probereq == true)
		{
			goto _issue_probersp;
		}

		if ( (ielen != 0 && false ==_rtw_memcmp((void *)(p+2), (void *)cur->Ssid.Ssid, cur->Ssid.SsidLength))
			|| (ielen == 0 && pmlmeinfo->hidden_ssid_mode)
		)
		{
			return _SUCCESS;
		}

_issue_probersp:

		if(check_fwstate(pmlmepriv, _FW_LINKED) == true &&
			pmlmepriv->cur_network.join_res == true)
		{
			//DBG_871X("+issue_probersp during ap mode\n");
			issue_probersp(rtlpriv, get_sa(pframe), is_valid_p2p_probereq);
		}

	}

	return _SUCCESS;

}

unsigned int OnProbeRsp(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	struct sta_info		*psta;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct sta_priv		*pstapriv = &rtlpriv->stapriv;
	uint8_t	*pframe = precv_frame->rx_data;




	if (pmlmeext->sitesurvey_res.state == SCAN_PROCESS)
	{
		report_survey_event(rtlpriv, precv_frame);
		return _SUCCESS;
	}

	#if 0 //move to validate_recv_mgnt_frame
	if (_rtw_memcmp(GetAddr3Ptr(pframe), get_my_bssid(&pmlmeinfo->network), ETH_ALEN))
	{
		if (pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS)
		{
			if ((psta = rtw_get_stainfo(pstapriv, GetAddr2Ptr(pframe))) != NULL)
			{
				psta->sta_stats.rx_mgnt_pkts++;
			}
		}
	}
	#endif

	return _SUCCESS;

}

unsigned int OnBeacon(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	int cam_idx;
	struct sta_info	*psta;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct sta_priv	*pstapriv = &rtlpriv->stapriv;
	uint8_t *pframe = precv_frame->rx_data;
	uint len = precv_frame->len;
	WLAN_BSSID_EX *pbss;
	int ret = _SUCCESS;
	uint8_t *p = NULL;
	uint32_t	 ielen = 0;

#ifdef CONFIG_ATTEMPT_TO_FIX_AP_BEACON_ERROR
	p = rtw_get_ie(pframe + sizeof(struct rtw_ieee80211_hdr_3addr) + _BEACON_IE_OFFSET_, _EXT_SUPPORTEDRATES_IE_, &ielen, precv_frame->len -sizeof(struct rtw_ieee80211_hdr_3addr) - _BEACON_IE_OFFSET_);
	if ((p != NULL) && (ielen > 0))
	{
		if ((*(p + 1 + ielen) == 0x2D) && (*(p + 2 + ielen) != 0x2D))
		{
			/* Invalid value 0x2D is detected in Extended Supported Rates (ESR) IE. Try to fix the IE length to avoid failed Beacon parsing. */
		       	DBG_871X("[WIFIDBG] Error in ESR IE is detected in Beacon of BSSID:"MAC_FMT". Fix the length of ESR IE to avoid failed Beacon parsing.\n", MAC_ARG(GetAddr3Ptr(pframe)));
		       	*(p + 1) = ielen - 1;
		}
	}
#endif

	if (pmlmeext->sitesurvey_res.state == SCAN_PROCESS)
	{
		report_survey_event(rtlpriv, precv_frame);
		return _SUCCESS;
	}

	if (_rtw_memcmp(GetAddr3Ptr(pframe), get_my_bssid(&pmlmeinfo->network), ETH_ALEN))
	{
		if (pmlmeinfo->state & WIFI_FW_AUTH_NULL)
		{
			//we should update current network before auth, or some IE is wrong
			pbss = (WLAN_BSSID_EX*)rtw_malloc(sizeof(WLAN_BSSID_EX));
			if (pbss) {
				if (collect_bss_info(rtlpriv, precv_frame, pbss) == _SUCCESS) {
					update_network(&(pmlmepriv->cur_network.network), pbss, rtlpriv, true);
					rtw_get_bcn_info(&(pmlmepriv->cur_network));
				}
				rtw_mfree(pbss);
			}

			//check the vendor of the assoc AP
			pmlmeinfo->assoc_AP_vendor = check_assoc_AP(pframe+sizeof(struct rtw_ieee80211_hdr_3addr), len-sizeof(struct rtw_ieee80211_hdr_3addr));

			//update TSF Value
			update_TSF(pmlmeext, pframe, len);

			//start auth
			start_clnt_auth(rtlpriv);

			return _SUCCESS;
		}

		if(((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE) && (pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS))
		{
			if ((psta = rtw_get_stainfo(pstapriv, GetAddr2Ptr(pframe))) != NULL)
			{
				#ifdef CONFIG_PATCH_JOIN_WRONG_CHANNEL
				//Merge from 8712 FW code
				if (cmp_pkt_chnl_diff(rtlpriv,pframe,len) != 0)
				{            // join wrong channel, deauth and reconnect
					issue_deauth(rtlpriv, (&(pmlmeinfo->network))->MacAddress, WLAN_REASON_DEAUTH_LEAVING);

					report_del_sta_event(rtlpriv,(&(pmlmeinfo->network))->MacAddress, WLAN_REASON_JOIN_WRONG_CHANNEL);
					pmlmeinfo->state &= (~WIFI_FW_ASSOC_SUCCESS);
					return _SUCCESS;
				}
				#endif //CONFIG_PATCH_JOIN_WRONG_CHANNEL

				ret = rtw_check_bcn_info(rtlpriv, pframe, len);
				if (!ret) {
						DBG_871X_LEVEL(_drv_always_, "ap has changed, disconnect now\n ");
						receive_disconnect(rtlpriv, pmlmeinfo->network.MacAddress , 65535);
						return _SUCCESS;
				}
				//update WMM, ERP in the beacon
				//todo: the timer is used instead of the number of the beacon received
				if ((sta_rx_pkts(psta) & 0xf) == 0)
				{
					//DBG_871X("update_bcn_info\n");
					update_beacon_info(rtlpriv, pframe, len, psta);
				}

#ifdef CONFIG_DFS
				process_csa_ie(rtlpriv, pframe, len);	//channel switch announcement
#endif //CONFIG_DFS
				#if 0 //move to validate_recv_mgnt_frame
				psta->sta_stats.rx_mgnt_pkts++;
				#endif
			}
		}
		else if((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)
		{
			if ((psta = rtw_get_stainfo(pstapriv, GetAddr2Ptr(pframe))) != NULL)
			{
				//update WMM, ERP in the beacon
				//todo: the timer is used instead of the number of the beacon received
				if ((sta_rx_pkts(psta) & 0xf) == 0)
				{
					//DBG_871X("update_bcn_info\n");
					update_beacon_info(rtlpriv, pframe, len, psta);
				}

				#if 0 //move to validate_recv_mgnt_frame
				psta->sta_stats.rx_mgnt_pkts++;
				#endif
			}
			else
			{
				//allocate a new CAM entry for IBSS station
				if ((cam_idx = allocate_fw_sta_entry(rtlpriv)) == NUM_STA)
				{
					goto _END_ONBEACON_;
				}

				//get supported rate
				if (update_sta_support_rate(rtlpriv, (pframe + WLAN_HDR_A3_LEN + _BEACON_IE_OFFSET_), (len - WLAN_HDR_A3_LEN - _BEACON_IE_OFFSET_), cam_idx) == _FAIL)
				{
					pmlmeinfo->FW_sta_info[cam_idx].status = 0;
					goto _END_ONBEACON_;
				}

				//update TSF Value
				update_TSF(pmlmeext, pframe, len);

				//report sta add event
				report_add_sta_event(rtlpriv, GetAddr2Ptr(pframe), cam_idx);
			}
		}
	}

_END_ONBEACON_:

	return _SUCCESS;

}

unsigned int OnAuth(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
#ifdef CONFIG_AP_MODE
	unsigned int	auth_mode, seq, ie_len;
	unsigned char	*sa, *p;
	u16	algorithm;
	int	status;
	static struct sta_info stat;
	struct	sta_info	*pstat=NULL;
	struct	sta_priv *pstapriv = &rtlpriv->stapriv;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t *pframe = precv_frame->rx_data;
	uint len = precv_frame->len;
	uint8_t	offset = 0;

	if((pmlmeinfo->state&0x03) != WIFI_FW_AP_STATE)
		return _FAIL;

	DBG_871X("+OnAuth\n");

	sa = GetAddr2Ptr(pframe);

	auth_mode = psecuritypriv->dot11AuthAlgrthm;

	if (GetPrivacy(pframe))
	{
		uint8_t	*iv;
		struct rx_pkt_attrib	 *prxattrib = &(precv_frame->attrib);

		prxattrib->hdrlen = WLAN_HDR_A3_LEN;
		prxattrib->encrypt = WEP40_ENCRYPTION;

		iv = pframe+prxattrib->hdrlen;
		prxattrib->key_index = ((iv[3]>>6)&0x3);

		prxattrib->iv_len = 4;
		prxattrib->icv_len = 4;

		rtw_wep_decrypt(rtlpriv, precv_frame);

		offset = 4;
	}

	algorithm = le16_to_cpu(*(u16 *)((u8 *)pframe + WLAN_HDR_A3_LEN + offset));
	seq 	= le16_to_cpu(*(u16 *)((u8 *)pframe + WLAN_HDR_A3_LEN + offset + 2));

	DBG_871X("auth alg=%x, seq=%X\n", algorithm, seq);

	if (auth_mode == 2 &&
			psecuritypriv->dot11PrivacyAlgrthm != WEP40_ENCRYPTION &&
			psecuritypriv->dot11PrivacyAlgrthm != WEP104_ENCRYPTION)
		auth_mode = 0;

	if ((algorithm > 0 && auth_mode == 0) ||	// rx a shared-key auth but shared not enabled
		(algorithm == 0 && auth_mode == 1) )	// rx a open-system auth but shared-key is enabled
	{
		DBG_871X("auth rejected due to bad alg [alg=%d, auth_mib=%d] %02X%02X%02X%02X%02X%02X\n",
			algorithm, auth_mode, sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);

		status = _STATS_NO_SUPP_ALG_;

		goto auth_fail;
	}

	if(rtw_access_ctrl(rtlpriv, sa) == false)
	{
		status = _STATS_UNABLE_HANDLE_STA_;
		goto auth_fail;
	}

	pstat = rtw_get_stainfo(pstapriv, sa);
	if (pstat == NULL)
	{
		// allocate a new one
		DBG_871X("going to alloc stainfo for sa="MAC_FMT"\n",  MAC_ARG(sa));
		pstat = rtw_alloc_stainfo(pstapriv, sa);
		if (pstat == NULL)
		{
			DBG_871X(" Exceed the upper limit of supported clients...\n");
			status = _STATS_UNABLE_HANDLE_STA_;
			goto auth_fail;
		}

		pstat->state = WIFI_FW_AUTH_NULL;
		pstat->auth_seq = 0;

		//pstat->flags = 0;
		//pstat->capability = 0;
	}
	else
	{
		spin_lock_bh(&pstapriv->asoc_list_lock);
		if(list_empty(&pstat->asoc_list)==false)
		{
			list_del_init(&pstat->asoc_list);
			pstapriv->asoc_list_cnt--;
			if (pstat->expire_to > 0)
			{
				//TODO: STA re_auth within expire_to
			}
		}
		spin_unlock_bh(&pstapriv->asoc_list_lock);

		if (seq==1) {
			//TODO: STA re_auth and auth timeout
		}
	}

	spin_lock_bh(&pstapriv->auth_list_lock);
	if (list_empty(&pstat->auth_list))
	{
		list_add_tail(&pstat->auth_list, &pstapriv->auth_list);
		pstapriv->auth_list_cnt++;
	}
	spin_unlock_bh(&pstapriv->auth_list_lock);

	if (pstat->auth_seq == 0)
		pstat->expire_to = pstapriv->auth_to;

	if ((pstat->auth_seq + 1) != seq)
	{
		DBG_871X("(1)auth rejected because out of seq [rx_seq=%d, exp_seq=%d]!\n",
			seq, pstat->auth_seq+1);
		status = _STATS_OUT_OF_AUTH_SEQ_;
		goto auth_fail;
	}

	if (algorithm==0 && (auth_mode == 0 || auth_mode == 2 || auth_mode == 3))
	{
		if (seq == 1)
		{
			pstat->state &= ~WIFI_FW_AUTH_NULL;
			pstat->state |= WIFI_FW_AUTH_SUCCESS;
			pstat->expire_to = pstapriv->assoc_to;
			pstat->authalg = algorithm;
		}
		else
		{
			DBG_871X("(2)auth rejected because out of seq [rx_seq=%d, exp_seq=%d]!\n",
				seq, pstat->auth_seq+1);
			status = _STATS_OUT_OF_AUTH_SEQ_;
			goto auth_fail;
		}
	}
	else // shared system or auto authentication
	{
		if (seq == 1)
		{
			//prepare for the challenging txt...

			//get_random_bytes((void *)pstat->chg_txt, 128);//TODO:
			memset((void *)pstat->chg_txt, 78, 128);

			pstat->state &= ~WIFI_FW_AUTH_NULL;
			pstat->state |= WIFI_FW_AUTH_STATE;
			pstat->authalg = algorithm;
			pstat->auth_seq = 2;
		}
		else if (seq == 3)
		{
			//checking for challenging txt...
			DBG_871X("checking for challenging txt...\n");

			p = rtw_get_ie(pframe + WLAN_HDR_A3_LEN + 4 + _AUTH_IE_OFFSET_ , _CHLGETXT_IE_, (int *)&ie_len,
					len - WLAN_HDR_A3_LEN - _AUTH_IE_OFFSET_ - 4);

			if((p==NULL) || (ie_len<=0))
			{
				DBG_871X("auth rejected because challenge failure!(1)\n");
				status = _STATS_CHALLENGE_FAIL_;
				goto auth_fail;
			}

			if (_rtw_memcmp((void *)(p + 2), pstat->chg_txt, 128))
			{
				pstat->state &= (~WIFI_FW_AUTH_STATE);
				pstat->state |= WIFI_FW_AUTH_SUCCESS;
				// challenging txt is correct...
				pstat->expire_to =  pstapriv->assoc_to;
			}
			else
			{
				DBG_871X("auth rejected because challenge failure!\n");
				status = _STATS_CHALLENGE_FAIL_;
				goto auth_fail;
			}
		}
		else
		{
			DBG_871X("(3)auth rejected because out of seq [rx_seq=%d, exp_seq=%d]!\n",
				seq, pstat->auth_seq+1);
			status = _STATS_OUT_OF_AUTH_SEQ_;
			goto auth_fail;
		}
	}


	// Now, we are going to issue_auth...
	pstat->auth_seq = seq + 1;

	issue_auth(rtlpriv, pstat, (unsigned short)(_STATS_SUCCESSFUL_));

	if (pstat->state & WIFI_FW_AUTH_SUCCESS)
		pstat->auth_seq = 0;


	return _SUCCESS;

auth_fail:

	if(pstat)
		rtw_free_stainfo(rtlpriv , pstat);

	pstat = &stat;
	memset((char *)pstat, '\0', sizeof(stat));
	pstat->auth_seq = 2;
	memcpy(pstat->hwaddr, sa, 6);

	issue_auth(rtlpriv, pstat, (unsigned short)status);

#endif
	return _FAIL;

}

unsigned int OnAuthClient(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	unsigned int	seq, len, status, algthm, offset;
	unsigned char	*p;
	unsigned int	go2asoc = 0;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t *pframe = precv_frame->rx_data;
	uint pkt_len = precv_frame->len;

	DBG_871X("%s\n", __FUNCTION__);

	//check A1 matches or not
	if (!_rtw_memcmp(rtlpriv->mac80211.mac_addr, get_da(pframe), ETH_ALEN))
		return _SUCCESS;

	if (!(pmlmeinfo->state & WIFI_FW_AUTH_STATE))
		return _SUCCESS;

	offset = (GetPrivacy(pframe))? 4: 0;

	algthm 	= le16_to_cpu(*(unsigned short *)((u8 *)pframe + WLAN_HDR_A3_LEN + offset));
	seq 	= le16_to_cpu(*(unsigned short *)((u8 *)pframe + WLAN_HDR_A3_LEN + offset + 2));
	status 	= le16_to_cpu(*(unsigned short *)((u8 *)pframe + WLAN_HDR_A3_LEN + offset + 4));

	if (status != 0)
	{
		DBG_871X("clnt auth fail, status: %d\n", status);
		if(status == 13)//&& pmlmeinfo->auth_algo == dot11AuthAlgrthm_Auto)
		{
			if(pmlmeinfo->auth_algo == dot11AuthAlgrthm_Shared)
				pmlmeinfo->auth_algo = dot11AuthAlgrthm_Open;
			else
				pmlmeinfo->auth_algo = dot11AuthAlgrthm_Shared;
			//pmlmeinfo->reauth_count = 0;
		}

		set_link_timer(pmlmeext, 1);
		goto authclnt_fail;
	}

	if (seq == 2)
	{
		if (pmlmeinfo->auth_algo == dot11AuthAlgrthm_Shared)
		{
			 // legendary shared system
			p = rtw_get_ie(pframe + WLAN_HDR_A3_LEN + _AUTH_IE_OFFSET_, _CHLGETXT_IE_, (int *)&len,
				pkt_len - WLAN_HDR_A3_LEN - _AUTH_IE_OFFSET_);

			if (p == NULL)
			{
				//DBG_871X("marc: no challenge text?\n");
				goto authclnt_fail;
			}

			memcpy((void *)(pmlmeinfo->chg_txt), (void *)(p + 2), len);
			pmlmeinfo->auth_seq = 3;
			issue_auth(rtlpriv, NULL, 0);
			set_link_timer(pmlmeext, REAUTH_TO);

			return _SUCCESS;
		}
		else
		{
			// open system
			go2asoc = 1;
		}
	}
	else if (seq == 4)
	{
		if (pmlmeinfo->auth_algo == dot11AuthAlgrthm_Shared)
		{
			go2asoc = 1;
		}
		else
		{
			goto authclnt_fail;
		}
	}
	else
	{
		// this is also illegal
		//DBG_871X("marc: clnt auth failed due to illegal seq=%x\n", seq);
		goto authclnt_fail;
	}

	if (go2asoc)
	{
		DBG_871X_LEVEL(_drv_always_, "auth success, start assoc\n");
		start_clnt_assoc(rtlpriv);
		return _SUCCESS;
	}

authclnt_fail:

	//pmlmeinfo->state &= ~(WIFI_FW_AUTH_STATE);

	return _FAIL;

}

unsigned int OnAssocReq(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
#ifdef CONFIG_AP_MODE
	u16 capab_info, listen_interval;
	struct rtw_ieee802_11_elems elems;
	struct sta_info	*pstat;
	unsigned char		reassoc, *p, *pos, *wpa_ie;
	unsigned char WMM_IE[] = {0x00, 0x50, 0xf2, 0x02, 0x00, 0x01};
	int		i, ie_len, wpa_ie_len, left;
	unsigned char		supportRate[16];
	int					supportRateNum;
	unsigned short		status = _STATS_SUCCESSFUL_;
	unsigned short		frame_type, ie_offset=0;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 	*cur = &(pmlmeinfo->network);
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	uint8_t *pframe = precv_frame->rx_data;
	uint pkt_len = precv_frame->len;

	if((pmlmeinfo->state&0x03) != WIFI_FW_AP_STATE)
		return _FAIL;

	frame_type = GetFrameSubType(pframe);
	if (frame_type == WIFI_ASSOCREQ)
	{
		reassoc = 0;
		ie_offset = _ASOCREQ_IE_OFFSET_;
	}
	else // WIFI_REASSOCREQ
	{
		reassoc = 1;
		ie_offset = _REASOCREQ_IE_OFFSET_;
	}


	if (pkt_len < IEEE80211_3ADDR_LEN + ie_offset) {
		DBG_871X("handle_assoc(reassoc=%d) - too short payload (len=%lu)"
		       "\n", reassoc, (unsigned long)pkt_len);
		return _FAIL;
	}

	pstat = rtw_get_stainfo(pstapriv, GetAddr2Ptr(pframe));
	if (pstat == (struct sta_info *)NULL)
	{
		status = _RSON_CLS2_;
		goto asoc_class2_error;
	}

	capab_info = RTW_GET_LE16(pframe + WLAN_HDR_A3_LEN);
	//capab_info = le16_to_cpu(*(unsigned short *)(pframe + WLAN_HDR_A3_LEN));
	//listen_interval = le16_to_cpu(*(unsigned short *)(pframe + WLAN_HDR_A3_LEN+2));
	listen_interval = RTW_GET_LE16(pframe + WLAN_HDR_A3_LEN+2);

	left = pkt_len - (IEEE80211_3ADDR_LEN + ie_offset);
	pos = pframe + (IEEE80211_3ADDR_LEN + ie_offset);


	DBG_871X("%s\n", __FUNCTION__);

	// check if this stat has been successfully authenticated/assocated
	if (!((pstat->state) & WIFI_FW_AUTH_SUCCESS))
	{
		if (!((pstat->state) & WIFI_FW_ASSOC_SUCCESS))
		{
			status = _RSON_CLS2_;
			goto asoc_class2_error;
		}
		else
		{
			pstat->state &= (~WIFI_FW_ASSOC_SUCCESS);
			pstat->state |= WIFI_FW_ASSOC_STATE;
		}
	}
	else
	{
		pstat->state &= (~WIFI_FW_AUTH_SUCCESS);
		pstat->state |= WIFI_FW_ASSOC_STATE;
	}

	pstat->capability = capab_info;

	//now parse all ieee802_11 ie to point to elems
	if (rtw_ieee802_11_parse_elems(pos, left, &elems, 1) == ParseFailed ||
	    !elems.ssid) {
		DBG_871X("STA " MAC_FMT " sent invalid association request\n",
		       MAC_ARG(pstat->hwaddr));
		status = _STATS_FAILURE_;
		goto OnAssocReqFail;
	}


	// now we should check all the fields...
	// checking SSID
	p = rtw_get_ie(pframe + WLAN_HDR_A3_LEN + ie_offset, _SSID_IE_, &ie_len,
		pkt_len - WLAN_HDR_A3_LEN - ie_offset);
	if (p == NULL)
	{
		status = _STATS_FAILURE_;
	}

	if (ie_len == 0) // broadcast ssid, however it is not allowed in assocreq
		status = _STATS_FAILURE_;
	else
	{
		// check if ssid match
		if (!_rtw_memcmp((void *)(p+2), cur->Ssid.Ssid, cur->Ssid.SsidLength))
			status = _STATS_FAILURE_;

		if (ie_len != cur->Ssid.SsidLength)
			status = _STATS_FAILURE_;
	}

	if(_STATS_SUCCESSFUL_ != status)
		goto OnAssocReqFail;

	// check if the supported rate is ok
	p = rtw_get_ie(pframe + WLAN_HDR_A3_LEN + ie_offset, _SUPPORTEDRATES_IE_, &ie_len, pkt_len - WLAN_HDR_A3_LEN - ie_offset);
	if (p == NULL) {
		DBG_871X("Rx a sta assoc-req which supported rate is empty!\n");
		// use our own rate set as statoin used
		//memcpy(supportRate, AP_BSSRATE, AP_BSSRATE_LEN);
		//supportRateNum = AP_BSSRATE_LEN;

		status = _STATS_FAILURE_;
		goto OnAssocReqFail;
	}
	else {
		memcpy(supportRate, p+2, ie_len);
		supportRateNum = ie_len;

		p = rtw_get_ie(pframe + WLAN_HDR_A3_LEN + ie_offset, _EXT_SUPPORTEDRATES_IE_ , &ie_len,
				pkt_len - WLAN_HDR_A3_LEN - ie_offset);
		if (p !=  NULL) {

			if(supportRateNum<=sizeof(supportRate))
			{
				memcpy(supportRate+supportRateNum, p+2, ie_len);
				supportRateNum += ie_len;
			}
		}
	}

	//todo: mask supportRate between AP & STA -> move to update raid
	//get_matched_rate(pmlmeext, supportRate, &supportRateNum, 0);

	//update station supportRate
	pstat->bssratelen = supportRateNum;
	memcpy(pstat->bssrateset, supportRate, supportRateNum);
	UpdateBrateTblForSoftAP(pstat->bssrateset, pstat->bssratelen);

	//check RSN/WPA/WPS
	pstat->dot8021xalg = 0;
      	pstat->wpa_psk = 0;
	pstat->wpa_group_cipher = 0;
	pstat->wpa2_group_cipher = 0;
	pstat->wpa_pairwise_cipher = 0;
	pstat->wpa2_pairwise_cipher = 0;
	memset(pstat->wpa_ie, 0, sizeof(pstat->wpa_ie));
	if((psecuritypriv->wpa_psk & BIT(1)) && elems.rsn_ie) {

		int group_cipher=0, pairwise_cipher=0;

		wpa_ie = elems.rsn_ie;
		wpa_ie_len = elems.rsn_ie_len;

		if(rtw_parse_wpa2_ie(wpa_ie-2, wpa_ie_len+2, &group_cipher, &pairwise_cipher, NULL) == _SUCCESS)
		{
			pstat->dot8021xalg = 1;//psk,  todo:802.1x
			pstat->wpa_psk |= BIT(1);

			pstat->wpa2_group_cipher = group_cipher&psecuritypriv->wpa2_group_cipher;
			pstat->wpa2_pairwise_cipher = pairwise_cipher&psecuritypriv->wpa2_pairwise_cipher;

			if(!pstat->wpa2_group_cipher)
				status = WLAN_STATUS_GROUP_CIPHER_NOT_VALID;

			if(!pstat->wpa2_pairwise_cipher)
				status = WLAN_STATUS_PAIRWISE_CIPHER_NOT_VALID;
		}
		else
		{
			status = WLAN_STATUS_INVALID_IE;
		}

	} else if ((psecuritypriv->wpa_psk & BIT(0)) && elems.wpa_ie) {

		int group_cipher=0, pairwise_cipher=0;

		wpa_ie = elems.wpa_ie;
		wpa_ie_len = elems.wpa_ie_len;

		if(rtw_parse_wpa_ie(wpa_ie-2, wpa_ie_len+2, &group_cipher, &pairwise_cipher, NULL) == _SUCCESS)
		{
			pstat->dot8021xalg = 1;//psk,  todo:802.1x
			pstat->wpa_psk |= BIT(0);

			pstat->wpa_group_cipher = group_cipher&psecuritypriv->wpa_group_cipher;
			pstat->wpa_pairwise_cipher = pairwise_cipher&psecuritypriv->wpa_pairwise_cipher;

			if(!pstat->wpa_group_cipher)
				status = WLAN_STATUS_GROUP_CIPHER_NOT_VALID;

			if(!pstat->wpa_pairwise_cipher)
				status = WLAN_STATUS_PAIRWISE_CIPHER_NOT_VALID;

		}
		else
		{
			status = WLAN_STATUS_INVALID_IE;
		}

	} else {
		wpa_ie = NULL;
		wpa_ie_len = 0;
	}

	if(_STATS_SUCCESSFUL_ != status)
		goto OnAssocReqFail;

	pstat->flags &= ~(WLAN_STA_WPS | WLAN_STA_MAYBE_WPS);
	//if (hapd->conf->wps_state && wpa_ie == NULL) { //todo: to check ap if supporting WPS
	if(wpa_ie == NULL) {
		if (elems.wps_ie) {
			DBG_871X("STA included WPS IE in "
				   "(Re)Association Request - assume WPS is "
				   "used\n");
			pstat->flags |= WLAN_STA_WPS;
			//wpabuf_free(sta->wps_ie);
			//sta->wps_ie = wpabuf_alloc_copy(elems.wps_ie + 4,
			//				elems.wps_ie_len - 4);
		} else {
			DBG_871X("STA did not include WPA/RSN IE "
				   "in (Re)Association Request - possible WPS "
				   "use\n");
			pstat->flags |= WLAN_STA_MAYBE_WPS;
		}


		// AP support WPA/RSN, and sta is going to do WPS, but AP is not ready
		// that the selected registrar of AP is _FLASE
		if((psecuritypriv->wpa_psk >0)
			&& (pstat->flags & (WLAN_STA_WPS|WLAN_STA_MAYBE_WPS)))
		{
			if(pmlmepriv->wps_beacon_ie)
			{
				uint8_t selected_registrar = 0;

				rtw_get_wps_attr_content(pmlmepriv->wps_beacon_ie, pmlmepriv->wps_beacon_ie_len, WPS_ATTR_SELECTED_REGISTRAR , &selected_registrar, NULL);

				if(!selected_registrar)
				{
					DBG_871X("selected_registrar is false , or AP is not ready to do WPS\n");

					status = _STATS_UNABLE_HANDLE_STA_;

					goto OnAssocReqFail;
				}
			}
		}

	}
	else
	{
		int copy_len;

		if(psecuritypriv->wpa_psk == 0)
		{
			DBG_871X("STA " MAC_FMT ": WPA/RSN IE in association "
		       	"request, but AP don't support WPA/RSN\n", MAC_ARG(pstat->hwaddr));

			status = WLAN_STATUS_INVALID_IE;

			goto OnAssocReqFail;

		}

		if (elems.wps_ie) {
			DBG_871X("STA included WPS IE in "
				   "(Re)Association Request - WPS is "
				   "used\n");
			pstat->flags |= WLAN_STA_WPS;
			copy_len=0;
		}
		else
		{
			copy_len = ((wpa_ie_len+2) > sizeof(pstat->wpa_ie)) ? (sizeof(pstat->wpa_ie)):(wpa_ie_len+2);
		}


		if(copy_len>0)
			memcpy(pstat->wpa_ie, wpa_ie-2, copy_len);

	}


	// check if there is WMM IE & support WWM-PS
	pstat->flags &= ~WLAN_STA_WME;
	pstat->qos_option = 0;
	pstat->qos_info = 0;
	pstat->has_legacy_ac = true;
	pstat->uapsd_vo = 0;
	pstat->uapsd_vi = 0;
	pstat->uapsd_be = 0;
	pstat->uapsd_bk = 0;
	if (pmlmepriv->qospriv.qos_option)
	{
		p = pframe + WLAN_HDR_A3_LEN + ie_offset; ie_len = 0;
		for (;;)
		{
			p = rtw_get_ie(p, _VENDOR_SPECIFIC_IE_, &ie_len, pkt_len - WLAN_HDR_A3_LEN - ie_offset);
			if (p != NULL) {
				if (_rtw_memcmp(p+2, WMM_IE, 6)) {

					pstat->flags |= WLAN_STA_WME;

					pstat->qos_option = 1;
					pstat->qos_info = *(p+8);

					pstat->max_sp_len = (pstat->qos_info>>5)&0x3;

					if((pstat->qos_info&0xf) !=0xf)
						pstat->has_legacy_ac = true;
					else
						pstat->has_legacy_ac = false;

					if(pstat->qos_info&0xf)
					{
						if(pstat->qos_info&BIT(0))
							pstat->uapsd_vo = BIT(0)|BIT(1);
						else
							pstat->uapsd_vo = 0;

						if(pstat->qos_info&BIT(1))
							pstat->uapsd_vi = BIT(0)|BIT(1);
						else
							pstat->uapsd_vi = 0;

						if(pstat->qos_info&BIT(2))
							pstat->uapsd_bk = BIT(0)|BIT(1);
						else
							pstat->uapsd_bk = 0;

						if(pstat->qos_info&BIT(3))
							pstat->uapsd_be = BIT(0)|BIT(1);
						else
							pstat->uapsd_be = 0;

					}

					break;
				}
			}
			else {
				break;
			}
			p = p + ie_len + 2;
		}
	}


	/* save HT capabilities in the sta object */
	memset(&pstat->htpriv.ht_cap, 0, sizeof(struct rtw_ieee80211_ht_cap));
	if (elems.ht_capabilities && elems.ht_capabilities_len >= sizeof(struct rtw_ieee80211_ht_cap))
	{
		pstat->flags |= WLAN_STA_HT;

		pstat->flags |= WLAN_STA_WME;

		memcpy(&pstat->htpriv.ht_cap, elems.ht_capabilities, sizeof(struct rtw_ieee80211_ht_cap));

	} else
		pstat->flags &= ~WLAN_STA_HT;


	if((pmlmepriv->htpriv.ht_option == false) && (pstat->flags&WLAN_STA_HT))
	{
		status = _STATS_FAILURE_;
		goto OnAssocReqFail;
	}


	if ((pstat->flags & WLAN_STA_HT) &&
		    ((pstat->wpa2_pairwise_cipher&WPA_CIPHER_TKIP) ||
		      (pstat->wpa_pairwise_cipher&WPA_CIPHER_TKIP)))
	{
		DBG_871X("HT: " MAC_FMT " tried to "
				   "use TKIP with HT association\n", MAC_ARG(pstat->hwaddr));

		//status = WLAN_STATUS_CIPHER_REJECTED_PER_POLICY;
		//goto OnAssocReqFail;
	}

	memset(&pstat->vhtpriv, 0, sizeof(struct vht_priv));
	if (elems.vht_capabilities && elems.vht_capabilities_len == 12) {
		pstat->flags |= WLAN_STA_VHT;

		memcpy(pstat->vhtpriv.vht_cap, elems.vht_capabilities, 12);

		if (elems.vht_op_mode_notify && elems.vht_op_mode_notify_len == 1) {
			pstat->vhtpriv.vht_bwmode = GET_VHT_OPERATING_MODE_FIELD_CHNL_WIDTH(elems.vht_op_mode_notify);
		}
	}
	else {
		pstat->flags &= ~WLAN_STA_HT;
	}

	if((pmlmepriv->vhtpriv.vht_option == false) && (pstat->flags&WLAN_STA_VHT))
	{
		status = _STATS_FAILURE_;
		goto OnAssocReqFail;
	}

       //
       //if (hapd->iface->current_mode->mode == HOSTAPD_MODE_IEEE80211G)//?
	pstat->flags |= WLAN_STA_NONERP;
	for (i = 0; i < pstat->bssratelen; i++) {
		if ((pstat->bssrateset[i] & 0x7f) > 22) {
			pstat->flags &= ~WLAN_STA_NONERP;
			break;
		}
	}

	if (pstat->capability & WLAN_CAPABILITY_SHORT_PREAMBLE)
		pstat->flags |= WLAN_STA_SHORT_PREAMBLE;
	else
		pstat->flags &= ~WLAN_STA_SHORT_PREAMBLE;



	if (status != _STATS_SUCCESSFUL_)
		goto OnAssocReqFail;

	//TODO: identify_proprietary_vendor_ie();
	// Realtek proprietary IE
	// identify if this is Broadcom sta
	// identify if this is ralink sta
	// Customer proprietary IE



	/* get a unique AID */
	if (pstat->aid > 0) {
		DBG_871X("  old AID %d\n", pstat->aid);
	} else {
		for (pstat->aid = 1; pstat->aid <= NUM_STA; pstat->aid++)
			if (pstapriv->sta_aid[pstat->aid - 1] == NULL)
				break;

		//if (pstat->aid > NUM_STA) {
		if (pstat->aid > pstapriv->max_num_sta) {

			pstat->aid = 0;

			DBG_871X("  no room for more AIDs\n");

			status = WLAN_STATUS_AP_UNABLE_TO_HANDLE_NEW_STA;

			goto OnAssocReqFail;


		} else {
			pstapriv->sta_aid[pstat->aid - 1] = pstat;
			DBG_871X("allocate new AID = (%d)\n", pstat->aid);
		}
	}


	pstat->state &= (~WIFI_FW_ASSOC_STATE);
	pstat->state |= WIFI_FW_ASSOC_SUCCESS;

	spin_lock_bh(&pstapriv->auth_list_lock);
	if (!list_empty(&pstat->auth_list))
	{
		list_del_init(&pstat->auth_list);
		pstapriv->auth_list_cnt--;
	}
	spin_unlock_bh(&pstapriv->auth_list_lock);

	spin_lock_bh(&pstapriv->asoc_list_lock);
	if (list_empty(&pstat->asoc_list))
	{
		pstat->expire_to = pstapriv->expire_to;
		list_add_tail(&pstat->asoc_list, &pstapriv->asoc_list);
		pstapriv->asoc_list_cnt++;
	}
	spin_unlock_bh(&pstapriv->asoc_list_lock);

	// now the station is qualified to join our BSS...
	if(pstat && (pstat->state & WIFI_FW_ASSOC_SUCCESS) && (_STATS_SUCCESSFUL_==status))
	{
		//.1 bss_cap_update & sta_info_update
		bss_cap_update_on_sta_join(rtlpriv, pstat);
		sta_info_update(rtlpriv, pstat);

		//issue assoc rsp before notify station join event.
		if (frame_type == WIFI_ASSOCREQ)
			issue_asocrsp(rtlpriv, status, pstat, WIFI_ASSOCRSP);
		else
			issue_asocrsp(rtlpriv, status, pstat, WIFI_REASSOCRSP);

		//.2 - report to upper layer
		DBG_871X("indicate_sta_join_event to upper layer - hostapd\n");
		rtw_indicate_sta_assoc_event(rtlpriv, pstat);

		//.3-(1) report sta add event
		report_add_sta_event(rtlpriv, pstat->hwaddr, pstat->aid);

/*
		//issue assoc rsp before notify station join event.
		if (frame_type == WIFI_ASSOCREQ)
			issue_asocrsp(rtlpriv, status, pstat, WIFI_ASSOCRSP);
		else
			issue_asocrsp(rtlpriv, status, pstat, WIFI_REASSOCRSP);
*/

	}

	return _SUCCESS;

asoc_class2_error:

	issue_deauth(rtlpriv, (void *)GetAddr2Ptr(pframe), status);

	return _FAIL;

OnAssocReqFail:


	pstat->aid = 0;
	if (frame_type == WIFI_ASSOCREQ)
		issue_asocrsp(rtlpriv, status, pstat, WIFI_ASSOCRSP);
	else
		issue_asocrsp(rtlpriv, status, pstat, WIFI_REASSOCRSP);


#endif /* CONFIG_AP_MODE */

	return _FAIL;

}

unsigned int OnAssocRsp(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	uint i;
	int res;
	unsigned short	status;
	PNDIS_802_11_VARIABLE_IEs	pIE;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	//WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);
	uint8_t *pframe = precv_frame->rx_data;
	uint pkt_len = precv_frame->len;
	PNDIS_802_11_VARIABLE_IEs	pWapiIE = NULL;

	DBG_871X("%s\n", __FUNCTION__);

	//check A1 matches or not
	if (!_rtw_memcmp(rtlpriv->mac80211.mac_addr, get_da(pframe), ETH_ALEN))
		return _SUCCESS;

	if (!(pmlmeinfo->state & (WIFI_FW_AUTH_SUCCESS | WIFI_FW_ASSOC_STATE)))
		return _SUCCESS;

	if (pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS)
		return _SUCCESS;

	del_timer_sync_ex(&pmlmeext->link_timer);

	//status
	if ((status = le16_to_cpu(*(unsigned short *)(pframe + WLAN_HDR_A3_LEN + 2))) > 0)
	{
		DBG_871X("assoc reject, status code: %d\n", status);
		pmlmeinfo->state = WIFI_FW_NULL_STATE;
		res = -4;
		goto report_assoc_result;
	}

	//get capabilities
	pmlmeinfo->capability = le16_to_cpu(*(unsigned short *)(pframe + WLAN_HDR_A3_LEN));

	//set slot time
	pmlmeinfo->slotTime = (pmlmeinfo->capability & BIT(10))? 9: 20;

	//AID
	res = pmlmeinfo->aid = (int)(le16_to_cpu(*(unsigned short *)(pframe + WLAN_HDR_A3_LEN + 4))&0x3fff);

	//following are moved to join event callback function
	//to handle HT, WMM, rate adaptive, update MAC reg
	//for not to handle the synchronous IO in the tasklet
	for (i = (6 + WLAN_HDR_A3_LEN); i < pkt_len;)
	{
		pIE = (PNDIS_802_11_VARIABLE_IEs)(pframe + i);

		switch (pIE->ElementID)
		{
			case _VENDOR_SPECIFIC_IE_:
				if (_rtw_memcmp(pIE->data, WMM_PARA_OUI, 6))	//WMM
				{
					WMM_param_handler(rtlpriv, pIE);
				}
				break;

			case _HT_CAPABILITY_IE_:	//HT caps
				HT_caps_handler(rtlpriv, pIE);
				break;

			case _HT_EXTRA_INFO_IE_:	//HT info
				HT_info_handler(rtlpriv, pIE);
				break;

			case EID_VHTCapability:
				VHT_caps_handler(rtlpriv, pIE);
				break;

			case EID_VHTOperation:
				VHT_operation_handler(rtlpriv, pIE);
				break;

			case _ERPINFO_IE_:
				ERP_IE_handler(rtlpriv, pIE);

			default:
				break;
		}

		i += (pIE->Length + 2);
	}

	pmlmeinfo->state &= (~WIFI_FW_ASSOC_STATE);
	pmlmeinfo->state |= WIFI_FW_ASSOC_SUCCESS;

	//Update Basic Rate Table for spec, 2010-12-28 , by thomas
	UpdateBrateTbl(rtlpriv, pmlmeinfo->network.SupportedRates);

report_assoc_result:
	if (res > 0) {
		rtw_buf_update(&pmlmepriv->assoc_rsp, &pmlmepriv->assoc_rsp_len, pframe, pkt_len);
	} else {
		rtw_buf_free(&pmlmepriv->assoc_rsp, &pmlmepriv->assoc_rsp_len);
	}

	report_join_res(rtlpriv, res);

	return _SUCCESS;
}

unsigned int OnDeAuth(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	unsigned short	reason;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t *pframe = precv_frame->rx_data;

	//check A3
	if (!(_rtw_memcmp(GetAddr3Ptr(pframe), get_my_bssid(&pmlmeinfo->network), ETH_ALEN)))
		return _SUCCESS;


	reason = le16_to_cpu(*(unsigned short *)(pframe + WLAN_HDR_A3_LEN));

	DBG_871X("%s Reason code(%d)\n", __FUNCTION__,reason);

#ifdef CONFIG_AP_MODE
	if(check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)
	{
		struct sta_info *psta;
		struct sta_priv *pstapriv = &rtlpriv->stapriv;

		//spin_lock_bh(&(pstapriv->sta_hash_lock), &irqL);
		//rtw_free_stainfo(rtlpriv, psta);
		//spin_unlock_bh(&(pstapriv->sta_hash_lock), &irqL);

		DBG_871X_LEVEL(_drv_always_, "ap recv deauth reason code(%d) sta:%pM\n",
			       	reason, GetAddr2Ptr(pframe));

		psta = rtw_get_stainfo(pstapriv, GetAddr2Ptr(pframe));
		if(psta)
		{
			uint8_t updated;

			spin_lock_bh(&pstapriv->asoc_list_lock);
			if(list_empty(&psta->asoc_list)==false)
			{
				list_del_init(&psta->asoc_list);
				pstapriv->asoc_list_cnt--;
				updated = ap_free_sta(rtlpriv, psta, false, reason);

			}
			spin_unlock_bh(&pstapriv->asoc_list_lock);

			associated_clients_update(rtlpriv, updated);
		}


		return _SUCCESS;
	}
	else
#endif
	{
		DBG_871X_LEVEL(_drv_always_, "sta recv deauth reason code(%d) sta:%pM\n",
			       	reason, GetAddr3Ptr(pframe));

		receive_disconnect(rtlpriv, GetAddr3Ptr(pframe) ,reason);
	}
	pmlmepriv->LinkDetectInfo.bBusyTraffic = false;
	return _SUCCESS;

}

unsigned int OnDisassoc(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	unsigned short	reason;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t *pframe = precv_frame->rx_data;

	//check A3
	if (!(_rtw_memcmp(GetAddr3Ptr(pframe), get_my_bssid(&pmlmeinfo->network), ETH_ALEN)))
		return _SUCCESS;


	reason = le16_to_cpu(*(unsigned short *)(pframe + WLAN_HDR_A3_LEN));

        DBG_871X("%s Reason code(%d)\n", __FUNCTION__,reason);

#ifdef CONFIG_AP_MODE
	if(check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)
	{
		struct sta_info *psta;
		struct sta_priv *pstapriv = &rtlpriv->stapriv;

		//spin_lock_bh(&(pstapriv->sta_hash_lock), &irqL);
		//rtw_free_stainfo(rtlpriv, psta);
		//spin_unlock_bh(&(pstapriv->sta_hash_lock), &irqL);

		DBG_871X_LEVEL(_drv_always_, "ap recv disassoc reason code(%d) sta:%pM\n",
				reason, GetAddr2Ptr(pframe));

		psta = rtw_get_stainfo(pstapriv, GetAddr2Ptr(pframe));
		if(psta)
		{
			uint8_t updated;

			spin_lock_bh(&pstapriv->asoc_list_lock);
			if(list_empty(&psta->asoc_list)==false)
			{
				list_del_init(&psta->asoc_list);
				pstapriv->asoc_list_cnt--;
				updated = ap_free_sta(rtlpriv, psta, false, reason);

			}
			spin_unlock_bh(&pstapriv->asoc_list_lock);

			associated_clients_update(rtlpriv, updated);
		}

		return _SUCCESS;
	}
	else
#endif
	{
		DBG_871X_LEVEL(_drv_always_, "ap recv disassoc reason code(%d) sta:%pM\n",
				reason, GetAddr3Ptr(pframe));

		receive_disconnect(rtlpriv, GetAddr3Ptr(pframe), reason);
	}
	pmlmepriv->LinkDetectInfo.bBusyTraffic = false;
	return _SUCCESS;

}

unsigned int OnAtim(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	DBG_871X("%s\n", __FUNCTION__);
	return _SUCCESS;
}

unsigned int on_action_spct_ch_switch(struct rtl_priv *rtlpriv, struct sta_info *psta, uint8_t *ies, uint ies_len)
{
	unsigned int ret = _FAIL;
	struct mlme_ext_priv *mlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(mlmeext->mlmext_info);

	if (!(pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS)) {
		ret = _SUCCESS;
		goto exit;
	}

	if ((pmlmeinfo->state & 0x03) == WIFI_FW_STATION_STATE) {

		int ch_switch_mode = -1, ch = -1, ch_switch_cnt = -1;
		int ch_offset = -1;
		uint8_t bwmode;
		struct ieee80211_info_element *ie;

		DBG_871X(FUNC_NDEV_FMT" from "MAC_FMT"\n",
			FUNC_NDEV_ARG(rtlpriv->ndev), MAC_ARG(psta->hwaddr));

		for_each_ie(ie, ies, ies_len) {
			if (ie->id == WLAN_EID_CHANNEL_SWITCH) {
				ch_switch_mode = ie->data[0];
				ch = ie->data[1];
				ch_switch_cnt = ie->data[2];
				DBG_871X("ch_switch_mode:%d, ch:%d, ch_switch_cnt:%d\n",
					ch_switch_mode, ch, ch_switch_cnt);
			}
			else if (ie->id == WLAN_EID_SECONDARY_CHANNEL_OFFSET) {
				ch_offset = secondary_ch_offset_to_hal_ch_offset(ie->data[0]);
				DBG_871X("ch_offset:%d\n", ch_offset);
			}
		}

		if (ch == -1)
			return _SUCCESS;

		if (ch_offset == -1)
			bwmode = mlmeext->cur_bwmode;
		else
			bwmode = (ch_offset == HAL_PRIME_CHNL_OFFSET_DONT_CARE) ?
				CHANNEL_WIDTH_20 : CHANNEL_WIDTH_40;

		ch_offset = (ch_offset == -1) ? mlmeext->cur_ch_offset : ch_offset;

		/* todo:
		 * 1. the decision of channel switching
		 * 2. things after channel switching
		 */

		ret = rtw_set_ch_cmd(rtlpriv, ch, bwmode, ch_offset, true);
	}

exit:
	return ret;
}

unsigned int on_action_spct(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	unsigned int ret = _FAIL;
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	uint8_t *pframe = precv_frame->rx_data;
	uint frame_len = precv_frame->len;
	uint8_t *frame_body = (uint8_t *)(pframe + sizeof(struct rtw_ieee80211_hdr_3addr));
	uint8_t category;
	uint8_t action;

	DBG_871X(FUNC_NDEV_FMT"\n", FUNC_NDEV_ARG(rtlpriv->ndev));

	psta = rtw_get_stainfo(pstapriv, GetAddr2Ptr(pframe));

	if (!psta)
		goto exit;

	category = frame_body[0];
	if(category != RTW_WLAN_CATEGORY_SPECTRUM_MGMT)
		goto exit;

	action = frame_body[1];
	switch (action) {
	case RTW_WLAN_ACTION_SPCT_MSR_REQ:
	case RTW_WLAN_ACTION_SPCT_MSR_RPRT:
	case RTW_WLAN_ACTION_SPCT_TPC_REQ:
	case RTW_WLAN_ACTION_SPCT_TPC_RPRT:
		break;
	case RTW_WLAN_ACTION_SPCT_CHL_SWITCH:
		#ifdef CONFIG_SPCT_CH_SWITCH
		ret = on_action_spct_ch_switch(rtlpriv, psta, &frame_body[2],
			frame_len-(frame_body-pframe)-2);
		#endif
		break;
	default:
		break;
	}

exit:
	return ret;
}

unsigned int OnAction_qos(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	return _SUCCESS;
}

unsigned int OnAction_dls(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	return _SUCCESS;
}

unsigned int OnAction_back(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	uint8_t *addr;
	struct sta_info *psta=NULL;
	struct recv_reorder_ctrl *preorder_ctrl;
	unsigned char		*frame_body;
	unsigned char		category, action;
	unsigned short	tid, status, reason_code = 0;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t *pframe = precv_frame->rx_data;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	//check RA matches or not
	if (!_rtw_memcmp(rtlpriv->mac80211.mac_addr, GetAddr1Ptr(pframe), ETH_ALEN))//for if1, sta/ap mode
		return _SUCCESS;

/*
	//check A1 matches or not
	if (!_rtw_memcmp(myid(&(rtlpriv->eeprompriv)), get_da(pframe), ETH_ALEN))
		return _SUCCESS;
*/
	DBG_871X("%s\n", __FUNCTION__);

	if((pmlmeinfo->state&0x03) != WIFI_FW_AP_STATE)
		if (!(pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS))
			return _SUCCESS;

	addr = GetAddr2Ptr(pframe);
	psta = rtw_get_stainfo(pstapriv, addr);

	if(psta==NULL)
		return _SUCCESS;

	frame_body = (unsigned char *)(pframe + sizeof(struct rtw_ieee80211_hdr_3addr));

	category = frame_body[0];
	if (category == RTW_WLAN_CATEGORY_BACK)// representing Block Ack
	{
		if (!pmlmeinfo->HT_enable)
		{
			return _SUCCESS;
		}

		action = frame_body[1];
		DBG_871X("%s, action=%d\n", __FUNCTION__, action);
		switch (action)
		{
			case RTW_WLAN_ACTION_ADDBA_REQ: //ADDBA request

				memcpy(&(pmlmeinfo->ADDBA_req), &(frame_body[2]), sizeof(struct ADDBA_request));
				//process_addba_req(rtlpriv, (uint8_t *)&(pmlmeinfo->ADDBA_req), GetAddr3Ptr(pframe));
				process_addba_req(rtlpriv, (uint8_t *)&(pmlmeinfo->ADDBA_req), addr);

				if(pmlmeinfo->bAcceptAddbaReq == true)
				{
					issue_action_BA(rtlpriv, addr, RTW_WLAN_ACTION_ADDBA_RESP, 0);
				}
				else
				{
					issue_action_BA(rtlpriv, addr, RTW_WLAN_ACTION_ADDBA_RESP, 37);//reject ADDBA Req
				}

				break;

			case RTW_WLAN_ACTION_ADDBA_RESP: //ADDBA response

				//status = frame_body[3] | (frame_body[4] << 8); //endian issue
				status = RTW_GET_LE16(&frame_body[3]);
				tid = ((frame_body[5] >> 2) & 0x7);

				if (status == 0)
				{	//successful
					DBG_871X("agg_enable for TID=%d\n", tid);
					psta->htpriv.agg_enable_bitmap |= 1 << tid;
					psta->htpriv.candidate_tid_bitmap &= ~BIT(tid);
				}
				else
				{
					psta->htpriv.agg_enable_bitmap &= ~BIT(tid);
				}

				//DBG_871X("marc: ADDBA RSP: %x\n", pmlmeinfo->agg_enable_bitmap);
				break;

			case RTW_WLAN_ACTION_DELBA: //DELBA
				if ((frame_body[3] & BIT(3)) == 0)
				{
					psta->htpriv.agg_enable_bitmap &= ~(1 << ((frame_body[3] >> 4) & 0xf));
					psta->htpriv.candidate_tid_bitmap &= ~(1 << ((frame_body[3] >> 4) & 0xf));

					//reason_code = frame_body[4] | (frame_body[5] << 8);
					reason_code = RTW_GET_LE16(&frame_body[4]);
				}
				else if((frame_body[3] & BIT(3)) == BIT(3))
				{
					tid = (frame_body[3] >> 4) & 0x0F;

					preorder_ctrl =  &psta->recvreorder_ctrl[tid];
					preorder_ctrl->enable = false;
					preorder_ctrl->indicate_seq = 0xffff;
				}

				DBG_871X("%s(): DELBA: %x(%x)\n", __FUNCTION__,pmlmeinfo->agg_enable_bitmap, reason_code);
				//todo: how to notify the host while receiving DELETE BA
				break;

			default:
				break;
		}
	}
	return _SUCCESS;
}

int32_t rtw_action_public_decache(struct recv_frame *recv_frame, int32_t token)
{
	struct rtl_priv *rtlpriv = recv_frame->rtlpriv;
	struct mlme_ext_priv *mlmeext = &(rtlpriv->mlmeextpriv);
	uint8_t *frame = recv_frame->rx_data;
	u16 seq_ctrl = ( (recv_frame->attrib.seq_num&0xffff) << 4) |
		(recv_frame->attrib.frag_num & 0xf);

	if (GetRetry(frame)) {
		if (token >= 0) {
			if ((seq_ctrl == mlmeext->action_public_rxseq)
				&& (token == mlmeext->action_public_dialog_token))
			{
				DBG_871X(FUNC_ADPT_FMT" seq_ctrl=0x%x, rxseq=0x%x, token:%d\n",
					FUNC_ADPT_ARG(rtlpriv), seq_ctrl, mlmeext->action_public_rxseq, token);
				return _FAIL;
			}
		} else {
			if (seq_ctrl == mlmeext->action_public_rxseq) {
				DBG_871X(FUNC_ADPT_FMT" seq_ctrl=0x%x, rxseq=0x%x\n",
					FUNC_ADPT_ARG(rtlpriv), seq_ctrl, mlmeext->action_public_rxseq);
				return _FAIL;
			}
		}
	}

	mlmeext->action_public_rxseq = seq_ctrl;

	if (token >= 0)
		mlmeext->action_public_dialog_token = token;

	return _SUCCESS;
}

unsigned int on_action_public_p2p(struct recv_frame *precv_frame)
{
	struct rtl_priv *rtlpriv = precv_frame->rtlpriv;
	uint8_t *pframe = precv_frame->rx_data;
	uint len = precv_frame->len;
	uint8_t *frame_body;
	uint8_t dialogToken=0;

	frame_body = (unsigned char *)(pframe + sizeof(struct rtw_ieee80211_hdr_3addr));

	dialogToken = frame_body[7];

	if (rtw_action_public_decache(precv_frame, dialogToken) == _FAIL)
		return _FAIL;


	return _SUCCESS;
}

unsigned int on_action_public_vendor(struct recv_frame *precv_frame)
{
	unsigned int ret = _FAIL;
	uint8_t *pframe = precv_frame->rx_data;
	uint frame_len = precv_frame->len;
	uint8_t *frame_body = pframe + sizeof(struct rtw_ieee80211_hdr_3addr);

	if (_rtw_memcmp(frame_body + 2, P2P_OUI, 4) == true) {
		ret = on_action_public_p2p(precv_frame);
	}

	return ret;
}

unsigned int on_action_public_default(struct recv_frame *precv_frame, uint8_t action)
{
	unsigned int ret = _FAIL;
	uint8_t *pframe = precv_frame->rx_data;
	uint frame_len = precv_frame->len;
	uint8_t *frame_body = pframe + sizeof(struct rtw_ieee80211_hdr_3addr);
	uint8_t token;
	struct rtl_priv *rtlpriv = precv_frame->rtlpriv;
	int cnt = 0;
	char msg[64];

	token = frame_body[2];

	if (rtw_action_public_decache(precv_frame, token) == _FAIL)
		goto exit;

	ret = _SUCCESS;

exit:
	return ret;
}

unsigned int on_action_public(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	unsigned int ret = _FAIL;
	uint8_t *pframe = precv_frame->rx_data;
	uint frame_len = precv_frame->len;
	uint8_t *frame_body = pframe + sizeof(struct rtw_ieee80211_hdr_3addr);
	uint8_t category, action;

	/* check RA matches or not */
	if (!_rtw_memcmp(rtlpriv->mac80211.mac_addr, GetAddr1Ptr(pframe), ETH_ALEN))
		goto exit;

	category = frame_body[0];
	if(category != RTW_WLAN_CATEGORY_PUBLIC)
		goto exit;

	action = frame_body[1];
	switch (action) {
	case ACT_PUBLIC_VENDOR:
		ret = on_action_public_vendor(precv_frame);
		break;
	default:
		ret = on_action_public_default(precv_frame, action);
		break;
	}

exit:
	return ret;
}

unsigned int OnAction_ht(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	return _SUCCESS;
}

unsigned int OnAction_wmm(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	return _SUCCESS;
}

unsigned int OnAction_p2p(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{

	return _SUCCESS;

}

unsigned int OnAction(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	int i;
	unsigned char	category;
	struct action_handler *ptable;
	unsigned char	*frame_body;
	uint8_t *pframe = precv_frame->rx_data;

	frame_body = (unsigned char *)(pframe + sizeof(struct rtw_ieee80211_hdr_3addr));

	category = frame_body[0];

	for(i = 0; i < sizeof(OnAction_tbl)/sizeof(struct action_handler); i++)
	{
		ptable = &OnAction_tbl[i];

		if(category == ptable->num)
			ptable->func(rtlpriv, precv_frame);

	}

	return _SUCCESS;

}

unsigned int DoReserved(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{

	//DBG_871X("rcvd mgt frame(%x, %x)\n", (GetFrameSubType(pframe) >> 4), *(unsigned int *)GetAddr1Ptr(pframe));
	return _SUCCESS;
}

struct xmit_frame *_alloc_mgtxmitframe(struct xmit_priv *pxmitpriv, bool once)
{
	struct xmit_frame *pmgntframe;
	struct xmit_buf *pxmitbuf;

	if (once)
		pmgntframe = rtw_alloc_xmitframe_once(pxmitpriv);
	else
		pmgntframe = rtw_alloc_xmitframe_ext(pxmitpriv);

	if (pmgntframe == NULL) {
		DBG_871X(FUNC_ADPT_FMT" alloc xmitframe fail, once:%d\n", FUNC_ADPT_ARG(pxmitpriv->rtlpriv), once);
		goto exit;
	}

	if ((pxmitbuf = rtw_alloc_xmitbuf_ext(pxmitpriv)) == NULL) {
		DBG_871X(FUNC_ADPT_FMT" alloc xmitbuf fail\n", FUNC_ADPT_ARG(pxmitpriv->rtlpriv));
		rtw_free_xmitframe(pxmitpriv, pmgntframe);
		pmgntframe = NULL;
		goto exit;
	}

	pmgntframe->frame_tag = MGNT_FRAMETAG;
	pmgntframe->pxmitbuf = pxmitbuf;
	pmgntframe->buf_addr = pxmitbuf->pbuf;
	pxmitbuf->priv_data = pmgntframe;

exit:
	return pmgntframe;

}

inline struct xmit_frame *alloc_mgtxmitframe(struct xmit_priv *pxmitpriv)
{
	return _alloc_mgtxmitframe(pxmitpriv, false);
}

inline struct xmit_frame *alloc_mgtxmitframe_once(struct xmit_priv *pxmitpriv)
{
	return _alloc_mgtxmitframe(pxmitpriv, true);
}


/****************************************************************************

Following are some TX fuctions for WiFi MLME

*****************************************************************************/

void update_mgnt_tx_rate(struct rtl_priv *rtlpriv, uint8_t rate)
{
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);

	pmlmeext->tx_rate = rate;
	//DBG_871X("%s(): rate = %x\n",__FUNCTION__, rate);
}

void update_mgntframe_attrib(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib)
{
	uint8_t	wireless_mode;
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);

	//memset((uint8_t *)(pattrib), 0, sizeof(struct pkt_attrib));

	pattrib->hdrlen = 24;
	pattrib->nr_frags = 1;
	pattrib->tx_priority = 7;
	pattrib->mac_id = 0;
	pattrib->tx_qsel = 0x12;

	pattrib->pktlen = 0;

	if (pmlmeext->tx_rate == IEEE80211_CCK_RATE_1MB)
		wireless_mode = WIRELESS_11B;
	else
		wireless_mode = WIRELESS_11G;
	pattrib->raid =  rtw_get_mgntframe_raid(rtlpriv, wireless_mode);

	pattrib->encrypt = NO_ENCRYPTION;
	pattrib->bswenc = false;

	pattrib->qos_en = false;
	pattrib->ht_en = false;
	pattrib->bwmode = CHANNEL_WIDTH_20;
	pattrib->ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
	pattrib->sgi = false;

	pattrib->seqnum = pmlmeext->mgnt_seq;

	pattrib->retry_ctrl = true;

	pattrib->mbssid = 0;

}

void update_mgntframe_attrib_addr(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe)
{
	uint8_t	*pframe;
	struct tx_pkt_attrib	*pattrib = &pmgntframe->tx_attrib;

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;

	memcpy(pattrib->ra, GetAddr1Ptr(pframe), ETH_ALEN);
	memcpy(pattrib->ta, GetAddr2Ptr(pframe), ETH_ALEN);
}

void dump_mgntframe(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe)
{
	if(rtlpriv->bSurpriseRemoved == true ||
		rtlpriv->bDriverStopped == true)
		return;

	rtw_hal_mgnt_xmit(rtlpriv, pmgntframe);
}

int32_t dump_mgntframe_and_wait(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe, int timeout_ms)
{
	int32_t ret = _FAIL;
	unsigned long flags;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct xmit_buf *pxmitbuf = pmgntframe->pxmitbuf;
	struct submit_ctx sctx;

	if(rtlpriv->bSurpriseRemoved == true ||
		rtlpriv->bDriverStopped == true)
		return ret;

	rtw_sctx_init(&sctx, timeout_ms);
	pxmitbuf->sctx = &sctx;

	ret = rtw_hal_mgnt_xmit(rtlpriv, pmgntframe);

	if (ret == _SUCCESS)
		ret = rtw_sctx_wait(&sctx);

	spin_lock_irqsave(&pxmitpriv->lock_sctx, flags);
	pxmitbuf->sctx = NULL;
	spin_unlock_irqrestore(&pxmitpriv->lock_sctx, flags);

	 return ret;
}

int32_t dump_mgntframe_and_wait_ack(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe)
{
	dump_mgntframe(rtlpriv, pmgntframe);
	msleep(50);
	return _SUCCESS;
}

int update_hidden_ssid(uint8_t *ies, uint32_t	 ies_len, uint8_t hidden_ssid_mode)
{
	uint8_t *ssid_ie;
	int ssid_len_ori;
	int len_diff = 0;

	ssid_ie = rtw_get_ie(ies,  WLAN_EID_SSID, &ssid_len_ori, ies_len);

	//DBG_871X("%s hidden_ssid_mode:%u, ssid_ie:%p, ssid_len_ori:%d\n", __FUNCTION__, hidden_ssid_mode, ssid_ie, ssid_len_ori);

	if(ssid_ie && ssid_len_ori>0)
	{
		switch(hidden_ssid_mode)
		{
			case 1:
			{
				uint8_t *next_ie = ssid_ie + 2 + ssid_len_ori;
				uint32_t	 remain_len = 0;

				remain_len = ies_len -(next_ie-ies);

				ssid_ie[1] = 0;
				memcpy(ssid_ie+2, next_ie, remain_len);
				len_diff -= ssid_len_ori;

				break;
			}
			case 2:
				memset(&ssid_ie[2], 0, ssid_len_ori);
				break;
			default:
				break;
		}
	}

	return len_diff;
}

void issue_beacon(struct rtl_priv *rtlpriv, int timeout_ms)
{
	struct xmit_frame	*pmgntframe;
	struct tx_pkt_attrib	*pattrib;
	unsigned char	*pframe;
	struct rtw_ieee80211_hdr *pwlanhdr;
	unsigned short *fctrl;
	unsigned int	rate_len;
	struct xmit_priv	*pxmitpriv = &(rtlpriv->xmitpriv);
#if defined (CONFIG_AP_MODE)
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
#endif //#if defined (CONFIG_AP_MODE)
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);
	uint8_t	bc_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};


	//DBG_871X("%s\n", __FUNCTION__);

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		DBG_871X("%s, alloc mgnt frame fail\n", __FUNCTION__);
		return;
	}
#if defined (CONFIG_AP_MODE)
	spin_lock_bh(&pmlmepriv->bcn_update_lock);
#endif //#if defined (CONFIG_AP_MODE)

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);
	pattrib->tx_qsel = 0x10;

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;


	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	memcpy(pwlanhdr->addr1, bc_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr3, get_my_bssid(cur_network), ETH_ALEN);

	SetSeqNum(pwlanhdr, 0/*pmlmeext->mgnt_seq*/);
	//pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_BEACON);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof (struct rtw_ieee80211_hdr_3addr);

	if( (pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
	{
		//DBG_871X("ie len=%d\n", cur_network->IELength);
		{
			int len_diff;
			memcpy(pframe, cur_network->IEs, cur_network->IELength);
			len_diff = update_hidden_ssid(
				pframe+_BEACON_IE_OFFSET_
				, cur_network->IELength-_BEACON_IE_OFFSET_
				, pmlmeinfo->hidden_ssid_mode
			);
			pframe += (cur_network->IELength+len_diff);
			pattrib->pktlen += (cur_network->IELength+len_diff);
		}

		{
			uint8_t *wps_ie;
			uint wps_ielen;
			uint8_t sr = 0;
			wps_ie = rtw_get_wps_ie(pmgntframe->buf_addr+TXDESC_OFFSET+sizeof (struct rtw_ieee80211_hdr_3addr)+_BEACON_IE_OFFSET_,
				pattrib->pktlen-sizeof (struct rtw_ieee80211_hdr_3addr)-_BEACON_IE_OFFSET_, NULL, &wps_ielen);
			if (wps_ie && wps_ielen>0) {
				rtw_get_wps_attr_content(wps_ie,  wps_ielen, WPS_ATTR_SELECTED_REGISTRAR, (uint8_t *)(&sr), NULL);
			}
			if (sr != 0)
				set_fwstate(pmlmepriv, WIFI_UNDER_WPS);
			else
				_clr_fwstate_(pmlmepriv, WIFI_UNDER_WPS);
		}


		goto _issue_bcn;

	}

	//below for ad-hoc mode

	//timestamp will be inserted by hardware
	pframe += 8;
	pattrib->pktlen += 8;

	// beacon interval: 2 bytes

	memcpy(pframe, (unsigned char *)(rtw_get_beacon_interval_from_ie(cur_network->IEs)), 2);

	pframe += 2;
	pattrib->pktlen += 2;

	// capability info: 2 bytes

	memcpy(pframe, (unsigned char *)(rtw_get_capability_from_ie(cur_network->IEs)), 2);

	pframe += 2;
	pattrib->pktlen += 2;

	// SSID
	pframe = rtw_set_ie(pframe, _SSID_IE_, cur_network->Ssid.SsidLength, cur_network->Ssid.Ssid, &pattrib->pktlen);

	// supported rates...
	rate_len = rtw_get_rateset_len(cur_network->SupportedRates);
	pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_, ((rate_len > 8)? 8: rate_len), cur_network->SupportedRates, &pattrib->pktlen);

	// DS parameter set
	pframe = rtw_set_ie(pframe, _DSSET_IE_, 1, (unsigned char *)&(cur_network->Configuration.DSConfig), &pattrib->pktlen);

	//if( (pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)
	{
		uint8_t erpinfo=0;
		uint32_t	 ATIMWindow;
		// IBSS Parameter Set...
		//ATIMWindow = cur->Configuration.ATIMWindow;
		ATIMWindow = 0;
		pframe = rtw_set_ie(pframe, _IBSS_PARA_IE_, 2, (unsigned char *)(&ATIMWindow), &pattrib->pktlen);

		//ERP IE
		pframe = rtw_set_ie(pframe, _ERPINFO_IE_, 1, &erpinfo, &pattrib->pktlen);
	}


	// EXTERNDED SUPPORTED RATE
	if (rate_len > 8)
	{
		pframe = rtw_set_ie(pframe, _EXT_SUPPORTEDRATES_IE_, (rate_len - 8), (cur_network->SupportedRates + 8), &pattrib->pktlen);
	}


	//todo:HT for adhoc

_issue_bcn:

#if defined (CONFIG_AP_MODE)
	pmlmepriv->update_bcn = false;

	spin_unlock_bh(&pmlmepriv->bcn_update_lock);
#endif //#if defined (CONFIG_AP_MODE)

	if ((pattrib->pktlen + TXDESC_SIZE) > 512)
	{
		DBG_871X("beacon frame too large\n");
		return;
	}

	pattrib->last_txcmdsz = pattrib->pktlen;

	//DBG_871X("issue bcn_sz=%d\n", pattrib->last_txcmdsz);
	if(timeout_ms > 0)
		dump_mgntframe_and_wait(rtlpriv, pmgntframe, timeout_ms);
	else
		dump_mgntframe(rtlpriv, pmgntframe);

}

void issue_probersp(struct rtl_priv *rtlpriv, unsigned char *da, uint8_t is_valid_p2p_probereq)
{
	struct xmit_frame			*pmgntframe;
	struct tx_pkt_attrib			*pattrib;
	unsigned char					*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short				*fctrl;
	unsigned char					*mac, *bssid;
	struct xmit_priv	*pxmitpriv = &(rtlpriv->xmitpriv);
#if defined (CONFIG_AP_MODE)
	uint8_t *pwps_ie;
	uint wps_ielen;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
#endif //#if defined (CONFIG_AP_MODE)
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);
	unsigned int	rate_len;

	//DBG_871X("%s\n", __FUNCTION__);

	if(da == NULL)
		return;

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		DBG_871X("%s, alloc mgnt frame fail\n", __FUNCTION__);
		return;
	}


	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	mac = rtlpriv->mac80211.mac_addr;
	bssid = cur_network->MacAddress;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;
	memcpy(pwlanhdr->addr1, da, ETH_ALEN);
	memcpy(pwlanhdr->addr2, mac, ETH_ALEN);
	memcpy(pwlanhdr->addr3, bssid, ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(fctrl, WIFI_PROBERSP);

	pattrib->hdrlen = sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = pattrib->hdrlen;
	pframe += pattrib->hdrlen;


	if(cur_network->IELength>MAX_IE_SZ)
		return;

#if defined (CONFIG_AP_MODE)
	if( (pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
	{
		pwps_ie = rtw_get_wps_ie(cur_network->IEs+_FIXED_IE_LENGTH_, cur_network->IELength-_FIXED_IE_LENGTH_, NULL, &wps_ielen);

		//inerset & update wps_probe_resp_ie
		if((pmlmepriv->wps_probe_resp_ie!=NULL) && pwps_ie && (wps_ielen>0))
		{
			uint wps_offset, remainder_ielen;
			uint8_t *premainder_ie;

			wps_offset = (uint)(pwps_ie - cur_network->IEs);

			premainder_ie = pwps_ie + wps_ielen;

			remainder_ielen = cur_network->IELength - wps_offset - wps_ielen;

			memcpy(pframe, cur_network->IEs, wps_offset);
			pframe += wps_offset;
			pattrib->pktlen += wps_offset;

			wps_ielen = (uint)pmlmepriv->wps_probe_resp_ie[1];//to get ie data len
			if((wps_offset+wps_ielen+2)<=MAX_IE_SZ)
			{
				memcpy(pframe, pmlmepriv->wps_probe_resp_ie, wps_ielen+2);
				pframe += wps_ielen+2;
				pattrib->pktlen += wps_ielen+2;
			}

			if((wps_offset+wps_ielen+2+remainder_ielen)<=MAX_IE_SZ)
			{
				memcpy(pframe, premainder_ie, remainder_ielen);
				pframe += remainder_ielen;
				pattrib->pktlen += remainder_ielen;
			}
		}
		else
		{
			memcpy(pframe, cur_network->IEs, cur_network->IELength);
			pframe += cur_network->IELength;
			pattrib->pktlen += cur_network->IELength;
		}

		/* retrieve SSID IE from cur_network->Ssid */
		{
			uint8_t *ssid_ie;
			int ssid_ielen;
			int ssid_ielen_diff;
			uint8_t buf[MAX_IE_SZ];
			uint8_t *ies = pmgntframe->buf_addr+TXDESC_OFFSET+sizeof(struct rtw_ieee80211_hdr_3addr);

			ssid_ie = rtw_get_ie(ies+_FIXED_IE_LENGTH_, _SSID_IE_, &ssid_ielen,
				(pframe-ies)-_FIXED_IE_LENGTH_);

			ssid_ielen_diff = cur_network->Ssid.SsidLength - ssid_ielen;

			if (ssid_ie &&  cur_network->Ssid.SsidLength) {
				uint remainder_ielen;
				uint8_t *remainder_ie;
				remainder_ie = ssid_ie+2;
				remainder_ielen = (pframe-remainder_ie);

				DBG_871X_LEVEL(_drv_warning_, FUNC_ADPT_FMT" remainder_ielen > MAX_IE_SZ\n", FUNC_ADPT_ARG(rtlpriv));
				if (remainder_ielen > MAX_IE_SZ) {
					remainder_ielen = MAX_IE_SZ;
				}

				memcpy(buf, remainder_ie, remainder_ielen);
				memcpy(remainder_ie+ssid_ielen_diff, buf, remainder_ielen);
				*(ssid_ie+1) = cur_network->Ssid.SsidLength;
				memcpy(ssid_ie+2, cur_network->Ssid.Ssid, cur_network->Ssid.SsidLength);

				pframe += ssid_ielen_diff;
				pattrib->pktlen += ssid_ielen_diff;
			}
		}
	}
	else
#endif
	{

		//timestamp will be inserted by hardware
		pframe += 8;
		pattrib->pktlen += 8;

		// beacon interval: 2 bytes

		memcpy(pframe, (unsigned char *)(rtw_get_beacon_interval_from_ie(cur_network->IEs)), 2);

		pframe += 2;
		pattrib->pktlen += 2;

		// capability info: 2 bytes

		memcpy(pframe, (unsigned char *)(rtw_get_capability_from_ie(cur_network->IEs)), 2);

		pframe += 2;
		pattrib->pktlen += 2;

		//below for ad-hoc mode

		// SSID
		pframe = rtw_set_ie(pframe, _SSID_IE_, cur_network->Ssid.SsidLength, cur_network->Ssid.Ssid, &pattrib->pktlen);

		// supported rates...
		rate_len = rtw_get_rateset_len(cur_network->SupportedRates);
		pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_, ((rate_len > 8)? 8: rate_len), cur_network->SupportedRates, &pattrib->pktlen);

		// DS parameter set
		pframe =rtw_set_ie(pframe, _DSSET_IE_, 1, (unsigned char *)&(cur_network->Configuration.DSConfig), &pattrib->pktlen);

		if( (pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)
		{
			uint8_t erpinfo=0;
			uint32_t	 ATIMWindow;
			// IBSS Parameter Set...
			//ATIMWindow = cur->Configuration.ATIMWindow;
			ATIMWindow = 0;
			pframe = rtw_set_ie(pframe, _IBSS_PARA_IE_, 2, (unsigned char *)(&ATIMWindow), &pattrib->pktlen);

			//ERP IE
			pframe = rtw_set_ie(pframe, _ERPINFO_IE_, 1, &erpinfo, &pattrib->pktlen);
		}


		// EXTERNDED SUPPORTED RATE
		if (rate_len > 8)
		{
			pframe = rtw_set_ie(pframe, _EXT_SUPPORTEDRATES_IE_, (rate_len - 8), (cur_network->SupportedRates + 8), &pattrib->pktlen);
		}


		//todo:HT for adhoc

	}

	pattrib->last_txcmdsz = pattrib->pktlen;


	dump_mgntframe(rtlpriv, pmgntframe);

	return;

}

int _issue_probereq(struct rtl_priv *rtlpriv, NDIS_802_11_SSID *pssid, uint8_t *da, int wait_ack)
{
	int ret = _FAIL;
	struct xmit_frame		*pmgntframe;
	struct tx_pkt_attrib		*pattrib;
	unsigned char			*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short		*fctrl;
	unsigned char			*mac;
	unsigned char			bssrate[NumRates];
	struct xmit_priv		*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	int	bssrate_len = 0;
	uint8_t	bc_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		goto exit;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);


	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	mac = rtlpriv->mac80211.mac_addr;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	if (da)
	{
		//	unicast probe request frame
		memcpy(pwlanhdr->addr1, da, ETH_ALEN);
		memcpy(pwlanhdr->addr3, da, ETH_ALEN);
	}
	else
	{
		//	broadcast probe request frame
		memcpy(pwlanhdr->addr1, bc_addr, ETH_ALEN);
		memcpy(pwlanhdr->addr3, bc_addr, ETH_ALEN);
	}

	memcpy(pwlanhdr->addr2, mac, ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_PROBEREQ);

	pframe += sizeof (struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof (struct rtw_ieee80211_hdr_3addr);

	if(pssid)
		pframe = rtw_set_ie(pframe, _SSID_IE_, pssid->SsidLength, pssid->Ssid, &(pattrib->pktlen));
	else
		pframe = rtw_set_ie(pframe, _SSID_IE_, 0, NULL, &(pattrib->pktlen));

	get_rate_set(rtlpriv, bssrate, &bssrate_len);

	if (bssrate_len > 8)
	{
		pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_ , 8, bssrate, &(pattrib->pktlen));
		pframe = rtw_set_ie(pframe, _EXT_SUPPORTEDRATES_IE_ , (bssrate_len - 8), (bssrate + 8), &(pattrib->pktlen));
	}
	else
	{
		pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_ , bssrate_len , bssrate, &(pattrib->pktlen));
	}

	//add wps_ie for wps2.0
	if(pmlmepriv->wps_probe_req_ie_len>0 && pmlmepriv->wps_probe_req_ie)
	{
		memcpy(pframe, pmlmepriv->wps_probe_req_ie, pmlmepriv->wps_probe_req_ie_len);
		pframe += pmlmepriv->wps_probe_req_ie_len;
		pattrib->pktlen += pmlmepriv->wps_probe_req_ie_len;
		//pmlmepriv->wps_probe_req_ie_len = 0 ;//reset to zero
	}

	pattrib->last_txcmdsz = pattrib->pktlen;

	if (wait_ack) {
		ret = dump_mgntframe_and_wait_ack(rtlpriv, pmgntframe);
	} else {
		dump_mgntframe(rtlpriv, pmgntframe);
		ret = _SUCCESS;
	}

exit:
	return ret;
}

inline void issue_probereq(struct rtl_priv *rtlpriv, NDIS_802_11_SSID *pssid, uint8_t *da)
{
	_issue_probereq(rtlpriv, pssid, da, false);
}

int issue_probereq_ex(struct rtl_priv *rtlpriv, NDIS_802_11_SSID *pssid, uint8_t *da,
	int try_cnt, int wait_ms)
{
	int ret;
	int i = 0;
	uint32_t	 start = jiffies;

	do
	{
		ret = _issue_probereq(rtlpriv, pssid, da, wait_ms>0?true:false);

		i++;

		if (rtlpriv->bDriverStopped || rtlpriv->bSurpriseRemoved)
			break;

		if(i < try_cnt && wait_ms > 0 && ret==_FAIL)
			msleep(wait_ms);

	}while((i<try_cnt) && ((ret==_FAIL)||(wait_ms==0)));

	if (ret != _FAIL) {
		ret = _SUCCESS;
		#ifndef DBG_XMIT_ACK
		goto exit;
		#endif
	}

exit:
	return ret;
}

// if psta == NULL, indiate we are station(client) now...
void issue_auth(struct rtl_priv *rtlpriv, struct sta_info *psta, unsigned short status)
{
	struct xmit_frame			*pmgntframe;
	struct tx_pkt_attrib			*pattrib;
	unsigned char					*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short				*fctrl;
	unsigned int					val32;
	unsigned short				val16;
	int use_shared_key = 0;
	struct xmit_priv			*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		return;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_AUTH);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);


	if(psta)// for AP mode
	{
		memcpy(pwlanhdr->addr1, psta->hwaddr, ETH_ALEN);
		memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
		memcpy(pwlanhdr->addr3, rtlpriv->mac80211.mac_addr, ETH_ALEN);


		// setting auth algo number
		val16 = (u16)psta->authalg;

		if(status != _STATS_SUCCESSFUL_)
			val16 = 0;

		if (val16)	{
			val16 = cpu_to_le16(val16);
			use_shared_key = 1;
		}

		pframe = rtw_set_fixed_ie(pframe, _AUTH_ALGM_NUM_, (unsigned char *)&val16, &(pattrib->pktlen));

		// setting auth seq number
		val16 =(u16)psta->auth_seq;
		val16 = cpu_to_le16(val16);
		pframe = rtw_set_fixed_ie(pframe, _AUTH_SEQ_NUM_, (unsigned char *)&val16, &(pattrib->pktlen));

		// setting status code...
		val16 = status;
		val16 = cpu_to_le16(val16);
		pframe = rtw_set_fixed_ie(pframe, _STATUS_CODE_, (unsigned char *)&val16, &(pattrib->pktlen));

		// added challenging text...
		if ((psta->auth_seq == 2) && (psta->state & WIFI_FW_AUTH_STATE) && (use_shared_key==1))
		{
			pframe = rtw_set_ie(pframe, _CHLGETXT_IE_, 128, psta->chg_txt, &(pattrib->pktlen));
		}
	}
	else
	{
		memcpy(pwlanhdr->addr1, get_my_bssid(&pmlmeinfo->network), ETH_ALEN);
		memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
		memcpy(pwlanhdr->addr3, get_my_bssid(&pmlmeinfo->network), ETH_ALEN);

		// setting auth algo number
		val16 = (pmlmeinfo->auth_algo == dot11AuthAlgrthm_Shared)? 1: 0;// 0:OPEN System, 1:Shared key
		if (val16)	{
			val16 = cpu_to_le16(val16);
			use_shared_key = 1;
		}
		//DBG_871X("%s auth_algo= %s auth_seq=%d\n",__FUNCTION__,(pmlmeinfo->auth_algo==0)?"OPEN":"SHARED",pmlmeinfo->auth_seq);

		//setting IV for auth seq #3
		if ((pmlmeinfo->auth_seq == 3) && (pmlmeinfo->state & WIFI_FW_AUTH_STATE) && (use_shared_key==1))
		{
			//DBG_871X("==> iv(%d),key_index(%d)\n",pmlmeinfo->iv,pmlmeinfo->key_index);
			val32 = ((pmlmeinfo->iv++) | (pmlmeinfo->key_index << 30));
			val32 = cpu_to_le32(val32);
			pframe = rtw_set_fixed_ie(pframe, 4, (unsigned char *)&val32, &(pattrib->pktlen));

			pattrib->iv_len = 4;
		}

		pframe = rtw_set_fixed_ie(pframe, _AUTH_ALGM_NUM_, (unsigned char *)&val16, &(pattrib->pktlen));

		// setting auth seq number
		val16 = pmlmeinfo->auth_seq;
		val16 = cpu_to_le16(val16);
		pframe = rtw_set_fixed_ie(pframe, _AUTH_SEQ_NUM_, (unsigned char *)&val16, &(pattrib->pktlen));


		// setting status code...
		val16 = status;
		val16 = cpu_to_le16(val16);
		pframe = rtw_set_fixed_ie(pframe, _STATUS_CODE_, (unsigned char *)&val16, &(pattrib->pktlen));

		// then checking to see if sending challenging text...
		if ((pmlmeinfo->auth_seq == 3) && (pmlmeinfo->state & WIFI_FW_AUTH_STATE) && (use_shared_key==1))
		{
			pframe = rtw_set_ie(pframe, _CHLGETXT_IE_, 128, pmlmeinfo->chg_txt, &(pattrib->pktlen));

			SetPrivacy(fctrl);

			pattrib->hdrlen = sizeof(struct rtw_ieee80211_hdr_3addr);

			pattrib->encrypt = WEP40_ENCRYPTION;

			pattrib->icv_len = 4;

			pattrib->pktlen += pattrib->icv_len;

		}

	}

	pattrib->last_txcmdsz = pattrib->pktlen;

	rtw_wep_encrypt(rtlpriv, (uint8_t *)pmgntframe);
	DBG_871X("%s\n", __FUNCTION__);
	dump_mgntframe(rtlpriv, pmgntframe);

	return;
}


void issue_asocrsp(struct rtl_priv *rtlpriv, unsigned short status, struct sta_info *pstat, int pkt_type)
{
#ifdef CONFIG_AP_MODE
	struct xmit_frame	*pmgntframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	struct tx_pkt_attrib *pattrib;
	unsigned char	*pbuf, *pframe;
	unsigned short val;
	unsigned short *fctrl;
	struct xmit_priv *pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX *pnetwork = &(pmlmeinfo->network);
	uint8_t *ie = pnetwork->IEs;

	DBG_871X("%s\n", __FUNCTION__);

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		return;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);


	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	memcpy((void *)GetAddr1Ptr(pwlanhdr), pstat->hwaddr, ETH_ALEN);
	memcpy((void *)GetAddr2Ptr(pwlanhdr), rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy((void *)GetAddr3Ptr(pwlanhdr), get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);


	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	if ((pkt_type == WIFI_ASSOCRSP) || (pkt_type == WIFI_REASSOCRSP))
		SetFrameSubType(pwlanhdr, pkt_type);
	else
		return;

	pattrib->hdrlen = sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen += pattrib->hdrlen;
	pframe += pattrib->hdrlen;

	//capability
	val = *(unsigned short *)rtw_get_capability_from_ie(ie);

	pframe = rtw_set_fixed_ie(pframe, _CAPABILITY_ , (unsigned char *)&val, &(pattrib->pktlen));

	status = cpu_to_le16(status);
	pframe = rtw_set_fixed_ie(pframe , _STATUS_CODE_ , (unsigned char *)&status, &(pattrib->pktlen));

	val = cpu_to_le16(pstat->aid | BIT(14) | BIT(15));
	pframe = rtw_set_fixed_ie(pframe, _ASOC_ID_ , (unsigned char *)&val, &(pattrib->pktlen));

	if (pstat->bssratelen <= 8)
	{
		pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_, pstat->bssratelen, pstat->bssrateset, &(pattrib->pktlen));
	}
	else
	{
		pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_, 8, pstat->bssrateset, &(pattrib->pktlen));
		pframe = rtw_set_ie(pframe, _EXT_SUPPORTEDRATES_IE_, (pstat->bssratelen-8), pstat->bssrateset+8, &(pattrib->pktlen));
	}

	if ((pstat->flags & WLAN_STA_HT) && (pmlmepriv->htpriv.ht_option))
	{
		uint ie_len=0;

		//FILL HT CAP INFO IE
		//p = hostapd_eid_ht_capabilities_info(hapd, p);
		pbuf = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _HT_CAPABILITY_IE_, &ie_len, (pnetwork->IELength - _BEACON_IE_OFFSET_));
		if(pbuf && ie_len>0)
		{
			memcpy(pframe, pbuf, ie_len+2);
			pframe += (ie_len+2);
			pattrib->pktlen +=(ie_len+2);
		}

		//FILL HT ADD INFO IE
		//p = hostapd_eid_ht_operation(hapd, p);
		pbuf = rtw_get_ie(ie + _BEACON_IE_OFFSET_, _HT_ADD_INFO_IE_, &ie_len, (pnetwork->IELength - _BEACON_IE_OFFSET_));
		if(pbuf && ie_len>0)
		{
			memcpy(pframe, pbuf, ie_len+2);
			pframe += (ie_len+2);
			pattrib->pktlen +=(ie_len+2);
		}

	}

	if ((pstat->flags & WLAN_STA_VHT) && (pmlmepriv->vhtpriv.vht_option))
	{
		uint32_t	 ie_len=0;

		//FILL VHT CAP IE
		pbuf = rtw_get_ie(ie + _BEACON_IE_OFFSET_, EID_VHTCapability, &ie_len, (pnetwork->IELength - _BEACON_IE_OFFSET_));
		if(pbuf && ie_len>0)
		{
			memcpy(pframe, pbuf, ie_len+2);
			pframe += (ie_len+2);
			pattrib->pktlen +=(ie_len+2);
		}

		//FILL VHT OPERATION IE
		pbuf = rtw_get_ie(ie + _BEACON_IE_OFFSET_, EID_VHTOperation, &ie_len, (pnetwork->IELength - _BEACON_IE_OFFSET_));
		if(pbuf && ie_len>0)
		{
			memcpy(pframe, pbuf, ie_len+2);
			pframe += (ie_len+2);
			pattrib->pktlen +=(ie_len+2);
		}
	}
	//FILL WMM IE
	if ((pstat->flags & WLAN_STA_WME) && (pmlmepriv->qospriv.qos_option))
	{
		uint ie_len=0;
		unsigned char WMM_PARA_IE[] = {0x00, 0x50, 0xf2, 0x02, 0x01, 0x01};

		for (pbuf = ie + _BEACON_IE_OFFSET_; ;pbuf+= (ie_len + 2))
		{
			pbuf = rtw_get_ie(pbuf, _VENDOR_SPECIFIC_IE_, &ie_len, (pnetwork->IELength - _BEACON_IE_OFFSET_ - (ie_len + 2)));
			if(pbuf && _rtw_memcmp(pbuf+2, WMM_PARA_IE, 6))
			{
				memcpy(pframe, pbuf, ie_len+2);
				pframe += (ie_len+2);
				pattrib->pktlen +=(ie_len+2);

				break;
			}

			if ((pbuf == NULL) || (ie_len == 0))
			{
				break;
			}
		}

	}


	if (pmlmeinfo->assoc_AP_vendor == HT_IOT_PEER_REALTEK)
	{
		pframe = rtw_set_ie(pframe, _VENDOR_SPECIFIC_IE_, 6 , REALTEK_96B_IE, &(pattrib->pktlen));
	}

	//add WPS IE ie for wps 2.0
	if(pmlmepriv->wps_assoc_resp_ie && pmlmepriv->wps_assoc_resp_ie_len>0)
	{
		memcpy(pframe, pmlmepriv->wps_assoc_resp_ie, pmlmepriv->wps_assoc_resp_ie_len);

		pframe += pmlmepriv->wps_assoc_resp_ie_len;
		pattrib->pktlen += pmlmepriv->wps_assoc_resp_ie_len;
	}

	pattrib->last_txcmdsz = pattrib->pktlen;

	dump_mgntframe(rtlpriv, pmgntframe);

#endif
}

void issue_assocreq(struct rtl_priv *rtlpriv)
{
	int ret = _FAIL;
	struct xmit_frame				*pmgntframe;
	struct tx_pkt_attrib				*pattrib;
	unsigned char					*pframe, *p;
	struct rtw_ieee80211_hdr			*pwlanhdr;
	unsigned short				*fctrl;
	unsigned short				val16;
	unsigned int					i, j, ie_len, index=0;
	unsigned char					rf_type, bssrate[NumRates], sta_bssrate[NumRates];
	PNDIS_802_11_VARIABLE_IEs	pIE;
	struct registry_priv	*pregpriv = &rtlpriv->registrypriv;
	struct xmit_priv		*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	int	bssrate_len = 0, sta_bssrate_len = 0;
	uint8_t	cbw40_enable = 0;

#ifdef CONFIG_DFS
	u16	cap;
#endif //CONFIG_DFS

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
		goto exit;

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);


	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;
	memcpy(pwlanhdr->addr1, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_ASSOCREQ);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	//caps

#ifdef CONFIG_DFS
	memcpy(&cap, rtw_get_capability_from_ie(pmlmeinfo->network.IEs), 2);
	cap |= BIT(8);
	memcpy(pframe, &cap, 2);
#else
	memcpy(pframe, rtw_get_capability_from_ie(pmlmeinfo->network.IEs), 2);
#endif //CONFIG_DFS

	pframe += 2;
	pattrib->pktlen += 2;

	//listen interval
	//todo: listen interval for power saving
	val16 = cpu_to_le16(3);
	memcpy(pframe ,(unsigned char *)&val16, 2);
	pframe += 2;
	pattrib->pktlen += 2;

	//SSID
	pframe = rtw_set_ie(pframe, _SSID_IE_,  pmlmeinfo->network.Ssid.SsidLength, pmlmeinfo->network.Ssid.Ssid, &(pattrib->pktlen));

	//supported rate & extended supported rate

#if 1	// Check if the AP's supported rates are also supported by STA.
	get_rate_set(rtlpriv, sta_bssrate, &sta_bssrate_len);
	//DBG_871X("sta_bssrate_len=%d\n", sta_bssrate_len);

	if(pmlmeext->cur_channel == 14)// for JAPAN, channel 14 can only uses B Mode(CCK)
	{
		sta_bssrate_len = 4;
	}


	//for (i = 0; i < sta_bssrate_len; i++) {
	//	DBG_871X("sta_bssrate[%d]=%02X\n", i, sta_bssrate[i]);
	//}

	for (i = 0; i < NDIS_802_11_LENGTH_RATES_EX; i++) {
		if (pmlmeinfo->network.SupportedRates[i] == 0) break;
		DBG_871X("network.SupportedRates[%d]=%02X\n", i, pmlmeinfo->network.SupportedRates[i]);
	}


	for (i = 0; i < NDIS_802_11_LENGTH_RATES_EX; i++) {
		if (pmlmeinfo->network.SupportedRates[i] == 0) break;


		// Check if the AP's supported rates are also supported by STA.
		for (j=0; j < sta_bssrate_len; j++) {
			 // Avoid the proprietary data rate (22Mbps) of Handlink WSG-4000 AP
			if ( (pmlmeinfo->network.SupportedRates[i]|IEEE80211_BASIC_RATE_MASK)
					== (sta_bssrate[j]|IEEE80211_BASIC_RATE_MASK)) {
				//DBG_871X("match i = %d, j=%d\n", i, j);
				break;
			} else {
				//DBG_871X("not match: %02X != %02X\n", (pmlmeinfo->network.SupportedRates[i]|IEEE80211_BASIC_RATE_MASK), (sta_bssrate[j]|IEEE80211_BASIC_RATE_MASK));
			}
		}

		if (j == sta_bssrate_len) {
			// the rate is not supported by STA
			DBG_871X("%s(): the rate[%d]=%02X is not supported by STA!\n",__FUNCTION__, i, pmlmeinfo->network.SupportedRates[i]);
		} else {
			// the rate is supported by STA
			bssrate[index++] = pmlmeinfo->network.SupportedRates[i];
		}
	}

	bssrate_len = index;
	DBG_871X("bssrate_len = %d\n", bssrate_len);

#else	// Check if the AP's supported rates are also supported by STA.
	for (bssrate_len = 0; bssrate_len < NumRates; bssrate_len++) {
		if (pmlmeinfo->network.SupportedRates[bssrate_len] == 0) break;

		if (pmlmeinfo->network.SupportedRates[bssrate_len] == 0x2C) // Avoid the proprietary data rate (22Mbps) of Handlink WSG-4000 AP
			break;

		bssrate[bssrate_len] = pmlmeinfo->network.SupportedRates[bssrate_len];
	}
#endif	// Check if the AP's supported rates are also supported by STA.

	if (bssrate_len == 0) {
		rtw_free_xmitbuf(pxmitpriv, pmgntframe->pxmitbuf);
		rtw_free_xmitframe(pxmitpriv, pmgntframe);
		goto exit; //don't connect to AP if no joint supported rate
	}


	if (bssrate_len > 8)
	{
		pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_ , 8, bssrate, &(pattrib->pktlen));
		pframe = rtw_set_ie(pframe, _EXT_SUPPORTEDRATES_IE_ , (bssrate_len - 8), (bssrate + 8), &(pattrib->pktlen));
	}
	else
	{
		pframe = rtw_set_ie(pframe, _SUPPORTEDRATES_IE_ , bssrate_len , bssrate, &(pattrib->pktlen));
	}

	//RSN
	p = rtw_get_ie((pmlmeinfo->network.IEs + sizeof(NDIS_802_11_FIXED_IEs)), _RSN_IE_2_, &ie_len, (pmlmeinfo->network.IELength - sizeof(NDIS_802_11_FIXED_IEs)));
	if (p != NULL)
	{
		pframe = rtw_set_ie(pframe, _RSN_IE_2_, ie_len, (p + 2), &(pattrib->pktlen));
	}

	//HT caps
	if(rtlpriv->mlmepriv.htpriv.ht_option==true)
	{
		p = rtw_get_ie((pmlmeinfo->network.IEs + sizeof(NDIS_802_11_FIXED_IEs)), _HT_CAPABILITY_IE_, &ie_len, (pmlmeinfo->network.IELength - sizeof(NDIS_802_11_FIXED_IEs)));
		if ((p != NULL) && (!(is_ap_in_tkip(rtlpriv))))
		{
			memcpy(&(pmlmeinfo->HT_caps), (p + 2), sizeof(struct HT_caps_element));

			//to disable 40M Hz support while gd_bw_40MHz_en = 0
			if (pmlmeext->cur_channel > 14) {
				if ((0x21 & 0xf0) > 0)
					cbw40_enable = 1;
			} else {
				if ((0x21 & 0x0f) > 0)
					cbw40_enable = 1;
			}

			if (cbw40_enable == 0)
			{
				pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info &= (~(BIT(6) | BIT(1)));
			}
			else
			{
				pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info |= BIT(1);
			}

			//todo: disable SM power save mode
			pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info |= 0x000c;

			rf_type = rtlpriv->phy.rf_type;

			switch(rf_type)
			{
				case RF_1T1R:

					if(pregpriv->rx_stbc)
						pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info |= cpu_to_le16(0x0100);//RX STBC One spatial stream

					memcpy(pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate, MCS_rate_1R, 16);
					break;

				case RF_2T2R:
				case RF_1T2R:
				default:

					if((pregpriv->rx_stbc == 0x3) ||//enable for 2.4/5 GHz
						((pmlmeext->cur_wireless_mode & WIRELESS_11_24N) && (pregpriv->rx_stbc == 0x1)) || //enable for 2.4GHz
						((pmlmeext->cur_wireless_mode & WIRELESS_11_5N) && (pregpriv->rx_stbc == 0x2))) //enable for 5GHz
					{
						DBG_871X("declare supporting RX STBC\n");
						pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info |= cpu_to_le16(0x0200);//RX STBC two spatial stream
					}
					#ifdef CONFIG_DISABLE_MCS13TO15
					if(pmlmeext->cur_bwmode == CHANNEL_WIDTH_40 && (pregpriv->wifi_spec!=1))
						memcpy(pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate, MCS_rate_2R_MCS13TO15_OFF, 16);
					else
					memcpy(pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate, MCS_rate_2R, 16);
					#else //CONFIG_DISABLE_MCS13TO15
					memcpy(pmlmeinfo->HT_caps.u.HT_cap_element.MCS_rate, MCS_rate_2R, 16);
					#endif //CONFIG_DISABLE_MCS13TO15
					break;
			}
			pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info = cpu_to_le16(pmlmeinfo->HT_caps.u.HT_cap_element.HT_caps_info);


			pframe = rtw_set_ie(pframe, _HT_CAPABILITY_IE_, ie_len , (uint8_t *)(&(pmlmeinfo->HT_caps)), &(pattrib->pktlen));

		}
	}

	//vendor specific IE, such as WPA, WMM, WPS
	for (i = sizeof(NDIS_802_11_FIXED_IEs); i < pmlmeinfo->network.IELength;)
	{
		pIE = (PNDIS_802_11_VARIABLE_IEs)(pmlmeinfo->network.IEs + i);

		switch (pIE->ElementID)
		{
			case _VENDOR_SPECIFIC_IE_:
				if ((_rtw_memcmp(pIE->data, RTW_WPA_OUI, 4)) ||
						(_rtw_memcmp(pIE->data, WMM_OUI, 4)) ||
						(_rtw_memcmp(pIE->data, WPS_OUI, 4)))
				{
					//Commented by Kurt 20110629
					//In some older APs, WPS handshake
					//would be fail if we append vender extensions informations to AP
					if(_rtw_memcmp(pIE->data, WPS_OUI, 4)){
						pIE->Length=14;
					}
					pframe = rtw_set_ie(pframe, _VENDOR_SPECIFIC_IE_, pIE->Length, pIE->data, &(pattrib->pktlen));
				}
				break;
			case EID_VHTCapability:
				if (rtlpriv->mlmepriv.vhtpriv.vht_option ==true) {
					pframe = rtw_set_ie(pframe, EID_VHTCapability, pIE->Length, pIE->data, &(pattrib->pktlen));
				}
				break;

			case EID_OpModeNotification:
				if (rtlpriv->mlmepriv.vhtpriv.vht_option ==true) {
					pframe = rtw_set_ie(pframe, EID_OpModeNotification, pIE->Length, pIE->data, &(pattrib->pktlen));
				}
				break;
			default:
				break;
		}

		i += (pIE->Length + 2);
	}

	if (pmlmeinfo->assoc_AP_vendor == HT_IOT_PEER_REALTEK)
	{
		pframe = rtw_set_ie(pframe, _VENDOR_SPECIFIC_IE_, 6 , REALTEK_96B_IE, &(pattrib->pktlen));
	}


	pattrib->last_txcmdsz = pattrib->pktlen;
	dump_mgntframe(rtlpriv, pmgntframe);

	ret = _SUCCESS;

exit:
	if (ret == _SUCCESS)
		rtw_buf_update(&pmlmepriv->assoc_req, &pmlmepriv->assoc_req_len, (uint8_t *)pwlanhdr, pattrib->pktlen);
	else
		rtw_buf_free(&pmlmepriv->assoc_req, &pmlmepriv->assoc_req_len);

	return;
}

//when wait_ack is ture, this function shoule be called at process context
static int _issue_nulldata(struct rtl_priv *rtlpriv, unsigned char *da, unsigned int power_mode, int wait_ack)
{
	int ret = _FAIL;
	struct xmit_frame			*pmgntframe;
	struct tx_pkt_attrib			*pattrib;
	unsigned char					*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short				*fctrl;
	struct xmit_priv	*pxmitpriv;
	struct mlme_ext_priv	*pmlmeext;
	struct mlme_ext_info	*pmlmeinfo;

	//DBG_871X("%s:%d\n", __FUNCTION__, power_mode);

	if(!rtlpriv)
		goto exit;

	pxmitpriv = &(rtlpriv->xmitpriv);
	pmlmeext = &(rtlpriv->mlmeextpriv);
	pmlmeinfo = &(pmlmeext->mlmext_info);

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		goto exit;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);
	pattrib->retry_ctrl = false;

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	if((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
	{
		SetFrDs(fctrl);
	}
	else if((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE)
	{
		SetToDs(fctrl);
	}

	if (power_mode)
	{
		SetPwrMgt(fctrl);
	}

	memcpy(pwlanhdr->addr1, da, ETH_ALEN);
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_DATA_NULL);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	pattrib->last_txcmdsz = pattrib->pktlen;

	if(wait_ack)
	{
		ret = dump_mgntframe_and_wait_ack(rtlpriv, pmgntframe);
	}
	else
	{
		dump_mgntframe(rtlpriv, pmgntframe);
		ret = _SUCCESS;
	}

exit:
	return ret;
}


//when wait_ms >0 , this function shoule be called at process context
//da == NULL for station mode
int issue_nulldata(struct rtl_priv *rtlpriv, unsigned char *da, unsigned int power_mode, int try_cnt, int wait_ms)
{
	int ret;
	int i = 0;
	uint32_t	 start = jiffies;
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	/* da == NULL, assum it's null data for sta to ap*/
	if (da == NULL)
		da = get_my_bssid(&(pmlmeinfo->network));

	do
	{
		ret = _issue_nulldata(rtlpriv, da, power_mode, wait_ms>0?true:false);

		i++;

		if (rtlpriv->bDriverStopped || rtlpriv->bSurpriseRemoved)
			break;

		if(i < try_cnt && wait_ms > 0 && ret==_FAIL)
			msleep(wait_ms);

	}while((i<try_cnt) && ((ret==_FAIL)||(wait_ms==0)));

	if (ret != _FAIL) {
		ret = _SUCCESS;
		#ifndef DBG_XMIT_ACK
		goto exit;
		#endif
	}

exit:
	return ret;
}

//when wait_ack is ture, this function shoule be called at process context
static int _issue_qos_nulldata(struct rtl_priv *rtlpriv, unsigned char *da, u16 tid, int wait_ack)
{
	int ret = _FAIL;
	struct xmit_frame			*pmgntframe;
	struct tx_pkt_attrib			*pattrib;
	unsigned char					*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short				*fctrl, *qc;
	struct xmit_priv			*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	DBG_871X("%s\n", __FUNCTION__);

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		goto exit;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);

	pattrib->hdrlen +=2;
	pattrib->qos_en = true;
	pattrib->eosp = 1;
	pattrib->ack_policy = 0;
	pattrib->mdata = 0;

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	if((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
	{
		SetFrDs(fctrl);
	}
	else if((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE)
	{
		SetToDs(fctrl);
	}

	if(pattrib->mdata)
		SetMData(fctrl);

	qc = (unsigned short *)(pframe + pattrib->hdrlen - 2);

	SetPriority(qc, tid);

	SetEOSP(qc, pattrib->eosp);

	SetAckpolicy(qc, pattrib->ack_policy);

	memcpy(pwlanhdr->addr1, da, ETH_ALEN);
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_QOS_DATA_NULL);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr_qos);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr_qos);

	pattrib->last_txcmdsz = pattrib->pktlen;

	if(wait_ack)
	{
		ret = dump_mgntframe_and_wait_ack(rtlpriv, pmgntframe);
	}
	else
	{
		dump_mgntframe(rtlpriv, pmgntframe);
		ret = _SUCCESS;
	}

exit:
	return ret;
}

//when wait_ms >0 , this function shoule be called at process context
//da == NULL for station mode
int issue_qos_nulldata(struct rtl_priv *rtlpriv, unsigned char *da, u16 tid, int try_cnt, int wait_ms)
{
	int ret;
	int i = 0;
	uint32_t	 start = jiffies;
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	/* da == NULL, assum it's null data for sta to ap*/
	if (da == NULL)
		da = get_my_bssid(&(pmlmeinfo->network));

	do
	{
		ret = _issue_qos_nulldata(rtlpriv, da, tid, wait_ms>0?true:false);

		i++;

		if (rtlpriv->bDriverStopped || rtlpriv->bSurpriseRemoved)
			break;

		if(i < try_cnt && wait_ms > 0 && ret==_FAIL)
			msleep(wait_ms);

	}while((i<try_cnt) && ((ret==_FAIL)||(wait_ms==0)));

	if (ret != _FAIL) {
		ret = _SUCCESS;
		#ifndef DBG_XMIT_ACK
		goto exit;
		#endif
	}

exit:
	return ret;
}

static int _issue_deauth(struct rtl_priv *rtlpriv, unsigned char *da, unsigned short reason, uint8_t wait_ack)
{
	struct xmit_frame			*pmgntframe;
	struct tx_pkt_attrib			*pattrib;
	unsigned char					*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short				*fctrl;
	struct xmit_priv			*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	int ret = _FAIL;

	//DBG_871X("%s to "MAC_FMT"\n", __func__, MAC_ARG(da));

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		goto exit;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);
	pattrib->retry_ctrl = false;

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	memcpy(pwlanhdr->addr1, da, ETH_ALEN);
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_DEAUTH);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	reason = cpu_to_le16(reason);
	pframe = rtw_set_fixed_ie(pframe, _RSON_CODE_ , (unsigned char *)&reason, &(pattrib->pktlen));

	pattrib->last_txcmdsz = pattrib->pktlen;


	if(wait_ack)
	{
		ret = dump_mgntframe_and_wait_ack(rtlpriv, pmgntframe);
	}
	else
	{
		dump_mgntframe(rtlpriv, pmgntframe);
		ret = _SUCCESS;
	}

exit:
	return ret;
}

int issue_deauth(struct rtl_priv *rtlpriv, unsigned char *da, unsigned short reason)
{
	DBG_871X("%s to "MAC_FMT"\n", __func__, MAC_ARG(da));
	return _issue_deauth(rtlpriv, da, reason, false);
}

int issue_deauth_ex(struct rtl_priv *rtlpriv, uint8_t *da, unsigned short reason, int try_cnt,
	int wait_ms)
{
	int ret;
	int i = 0;
	uint32_t	 start = jiffies;

	do
	{
		ret = _issue_deauth(rtlpriv, da, reason, wait_ms>0?true:false);

		i++;

		if (rtlpriv->bDriverStopped || rtlpriv->bSurpriseRemoved)
			break;

		if(i < try_cnt && wait_ms > 0 && ret==_FAIL)
			msleep(wait_ms);

	}while((i<try_cnt) && ((ret==_FAIL)||(wait_ms==0)));

	if (ret != _FAIL) {
		ret = _SUCCESS;
		#ifndef DBG_XMIT_ACK
		goto exit;
		#endif
	}

exit:
	return ret;
}

void issue_action_spct_ch_switch(struct rtl_priv *rtlpriv, uint8_t *ra, uint8_t new_ch, uint8_t ch_offset)
{
	struct list_head		*plist, *phead;
	struct xmit_frame			*pmgntframe;
	struct tx_pkt_attrib			*pattrib;
	unsigned char				*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short			*fctrl;
	struct xmit_priv			*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);


	DBG_871X(FUNC_NDEV_FMT" ra="MAC_FMT", ch:%u, offset:%u\n",
		FUNC_NDEV_ARG(rtlpriv->ndev), MAC_ARG(ra), new_ch, ch_offset);

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
		return;

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	memcpy(pwlanhdr->addr1, ra, ETH_ALEN); /* RA */
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN); /* TA */
	memcpy(pwlanhdr->addr3, ra, ETH_ALEN); /* DA = RA */

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_ACTION);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	/* category, action */
	{
		uint8_t category, action;
		category = RTW_WLAN_CATEGORY_SPECTRUM_MGMT;
		action = RTW_WLAN_ACTION_SPCT_CHL_SWITCH;

		pframe = rtw_set_fixed_ie(pframe, 1, &(category), &(pattrib->pktlen));
		pframe = rtw_set_fixed_ie(pframe, 1, &(action), &(pattrib->pktlen));
	}

	pframe = rtw_set_ie_ch_switch(pframe, &(pattrib->pktlen), 0, new_ch, 0);
	pframe = rtw_set_ie_secondary_ch_offset(pframe, &(pattrib->pktlen),
		hal_ch_offset_to_secondary_ch_offset(ch_offset));

	pattrib->last_txcmdsz = pattrib->pktlen;

	dump_mgntframe(rtlpriv, pmgntframe);

}

void issue_action_BA(struct rtl_priv *rtlpriv, unsigned char *raddr, unsigned char action, unsigned short status)
{
	uint8_t	category = RTW_WLAN_CATEGORY_BACK;
	u16	start_seq;
	u16	BA_para_set;
	u16	reason_code;
	u16	BA_timeout_value;
	u16	BA_starting_seqctrl;
	HT_CAP_AMPDU_FACTOR max_rx_ampdu_factor;
	struct xmit_frame		*pmgntframe;
	struct tx_pkt_attrib		*pattrib;
	uint8_t					*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	u16					*fctrl;
	struct xmit_priv		*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct sta_info		*psta;
	struct sta_priv		*pstapriv = &rtlpriv->stapriv;
	struct registry_priv	 	*pregpriv = &rtlpriv->registrypriv;

	DBG_871X("%s, category=%d, action=%d, status=%d\n", __FUNCTION__, category, action, status);

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		return;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	//memcpy(pwlanhdr->addr1, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);
	memcpy(pwlanhdr->addr1, raddr, ETH_ALEN);
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_ACTION);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	pframe = rtw_set_fixed_ie(pframe, 1, &(category), &(pattrib->pktlen));
	pframe = rtw_set_fixed_ie(pframe, 1, &(action), &(pattrib->pktlen));

      status = cpu_to_le16(status);


	if (category == 3)
	{
		switch (action)
		{
			case 0: //ADDBA req
				do {
					pmlmeinfo->dialogToken++;
				} while (pmlmeinfo->dialogToken == 0);
				pframe = rtw_set_fixed_ie(pframe, 1, &(pmlmeinfo->dialogToken), &(pattrib->pktlen));

				{
					BA_para_set = (0x1002 | ((status & 0xf) << 2)); //immediate ack & 64 buffer size
				}
				//sys_mib.BA_para_set = 0x0802; //immediate ack & 32 buffer size
				BA_para_set = cpu_to_le16(BA_para_set);
				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&(BA_para_set)), &(pattrib->pktlen));

				//BA_timeout_value = 0xffff;//max: 65535 TUs(~ 65 ms)
				BA_timeout_value = 5000;//~ 5ms
				BA_timeout_value = cpu_to_le16(BA_timeout_value);
				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&(BA_timeout_value)), &(pattrib->pktlen));

				//if ((psta = rtw_get_stainfo(pstapriv, pmlmeinfo->network.MacAddress)) != NULL)
				if ((psta = rtw_get_stainfo(pstapriv, raddr)) != NULL)
				{
					start_seq = (psta->sta_xmitpriv.txseq_tid[status & 0x07]&0xfff) + 1;

					DBG_871X("BA_starting_seqctrl = %d for TID=%d\n", start_seq, status & 0x07);

					psta->BA_starting_seqctrl[status & 0x07] = start_seq;

					BA_starting_seqctrl = start_seq << 4;
				}

				BA_starting_seqctrl = cpu_to_le16(BA_starting_seqctrl);
				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&(BA_starting_seqctrl)), &(pattrib->pktlen));
				break;

			case 1: //ADDBA rsp
				pframe = rtw_set_fixed_ie(pframe, 1, &(pmlmeinfo->ADDBA_req.dialog_token), &(pattrib->pktlen));
				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&status), &(pattrib->pktlen));
				/*
				//BA_para_set = cpu_to_le16((le16_to_cpu(pmlmeinfo->ADDBA_req.BA_para_set) & 0x3f) | 0x1000); //64 buffer size
				*/
				max_rx_ampdu_factor = MAX_AMPDU_FACTOR_64K;
				if(MAX_AMPDU_FACTOR_64K == max_rx_ampdu_factor)
					BA_para_set = ((le16_to_cpu(pmlmeinfo->ADDBA_req.BA_para_set) & 0x3f) | 0x1000); //64 buffer size
				else if(MAX_AMPDU_FACTOR_32K == max_rx_ampdu_factor)
					BA_para_set = ((le16_to_cpu(pmlmeinfo->ADDBA_req.BA_para_set) & 0x3f) | 0x0800); //32 buffer size
				else if(MAX_AMPDU_FACTOR_16K == max_rx_ampdu_factor)
					BA_para_set = ((le16_to_cpu(pmlmeinfo->ADDBA_req.BA_para_set) & 0x3f) | 0x0400); //16 buffer size
				else if(MAX_AMPDU_FACTOR_8K == max_rx_ampdu_factor)
					BA_para_set = ((le16_to_cpu(pmlmeinfo->ADDBA_req.BA_para_set) & 0x3f) | 0x0200); //8 buffer size
				else
					BA_para_set = ((le16_to_cpu(pmlmeinfo->ADDBA_req.BA_para_set) & 0x3f) | 0x1000); //64 buffer size


				if(pregpriv->ampdu_amsdu==0)//disabled
					BA_para_set = cpu_to_le16(BA_para_set & ~BIT(0));
				else if(pregpriv->ampdu_amsdu==1)//enabled
					BA_para_set = cpu_to_le16(BA_para_set | BIT(0));
				else //auto
					BA_para_set = cpu_to_le16(BA_para_set);

				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&(BA_para_set)), &(pattrib->pktlen));
				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&(pmlmeinfo->ADDBA_req.BA_timeout_value)), &(pattrib->pktlen));
				break;
			case 2://DELBA
				BA_para_set = (status & 0x1F) << 3;
				BA_para_set = cpu_to_le16(BA_para_set);
				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&(BA_para_set)), &(pattrib->pktlen));

				reason_code = 37;//Requested from peer STA as it does not want to use the mechanism
				reason_code = cpu_to_le16(reason_code);
				pframe = rtw_set_fixed_ie(pframe, 2, (unsigned char *)(&(reason_code)), &(pattrib->pktlen));
				break;
			default:
				break;
		}
	}

	pattrib->last_txcmdsz = pattrib->pktlen;

	dump_mgntframe(rtlpriv, pmgntframe);
}

static void issue_action_BSSCoexistPacket(struct rtl_priv *rtlpriv)
{
	struct list_head		*plist, *phead;
	unsigned char category, action;
	struct xmit_frame			*pmgntframe;
	struct tx_pkt_attrib			*pattrib;
	unsigned char				*pframe;
	struct rtw_ieee80211_hdr	*pwlanhdr;
	unsigned short			*fctrl;
	struct	wlan_network	*pnetwork = NULL;
	struct xmit_priv			*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct __queue		*queue	= &(pmlmepriv->scanned_queue);
	uint8_t InfoContent[16] = {0};
	uint8_t ICS[8][15];
	if((pmlmepriv->num_FortyMHzIntolerant==0) || (pmlmepriv->num_sta_no_ht==0))
		return;

	if(true == pmlmeinfo->bwmode_updated)
		return;


	DBG_871X("%s\n", __FUNCTION__);


	category = RTW_WLAN_CATEGORY_PUBLIC;
	action = ACT_PUBLIC_BSSCOEXIST;

	if ((pmgntframe = alloc_mgtxmitframe(pxmitpriv)) == NULL)
	{
		return;
	}

	//update attribute
	pattrib = &pmgntframe->tx_attrib;
	update_mgntframe_attrib(rtlpriv, pattrib);

	memset(pmgntframe->buf_addr, 0, WLANHDR_OFFSET + TXDESC_OFFSET);

	pframe = (uint8_t *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	pwlanhdr = (struct rtw_ieee80211_hdr *)pframe;

	fctrl = &(pwlanhdr->frame_ctl);
	*(fctrl) = 0;

	memcpy(pwlanhdr->addr1, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);
	memcpy(pwlanhdr->addr2, rtlpriv->mac80211.mac_addr, ETH_ALEN);
	memcpy(pwlanhdr->addr3, get_my_bssid(&(pmlmeinfo->network)), ETH_ALEN);

	SetSeqNum(pwlanhdr, pmlmeext->mgnt_seq);
	pmlmeext->mgnt_seq++;
	SetFrameSubType(pframe, WIFI_ACTION);

	pframe += sizeof(struct rtw_ieee80211_hdr_3addr);
	pattrib->pktlen = sizeof(struct rtw_ieee80211_hdr_3addr);

	pframe = rtw_set_fixed_ie(pframe, 1, &(category), &(pattrib->pktlen));
	pframe = rtw_set_fixed_ie(pframe, 1, &(action), &(pattrib->pktlen));


	//
	if(pmlmepriv->num_FortyMHzIntolerant>0)
	{
		uint8_t iedata=0;

		iedata |= BIT(2);//20 MHz BSS Width Request

		pframe = rtw_set_ie(pframe, EID_BSSCoexistence,  1, &iedata, &(pattrib->pktlen));

	}


	//
	memset(ICS, 0, sizeof(ICS));
	if(pmlmepriv->num_sta_no_ht>0)
	{
		int i;

		spin_lock_bh(&(pmlmepriv->scanned_queue.lock));

		phead = get_list_head(queue);
		plist = get_next(phead);

		while(1)
		{
			int len;
			uint8_t *p;
			WLAN_BSSID_EX *pbss_network;

			if (rtw_end_of_queue_search(phead,plist)== true)
				break;

			pnetwork = container_of(plist, struct wlan_network, list);

			plist = get_next(plist);

			pbss_network = (WLAN_BSSID_EX *)&pnetwork->network;

			p = rtw_get_ie(pbss_network->IEs + _FIXED_IE_LENGTH_, _HT_CAPABILITY_IE_, &len, pbss_network->IELength - _FIXED_IE_LENGTH_);
			if((p==NULL) || (len==0))//non-HT
			{
				if((pbss_network->Configuration.DSConfig<=0) || (pbss_network->Configuration.DSConfig>14))
					continue;

				ICS[0][pbss_network->Configuration.DSConfig]=1;

				if(ICS[0][0] == 0)
					ICS[0][0] = 1;
			}

		}

		spin_unlock_bh(&(pmlmepriv->scanned_queue.lock));


		for(i= 0;i<8;i++)
		{
			if(ICS[i][0] == 1)
			{
				int j, k = 0;

				InfoContent[k] = i;
				//SET_BSS_INTOLERANT_ELE_REG_CLASS(InfoContent,i);
				k++;

				for(j=1;j<=14;j++)
				{
					if(ICS[i][j]==1)
					{
						if(k<16)
						{
							InfoContent[k] = j; //channel number
							//SET_BSS_INTOLERANT_ELE_CHANNEL(InfoContent+k, j);
							k++;
						}
					}
				}

				pframe = rtw_set_ie(pframe, EID_BSSIntolerantChlReport, k, InfoContent, &(pattrib->pktlen));

			}

		}


	}


	pattrib->last_txcmdsz = pattrib->pktlen;

	dump_mgntframe(rtlpriv, pmgntframe);
}

unsigned int send_delba(struct rtl_priv *rtlpriv, uint8_t initiator, uint8_t *addr)
{
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct sta_info *psta = NULL;
	//struct recv_reorder_ctrl *preorder_ctrl;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	u16 tid;

	if((pmlmeinfo->state&0x03) != WIFI_FW_AP_STATE)
		if (!(pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS))
			return _SUCCESS;

	psta = rtw_get_stainfo(pstapriv, addr);
	if(psta==NULL)
		return _SUCCESS;

	//DBG_871X("%s:%s\n", __FUNCTION__, (initiator==0)?"RX_DIR":"TX_DIR");

	if(initiator==0) // recipient
	{
		for(tid = 0;tid<MAXTID;tid++)
		{
			if(psta->recvreorder_ctrl[tid].enable == true)
			{
				DBG_871X("rx agg disable tid(%d)\n",tid);
				issue_action_BA(rtlpriv, addr, RTW_WLAN_ACTION_DELBA, (((tid <<1) |initiator)&0x1F));
				psta->recvreorder_ctrl[tid].enable = false;
				psta->recvreorder_ctrl[tid].indicate_seq = 0xffff;
			}
		}
	}
	else if(initiator == 1)// originator
	{
		//DBG_871X("tx agg_enable_bitmap(0x%08x)\n", psta->htpriv.agg_enable_bitmap);
		for(tid = 0;tid<MAXTID;tid++)
		{
			if(psta->htpriv.agg_enable_bitmap & BIT(tid))
			{
				DBG_871X("tx agg disable tid(%d)\n",tid);
				issue_action_BA(rtlpriv, addr, RTW_WLAN_ACTION_DELBA, (((tid <<1) |initiator)&0x1F) );
				psta->htpriv.agg_enable_bitmap &= ~BIT(tid);
				psta->htpriv.candidate_tid_bitmap &= ~BIT(tid);

			}
		}
	}

	return _SUCCESS;

}

unsigned int send_beacon(struct rtl_priv *rtlpriv)
{
	uint8_t	bxmitok = false;
	int	issue=0;
	int poll = 0;


	uint32_t	 start = jiffies;

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BCN_VALID, NULL);
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_DL_BCN_SEL, NULL);
	do{
		issue_beacon(rtlpriv, 100);
		issue++;
		do {
			rtw_yield_os();
			rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_BCN_VALID, (uint8_t *)(&bxmitok));
			poll++;
		}while((poll%10)!=0 && false == bxmitok && !rtlpriv->bSurpriseRemoved && !rtlpriv->bDriverStopped);

	}while(false == bxmitok && issue<100 && !rtlpriv->bSurpriseRemoved && !rtlpriv->bDriverStopped);

	if(rtlpriv->bSurpriseRemoved || rtlpriv->bDriverStopped)
	{
		return _FAIL;
	}


	if(false == bxmitok)
	{
		DBG_871X("%s fail! %u ms\n", __FUNCTION__, rtw_get_passing_time_ms(start));
		return _FAIL;
	}
	else
	{
		uint32_t	 passing_time = rtw_get_passing_time_ms(start);

		if(passing_time > 100 || issue > 3)
			DBG_871X("%s success, issue:%d, poll:%d, %u ms\n", __FUNCTION__, issue, poll, rtw_get_passing_time_ms(start));
		//else
		//	DBG_871X("%s success, issue:%d, poll:%d, %u ms\n", __FUNCTION__, issue, poll, rtw_get_passing_time_ms(start));

		return _SUCCESS;
	}


}

/****************************************************************************

Following are some utitity fuctions for WiFi MLME

*****************************************************************************/

bool IsLegal5GChannel(struct rtl_priv *rtlpriv, uint8_t	channel)
{

	int i=0;
	uint8_t Channel_5G[45] = {36,38,40,42,44,46,48,50,52,54,56,58,
		60,62,64,100,102,104,106,108,110,112,114,116,118,120,122,
		124,126,128,130,132,134,136,138,140,149,151,153,155,157,159,
		161,163,165};
	for(i=0;i<sizeof(Channel_5G);i++)
		if(channel == Channel_5G[i])
			return true;
	return false;
}

void site_survey(struct rtl_priv *rtlpriv)
{
	unsigned char		survey_channel = 0, val8;
	RT_SCAN_TYPE	ScanType = SCAN_PASSIVE;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint32_t	 initialgain = 0;

	{
		struct rtw_ieee80211_channel *ch;
		if (pmlmeext->sitesurvey_res.channel_idx < pmlmeext->sitesurvey_res.ch_num) {
			ch = &pmlmeext->sitesurvey_res.ch[pmlmeext->sitesurvey_res.channel_idx];
			survey_channel = ch->hw_value;
			ScanType = (ch->flags & RTW_IEEE80211_CHAN_PASSIVE_SCAN) ? SCAN_PASSIVE : SCAN_ACTIVE;
		}
	}

	if(survey_channel != 0)
	{
		//PAUSE 4-AC Queue when site_survey
		//rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_TXPAUSE, (uint8_t *)(&val8));
		//val8 |= 0x0f;
		//rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_TXPAUSE, (uint8_t *)(&val8));
		if(pmlmeext->sitesurvey_res.channel_idx == 0)
		{
#ifdef DBG_FIXED_CHAN
			if(pmlmeext->fixed_chan !=0xff)
				set_channel_bwmode(rtlpriv, pmlmeext->fixed_chan, HAL_PRIME_CHNL_OFFSET_DONT_CARE, CHANNEL_WIDTH_20);
			else
#endif
				set_channel_bwmode(rtlpriv, survey_channel, HAL_PRIME_CHNL_OFFSET_DONT_CARE, CHANNEL_WIDTH_20);
		}
		else
		{
#ifdef DBG_FIXED_CHAN
			if(pmlmeext->fixed_chan!=0xff)
				SelectChannel(rtlpriv, pmlmeext->fixed_chan);
			else
#endif
				SelectChannel(rtlpriv, survey_channel);
		}


		if(ScanType == SCAN_ACTIVE) //obey the channel plan setting...
		{
			{
				int i;
				for(i=0;i<RTW_SSID_SCAN_AMOUNT;i++){
					if(pmlmeext->sitesurvey_res.ssid[i].SsidLength) {
						//todo: to issue two probe req???
						issue_probereq(rtlpriv, &(pmlmeext->sitesurvey_res.ssid[i]), NULL);
						//msleep(SURVEY_TO>>1);
						issue_probereq(rtlpriv, &(pmlmeext->sitesurvey_res.ssid[i]), NULL);
					}
				}

				if(pmlmeext->sitesurvey_res.scan_mode == SCAN_ACTIVE) {
					//todo: to issue two probe req???
					issue_probereq(rtlpriv, NULL, NULL);
					//msleep(SURVEY_TO>>1);
					issue_probereq(rtlpriv, NULL, NULL);
				}
			}
		}

			set_survey_timer(pmlmeext, pmlmeext->chan_scan_time);

	}
	else
	{

		//	channel number is 0 or this channel is not valid.



		{
			pmlmeext->sitesurvey_res.state = SCAN_COMPLETE;

			//switch back to the original channel
			//SelectChannel(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset);

			{
			}

			//flush 4-AC Queue after site_survey
			//val8 = 0;
			//rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_TXPAUSE, (uint8_t *)(&val8));

			//config MSR
			Set_MSR(rtlpriv, (pmlmeinfo->state & 0x3));

			initialgain = 0xff; //restore RX GAIN
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_INITIAL_GAIN, (uint8_t *)(&initialgain));
			//Switch_DM_Func(rtlpriv, DYNAMIC_ALL_FUNC_ENABLE, true);

			if (is_client_associated_to_ap(rtlpriv) == true)
			{
				issue_nulldata(rtlpriv, NULL, 0, 3, 500);

			}

			val8 = 0; //survey done
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_SITESURVEY, (uint8_t *)(&val8));

			report_surveydone_event(rtlpriv);

			pmlmeext->chan_scan_time = SURVEY_TO;
			pmlmeext->sitesurvey_res.state = SCAN_DISABLE;

			issue_action_BSSCoexistPacket(rtlpriv);
			issue_action_BSSCoexistPacket(rtlpriv);
			issue_action_BSSCoexistPacket(rtlpriv);

		}


	}

	return;

}

//collect bss info from Beacon and Probe response frames.
uint8_t collect_bss_info(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame, WLAN_BSSID_EX *bssid)
{
	int	i;
	uint32_t	len;
	uint8_t	*p;
	u16	val16, subtype;
	uint8_t	*pframe = precv_frame->rx_data;
	uint32_t	packet_len = precv_frame->len;
	struct registry_priv 	*pregistrypriv = &rtlpriv->registrypriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	len = packet_len - sizeof(struct rtw_ieee80211_hdr_3addr);

	if (len > MAX_IE_SZ)
	{
		//DBG_871X("IE too long for survey event\n");
		return _FAIL;
	}

	memset(bssid, 0, sizeof(WLAN_BSSID_EX));

	subtype = GetFrameSubType(pframe);

	if(subtype==WIFI_BEACON)
		bssid->Reserved[0] = 1;
	else
		bssid->Reserved[0] = 0;

	bssid->Length = sizeof(WLAN_BSSID_EX) - MAX_IE_SZ + len;

	//below is to copy the information element
	bssid->IELength = len;
	memcpy(bssid->IEs, (pframe + sizeof(struct rtw_ieee80211_hdr_3addr)), bssid->IELength);

	//get the signal strength
	//bssid->Rssi = precv_frame->u.hdr.attrib.SignalStrength; // 0-100 index.
	bssid->Rssi = precv_frame->attrib.phy_info.RecvSignalPower; // in dBM.raw data
	bssid->PhyInfo.SignalQuality = precv_frame->attrib.phy_info.SignalQuality;//in percentage
	bssid->PhyInfo.SignalStrength = precv_frame->attrib.phy_info.SignalStrength;//in percentage

	// checking SSID
	if ((p = rtw_get_ie(bssid->IEs + _FIXED_IE_LENGTH_, _SSID_IE_, &len, bssid->IELength - _FIXED_IE_LENGTH_)) == NULL)
	{
		DBG_871X("marc: cannot find SSID for survey event\n");
		return _FAIL;
	}

	if (*(p + 1))
	{
		if (len > NDIS_802_11_LENGTH_SSID)
		{
			DBG_871X("%s()-%d: IE too long (%d) for survey event\n", __FUNCTION__, __LINE__, len);
			return _FAIL;
		}
		memcpy(bssid->Ssid.Ssid, (p + 2), *(p + 1));
		bssid->Ssid.SsidLength = *(p + 1);
	}
	else
	{
		bssid->Ssid.SsidLength = 0;
	}

	memset(bssid->SupportedRates, 0, NDIS_802_11_LENGTH_RATES_EX);

	//checking rate info...
	i = 0;
	p = rtw_get_ie(bssid->IEs + _FIXED_IE_LENGTH_, _SUPPORTEDRATES_IE_, &len, bssid->IELength - _FIXED_IE_LENGTH_);
	if (p != NULL)
	{
		if (len > NDIS_802_11_LENGTH_RATES_EX)
		{
			DBG_871X("%s()-%d: IE too long (%d) for survey event\n", __FUNCTION__, __LINE__, len);
			return _FAIL;
		}
		memcpy(bssid->SupportedRates, (p + 2), len);
		i = len;
	}

	p = rtw_get_ie(bssid->IEs + _FIXED_IE_LENGTH_, _EXT_SUPPORTEDRATES_IE_, &len, bssid->IELength - _FIXED_IE_LENGTH_);
	if (p != NULL)
	{
		if (len > (NDIS_802_11_LENGTH_RATES_EX-i))
		{
			DBG_871X("%s()-%d: IE too long (%d) for survey event\n", __FUNCTION__, __LINE__, len);
			return _FAIL;
		}
		memcpy(bssid->SupportedRates + i, (p + 2), len);
	}

	//todo:
	{
		bssid->NetworkTypeInUse = Ndis802_11OFDM24;
	}

	if (bssid->IELength < 12)
		return _FAIL;

	// Checking for DSConfig
	p = rtw_get_ie(bssid->IEs + _FIXED_IE_LENGTH_, _DSSET_IE_, &len, bssid->IELength - _FIXED_IE_LENGTH_);

	bssid->Configuration.DSConfig = 0;
	bssid->Configuration.Length = 0;

	if (p)
	{
		bssid->Configuration.DSConfig = *(p + 2);
	}
	else
	{// In 5G, some ap do not have DSSET IE
		// checking HT info for channel
		p = rtw_get_ie(bssid->IEs + _FIXED_IE_LENGTH_, _HT_ADD_INFO_IE_, &len, bssid->IELength - _FIXED_IE_LENGTH_);
		if(p)
		{
			struct HT_info_element *HT_info = (struct HT_info_element *)(p + 2);
			bssid->Configuration.DSConfig = HT_info->primary_channel;
		}
		else
		{ // use current channel
			bssid->Configuration.DSConfig = rtw_get_oper_ch(rtlpriv);
		}
	}

	memcpy(&bssid->Configuration.BeaconPeriod, rtw_get_beacon_interval_from_ie(bssid->IEs), 2);
	bssid->Configuration.BeaconPeriod = le32_to_cpu(bssid->Configuration.BeaconPeriod);

	val16 = rtw_get_capability((WLAN_BSSID_EX *)bssid);

	if (val16 & BIT(0))
	{
		bssid->InfrastructureMode = Ndis802_11Infrastructure;
		memcpy(bssid->MacAddress, GetAddr2Ptr(pframe), ETH_ALEN);
	}
	else
	{
		bssid->InfrastructureMode = Ndis802_11IBSS;
		memcpy(bssid->MacAddress, GetAddr3Ptr(pframe), ETH_ALEN);
	}

	if (val16 & BIT(4))
		bssid->Privacy = 1;
	else
		bssid->Privacy = 0;

	bssid->Configuration.ATIMWindow = 0;

	// mark bss info receving from nearby channel as SignalQuality 101
	if(bssid->Configuration.DSConfig != rtw_get_oper_ch(rtlpriv))
	{
		bssid->PhyInfo.SignalQuality= 101;
	}

	return _SUCCESS;
}

void start_create_ibss(struct rtl_priv* rtlpriv)
{
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	unsigned short	caps;
	uint8_t	val8;
	uint8_t	join_type;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX		*pnetwork = (WLAN_BSSID_EX*)(&(pmlmeinfo->network));
	pmlmeext->cur_channel = (uint8_t)pnetwork->Configuration.DSConfig;
	mac->beacon_interval = get_beacon_interval(pnetwork);

	//update wireless mode
	update_wireless_mode(rtlpriv);

	//udpate capability
	caps = rtw_get_capability((WLAN_BSSID_EX *)pnetwork);
	update_capinfo(rtlpriv, caps);
	if(caps&cap_IBSS)//adhoc master
	{
		//set_opmode_cmd(rtlpriv, adhoc);//removed

		val8 = 0xcf;
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_SEC_CFG, (uint8_t *)(&val8));

		//switch channel
		//SelectChannel(rtlpriv, pmlmeext->cur_channel, HAL_PRIME_CHNL_OFFSET_DONT_CARE);
		set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, HAL_PRIME_CHNL_OFFSET_DONT_CARE, CHANNEL_WIDTH_20);

		beacon_timing_control(rtlpriv);

		//set msr to WIFI_FW_ADHOC_STATE
		pmlmeinfo->state = WIFI_FW_ADHOC_STATE;
		Set_MSR(rtlpriv, (pmlmeinfo->state & 0x3));

		//issue beacon
		if(send_beacon(rtlpriv)==_FAIL)
		{
			report_join_res(rtlpriv, -1);
			pmlmeinfo->state = WIFI_FW_NULL_STATE;
		}
		else
		{
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BSSID, rtlpriv->registrypriv.dev_network.MacAddress);
			join_type = 0;
			rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_JOIN, (uint8_t *)(&join_type));

			report_join_res(rtlpriv, 1);
			pmlmeinfo->state |= WIFI_FW_ASSOC_SUCCESS;
		}
	}
	else
	{
		DBG_871X("start_create_ibss, invalid cap:%x\n", caps);
		return;
	}

}

void start_clnt_join(struct rtl_priv* rtlpriv)
{
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	unsigned short	caps;
	uint8_t	val8;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX		*pnetwork = (WLAN_BSSID_EX*)(&(pmlmeinfo->network));
	int beacon_timeout;

	pmlmeext->cur_channel = (uint8_t)pnetwork->Configuration.DSConfig;
	mac->beacon_interval = get_beacon_interval(pnetwork);

	//switch channel
	set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);

	//update wireless mode
	update_wireless_mode(rtlpriv);

	//udpate capability
	caps = rtw_get_capability((WLAN_BSSID_EX *)pnetwork);
	update_capinfo(rtlpriv, caps);
	if (caps&cap_ESS)
	{
		Set_MSR(rtlpriv, WIFI_FW_STATION_STATE);

		val8 = (pmlmeinfo->auth_algo == dot11AuthAlgrthm_8021X)? 0xcc: 0xcf;

		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_SEC_CFG, (uint8_t *)(&val8));

		//switch channel
		//set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);

		//here wait for receiving the beacon to start auth
		//and enable a timer
		beacon_timeout = decide_wait_for_beacon_timeout(mac->beacon_interval);
		set_link_timer(pmlmeext, beacon_timeout);
		_set_timer( &rtlpriv->mlmepriv.assoc_timer,
			(REAUTH_TO * REAUTH_LIMIT) + (REASSOC_TO*REASSOC_LIMIT) +beacon_timeout);

		pmlmeinfo->state = WIFI_FW_AUTH_NULL | WIFI_FW_STATION_STATE;
	}
	else if (caps&cap_IBSS) //adhoc client
	{
		Set_MSR(rtlpriv, WIFI_FW_ADHOC_STATE);

		val8 = 0xcf;
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_SEC_CFG, (uint8_t *)(&val8));

		//switch channel
		set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);

		beacon_timing_control(rtlpriv);

		pmlmeinfo->state = WIFI_FW_ADHOC_STATE;

		report_join_res(rtlpriv, 1);
	}
	else
	{
		//DBG_871X("marc: invalid cap:%x\n", caps);
		return;
	}

}

void start_clnt_auth(struct rtl_priv* rtlpriv)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	del_timer_sync_ex(&pmlmeext->link_timer);

	pmlmeinfo->state &= (~WIFI_FW_AUTH_NULL);
	pmlmeinfo->state |= WIFI_FW_AUTH_STATE;

	pmlmeinfo->auth_seq = 1;
	pmlmeinfo->reauth_count = 0;
	pmlmeinfo->reassoc_count = 0;
	pmlmeinfo->link_count = 0;
	pmlmeext->retry = 0;


	// Because of AP's not receiving deauth before
	// AP may: 1)not response auth or 2)deauth us after link is complete
	// issue deauth before issuing auth to deal with the situation
	//	Commented by Albert 2012/07/21
	//	For the Win8 P2P connection, it will be hard to have a successful connection if this Wi-Fi doesn't connect to it.
	issue_deauth(rtlpriv, (&(pmlmeinfo->network))->MacAddress, WLAN_REASON_DEAUTH_LEAVING);

	DBG_871X_LEVEL(_drv_always_, "start auth\n");
	issue_auth(rtlpriv, NULL, 0);

	set_link_timer(pmlmeext, REAUTH_TO);

}


void start_clnt_assoc(struct rtl_priv* rtlpriv)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	del_timer_sync_ex(&pmlmeext->link_timer);

	pmlmeinfo->state &= (~(WIFI_FW_AUTH_NULL | WIFI_FW_AUTH_STATE));
	pmlmeinfo->state |= (WIFI_FW_AUTH_SUCCESS | WIFI_FW_ASSOC_STATE);

	issue_assocreq(rtlpriv);

	set_link_timer(pmlmeext, REASSOC_TO);
}

unsigned int receive_disconnect(struct rtl_priv *rtlpriv, unsigned char *MacAddr, unsigned short reason)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	//check A3
	if (!(_rtw_memcmp(MacAddr, get_my_bssid(&pmlmeinfo->network), ETH_ALEN)))
		return _SUCCESS;

	DBG_871X("%s\n", __FUNCTION__);

	if((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE)
	{
		if (pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS)
		{
			pmlmeinfo->state = WIFI_FW_NULL_STATE;
			report_del_sta_event(rtlpriv, MacAddr, reason);

		}
		else if (pmlmeinfo->state & WIFI_FW_LINKING_STATE)
		{
			pmlmeinfo->state = WIFI_FW_NULL_STATE;
			report_join_res(rtlpriv, -2);
		}
	}

	return _SUCCESS;
}


/****************************************************************************

Following are the functions to report events

*****************************************************************************/

void report_survey_event(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	struct cmd_obj *pcmd_obj;
	uint8_t	*pevtcmd;
	uint32_t	 cmdsz;
	struct survey_event	*psurvey_evt;
	struct C2HEvent_Header *pc2h_evt_hdr;
	struct mlme_ext_priv *pmlmeext;
	struct cmd_priv *pcmdpriv;
	//uint8_t *pframe = precv_frame->u.hdr.rx_data;
	//uint len = precv_frame->u.hdr.len;

	if(!rtlpriv)
		return;

	pmlmeext = &rtlpriv->mlmeextpriv;
	pcmdpriv = &rtlpriv->cmdpriv;


	if ((pcmd_obj = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj))) == NULL)
	{
		return;
	}

	cmdsz = (sizeof(struct survey_event) + sizeof(struct C2HEvent_Header));
	if ((pevtcmd = (uint8_t *)rtw_zmalloc(cmdsz)) == NULL)
	{
		rtw_mfree(pcmd_obj);
		return;
	}

	INIT_LIST_HEAD(&pcmd_obj->list);

	pcmd_obj->cmdcode = GEN_CMD_CODE(_Set_MLME_EVT);
	pcmd_obj->cmdsz = cmdsz;
	pcmd_obj->parmbuf = pevtcmd;

	pcmd_obj->rsp = NULL;
	pcmd_obj->rspsz  = 0;

	pc2h_evt_hdr = (struct C2HEvent_Header*)(pevtcmd);
	pc2h_evt_hdr->len = sizeof(struct survey_event);
	pc2h_evt_hdr->ID = GEN_EVT_CODE(_Survey);
	pc2h_evt_hdr->seq = atomic_inc_return(&pmlmeext->event_seq);

	psurvey_evt = (struct survey_event*)(pevtcmd + sizeof(struct C2HEvent_Header));

	if (collect_bss_info(rtlpriv, precv_frame, (WLAN_BSSID_EX *)&psurvey_evt->bss) == _FAIL)
	{
		/* ULLI check usage of cmdsz */

		rtw_mfree(pcmd_obj);
		rtw_mfree(pevtcmd);
		return;
	}

	rtw_enqueue_cmd(pcmdpriv, pcmd_obj);

	pmlmeext->sitesurvey_res.bss_cnt++;

	return;

}

void report_surveydone_event(struct rtl_priv *rtlpriv)
{
	struct cmd_obj *pcmd_obj;
	uint8_t	*pevtcmd;
	uint32_t	 cmdsz;
	struct surveydone_event *psurveydone_evt;
	struct C2HEvent_Header	*pc2h_evt_hdr;
	struct mlme_ext_priv		*pmlmeext = &rtlpriv->mlmeextpriv;
	struct cmd_priv *pcmdpriv = &rtlpriv->cmdpriv;

	if ((pcmd_obj = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj))) == NULL)
	{
		return;
	}

	cmdsz = (sizeof(struct surveydone_event) + sizeof(struct C2HEvent_Header));
	if ((pevtcmd = (uint8_t *)rtw_zmalloc(cmdsz)) == NULL)
	{
		rtw_mfree(pcmd_obj);
		return;
	}

	INIT_LIST_HEAD(&pcmd_obj->list);

	pcmd_obj->cmdcode = GEN_CMD_CODE(_Set_MLME_EVT);
	pcmd_obj->cmdsz = cmdsz;
	pcmd_obj->parmbuf = pevtcmd;

	pcmd_obj->rsp = NULL;
	pcmd_obj->rspsz  = 0;

	pc2h_evt_hdr = (struct C2HEvent_Header*)(pevtcmd);
	pc2h_evt_hdr->len = sizeof(struct surveydone_event);
	pc2h_evt_hdr->ID = GEN_EVT_CODE(_SurveyDone);
	pc2h_evt_hdr->seq = atomic_inc_return(&pmlmeext->event_seq);

	psurveydone_evt = (struct surveydone_event*)(pevtcmd + sizeof(struct C2HEvent_Header));
	psurveydone_evt->bss_cnt = pmlmeext->sitesurvey_res.bss_cnt;

	DBG_871X("survey done event(%x)\n", psurveydone_evt->bss_cnt);

	rtw_enqueue_cmd(pcmdpriv, pcmd_obj);

	return;

}

void report_join_res(struct rtl_priv *rtlpriv, int res)
{
	struct cmd_obj *pcmd_obj;
	uint8_t	*pevtcmd;
	uint32_t	 cmdsz;
	struct joinbss_event		*pjoinbss_evt;
	struct C2HEvent_Header	*pc2h_evt_hdr;
	struct mlme_ext_priv		*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct cmd_priv *pcmdpriv = &rtlpriv->cmdpriv;

	if ((pcmd_obj = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj))) == NULL)
	{
		return;
	}

	cmdsz = (sizeof(struct joinbss_event) + sizeof(struct C2HEvent_Header));
	if ((pevtcmd = (uint8_t *)rtw_zmalloc(cmdsz)) == NULL)
	{
		rtw_mfree(pcmd_obj);
		return;
	}

	INIT_LIST_HEAD(&pcmd_obj->list);

	pcmd_obj->cmdcode = GEN_CMD_CODE(_Set_MLME_EVT);
	pcmd_obj->cmdsz = cmdsz;
	pcmd_obj->parmbuf = pevtcmd;

	pcmd_obj->rsp = NULL;
	pcmd_obj->rspsz  = 0;

	pc2h_evt_hdr = (struct C2HEvent_Header*)(pevtcmd);
	pc2h_evt_hdr->len = sizeof(struct joinbss_event);
	pc2h_evt_hdr->ID = GEN_EVT_CODE(_JoinBss);
	pc2h_evt_hdr->seq = atomic_inc_return(&pmlmeext->event_seq);

	pjoinbss_evt = (struct joinbss_event*)(pevtcmd + sizeof(struct C2HEvent_Header));
	memcpy((unsigned char *)(&(pjoinbss_evt->network.network)), &(pmlmeinfo->network), sizeof(WLAN_BSSID_EX));
	pjoinbss_evt->network.join_res 	= pjoinbss_evt->network.aid = res;

	DBG_871X("report_join_res(%d)\n", res);


	rtw_joinbss_event_prehandle(rtlpriv, (uint8_t *)&pjoinbss_evt->network);


	rtw_enqueue_cmd(pcmdpriv, pcmd_obj);

	return;

}

void report_del_sta_event(struct rtl_priv *rtlpriv, unsigned char* MacAddr, unsigned short reason)
{
	struct cmd_obj *pcmd_obj;
	uint8_t	*pevtcmd;
	uint32_t	 cmdsz;
	struct sta_info *psta;
	int	mac_id;
	struct stadel_event			*pdel_sta_evt;
	struct C2HEvent_Header	*pc2h_evt_hdr;
	struct mlme_ext_priv		*pmlmeext = &rtlpriv->mlmeextpriv;
	struct cmd_priv *pcmdpriv = &rtlpriv->cmdpriv;

	if ((pcmd_obj = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj))) == NULL)
	{
		return;
	}

	cmdsz = (sizeof(struct stadel_event) + sizeof(struct C2HEvent_Header));
	if ((pevtcmd = (uint8_t *)rtw_zmalloc(cmdsz)) == NULL)
	{
		rtw_mfree(pcmd_obj);
		return;
	}

	INIT_LIST_HEAD(&pcmd_obj->list);

	pcmd_obj->cmdcode = GEN_CMD_CODE(_Set_MLME_EVT);
	pcmd_obj->cmdsz = cmdsz;
	pcmd_obj->parmbuf = pevtcmd;

	pcmd_obj->rsp = NULL;
	pcmd_obj->rspsz  = 0;

	pc2h_evt_hdr = (struct C2HEvent_Header*)(pevtcmd);
	pc2h_evt_hdr->len = sizeof(struct stadel_event);
	pc2h_evt_hdr->ID = GEN_EVT_CODE(_DelSTA);
	pc2h_evt_hdr->seq = atomic_inc_return(&pmlmeext->event_seq);

	pdel_sta_evt = (struct stadel_event*)(pevtcmd + sizeof(struct C2HEvent_Header));
	memcpy((unsigned char *)(&(pdel_sta_evt->macaddr)), MacAddr, ETH_ALEN);
	memcpy((unsigned char *)(pdel_sta_evt->rsvd),(unsigned char *)(&reason),2);


	psta = rtw_get_stainfo(&rtlpriv->stapriv, MacAddr);
	if(psta)
		mac_id = (int)psta->mac_id;
	else
		mac_id = (-1);

	pdel_sta_evt->mac_id = mac_id;

	DBG_871X("report_del_sta_event: delete STA, mac_id=%d\n", mac_id);

	rtw_enqueue_cmd(pcmdpriv, pcmd_obj);

	return;
}

void report_add_sta_event(struct rtl_priv *rtlpriv, unsigned char* MacAddr, int cam_idx)
{
	struct cmd_obj *pcmd_obj;
	uint8_t	*pevtcmd;
	uint32_t	 cmdsz;
	struct stassoc_event		*padd_sta_evt;
	struct C2HEvent_Header	*pc2h_evt_hdr;
	struct mlme_ext_priv		*pmlmeext = &rtlpriv->mlmeextpriv;
	struct cmd_priv *pcmdpriv = &rtlpriv->cmdpriv;

	if ((pcmd_obj = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj))) == NULL)
	{
		return;
	}

	cmdsz = (sizeof(struct stassoc_event) + sizeof(struct C2HEvent_Header));
	if ((pevtcmd = (uint8_t *)rtw_zmalloc(cmdsz)) == NULL)
	{
		rtw_mfree(pcmd_obj);
		return;
	}

	INIT_LIST_HEAD(&pcmd_obj->list);

	pcmd_obj->cmdcode = GEN_CMD_CODE(_Set_MLME_EVT);
	pcmd_obj->cmdsz = cmdsz;
	pcmd_obj->parmbuf = pevtcmd;

	pcmd_obj->rsp = NULL;
	pcmd_obj->rspsz  = 0;

	pc2h_evt_hdr = (struct C2HEvent_Header*)(pevtcmd);
	pc2h_evt_hdr->len = sizeof(struct stassoc_event);
	pc2h_evt_hdr->ID = GEN_EVT_CODE(_AddSTA);
	pc2h_evt_hdr->seq = atomic_inc_return(&pmlmeext->event_seq);

	padd_sta_evt = (struct stassoc_event*)(pevtcmd + sizeof(struct C2HEvent_Header));
	memcpy((unsigned char *)(&(padd_sta_evt->macaddr)), MacAddr, ETH_ALEN);
	padd_sta_evt->cam_id = cam_idx;

	DBG_871X("report_add_sta_event: add STA\n");

	rtw_enqueue_cmd(pcmdpriv, pcmd_obj);

	return;
}


/****************************************************************************

Following are the event callback functions

*****************************************************************************/

//for sta/adhoc mode
void update_sta_info(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	struct mlme_priv *pmlmepriv = &(rtlpriv->mlmepriv);
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	//ERP
	VCS_update(rtlpriv, psta);

	//HT
	if(pmlmepriv->htpriv.ht_option)
	{
		psta->htpriv.ht_option = true;

		psta->htpriv.ampdu_enable = pmlmepriv->htpriv.ampdu_enable;

		if (support_short_GI(rtlpriv, &(pmlmeinfo->HT_caps)))
			psta->htpriv.sgi = true;

		psta->qos_option = true;

	}
	else
	{
		psta->htpriv.ht_option = false;

		psta->htpriv.ampdu_enable = false;

		psta->htpriv.sgi = false;
		psta->qos_option = false;

	}
	psta->htpriv.bwmode = pmlmeext->cur_bwmode;
	psta->htpriv.ch_offset = pmlmeext->cur_ch_offset;

	psta->htpriv.agg_enable_bitmap = 0x0;//reset
	psta->htpriv.candidate_tid_bitmap = 0x0;//reset

	//QoS
	if(pmlmepriv->qospriv.qos_option)
		psta->qos_option = true;

	if (pmlmepriv->vhtpriv.vht_bwmode < CHANNEL_WIDTH_80)
		pmlmepriv->vhtpriv.sgi = psta->htpriv.sgi;
	memcpy(&psta->vhtpriv, &pmlmepriv->vhtpriv, sizeof(struct vht_priv));

	spin_lock_bh(&psta->lock);
	psta->state = _FW_LINKED;
	spin_unlock_bh(&psta->lock);

}

void mlmeext_joinbss_event_callback(struct rtl_priv *rtlpriv, int join_res)
{
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	struct sta_info		*psta, *psta_bmc;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);
	struct sta_priv		*pstapriv = &rtlpriv->stapriv;
	uint8_t	join_type;
	u16 media_status;

	if(join_res < 0)
	{
		join_type = 1;
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_JOIN, (uint8_t *)(&join_type));
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BSSID, null_addr);

		goto exit_mlmeext_joinbss_event_callback;
	}

	if((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)
	{
		//for bc/mc
		psta_bmc = rtw_get_bcmc_stainfo(rtlpriv);
		if(psta_bmc)
		{
			pmlmeinfo->FW_sta_info[psta_bmc->mac_id].psta = psta_bmc;
			update_bmc_sta_support_rate(rtlpriv, psta_bmc->mac_id);
			Update_RA_Entry(rtlpriv, psta_bmc);
		}
	}

	// update IOT-releated issue
	update_IOT_info(rtlpriv);

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BASIC_RATE, cur_network->SupportedRates);

	//BCN interval
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BEACON_INTERVAL, (uint8_t *)(&mac->beacon_interval));

	//udpate capability
	update_capinfo(rtlpriv, pmlmeinfo->capability);

	//WMM, Update EDCA param
	WMMOnAssocRsp(rtlpriv);

	//HT
	HTOnAssocRsp(rtlpriv);

	//VHT
	VHTOnAssocRsp(rtlpriv);

	psta = rtw_get_stainfo(pstapriv, cur_network->MacAddress);
	if (psta) //only for infra. mode
	{
		pmlmeinfo->FW_sta_info[psta->mac_id].psta = psta;

		//DBG_871X("set_sta_rate\n");

		psta->wireless_mode = pmlmeext->cur_wireless_mode;

		//set per sta rate after updating HT cap.
		set_sta_rate(rtlpriv, psta);

		#if (RATE_ADAPTIVE_SUPPORT==1)	//for 88E RA
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv,HW_VAR_TX_RPT_MAX_MACID, (uint8_t *)&psta->mac_id);
		#endif
		media_status = (psta->mac_id<<8)|1; //  MACID|OPMODE: 1 means connect
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv,HW_VAR_H2C_MEDIA_STATUS_RPT,(uint8_t *)&media_status);
	}

	join_type = 2;
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_JOIN, (uint8_t *)(&join_type));

	if((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE)
	{
		// correcting TSF
		correct_TSF(rtlpriv, pmlmeext);

		//set_link_timer(pmlmeext, DISCONNECT_TO);
	}

		rtw_lps_ctrl_wk_cmd(rtlpriv, LPS_CTRL_CONNECT, 0);

exit_mlmeext_joinbss_event_callback:


	DBG_871X("=>%s\n", __FUNCTION__);

}

//currently only adhoc mode will go here
void mlmeext_sta_add_event_callback(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t	join_type;

	DBG_871X("%s\n", __FUNCTION__);

	if((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE)
	{
		if(pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS)//adhoc master or sta_count>1
		{
			//nothing to do
		}
		else//adhoc client
		{
			//update TSF Value
			//update_TSF(pmlmeext, pframe, len);

			// correcting TSF
			correct_TSF(rtlpriv, pmlmeext);

			//start beacon
			if(send_beacon(rtlpriv)==_FAIL)
			{
				pmlmeinfo->FW_sta_info[psta->mac_id].status = 0;

				pmlmeinfo->state ^= WIFI_FW_ADHOC_STATE;

				return;
			}

			pmlmeinfo->state |= WIFI_FW_ASSOC_SUCCESS;

		}

		join_type = 2;
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_JOIN, (uint8_t *)(&join_type));
	}

	pmlmeinfo->FW_sta_info[psta->mac_id].psta = psta;

	//rate radaptive
	Update_RA_Entry(rtlpriv, psta);

	//update adhoc sta_info
	update_sta_info(rtlpriv, psta);

}

void mlmeext_sta_del_event_callback(struct rtl_priv *rtlpriv)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	if (is_client_associated_to_ap(rtlpriv) || is_IBSS_empty(rtlpriv))
	{
		//set_opmode_cmd(rtlpriv, infra_client_with_mlme);

		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_DISCONNECT, 0);
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BSSID, null_addr);


		//switch to the 20M Hz mode after disconnect
		pmlmeext->cur_bwmode = CHANNEL_WIDTH_20;
		pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;

		//SelectChannel(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset);
		set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);


		flush_all_cam_entry(rtlpriv);

		pmlmeinfo->state = WIFI_FW_NULL_STATE;

		//set MSR to no link state -> infra. mode
		Set_MSR(rtlpriv, _HW_STATE_STATION_);

		del_timer_sync_ex(&pmlmeext->link_timer);

	}

}

/****************************************************************************

Following are the functions for the timer handlers

*****************************************************************************/
void _linked_info_dump(struct rtl_priv *rtlpriv);
void _linked_info_dump(struct rtl_priv *rtlpriv)
{
	struct mlme_ext_priv    *pmlmeext = &rtlpriv->mlmeextpriv;
      struct mlme_ext_info    *pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t mac_id;
	int UndecoratedSmoothedPWDB;

	if((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE)
	{
		mac_id=0;
	}
	else if((pmlmeinfo->state&0x03) == _HW_STATE_AP_)
	{
		mac_id=2;
	}
}

uint8_t chk_ap_is_alive(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	uint8_t ret = false;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	#ifdef DBG_EXPIRATION_CHK
	DBG_871X(FUNC_ADPT_FMT" rx:"STA_PKTS_FMT", beacon:%llu, probersp_to_self:%llu"
				/*", probersp_bm:%llu, probersp_uo:%llu, probereq:%llu, BI:%u"*/
				", retry:%u\n"
		, FUNC_ADPT_ARG(rtlpriv)
		, STA_RX_PKTS_DIFF_ARG(psta)
		, psta->sta_stats.rx_beacon_pkts - psta->sta_stats.last_rx_beacon_pkts
		, psta->sta_stats.rx_probersp_pkts - psta->sta_stats.last_rx_probersp_pkts
		/*, psta->sta_stats.rx_probersp_bm_pkts - psta->sta_stats.last_rx_probersp_bm_pkts
		, psta->sta_stats.rx_probersp_uo_pkts - psta->sta_stats.last_rx_probersp_uo_pkts
		, psta->sta_stats.rx_probereq_pkts - psta->sta_stats.last_rx_probereq_pkts
		, pmlmeinfo->bcn_interval*/
		, pmlmeext->retry
	);

	DBG_871X(FUNC_ADPT_FMT" tx_pkts:%llu, link_count:%u\n", FUNC_ADPT_ARG(rtlpriv)
		, rtlpriv->xmitpriv.tx_pkts
		, pmlmeinfo->link_count
	);
	#endif

	if((sta_rx_data_pkts(psta) == sta_last_rx_data_pkts(psta))
		&& sta_rx_beacon_pkts(psta) == sta_last_rx_beacon_pkts(psta)
		&& sta_rx_probersp_pkts(psta) == sta_last_rx_probersp_pkts(psta)
	)
	{
		ret = false;
	}
	else
	{
		ret = true;
	}

	sta_update_last_rx_pkts(psta);

	return ret;
}

void linked_status_chk(struct rtl_priv *rtlpriv)
{
	uint32_t	i;
	struct sta_info		*psta;
	struct xmit_priv		*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct sta_priv		*pstapriv = &rtlpriv->stapriv;

	 _linked_info_dump(rtlpriv);

	if (is_client_associated_to_ap(rtlpriv))
	{
		//linked infrastructure client mode

		int tx_chk = _SUCCESS, rx_chk = _SUCCESS;
		int rx_chk_limit;

		#if defined(DBG_ROAMING_TEST)
		rx_chk_limit = 1;
		#else
		rx_chk_limit = 8;
		#endif

		if ((psta = rtw_get_stainfo(pstapriv, pmlmeinfo->network.MacAddress)) != NULL)
		{
			bool is_p2p_enable = false;

			if (chk_ap_is_alive(rtlpriv, psta) == false)
				rx_chk = _FAIL;

			if (pxmitpriv->last_tx_pkts == pxmitpriv->tx_pkts)
				tx_chk = _FAIL;

			{
				if (rx_chk != _SUCCESS) {
					if (pmlmeext->retry == 0) {
						#ifdef DBG_EXPIRATION_CHK
						DBG_871X("issue_probereq to trigger probersp, retry=%d\n", pmlmeext->retry);
						#endif
						issue_probereq(rtlpriv, &pmlmeinfo->network.Ssid, pmlmeinfo->network.MacAddress);
						issue_probereq(rtlpriv, &pmlmeinfo->network.Ssid, pmlmeinfo->network.MacAddress);
						issue_probereq(rtlpriv, &pmlmeinfo->network.Ssid, pmlmeinfo->network.MacAddress);
					}
				}

				if (tx_chk != _SUCCESS && pmlmeinfo->link_count++ == 0xf) {
					#ifdef DBG_EXPIRATION_CHK
					DBG_871X("%s issue_nulldata 0\n", __FUNCTION__);
					#endif
					tx_chk = issue_nulldata(rtlpriv, NULL, 0, 1, 0);
				}
			}

			if (rx_chk == _FAIL) {
				pmlmeext->retry++;
				if (pmlmeext->retry > rx_chk_limit) {
					DBG_871X_LEVEL(_drv_always_, FUNC_ADPT_FMT" disconnect or roaming\n",
						FUNC_ADPT_ARG(rtlpriv));
					receive_disconnect(rtlpriv, pmlmeinfo->network.MacAddress
						, WLAN_REASON_EXPIRATION_CHK);
					return;
				}
			} else {
				pmlmeext->retry = 0;
			}

			if (tx_chk == _FAIL) {
				pmlmeinfo->link_count &= 0xf;
			} else {
				pxmitpriv->last_tx_pkts = pxmitpriv->tx_pkts;
				pmlmeinfo->link_count = 0;
			}

		} //end of if ((psta = rtw_get_stainfo(pstapriv, passoc_res->network.MacAddress)) != NULL)
	}
	else if (is_client_associated_to_ibss(rtlpriv))
	{
		//linked IBSS mode
		//for each assoc list entry to check the rx pkt counter
		for (i = IBSS_START_MAC_ID; i < NUM_STA; i++)
		{
			if (pmlmeinfo->FW_sta_info[i].status == 1)
			{
				psta = pmlmeinfo->FW_sta_info[i].psta;

				if(NULL==psta) continue;

				if (pmlmeinfo->FW_sta_info[i].rx_pkt == sta_rx_pkts(psta))
				{

					if(pmlmeinfo->FW_sta_info[i].retry<3)
					{
						pmlmeinfo->FW_sta_info[i].retry++;
					}
					else
					{
						pmlmeinfo->FW_sta_info[i].retry = 0;
						pmlmeinfo->FW_sta_info[i].status = 0;
						report_del_sta_event(rtlpriv, psta->hwaddr
							, 65535// indicate disconnect caused by no rx
						);
					}
				}
				else
				{
					pmlmeinfo->FW_sta_info[i].retry = 0;
					pmlmeinfo->FW_sta_info[i].rx_pkt = (uint32_t)sta_rx_pkts(psta);
				}
			}
		}

		//set_link_timer(pmlmeext, DISCONNECT_TO);

	}

}

void survey_timer_hdl(struct rtl_priv *rtlpriv)
{
	struct cmd_obj	*ph2c;
	struct sitesurvey_parm	*psurveyPara;
	struct cmd_priv					*pcmdpriv=&rtlpriv->cmdpriv;
	struct mlme_ext_priv 		*pmlmeext = &rtlpriv->mlmeextpriv;

	//DBG_871X("marc: survey timer\n");

	//issue rtw_sitesurvey_cmd
	if (pmlmeext->sitesurvey_res.state > SCAN_START)
	{
		if(pmlmeext->sitesurvey_res.state ==  SCAN_PROCESS)
		{
				pmlmeext->sitesurvey_res.channel_idx++;
		}

		if(pmlmeext->scan_abort == true)
		{
			{
				pmlmeext->sitesurvey_res.channel_idx = pmlmeext->sitesurvey_res.ch_num;
				DBG_871X("%s idx:%d\n", __FUNCTION__
					, pmlmeext->sitesurvey_res.channel_idx
				);
			}

			pmlmeext->scan_abort = false;//reset
		}

		if ((ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj))) == NULL)
		{
			goto exit_survey_timer_hdl;
		}

		if ((psurveyPara = (struct sitesurvey_parm*)rtw_zmalloc(sizeof(struct sitesurvey_parm))) == NULL)
		{
			rtw_mfree(ph2c);
			goto exit_survey_timer_hdl;
		}

		init_h2fwcmd_w_parm_no_rsp(ph2c, psurveyPara, GEN_CMD_CODE(_SiteSurvey));
		rtw_enqueue_cmd(pcmdpriv, ph2c);
	}


exit_survey_timer_hdl:

	return;
}

void link_timer_hdl(struct rtl_priv *rtlpriv)
{
	//static unsigned int		rx_pkt = 0;
	//static u64				tx_cnt = 0;
	//struct xmit_priv		*pxmitpriv = &(rtlpriv->xmitpriv);
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	//struct sta_priv		*pstapriv = &rtlpriv->stapriv;


	if (pmlmeinfo->state & WIFI_FW_AUTH_NULL)
	{
		DBG_871X("link_timer_hdl:no beacon while connecting\n");
		pmlmeinfo->state = WIFI_FW_NULL_STATE;
		report_join_res(rtlpriv, -3);
	}
	else if (pmlmeinfo->state & WIFI_FW_AUTH_STATE)
	{
		//re-auth timer
		if (++pmlmeinfo->reauth_count > REAUTH_LIMIT)
		{
			//if (pmlmeinfo->auth_algo != dot11AuthAlgrthm_Auto)
			//{
				pmlmeinfo->state = 0;
				report_join_res(rtlpriv, -1);
				return;
			//}
			//else
			//{
			//	pmlmeinfo->auth_algo = dot11AuthAlgrthm_Shared;
			//	pmlmeinfo->reauth_count = 0;
			//}
		}

		DBG_871X("link_timer_hdl: auth timeout and try again\n");
		pmlmeinfo->auth_seq = 1;
		issue_auth(rtlpriv, NULL, 0);
		set_link_timer(pmlmeext, REAUTH_TO);
	}
	else if (pmlmeinfo->state & WIFI_FW_ASSOC_STATE)
	{
		//re-assoc timer
		if (++pmlmeinfo->reassoc_count > REASSOC_LIMIT)
		{
			pmlmeinfo->state = WIFI_FW_NULL_STATE;
			report_join_res(rtlpriv, -2);
			return;
		}

		DBG_871X("link_timer_hdl: assoc timeout and try again\n");
		issue_assocreq(rtlpriv);
		set_link_timer(pmlmeext, REASSOC_TO);
	}

	return;
}

void addba_timer_hdl(struct sta_info *psta)
{
	struct ht_priv	*phtpriv;

	if(!psta)
		return;

	phtpriv = &psta->htpriv;

	if((phtpriv->ht_option==true) && (phtpriv->ampdu_enable==true))
	{
		if(phtpriv->candidate_tid_bitmap)
			phtpriv->candidate_tid_bitmap=0x0;

	}
}

uint8_t NULL_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	return H2C_SUCCESS;
}


uint8_t setopmode_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	uint8_t	type;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct setopmode_parm *psetop = (struct setopmode_parm *)pbuf;

	if(psetop->mode == Ndis802_11APMode)
	{
		pmlmeinfo->state = WIFI_FW_AP_STATE;
		type = _HW_STATE_AP_;
	}
	else if(psetop->mode == Ndis802_11Infrastructure)
	{
		pmlmeinfo->state &= ~(BIT(0)|BIT(1));// clear state
		pmlmeinfo->state |= WIFI_FW_STATION_STATE;//set to 	STATION_STATE
		type = _HW_STATE_STATION_;
	}
	else if(psetop->mode == Ndis802_11IBSS)
	{
		type = _HW_STATE_ADHOC_;
	}
	else
	{
		type = _HW_STATE_NOLINK_;
	}

	rtlpriv->cfg->ops->set_network_type(rtlpriv, type);
	//Set_NETYPE0_MSR(rtlpriv, type);

	return H2C_SUCCESS;

}

uint8_t createbss_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX	*pnetwork = (WLAN_BSSID_EX*)(&(pmlmeinfo->network));
	struct joinbss_parm *pparm = (struct joinbss_parm *)pbuf;
	//uint32_t	initialgain;


	if(pparm->network.InfrastructureMode == Ndis802_11APMode)
	{
#ifdef CONFIG_AP_MODE

		if(pmlmeinfo->state == WIFI_FW_AP_STATE)
		{
			//todo:
			return H2C_SUCCESS;
		}
#endif
	}

	//below is for ad-hoc master
	if(pparm->network.InfrastructureMode == Ndis802_11IBSS)
	{
		rtw_joinbss_reset(rtlpriv);

		pmlmeext->cur_bwmode = CHANNEL_WIDTH_20;
		pmlmeext->cur_ch_offset= HAL_PRIME_CHNL_OFFSET_DONT_CARE;
		pmlmeinfo->ERP_enable = 0;
		pmlmeinfo->WMM_enable = 0;
		pmlmeinfo->HT_enable = 0;
		pmlmeinfo->HT_caps_enable = 0;
		pmlmeinfo->HT_info_enable = 0;
		pmlmeinfo->agg_enable_bitmap = 0;
		pmlmeinfo->candidate_tid_bitmap = 0;

		//config the initial gain under linking, need to write the BB registers
		//initialgain = 0x1E;
		//rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_INITIAL_GAIN, (uint8_t *)(&initialgain));

		//cancel link timer
		del_timer_sync_ex(&pmlmeext->link_timer);

		//clear CAM
		flush_all_cam_entry(rtlpriv);

		memcpy(pnetwork, pbuf, FIELD_OFFSET(WLAN_BSSID_EX, IELength));
		pnetwork->IELength = ((WLAN_BSSID_EX *)pbuf)->IELength;

		if(pnetwork->IELength>MAX_IE_SZ)//Check pbuf->IELength
			return H2C_PARAMETERS_ERROR;

		memcpy(pnetwork->IEs, ((WLAN_BSSID_EX *)pbuf)->IEs, pnetwork->IELength);

		start_create_ibss(rtlpriv);

	}

	return H2C_SUCCESS;

}

uint8_t join_cmd_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	uint8_t	join_type;
	PNDIS_802_11_VARIABLE_IEs	pIE;
	struct registry_priv	*pregpriv = &rtlpriv->registrypriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX		*pnetwork = (WLAN_BSSID_EX*)(&(pmlmeinfo->network));
	uint32_t	 i;
	uint8_t	cbw40_enable=0;
        //uint32_t	initialgain;
	//uint32_t	acparm;

	//check already connecting to AP or not
	if (pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS)
	{
		if (pmlmeinfo->state & WIFI_FW_STATION_STATE)
		{
			issue_deauth_ex(rtlpriv, pnetwork->MacAddress, WLAN_REASON_DEAUTH_LEAVING, 5, 100);
		}

		pmlmeinfo->state = WIFI_FW_NULL_STATE;

		//clear CAM
		flush_all_cam_entry(rtlpriv);

		del_timer_sync_ex(&pmlmeext->link_timer);

		//set MSR to nolink -> infra. mode
		//Set_MSR(rtlpriv, _HW_STATE_NOLINK_);
		Set_MSR(rtlpriv, _HW_STATE_STATION_);


		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_DISCONNECT, 0);
	}

	rtw_joinbss_reset(rtlpriv);

	pmlmeext->cur_bwmode = CHANNEL_WIDTH_20;
	pmlmeext->cur_ch_offset= HAL_PRIME_CHNL_OFFSET_DONT_CARE;
	pmlmeinfo->ERP_enable = 0;
	pmlmeinfo->WMM_enable = 0;
	pmlmeinfo->HT_enable = 0;
	pmlmeinfo->HT_caps_enable = 0;
	pmlmeinfo->HT_info_enable = 0;
	pmlmeinfo->agg_enable_bitmap = 0;
	pmlmeinfo->candidate_tid_bitmap = 0;
	pmlmeinfo->bwmode_updated = false;
	//pmlmeinfo->assoc_AP_vendor = HT_IOT_PEER_MAX;
	pmlmeinfo->VHT_enable = 0;

	memcpy(pnetwork, pbuf, FIELD_OFFSET(WLAN_BSSID_EX, IELength));
	pnetwork->IELength = ((WLAN_BSSID_EX *)pbuf)->IELength;

	if(pnetwork->IELength>MAX_IE_SZ)//Check pbuf->IELength
		return H2C_PARAMETERS_ERROR;

	memcpy(pnetwork->IEs, ((WLAN_BSSID_EX *)pbuf)->IEs, pnetwork->IELength);

	//Check AP vendor to move rtw_joinbss_cmd()
	//pmlmeinfo->assoc_AP_vendor = check_assoc_AP(pnetwork->IEs, pnetwork->IELength);

	//sizeof(NDIS_802_11_FIXED_IEs)
	for (i = _FIXED_IE_LENGTH_; i < pnetwork->IELength;)
	{
		pIE = (PNDIS_802_11_VARIABLE_IEs)(pnetwork->IEs + i);

		switch (pIE->ElementID)
		{
			case _VENDOR_SPECIFIC_IE_://Get WMM IE.
				if ( _rtw_memcmp(pIE->data, WMM_OUI, 4) )
				{
					WMM_param_handler(rtlpriv, pIE);
				}
				break;

			case _HT_CAPABILITY_IE_:	//Get HT Cap IE.
				pmlmeinfo->HT_caps_enable = 1;
				break;

			case _HT_EXTRA_INFO_IE_:	//Get HT Info IE.
				pmlmeinfo->HT_info_enable = 1;

				//spec case only for cisco's ap because cisco's ap issue assoc rsp using mcs rate @40MHz or @20MHz
				{
					struct HT_info_element *pht_info = (struct HT_info_element *)(pIE->data);

					if (pnetwork->Configuration.DSConfig > 14) {
						if ((0x21 & 0xf0) > CHANNEL_WIDTH_20)
							cbw40_enable = 1;
					} else {
						if ((0x21 & 0x0f) > CHANNEL_WIDTH_20)
							cbw40_enable = 1;
					}

					if ((cbw40_enable) && (pht_info->infos[0] & BIT(2)))
					{
						//switch to the 40M Hz mode according to the AP
						pmlmeext->cur_bwmode = CHANNEL_WIDTH_40;
						switch (pht_info->infos[0] & 0x3)
						{
							case 1:
								pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_LOWER;
								break;

							case 3:
								pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_UPPER;
								break;

							default:
								pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
								break;
						}

						DBG_871X("set HT ch/bw before connected\n");
					}
				}
				break;
			case EID_VHTCapability://Get VHT Cap IE.
				pmlmeinfo->VHT_enable = 1;
				break;

			case EID_VHTOperation://Get VHT Operation IE.
				if (pmlmeext->cur_bwmode == CHANNEL_WIDTH_40) {
					if((GET_VHT_OPERATION_ELE_CHL_WIDTH(pIE->data) >= 1)
						&& ((0x21 & 0xf0) >= CHANNEL_WIDTH_80)) {
						pmlmeext->cur_bwmode = CHANNEL_WIDTH_80;
						DBG_871X("VHT center ch = %d\n", GET_VHT_OPERATION_ELE_CENTER_FREQ1(pIE->data));
						DBG_871X("set VHT ch/bw before connected\n");
					}
				}
				break;
			default:
				break;
		}

		i += (pIE->Length + 2);
	}
	//disable dynamic functions, such as high power, DIG
	//Switch_DM_Func(rtlpriv, DYNAMIC_FUNC_DISABLE, false);

	//config the initial gain under linking, need to write the BB registers
	//initialgain = 0x1E;
	//rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_INITIAL_GAIN, (uint8_t *)(&initialgain));

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BSSID, pmlmeinfo->network.MacAddress);
	join_type = 0;
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_JOIN, (uint8_t *)(&join_type));

	//cancel link timer
	del_timer_sync_ex(&pmlmeext->link_timer);

	start_clnt_join(rtlpriv);

	return H2C_SUCCESS;

}

uint8_t disconnect_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf)
{
	struct disconnect_parm *param = (struct disconnect_parm *)pbuf;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX		*pnetwork = (WLAN_BSSID_EX*)(&(pmlmeinfo->network));
	uint8_t	val8;

	if (is_client_associated_to_ap(rtlpriv))
	{
		issue_deauth_ex(rtlpriv, pnetwork->MacAddress, WLAN_REASON_DEAUTH_LEAVING, param->deauth_timeout_ms/100, 100);
	}

	//set_opmode_cmd(rtlpriv, infra_client_with_mlme);

	//pmlmeinfo->state = WIFI_FW_NULL_STATE;


	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_DISCONNECT, 0);
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BSSID, null_addr);

	if(((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
	{
		//Stop BCN
		val8 = 0;
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BCN_FUNC, (uint8_t *)(&val8));
	}


	//set MSR to no link state -> infra. mode
	Set_MSR(rtlpriv, _HW_STATE_STATION_);

	pmlmeinfo->state = WIFI_FW_NULL_STATE;

		//switch to the 20M Hz mode after disconnect
		pmlmeext->cur_bwmode = CHANNEL_WIDTH_20;
		pmlmeext->cur_ch_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;

		set_channel_bwmode(rtlpriv, pmlmeext->cur_channel, pmlmeext->cur_ch_offset, pmlmeext->cur_bwmode);


	flush_all_cam_entry(rtlpriv);

	del_timer_sync_ex(&pmlmeext->link_timer);

	rtw_free_uc_swdec_pending_queue(rtlpriv);

	return 	H2C_SUCCESS;
}

int rtw_scan_ch_decision(struct rtl_priv *rtlpriv, struct rtw_ieee80211_channel *out,
	uint32_t	 out_num, struct rtw_ieee80211_channel *in, uint32_t	 in_num)
{
	int i, j;
	int scan_ch_num = 0;
	int set_idx;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;

	/* clear out first */
	memset(out, 0, sizeof(struct rtw_ieee80211_channel)*out_num);

	/* acquire channels from in */
	j = 0;
	for (i=0;i<in_num;i++) {
		if (0)
		DBG_871X(FUNC_ADPT_FMT" "CHAN_FMT"\n", FUNC_ADPT_ARG(rtlpriv), CHAN_ARG(&in[i]));
		if(in[i].hw_value && !(in[i].flags & RTW_IEEE80211_CHAN_DISABLED)
			&& (set_idx=rtw_ch_set_search_ch(pmlmeext->channel_set, in[i].hw_value)) >=0
		)
		{
			memcpy(&out[j], &in[i], sizeof(struct rtw_ieee80211_channel));

			if(pmlmeext->channel_set[set_idx].ScanType == SCAN_PASSIVE)
				out[j].flags &= RTW_IEEE80211_CHAN_PASSIVE_SCAN;

			j++;
		}
		if(j>=out_num)
			break;
	}

	/* if out is empty, use channel_set as default */
	if(j == 0) {
		for (i=0;i<pmlmeext->max_chan_nums;i++) {
			out[i].hw_value = pmlmeext->channel_set[i].ChannelNum;

			if(pmlmeext->channel_set[i].ScanType == SCAN_PASSIVE)
				out[i].flags &= RTW_IEEE80211_CHAN_PASSIVE_SCAN;

			j++;
		}
	}

	if (rtlpriv->setband == GHZ_24) {				// 2.4G
		for (i=0; i < j ; i++) {
			if (out[i].hw_value > 35)
				memset(&out[i], 0 , sizeof(struct rtw_ieee80211_channel));
			else
				scan_ch_num++;
		}
		j = scan_ch_num;
	} else if  (rtlpriv->setband == GHZ_50) {			// 5G
		for (i=0; i < j ; i++) {
			if (out[i].hw_value > 35) {
				memcpy(&out[scan_ch_num++], &out[i], sizeof(struct rtw_ieee80211_channel));
			}
		}
		j = scan_ch_num;
	} else
		{}

	return j;
}

uint8_t sitesurvey_cmd_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct sitesurvey_parm	*pparm = (struct sitesurvey_parm *)pbuf;
	uint8_t	bdelayscan = false;
	uint8_t	val8;
	uint32_t	initialgain;
	uint32_t	i;

	if (pmlmeext->sitesurvey_res.state == SCAN_DISABLE)
	{

		pmlmeext->sitesurvey_res.state = SCAN_START;
		pmlmeext->sitesurvey_res.bss_cnt = 0;
		pmlmeext->sitesurvey_res.channel_idx = 0;

		for(i=0;i<RTW_SSID_SCAN_AMOUNT;i++){
			if(pparm->ssid[i].SsidLength) {
				memcpy(pmlmeext->sitesurvey_res.ssid[i].Ssid, pparm->ssid[i].Ssid, IW_ESSID_MAX_SIZE);
				pmlmeext->sitesurvey_res.ssid[i].SsidLength= pparm->ssid[i].SsidLength;
			} else {
				pmlmeext->sitesurvey_res.ssid[i].SsidLength= 0;
			}
		}

		pmlmeext->sitesurvey_res.ch_num = rtw_scan_ch_decision(rtlpriv
			, pmlmeext->sitesurvey_res.ch, RTW_CHANNEL_SCAN_AMOUNT
			, pparm->ch, pparm->ch_num
		);

		pmlmeext->sitesurvey_res.scan_mode = pparm->scan_mode;


		//issue null data if associating to the AP
		if (is_client_associated_to_ap(rtlpriv) == true)
		{
			pmlmeext->sitesurvey_res.state = SCAN_TXNULL;

			issue_nulldata(rtlpriv, NULL, 1, 3, 500);

			bdelayscan = true;
		}
		if(bdelayscan)
		{
			//delay 50ms to protect nulldata(1).
			set_survey_timer(pmlmeext, 50);
			return H2C_SUCCESS;
		}
	}

	if ((pmlmeext->sitesurvey_res.state == SCAN_START) || (pmlmeext->sitesurvey_res.state == SCAN_TXNULL))
	{

		//config the initial gain under scaning, need to write the BB registers
			initialgain = 0x1e;

		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_INITIAL_GAIN, (uint8_t *)(&initialgain));

		//set MSR to no link state
		Set_MSR(rtlpriv, _HW_STATE_NOLINK_);

		val8 = 1; //under site survey
		rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_MLME_SITESURVEY, (uint8_t *)(&val8));

		pmlmeext->sitesurvey_res.state = SCAN_PROCESS;
	}

	site_survey(rtlpriv);

	return H2C_SUCCESS;

}

uint8_t setauth_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf)
{
	struct setauth_parm		*pparm = (struct setauth_parm *)pbuf;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	if (pparm->mode < 4)
	{
		pmlmeinfo->auth_algo = pparm->mode;
	}

	return 	H2C_SUCCESS;
}

uint8_t setkey_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	struct setkey_parm		*pparm = (struct setkey_parm *)pbuf;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	unsigned char					null_sta[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	//main tx key for wep.
	if(pparm->set_tx)
		pmlmeinfo->key_index = pparm->keyid;

	rtw_cam_add_one_entry(rtlpriv, null_sta, pparm->keyid, pparm->keyid,
			      0, pparm->algorithm, pparm->key);

	//allow multicast packets to driver
        rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_ON_RCR_AM, null_addr);

	return H2C_SUCCESS;
}

uint8_t set_stakey_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	uint8_t cam_id = 0;//cam_entry
	uint8_t ret = H2C_SUCCESS;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct set_stakey_parm	*pparm = (struct set_stakey_parm *)pbuf;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct sta_info *psta;

	//cam_entry:
	//0~3 for default key

	//for concurrent mode (ap+sta, sta+sta):
	//default key is disable, using sw encrypt/decrypt
	//camid 0, 1, 2, 3 is default entry for default key/group key
	//macid = 1 is for bc/mc stainfo, no mapping to camid
	//macid = 0 mapping to camid 4
	//for macid >=2, camid = macid+3;


	if(pparm->algorithm == NO_ENCRYPTION)	// clear cam entry
	{
		clear_cam_entry(rtlpriv, pparm->id);
		ret = H2C_SUCCESS;
		goto exit_set_stakey_hdl;
	}

	if((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
	{
		psta = rtw_get_stainfo(pstapriv, pparm->addr);
		if(psta)
		{
			DBG_871X("r871x_set_stakey_hdl(): enc_algorithm=%d\n", pparm->algorithm);

			if((psta->mac_id == 1) || (psta->mac_id>(NUM_STA-4)))
			{
				DBG_871X("r871x_set_stakey_hdl():set_stakey failed, mac_id(aid)=%d\n", psta->mac_id);
				ret = H2C_REJECTED;
				goto exit_set_stakey_hdl;
			}

			cam_id = (uint8_t)rtw_get_camid(psta->mac_id);//0~3 for default key, cmd_id=macid + 3;

			DBG_871X("Write CAM, mac_addr=%x:%x:%x:%x:%x:%x, cam_entry=%d\n", pparm->addr[0],
						pparm->addr[1], pparm->addr[2], pparm->addr[3], pparm->addr[4],
						pparm->addr[5], cam_id);

			rtw_cam_add_one_entry(rtlpriv, pparm->addr, 0, cam_id,
					pparm->algorithm, 1, pparm->key);

			ret = H2C_SUCCESS_RSP;
			goto exit_set_stakey_hdl;

		}
		else
		{
			DBG_871X("r871x_set_stakey_hdl(): sta has been free\n");
			ret = H2C_REJECTED;
			goto exit_set_stakey_hdl;
		}

	}


	//below for sta mode

	if((psta = rtw_get_stainfo(pstapriv, pmlmeinfo->network.MacAddress)))
		cam_id = (uint8_t)rtw_get_camid(psta->mac_id);
	else
		cam_id = 4;

	rtw_cam_add_one_entry(rtlpriv, pparm->addr, 0, cam_id,
			pparm->algorithm, 1, pparm->key);

	pmlmeinfo->enc_algo = pparm->algorithm;

exit_set_stakey_hdl:

	DBG_871X_LEVEL(_drv_always_, "set pairwise key to hw: alg:%d(WEP40-1 WEP104-5 TKIP-2 AES-4) camid:%d\n",
		       	pparm->algorithm, cam_id);

	return ret;

}

uint8_t add_ba_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf)
{
	struct addBaReq_parm 	*pparm = (struct addBaReq_parm *)pbuf;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	struct sta_info *psta = rtw_get_stainfo(&rtlpriv->stapriv, pparm->addr);

	if(!psta)
		return 	H2C_SUCCESS;

	if (((pmlmeinfo->state & WIFI_FW_ASSOC_SUCCESS) && (pmlmeinfo->HT_enable)) ||
		((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
	{
		//pmlmeinfo->ADDBA_retry_count = 0;
		//pmlmeinfo->candidate_tid_bitmap |= (0x1 << pparm->tid);
		//psta->htpriv.candidate_tid_bitmap |= BIT(pparm->tid);
		issue_action_BA(rtlpriv, pparm->addr, RTW_WLAN_ACTION_ADDBA_REQ, (u16)pparm->tid);
		//_set_timer(&pmlmeext->ADDBA_timer, ADDBA_TO);
		_set_timer(&psta->addba_retry_timer, ADDBA_TO);
	}
	else
	{
		psta->htpriv.candidate_tid_bitmap &= ~BIT(pparm->tid);
	}
	return 	H2C_SUCCESS;
}

uint8_t set_tx_beacon_cmd(struct rtl_priv* rtlpriv)
{
	struct cmd_obj	*ph2c;
	struct Tx_Beacon_param 	*ptxBeacon_parm;
	struct cmd_priv	*pcmdpriv = &(rtlpriv->cmdpriv);
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t	res = _SUCCESS;
	int len_diff = 0;



	if ((ph2c = (struct cmd_obj*)rtw_zmalloc(sizeof(struct cmd_obj))) == NULL)
	{
		res= _FAIL;
		goto exit;
	}

	if ((ptxBeacon_parm = (struct Tx_Beacon_param *)rtw_zmalloc(sizeof(struct Tx_Beacon_param))) == NULL)
	{
		rtw_mfree(ph2c);
		res= _FAIL;
		goto exit;
	}

	memcpy(&(ptxBeacon_parm->network), &(pmlmeinfo->network), sizeof(WLAN_BSSID_EX));

	len_diff = update_hidden_ssid(
		ptxBeacon_parm->network.IEs+_BEACON_IE_OFFSET_
		, ptxBeacon_parm->network.IELength-_BEACON_IE_OFFSET_
		, pmlmeinfo->hidden_ssid_mode
	);
	ptxBeacon_parm->network.IELength += len_diff;

	init_h2fwcmd_w_parm_no_rsp(ph2c, ptxBeacon_parm, GEN_CMD_CODE(_TX_Beacon));

	res = rtw_enqueue_cmd(pcmdpriv, ph2c);


exit:



	return res;
}


uint8_t mlme_evt_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf)
{
	uint8_t evt_code, evt_seq;
	u16 evt_sz;
	uint 	*peventbuf;
	void (*event_callback)(struct rtl_priv *dev, uint8_t *pbuf);
	struct evt_priv *pevt_priv = &(rtlpriv->evtpriv);

	peventbuf = (uint*)pbuf;
	evt_sz = (u16)(*peventbuf&0xffff);
	evt_seq = (uint8_t)((*peventbuf>>24)&0x7f);
	evt_code = (uint8_t)((*peventbuf>>16)&0xff);


	#ifdef CHECK_EVENT_SEQ
	// checking event sequence...
	if (evt_seq != (atomic_read(&pevt_priv->event_seq) & 0x7f) )
	{
		pevt_priv->event_seq = (evt_seq+1)&0x7f;

		goto _abort_event_;
	}
	#endif

	// checking if event code is valid
	if (evt_code >= MAX_C2HEVT)
	{
		goto _abort_event_;
	}

	// checking if event size match the event parm size
	if ((wlanevents[evt_code].parmsize != 0) &&
			(wlanevents[evt_code].parmsize != evt_sz))
	{

		goto _abort_event_;

	}

	atomic_inc(&pevt_priv->event_seq);

	peventbuf += 2;

	if(peventbuf)
	{
		event_callback = wlanevents[evt_code].event_callback;
		event_callback(rtlpriv, (uint8_t *)peventbuf);

		pevt_priv->evt_done_cnt++;
	}


_abort_event_:


	return H2C_SUCCESS;

}

uint8_t tx_beacon_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf)
{
	if(send_beacon(rtlpriv)==_FAIL)
	{
		DBG_871X("issue_beacon, fail!\n");
		return H2C_PARAMETERS_ERROR;
	}
#ifdef CONFIG_AP_MODE
	else //tx bc/mc frames after update TIM
	{
		struct sta_info *psta_bmc;
		struct list_head	*xmitframe_plist, *xmitframe_phead;
		struct xmit_frame *pxmitframe=NULL;
		struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
		struct sta_priv  *pstapriv = &rtlpriv->stapriv;

		//for BC/MC Frames
		psta_bmc = rtw_get_bcmc_stainfo(rtlpriv);
		if(!psta_bmc)
			return H2C_SUCCESS;

		if((pstapriv->tim_bitmap&BIT(0)) && (psta_bmc->sleepq_len>0))
		{
			msleep(10);// 10ms, ATIM(HIQ) Windows
			//spin_lock_bh(&psta_bmc->sleep_q.lock, &irqL);
			spin_lock_bh(&pxmitpriv->lock);

			xmitframe_phead = get_list_head(&psta_bmc->sleep_q);
			xmitframe_plist = get_next(xmitframe_phead);

			while ((rtw_end_of_queue_search(xmitframe_phead, xmitframe_plist)) == false)
			{
				pxmitframe = container_of(xmitframe_plist, struct xmit_frame, list);

				xmitframe_plist = get_next(xmitframe_plist);

				list_del_init(&pxmitframe->list);

				psta_bmc->sleepq_len--;
				if(psta_bmc->sleepq_len>0)
					pxmitframe->tx_attrib.mdata = 1;
				else
					pxmitframe->tx_attrib.mdata = 0;

				pxmitframe->tx_attrib.triggered=1;

				pxmitframe->tx_attrib.tx_qsel = 0x11;//HIQ

				rtlpriv->cfg->ops->hal_xmitframe_enqueue(rtlpriv, pxmitframe);

				//pstapriv->tim_bitmap &= ~BIT(0);

			}

			//spin_unlock_bh(&psta_bmc->sleep_q.lock, &irqL);
			spin_unlock_bh(&pxmitpriv->lock);


		}

	}
#endif

	return H2C_SUCCESS;

}

void change_band_update_ie(struct rtl_priv *rtlpriv, WLAN_BSSID_EX *pnetwork)
{
	uint8_t	network_type,rate_len, total_rate_len,remainder_rate_len;
	struct mlme_ext_priv *pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);
	uint8_t	erpinfo=0x4;

	//DBG_871X("%s\n", __FUNCTION__);

	if(pmlmeext->cur_channel >= 36)
	{
		network_type = WIRELESS_11A;
		total_rate_len = IEEE80211_NUM_OFDM_RATESLEN;
		DBG_871X("%s(): change to 5G Band\n",__FUNCTION__);
		rtw_remove_bcn_ie(rtlpriv, pnetwork, _ERPINFO_IE_);
	}
	else
	{
		network_type = WIRELESS_11BG;
		total_rate_len = IEEE80211_CCK_RATE_LEN+IEEE80211_NUM_OFDM_RATESLEN;
		DBG_871X("%s(): change to 2.4G Band\n",__FUNCTION__);
		rtw_add_bcn_ie(rtlpriv, pnetwork, _ERPINFO_IE_, &erpinfo, 1);
	}

	rtw_set_supported_rate(pnetwork->SupportedRates, network_type);

	UpdateBrateTbl(rtlpriv, pnetwork->SupportedRates);
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_BASIC_RATE, pnetwork->SupportedRates);

	if(total_rate_len > 8)
	{
		rate_len = 8;
		remainder_rate_len = total_rate_len - 8;
	}
	else
	{
		rate_len = total_rate_len;
		remainder_rate_len = 0;
	}

	rtw_add_bcn_ie(rtlpriv, pnetwork, _SUPPORTEDRATES_IE_, pnetwork->SupportedRates, rate_len);

	if(remainder_rate_len)
	{
		rtw_add_bcn_ie(rtlpriv, pnetwork, _EXT_SUPPORTEDRATES_IE_, (pnetwork->SupportedRates+8), remainder_rate_len);
	}
	else
	{
		rtw_remove_bcn_ie(rtlpriv, pnetwork, _EXT_SUPPORTEDRATES_IE_);
	}
}


uint8_t set_ch_hdl(struct rtl_priv *rtlpriv, uint8_t *pbuf)
{
	struct set_ch_parm *set_ch_parm;
	struct mlme_priv		*pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;

	if(!pbuf)
		return H2C_PARAMETERS_ERROR;

	set_ch_parm = (struct set_ch_parm *)pbuf;

	DBG_871X(FUNC_NDEV_FMT" ch:%u, bw:%u, ch_offset:%u\n",
		FUNC_NDEV_ARG(rtlpriv->ndev),
		set_ch_parm->ch, set_ch_parm->bw, set_ch_parm->ch_offset);

	pmlmeext->cur_channel = set_ch_parm->ch;
	pmlmeext->cur_ch_offset = set_ch_parm->ch_offset;
	pmlmeext->cur_bwmode = set_ch_parm->bw;

	set_channel_bwmode(rtlpriv, set_ch_parm->ch, set_ch_parm->ch_offset, set_ch_parm->bw);

	return 	H2C_SUCCESS;
}

uint8_t set_csa_hdl(struct rtl_priv *rtlpriv, unsigned char *pbuf)
{
#ifdef CONFIG_DFS
	struct SetChannelSwitch_param *setChannelSwitch_param;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct mlme_ext_priv *pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;
	uint8_t new_ch_no;
	uint8_t gval8 = 0x00, sval8 = 0xff;

	if(!pbuf)
		return H2C_PARAMETERS_ERROR;

	setChannelSwitch_param = (struct SetChannelSwitch_param *)pbuf;
	new_ch_no = setChannelSwitch_param->new_ch_no;

	rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_TXPAUSE, &gval8);

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_TXPAUSE, &sval8);

	DBG_871X("DFS detected! Swiching channel to %d!\n", new_ch_no);
	SelectChannel(rtlpriv, new_ch_no);

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_TXPAUSE, &gval8);

	rtw_free_network_queue(rtlpriv, true);
	rtw_indicate_disconnect(rtlpriv);

	if ( ((new_ch_no >= 52) && (new_ch_no <= 64)) ||((new_ch_no >= 100) && (new_ch_no <= 140)) ) {
		DBG_871X("Switched to DFS band (ch %02x) again!!\n", new_ch_no);
	}

	return 	H2C_SUCCESS;
#else
	return	H2C_REJECTED;
#endif //CONFIG_DFS

}

