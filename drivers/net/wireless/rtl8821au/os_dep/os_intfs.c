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
#define _OS_INTFS_C_

#include <linux/ip.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>

#include <drv_types.h>

/* module param defaults */
static int rtw_channel = 1;/* ad-hoc support requirement */
static int rtw_rts_thresh = 2347;
static int rtw_preamble = PREAMBLE_LONG;	/* long, short, auto */
static int rtw_adhoc_tx_pwr = 1;
static int rtw_soft_ap = 0;
				/* int smart_ps = 1; */
#ifdef CONFIG_PM
static int rtw_power_mgnt = 1;
#else
static int rtw_power_mgnt = PS_MODE_ACTIVE;
#endif

static int rtw_smart_ps = 2;


static int rtw_long_retry_lmt = 7;
static int rtw_short_retry_lmt = 7;
static int rtw_busy_thresh = 40;
/* int qos_enable = 0; */
static int rtw_ack_policy = NORMAL_ACK;

static int rtw_acm_method = 0;			/* 0:By SW 1:By HW. */

static int rtw_wmm_enable = 1;		/* default is set to enable the wmm. */
static int rtw_uapsd_enable = 0;

/*
 *  0: 20 MHz, 1: 40 MHz, 2: 80 MHz, 3: 160MHz, 4: 80+80MHz
 *  2.4G use bit 0 ~ 3, 5G use bit 4 ~ 7
 * 0x21 means enable 2.4G 40MHz & 5G 80MHz
 */
static int rtw_rx_stbc = 1;		/* 0: disable, bit(0):enable 2.4g, bit(1):enable 5g, default is set to enable 2.4GHZ for IOT issue with bufflao's AP at 5GHZ */

static int rtw_ampdu_amsdu = 0;/*  0: disabled, 1:enabled, 2:auto */
/*
 *  Short GI support Bit Map
 *  BIT0 - 20MHz, 0: support, 1: non-support
 *  BIT1 - 40MHz, 0: support, 1: non-support
 *  BIT2 - 80MHz, 0: support, 1: non-support
 *  BIT3 - 160MHz, 0: support, 1: non-support
 */

/*
 *
 *  BIT0: Enable VHT LDPC Rx,
 *  BIT1: Enable VHT LDPC Tx,
 *  BIT4: Enable HT LDPC Rx,
 *  BIT5: Enable HT LDPC Tx
 */
/*  BIT0: Enable VHT STBC Rx,
 *  BIT1: Enable VHT STBC Tx,
 *  BIT4: Enable HT STBC Rx,
 *  BIT5: Enable HT STBC Tx
 */
/*
 * BIT0: Enable VHT Beamformer,
 * BIT1: Enable VHT Beamformee,
 * BIT4: Enable HT Beamformer,
 * BIT5: Enable HT Beamformee
 */
static int rtw_beamform_cap = 0;

static int rtw_low_power = 0;

static int rtw_AcceptAddbaReq = true;/* 0:Reject AP's Add BA req, 1:Accept AP's Add BA req. */

static int rtw_enusbss = 0;	/* 0:disable,1:enable */

static int rtw_hwpdn_mode = 2;	/* 0:disable,1:enable,2: by EFUSE config */

static int rtw_hwpwrp_detect = 0; /* HW power  ping detect 0:disable , 1:enable */

/* ULLI: check var rtw_hw_wps_pbc */
static int rtw_hw_wps_pbc = 1;

#ifdef CONFIG_TX_MCAST2UNI
int __rtw_mc2u_disable = 0;
#endif

char *__rtw_initmac = 0;  /* temp mac address if users want to use instead of the mac address in Efuse */



module_param(__rtw_initmac, charp, 0644);
module_param(rtw_channel, int, 0644);
module_param(rtw_wmm_enable, int, 0644);
module_param(rtw_busy_thresh, int, 0644);

module_param(rtw_rx_stbc, int, 0644);
module_param(rtw_ampdu_amsdu, int, 0644);

module_param(rtw_power_mgnt, int, 0644);
module_param(rtw_smart_ps, int, 0644);
module_param(rtw_low_power, int, 0644);

module_param(rtw_enusbss, int, 0644);
module_param(rtw_hwpdn_mode, int, 0644);
module_param(rtw_hwpwrp_detect, int, 0644);

module_param(rtw_hw_wps_pbc, int, 0644);

#ifdef CONFIG_TX_MCAST2UNI
module_param(__rtw_mc2u_disable, int, 0644);
#endif

uint loadparam(struct rtl_priv *rtlpriv, struct net_device *ndev)
{

	uint status = _SUCCESS;
	struct registry_priv  *registry_par = &rtlpriv->registrypriv;

	memcpy(registry_par->ssid.Ssid, "ANY", 3);
	registry_par->ssid.SsidLength = 3;

	registry_par->channel = (uint8_t)rtw_channel;

	if (IsSupported24G(WIRELESS_MODE_MAX) && (!IsSupported5G(WIRELESS_MODE_MAX))
		&& (registry_par->channel > 14)) {
		registry_par->channel = 1;
	} else if (IsSupported5G(WIRELESS_MODE_MAX) && (!IsSupported24G(WIRELESS_MODE_MAX))
		&& (registry_par->channel <= 14)) {
		registry_par->channel = 36;
	}

	registry_par->rts_thresh = (u16)rtw_rts_thresh;
	registry_par->preamble = (uint8_t)rtw_preamble;
	registry_par->adhoc_tx_pwr = (uint8_t)rtw_adhoc_tx_pwr;
	registry_par->soft_ap =  (uint8_t)rtw_soft_ap;
	registry_par->smart_ps =  (uint8_t)rtw_smart_ps;
	registry_par->power_mgnt = (uint8_t)rtw_power_mgnt;
	registry_par->long_retry_lmt = (uint8_t)rtw_long_retry_lmt;
	registry_par->short_retry_lmt = (uint8_t)rtw_short_retry_lmt;
	registry_par->busy_thresh = (u16)rtw_busy_thresh;
	/* registry_par->qos_enable = (uint8_t)rtw_qos_enable; */
	registry_par->ack_policy = (uint8_t)rtw_ack_policy;

	registry_par->acm_method = (uint8_t)rtw_acm_method;

	 /* UAPSD */
	registry_par->wmm_enable = (uint8_t)rtw_wmm_enable;
	registry_par->uapsd_enable = (uint8_t)rtw_uapsd_enable;

	registry_par->rx_stbc = (uint8_t)rtw_rx_stbc;
	registry_par->ampdu_amsdu = (uint8_t)rtw_ampdu_amsdu;

	registry_par->beamform_cap = (uint8_t)rtw_beamform_cap;
	registry_par->low_power = (uint8_t)rtw_low_power;

	registry_par->bAcceptAddbaReq = (uint8_t)rtw_AcceptAddbaReq;

#ifdef CONFIG_AUTOSUSPEND
	registry_par->usbss_enable = (uint8_t)rtw_enusbss;	/* 0:disable,1:enable */
#endif

	registry_par->hw_wps_pbc = (uint8_t)rtw_hw_wps_pbc;

	return status;
}

struct net_device_stats *rtw_net_get_stats(struct net_device *ndev)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct xmit_priv *pxmitpriv = &(rtlpriv->xmitpriv);
	struct recv_priv *precvpriv = &(rtlpriv->recvpriv);

	rtlpriv->stats.tx_packets = pxmitpriv->tx_pkts;	/* pxmitpriv->tx_pkts++; */
	rtlpriv->stats.rx_packets = precvpriv->rx_pkts;	/* precvpriv->rx_pkts++; */
	rtlpriv->stats.tx_dropped = pxmitpriv->tx_drop;
	rtlpriv->stats.rx_dropped = precvpriv->rx_drop;
	rtlpriv->stats.tx_bytes = pxmitpriv->tx_bytes;
	rtlpriv->stats.rx_bytes = precvpriv->rx_bytes;

	return &rtlpriv->stats;
}

/*
 * AC to queue mapping
 *
 * AC_VO -> queue 0
 * AC_VI -> queue 1
 * AC_BE -> queue 2
 * AC_BK -> queue 3
 */
static const u16 rtw_1d_to_queue[8] = { 2, 3, 3, 2, 1, 1, 0, 0 };

/* Given a data frame determine the 802.1p/1d tag to use. */
unsigned int rtw_classify8021d(struct sk_buff *skb)
{
	unsigned int dscp;

	/* skb->priority values from 256->263 are magic values to
	 * directly indicate a specific 802.1d priority.  This is used
	 * to allow 802.1d priority to be passed directly in from VLAN
	 * tags, etc.
	 */
	if (skb->priority >= 256 && skb->priority <= 263)
		return skb->priority - 256;

	switch (skb->protocol) {
	case htons(ETH_P_IP):
		dscp = ip_hdr(skb)->tos & 0xfc;
		break;
	default:
		return 0;
	}

	return dscp >> 5;
}

u16 rtw_recv_select_queue(struct sk_buff *skb)
{
	struct iphdr *piphdr;
	unsigned int dscp;
	u16	eth_type;
	u32 priority;
	uint8_t *pdata = skb->data;

	memcpy(&eth_type, pdata+(ETH_ALEN<<1), 2);

	switch (eth_type) {
	case htons(ETH_P_IP):

		piphdr = (struct iphdr *)(pdata+ETH_HLEN);

		dscp = piphdr->tos & 0xfc;

		priority = dscp >> 5;

		break;
	default:
		priority = 0;
	}

	return rtw_1d_to_queue[priority];

}

u32 rtw_start_drv_threads(struct rtl_priv *rtlpriv)
{
	u32 _status = _SUCCESS;
	int _unused;

	{
		rtlpriv->cmdThread = kthread_run(rtw_cmd_thread, rtlpriv, "RTW_CMD_THREAD");
		if (IS_ERR(rtlpriv->cmdThread))
			_status = _FAIL;
		else
			_unused = down_interruptible(&rtlpriv->cmdpriv.terminate_cmdthread_sema); /* wait for cmd_thread to run */
	}
	return _status;

}

void rtw_stop_drv_threads(struct rtl_priv *rtlpriv)
{
	int _unused;

	/* Below is to termindate rtw_cmd_thread & event_thread... */
	up(&rtlpriv->cmdpriv.cmd_queue_sema);
	/* up(&rtlpriv->cmdpriv.cmd_done_sema); */
	if (rtlpriv->cmdThread) {
		_unused = down_interruptible(&rtlpriv->cmdpriv.terminate_cmdthread_sema);
	}
}
