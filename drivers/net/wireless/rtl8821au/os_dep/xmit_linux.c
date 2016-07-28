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
#define _XMIT_OSDEP_C_

#include <drv_types.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}



void rtw_set_tx_chksum_offload(struct sk_buff *pkt, struct tx_pkt_attrib *pattrib)
{

#ifdef CONFIG_TCP_CSUM_OFFLOAD_TX
	struct sk_buff *skb = (struct sk_buff *)pkt;
	pattrib->hw_tcp_csum = 0;

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if (skb_shinfo(skb)->nr_frags == 0) {
			const struct iphdr *ip = ip_hdr(skb);
			if (ip->protocol == IPPROTO_TCP) {
				/* TCP checksum offload by HW */
				DBG_871X("CHECKSUM_PARTIAL TCP\n");
				pattrib->hw_tcp_csum = 1;
				/* skb_checksum_help(skb); */
			} else if (ip->protocol == IPPROTO_UDP) {
				/* DBG_871X("CHECKSUM_PARTIAL UDP\n"); */
#if 1
				skb_checksum_help(skb);
#else
				/* Set UDP checksum = 0 to skip checksum check */
				struct udphdr *udp = skb_transport_header(skb);
				udp->check = 0;
#endif
			} else {
				DBG_871X("%s-%d TCP CSUM offload Error!!\n", __FUNCTION__, __LINE__);
				WARN_ON(1);     /* we need a WARN() */
			}
		} else {
			/* IP fragmentation case */
			DBG_871X("%s-%d nr_frags != 0, using skb_checksum_help(skb);!!\n", __FUNCTION__, __LINE__);
                	skb_checksum_help(skb);
		}
	}
#endif
}

int rtw_os_xmit_resource_alloc(struct rtl_priv *rtlpriv, struct xmit_buf *pxmitbuf, u32 alloc_sz, uint8_t flag)
{
	if (alloc_sz > 0) {
		pxmitbuf->pallocated_buf = rtw_zmalloc(alloc_sz);
		if (pxmitbuf->pallocated_buf == NULL) {
			return _FAIL;
		}

		pxmitbuf->pbuf = (uint8_t *)N_BYTE_ALIGMENT((__kernel_size_t)(pxmitbuf->pallocated_buf), XMITBUF_ALIGN_SZ);
	}

	if (flag) {
		int i;
		for (i = 0; i < 8; i++) {
			pxmitbuf->pxmit_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
			if(pxmitbuf->pxmit_urb[i] == NULL) {
				DBG_871X("pxmitbuf->pxmit_urb[i]==NULL");
				return _FAIL;
			}
		}
	}

	return _SUCCESS;
}

void rtw_os_xmit_resource_free(struct rtl_priv *rtlpriv, struct xmit_buf *pxmitbuf,u32 free_sz, uint8_t flag)
{
	if (flag) {
		int i;

		for(i=0; i<8; i++) {
			if(pxmitbuf->pxmit_urb[i]) {
				/* usb_kill_urb(pxmitbuf->pxmit_urb[i]); */
				usb_free_urb(pxmitbuf->pxmit_urb[i]);
			}
		}
	}

	if (free_sz > 0 ) {
		if(pxmitbuf->pallocated_buf) {
			rtw_mfree(pxmitbuf->pallocated_buf);
		}
	}
}

#define WMM_XMIT_THRESHOLD	(NR_XMITFRAME*2/5)

void rtw_os_pkt_complete(struct rtl_priv *rtlpriv, struct sk_buff *pkt)
{
	u16 queue;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;

	queue = skb_get_queue_mapping(pkt);
	if(__netif_subqueue_stopped(rtlpriv->ndev, queue))
		netif_wake_subqueue(rtlpriv->ndev, queue);

	dev_kfree_skb_any(pkt);
}

void rtw_os_xmit_complete(struct rtl_priv *rtlpriv, struct xmit_frame *pxframe)
{
	if(pxframe->skb) {
		/*
		 * RT_TRACE(_module_xmit_osdep_c_,_drv_err_,("linux : rtw_os_xmit_complete, dev_kfree_skb()\n"));
		 *
		 * dev_kfree_skb_any(pxframe->pkt);
		 */
		rtw_os_pkt_complete(rtlpriv, pxframe->skb);
	}

	pxframe->skb = NULL;
}

void rtw_os_xmit_schedule(struct rtl_priv *rtlpriv)
{
	struct rtl_priv *pri_adapter = rtlpriv;

	struct xmit_priv *pxmitpriv;

	if(!rtlpriv)
		return;

	pxmitpriv = &rtlpriv->xmitpriv;

	spin_lock_bh(&pxmitpriv->lock);

	if(rtw_txframes_pending(rtlpriv)) {
		tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
	}

	spin_unlock_bh(&pxmitpriv->lock);
}

static void rtw_check_xmit_resource(struct rtl_priv *rtlpriv, struct sk_buff *pkt)
{
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	u16	queue;

	queue = skb_get_queue_mapping(pkt);
	if(pxmitpriv->free_xmitframe_cnt<=4) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(rtlpriv->ndev, queue)))
			netif_stop_subqueue(rtlpriv->ndev, queue);
	}
}

#ifdef CONFIG_TX_MCAST2UNI
int rtw_mlcst2unicst(struct rtl_priv *rtlpriv, struct sk_buff *skb)
{
	struct	sta_priv *pstapriv = &rtlpriv->stapriv;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct list_head	*phead, *plist;
	struct sk_buff *newskb;
	struct sta_info *psta = NULL;
	uint8_t chk_alive_num = 0;
	char chk_alive_list[NUM_STA];
	uint8_t bc_addr[6]={0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	uint8_t null_addr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	int i;
	int32_t	res;

	spin_lock_bh(&pstapriv->asoc_list_lock);
	phead = &pstapriv->asoc_list;
	plist = phead->next;

	/* free sta asoc_queue */
	while ((rtw_end_of_queue_search(phead, plist)) == false) {
		int stainfo_offset;
		psta = container_of(plist, struct sta_info, asoc_list);
		plist = plist->next;

		stainfo_offset = rtw_stainfo_offset(pstapriv, psta);
		if (stainfo_offset_valid(stainfo_offset)) {
			chk_alive_list[chk_alive_num++] = stainfo_offset;
		}
	}
	spin_unlock_bh(&pstapriv->asoc_list_lock);

	for (i = 0; i < chk_alive_num; i++) {
		psta = rtw_get_stainfo_by_offset(pstapriv, chk_alive_list[i]);
		if(!(psta->state &_FW_LINKED))
			continue;

		/* avoid come from STA1 and send back STA1 */
		if (_rtw_memcmp(psta->hwaddr, &skb->data[6], 6) == true
			|| _rtw_memcmp(psta->hwaddr, null_addr, 6) == true
			|| _rtw_memcmp(psta->hwaddr, bc_addr, 6) == true
		)
			continue;

		newskb = skb_copy(skb, GFP_ATOMIC);

		if (newskb) {
			memcpy(newskb->data, psta->hwaddr, 6);
			res = rtw_xmit(rtlpriv, &newskb);
			if (res < 0) {
				DBG_871X("%s()-%d: rtw_xmit() return error!\n", __FUNCTION__, __LINE__);
				pxmitpriv->tx_drop++;
				dev_kfree_skb_any(newskb);
			} else
				pxmitpriv->tx_pkts++;
		} else {
			DBG_871X("%s-%d: skb_copy() failed!\n", __FUNCTION__, __LINE__);
			pxmitpriv->tx_drop++;
			/* dev_kfree_skb_any(skb); */
			return false;	/* Caller shall tx this multicast frame via normal way. */
		}
	}

	dev_kfree_skb_any(skb);
	return true;
}
#endif


int rtw_xmit_entry(struct sk_buff *pkt, struct net_device *ndev)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
#ifdef CONFIG_TX_MCAST2UNI
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	extern int __rtw_mc2u_disable;
#endif
	int32_t res = 0;
	u16 queue;

	if (rtw_if_up(rtlpriv) == false) {
		goto drop_packet;
	}

	rtw_check_xmit_resource(rtlpriv, pkt);

#ifdef CONFIG_TX_MCAST2UNI
	if ( !__rtw_mc2u_disable && check_fwstate(pmlmepriv, WIFI_AP_STATE) == true &&
	   ( IP_MCAST_MAC(pkt->data) || ICMPV6_MCAST_MAC(pkt->data) )
		)
	{
		if ( pxmitpriv->free_xmitframe_cnt > (NR_XMITFRAME/4) ) {
			res = rtw_mlcst2unicst(rtlpriv, pkt);
			if (res == true) {
				goto exit;
			}
		} else {
			/*
			 * DBG_871X("Stop M2U(%d, %d)! ", pxmitpriv->free_xmitframe_cnt, pxmitpriv->free_xmitbuf_cnt);
			 * DBG_871X("!m2u );
			 */
		}
	}
#endif

	res = rtw_xmit(rtlpriv, &pkt);
	if (res < 0) {
		goto drop_packet;
	}

	pxmitpriv->tx_pkts++;
	goto exit;

drop_packet:
	pxmitpriv->tx_drop++;
	dev_kfree_skb_any(pkt);

exit:



	return 0;
}

