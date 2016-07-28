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
#define _RECV_OSDEP_C_

#include <linux/etherdevice.h>
#include <net/iw_handler.h>
#include <linux/if_arp.h>

#include <drv_types.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

int rtw_os_alloc_recvframe(struct rtl_priv *rtlpriv, struct recv_frame *precvframe, uint8_t *pdata, struct sk_buff *pskb)
{
	int res = _SUCCESS;
	uint8_t	shift_sz = 0;
	u32	skb_len, alloc_sz;
	struct sk_buff *pkt_copy = NULL;
	struct rx_pkt_attrib *pattrib = &precvframe->attrib;


	if(pdata == NULL) {
		precvframe->skb = NULL;
		res = _FAIL;
		return res;
	}


	/*
	 * 	Modified by Albert 20101213
	 * 	For 8 bytes IP header alignment.
	 */
	shift_sz = pattrib->qos ? 6:0;	/* Qos data, wireless lan header length is 26 */

	skb_len = pattrib->pkt_len;

	/*
	 *  for first fragment packet, driver need allocate 1536+drvinfo_sz+RXDESC_SIZE to defrag packet.
	 *  modify alloc_sz for recvive crc error packet by thomas 2011-06-02
	 */
	if((pattrib->mfrag == 1)&&(pattrib->frag_num == 0)) {
		/* alloc_sz = 1664;	//1664 is 128 alignment. */
		alloc_sz = (skb_len <= 1650) ? 1664:(skb_len + 14);
	} else {
		alloc_sz = skb_len;
		/*
		 * 6 is for IP header 8 bytes alignment in QoS packet case.
		 * 8 is for skb->data 4 bytes alignment.
		 */
		alloc_sz += 14;
	}

	/* ULLI : why copy skb */

	pkt_copy = netdev_alloc_skb(rtlpriv->ndev, alloc_sz);

	if(pkt_copy) {
		pkt_copy->dev = rtlpriv->ndev;
		precvframe->skb = pkt_copy;
		precvframe->rx_head = pkt_copy->data;
		precvframe->rx_end = pkt_copy->data + alloc_sz;
		skb_reserve(pkt_copy, 8 - ((__kernel_size_t)( pkt_copy->data) & 7 ));	/* force pkt_copy->data at 8-byte alignment address */
		skb_reserve(pkt_copy, shift_sz);				/* force ip_hdr at 8-byte alignment address according to shift_sz. */
		memcpy(pkt_copy->data, pdata, skb_len);
		precvframe->rx_data = precvframe->rx_tail = pkt_copy->data;
	} else 	{
		if((pattrib->mfrag == 1)&&(pattrib->frag_num == 0)) {
			DBG_871X("%s: alloc_skb fail , drop frag frame \n", __FUNCTION__);
			/* rtw_free_recvframe(precvframe, pfree_recv_queue); */
			res = _FAIL;
			goto out;
		}

		if(pskb == NULL) {
			res = _FAIL;
			goto out;
		}

		precvframe->skb = skb_clone(pskb, GFP_ATOMIC);
		if(precvframe->skb) {
			precvframe->rx_head = precvframe->rx_data = precvframe->rx_tail = pdata;
			precvframe->rx_end =  pdata + alloc_sz;
		} else 	{
			DBG_871X("%s: skb_clone fail\n", __FUNCTION__);
			/*
			 * rtw_free_recvframe(precvframe, pfree_recv_queue);
			 * goto _exit_recvbuf2recvframe;
			 */
			res = _FAIL;
		}
	}

out:
	return res;

}

void rtw_os_free_recvframe(struct recv_frame *precvframe)
{
	if(precvframe->skb) {
		dev_kfree_skb_any(precvframe->skb);	/* free skb by driver */

		precvframe->skb = NULL;
	}
}

/* alloc os related resource in struct recv_buf */
int rtw_os_recvbuf_resource_alloc(struct rtl_priv *rtlpriv, struct recv_buf *precvbuf)
{
	int res=_SUCCESS;

	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	struct usb_device *pusbd = rtlusb->udev;

	precvbuf->irp_pending = false;
	precvbuf->purb = usb_alloc_urb(0, GFP_KERNEL);
	if(precvbuf->purb == NULL){
		res = _FAIL;
	}

	precvbuf->skb = NULL;

	precvbuf->reuse = false;

	precvbuf->len = 0;

	return res;
}

/* free os related resource in struct recv_buf */
int rtw_os_recvbuf_resource_free(struct rtl_priv *rtlpriv, struct recv_buf *precvbuf)
{
	int ret = _SUCCESS;

	if(precvbuf->purb) {
		/* usb_kill_urb(precvbuf->purb); */
		usb_free_urb(precvbuf->purb);
	}

	if(precvbuf->skb)
		dev_kfree_skb_any(precvbuf->skb);


	return ret;

}

struct sk_buff  *rtw_os_alloc_msdu_pkt(struct recv_frame *prframe, u16 nSubframe_Length, uint8_t *pdata)
{
	u16	eth_type;
	uint8_t	*data_ptr;
	struct sk_buff  *sub_skb;
	struct rx_pkt_attrib *pattrib;

	pattrib = &prframe->attrib;

#ifdef CONFIG_SKB_COPY
	sub_skb = dev_alloc_skb(nSubframe_Length + 12);
	if(sub_skb) {
		skb_reserve(sub_skb, 12);
		data_ptr = (uint8_t *)skb_put(sub_skb, nSubframe_Length);
		memcpy(data_ptr, (pdata + ETH_HLEN), nSubframe_Length);
	} else
#endif
	{
		/* ULLI : another place who clones skb */

		sub_skb = skb_clone(prframe->skb, GFP_ATOMIC);
		if(sub_skb) {
			sub_skb->data = pdata + ETH_HLEN;
			sub_skb->len = nSubframe_Length;
			skb_set_tail_pointer(sub_skb, nSubframe_Length);
		} else 	{
			DBG_871X("%s(): skb_clone() Fail!!!\n",__FUNCTION__);
			return NULL;
		}
	}

	eth_type = RTW_GET_BE16(&sub_skb->data[6]);

	if (sub_skb->len >= 8 &&
		((_rtw_memcmp(sub_skb->data, rtw_rfc1042_header, SNAP_SIZE) &&
		  eth_type != ETH_P_AARP && eth_type != ETH_P_IPX) ||
		 _rtw_memcmp(sub_skb->data, rtw_bridge_tunnel_header, SNAP_SIZE) )) {
		/* remove RFC1042 or Bridge-Tunnel encapsulation and replace EtherType */
		skb_pull(sub_skb, SNAP_SIZE);
		memcpy(skb_push(sub_skb, ETH_ALEN), pattrib->src, ETH_ALEN);
		memcpy(skb_push(sub_skb, ETH_ALEN), pattrib->dst, ETH_ALEN);
	} else {
		u16 len;
		/* Leave Ethernet header part of hdr and full payload */
		len = htons(sub_skb->len);
		memcpy(skb_push(sub_skb, 2), &len, 2);
		memcpy(skb_push(sub_skb, ETH_ALEN), pattrib->src, ETH_ALEN);
		memcpy(skb_push(sub_skb, ETH_ALEN), pattrib->dst, ETH_ALEN);
	}

	return sub_skb;
}

void rtw_os_recv_indicate_pkt(struct rtl_priv *rtlpriv, struct sk_buff *skb, struct rx_pkt_attrib *pattrib)
{
	struct mlme_priv*pmlmepriv = &rtlpriv->mlmepriv;

	/* Indicat the packets to upper layer */
	if (skb) {
		if(check_fwstate(pmlmepriv, WIFI_AP_STATE) == true) {
			struct sk_buff  *skb_dup = NULL;
		 	struct sta_info *psta = NULL;
		 	struct sta_priv *pstapriv = &rtlpriv->stapriv;

			/* DBG_871X("bmcast=%d\n", bmcast); */

			if(_rtw_memcmp(pattrib->dst, rtlpriv->mac80211.mac_addr, ETH_ALEN)==false) {
				/* DBG_871X("not ap psta=%p, addr=%pM\n", psta, pattrib->dst); */

				if(is_multicast_ether_addr(pattrib->dst)) {
					psta = rtw_get_bcmc_stainfo(rtlpriv);
					skb_dup = skb_clone(skb, GFP_ATOMIC);
				} else {
					psta = rtw_get_stainfo(pstapriv, pattrib->dst);
				}

				if(psta) {
					struct net_device *ndev= (struct net_device*)rtlpriv->ndev;

					/* DBG_871X("directly forwarding to the rtw_xmit_entry\n"); */

					/* skb->ip_summed = CHECKSUM_NONE; */
					skb->dev = ndev;
					skb_set_queue_mapping(skb, rtw_recv_select_queue(skb));

					rtw_xmit_entry(skb, ndev);

					if(is_multicast_ether_addr(pattrib->dst) && (skb_dup != NULL) ) {
						skb = skb_dup;
					} else {
						return;
					}
				}
			}
			else {
				/* to APself */
				/* DBG_871X("to APSelf\n"); */
			}
		}

		skb->protocol = eth_type_trans(skb, rtlpriv->ndev);
		skb->dev = rtlpriv->ndev;

		skb->ip_summed = CHECKSUM_NONE;
		netif_rx(skb);
	}
}

void rtw_handle_tkip_mic_err(struct rtl_priv *rtlpriv,uint8_t bgroup)
{
	union iwreq_data wrqu;
	struct iw_michaelmicfailure    ev;
	struct mlme_priv*              pmlmepriv  = &rtlpriv->mlmepriv;
	struct security_priv	*psecuritypriv = &rtlpriv->securitypriv;
	u32 cur_time = 0;

	if (psecuritypriv->last_mic_err_time == 0) {
		psecuritypriv->last_mic_err_time = jiffies;
	} else {
		cur_time = jiffies;

		if(cur_time - psecuritypriv->last_mic_err_time < 60*HZ) {
			psecuritypriv->btkip_countermeasure = true;
			psecuritypriv->last_mic_err_time = 0;
			psecuritypriv->btkip_countermeasure_time = cur_time;
		} else {
			psecuritypriv->last_mic_err_time = jiffies;
		}
	}

	memset(&ev, 0x00, sizeof(ev));
	if (bgroup) {
	    ev.flags |= IW_MICFAILURE_GROUP;
	} else {
	    ev.flags |= IW_MICFAILURE_PAIRWISE;
	}

	ev.src_addr.sa_family = ARPHRD_ETHER;
	memcpy( ev.src_addr.sa_data, &pmlmepriv->assoc_bssid[ 0 ], ETH_ALEN );

	memset( &wrqu, 0x00, sizeof( wrqu ) );
	wrqu.data.length = sizeof( ev );

	wireless_send_event( rtlpriv->ndev, IWEVMICHAELMICFAILURE, &wrqu, (char*) &ev );
}


int rtw_recv_indicatepkt(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame)
{
	struct recv_priv *precvpriv;
	struct __queue	*pfree_recv_queue;
	struct sk_buff  *skb;
	struct mlme_priv*pmlmepriv = &rtlpriv->mlmepriv;
	struct rx_pkt_attrib *pattrib = &precv_frame->attrib;

	precvpriv = &(rtlpriv->recvpriv);
	pfree_recv_queue = &(precvpriv->free_recv_queue);

	skb = precv_frame->skb;
	if (skb == NULL) {
		goto _recv_indicatepkt_drop;
	}

	skb->data = precv_frame->rx_data;

	skb_set_tail_pointer(skb, precv_frame->len);

	skb->len = precv_frame->len;

	rtw_os_recv_indicate_pkt(rtlpriv, skb, pattrib);

_recv_indicatepkt_end:

	precv_frame->skb = NULL; 		/* pointers to NULL before rtw_free_recvframe() */

	rtw_free_recvframe(precv_frame, pfree_recv_queue);


        return _SUCCESS;

_recv_indicatepkt_drop:

	 /* enqueue back to free_recv_queue */
	 if(precv_frame)
		 rtw_free_recvframe(precv_frame, pfree_recv_queue);

	 return _FAIL;



}

void _rtw_reordering_ctrl_timeout_handler (void *FunctionContext);
void _rtw_reordering_ctrl_timeout_handler (void *FunctionContext)
{
	struct recv_reorder_ctrl *preorder_ctrl = (struct recv_reorder_ctrl *)FunctionContext;
	rtw_reordering_ctrl_timeout_handler(preorder_ctrl);
}

void rtw_init_recv_timer(struct recv_reorder_ctrl *preorder_ctrl)
{
	struct rtl_priv *rtlpriv = preorder_ctrl->rtlpriv;

	_init_timer(&(preorder_ctrl->reordering_ctrl_timer), rtlpriv->ndev, _rtw_reordering_ctrl_timeout_handler, preorder_ctrl);

}

