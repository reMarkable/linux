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
#define _RTW_XMIT_C_

#include <linux/ip.h>
#include <linux/etherdevice.h>
#include <drv_types.h>
#include <rtw_ap.h>
#include <linux/udp.h>
#include <rtl8812a_xmit.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

#undef DBG_8192C
static inline void DBG_8192C(const char *fmt, ...)
{
}

static struct list_head *get_next(struct list_head	*list)
{
	return list->next;
}

#define _drv_always_		1
#undef DBG_871X_LEVEL
static inline void DBG_871X_LEVEL(const int level, const char *fmt, ...)
{
}

static uint8_t P802_1H_OUI[P80211_OUI_LEN] = { 0x00, 0x00, 0xf8 };
static uint8_t RFC1042_OUI[P80211_OUI_LEN] = { 0x00, 0x00, 0x00 };


struct pkt_file {
	struct sk_buff *skb;
	__kernel_size_t pkt_len;	 //the remainder length of the open_file
	uint8_t *cur_buffer;
	uint8_t *buf_start;
	uint8_t *cur_addr;
	__kernel_size_t buf_len;
};

static uint rtw_remainder_len(struct pkt_file *pfile)
{
	return (pfile->buf_len - ((SIZE_PTR)(pfile->cur_addr) - (SIZE_PTR)(pfile->buf_start)));
}

static void _rtw_open_pktfile (struct sk_buff *skb, struct pkt_file *pfile)
{
	pfile->skb = skb;
	pfile->cur_addr = pfile->buf_start = skb->data;
	pfile->pkt_len = pfile->buf_len = skb->len;

	pfile->cur_buffer = pfile->buf_start ;
}

static uint _rtw_pktfile_read (struct pkt_file *pfile, uint8_t *rmem, uint rlen)
{
	uint	len = 0;

	len =  rtw_remainder_len(pfile);
	len = (rlen > len)? len: rlen;

	if(rmem)
		skb_copy_bits(pfile->skb, pfile->buf_len-pfile->pkt_len, rmem, len);

	pfile->cur_addr += len;
	pfile->pkt_len -= len;

	return len;
}


static void set_qos(struct sk_buff *skb, struct tx_pkt_attrib *pattrib)
{
	struct iphdr *ip_hdr;
	int32_t UserPriority = 0;

	/* get UserPriority from IP hdr */
	if (pattrib->ether_type == 0x0800) {
		ip_hdr = (struct iphdr *) (skb->data + ETH_HLEN);
		/* UserPriority = (ntohs(ip_hdr.tos) >> 5) & 0x3; */
		UserPriority = ip_hdr->tos >> 5;
	} else if (pattrib->ether_type == 0x888e) {
		/*
		 *  "When priority processing of data frames is supported,
		 *  a STA's SME should send EAPOL-Key frames at the highest priority."
		 */
		UserPriority = 7;
	}

	pattrib->tx_priority = UserPriority;
	pattrib->hdrlen = WLAN_HDR_A3_QOS_LEN;
	pattrib->subtype = WIFI_QOS_DATA_TYPE;
}

static int rtw_endofpktfile(struct pkt_file *pfile)
{

	if (pfile->pkt_len == 0) {
		return true;
	}

	return false;
}

static void _init_txservq(struct tx_servq *ptxservq)
{
	INIT_LIST_HEAD(&ptxservq->tx_pending);
	_rtw_init_queue(&ptxservq->sta_pending);
}


void	_rtw_init_sta_xmit_priv(struct sta_xmit_priv *psta_xmitpriv)
{
	memset((unsigned char *)psta_xmitpriv, 0, sizeof (struct sta_xmit_priv));

	spin_lock_init(&psta_xmitpriv->lock);

	/*
	 * for(i = 0 ; i < MAX_NUMBLKS; i++)
	 * 	_init_txservq(&(psta_xmitpriv->blk_q[i]));
	 */

	_init_txservq(&psta_xmitpriv->be_q);
	_init_txservq(&psta_xmitpriv->bk_q);
	_init_txservq(&psta_xmitpriv->vi_q);
	_init_txservq(&psta_xmitpriv->vo_q);
	INIT_LIST_HEAD(&psta_xmitpriv->legacy_dz);
	INIT_LIST_HEAD(&psta_xmitpriv->apsd);
}

int32_t	_rtw_init_xmit_priv(struct xmit_priv *pxmitpriv, struct rtl_priv *rtlpriv)
{
	int i;
	struct xmit_buf *pxmitbuf;
	struct xmit_frame *pxframe;
	int	res = _SUCCESS;
	uint32_t	 max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
	uint32_t	 num_xmit_extbuf = NR_XMIT_EXTBUFF;

	/*
	 * We don't need to memset rtlpriv->XXX to zero, because rtlpriv is allocated by rtw_zvmalloc().
	 * memset((unsigned char *)pxmitpriv, 0, sizeof(struct xmit_priv));
	 */

	spin_lock_init(&pxmitpriv->lock);
	spin_lock_init(&pxmitpriv->lock_sctx);
	sema_init(&pxmitpriv->xmit_sema, 0);
	sema_init(&pxmitpriv->terminate_xmitthread_sema, 0);

	/*
	Please insert all the queue initializaiton using _rtw_init_queue below
	*/

	pxmitpriv->rtlpriv = rtlpriv;

	/*
	 * for(i = 0 ; i < MAX_NUMBLKS; i++)
	 * 	_rtw_init_queue(&pxmitpriv->blk_strms[i]);
	 */

	_rtw_init_queue(&pxmitpriv->be_pending);
	_rtw_init_queue(&pxmitpriv->bk_pending);
	_rtw_init_queue(&pxmitpriv->vi_pending);
	_rtw_init_queue(&pxmitpriv->vo_pending);

	/*
	 * _rtw_init_queue(&pxmitpriv->legacy_dz_queue);
	 * _rtw_init_queue(&pxmitpriv->apsd_queue);
	 */

	_rtw_init_queue(&pxmitpriv->free_xmit_queue);

	/*
	Please allocate memory with the sz = (struct xmit_frame) * NR_XMITFRAME,
	and initialize free_xmit_frame below.
	Please also apply  free_txobj to link_up all the xmit_frames...
	*/

	pxmitpriv->pallocated_frame_buf = rtw_zvmalloc(NR_XMITFRAME * sizeof(struct xmit_frame) + 4);

	if (pxmitpriv->pallocated_frame_buf == NULL) {
		pxmitpriv->pxmit_frame_buf = NULL;
		res = _FAIL;
		goto exit;
	}
	pxmitpriv->pxmit_frame_buf = (uint8_t *)N_BYTE_ALIGMENT((SIZE_PTR)(pxmitpriv->pallocated_frame_buf), 4);
	/*
	 * pxmitpriv->pxmit_frame_buf = pxmitpriv->pallocated_frame_buf + 4 -
	 * 						((SIZE_PTR) (pxmitpriv->pallocated_frame_buf) &3);
	 */

	pxframe = (struct xmit_frame *) pxmitpriv->pxmit_frame_buf;

	for (i = 0; i < NR_XMITFRAME; i++) {
		INIT_LIST_HEAD(&(pxframe->list));

		pxframe->rtlpriv = rtlpriv;
		pxframe->frame_tag = NULL_FRAMETAG;

		pxframe->skb = NULL;

		pxframe->buf_addr = NULL;
		pxframe->pxmitbuf = NULL;

		list_add_tail(&(pxframe->list), &(pxmitpriv->free_xmit_queue.list));

		pxframe++;
	}

	pxmitpriv->free_xmitframe_cnt = NR_XMITFRAME;

	pxmitpriv->frag_len = MAX_FRAG_THRESHOLD;


	/* init xmit_buf */
	_rtw_init_queue(&pxmitpriv->free_xmitbuf_queue);
	_rtw_init_queue(&pxmitpriv->pending_xmitbuf_queue);

	pxmitpriv->pallocated_xmitbuf = rtw_zvmalloc(NR_XMITBUFF * sizeof(struct xmit_buf) + 4);

	if (pxmitpriv->pallocated_xmitbuf == NULL) {
		res = _FAIL;
		goto exit;
	}

	pxmitpriv->pxmitbuf = (uint8_t *)N_BYTE_ALIGMENT((SIZE_PTR)(pxmitpriv->pallocated_xmitbuf), 4);
	/*
	 * pxmitpriv->pxmitbuf = pxmitpriv->pallocated_xmitbuf + 4 -
	 * 						((SIZE_PTR) (pxmitpriv->pallocated_xmitbuf) &3);
	 */

	pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmitbuf;

	for (i = 0; i < NR_XMITBUFF; i++) {
		INIT_LIST_HEAD(&pxmitbuf->list);

		pxmitbuf->priv_data = NULL;
		pxmitbuf->rtlpriv = rtlpriv;
		pxmitbuf->buf_tag = XMITBUF_DATA;

		/* Tx buf allocation may fail sometimes, so sleep and retry. */
		res = rtw_os_xmit_resource_alloc(rtlpriv, pxmitbuf, (MAX_XMITBUF_SZ + XMITBUF_ALIGN_SZ), true);
		if (res == _FAIL) {
			msleep(10);
			res = rtw_os_xmit_resource_alloc(rtlpriv, pxmitbuf, (MAX_XMITBUF_SZ + XMITBUF_ALIGN_SZ), true);
			if (res == _FAIL) {
				goto exit;
			}
		}


		pxmitbuf->flags = XMIT_VO_QUEUE;

		list_add_tail(&pxmitbuf->list, &(pxmitpriv->free_xmitbuf_queue.list));

		pxmitbuf++;

	}

	pxmitpriv->free_xmitbuf_cnt = NR_XMITBUFF;

	/* init xframe_ext queue,  the same count as extbuf  */
	_rtw_init_queue(&pxmitpriv->free_xframe_ext_queue);

	pxmitpriv->xframe_ext_alloc_addr = rtw_zvmalloc(num_xmit_extbuf * sizeof(struct xmit_frame) + 4);

	if (pxmitpriv->xframe_ext_alloc_addr  == NULL) {
		pxmitpriv->xframe_ext = NULL;
		res = _FAIL;
		goto exit;
	}
	pxmitpriv->xframe_ext = (uint8_t *)N_BYTE_ALIGMENT((SIZE_PTR)(pxmitpriv->xframe_ext_alloc_addr), 4);
	pxframe = (struct xmit_frame *)pxmitpriv->xframe_ext;

	for (i = 0; i < num_xmit_extbuf; i++) {
		INIT_LIST_HEAD(&(pxframe->list));

		pxframe->rtlpriv = rtlpriv;
		pxframe->frame_tag = NULL_FRAMETAG;

		pxframe->skb = NULL;

		pxframe->buf_addr = NULL;
		pxframe->pxmitbuf = NULL;

		pxframe->ext_tag = 1;

		list_add_tail(&(pxframe->list), &(pxmitpriv->free_xframe_ext_queue.list));

		pxframe++;
	}
	pxmitpriv->free_xframe_ext_cnt = num_xmit_extbuf;

	/* Init xmit extension buff */
	_rtw_init_queue(&pxmitpriv->free_xmit_extbuf_queue);

	pxmitpriv->pallocated_xmit_extbuf = rtw_zvmalloc(num_xmit_extbuf * sizeof(struct xmit_buf) + 4);

	if (pxmitpriv->pallocated_xmit_extbuf  == NULL) {
		res = _FAIL;
		goto exit;
	}

	pxmitpriv->pxmit_extbuf = (uint8_t *)N_BYTE_ALIGMENT((SIZE_PTR)(pxmitpriv->pallocated_xmit_extbuf), 4);

	pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmit_extbuf;

	for (i = 0; i < num_xmit_extbuf; i++) {
		INIT_LIST_HEAD(&pxmitbuf->list);

		pxmitbuf->priv_data = NULL;
		pxmitbuf->rtlpriv = rtlpriv;
		pxmitbuf->buf_tag = XMITBUF_MGNT;

		res = rtw_os_xmit_resource_alloc(rtlpriv, pxmitbuf, max_xmit_extbuf_size + XMITBUF_ALIGN_SZ, true);
		if (res == _FAIL) {
			res = _FAIL;
			goto exit;
		}


		list_add_tail(&pxmitbuf->list, &(pxmitpriv->free_xmit_extbuf_queue.list));
		pxmitbuf++;

	}

	pxmitpriv->free_xmit_extbuf_cnt = num_xmit_extbuf;


	pxmitbuf = &pxmitpriv->pcmd_xmitbuf;
	if (pxmitbuf) {
		INIT_LIST_HEAD(&pxmitbuf->list);

		pxmitbuf->priv_data = NULL;
		pxmitbuf->rtlpriv = rtlpriv;
		pxmitbuf->buf_tag = XMITBUF_CMD;

		res = rtw_os_xmit_resource_alloc(rtlpriv, pxmitbuf, 0, true);
		if (res == _FAIL) {
			res = _FAIL;
			goto exit;
		}
	}

	rtw_alloc_hwxmits(rtlpriv);
	rtw_init_hwxmits(pxmitpriv->hwxmits);

	for (i = 0; i < 4; i++) {
		pxmitpriv->wmm_para_seq[i] = i;
	}

	pxmitpriv->txirp_cnt = 1;

	sema_init(&(pxmitpriv->tx_retevt), 0);

	tasklet_init(&pxmitpriv->xmit_tasklet,
	     (void(*)(unsigned long))rtl8812au_xmit_tasklet,
	     (unsigned long)rtlpriv);


exit:

	return res;
}



void _rtw_free_xmit_priv (struct xmit_priv *pxmitpriv)
{
	int i;
	struct rtl_priv *rtlpriv = pxmitpriv->rtlpriv;
	struct xmit_frame *pxmitframe = (struct xmit_frame *) pxmitpriv->pxmit_frame_buf;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmitbuf;
	uint32_t	 max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
	uint32_t	 num_xmit_extbuf = NR_XMIT_EXTBUFF;

	if (pxmitpriv->pxmit_frame_buf == NULL)
		goto out;

	for (i = 0; i < NR_XMITFRAME; i++) {
		rtw_os_xmit_complete(rtlpriv, pxmitframe);

		pxmitframe++;
	}

	for (i = 0; i < NR_XMITBUFF; i++) {
		rtw_os_xmit_resource_free(rtlpriv, pxmitbuf, (MAX_XMITBUF_SZ + XMITBUF_ALIGN_SZ), true);

		pxmitbuf++;
	}

	if (pxmitpriv->pallocated_frame_buf) {
		rtw_vmfree(pxmitpriv->pallocated_frame_buf);
	}


	if (pxmitpriv->pallocated_xmitbuf) {
		rtw_vmfree(pxmitpriv->pallocated_xmitbuf);
	}

	/* free xframe_ext queue,  the same count as extbuf  */
	pxmitframe = (struct xmit_frame *)pxmitpriv->xframe_ext;
	if (pxmitframe) {
		for (i = 0; i < num_xmit_extbuf; i++) {
			rtw_os_xmit_complete(rtlpriv, pxmitframe);
			pxmitframe++;
		}
	}
	if (pxmitpriv->xframe_ext_alloc_addr)
		rtw_vmfree(pxmitpriv->xframe_ext_alloc_addr);

	pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmit_extbuf;

	for (i = 0; i < num_xmit_extbuf; i++) {
		rtw_os_xmit_resource_free(rtlpriv, pxmitbuf, (max_xmit_extbuf_size + XMITBUF_ALIGN_SZ), true);

		pxmitbuf++;
	}

	if (pxmitpriv->pallocated_xmit_extbuf) {
		rtw_vmfree(pxmitpriv->pallocated_xmit_extbuf);
	}

	pxmitbuf = &pxmitpriv->pcmd_xmitbuf;
	rtw_os_xmit_resource_free(rtlpriv, pxmitbuf, 0, true);
out:
	;
}

static void update_attrib_vcs_info(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	uint32_t	sz;
	struct tx_pkt_attrib	*pattrib = &pxmitframe->tx_attrib;
	/* struct sta_info	*psta = pattrib->psta; */
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

/*
	if (pattrib->psta) {
		psta = pattrib->psta;
	} else {
		DBG_871X("%s, call rtw_get_stainfo()\n", __func__);
		psta=rtw_get_stainfo(&rtlpriv->stapriv ,&pattrib->ra[0] );
	}

	if (psta==NULL) {
		DBG_871X("%s, psta==NUL\n", __func__);
		return;
	}

	if (!(psta->state &_FW_LINKED)) {
		DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
		return;
	}
*/

	if (pattrib->nr_frags != 1) {
		sz = rtlpriv->xmitpriv.frag_len;
	} else {	/* no frag */
		sz = pattrib->last_txcmdsz;
	}

	/*
	 *  (1) RTS_Threshold is compared to the MPDU, not MSDU.
	 *  (2) If there are more than one frag in  this MSDU, only the first frag uses protection frame.
	 * 		Other fragments are protected by previous fragment.
	 * 		So we only need to check the length of first fragment.
	 */
	if (pmlmeext->cur_wireless_mode < WIRELESS_11_24N) {
		if (sz > rtlpriv->registrypriv.rts_thresh) {
			pattrib->vcs_mode = RTS_CTS;
		} else {
			if (pattrib->rtsen)
				pattrib->vcs_mode = RTS_CTS;
			else if (pattrib->cts2self)
				pattrib->vcs_mode = CTS_TO_SELF;
			else
				pattrib->vcs_mode = NONE_VCS;
		}
	} else {
		while (true) {
			/* IOT action */
			if ((pmlmeinfo->assoc_AP_vendor == HT_IOT_PEER_ATHEROS)
			 && (pattrib->ampdu_en == true)
			 && (rtlpriv->securitypriv.dot11PrivacyAlgrthm == AESCCMP_ENCRYPTION)) {
				pattrib->vcs_mode = CTS_TO_SELF;
				break;
			}

			/* check ERP protection */
			if (pattrib->rtsen || pattrib->cts2self) {
				if (pattrib->rtsen)
					pattrib->vcs_mode = RTS_CTS;
				else if (pattrib->cts2self)
					pattrib->vcs_mode = CTS_TO_SELF;

				break;
			}

			/* check HT op mode */
			if (pattrib->ht_en) {
				uint8_t HTOpMode = pmlmeinfo->HT_protection;
				if ((pmlmeext->cur_bwmode && (HTOpMode == 2 || HTOpMode == 3)) ||
				  (!pmlmeext->cur_bwmode && HTOpMode == 3)) {
					pattrib->vcs_mode = RTS_CTS;
					break;
				}
			}

			/* check rts */
			if (sz > rtlpriv->registrypriv.rts_thresh) {
				pattrib->vcs_mode = RTS_CTS;
				break;
			}

			/* to do list: check MIMO power save condition. */

			/* check AMPDU aggregation for TXOP */
#if 0			/* ULLI we maqy keep this here */
			if ((pattrib->ampdu_en == true) && (!IS_HARDWARE_TYPE_JAGUAR(rtlhal))) {
				pattrib->vcs_mode = RTS_CTS;
				break;
			}
#endif
			pattrib->vcs_mode = NONE_VCS;
			break;
		}
	}
}

static void update_attrib_phy_info(struct tx_pkt_attrib *pattrib, struct sta_info *psta)
{
	pattrib->rtsen = psta->rtsen;
	pattrib->cts2self = psta->cts2self;

	pattrib->mdata = 0;
	pattrib->eosp = 0;
	pattrib->triggered = 0;

	/* qos_en, ht_en, init rate, ,bw, ch_offset, sgi */
	pattrib->qos_en = psta->qos_option;

	pattrib->raid = psta->raid;

	if (psta->vhtpriv.vht_option) {
		pattrib->bwmode = psta->vhtpriv.vht_bwmode;
		pattrib->sgi = psta->vhtpriv.sgi;

		if (TEST_FLAG(psta->vhtpriv.ldpc_cap, LDPC_VHT_ENABLE_TX))
			pattrib->ldpc = 1;

		if (TEST_FLAG(psta->vhtpriv.stbc_cap, STBC_VHT_ENABLE_TX))
			pattrib->stbc = 1;
	} else
	{
		pattrib->bwmode = psta->htpriv.bwmode;
		pattrib->sgi = psta->htpriv.sgi;
	}

	pattrib->ht_en = psta->htpriv.ht_option;
	pattrib->ch_offset = psta->htpriv.ch_offset;
	pattrib->ampdu_en = false;
	/*
	 * if(pattrib->ht_en && psta->htpriv.ampdu_enable) {
	 * 	if(psta->htpriv.agg_enable_bitmap & BIT(pattrib->priority))
	 * 		pattrib->ampdu_en = true;
	 * }
	 */

	pattrib->retry_ctrl = false;


}

static int32_t update_attrib_sec_info(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib, struct sta_info *psta)
{
	int res = _SUCCESS;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	int bmcast = is_multicast_ether_addr(pattrib->ra);

	memset(pattrib->dot118021x_UncstKey.skey,  0, 16);
	memset(pattrib->dot11tkiptxmickey.skey,  0, 16);

	if (psta->ieee8021x_blocked == true) {
		pattrib->encrypt = 0;

		if ((pattrib->ether_type != 0x888e)
		 && (check_fwstate(pmlmepriv, WIFI_MP_STATE) == false)) {
			res = _FAIL;
			goto exit;
		}
	} else {
		GET_ENCRY_ALGO(psecuritypriv, psta, pattrib->encrypt, bmcast);

		switch (psecuritypriv->dot11AuthAlgrthm) {
		case dot11AuthAlgrthm_Open:
		case dot11AuthAlgrthm_Shared:
		case dot11AuthAlgrthm_Auto:
			pattrib->key_idx = (uint8_t)psecuritypriv->dot11PrivacyKeyIndex;
			break;
		case dot11AuthAlgrthm_8021X:
			if (bmcast)
				pattrib->key_idx = (uint8_t)psecuritypriv->dot118021XGrpKeyid;
			else
				pattrib->key_idx = 0;
			break;
		default:
			pattrib->key_idx = 0;
			break;
		}

		/* For WPS 1.0 WEP, driver should not encrypt EAPOL Packet for WPS handshake. */
		if (((pattrib->encrypt == WEP40_ENCRYPTION)
		 || (pattrib->encrypt == WEP104_ENCRYPTION)) && (pattrib->ether_type == 0x888e))
			pattrib->encrypt = NO_ENCRYPTION;

	}

	switch (pattrib->encrypt) {
	case WEP40_ENCRYPTION:
	case WEP104_ENCRYPTION:
		pattrib->iv_len = 4;
		pattrib->icv_len = 4;
		WEP_IV(pattrib->iv, psta->dot11txpn, pattrib->key_idx);
		break;

	case TKIP_ENCRYPTION:
		pattrib->iv_len = 8;
		pattrib->icv_len = 4;

		if (psecuritypriv->busetkipkey == _FAIL) {
			res = _FAIL;
			goto exit;
		}

		if (bmcast)
			TKIP_IV(pattrib->iv, psta->dot11txpn, pattrib->key_idx);
		else
			TKIP_IV(pattrib->iv, psta->dot11txpn, 0);


		memcpy(pattrib->dot11tkiptxmickey.skey, psta->dot11tkiptxmickey.skey, 16);

		break;

	case AESCCMP_ENCRYPTION:
		pattrib->iv_len = 8;
		pattrib->icv_len = 8;

		if (bmcast)
			AES_IV(pattrib->iv, psta->dot11txpn, pattrib->key_idx);
		else
			AES_IV(pattrib->iv, psta->dot11txpn, 0);

		break;

	default:
		pattrib->iv_len = 0;
		pattrib->icv_len = 0;
		break;
	}

	if (pattrib->encrypt > 0)
		memcpy(pattrib->dot118021x_UncstKey.skey, psta->dot118021x_UncstKey.skey, 16);

exit:

	return res;

}

uint8_t	qos_acm(uint8_t acm_mask, uint8_t priority)
{
	uint8_t	change_priority = priority;

	switch (priority) {
	case 0:
	case 3:
		if (acm_mask & BIT(1))
			change_priority = 1;
		break;
	case 1:
	case 2:
		break;
	case 4:
	case 5:
		if (acm_mask & BIT(2))
			change_priority = 0;
		break;
	case 6:
	case 7:
		if (acm_mask & BIT(3))
			change_priority = 5;
		break;
	default:
		DBG_871X("qos_acm(): invalid pattrib->priority: %d!!!\n", priority);
		break;
	}

	return change_priority;
}

static int32_t update_attrib(struct rtl_priv *rtlpriv, struct sk_buff *skb, struct tx_pkt_attrib *pattrib)
{
	uint i;
	struct sta_info *psta = NULL;
	struct ethhdr *etherhdr;

	int bmcast;
	struct sta_priv		*pstapriv = &rtlpriv->stapriv;
	struct security_priv	*psecuritypriv = &rtlpriv->securitypriv;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct qos_priv		*pqospriv = &pmlmepriv->qospriv;
	int res = _SUCCESS;

	etherhdr = (struct ethhdr *) skb->data;

	pattrib->ether_type = ntohs(etherhdr->h_proto);

	memcpy(pattrib->dst, &(etherhdr->h_dest), ETH_ALEN);
	memcpy(pattrib->src, &(etherhdr->h_source), ETH_ALEN);

	if ((check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true) ||
		(check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true)) {
		memcpy(pattrib->ra, pattrib->dst, ETH_ALEN);
		memcpy(pattrib->ta, pattrib->src, ETH_ALEN);
	} else if (check_fwstate(pmlmepriv, WIFI_STATION_STATE)) {
		memcpy(pattrib->ra, get_bssid(pmlmepriv), ETH_ALEN);
		memcpy(pattrib->ta, pattrib->src, ETH_ALEN);
	} else if (check_fwstate(pmlmepriv, WIFI_AP_STATE)) {
		memcpy(pattrib->ra, pattrib->dst, ETH_ALEN);
		memcpy(pattrib->ta, get_bssid(pmlmepriv), ETH_ALEN);
	}

	pattrib->pktlen = skb->len - ETH_HLEN;

	if (ETH_P_IP == pattrib->ether_type) {
#if 0
		/*
		 *  The following is for DHCP and ARP packet, we use cck1M to tx these packets and let LPS awake some time
		 *  to prevent DHCP protocol fail
		 */
		uint8_t tmp[24];
		_rtw_pktfile_read(&pktfile, &tmp[0], 24);
#endif
		/* ULLI : get udp header */
		struct udphdr *udp = (struct udphdr *) skb->data + ETH_HLEN + 20;
		pattrib->dhcp_pkt = 0;

		if (pattrib->pktlen > 282) {	/* MINIMUM_DHCP_PACKET_SIZE) { */
			if (ETH_P_IP == pattrib->ether_type) {
				/* IP header */
				if (((udp->source == 68) && (udp->dest == 67)) ||
				    ((udp->source == 67) && (udp->dest == 68))) {
					/* 68 : UDP BOOTP client */
					/* 67 : UDP BOOTP server */
					/*
					 * Use low rate to send DHCP packet.
					 * if(pMgntInfo->IOTAction & HT_IOT_ACT_WA_IOT_Broadcom) {
					 * 	tcb_desc->DataRate = MgntQuery_TxRateExcludeCCKRates(ieee);//0xc;//ofdm 6m
					 * 	tcb_desc->bTxDisableRateFallBack = false;
					 * } else
					 * 	pTcb->DataRate = rtlpriv->MgntInfo.LowestBasicRate;
					 * RTPRINT(FDM, WA_IOT, ("DHCP TranslateHeader(), pTcb->DataRate = 0x%x\n", pTcb->DataRate));
					 */
					pattrib->dhcp_pkt = 1;
				}
			}
		}
	} else if (0x888e == pattrib->ether_type) {
		DBG_871X_LEVEL(_drv_always_, "send eapol packet\n");
	}

	/* If EAPOL , ARP , OR DHCP packet, driver must be in active mode. */
	if ((pattrib->ether_type == 0x0806) || (pattrib->ether_type == 0x888e) || (pattrib->dhcp_pkt == 1)) {
		rtw_lps_ctrl_wk_cmd(rtlpriv, LPS_CTRL_SPECIAL_PACKET, 1);
	}

	bmcast = is_multicast_ether_addr(pattrib->ra);

	/* get sta_info */
	if (bmcast) {
		psta = rtw_get_bcmc_stainfo(rtlpriv);
	} else {
		psta = rtw_get_stainfo(pstapriv, pattrib->ra);
		if (psta == NULL) {	/* if we cannot get psta => drrp the pkt */
			res = _FAIL;
			goto exit;
		} else if ((check_fwstate(pmlmepriv, WIFI_AP_STATE) == true) && (!(psta->state & _FW_LINKED))) {
			res = _FAIL;
			goto exit;
		}
	}

	if (psta == NULL) {	/* if we cannot get psta => drop the pkt */
		res = _FAIL;
		goto exit;
	}

	if (!(psta->state & _FW_LINKED))	{
		DBG_871X("%s, psta("MAC_FMT")->state(0x%x) != _FW_LINKED\n", __func__, MAC_ARG(psta->hwaddr), psta->state);
		return _FAIL;
	}



	/* TODO:spinlock_t */
	if (update_attrib_sec_info(rtlpriv, pattrib, psta) == _FAIL) {
		res = _FAIL;
		goto exit;
	}

	update_attrib_phy_info(pattrib, psta);

	pattrib->mac_id = psta->mac_id;
	/* DBG_8192C("%s ==> mac_id(%d)\n",__FUNCTION__,pattrib->mac_id ); */

	pattrib->psta = psta;
	/* TODO:_unlock */

	pattrib->pctrl = 0;

	pattrib->ack_policy = 0;
	/* get ether_hdr_len */
	pattrib->pkt_hdrlen = ETH_HLEN; /*(pattrib->ether_type == 0x8100) ? (14 + 4 ): 14; //vlan tag */

	pattrib->hdrlen = WLAN_HDR_A3_LEN;
	pattrib->subtype = WIFI_DATA_TYPE;
	pattrib->tx_priority = 0;

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE|WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE)) {
		if (pattrib->qos_en)
			set_qos(skb, pattrib);
	} else {
		if (pqospriv->qos_option) {
			set_qos(skb, pattrib);

			if (pmlmepriv->acm_mask != 0) {
				pattrib->tx_priority = qos_acm(pmlmepriv->acm_mask, pattrib->tx_priority);
			}
		}
	}

	/* pattrib->priority = 5; //force to used VI queue, for testing */

	if (pattrib->encrypt &&
	    ((rtlpriv->securitypriv.sw_encrypt == true) || (psecuritypriv->hw_decrypted == false))) {
		pattrib->bswenc = true;
	} else {
		pattrib->bswenc = false;
	}

	rtw_set_tx_chksum_offload(skb, pattrib);

exit:

	return res;
}

static int32_t xmitframe_addmic(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	int curfragnum, length;
	uint8_t	*pframe, *payload, mic[8];
	struct	mic_data		micdata;
	/* struct	sta_info		*stainfo; */
	struct	qos_priv   *pqospriv = &(rtlpriv->mlmepriv.qospriv);
	struct	tx_pkt_attrib	 *pattrib = &pxmitframe->tx_attrib;
	struct 	security_priv	*psecuritypriv = &rtlpriv->securitypriv;
	struct	xmit_priv		*pxmitpriv = &rtlpriv->xmitpriv;
	uint8_t priority[4] = {0x0, 0x0, 0x0, 0x0};
	uint8_t hw_hdr_offset = 0;
	int bmcst = is_multicast_ether_addr(pattrib->ra);

/*
	if(pattrib->psta)
	{
		stainfo = pattrib->psta;
	}
	else
	{
		DBG_871X("%s, call rtw_get_stainfo()\n", __func__);
		stainfo=rtw_get_stainfo(&rtlpriv->stapriv ,&pattrib->ra[0]);
	}

	if(stainfo==NULL)
	{
		DBG_871X("%s, psta==NUL\n", __func__);
		return _FAIL;
	}

	if(!(stainfo->state &_FW_LINKED))
	{
		DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, stainfo->state);
		return _FAIL;
	}
*/

	hw_hdr_offset = TXDESC_SIZE + (pxmitframe->pkt_offset * PACKET_OFFSET_SZ);;

	if (pattrib->encrypt == TKIP_ENCRYPTION) {	/* if(psecuritypriv->dot11PrivacyAlgrthm==_TKIP_PRIVACY_) */
		/*
		 * encode mic code
		 * if(stainfo!= NULL)
		 */
		{
			uint8_t null_key[16] = {
				0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
				0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

			pframe = pxmitframe->buf_addr + hw_hdr_offset;

			if (bmcst) {
				if (_rtw_memcmp(psecuritypriv->dot118021XGrptxmickey[psecuritypriv->dot118021XGrpKeyid].skey, null_key, 16) == true) {
					/*
					 * DbgPrint("\nxmitframe_addmic:stainfo->dot11tkiptxmickey==0\n");
					 * msleep(10);
					 */
					return _FAIL;
				}
				/* start to calculate the mic code */
				rtw_secmicsetkey(&micdata, psecuritypriv->dot118021XGrptxmickey[psecuritypriv->dot118021XGrpKeyid].skey);
			} else 	{
				if (_rtw_memcmp(&pattrib->dot11tkiptxmickey.skey[0], null_key, 16) == true) {
					/*
					 * DbgPrint("\nxmitframe_addmic:stainfo->dot11tkiptxmickey==0\n");
					 * msleep(10);
					 */
					return _FAIL;
				}
				/* start to calculate the mic code */
				rtw_secmicsetkey(&micdata, &pattrib->dot11tkiptxmickey.skey[0]);
			}

			if (pframe[1] & 1) {	/* ToDS==1 */
				rtw_secmicappend(&micdata, &pframe[16], 6);  /* DA */
				if (pframe[1] & 2)  /* From Ds==1 */
					rtw_secmicappend(&micdata, &pframe[24], 6);
				else
					rtw_secmicappend(&micdata, &pframe[10], 6);
			} else {	/* ToDS==0 */
				rtw_secmicappend(&micdata, &pframe[4], 6);   /* DA */
				if (pframe[1]&2)  /* From Ds==1 */
					rtw_secmicappend(&micdata, &pframe[16], 6);
				else
					rtw_secmicappend(&micdata, &pframe[10], 6);

			}

			/* if(pqospriv->qos_option==1) */
			if (pattrib->qos_en)
				priority[0] = (uint8_t)pxmitframe->tx_attrib.tx_priority;


			rtw_secmicappend(&micdata, &priority[0], 4);

			payload = pframe;

			for (curfragnum = 0; curfragnum < pattrib->nr_frags; curfragnum++) {
				payload = (uint8_t *)RND4((SIZE_PTR)(payload));

				payload = payload+pattrib->hdrlen+pattrib->iv_len;
				if ((curfragnum+1) == pattrib->nr_frags) {
					length = pattrib->last_txcmdsz-pattrib->hdrlen-pattrib->iv_len-((pattrib->bswenc) ? pattrib->icv_len : 0);
					rtw_secmicappend(&micdata, payload, length);
					payload = payload + length;
				} else {
					length = pxmitpriv->frag_len-pattrib->hdrlen-pattrib->iv_len-((pattrib->bswenc) ? pattrib->icv_len : 0);
					rtw_secmicappend(&micdata, payload, length);
					payload = payload + length+pattrib->icv_len;
				}
			}
			rtw_secgetmic(&micdata, &(mic[0]));

			memcpy(payload, &(mic[0]), 8);
			pattrib->last_txcmdsz += 8;

			payload = payload-pattrib->last_txcmdsz + 8;
			for (curfragnum = 0; curfragnum < pattrib->last_txcmdsz; curfragnum = curfragnum + 8)
				;
			}
/*
			else{
				RT_TRACE(_module_rtl871x_xmit_c_,_drv_err_,("xmitframe_addmic: rtw_get_stainfo==NULL!!!\n"));
			}
*/
	}



	return _SUCCESS;
}

static int32_t xmitframe_swencrypt(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	struct	tx_pkt_attrib	 *pattrib = &pxmitframe->tx_attrib;
	/* struct 	security_priv	*psecuritypriv=&rtlpriv->securitypriv; */

	/* if((psecuritypriv->sw_encrypt)||(pattrib->bswenc)) */
	if (pattrib->bswenc) {
		/* DBG_871X("start xmitframe_swencrypt\n"); */
		switch (pattrib->encrypt) {
		case WEP40_ENCRYPTION:
		case WEP104_ENCRYPTION:
			rtw_wep_encrypt(rtlpriv, (uint8_t *)pxmitframe);
			break;
		case TKIP_ENCRYPTION:
			rtw_tkip_encrypt(rtlpriv, (uint8_t *)pxmitframe);
			break;
		case AESCCMP_ENCRYPTION:
			rtw_aes_encrypt(rtlpriv, (uint8_t *)pxmitframe);
			break;
		default:
				break;
		}

	} else {
		;
	}

	return _SUCCESS;
}

int32_t rtw_make_wlanhdr (struct rtl_priv *rtlpriv , uint8_t *hdr, struct tx_pkt_attrib *pattrib)
{
	u16 *qc;

	struct rtw_ieee80211_hdr *pwlanhdr = (struct rtw_ieee80211_hdr *)hdr;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct qos_priv *pqospriv = &pmlmepriv->qospriv;
	uint8_t qos_option = false;

	int res = _SUCCESS;
	u16 *fctrl = &pwlanhdr->frame_ctl;

	/* struct sta_info *psta; */

	/* int bmcst = is_multicast_ether_addr(pattrib->ra); */



/*
	psta = rtw_get_stainfo(&rtlpriv->stapriv, pattrib->ra);
	if(pattrib->psta != psta)
	{
		DBG_871X("%s, pattrib->psta(%p) != psta(%p)\n", __func__, pattrib->psta, psta);
		return;
	}

	if(psta==NULL)
	{
		DBG_871X("%s, psta==NUL\n", __func__);
		return _FAIL;
	}

	if(!(psta->state &_FW_LINKED))
	{
		DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
		return _FAIL;
	}
*/

	memset(hdr, 0, WLANHDR_OFFSET);

	SetFrameSubType(fctrl, pattrib->subtype);

	if (pattrib->subtype & WIFI_DATA_TYPE) {
		if ((check_fwstate(pmlmepriv,  WIFI_STATION_STATE) == true)) {
			/* to_ds = 1, fr_ds = 0; */
			{
				/* Data transfer to AP */
				SetToDs(fctrl);
				memcpy(pwlanhdr->addr1, get_bssid(pmlmepriv), ETH_ALEN);
				memcpy(pwlanhdr->addr2, pattrib->src, ETH_ALEN);
				memcpy(pwlanhdr->addr3, pattrib->dst, ETH_ALEN);
			}

			if (pqospriv->qos_option)
				qos_option = true;

		} else if ((check_fwstate(pmlmepriv,  WIFI_AP_STATE) == true)) {
			/* to_ds = 0, fr_ds = 1; */
			SetFrDs(fctrl);
			memcpy(pwlanhdr->addr1, pattrib->dst, ETH_ALEN);
			memcpy(pwlanhdr->addr2, get_bssid(pmlmepriv), ETH_ALEN);
			memcpy(pwlanhdr->addr3, pattrib->src, ETH_ALEN);

			if (pattrib->qos_en)
				qos_option = true;
		} else if ((check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) == true)
		    || (check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) == true)) {
			memcpy(pwlanhdr->addr1, pattrib->dst, ETH_ALEN);
			memcpy(pwlanhdr->addr2, pattrib->src, ETH_ALEN);
			memcpy(pwlanhdr->addr3, get_bssid(pmlmepriv), ETH_ALEN);

			if (pattrib->qos_en)
				qos_option = true;
		} else {
			res = _FAIL;
			goto exit;
		}

		if (pattrib->mdata)
			SetMData(fctrl);

		if (pattrib->encrypt)
			SetPrivacy(fctrl);

		if (qos_option) {
			qc = (unsigned short *)(hdr + pattrib->hdrlen - 2);

			if (pattrib->tx_priority)
				SetPriority(qc, pattrib->tx_priority);

			SetEOSP(qc, pattrib->eosp);

			SetAckpolicy(qc, pattrib->ack_policy);
		}

		/* TODO: fill HT Control Field */

		/* Update Seq Num will be handled by f/w */
		{
			struct sta_info *psta;
			psta = rtw_get_stainfo(&rtlpriv->stapriv, pattrib->ra);
			if (pattrib->psta != psta) {
				DBG_871X("%s, pattrib->psta(%p) != psta(%p)\n", __func__, pattrib->psta, psta);
				return _FAIL;
			}

			if (psta == NULL) {
				DBG_871X("%s, psta==NUL\n", __func__);
				return _FAIL;
			}

			if (!(psta->state & _FW_LINKED)) {
				DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
				return _FAIL;
			}

			/* if(psta) */
			{

				psta->sta_xmitpriv.txseq_tid[pattrib->tx_priority]++;
				psta->sta_xmitpriv.txseq_tid[pattrib->tx_priority] &= 0xFFF;

				pattrib->seqnum = psta->sta_xmitpriv.txseq_tid[pattrib->tx_priority];

				SetSeqNum(hdr, pattrib->seqnum);

				/* check if enable ampdu */
				if (pattrib->ht_en && psta->htpriv.ampdu_enable) {
					if (psta->htpriv.agg_enable_bitmap & BIT(pattrib->tx_priority))
					pattrib->ampdu_en = true;
				}

				/* re-check if enable ampdu by BA_starting_seqctrl */
				if (pattrib->ampdu_en == true) 	{
					u16 tx_seq;

					tx_seq = psta->BA_starting_seqctrl[pattrib->tx_priority & 0x0f];

					/* check BA_starting_seqctrl */
					if (SN_LESS(pattrib->seqnum, tx_seq)) {
						/* DBG_871X("tx ampdu seqnum(%d) < tx_seq(%d)\n", pattrib->seqnum, tx_seq); */
						pattrib->ampdu_en = false;	/* AGG BK */
					} else if (SN_EQUAL(pattrib->seqnum, tx_seq)) {
						psta->BA_starting_seqctrl[pattrib->tx_priority & 0x0f] = (tx_seq+1)&0xfff;

						pattrib->ampdu_en = true;	/* AGG EN */
					} else 	{
						/* DBG_871X("tx ampdu over run\n"); */
						psta->BA_starting_seqctrl[pattrib->tx_priority & 0x0f] = (pattrib->seqnum+1)&0xfff;
						pattrib->ampdu_en = true;	/* AGG EN */
					}

				}
			}
		}

	} else {

	}

exit:

	return res;
}

int32_t rtw_txframes_pending(struct rtl_priv *rtlpriv)
{
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;

	return (!list_empty(&pxmitpriv->be_pending.list) ||
		!list_empty(&pxmitpriv->bk_pending.list) ||
		!list_empty(&pxmitpriv->vi_pending.list) ||
		!list_empty(&pxmitpriv->vo_pending.list));
}

bool rtw_txframes_sta_ac_pending(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib)
{
	struct sta_info *psta;
	struct tx_servq *ptxservq;
	int priority = pattrib->tx_priority;
/*
	if(pattrib->psta)
	{
		psta = pattrib->psta;
	}
	else
	{
		DBG_871X("%s, call rtw_get_stainfo()\n", __func__);
		psta=rtw_get_stainfo(&rtlpriv->stapriv ,&pattrib->ra[0]);
	}
*/
	psta = rtw_get_stainfo(&rtlpriv->stapriv, pattrib->ra);
	if (pattrib->psta != psta) {
		DBG_871X("%s, pattrib->psta(%p) != psta(%p)\n", __func__, pattrib->psta, psta);
		return 0;
	}

	if (psta == NULL) {
		DBG_871X("%s, psta==NUL\n", __func__);
		return 0;
	}

	if (!(psta->state & _FW_LINKED)) {
		DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
		return 0;
	}

	switch (priority) {
	case 1:
	case 2:
		ptxservq = &(psta->sta_xmitpriv.bk_q);
		break;
	case 4:
	case 5:
		ptxservq = &(psta->sta_xmitpriv.vi_q);
		break;
	case 6:
	case 7:
		ptxservq = &(psta->sta_xmitpriv.vo_q);
		break;
	case 0:
	case 3:
	default:
		ptxservq = &(psta->sta_xmitpriv.be_q);
	break;

	}

	/* ULLI : return true if !list_empty() */

	return list_empty (&(ptxservq->sta_pending.list)) ? false : true;
}

/*
 * Calculate wlan 802.11 packet MAX size from pkt_attrib
 * This function doesn't consider fragment case
 */
uint32_t	 rtw_calculate_wlan_pkt_size_by_attribue(struct tx_pkt_attrib *pattrib)
{
	uint32_t	len = 0;

	len = pattrib->hdrlen + pattrib->iv_len;	/* WLAN Header and IV */
	len += SNAP_SIZE + sizeof(u16);		/* LLC */
	len += pattrib->pktlen;
	if (pattrib->encrypt == TKIP_ENCRYPTION)
		len += 8;	/* MIC */
	len += ((pattrib->bswenc) ? pattrib->icv_len : 0);	/* ICV */

	return len;
}

/*

This sub-routine will perform all the following:

1. remove 802.3 header.
2. create wlan_header, based on the info in pxmitframe
3. append sta's iv/ext-iv
4. append LLC
5. move frag chunk from pframe to pxmitframe->mem
6. apply sw-encrypt, if necessary.

*/
int32_t rtw_xmitframe_coalesce(struct rtl_priv *rtlpriv, struct sk_buff *skb,
			       struct xmit_frame *pxmitframe)
{
	struct pkt_file pktfile;
	int32_t frg_inx, frg_len, mpdu_len, llc_sz, mem_sz;
	__kernel_size_t addr;
	u8 *mem_start;
	u8 hw_hdr_offset;
	struct xmit_priv	*pxmitpriv = &rtlpriv->xmitpriv;
	struct tx_pkt_attrib	*pattrib = &pxmitframe->tx_attrib;
	uint8_t *pbuf_start;
	int32_t res = _SUCCESS;

	if (pxmitframe->buf_addr == NULL) {
		DBG_8192C("==> %s buf_addr==NULL \n", __FUNCTION__);
		return _FAIL;
	}

	pbuf_start = pxmitframe->buf_addr;
	hw_hdr_offset =  TXDESC_SIZE + (pxmitframe->pkt_offset * PACKET_OFFSET_SZ);
	mem_start = pbuf_start + hw_hdr_offset;

	if (rtw_make_wlanhdr(rtlpriv, mem_start, pattrib) == _FAIL) {
		DBG_8192C("rtw_xmitframe_coalesce: rtw_make_wlanhdr fail; drop pkt\n");
		res = _FAIL;
		goto exit;
	}

	_rtw_open_pktfile(skb, &pktfile);
	_rtw_pktfile_read(&pktfile, NULL, pattrib->pkt_hdrlen);

	frg_inx = 0;
	frg_len = pxmitpriv->frag_len - 4;	/* 2346-4 = 2342 */

	while (1) {
		u8 *pframe;
		llc_sz = 0;

		mpdu_len = frg_len;

		pframe = mem_start;

		SetMFrag(mem_start);

		pframe += pattrib->hdrlen;
		mpdu_len -= pattrib->hdrlen;

		/* adding icv, if necessary... */
		if (pattrib->iv_len) {
			memcpy(pframe, pattrib->iv, pattrib->iv_len);
			pframe += pattrib->iv_len;
			mpdu_len -= pattrib->iv_len;
		}

		if (frg_inx == 0) {
			llc_sz = rtw_put_snap(pframe, pattrib->ether_type);
			pframe += llc_sz;
			mpdu_len -= llc_sz;
		}

		if ((pattrib->icv_len > 0) && (pattrib->bswenc)) {
			mpdu_len -= pattrib->icv_len;
		}


		if (is_multicast_ether_addr(pattrib->ra)) {
			/* don't do fragment to broadcat/multicast packets */
			mem_sz = _rtw_pktfile_read(&pktfile, pframe, pattrib->pktlen);
		} else {
			mem_sz = _rtw_pktfile_read(&pktfile, pframe, mpdu_len);
		}

		pframe += mem_sz;

		if ((pattrib->icv_len > 0) && (pattrib->bswenc)) {
			memcpy(pframe, pattrib->icv, pattrib->icv_len);
			pframe += pattrib->icv_len;
		}

		frg_inx++;

		if (is_multicast_ether_addr(pattrib->ra) || (rtw_endofpktfile(&pktfile) == true)) {
			pattrib->nr_frags = frg_inx;

			pattrib->last_txcmdsz = pattrib->hdrlen + pattrib->iv_len + ((pattrib->nr_frags == 1) ? llc_sz : 0) +
					((pattrib->bswenc) ? pattrib->icv_len : 0) + mem_sz;

			ClearMFrag(mem_start);

			break;
		} else {
			;
		}

		addr = (SIZE_PTR)(pframe);

		mem_start = (unsigned char *)RND4(addr) + hw_hdr_offset;
		memcpy(mem_start, pbuf_start + hw_hdr_offset, pattrib->hdrlen);

	}

	if (xmitframe_addmic(rtlpriv, pxmitframe) == _FAIL) {
		DBG_8192C("xmitframe_addmic(rtlpriv, pxmitframe)==_FAIL\n");
		res = _FAIL;
		goto exit;
	}

	xmitframe_swencrypt(rtlpriv, pxmitframe);

	if (is_multicast_ether_addr(pattrib->ra) == false)
		update_attrib_vcs_info(rtlpriv, pxmitframe);
	else
		pattrib->vcs_mode = NONE_VCS;

exit:

	return res;
}

/* Logical Link Control(LLC) SubNetwork Attachment Point(SNAP) header
 * IEEE LLC/SNAP header contains 8 octets
 * First 3 octets comprise the LLC portion
 * SNAP portion, 5 octets, is divided into two fields:
 *	Organizationally Unique Identifier(OUI), 3 octets,
 *	type, defined by that organization, 2 octets.
 */
int32_t rtw_put_snap(uint8_t *data, u16 h_proto)
{
	struct ieee80211_snap_hdr *snap;
	uint8_t *oui;

	snap = (struct ieee80211_snap_hdr *)data;
	snap->dsap = 0xaa;
	snap->ssap = 0xaa;
	snap->ctrl = 0x03;

	if (h_proto == 0x8137 || h_proto == 0x80f3)
		oui = P802_1H_OUI;
	else
		oui = RFC1042_OUI;

	snap->oui[0] = oui[0];
	snap->oui[1] = oui[1];
	snap->oui[2] = oui[2];

	*(u16 *)(data + SNAP_SIZE) = htons(h_proto);

	return SNAP_SIZE + sizeof(u16);
}

void rtw_update_protection(struct rtl_priv *rtlpriv, uint8_t *ie, uint ie_len)
{

	uint	protection;
	uint8_t	*perp;
	int	 erp_len;
	struct	xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct	registry_priv *pregistrypriv = &rtlpriv->registrypriv;

	switch (pxmitpriv->vcs_setting) {
	case DISABLE_VCS:
		pxmitpriv->vcs = NONE_VCS;
		break;

	case ENABLE_VCS:
		break;

	case AUTO_VCS:
	default:
		perp = rtw_get_ie(ie, _ERPINFO_IE_, &erp_len, ie_len);
		if (perp == NULL) {
			pxmitpriv->vcs = NONE_VCS;
		} else {
			protection = (*(perp + 2)) & BIT(1);
			if (protection) {
				if (RTS_CTS == RTS_CTS)
					pxmitpriv->vcs = RTS_CTS;
				else
					pxmitpriv->vcs = CTS_TO_SELF;
			} else
				pxmitpriv->vcs = NONE_VCS;
		}

		break;

	}

}

void rtw_count_tx_stats(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe, int sz)
{
	struct sta_info *psta = NULL;
	struct stainfo_stats *pstats = NULL;
	struct xmit_priv	*pxmitpriv = &rtlpriv->xmitpriv;
	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;

	if ((pxmitframe->frame_tag&0x0f) == DATA_FRAMETAG) {
		pxmitpriv->tx_bytes += sz;
		pmlmepriv->LinkDetectInfo.NumTxOkInPeriod += pxmitframe->agg_num;

		psta = pxmitframe->tx_attrib.psta;
		if (psta) {
			pstats = &psta->sta_stats;
			pstats->tx_pkts += pxmitframe->agg_num;
			pstats->tx_bytes += sz;
		}
	}
}

struct xmit_buf *rtw_alloc_cmd_xmitbuf(struct xmit_priv *pxmitpriv, uint32_t	 buffsize)
{
	struct xmit_buf *pxmitbuf =  NULL;

	pxmitbuf = &pxmitpriv->pcmd_xmitbuf;

	if (pxmitbuf !=  NULL) {
		if (rtw_os_xmit_resource_alloc(pxmitpriv->rtlpriv, pxmitbuf, (buffsize + XMITBUF_ALIGN_SZ), false) == _FAIL) {
			return NULL;
		}

		pxmitbuf->alloc_sz = buffsize + XMITBUF_ALIGN_SZ;

		pxmitbuf->priv_data = NULL;


		if (pxmitbuf->sctx) {
			DBG_871X("%s pxmitbuf->sctx is not NULL\n", __func__);
			rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_BUF_ALLOC);
		}
	} else {
		DBG_871X("%s fail, no xmitbuf available !!!\n", __func__);
	}

	return pxmitbuf;
}

int32_t	rtw_free_cmd_xmitbuf(struct xmit_priv *pxmitpriv)
{
	struct xmit_buf *pxmitbuf =  NULL;

	pxmitbuf = &pxmitpriv->pcmd_xmitbuf;
	if (pxmitbuf == NULL) {
		DBG_871X("%s fail, no xmitbuf available !!!\n", __func__);
		return _FAIL;
	} else {
		rtw_os_xmit_resource_free(pxmitbuf->rtlpriv, pxmitbuf, pxmitbuf->alloc_sz, false);
	}

	return _SUCCESS;
}

struct xmit_frame *rtw_alloc_cmdxmitframe(struct xmit_priv *pxmitpriv, uint32_t	 buffsize)
{
	struct xmit_frame		*pcmdframe;
	struct xmit_buf		*pxmitbuf;

	pcmdframe = rtw_alloc_xmitframe(pxmitpriv);
	if (pcmdframe == NULL) {
		DBG_871X("%s, alloc xmitframe fail\n", __FUNCTION__);
		return NULL;
	}

	pxmitbuf = rtw_alloc_cmd_xmitbuf(pxmitpriv, buffsize);
	if (pxmitbuf == NULL) {
		DBG_871X("%s, alloc xmitbuf fail\n", __FUNCTION__);
		rtw_free_xmitframe(pxmitpriv, pcmdframe);
		return NULL;
	}

	pcmdframe->frame_tag = MGNT_FRAMETAG;

	pcmdframe->pxmitbuf = pxmitbuf;

	pcmdframe->buf_addr = pxmitbuf->pbuf;

	pxmitbuf->priv_data = pcmdframe;

	return pcmdframe;

}

void	rtw_free_cmdxmitframe(struct xmit_priv *pxmitpriv, struct xmit_frame *pxmitframe)
{
	rtw_free_xmitframe(pxmitpriv, pxmitframe);
	rtw_free_cmd_xmitbuf(pxmitpriv);
}

struct xmit_buf *rtw_alloc_xmitbuf_ext(struct xmit_priv *pxmitpriv)
{
	unsigned long flags;
	struct xmit_buf *pxmitbuf =  NULL;
	struct list_head *plist, *phead;
	struct __queue *pfree_queue = &pxmitpriv->free_xmit_extbuf_queue;

	spin_lock_irqsave(&pfree_queue->lock, flags);

	if (list_empty(&pfree_queue->list)) {
		pxmitbuf = NULL;
	} else {

		phead = get_list_head(pfree_queue);

		plist = get_next(phead);

		pxmitbuf = container_of(plist, struct xmit_buf, list);

		list_del_init(&(pxmitbuf->list));
	}

	if (pxmitbuf !=  NULL) {
		pxmitpriv->free_xmit_extbuf_cnt--;


		pxmitbuf->priv_data = NULL;


		if (pxmitbuf->sctx) {
			DBG_871X("%s pxmitbuf->sctx is not NULL\n", __func__);
			rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_BUF_ALLOC);
		}

	}

	spin_unlock_irqrestore(&pfree_queue->lock, flags);

	return pxmitbuf;
}

int32_t rtw_free_xmitbuf_ext(struct xmit_priv *pxmitpriv, struct xmit_buf *pxmitbuf)
{
	unsigned long flags;
	struct __queue *pfree_queue = &pxmitpriv->free_xmit_extbuf_queue;

	if (pxmitbuf == NULL) {
		return _FAIL;
	}

	spin_lock_irqsave(&pfree_queue->lock, flags);

	list_del_init(&pxmitbuf->list);

	list_add_tail(&(pxmitbuf->list), get_list_head(pfree_queue));
	pxmitpriv->free_xmit_extbuf_cnt++;

	spin_unlock_irqrestore(&pfree_queue->lock, flags);

	return _SUCCESS;
}

struct xmit_buf *rtw_alloc_xmitbuf(struct xmit_priv *pxmitpriv)
{
	unsigned long flags;
	struct xmit_buf *pxmitbuf =  NULL;
	struct list_head *plist, *phead;
	struct __queue *pfree_xmitbuf_queue = &pxmitpriv->free_xmitbuf_queue;

	/* DBG_871X("+rtw_alloc_xmitbuf\n"); */

	spin_lock_irqsave(&pfree_xmitbuf_queue->lock, flags);

	if (list_empty(&pfree_xmitbuf_queue->list)) {
		pxmitbuf = NULL;
	} else {

		phead = get_list_head(pfree_xmitbuf_queue);

		plist = get_next(phead);

		pxmitbuf = container_of(plist, struct xmit_buf, list);

		list_del_init(&(pxmitbuf->list));
	}

	if (pxmitbuf !=  NULL) {
		pxmitpriv->free_xmitbuf_cnt--;
		/* DBG_871X("alloc, free_xmitbuf_cnt=%d\n", pxmitpriv->free_xmitbuf_cnt); */

		pxmitbuf->priv_data = NULL;


		if (pxmitbuf->sctx) {
			DBG_871X("%s pxmitbuf->sctx is not NULL\n", __func__);
			rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_BUF_ALLOC);
		}
	}

	spin_unlock_irqrestore(&pfree_xmitbuf_queue->lock, flags);

	return pxmitbuf;
}

int32_t rtw_free_xmitbuf(struct xmit_priv *pxmitpriv, struct xmit_buf *pxmitbuf)
{
	unsigned long flags;
	struct __queue *pfree_xmitbuf_queue = &pxmitpriv->free_xmitbuf_queue;

	/* DBG_871X("+rtw_free_xmitbuf\n"); */

	if (pxmitbuf == NULL) {
		return _FAIL;
	}

	if (pxmitbuf->sctx) {
		DBG_871X("%s pxmitbuf->sctx is not NULL\n", __func__);
		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_BUF_FREE);
	}

	if (pxmitbuf->buf_tag == XMITBUF_CMD) {
		;
	} else if (pxmitbuf->buf_tag == XMITBUF_MGNT) {
		rtw_free_xmitbuf_ext(pxmitpriv, pxmitbuf);
	} else {
		spin_lock_irqsave(&pfree_xmitbuf_queue->lock, flags);

		list_del_init(&pxmitbuf->list);

		list_add_tail(&(pxmitbuf->list), get_list_head(pfree_xmitbuf_queue));

		pxmitpriv->free_xmitbuf_cnt++;
		/* DBG_871X("FREE, free_xmitbuf_cnt=%d\n", pxmitpriv->free_xmitbuf_cnt); */
		spin_unlock_irqrestore(&pfree_xmitbuf_queue->lock, flags);
	}

	return _SUCCESS;
}

void rtw_init_xmitframe(struct xmit_frame *pxframe)
{
	if (pxframe !=  NULL) {		/* default value setting */
		pxframe->buf_addr = NULL;
		pxframe->pxmitbuf = NULL;

		memset(&pxframe->tx_attrib, 0, sizeof(struct tx_pkt_attrib));
		/* pxframe->attrib.psta = NULL; */

		pxframe->frame_tag = DATA_FRAMETAG;

		pxframe->skb = NULL;
		pxframe->pkt_offset = 1;	/*default use pkt_offset to fill tx desc */

		pxframe->agg_num = 1;
	}
}

/*
Calling context:
1. OS_TXENTRY
2. RXENTRY (rx_thread or RX_ISR/RX_CallBack)

If we turn on USE_RXTHREAD, then, no need for critical section.
Otherwise, we must use _enter/_exit critical to protect free_xmit_queue...

Must be very very cautious...

*/
struct xmit_frame *rtw_alloc_xmitframe(struct xmit_priv *pxmitpriv)	/* (struct __queue *pfree_xmit_queue) */
{
	/*
		Please remember to use all the osdep_service api,
		and lock/unlock or _enter/_exit critical to protect
		pfree_xmit_queue
	*/

	struct xmit_frame *pxframe = NULL;
	struct list_head *plist, *phead;
	struct __queue *pfree_xmit_queue = &pxmitpriv->free_xmit_queue;

	spin_lock_bh(&pfree_xmit_queue->lock);

	if (list_empty(&pfree_xmit_queue->list)) {
		pxframe =  NULL;
	} else {
		phead = get_list_head(pfree_xmit_queue);

		plist = get_next(phead);

		pxframe = container_of(plist, struct xmit_frame, list);

		list_del_init(&(pxframe->list));
		pxmitpriv->free_xmitframe_cnt--;
	}

	spin_unlock_bh(&pfree_xmit_queue->lock);

	rtw_init_xmitframe(pxframe);

	return pxframe;
}

struct xmit_frame *rtw_alloc_xmitframe_ext(struct xmit_priv *pxmitpriv)
{
	struct xmit_frame *pxframe = NULL;
	struct list_head *plist, *phead;
	struct __queue *queue = &pxmitpriv->free_xframe_ext_queue;

	spin_lock_bh(&queue->lock);

	if (list_empty(&queue->list)) {
		pxframe =  NULL;
	} else {
		phead = get_list_head(queue);
		plist = get_next(phead);
		pxframe = container_of(plist, struct xmit_frame, list);

		list_del_init(&(pxframe->list));
		pxmitpriv->free_xframe_ext_cnt--;
	}

	spin_unlock_bh(&queue->lock);

	rtw_init_xmitframe(pxframe);

	return pxframe;
}

struct xmit_frame *rtw_alloc_xmitframe_once(struct xmit_priv *pxmitpriv)
{
	struct xmit_frame *pxframe = NULL;
	uint8_t *alloc_addr;

	alloc_addr = rtw_zmalloc(sizeof(struct xmit_frame) + 4);

	if (alloc_addr == NULL)
		goto exit;

	pxframe = (struct xmit_frame *)N_BYTE_ALIGMENT((SIZE_PTR)(alloc_addr), 4);
	pxframe->alloc_addr = alloc_addr;

	pxframe->rtlpriv = pxmitpriv->rtlpriv;
	pxframe->frame_tag = NULL_FRAMETAG;

	pxframe->skb = NULL;

	pxframe->buf_addr = NULL;
	pxframe->pxmitbuf = NULL;

	rtw_init_xmitframe(pxframe);

	DBG_871X("################## %s ##################\n", __func__);

exit:
	return pxframe;
}

int32_t rtw_free_xmitframe(struct xmit_priv *pxmitpriv, struct xmit_frame *pxmitframe)
{
	struct __queue *queue;
	struct rtl_priv *rtlpriv = pxmitpriv->rtlpriv;
	struct sk_buff *skb = NULL;

	if (pxmitframe == NULL) {
		goto exit;
	}

	if (pxmitframe->skb) {
		skb = pxmitframe->skb;
		pxmitframe->skb = NULL;
	}

	if (pxmitframe->alloc_addr) {
		DBG_871X("################## %s with alloc_addr ##################\n", __func__);
		rtw_mfree(pxmitframe->alloc_addr);
		goto check_pkt_complete;
	}

	if (pxmitframe->ext_tag == 0)
		queue = &pxmitpriv->free_xmit_queue;
	else if (pxmitframe->ext_tag == 1)
		queue = &pxmitpriv->free_xframe_ext_queue;
	else {
	}

	spin_lock_bh(&queue->lock);

	list_del_init(&pxmitframe->list);
	list_add_tail(&pxmitframe->list, get_list_head(queue));
	if (pxmitframe->ext_tag == 0) {
		pxmitpriv->free_xmitframe_cnt++;
	} else if (pxmitframe->ext_tag == 1) {
		pxmitpriv->free_xframe_ext_cnt++;
	} else {
		;
	}

	spin_unlock_bh(&queue->lock);

check_pkt_complete:

	if (skb)
		rtw_os_pkt_complete(rtlpriv, skb);

exit:

	return _SUCCESS;
}

void rtw_free_xmitframe_queue(struct xmit_priv *pxmitpriv, struct __queue *pframequeue)
{
	struct list_head	*plist, *phead;
	struct	xmit_frame 	*pxmitframe;

	spin_lock_bh(&(pframequeue->lock));

	phead = get_list_head(pframequeue);
	plist = get_next(phead);

	while (rtw_end_of_queue_search(phead, plist) == false) {

		pxmitframe = container_of(plist, struct xmit_frame, list);

		plist = get_next(plist);

		rtw_free_xmitframe(pxmitpriv, pxmitframe);

	}
	spin_unlock_bh(&(pframequeue->lock));

}


int32_t rtw_xmitframe_enqueue(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	if (rtw_xmit_classifier(rtlpriv, pxmitframe) == _FAIL) {
/*		pxmitframe->pkt = NULL; */
		return _FAIL;
	}

	return _SUCCESS;
}


static struct xmit_frame *dequeue_one_xmitframe(struct xmit_priv *pxmitpriv, struct hw_xmit *phwxmit, struct tx_servq *ptxservq, struct __queue *pframe_queue)
{
	struct list_head	*xmitframe_plist, *xmitframe_phead;
	struct	xmit_frame	*pxmitframe = NULL;

	xmitframe_phead = get_list_head(pframe_queue);
	xmitframe_plist = get_next(xmitframe_phead);

	while ((rtw_end_of_queue_search(xmitframe_phead, xmitframe_plist)) == false) {
		pxmitframe = container_of(xmitframe_plist, struct xmit_frame, list);

		xmitframe_plist = get_next(xmitframe_plist);

		list_del_init(&pxmitframe->list);

		/* list_add_tail(&pxmitframe->list, &phwxmit->pending); */

		/* ptxservq->qcnt--; */

		break;

		/* ULLI : stupid break ?? */
		pxmitframe = NULL;

	}

	return pxmitframe;
}

struct xmit_frame *rtw_dequeue_xframe(struct xmit_priv *pxmitpriv, struct hw_xmit *phwxmit_i)
{
	struct list_head *sta_plist, *sta_phead;
	struct hw_xmit *phwxmit;
	struct tx_servq *ptxservq = NULL;
	struct __queue *pframe_queue = NULL;
	struct xmit_frame *pxmitframe = NULL;
	struct rtl_priv *rtlpriv = pxmitpriv->rtlpriv;
	struct registry_priv	*pregpriv = &rtlpriv->registrypriv;
	int i, inx[4];

	inx[0] = 0; inx[1] = 1; inx[2] = 2; inx[3] = 3;

	spin_lock_bh(&pxmitpriv->lock);

	for (i = 0; i < HWXMIT_ENTRY; i++) {
		phwxmit = phwxmit_i + inx[i];

		/* spin_lock_irqsave(&phwxmit->sta_queue->lock, &irqL0); */

		sta_phead = get_list_head(phwxmit->sta_queue);
		sta_plist = get_next(sta_phead);

		while ((rtw_end_of_queue_search(sta_phead, sta_plist)) == false) {

			ptxservq = container_of(sta_plist, struct tx_servq, tx_pending);

			pframe_queue = &ptxservq->sta_pending;

			pxmitframe = dequeue_one_xmitframe(pxmitpriv, phwxmit, ptxservq, pframe_queue);

			if (pxmitframe) {
				/* Remove sta node when there is no pending packets. */
				if (list_empty(&pframe_queue->list))	/* must be done after get_next and before break */
					list_del_init(&ptxservq->tx_pending);

				/* spin_unlock_irqrestore(&phwxmit->sta_queue->lock, &irqL0); */

				goto exit;
			}

			sta_plist = get_next(sta_plist);

		}

		/* spin_unlock_irqrestore(&phwxmit->sta_queue->lock, &irqL0); */

	}

exit:

	spin_unlock_bh(&pxmitpriv->lock);

	return pxmitframe;
}

static struct tx_servq *rtw_get_sta_pending(struct rtl_priv *rtlpriv, struct sta_info *psta, int up, uint8_t *ac)
{
	struct tx_servq *ptxservq = NULL;

	switch (up) {
	case 1:
	case 2:
		ptxservq = &(psta->sta_xmitpriv.bk_q);
		*(ac) = 3;
		break;

	case 4:
	case 5:
		ptxservq = &(psta->sta_xmitpriv.vi_q);
		*(ac) = 1;
		break;

	case 6:
	case 7:
		ptxservq = &(psta->sta_xmitpriv.vo_q);
		*(ac) = 0;
		break;

	case 0:
	case 3:
	default:
		ptxservq = &(psta->sta_xmitpriv.be_q);
		*(ac) = 2;
	break;

	}

	return ptxservq;
}

/*
 * Will enqueue pxmitframe to the proper queue,
 * and indicate it to xx_pending list.....
 */
int32_t rtw_xmit_classifier(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	/* _irqL irqL0; */
	uint8_t	ac_index;
	struct sta_info	*psta;
	struct tx_servq	*ptxservq;
	struct tx_pkt_attrib	*pattrib = &pxmitframe->tx_attrib;
	struct sta_priv	*pstapriv = &rtlpriv->stapriv;
	struct hw_xmit	*phwxmits =  rtlpriv->xmitpriv.hwxmits;
	int res = _SUCCESS;

/*
	if (pattrib->psta) {
		psta = pattrib->psta;
	} else {
		DBG_871X("%s, call rtw_get_stainfo()\n", __func__);
		psta = rtw_get_stainfo(pstapriv, pattrib->ra);
	}
*/

	psta = rtw_get_stainfo(&rtlpriv->stapriv, pattrib->ra);
	if (pattrib->psta != psta) {
		DBG_871X("%s, pattrib->psta(%p) != psta(%p)\n", __func__, pattrib->psta, psta);
		return _FAIL;
	}

	if (psta == NULL) {
		res = _FAIL;
		DBG_8192C("rtw_xmit_classifier: psta == NULL\n");
		goto exit;
	}

	if (!(psta->state & _FW_LINKED)) {
		DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
		return _FAIL;
	}

	ptxservq = rtw_get_sta_pending(rtlpriv, psta, pattrib->tx_priority, (uint8_t *)(&ac_index));

	/* spin_lock_irqsave(&pstapending->lock, &irqL0); */

	if (list_empty(&ptxservq->tx_pending)) {
		list_add_tail(&ptxservq->tx_pending, get_list_head(phwxmits[ac_index].sta_queue));
	}

	/* spin_lock_irqsave(&ptxservq->sta_pending.lock, &irqL1); */

	list_add_tail(&pxmitframe->list, get_list_head(&ptxservq->sta_pending));

	/* spin_unlock_irqrestore(&ptxservq->sta_pending.lock, &irqL1); */

	/* spin_unlock_irqrestore(&pstapending->lock, &irqL0); */

exit:

	return res;
}

void rtw_alloc_hwxmits(struct rtl_priv *rtlpriv)
{
	struct hw_xmit *hwxmits;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;

	hwxmits = pxmitpriv->hwxmits;

		/* pxmitpriv->vo_txqueue.head = 0; */
		/* hwxmits[0] .phwtxqueue = &pxmitpriv->vo_txqueue; */
		hwxmits[0].sta_queue = &pxmitpriv->vo_pending;

		/* pxmitpriv->vi_txqueue.head = 0; */
		/* hwxmits[1] .phwtxqueue = &pxmitpriv->vi_txqueue; */
		hwxmits[1].sta_queue = &pxmitpriv->vi_pending;

		/* pxmitpriv->be_txqueue.head = 0; */
		/* hwxmits[2] .phwtxqueue = &pxmitpriv->be_txqueue; */
		hwxmits[2].sta_queue = &pxmitpriv->be_pending;

		/* pxmitpriv->bk_txqueue.head = 0; */
		/* hwxmits[3] .phwtxqueue = &pxmitpriv->bk_txqueue; */
		hwxmits[3].sta_queue = &pxmitpriv->bk_pending;
}


void rtw_init_hwxmits(struct hw_xmit *phwxmit)
{
	int i;
	for (i = 0; i < HWXMIT_ENTRY; i++, phwxmit++) {
		/*
		 * spin_lock_init(&phwxmit->xmit_lock);
		 * INIT_LIST_HEAD(&phwxmit->pending);
		 * phwxmit->txcmdcnt = 0;
		 */
	}
}


static void do_queue_select(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib)
{
	uint8_t qsel;

	qsel = pattrib->tx_priority;

	pattrib->tx_qsel = qsel;
}

/*
 * The main transmit(tx) entry
 *
 * Return
 *	1	enqueue
 *	0	success, hardware will handle this xmit frame(packet)
 *	<0	fail
 */
int32_t rtw_xmit(struct rtl_priv *rtlpriv, struct sk_buff **ppkt)
{
	static uint32_t	 start = 0;
	static uint32_t	 drop_cnt = 0;
#ifdef CONFIG_AP_MODE
#endif
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct xmit_frame *pxmitframe = NULL;

	int32_t res;

	if (start == 0)
		start = jiffies;

	pxmitframe = rtw_alloc_xmitframe(pxmitpriv);

	if (rtw_get_passing_time_ms(start) > 2000) {
		if (drop_cnt)
			DBG_871X("DBG_TX_DROP_FRAME %s no more pxmitframe, drop_cnt:%u\n", __FUNCTION__, drop_cnt);
		start = jiffies;
		drop_cnt = 0;
	}

	if (pxmitframe == NULL) {
		drop_cnt++;
		return -1;
	}

	res = update_attrib(rtlpriv, *ppkt, &pxmitframe->tx_attrib);

	if (res == _FAIL) {
		rtw_free_xmitframe(pxmitpriv, pxmitframe);
		return -1;
	}
	pxmitframe->skb = *ppkt;
	do_queue_select(rtlpriv, &pxmitframe->tx_attrib);

#ifdef CONFIG_AP_MODE
	spin_lock_bh(&pxmitpriv->lock);
	if (xmitframe_enqueue_for_sleeping_sta(rtlpriv, pxmitframe) == true) {
		spin_unlock_bh(&pxmitpriv->lock);
		return 1;
	}
	spin_unlock_bh(&pxmitpriv->lock);
#endif

	if (rtlpriv->cfg->ops->hal_xmit(rtlpriv, pxmitframe) == false)
		return 1;

	return 0;
}


#if defined(CONFIG_AP_MODE)

int xmitframe_enqueue_for_sleeping_sta(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	int ret = false;
	struct sta_info *psta = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct tx_pkt_attrib *pattrib = &pxmitframe->tx_attrib;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	int bmcst = is_multicast_ether_addr(pattrib->ra);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) == false)
	    return ret;
/*
	if(pattrib->psta)
	{
		psta = pattrib->psta;
	}
	else
	{
		DBG_871X("%s, call rtw_get_stainfo()\n", __func__);
		psta=rtw_get_stainfo(pstapriv, pattrib->ra);
	}
*/
	psta = rtw_get_stainfo(&rtlpriv->stapriv, pattrib->ra);
	if (pattrib->psta != psta) {
		DBG_871X("%s, pattrib->psta(%p) != psta(%p)\n", __func__, pattrib->psta, psta);
		return false;
	}

	if (psta == NULL) {
		DBG_871X("%s, psta==NUL\n", __func__);
		return false;
	}

	if (!(psta->state & _FW_LINKED)) {
		DBG_871X("%s, psta->state(0x%x) != _FW_LINKED\n", __func__, psta->state);
		return false;
	}

	if (pattrib->triggered == 1) {
		/* DBG_871X("directly xmit pspoll_triggered packet\n"); */

		/* pattrib->triggered=0; */

		if (bmcst)
			pattrib->tx_qsel = 0x11;	/* HIQ */


		return ret;
	}


	if (bmcst) {
		spin_lock_bh(&psta->sleep_q.lock);

		if (pstapriv->sta_dz_bitmap) {
			/* if anyone sta is in ps mode */

			/* pattrib->qsel = 0x11;//HIQ */

			list_del_init(&pxmitframe->list);

			/* spin_lock_bh(&psta->sleep_q.lock, &irqL); */

			list_add_tail(&pxmitframe->list, get_list_head(&psta->sleep_q));

			psta->sleepq_len++;

			pstapriv->tim_bitmap |= BIT(0);
			pstapriv->sta_dz_bitmap |= BIT(0);

			/* DBG_871X("enqueue, sq_len=%d, tim=%x\n", psta->sleepq_len, pstapriv->tim_bitmap); */

			update_beacon(rtlpriv, _TIM_IE_, NULL, false);	/* tx bc/mc packets after upate bcn */

			/* spin_unlock_bh(&psta->sleep_q.lock, &irqL); */

			ret = true;

		}

		spin_unlock_bh(&psta->sleep_q.lock);

		return ret;

	}


	spin_lock_bh(&psta->sleep_q.lock);

	if (psta->state & WIFI_SLEEP_STATE) {
		uint8_t wmmps_ac = 0;

		if (pstapriv->sta_dz_bitmap&BIT(psta->aid)) {
			list_del_init(&pxmitframe->list);

			/*spin_lock_bh(&psta->sleep_q.lock, &irqL); */

			list_add_tail(&pxmitframe->list, get_list_head(&psta->sleep_q));

			psta->sleepq_len++;

			switch (pattrib->tx_priority) {
			case 1:
			case 2:
				wmmps_ac = psta->uapsd_bk&BIT(0);
				break;
			case 4:
			case 5:
				wmmps_ac = psta->uapsd_vi&BIT(0);
				break;
			case 6:
			case 7:
				wmmps_ac = psta->uapsd_vo&BIT(0);
				break;
			case 0:
			case 3:
			default:
				wmmps_ac = psta->uapsd_be&BIT(0);
				break;
			}

			if (wmmps_ac)
				psta->sleepq_ac_len++;

			if (((psta->has_legacy_ac) && (!wmmps_ac)) || ((!psta->has_legacy_ac) && (wmmps_ac))) {
				pstapriv->tim_bitmap |= BIT(psta->aid);

				/* DBG_871X("enqueue, sq_len=%d, tim=%x\n", psta->sleepq_len, pstapriv->tim_bitmap); */

				if (psta->sleepq_len == 1) {
					/* DBG_871X("sleepq_len==1, update BCNTIM\n"); */
					/* upate BCN for TIM IE */
					update_beacon(rtlpriv, _TIM_IE_, NULL, false);
				}
			}

			/*spin_unlock_bh(&psta->sleep_q.lock, &irqL); */

			/*
			 * if(psta->sleepq_len > (NR_XMITFRAME>>3)) {
			 *	wakeup_sta_to_xmit(rtlpriv, psta);
			 * }
			 */

			ret = true;

		}

	}

	spin_unlock_bh(&psta->sleep_q.lock);

	return ret;

}

static void dequeue_xmitframes_to_sleeping_queue(struct rtl_priv *rtlpriv, struct sta_info *psta, struct __queue *pframequeue)
{
	int ret;
	struct list_head	*plist, *phead;
	uint8_t	ac_index;
	struct tx_servq	*ptxservq;
	struct tx_pkt_attrib	*pattrib;
	struct xmit_frame 	*pxmitframe;
	struct hw_xmit *phwxmits =  rtlpriv->xmitpriv.hwxmits;

	phead = get_list_head(pframequeue);
	plist = get_next(phead);

	while (rtw_end_of_queue_search(phead, plist) == false) {
		pxmitframe = container_of(plist, struct xmit_frame, list);

		plist = get_next(plist);

		ret = xmitframe_enqueue_for_sleeping_sta(rtlpriv, pxmitframe);

		if (true == ret) {
			pattrib = &pxmitframe->tx_attrib;

			ptxservq = rtw_get_sta_pending(rtlpriv, psta, pattrib->tx_priority, (uint8_t *)(&ac_index));
		} else 	{
			/* DBG_871X("xmitframe_enqueue_for_sleeping_sta return false\n"); */
		}

	}

}

void stop_sta_xmit(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	struct sta_info *psta_bmc;
	struct sta_xmit_priv *pstaxmitpriv;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;

	pstaxmitpriv = &psta->sta_xmitpriv;

	/* for BC/MC Frames */
	psta_bmc = rtw_get_bcmc_stainfo(rtlpriv);


	spin_lock_bh(&pxmitpriv->lock);

	psta->state |= WIFI_SLEEP_STATE;

	pstapriv->sta_dz_bitmap |= BIT(psta->aid);

	dequeue_xmitframes_to_sleeping_queue(rtlpriv, psta, &pstaxmitpriv->vo_q.sta_pending);
	list_del_init(&(pstaxmitpriv->vo_q.tx_pending));


	dequeue_xmitframes_to_sleeping_queue(rtlpriv, psta, &pstaxmitpriv->vi_q.sta_pending);
	list_del_init(&(pstaxmitpriv->vi_q.tx_pending));


	dequeue_xmitframes_to_sleeping_queue(rtlpriv, psta, &pstaxmitpriv->be_q.sta_pending);
	list_del_init(&(pstaxmitpriv->be_q.tx_pending));


	dequeue_xmitframes_to_sleeping_queue(rtlpriv, psta, &pstaxmitpriv->bk_q.sta_pending);
	list_del_init(&(pstaxmitpriv->bk_q.tx_pending));

	/* for BC/MC Frames */
	pstaxmitpriv = &psta_bmc->sta_xmitpriv;
	dequeue_xmitframes_to_sleeping_queue(rtlpriv, psta_bmc, &pstaxmitpriv->be_q.sta_pending);
	list_del_init(&(pstaxmitpriv->be_q.tx_pending));

	spin_unlock_bh(&pxmitpriv->lock);


}

void wakeup_sta_to_xmit(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	uint8_t update_mask = 0, wmmps_ac = 0;
	struct sta_info *psta_bmc;
	struct list_head	*xmitframe_plist, *xmitframe_phead;
	struct xmit_frame *pxmitframe = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;

	psta_bmc = rtw_get_bcmc_stainfo(rtlpriv);

	/* spin_lock_bh(&psta->sleep_q.lock, &irqL); */
	spin_lock_bh(&pxmitpriv->lock);

	xmitframe_phead = get_list_head(&psta->sleep_q);
	xmitframe_plist = get_next(xmitframe_phead);

	while ((rtw_end_of_queue_search(xmitframe_phead, xmitframe_plist)) == false) {
		pxmitframe = container_of(xmitframe_plist, struct xmit_frame, list);

		xmitframe_plist = get_next(xmitframe_plist);

		list_del_init(&pxmitframe->list);

		switch (pxmitframe->tx_attrib.tx_priority) {
		case 1:
		case 2:
			wmmps_ac = psta->uapsd_bk&BIT(1);
			break;
		case 4:
		case 5:
			wmmps_ac = psta->uapsd_vi&BIT(1);
			break;
		case 6:
		case 7:
			wmmps_ac = psta->uapsd_vo&BIT(1);
			break;
		case 0:
		case 3:
		default:
			wmmps_ac = psta->uapsd_be&BIT(1);
			break;
		}

		psta->sleepq_len--;
		if (psta->sleepq_len > 0)
			pxmitframe->tx_attrib.mdata = 1;
		else
			pxmitframe->tx_attrib.mdata = 0;

		if (wmmps_ac) {
			psta->sleepq_ac_len--;
			if (psta->sleepq_ac_len > 0) {
				pxmitframe->tx_attrib.mdata = 1;
				pxmitframe->tx_attrib.eosp = 0;
			} else {
				pxmitframe->tx_attrib.mdata = 0;
				pxmitframe->tx_attrib.eosp = 1;
			}
		}

		pxmitframe->tx_attrib.triggered = 1;

/*
		spin_unlock_bh(&psta->sleep_q.lock, &irqL);
		if(rtw_hal_xmit(rtlpriv, pxmitframe) == true)
		{
			rtw_os_xmit_complete(rtlpriv, pxmitframe);
		}
		spin_lock_bh(&psta->sleep_q.lock, &irqL);
*/
		rtlpriv->cfg->ops->hal_xmitframe_enqueue(rtlpriv, pxmitframe);


	}

	/* for BC/MC Frames */
	if (!psta_bmc)
		goto _exit;

	if ((pstapriv->sta_dz_bitmap & 0xfffe) == 0x0) {
		/* no any sta in ps mode */

		xmitframe_phead = get_list_head(&psta_bmc->sleep_q);
		xmitframe_plist = get_next(xmitframe_phead);

		while ((rtw_end_of_queue_search(xmitframe_phead, xmitframe_plist)) == false) {
			pxmitframe = container_of(xmitframe_plist, struct xmit_frame, list);

			xmitframe_plist = get_next(xmitframe_plist);

			list_del_init(&pxmitframe->list);

			psta_bmc->sleepq_len--;
			if (psta_bmc->sleepq_len > 0)
				pxmitframe->tx_attrib.mdata = 1;
			else
				pxmitframe->tx_attrib.mdata = 0;


			pxmitframe->tx_attrib.triggered = 1;
/*
			spin_unlock_bh(&psta_bmc->sleep_q.lock, &irqL);
			if(rtw_hal_xmit(rtlpriv, pxmitframe) == true)
			{
				rtw_os_xmit_complete(rtlpriv, pxmitframe);
			}
			spin_lock_bh(&psta_bmc->sleep_q.lock, &irqL);

*/
			rtlpriv->cfg->ops->hal_xmitframe_enqueue(rtlpriv, pxmitframe);

		}

		if (psta_bmc->sleepq_len == 0) {
			pstapriv->tim_bitmap &= ~BIT(0);
			pstapriv->sta_dz_bitmap &= ~BIT(0);

			/* DBG_871X("wakeup to xmit, qlen==0, update_BCNTIM, tim=%x\n", pstapriv->tim_bitmap); */
			/* upate BCN for TIM IE */
			/* update_BCNTIM(rtlpriv); */
			update_mask |= BIT(1);
		}

	}

	if (psta->sleepq_len == 0) {
		pstapriv->tim_bitmap &= ~BIT(psta->aid);

		/* DBG_871X("wakeup to xmit, qlen==0, update_BCNTIM, tim=%x\n", pstapriv->tim_bitmap); */
		/* upate BCN for TIM IE */
		/* update_BCNTIM(rtlpriv); */
		update_mask = BIT(0);

		if (psta->state & WIFI_SLEEP_STATE)
			psta->state ^= WIFI_SLEEP_STATE;

		if (psta->state & WIFI_STA_ALIVE_CHK_STATE) {
			psta->expire_to = pstapriv->expire_to;
			psta->state ^= WIFI_STA_ALIVE_CHK_STATE;
		}

		pstapriv->sta_dz_bitmap &= ~BIT(psta->aid);
	}

_exit:

	/*spin_unlock_bh(&psta_bmc->sleep_q.lock, &irqL); */
	spin_unlock_bh(&pxmitpriv->lock);

	if (update_mask) {
		/* update_BCNTIM(rtlpriv); */
		/* printk("%s => call update_beacon\n",__FUNCTION__); */
		update_beacon(rtlpriv, _TIM_IE_, NULL, false);
	}

}

void xmit_delivery_enabled_frames(struct rtl_priv *rtlpriv, struct sta_info *psta)
{
	uint8_t wmmps_ac = 0;
	struct list_head *xmitframe_plist, *xmitframe_phead;
	struct xmit_frame *pxmitframe = NULL;
	struct sta_priv *pstapriv = &rtlpriv->stapriv;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;

	/* spin_lock_bh(&psta->sleep_q.lock, &irqL); */
	spin_lock_bh(&pxmitpriv->lock);

	xmitframe_phead = get_list_head(&psta->sleep_q);
	xmitframe_plist = get_next(xmitframe_phead);

	while ((rtw_end_of_queue_search(xmitframe_phead, xmitframe_plist)) == false) {
		pxmitframe = container_of(xmitframe_plist, struct xmit_frame, list);

		xmitframe_plist = get_next(xmitframe_plist);

		switch (pxmitframe->tx_attrib.tx_priority) {
		case 1:
		case 2:
			wmmps_ac = psta->uapsd_bk&BIT(1);
			break;
		case 4:
		case 5:
			wmmps_ac = psta->uapsd_vi&BIT(1);
			break;
		case 6:
		case 7:
			wmmps_ac = psta->uapsd_vo&BIT(1);
			break;
		case 0:
		case 3:
		default:
			wmmps_ac = psta->uapsd_be&BIT(1);
			break;
		}

		if (!wmmps_ac)
			continue;

		list_del_init(&pxmitframe->list);

		psta->sleepq_len--;
		psta->sleepq_ac_len--;

		if (psta->sleepq_ac_len > 0) {
			pxmitframe->tx_attrib.mdata = 1;
			pxmitframe->tx_attrib.eosp = 0;
		} else {
			pxmitframe->tx_attrib.mdata = 0;
			pxmitframe->tx_attrib.eosp = 1;
		}

		pxmitframe->tx_attrib.triggered = 1;

/*
		if(rtw_hal_xmit(rtlpriv, pxmitframe) == true)
		{
			rtw_os_xmit_complete(rtlpriv, pxmitframe);
		}
*/
		rtlpriv->cfg->ops->hal_xmitframe_enqueue(rtlpriv, pxmitframe);

		if ((psta->sleepq_ac_len == 0) && (!psta->has_legacy_ac) && (wmmps_ac)) {
			pstapriv->tim_bitmap &= ~BIT(psta->aid);

			/* DBG_871X("wakeup to xmit, qlen==0, update_BCNTIM, tim=%x\n", pstapriv->tim_bitmap); */
			/* upate BCN for TIM IE */
			/* update_BCNTIM(rtlpriv); */
			update_beacon(rtlpriv, _TIM_IE_, NULL, false);
			/* update_mask = BIT(0); */
		}

	}

	/* spin_unlock_bh(&psta->sleep_q.lock, &irqL); */
	spin_unlock_bh(&pxmitpriv->lock);

}

#endif


void rtw_sctx_init(struct submit_ctx *sctx, int timeout_ms)
{
	sctx->timeout_ms = timeout_ms;
	sctx->submit_time = jiffies;

	init_completion(&sctx->done);

	sctx->status = RTW_SCTX_SUBMITTED;
}

int rtw_sctx_wait(struct submit_ctx *sctx)
{
	int ret = _FAIL;
	unsigned long expire;
	int status = 0;

	expire = sctx->timeout_ms ? msecs_to_jiffies(sctx->timeout_ms) : MAX_SCHEDULE_TIMEOUT;
	if (!wait_for_completion_timeout(&sctx->done, expire)) {
		/* timeout, do something?? */
		status = RTW_SCTX_DONE_TIMEOUT;
		DBG_871X("%s timeout\n", __func__);
	} else {
		status = sctx->status;
	}

	if (status == RTW_SCTX_DONE_SUCCESS) {
		ret = _SUCCESS;
	}

	return ret;
}

bool rtw_sctx_chk_waring_status(int status)
{
	switch (status) {
	case RTW_SCTX_DONE_UNKNOWN:
	case RTW_SCTX_DONE_BUF_ALLOC:
	case RTW_SCTX_DONE_BUF_FREE:

	case RTW_SCTX_DONE_DRV_STOP:
	case RTW_SCTX_DONE_DEV_REMOVE:
		return true;
	default:
		return false;
	}
}

void rtw_sctx_done_err(struct submit_ctx **sctx, int status)
{
	if (*sctx) {
		if (rtw_sctx_chk_waring_status(status))
			DBG_871X("%s status:%d\n", __func__, status);
		(*sctx)->status = status;

		complete(&((*sctx)->done));

		*sctx = NULL;
	}
}

void rtw_sctx_done(struct submit_ctx **sctx)
{
	rtw_sctx_done_err(sctx, RTW_SCTX_DONE_SUCCESS);
}

