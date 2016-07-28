#ifdef CONFIG_RTLWIFI

#include <../drivers/net/wireless/realtek/rtlwifi/wifi.h>
#include <../drivers/net/wireless/realtek/rtlwifi/core.h>
#include <../drivers/net/wireless/realtek/rtlwifi/ps.h>
#include <../drivers/net/wireless/realtek/rtlwifi/base.h>

#else

#include <drv_types.h>
#include <linux/etherdevice.h>
#include <rtl8812a_hal.h>
#include "trx.h"
#include "def.h"
#include <usb_ops.h>

#endif

/* ULLI : reference -> u16 rtl8192cu_mq_to_hwq() */
/* ULLI : for rtl_hal_usbint_cfg ->usb_mq_to_hwq */

static uint32_t rtw_get_ff_hwaddr(struct xmit_frame *pxmitframe)
{
	uint32_t	 addr;
	struct tx_pkt_attrib *pattrib = &pxmitframe->tx_attrib;

	switch (pattrib->tx_qsel) {
	case 0:
	case 3:
		addr = RTL_TXQ_BE;
		break;
	case 1:
	case 2:
		addr = RTL_TXQ_BK;
		break;
	case 4:
	case 5:
		addr = RTL_TXQ_VI;
		break;
	case 6:
	case 7:
		addr = RTL_TXQ_VO;
		break;
	case 0x10:
		addr = RTL_TXQ_BCN;
		break;
	case 0x11:	/* BC/MC in PS (HIQ) */
		addr = RTL_TXQ_HI;
		break;
	case 0x12:
	default:
		addr = RTL_TXQ_MGT;
		break;

	}

	return addr;

}

static void rtl8812a_cal_txdesc_chksum(uint8_t *ptxdesc)
{
	u16 *usPtr;
	uint32_t count;
	uint32_t index;
	u16 checksum = 0;

	usPtr = (u16 *)ptxdesc;
	/*
	 * checksume is always calculated by first 32 bytes,
	 * and it doesn't depend on TX DESC length.
	 * Thomas,Lucas@SD4,20130515
	 */
	count = 16;

	/* Clear first */
	SET_TX_DESC_TX_DESC_CHECKSUM(ptxdesc, 0);

	for (index = 0; index < count; index++) {
		checksum = checksum ^ le16_to_cpu(*(usPtr + index));
	}

	SET_TX_DESC_TX_DESC_CHECKSUM(ptxdesc, checksum);
}


static uint8_t	MRateToHwRate(uint8_t rate)
{
	uint8_t	ret = DESC_RATE1M;

	switch(rate)
	{
		// CCK and OFDM non-HT rates
	case IEEE80211_CCK_RATE_1MB:	ret = DESC_RATE1M;	break;
	case IEEE80211_CCK_RATE_2MB:	ret = DESC_RATE2M;	break;
	case IEEE80211_CCK_RATE_5MB:	ret = DESC_RATE5_5M;	break;
	case IEEE80211_CCK_RATE_11MB:	ret = DESC_RATE11M;	break;
	case IEEE80211_OFDM_RATE_6MB:	ret = DESC_RATE6M;	break;
	case IEEE80211_OFDM_RATE_9MB:	ret = DESC_RATE9M;	break;
	case IEEE80211_OFDM_RATE_12MB:	ret = DESC_RATE12M;	break;
	case IEEE80211_OFDM_RATE_18MB:	ret = DESC_RATE18M;	break;
	case IEEE80211_OFDM_RATE_24MB:	ret = DESC_RATE24M;	break;
	case IEEE80211_OFDM_RATE_36MB:	ret = DESC_RATE36M;	break;
	case IEEE80211_OFDM_RATE_48MB:	ret = DESC_RATE48M;	break;
	case IEEE80211_OFDM_RATE_54MB:	ret = DESC_RATE54M;	break;

		// HT rates since here
	//case MGN_MCS0:		ret = DESC_RATEMCS0;	break;
	//case MGN_MCS1:		ret = DESC_RATEMCS1;	break;
	//case MGN_MCS2:		ret = DESC_RATEMCS2;	break;
	//case MGN_MCS3:		ret = DESC_RATEMCS3;	break;
	//case MGN_MCS4:		ret = DESC_RATEMCS4;	break;
	//case MGN_MCS5:		ret = DESC_RATEMCS5;	break;
	//case MGN_MCS6:		ret = DESC_RATEMCS6;	break;
	//case MGN_MCS7:		ret = DESC_RATEMCS7;	break;

	default:		break;
	}

	return ret;
}


void rtl8821au_fill_fake_txdesc(struct rtl_priv *rtlpriv, uint8_t *pDesc,
	uint32_t BufferLen, uint8_t IsPsPoll, uint8_t IsBTQosNull)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;

	/* Clear all status */
	memset(pDesc, 0, TXDESC_SIZE);

	SET_TX_DESC_FIRST_SEG(pDesc, 1);
	SET_TX_DESC_LAST_SEG(pDesc, 1);

	SET_TX_DESC_OFFSET(pDesc, TXDESC_SIZE);

	SET_TX_DESC_PKT_SIZE(pDesc, BufferLen);

	SET_TX_DESC_QUEUE_SEL(pDesc,  QSLT_MGNT);

	if (pmlmeext->cur_wireless_mode & WIRELESS_11B) {
		SET_TX_DESC_RATE_ID(pDesc, RATEID_IDX_B);
	} else {
		SET_TX_DESC_RATE_ID(pDesc, RATEID_IDX_G);
	}

	/*
	 * Set NAVUSEHDR to prevent Ps-poll AId filed to be changed to error vlaue by Hw.
	 */
	if (IsPsPoll) {
		SET_TX_DESC_NAV_USE_HDR(pDesc, 1);
	} else {
		SET_TX_DESC_HWSEQ_EN(pDesc, 1); // Hw set sequence number
	}

	if (IsBTQosNull) {
		SET_TX_DESC_BT_INT(pDesc, 1);
	}

	SET_TX_DESC_USE_RATE(pDesc, 1);
	SET_TX_DESC_OWN(pDesc, 1);

	SET_TX_DESC_TX_RATE(pDesc, MRateToHwRate(pmlmeext->tx_rate));

	// USB interface drop packet if the checksum of descriptor isn't correct.
	// Using this checksum can let hardware recovery from packet bulk out error (e.g. Cancel URC, Bulk out error.).
	rtl8812a_cal_txdesc_chksum(pDesc);
}

/* ULLI TX DESC */
/*
 * Description: In normal chip, we should send some packet to Hw which will be used by Fw
 * in FW LPS mode. The function is to fill the Tx descriptor of this packets, then
 * Fw can tell Hw to send these packet derectly.
 *
*/
static void rtl8812a_fill_txdesc_sectype(struct tx_pkt_attrib *pattrib, uint8_t *ptxdesc)
{
	if ((pattrib->encrypt > 0) && !pattrib->bswenc) {
		switch (pattrib->encrypt) {
		/* SEC_TYPE : 0:NO_ENC,1:WEP40/TKIP,2:WAPI,3:AES */
		case WEP40_ENCRYPTION:
		case WEP104_ENCRYPTION:
		case TKIP_ENCRYPTION:
		case RSERVED_ENCRYPTION:
			SET_TX_DESC_SEC_TYPE(ptxdesc, 0x1);
			break;
		case AESCCMP_ENCRYPTION:
			SET_TX_DESC_SEC_TYPE(ptxdesc, 0x3);
			break;
		case NO_ENCRYPTION:
		default:
			SET_TX_DESC_SEC_TYPE(ptxdesc, 0x0);
			break;

		}

	}

}


static void rtl8812a_fill_txdesc_phy(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib, uint8_t *ptxdesc)
{
	/* DBG_8192C("bwmode=%d, ch_off=%d\n", pattrib->bwmode, pattrib->ch_offset); */

	if (pattrib->ht_en) {
		/*  Set Bandwidth and sub-channel settings. */
		SET_TX_DESC_DATA_BW(ptxdesc, BWMapping_8812(rtlpriv,pattrib));
		/* SET_TX_DESC_DATA_SC(ptxdesc, SCMapping_8812(rtlpriv,pattrib)); */
	}
}

static void rtl8812a_fill_txdesc_vcs(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib, uint8_t *ptxdesc)
{
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	//DBG_8192C("vcs_mode=%d\n", pattrib->vcs_mode);

	if (pattrib->vcs_mode) {
		switch(pattrib->vcs_mode) {
		case RTS_CTS:
			SET_TX_DESC_RTS_ENABLE(ptxdesc, 1);
			break;
		case CTS_TO_SELF:
			SET_TX_DESC_CTS2SELF(ptxdesc, 1);
			break;
		case NONE_VCS:
		default:
			break;
		}
		if (pmlmeinfo->preamble_mode == PREAMBLE_SHORT)
			SET_TX_DESC_RTS_SHORT(ptxdesc, 1);

		SET_TX_DESC_RTS_RATE(ptxdesc, 0x8);	/*RTS Rate=24M */

		SET_TX_DESC_RTS_RATE_FB_LIMIT(ptxdesc, 0xf);

		/*
		 * Enable HW RTS
		 * SET_TX_DESC_HW_RTS_ENABLE(ptxdesc, 1);
		 */
	}
}

static inline uint8_t rtw_usb_bulk_size_boundary(struct rtl_priv * rtlpriv,int buf_len)
{
	uint8_t rst = true;

	if (IS_SUPER_SPEED_USB(rtlpriv))
		rst = (0 == (buf_len) % USB_SUPER_SPEED_BULK_SIZE)?true:false;
	if (IS_HIGH_SPEED_USB(rtlpriv))
		rst = (0 == (buf_len) % USB_HIGH_SPEED_BULK_SIZE)?true:false;
	else
		rst = (0 == (buf_len) % USB_FULL_SPEED_BULK_SIZE)?true:false;
	return rst;
}

static int32_t update_txdesc(struct xmit_frame *pxmitframe, uint8_t *pmem, int32_t sz , uint8_t bagg_pkt)
{
	int	pull = 0;
	uint8_t offset;
	struct rtl_priv *rtlpriv = pxmitframe->rtlpriv;
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct tx_pkt_attrib	*pattrib = &pxmitframe->tx_attrib;
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;
	uint8_t	*ptxdesc =  pmem;
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	int	bmcst = is_multicast_ether_addr(pattrib->ra);

	if ((!bagg_pkt) && (rtw_usb_bulk_size_boundary(rtlpriv, TXDESC_SIZE+sz) == false)) {
		ptxdesc = (pmem+PACKET_OFFSET_SZ);
		/* DBG_8192C("==> non-agg-pkt,shift pointer...\n"); */
		pull = 1;
	}

	memset(ptxdesc, 0, TXDESC_SIZE);

	/* 4 offset 0 */
	SET_TX_DESC_FIRST_SEG(ptxdesc, 1);
	SET_TX_DESC_LAST_SEG(ptxdesc, 1);
	SET_TX_DESC_OWN(ptxdesc, 1);

	/* DBG_8192C("%s==> pkt_len=%d,bagg_pkt=%02x\n",__FUNCTION__,sz,bagg_pkt); */
	SET_TX_DESC_PKT_SIZE(ptxdesc, sz);

	offset = TXDESC_SIZE + OFFSET_SZ;

	/* DBG_8192C("%s==>offset(0x%02x)  \n",__FUNCTION__,offset); */
	SET_TX_DESC_OFFSET(ptxdesc, offset);

	if (bmcst) {
		SET_TX_DESC_BMC(ptxdesc, 1);
	}

	if (!bagg_pkt) {
		if ((pull) && (pxmitframe->pkt_offset > 0)) {
			pxmitframe->pkt_offset = pxmitframe->pkt_offset - 1;
		}
	}

	/*
	 * DBG_8192C("%s, pkt_offset=0x%02x\n",__FUNCTION__,pxmitframe->pkt_offset);
	 * pkt_offset, unit:8 bytes padding
	 */
	if (pxmitframe->pkt_offset > 0) {
		SET_TX_DESC_PKT_OFFSET(ptxdesc, pxmitframe->pkt_offset);
	}

	SET_TX_DESC_MACID(ptxdesc, pattrib->mac_id);
	SET_TX_DESC_RATE_ID(ptxdesc, pattrib->raid);

	SET_TX_DESC_QUEUE_SEL(ptxdesc,  pattrib->tx_qsel);

	/* offset 12 */

	if (!pattrib->qos_en) {
		SET_TX_DESC_HWSEQ_EN(ptxdesc, 1); /* Hw set sequence number */
	} else {
		SET_TX_DESC_SEQ(ptxdesc, pattrib->seqnum);
	}

	if ((pxmitframe->frame_tag&0x0f) == DATA_FRAMETAG) {
		/* DBG_8192C("pxmitframe->frame_tag == DATA_FRAMETAG\n"); */

		rtl8812a_fill_txdesc_sectype(pattrib, ptxdesc);

		/* offset 20 */
		if (pxmitframe->agg_num > 1) {
			/* DBG_8192C("%s agg_num:%d\n",__FUNCTION__,pxmitframe->agg_num ); */
			SET_TX_DESC_USB_TXAGG_NUM(ptxdesc, pxmitframe->agg_num);
		}

		rtl8812a_fill_txdesc_vcs(rtlpriv, pattrib, ptxdesc);

		if ((pattrib->ether_type != 0x888e) &&
		    (pattrib->ether_type != 0x0806) &&
		    (pattrib->ether_type != 0x88b4) &&
		    (pattrib->dhcp_pkt != 1)) {
			/* Non EAP & ARP & DHCP type data packet */

			if (pattrib->ampdu_en == true) {
				SET_TX_DESC_AGG_ENABLE(ptxdesc, 1);
				SET_TX_DESC_MAX_AGG_NUM(ptxdesc, 0x1f);
				/* Set A-MPDU aggregation. */
				SET_TX_DESC_AMPDU_DENSITY(ptxdesc, pHalData->AMPDUDensity);
			} else {
				SET_TX_DESC_AGG_BREAK(ptxdesc, 1);
			}

			rtl8812a_fill_txdesc_phy(rtlpriv, pattrib, ptxdesc);

			/* DATA  Rate FB LMT */
			SET_TX_DESC_DATA_RATE_FB_LIMIT(ptxdesc, 0x1f);

			if (pHalData->fw_ractrl == false) {
				SET_TX_DESC_USE_RATE(ptxdesc, 1);

				if (pdmpriv->INIDATA_RATE[pattrib->mac_id] & BIT(7))
					SET_TX_DESC_DATA_SHORT(ptxdesc, 	1);

				SET_TX_DESC_TX_RATE(ptxdesc, (pdmpriv->INIDATA_RATE[pattrib->mac_id] & 0x7F));
			}

			if (rtlpriv->fix_rate != 0xFF) { 	/* modify data rate by iwpriv */
				SET_TX_DESC_USE_RATE(ptxdesc, 1);
				if (rtlpriv->fix_rate & BIT(7))
					SET_TX_DESC_DATA_SHORT(ptxdesc, 	1);

				SET_TX_DESC_TX_RATE(ptxdesc, (rtlpriv->fix_rate & 0x7F));
			}

			if (pattrib->ldpc)
				SET_TX_DESC_DATA_LDPC(ptxdesc, 1);
			if (pattrib->stbc)
				SET_TX_DESC_DATA_STBC(ptxdesc, 1);
		} else {
			/*
			 *  EAP data packet and ARP packet and DHCP.
			 *  Use the 1M data rate to send the EAP/ARP packet.
			 *  This will maybe make the handshake smooth.
			 */

			SET_TX_DESC_USE_RATE(ptxdesc, 1);
			SET_TX_DESC_AGG_BREAK(ptxdesc, 1);

			/* HW will ignore this setting if the transmission rate is legacy OFDM. */
			if (pmlmeinfo->preamble_mode == PREAMBLE_SHORT) {
				SET_TX_DESC_DATA_SHORT(ptxdesc, 1);
			}

			SET_TX_DESC_TX_RATE(ptxdesc, MRateToHwRate(pmlmeext->tx_rate));
		}
	} else if ((pxmitframe->frame_tag&0x0f) == MGNT_FRAMETAG) {
		/* DBG_8192C("pxmitframe->frame_tag == MGNT_FRAMETAG\n"); */

		if (IS_HARDWARE_TYPE_8821(rtlhal))
			SET_TX_DESC_MBSSID_8821(ptxdesc, pattrib->mbssid);

		/* offset 20 */
		SET_TX_DESC_RETRY_LIMIT_ENABLE(ptxdesc, 1);

		if (pattrib->retry_ctrl == true) {
			SET_TX_DESC_DATA_RETRY_LIMIT(ptxdesc, 6);
		} else {
			SET_TX_DESC_DATA_RETRY_LIMIT(ptxdesc, 12);
		}

		SET_TX_DESC_USE_RATE(ptxdesc, 1);
		{
			SET_TX_DESC_TX_RATE(ptxdesc, MRateToHwRate(pmlmeext->tx_rate));
		}
	} else if ((pxmitframe->frame_tag&0x0f) == TXAGG_FRAMETAG) {
		dev_dbg(&(rtlpriv->ndev->dev), "pxmitframe->frame_tag == TXAGG_FRAMETAG\n");
	} else {
		dev_dbg(&(rtlpriv->ndev->dev), "pxmitframe->frame_tag = %d\n", pxmitframe->frame_tag);

		SET_TX_DESC_USE_RATE(ptxdesc, 1);
		SET_TX_DESC_TX_RATE(ptxdesc, MRateToHwRate(pmlmeext->tx_rate));
	}

	rtl8812a_cal_txdesc_chksum(ptxdesc);
	_dbg_dump_tx_info(rtlpriv, pxmitframe->frame_tag, ptxdesc);
	return pull;
}




/* for non-agg data frame or  management frame */
static int32_t rtw_dump_xframe(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	int32_t ret = _SUCCESS;
	int32_t inner_ret = _SUCCESS;
	int t, sz, w_sz, pull = 0;
	uint8_t *mem_addr;
	uint32_t ff_hwaddr;
	struct xmit_buf *pxmitbuf = pxmitframe->pxmitbuf;
	struct tx_pkt_attrib *pattrib = &pxmitframe->tx_attrib;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct security_priv *psecuritypriv = &rtlpriv->securitypriv;
	if ((pxmitframe->frame_tag == DATA_FRAMETAG) &&
	    (pxmitframe->tx_attrib.ether_type != 0x0806) &&
	    (pxmitframe->tx_attrib.ether_type != 0x888e) &&
	    (pxmitframe->tx_attrib.ether_type != 0x88b4) &&
	    (pxmitframe->tx_attrib.dhcp_pkt != 1)) {
		rtw_issue_addbareq_cmd(rtlpriv, pxmitframe);
	}
	mem_addr = pxmitframe->buf_addr;

	for (t = 0; t < pattrib->nr_frags; t++) {
		if (inner_ret != _SUCCESS && ret == _SUCCESS)
			ret = _FAIL;

		if (t != (pattrib->nr_frags - 1)) {
			sz = pxmitpriv->frag_len;
			sz = sz - 4 - (psecuritypriv->sw_encrypt ? 0 : pattrib->icv_len);
		} else {
			/* no frag */
			sz = pattrib->last_txcmdsz;
		}

		pull = update_txdesc(pxmitframe, mem_addr, sz, false);

		if (pull) {
			mem_addr += PACKET_OFFSET_SZ; /* pull txdesc head */

			/* pxmitbuf ->pbuf = mem_addr; */
			pxmitframe->buf_addr = mem_addr;

			w_sz = sz + TXDESC_SIZE;
		} else 	{
			w_sz = sz + TXDESC_SIZE + PACKET_OFFSET_SZ;
		}

		ff_hwaddr = rtw_get_ff_hwaddr(pxmitframe);

		inner_ret =  _rtlw_usb_transmit(rtlpriv, ff_hwaddr, w_sz, pxmitbuf);

		rtw_count_tx_stats(rtlpriv, pxmitframe, sz);

		/* DBG_8192C("rtw_write_port, w_sz=%d, sz=%d, txdesc_sz=%d, tid=%d\n", w_sz, sz, w_sz-sz, pattrib->priority); */

		mem_addr += w_sz;

		mem_addr = (uint8_t *)RND4(((__kernel_size_t)(mem_addr)));

	}

	rtw_free_xmitframe(pxmitpriv, pxmitframe);

	if  (ret != _SUCCESS)
		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_UNKNOWN);

	return ret;
}

static uint32_t xmitframe_need_length(struct xmit_frame *pxmitframe)
{
	struct tx_pkt_attrib *pattrib = &pxmitframe->tx_attrib;

	uint32_t	len = 0;

	/* no consider fragement */
	len = pattrib->hdrlen + pattrib->iv_len +
		SNAP_SIZE + sizeof(u16) +
		pattrib->pktlen +
		((pattrib->bswenc) ? pattrib->icv_len : 0);

	if (pattrib->encrypt == TKIP_ENCRYPTION)
		len += 8;

	return len;
}

#define IDEA_CONDITION 1	/* check all packets before enqueue */
int32_t rtl8812au_xmitframe_complete(struct rtl_priv *rtlpriv,
				     struct xmit_priv *pxmitpriv, struct xmit_buf *pxmitbuf)
{
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);
	struct xmit_frame *pxmitframe = NULL;
	struct xmit_frame *pfirstframe = NULL;

	/* aggregate variable */
	struct hw_xmit *phwxmit;
	struct sta_info *psta = NULL;
	struct tx_servq *ptxservq = NULL;

	struct list_head *item;

	uint32_t	pbuf;		/* next pkt address */
	uint32_t	pbuf_tail;	/* last pkt tail */
	uint32_t	len;		/* packet length, except TXDESC_SIZE and PKT_OFFSET */

	uint32_t	bulkSize = rtl_usbdev(rtlpriv)->max_bulk_out_size;
	uint8_t	descCount;
	uint32_t	bulkPtr;

	/* dump frame variable */
	uint32_t ff_hwaddr;

#ifndef IDEA_CONDITION
	int res = _SUCCESS;
#endif

	/* check xmitbuffer is ok */
	if (pxmitbuf == NULL) {
		pxmitbuf = rtw_alloc_xmitbuf(pxmitpriv);
		if (pxmitbuf == NULL) {
			/* DBG_871X("%s #1, connot alloc xmitbuf!!!! \n",__FUNCTION__); */
			return false;
		}
	}

/* DBG_8192C("%s ===================================== \n",__FUNCTION__); */
	/* 3 1. pick up first frame */
	do {
		rtw_free_xmitframe(pxmitpriv, pxmitframe);

		pxmitframe = rtw_dequeue_xframe(pxmitpriv, pxmitpriv->hwxmits);
		if (pxmitframe == NULL) {
			/*
			 * no more xmit frame, release xmit buffer
			 * DBG_8192C("no more xmit frame ,return\n");
			 */
			rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
			return false;
		}

#ifndef IDEA_CONDITION
		if (pxmitframe->frame_tag != DATA_FRAMETAG) {
			/* rtw_free_xmitframe(pxmitpriv, pxmitframe); */
			continue;
		}

		/* TID 0~15 */
		if ((pxmitframe->attrib.tx_priority < 0) ||
		    (pxmitframe->attrib.tx_priority > 15)) {
			/* rtw_free_xmitframe(pxmitpriv, pxmitframe); */
			continue;
		}
#endif
		/* DBG_8192C("==> pxmitframe->attrib.priority:%d\n",pxmitframe->attrib.priority); */
		pxmitframe->pxmitbuf = pxmitbuf;
		pxmitframe->buf_addr = pxmitbuf->pbuf;
		pxmitbuf->priv_data = pxmitframe;

		pxmitframe->agg_num = 1; 	/* alloc xmitframe should assign to 1. */
		pxmitframe->pkt_offset = 1; 	/* first frame of aggregation, reserve offset */

		if (rtw_xmitframe_coalesce(rtlpriv, pxmitframe->skb, pxmitframe) == false) {
			dev_dbg(&(rtlpriv->ndev->dev), "%s coalesce 1st xmitframe failed \n", __FUNCTION__);
			continue;
		}


		/* always return ndis_packet after rtw_xmitframe_coalesce */
		rtw_os_xmit_complete(rtlpriv, pxmitframe);

		break;
	} while (1);

	/* 3 2. aggregate same priority and same DA(AP or STA) frames */
	pfirstframe = pxmitframe;
	len = xmitframe_need_length(pfirstframe) + TXDESC_SIZE+(pfirstframe->pkt_offset*PACKET_OFFSET_SZ);
	pbuf_tail = len;
	pbuf = _RND8(pbuf_tail);

	/* check pkt amount in one bulk */
	descCount = 0;
	bulkPtr = bulkSize;
	if (pbuf < bulkPtr)
		descCount++;
	else {
		descCount = 0;
		bulkPtr = ((pbuf / bulkSize) + 1) * bulkSize; /* round to next bulkSize */
	}

	/* ULLI sta enqueue code into tx queue */

	/* dequeue same priority packet from station tx queue */
	psta = pfirstframe->tx_attrib.psta;
	switch (pfirstframe->tx_attrib.tx_priority) {
	case 1:
	case 2:
		ptxservq = &(psta->sta_xmitpriv.bk_q);
		phwxmit = &pxmitpriv->hwxmits[3];
		break;

	case 4:
	case 5:
		ptxservq = &(psta->sta_xmitpriv.vi_q);
		phwxmit = &pxmitpriv->hwxmits[1];
		break;

	case 6:
	case 7:
		ptxservq = &(psta->sta_xmitpriv.vo_q);
		phwxmit = &pxmitpriv->hwxmits[0];
		break;

	case 0:
	case 3:
	default:
		ptxservq = &(psta->sta_xmitpriv.be_q);
		phwxmit = &pxmitpriv->hwxmits[2];
		break;
	}
/*
 * DBG_8192C("==> pkt_no=%d,pkt_len=%d,len=%d,RND8_LEN=%d,pkt_offset=0x%02x\n",
 * 	pxmitframe->agg_num,pxmitframe->attrib.last_txcmdsz,len,pbuf,pxmitframe->pkt_offset );
 */

	spin_lock_bh(&pxmitpriv->lock);

	list_for_each(item, get_list_head(&ptxservq->sta_pending)) {
		pxmitframe = list_entry(item, struct xmit_frame, list);

		pxmitframe->agg_num = 0; 	/* not first frame of aggregation */
		pxmitframe->pkt_offset = 0; 	/* not first frame of aggregation, no need to reserve offset */

		len = xmitframe_need_length(pxmitframe) + TXDESC_SIZE + (pxmitframe->pkt_offset*PACKET_OFFSET_SZ);

		if (_RND8(pbuf + len) > MAX_XMITBUF_SZ) {
		/* if (_RND8(pbuf + len) > (MAX_XMITBUF_SZ/2))//to do : for TX TP finial tune , Georgia 2012-0323 */

			/* DBG_8192C("%s....len> MAX_XMITBUF_SZ\n",__FUNCTION__); */
			pxmitframe->agg_num = 1;
			pxmitframe->pkt_offset = 1;
			break;
		}
		list_del_init(&pxmitframe->list);

#ifndef IDEA_CONDITION
		/*  suppose only data frames would be in queue */
		if (pxmitframe->frame_tag != DATA_FRAMETAG) {
			rtw_free_xmitframe(pxmitpriv, pxmitframe);
			continue;
		}

		/* TID 0~15 */
		if ((pxmitframe->attrib.priority < 0) ||
		    (pxmitframe->attrib.priority > 15)) {
			rtw_free_xmitframe(pxmitpriv, pxmitframe);
			continue;
		}
#endif

		/* pxmitframe->pxmitbuf = pxmitbuf; */
		pxmitframe->buf_addr = pxmitbuf->pbuf + pbuf;

		if (rtw_xmitframe_coalesce(rtlpriv, pxmitframe->skb, pxmitframe) == false) {
			dev_dbg(&(rtlpriv->ndev->dev), "%s coalesce failed \n", __FUNCTION__);
			rtw_free_xmitframe(pxmitpriv, pxmitframe);
			continue;
		}

		/*
		 * DBG_8192C("==> pxmitframe->attrib.priority:%d\n",pxmitframe->attrib.priority);
		 * always return ndis_packet after rtw_xmitframe_coalesce
		 */
		rtw_os_xmit_complete(rtlpriv, pxmitframe);

		/* (len - TXDESC_SIZE) == pxmitframe->attrib.last_txcmdsz */
		update_txdesc(pxmitframe, pxmitframe->buf_addr, pxmitframe->tx_attrib.last_txcmdsz, true);

		/* don't need xmitframe any more */
		rtw_free_xmitframe(pxmitpriv, pxmitframe);

		/* handle pointer and stop condition */
		pbuf_tail = pbuf + len;
		pbuf = _RND8(pbuf_tail);


		pfirstframe->agg_num++;
		if (MAX_TX_AGG_PACKET_NUMBER == pfirstframe->agg_num)
			break;

		if (pbuf < bulkPtr) {
			descCount++;
			if (descCount == pHalData->UsbTxAggDescNum)
				break;
		} else {
			descCount = 0;
			bulkPtr = ((pbuf / bulkSize) + 1) * bulkSize;
		}
	}	/* end while( aggregate same priority and same DA(AP or STA) frames) */


	/* ULLI : Huh ???, must follow this ... */

	if (list_empty(&ptxservq->sta_pending.list))
		list_del_init(&ptxservq->tx_pending);

	spin_unlock_bh(&pxmitpriv->lock);
	if ((pfirstframe->tx_attrib.ether_type != 0x0806) &&
	    (pfirstframe->tx_attrib.ether_type != 0x888e) &&
	    (pfirstframe->tx_attrib.ether_type != 0x88b4) &&
	    (pfirstframe->tx_attrib.dhcp_pkt != 1)) {
		rtw_issue_addbareq_cmd(rtlpriv, pfirstframe);
	}
	/* 3 3. update first frame txdesc */
	if ((pbuf_tail % bulkSize) == 0) {
		/* remove pkt_offset */
		pbuf_tail -= PACKET_OFFSET_SZ;
		pfirstframe->buf_addr += PACKET_OFFSET_SZ;
		pfirstframe->pkt_offset--;
		/* DBG_8192C("$$$$$ buf size equal to USB block size $$$$$$\n"); */
	}

	update_txdesc(pfirstframe, pfirstframe->buf_addr, pfirstframe->tx_attrib.last_txcmdsz, true);

	/* 3 4. write xmit buffer to USB FIFO */
	ff_hwaddr = rtw_get_ff_hwaddr(pfirstframe);
/* DBG_8192C("%s ===================================== write port,buf_size(%d) \n",__FUNCTION__,pbuf_tail); */
	/* xmit address == ((xmit_frame*)pxmitbuf->priv_data)->buf_addr */
	 _rtlw_usb_transmit(rtlpriv, ff_hwaddr, pbuf_tail, pxmitbuf);


	/* 3 5. update statisitc */
	pbuf_tail -= (pfirstframe->agg_num * TXDESC_SIZE);
	pbuf_tail -= (pfirstframe->pkt_offset * PACKET_OFFSET_SZ);


	rtw_count_tx_stats(rtlpriv, pfirstframe, pbuf_tail);

	rtw_free_xmitframe(pxmitpriv, pfirstframe);

	return true;
}

static int32_t xmitframe_direct(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	int32_t res = _SUCCESS;
	/* DBG_8192C("==> %s \n",__FUNCTION__); */

	res = rtw_xmitframe_coalesce(rtlpriv, pxmitframe->skb, pxmitframe);
	if (res == _SUCCESS) {
		rtw_dump_xframe(rtlpriv, pxmitframe);
	} else {
		dev_dbg(&(rtlpriv->ndev->dev), "==> %s xmitframe_coalsece failed\n", __FUNCTION__);
	}

	return res;
}

/*
 * Return
 *	true	dump packet directly
 *	false	enqueue packet
 */
static int32_t pre_xmitframe(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	int32_t res;
	struct xmit_buf *pxmitbuf = NULL;
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct tx_pkt_attrib *pattrib = &pxmitframe->tx_attrib;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;

	spin_lock_bh(&pxmitpriv->lock);

	/* DBG_8192C("==> %s \n",__FUNCTION__); */

	if (rtw_txframes_sta_ac_pending(rtlpriv, pattrib)) {
		/* DBG_8192C("enqueue AC(%d)\n",pattrib->priority); */
		goto enqueue;
	}


	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY|_FW_UNDER_LINKING) == true)
		goto enqueue;

	pxmitbuf = rtw_alloc_xmitbuf(pxmitpriv);
	if (pxmitbuf == NULL)
		goto enqueue;

	spin_unlock_bh(&pxmitpriv->lock);

	pxmitframe->pxmitbuf = pxmitbuf;
	pxmitframe->buf_addr = pxmitbuf->pbuf;
	pxmitbuf->priv_data = pxmitframe;

	if (xmitframe_direct(rtlpriv, pxmitframe) != _SUCCESS) {
		rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
		rtw_free_xmitframe(pxmitpriv, pxmitframe);
	}

	return true;

enqueue:
	res = rtw_xmitframe_enqueue(rtlpriv, pxmitframe);
	spin_unlock_bh(&pxmitpriv->lock);

	if (res != _SUCCESS) {
		rtw_free_xmitframe(pxmitpriv, pxmitframe);

		/* Trick, make the statistics correct */
		pxmitpriv->tx_pkts--;
		pxmitpriv->tx_drop++;
		return true;
	}

	return false;
}

int32_t rtl8812au_mgnt_xmit(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe)
{
	return rtw_dump_xframe(rtlpriv, pmgntframe);
}

/*
 * Return
 *	true	dump packet directly ok
 *	false	temporary can't transmit packets to hardware
 */
int32_t rtl8812au_hal_xmit(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	return pre_xmitframe(rtlpriv, pxmitframe);
}

int32_t	 rtl8812au_hal_xmitframe_enqueue(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe)
{
	struct xmit_priv 	*pxmitpriv = &rtlpriv->xmitpriv;
	int32_t err;

	err = rtw_xmitframe_enqueue(rtlpriv, pxmitframe);
	if (err != _SUCCESS) {
		rtw_free_xmitframe(pxmitpriv, pxmitframe);

		/* Trick, make the statistics correct */
		pxmitpriv->tx_pkts--;
		pxmitpriv->tx_drop++;
	} else {
		tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
	}

	return err;

}

void _dbg_dump_tx_info(struct rtl_priv	*rtlpriv,int frame_tag, uint8_t *ptxdesc)
{
}

/*
 * Description:
 *	Aggregation packets and send to hardware
 *
 * Return:
 *	0	Success
 *	-1	Hardware resource(TX FIFO) not ready
 *	-2	Software resource(xmitbuf) not ready
 */



u8 BWMapping_8812(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib)
{
	uint8_t	BWSettingOfDesc = 0;

	/*
	 * DBG_871X("BWMapping pHalData->CurrentChannelBW %d, pattrib->bwmode %d \n",pHalData->CurrentChannelBW,pattrib->bwmode);
	 */

	if (rtlpriv->phy.current_chan_bw == CHANNEL_WIDTH_80) {
		if (pattrib->bwmode == CHANNEL_WIDTH_80)
			BWSettingOfDesc= 2;
		else if (pattrib->bwmode == CHANNEL_WIDTH_40)
			BWSettingOfDesc = 1;
		else
			BWSettingOfDesc = 0;
	} else if(rtlpriv->phy.current_chan_bw == CHANNEL_WIDTH_40) {
		if((pattrib->bwmode == CHANNEL_WIDTH_40) || (pattrib->bwmode == CHANNEL_WIDTH_80))
			BWSettingOfDesc = 1;
		else
			BWSettingOfDesc = 0;
	} else
		BWSettingOfDesc = 0;

	return BWSettingOfDesc;
}

/* ULLI check secondary channel mappen */

u8 SCMapping_8812(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib)
{
	uint8_t	SCSettingOfDesc = 0;
	struct rtl_mac *mac = &(rtlpriv->mac80211);

	/*
	 * DBG_871X("SCMapping: pHalData->CurrentChannelBW %d, pHalData->nCur80MhzPrimeSC %d, pHalData->nCur40MhzPrimeSC %d \n",pHalData->CurrentChannelBW,pHalData->nCur80MhzPrimeSC,pHalData->nCur40MhzPrimeSC);
	 */

	if (rtlpriv->phy.current_chan_bw == CHANNEL_WIDTH_80) {
		if(pattrib->bwmode == CHANNEL_WIDTH_80) {
			SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;
		} else if(pattrib->bwmode == CHANNEL_WIDTH_40) {
			if(mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER)
				SCSettingOfDesc = VHT_DATA_SC_40_LOWER_OF_80MHZ;
			else if(mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER)
				SCSettingOfDesc = VHT_DATA_SC_40_UPPER_OF_80MHZ;
			else
				RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "SCMapping: Not Correct Primary40MHz Setting \n");
		} else {
			if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER))
				SCSettingOfDesc = VHT_DATA_SC_20_LOWEST_OF_80MHZ;
			else if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER))
				SCSettingOfDesc = VHT_DATA_SC_20_LOWER_OF_80MHZ;
			else if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER))
				SCSettingOfDesc = VHT_DATA_SC_20_UPPER_OF_80MHZ;
			else if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER))
				SCSettingOfDesc = VHT_DATA_SC_20_UPPERST_OF_80MHZ;
			else
				RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "SCMapping: Not Correct Primary40MHz Setting \n");
		}
	} else if(rtlpriv->phy.current_chan_bw== CHANNEL_WIDTH_40) {
		/*
		 * DBG_871X("SCMapping: HT Case: pHalData->CurrentChannelBW %d, pHalData->nCur40MhzPrimeSC %d \n",pHalData->CurrentChannelBW,pHalData->nCur40MhzPrimeSC);
		 */

		if(pattrib->bwmode == CHANNEL_WIDTH_40) {
			SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;
		} else if(pattrib->bwmode == CHANNEL_WIDTH_20) {
			if(mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER) {
				SCSettingOfDesc = VHT_DATA_SC_20_UPPER_OF_80MHZ;
			} else if(mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER) {
				SCSettingOfDesc = VHT_DATA_SC_20_LOWER_OF_80MHZ;
			} else 		{
				SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;
			}

		}
	} else {
		SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;
	}

	return SCSettingOfDesc;
}

/** RX */


static u8 odm_QueryRxPwrPercentage(s8 AntPower)
{
	if ((AntPower <= -100) || (AntPower >= 20)) {
		return	0;
	} else if (AntPower >= 0) {
		return	100;
	} else {
		return	(100+AntPower);
	}

}

/*
 * 2012/01/12 MH MOve some signal strength smooth method to MP HAL layer.
 * IF other SW team do not support the feature, remove this section.??
 */

/*
 * ULLI : this function is in rtlwifi
 * ULLI : but with other values
 */
static long rtl_signal_scale_mapping(struct rtl_priv *rtlpriv, int32_t CurrSig)
{
	int32_t RetSig;

	if (CurrSig >= 51 && CurrSig <= 100) {
		RetSig = 100;
	} else if (CurrSig >= 41 && CurrSig <= 50) {
		RetSig = 80 + ((CurrSig - 40)*2);
	} else if (CurrSig >= 31 && CurrSig <= 40) {
		RetSig = 66 + (CurrSig - 30);
	} else if (CurrSig >= 21 && CurrSig <= 30) {
		RetSig = 54 + (CurrSig - 20);
	} else if (CurrSig >= 10 && CurrSig <= 20) {
		RetSig = 42 + (((CurrSig - 10) * 2) / 3);
	} else if (CurrSig >= 5 && CurrSig <= 9) {
		RetSig = 22 + (((CurrSig - 5) * 3) / 2);
	} else if (CurrSig >= 1 && CurrSig <= 4) {
		RetSig = 6 + (((CurrSig - 1) * 3) / 2);
	} else {
		RetSig = CurrSig;
	}

	return RetSig;
}

static u8 odm_EVMdbToPercentage(s8 Value)
{
	/*
	 *  -33dB~0dB to 0%~99%
	 */
	s8 ret_val;

	ret_val = Value;
	/* ret_val /= 2; */

	/* ODM_RT_DISP(FRX, RX_PHY_SQ, ("EVMdbToPercentage92C Value=%d / %x \n", ret_val, ret_val)); */

	if (ret_val >= 0)
		ret_val = 0;
	if (ret_val <= -33)
		ret_val = -33;

	ret_val = 0 - ret_val;
	ret_val *= 3;

	if (ret_val == 99)
		ret_val = 100;

	return ret_val;
}

static u8 odm_EVMdbm_JaguarSeries(s8 Value)
{
	s8 ret_val = Value;

	/* -33dB~0dB to 33dB ~ 0dB */
	if (ret_val == -128)
		ret_val = 127;
	else if (ret_val < 0)
		ret_val = 0 - ret_val;

	ret_val  = ret_val >> 1;
	return ret_val;
}

static u16 odm_Cfo(s8 Value)
{
	s16  ret_val;

	if (Value < 0) {
		ret_val = 0 - Value;
		ret_val = (ret_val << 1) + (ret_val >> 1);	/*  *2.5~=312.5/2^7 */
		ret_val = ret_val | BIT(12);  			/* set BIT(12) as 1 for negative cfo */
	} else {
		ret_val = Value;
		ret_val = (ret_val << 1) + (ret_val>>1);	/*  *2.5~=312.5/2^7 */
	}
	return ret_val;
}


/*
 * Endianness before calling this API
 */

static void odm_Process_RSSIForDM(struct _rtw_dm *pDM_Odm, struct _ODM_Phy_Status_Info_ *pPhyInfo,
	struct _ODM_Per_Pkt_Info_ *pPktinfo);

static void query_rxphystatus(struct _rtw_dm *	pDM_Odm, struct _ODM_Phy_Status_Info_ *pPhyInfo,
	u8 *pPhyStatus, struct _ODM_Per_Pkt_Info_ *pPktinfo)
{
	struct rtl_priv *rtlpriv = pDM_Odm->rtlpriv;
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	u8	i, Max_spatial_stream;
	s8	rx_pwr[4], rx_pwr_all = 0;
	u8	EVM, EVMdbm, PWDB_ALL = 0, PWDB_ALL_BT;
	u8	RSSI, total_rssi = 0;
	u8	isCCKrate = 0;
	u8	rf_rx_num = 0;
	u8	cck_highpwr = 0;
	u8	LNA_idx, VGA_idx;

	struct rw_fwinfo_8821au *pPhyStaRpt = (struct rw_fwinfo_8821au *) pPhyStatus;

	if (pPktinfo->DataRate <= DESC_RATE54M) {
		switch (pPhyStaRpt->r_RFMOD) {
		case 1:
			if (pPhyStaRpt->sub_chnl == 0)
				pPhyInfo->BandWidth = 1;
			else
				pPhyInfo->BandWidth = 0;
			break;

		case 2:
			if (pPhyStaRpt->sub_chnl == 0)
				pPhyInfo->BandWidth = 2;
			else if (pPhyStaRpt->sub_chnl == 9 || pPhyStaRpt->sub_chnl == 10)
				pPhyInfo->BandWidth = 1;
			else
				pPhyInfo->BandWidth = 0;
			break;

		default:
		case 0:
			pPhyInfo->BandWidth = 0;
			break;
		}
	}

	if (pPktinfo->DataRate <= DESC_RATE11M)
		isCCKrate = true;
	else
		isCCKrate = false;

	pPhyInfo->RxMIMOSignalQuality[RF90_PATH_A] = -1;
	pPhyInfo->RxMIMOSignalQuality[RF90_PATH_B] = -1;

	if (isCCKrate) {
		u8 cck_agc_rpt;

		/*
		 *  (1)Hardware does not provide RSSI for CCK
		 *  (2)PWDB, Average PWDB cacluated by hardware (for rate adaptive)
		 */

		/* if (pHalData->eRFPowerState == eRfOn) */
			cck_highpwr = rtlpriv->phy.cck_high_power;
		/*
		 * else
		 * 	cck_highpwr = false;
		 */

		cck_agc_rpt =  pPhyStaRpt->cfosho[0] ;

		LNA_idx = ((cck_agc_rpt & 0xE0) >> 5);
		VGA_idx = (cck_agc_rpt & 0x1F);
		if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
			switch (LNA_idx) {
			case 7:
				if (VGA_idx <= 27)
					rx_pwr_all = -100 + 2*(27-VGA_idx);	/* VGA_idx = 27~2 */
				else
					rx_pwr_all = -100;
				break;

			case 6:
				rx_pwr_all = -48 + 2*(2-VGA_idx);	/* VGA_idx = 2~0 */
				break;

			case 5:
				rx_pwr_all = -42 + 2*(7-VGA_idx);	/* VGA_idx = 7~5 */
				break;

			case 4:
				rx_pwr_all = -36 + 2*(7-VGA_idx);	/* VGA_idx = 7~4 */
				break;

			case 3:
				/* rx_pwr_all = -28 + 2*(7-VGA_idx); */	/* VGA_idx = 7~0 */
				rx_pwr_all = -24 + 2*(7-VGA_idx); 	/* VGA_idx = 7~0 */
				break;

			case 2:
				if (cck_highpwr)
					rx_pwr_all = -12 + 2*(5-VGA_idx);	/* VGA_idx = 5~0 */
				else
					rx_pwr_all = -6 + 2*(5-VGA_idx);
				break;
			case 1:
				rx_pwr_all = 8-2*VGA_idx;
				break;
			case 0:
				rx_pwr_all = 14-2*VGA_idx;
				break;
			default:
				/* DbgPrint("CCK Exception default\n"); */
				break;
			}
			rx_pwr_all += 6;
			PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);
			if (cck_highpwr == false) {
				if (PWDB_ALL >= 80)
					PWDB_ALL = ((PWDB_ALL-80)<<1)+((PWDB_ALL-80)>>1)+80;
				else if ((PWDB_ALL <= 78) && (PWDB_ALL >= 20))
					PWDB_ALL += 3;

				if (PWDB_ALL > 100)
					PWDB_ALL = 100;
			}
		} else if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
			s8 Pout = -6;

			switch (LNA_idx) {
			case 5:
				rx_pwr_all = Pout - 32 - (2*VGA_idx);
				break;
			case 4:
				rx_pwr_all = Pout - 24 - (2*VGA_idx);
				break;
			case 2:
				rx_pwr_all = Pout - 11 - (2*VGA_idx);
				break;
			case 1:
				rx_pwr_all = Pout + 5 - (2*VGA_idx);
				break;
			case 0:
				rx_pwr_all = Pout + 21 - (2*VGA_idx);
				break;
			}
			PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);
		}

		pPhyInfo->RxPWDBAll = PWDB_ALL;
		/*
		 * if (pPktinfo->StationID == 0) {
		 * 	DbgPrint("CCK: LNA_idx = %d, VGA_idx = %d, pPhyInfo->RxPWDBAll = %d\n",
		 * 		LNA_idx, VGA_idx, pPhyInfo->RxPWDBAll);
		 * }
		 */
		pPhyInfo->BTRxRSSIPercentage = PWDB_ALL;
		pPhyInfo->RecvSignalPower = rx_pwr_all;
		/*
		 * (3) Get Signal Quality (EVM)
		 */

		if (pPktinfo->bPacketMatchBSSID) {
			u8	SQ, SQ_rpt;

			if (pPhyInfo->RxPWDBAll > 40) {
				SQ = 100;
			} else {
				SQ_rpt = pPhyStaRpt->pwdb_all;

				if (SQ_rpt > 64)
					SQ = 0;
				else if (SQ_rpt < 20)
					SQ = 100;
				else
					SQ = ((64-SQ_rpt) * 100) / 44;

			}

			/* DbgPrint("cck SQ = %d\n", SQ); */
			pPhyInfo->SignalQuality = SQ;
			pPhyInfo->RxMIMOSignalQuality[RF90_PATH_A] = SQ;
			pPhyInfo->RxMIMOSignalQuality[RF90_PATH_B] = -1;
		}
	} else {
		/* is OFDM rate */

		/*
		 * (1)Get RSSI for OFDM rate
		 */

		for (i = RF90_PATH_A; i < ODM_RF_PATH_MAX; i++) {
			/*
			 *  2008/01/30 MH we will judge RF RX path now.
			 * DbgPrint("pDM_Odm->RFPathRxEnable = %x\n", pDM_Odm->RFPathRxEnable);
			 */
			if (rtlpriv->dm.rfpath_rxenable[i]) {
				rf_rx_num++;
			}
			/*
			 * else
			 * 	continue;
			 */

			/*
			 * 2012.05.25 LukeLee: Testchip AGC report is wrong, it should be restored back to old formula in MP chip
			 * if ((pDM_Odm->SupportICType & (ODM_RTL8812|ODM_RTL8821)) && (!pDM_Odm->bIsMPChip))
			 */

			 rx_pwr[i] = (pPhyStaRpt->gain_trsw[i]&0x7F) - 110;

			/*
			 * else
			 * 	rx_pwr[i] = ((pPhyStaRpt->gain_trsw[i]& 0x3F)*2) - 110;  //OLD FORMULA
			 */

			pPhyInfo->RxPwr[i] = rx_pwr[i];

			/* Translate DBM to percentage. */
			RSSI = odm_QueryRxPwrPercentage(rx_pwr[i]);

			total_rssi += RSSI;
			/* RT_DISP(FRX, RX_PHY_SS, ("RF-%d RXPWR=%x RSSI=%d\n", i, rx_pwr[i], RSSI)); */



			pPhyInfo->RxMIMOSignalStrength[i] = (u8) RSSI;

			/* Get Rx snr value in DB */
			pPhyInfo->RxSNR[i] = pPhyStaRpt->rxsnr[i]/2;

			/*
			 *  (2) CFO_short  & CFO_tail
			 */

			pPhyInfo->Cfo_short[i] = odm_Cfo((pPhyStaRpt->cfosho[i]));
			pPhyInfo->Cfo_tail[i] = odm_Cfo((pPhyStaRpt->cfotail[i]));

			/* Record Signal Strength for next packet */
			if (pPktinfo->bPacketMatchBSSID) {
				;
			}
		}


		/*
		 *  (3)PWDB, Average PWDB cacluated by hardware (for rate adaptive)
		 *
		 * 2012.05.25 LukeLee: Testchip AGC report is wrong, it should be restored back to old formula in MP chip
		 */
		if ((!IS_NORMAL_CHIP(rtlhal->version)))
			rx_pwr_all = (pPhyStaRpt->pwdb_all & 0x7f) - 110;
		else
			rx_pwr_all = (((pPhyStaRpt->pwdb_all) >> 1) & 0x7f) - 110;	 /* OLD FORMULA */

		PWDB_ALL_BT = PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);


		pPhyInfo->RxPWDBAll = PWDB_ALL;
#if 0		/* Currently no value in rtlwifi for this */
		RT_TRACE(rtlpriv, COMP_RSSI_MONITOR, DBG_LOUD, "ODM OFDM RSSI=%d\n",pPhyInfo->RxPWDBAll);
#endif
		pPhyInfo->BTRxRSSIPercentage = PWDB_ALL_BT;
		pPhyInfo->RxPower = rx_pwr_all;
		pPhyInfo->RecvSignalPower = rx_pwr_all;

		/*
		 * DbgPrint("OFDM: pPhyInfo->RxPWDBAll = %d, pPhyInfo->RxMIMOSignalStrength[0] = %d, pPhyInfo->RxMIMOSignalStrength[1] = %d\n",
		 *	pPhyInfo->RxPWDBAll, pPhyInfo->RxMIMOSignalStrength[0], pPhyInfo->RxMIMOSignalStrength[1]);
		 */


		{	/* pMgntInfo->CustomerID != RT_CID_819x_Lenovo */
			/*
			 * (4)EVM of OFDM rate
			 */

			if ((pPktinfo->DataRate >= DESC_RATEMCS8)
			 && (pPktinfo->DataRate <= DESC_RATEMCS15))
				Max_spatial_stream = 2;
			else if ((pPktinfo->DataRate >= DESC_RATEVHT2SS_MCS0)
			      && (pPktinfo->DataRate <= DESC_RATEVHT2SS_MCS9))
				Max_spatial_stream = 2;
			else
				Max_spatial_stream = 1;

			for (i = 0; i < Max_spatial_stream; i++) {
				/*
				 *  Do not use shift operation like "rx_evmX >>= 1" because the compilor of free build environment
				 *  fill most significant bit to "zero" when doing shifting operation which may change a negative
				 *  value to positive one, then the dbm value (which is supposed to be negative)  is not correct anymore.
				 */
				EVM = odm_EVMdbToPercentage((pPhyStaRpt->rxevm[i]));	/* dbm */
				EVMdbm = odm_EVMdbm_JaguarSeries(pPhyStaRpt->rxevm[i]);
				/*
				 * RT_DISP(FRX, RX_PHY_SQ, ("RXRATE=%x RXEVM=%x EVM=%s%d\n",
				 * pPktinfo->DataRate, pPhyStaRpt->rxevm[i], "%", EVM));
				 */

				if (pPktinfo->bPacketMatchBSSID) {
					if (i == RF90_PATH_A) {	/* Fill value in RFD, Get the first spatial stream only */
						pPhyInfo->SignalQuality = EVM;
					}
					pPhyInfo->RxMIMOSignalQuality[i] = EVM;
					pPhyInfo->RxMIMOEVMdbm[i] = EVMdbm;
				}
			}
		}
		/* 2 For dynamic ATC switch */
			if (pPktinfo->bPacketMatchBSSID) {
				/* 3 Update CFO report for path-A & path-B */
				for (i = RF90_PATH_A; i < ODM_RF_PATH_MAX; i++) {
					pDM_Odm->CFO_tail[i] = (int)pPhyStaRpt->cfotail[i];
				}

				/* 3 Update packet counter */
				if (pDM_Odm->packetCount == 0xffffffff)
					pDM_Odm->packetCount = 0;
				else
					pDM_Odm->packetCount++;

				/*
				 * ODM_RT_TRACE(pDM_Odm, ODM_COMP_DYNAMIC_ATC, DBG_LOUD,
				 * 	("pPhyStaRpt->path_cfotail[i] = 0x%x, pDM_Odm->CFO_tail[i] = 0x%x\n", pPhyStaRpt->path_cfotail[0], pDM_Odm->CFO_tail[1]));
				 */
			}
	}
	/* DbgPrint("isCCKrate= %d, pPhyInfo->SignalStrength=%d % PWDB_AL=%d rf_rx_num=%d\n", isCCKrate, pPhyInfo->SignalStrength, PWDB_ALL, rf_rx_num); */

	/*
	 * UI BSS List signal strength(in percentage), make it good looking, from 0~100.
	 * It is assigned to the BSS List in GetValueFromBeaconOrProbeRsp().
	 */
	if (isCCKrate) {
		pPhyInfo->SignalStrength = (u8)(rtl_signal_scale_mapping(rtlpriv, PWDB_ALL));	/* PWDB_ALL; */
	} else {
		if (rf_rx_num != 0) {
			/* ULLI crap inside this call */
			pPhyInfo->SignalStrength = (u8)(rtl_signal_scale_mapping(rtlpriv, total_rssi /= rf_rx_num));
		}
	}
	pDM_Odm->RxPWDBAve = pDM_Odm->RxPWDBAve + pPhyInfo->RxPWDBAll;

	rtldm->fat_table.antsel_rx_keep_0 = pPhyStaRpt->antidx_anta;

	/*
	 * DbgPrint("pPhyStaRpt->antidx_anta = %d, pPhyStaRpt->antidx_antb = %d, pPhyStaRpt->resvd_1 = %d",
	 * 	pPhyStaRpt->antidx_anta, pPhyStaRpt->antidx_antb, pPhyStaRpt->resvd_1);
	 */

	/*
	 * DbgPrint("----------------------------\n");
	 * DbgPrint("pPktinfo->StationID=%d, pPktinfo->DataRate=0x%x\n",pPktinfo->StationID, pPktinfo->DataRate);
	 * DbgPrint("pPhyStaRpt->gain_trsw[0]=0x%x, pPhyStaRpt->gain_trsw[1]=0x%x, pPhyStaRpt->pwdb_all=0x%x\n",
	 *	pPhyStaRpt->gain_trsw[0],pPhyStaRpt->gain_trsw[1], pPhyStaRpt->pwdb_all);
	 * DbgPrint("pPhyInfo->RxMIMOSignalStrength[0]=%d, pPhyInfo->RxMIMOSignalStrength[1]=%d, RxPWDBAll=%d\n",
	 *	pPhyInfo->RxMIMOSignalStrength[0], pPhyInfo->RxMIMOSignalStrength[1], pPhyInfo->RxPWDBAll);
	 */
	odm_Process_RSSIForDM(pDM_Odm, pPhyInfo, pPktinfo);
}


static void odm_Process_RSSIForDM(struct _rtw_dm *pDM_Odm, struct _ODM_Phy_Status_Info_ *pPhyInfo,
	struct _ODM_Per_Pkt_Info_ *pPktinfo)
{
	struct rtl_priv *rtlpriv = pDM_Odm->rtlpriv;
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);
	int32_t		UndecoratedSmoothedPWDB, UndecoratedSmoothedCCK, UndecoratedSmoothedOFDM, RSSI_Ave;
	u8		isCCKrate = 0;
	u8		RSSI_max, RSSI_min, i;
	uint32_t	OFDM_pkt = 0;
	uint32_t	Weighting = 0;

	struct sta_info *pEntry;

	if (pPktinfo->StationID == 0xFF)
		return;

	pEntry = rtldm->pODM_StaInfo[pPktinfo->StationID];

	if (!IS_STA_VALID(pEntry)) {
		return;
	}
	if ((!pPktinfo->bPacketMatchBSSID)) {
		return;
	}

	if (pPktinfo->bPacketBeacon)
		rtlpriv->dm.dbginfo.num_qry_beacon_pkt++;
	isCCKrate = (pPktinfo->DataRate <= DESC_RATE11M) ? true : false;
	pDM_Odm->RxRate = pPktinfo->DataRate;
	/*
	if (!isCCKrate)
	{
		DbgPrint("OFDM: pPktinfo->StationID=%d, isCCKrate=%d, pPhyInfo->RxPWDBAll=%d\n",
			pPktinfo->StationID, isCCKrate, pPhyInfo->RxPWDBAll);
	}
	*/
/*
	if (pDM_Odm->SupportICType == ODM_RTL8812)
	{
		pPATHDIV_T	pDM_PathDiv = &pDM_Odm->DM_PathDiv;
		if (pPktinfo->bPacketToSelf || pPktinfo->bPacketMatchBSSID)
		{
			if (pPktinfo->DataRate > DESC8812_RATE11M)
				ODM_PathStatistics_8812A(pDM_Odm, pPktinfo->StationID,
				pPhyInfo->RxMIMOSignalStrength[RF90_PATH_A], pPhyInfo->RxMIMOSignalStrength[RF90_PATH_B]);
		}
	}
*/
	/* -----------------Smart Antenna Debug Message------------------ */

	UndecoratedSmoothedCCK =  pEntry->rssi_stat.UndecoratedSmoothedCCK;
	UndecoratedSmoothedOFDM = pEntry->rssi_stat.UndecoratedSmoothedOFDM;
	UndecoratedSmoothedPWDB = pEntry->rssi_stat.UndecoratedSmoothedPWDB;

	if (pPktinfo->bPacketToSelf || pPktinfo->bPacketBeacon) {
		if (!isCCKrate) {	/* ofdm rate */
			if (pPhyInfo->RxMIMOSignalStrength[RF90_PATH_B] == 0) {
				RSSI_Ave = pPhyInfo->RxMIMOSignalStrength[RF90_PATH_A];
				pDM_Odm->RSSI_A = pPhyInfo->RxMIMOSignalStrength[RF90_PATH_A];
				pDM_Odm->RSSI_B = 0;
			} else{
				/*
				 * DbgPrint("pRfd->Status.RxMIMOSignalStrength[0] = %d, pRfd->Status.RxMIMOSignalStrength[1] = %d \n",
				 * 	pRfd->Status.RxMIMOSignalStrength[0], pRfd->Status.RxMIMOSignalStrength[1]);
				 */
				pDM_Odm->RSSI_A =  pPhyInfo->RxMIMOSignalStrength[RF90_PATH_A];
				pDM_Odm->RSSI_B = pPhyInfo->RxMIMOSignalStrength[RF90_PATH_B];

				if (pPhyInfo->RxMIMOSignalStrength[RF90_PATH_A] > pPhyInfo->RxMIMOSignalStrength[RF90_PATH_B]) {
					RSSI_max = pPhyInfo->RxMIMOSignalStrength[RF90_PATH_A];
					RSSI_min = pPhyInfo->RxMIMOSignalStrength[RF90_PATH_B];
				} else {
					RSSI_max = pPhyInfo->RxMIMOSignalStrength[RF90_PATH_B];
					RSSI_min = pPhyInfo->RxMIMOSignalStrength[RF90_PATH_A];
				}

				if ((RSSI_max - RSSI_min) < 3)
					RSSI_Ave = RSSI_max;
				else if ((RSSI_max - RSSI_min) < 6)
					RSSI_Ave = RSSI_max - 1;
				else if ((RSSI_max - RSSI_min) < 10)
					RSSI_Ave = RSSI_max - 2;
				else
					RSSI_Ave = RSSI_max - 3;
			}

			/* 1 Process OFDM RSSI */
			if (UndecoratedSmoothedOFDM <= 0) {
				/* initialize */
				UndecoratedSmoothedOFDM = pPhyInfo->RxPWDBAll;
			} else {
				if (pPhyInfo->RxPWDBAll > (uint32_t)UndecoratedSmoothedOFDM) {
					UndecoratedSmoothedOFDM =
						(((UndecoratedSmoothedOFDM)*(Rx_Smooth_Factor-1))
						+ (RSSI_Ave)) / (Rx_Smooth_Factor);
					UndecoratedSmoothedOFDM = UndecoratedSmoothedOFDM + 1;
				} else {
					UndecoratedSmoothedOFDM =
						(((UndecoratedSmoothedOFDM)*(Rx_Smooth_Factor-1))
						+ (RSSI_Ave)) / (Rx_Smooth_Factor);
				}
			}

			pEntry->rssi_stat.PacketMap = (pEntry->rssi_stat.PacketMap<<1) | BIT(0);
		} else {
			RSSI_Ave = pPhyInfo->RxPWDBAll;
			pDM_Odm->RSSI_A = (u8) pPhyInfo->RxPWDBAll;
			pDM_Odm->RSSI_B = 0xFF;

			/* 1 Process CCK RSSI */
			if (UndecoratedSmoothedCCK <= 0)	{	/*  initialize */
				UndecoratedSmoothedCCK = pPhyInfo->RxPWDBAll;
			} else 	{
				if (pPhyInfo->RxPWDBAll > (uint32_t)UndecoratedSmoothedCCK) {
					UndecoratedSmoothedCCK =
						(((UndecoratedSmoothedCCK)*(Rx_Smooth_Factor-1))
						+ (pPhyInfo->RxPWDBAll)) / (Rx_Smooth_Factor);
					UndecoratedSmoothedCCK = UndecoratedSmoothedCCK + 1;
				} else {
					UndecoratedSmoothedCCK =
						(((UndecoratedSmoothedCCK)*(Rx_Smooth_Factor-1))
						+ (pPhyInfo->RxPWDBAll)) / (Rx_Smooth_Factor);
				}
			}
			pEntry->rssi_stat.PacketMap = pEntry->rssi_stat.PacketMap<<1;
		}

		/* if (pEntry) */
		{
			/* 2011.07.28 LukeLee: modified to prevent unstable CCK RSSI */
			if (pEntry->rssi_stat.ValidBit >= 64)
				pEntry->rssi_stat.ValidBit = 64;
			else
				pEntry->rssi_stat.ValidBit++;

			for (i = 0; i < pEntry->rssi_stat.ValidBit; i++)
				OFDM_pkt += (u8)(pEntry->rssi_stat.PacketMap>>i)&BIT(0);

			if (pEntry->rssi_stat.ValidBit == 64) {
				Weighting = ((OFDM_pkt<<4) > 64)?64:(OFDM_pkt<<4);
				UndecoratedSmoothedPWDB = (Weighting*UndecoratedSmoothedOFDM+(64-Weighting)*UndecoratedSmoothedCCK)>>6;
			} else {
				if (pEntry->rssi_stat.ValidBit != 0)
					UndecoratedSmoothedPWDB = (OFDM_pkt*UndecoratedSmoothedOFDM+(pEntry->rssi_stat.ValidBit-OFDM_pkt)*UndecoratedSmoothedCCK)/pEntry->rssi_stat.ValidBit;
				else
					UndecoratedSmoothedPWDB = 0;
			}

			pEntry->rssi_stat.UndecoratedSmoothedCCK = UndecoratedSmoothedCCK;
			pEntry->rssi_stat.UndecoratedSmoothedOFDM = UndecoratedSmoothedOFDM;
			pEntry->rssi_stat.UndecoratedSmoothedPWDB = UndecoratedSmoothedPWDB;

			/*
			 * DbgPrint("OFDM_pkt=%d, Weighting=%d\n", OFDM_pkt, Weighting);
			 * DbgPrint("UndecoratedSmoothedOFDM=%d, UndecoratedSmoothedPWDB=%d, UndecoratedSmoothedCCK=%d\n",
			 * 	UndecoratedSmoothedOFDM, UndecoratedSmoothedPWDB, UndecoratedSmoothedCCK);
			 */

		}

	}
}

static int32_t  translate2dbm(uint8_t signal_strength_idx)
{
	int32_t	signal_power; // in dBm.


	// Translate to dBm (x=0.5y-95).
	signal_power = (int32_t)((signal_strength_idx + 1) >> 1);
	signal_power -= 95;

	return signal_power;
}


static void process_rssi(struct rtl_priv *rtlpriv,struct recv_frame *prframe)
{
	struct rx_pkt_attrib *pattrib = &prframe->attrib;
#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	struct signal_stat * signal_stat = &rtlpriv->recvpriv.signal_strength_data;
#endif //CONFIG_NEW_SIGNAL_STAT_PROCESS

	//DBG_8192C("process_rssi=> pattrib->rssil(%d) signal_strength(%d)\n ",pattrib->RecvSignalPower,pattrib->signal_strength);
	//if(pRfd->Status.bPacketToSelf || pRfd->Status.bPacketBeacon)
	{

	#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
		if(signal_stat->update_req) {
			signal_stat->total_num = 0;
			signal_stat->total_val = 0;
			signal_stat->update_req = 0;
		}

		signal_stat->total_num++;
		signal_stat->total_val  += pattrib->phy_info.SignalStrength;
		signal_stat->avg_val = signal_stat->total_val / signal_stat->total_num;
	#else //CONFIG_NEW_SIGNAL_STAT_PROCESS

		//rtlpriv->RxStats.RssiCalculateCnt++;	//For antenna Test
		if(rtlpriv->recvpriv.signal_strength_data.total_num++ >= PHY_RSSI_SLID_WIN_MAX)
		{
			rtlpriv->recvpriv.signal_strength_data.total_num = PHY_RSSI_SLID_WIN_MAX;
			last_rssi = rtlpriv->recvpriv.signal_strength_data.elements[rtlpriv->recvpriv.signal_strength_data.index];
			rtlpriv->recvpriv.signal_strength_data.total_val -= last_rssi;
		}
		rtlpriv->recvpriv.signal_strength_data.total_val  +=pattrib->phy_info.SignalStrength;

		rtlpriv->recvpriv.signal_strength_data.elements[rtlpriv->recvpriv.signal_strength_data.index++] = pattrib->phy_info.SignalStrength;
		if(rtlpriv->recvpriv.signal_strength_data.index >= PHY_RSSI_SLID_WIN_MAX)
			rtlpriv->recvpriv.signal_strength_data.index = 0;


		tmp_val = rtlpriv->recvpriv.signal_strength_data.total_val/rtlpriv->recvpriv.signal_strength_data.total_num;

		if(rtlpriv->recvpriv.is_signal_dbg) {
			rtlpriv->recvpriv.signal_strength= rtlpriv->recvpriv.signal_strength_dbg;
			rtlpriv->recvpriv.rssi=(s8)translate2dbm((uint8_t)rtlpriv->recvpriv.signal_strength_dbg);
		} else {
			rtlpriv->recvpriv.signal_strength= tmp_val;
			rtlpriv->recvpriv.rssi=(s8)translate2dbm((uint8_t)tmp_val);
		}

		RT_TRACE(_module_rtl871x_recv_c_,_drv_info_,("UI RSSI = %d, ui_rssi.TotalVal = %d, ui_rssi.TotalNum = %d\n", tmp_val, rtlpriv->recvpriv.signal_strength_data.total_val,rtlpriv->recvpriv.signal_strength_data.total_num));
	#endif //CONFIG_NEW_SIGNAL_STAT_PROCESS
	}

}// Process_UI_RSSI_8192C



static void process_link_qual(struct rtl_priv *rtlpriv,struct recv_frame *prframe)
{
 	struct rx_pkt_attrib *pattrib;
#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	struct signal_stat * signal_stat;
#endif //CONFIG_NEW_SIGNAL_STAT_PROCESS

	if(prframe == NULL || rtlpriv==NULL){
		return;
	}

	pattrib = &prframe->attrib;
#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	signal_stat = &rtlpriv->recvpriv.signal_qual_data;
#endif //CONFIG_NEW_SIGNAL_STAT_PROCESS

	//DBG_8192C("process_link_qual=> pattrib->signal_qual(%d)\n ",pattrib->signal_qual);

#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	if(signal_stat->update_req) {
		signal_stat->total_num = 0;
		signal_stat->total_val = 0;
		signal_stat->update_req = 0;
	}

	signal_stat->total_num++;
	signal_stat->total_val  += pattrib->phy_info.SignalQuality;
	signal_stat->avg_val = signal_stat->total_val / signal_stat->total_num;

#else //CONFIG_NEW_SIGNAL_STAT_PROCESS
	if(pattrib->phy_info.SignalQuality != 0)
	{
			//
			// 1. Record the general EVM to the sliding window.
			//
			if(rtlpriv->recvpriv.signal_qual_data.total_num++ >= PHY_LINKQUALITY_SLID_WIN_MAX)
			{
				rtlpriv->recvpriv.signal_qual_data.total_num = PHY_LINKQUALITY_SLID_WIN_MAX;
				last_evm = rtlpriv->recvpriv.signal_qual_data.elements[rtlpriv->recvpriv.signal_qual_data.index];
				rtlpriv->recvpriv.signal_qual_data.total_val -= last_evm;
			}
			rtlpriv->recvpriv.signal_qual_data.total_val += pattrib->phy_info.SignalQuality;

			rtlpriv->recvpriv.signal_qual_data.elements[rtlpriv->recvpriv.signal_qual_data.index++] = pattrib->phy_info.SignalQuality;
			if(rtlpriv->recvpriv.signal_qual_data.index >= PHY_LINKQUALITY_SLID_WIN_MAX)
				rtlpriv->recvpriv.signal_qual_data.index = 0;

			RT_TRACE(_module_rtl871x_recv_c_,_drv_info_,("Total SQ=%d  pattrib->signal_qual= %d\n", rtlpriv->recvpriv.signal_qual_data.total_val, pattrib->phy_info.SignalQuality));

			// <1> Showed on UI for user, in percentage.
			tmpVal = rtlpriv->recvpriv.signal_qual_data.total_val/rtlpriv->recvpriv.signal_qual_data.total_num;
			rtlpriv->recvpriv.signal_qual=(uint8_t)tmpVal;

	}
	else
	{
		RT_TRACE(_module_rtl871x_recv_c_,_drv_err_,(" pattrib->signal_qual =%d\n", pattrib->phy_info.SignalQuality));
	}
#endif //CONFIG_NEW_SIGNAL_STAT_PROCESS

}

static void process_phy_info(struct rtl_priv *rtlpriv, struct recv_frame *precvframe)
{

	//
	// Check RSSI
	//
	process_rssi(rtlpriv, precvframe);
	//
	// Check PWDB.
	//
	//process_PWDB(rtlpriv, precvframe);

	//UpdateRxSignalStatistics8192C(rtlpriv, pRfd);
	//
	// Check EVM
	//
	process_link_qual(rtlpriv,  precvframe);

}

void rtl8812_query_rx_desc_status(struct rtl_priv *rtlpriv,
				   struct rx_pkt_attrib	*pattrib,
				   struct recv_frame *precvframe, uint8_t *pdesc)
{
	memset(pattrib, 0, sizeof(struct rx_pkt_attrib));

	//Offset 0
	pattrib->pkt_len = (u16)GET_RX_STATUS_DESC_PKT_LEN(pdesc);//(le32_to_cpu(pdesc->rxdw0)&0x00003fff)
	pattrib->crc_err = (uint8_t)GET_RX_STATUS_DESC_CRC32(pdesc);//((le32_to_cpu(pdesc->rxdw0) >> 14) & 0x1);
	pattrib->icv_err = (uint8_t)GET_RX_STATUS_DESC_ICV(pdesc);//((le32_to_cpu(pdesc->rxdw0) >> 15) & 0x1);
	pattrib->drvinfo_sz = (uint8_t)GET_RX_STATUS_DESC_DRVINFO_SIZE(pdesc) * 8;//((le32_to_cpu(pdesc->rxdw0) >> 16) & 0xf) * 8;//uint 2^3 = 8 bytes
	pattrib->encrypt = (uint8_t)GET_RX_STATUS_DESC_SECURITY(pdesc);//((le32_to_cpu(pdesc->rxdw0) >> 20) & 0x7);
	pattrib->qos = (uint8_t)GET_RX_STATUS_DESC_QOS(pdesc);//(( le32_to_cpu( pdesc->rxdw0 ) >> 23) & 0x1);// Qos data, wireless lan header length is 26
	pattrib->shift_sz = (uint8_t)GET_RX_STATUS_DESC_SHIFT(pdesc);//((le32_to_cpu(pdesc->rxdw0) >> 24) & 0x3);
	pattrib->physt = (uint8_t)GET_RX_STATUS_DESC_PHY_STATUS(pdesc);//((le32_to_cpu(pdesc->rxdw0) >> 26) & 0x1);
	pattrib->bdecrypted = !GET_RX_STATUS_DESC_SWDEC(pdesc);//(le32_to_cpu(pdesc->rxdw0) & BIT(27))? 0:1;

	//Offset 4
	pattrib->priority = (uint8_t)GET_RX_STATUS_DESC_TID(pdesc);//((le32_to_cpu(pdesc->rxdw1) >> 8) & 0xf);
	pattrib->mdata = (uint8_t)GET_RX_STATUS_DESC_MORE_DATA(pdesc);//((le32_to_cpu(pdesc->rxdw1) >> 26) & 0x1);
	pattrib->mfrag = (uint8_t)GET_RX_STATUS_DESC_MORE_FRAG(pdesc);//((le32_to_cpu(pdesc->rxdw1) >> 27) & 0x1);//more fragment bit

	//Offset 8
	pattrib->seq_num = (u16)GET_RX_STATUS_DESC_SEQ(pdesc);//(le32_to_cpu(pdesc->rxdw2) & 0x00000fff);
	pattrib->frag_num = (uint8_t)GET_RX_STATUS_DESC_FRAG(pdesc);//((le32_to_cpu(pdesc->rxdw2) >> 12) & 0xf);//fragmentation number

	if (GET_RX_STATUS_DESC_RPT_SEL(pdesc))
		pattrib->pkt_rpt_type = C2H_PACKET;
	else
		pattrib->pkt_rpt_type = NORMAL_RX;

	//Offset 12
	pattrib->data_rate=(uint8_t)GET_RX_STATUS_DESC_RX_RATE(pdesc);//((le32_to_cpu(pdesc->rxdw3))&0x7f);

	//Offset 16
	//Offset 20

}


/*
 * Notice:
 *	Before calling this function,
 *	precvframe->u.hdr.rx_data should be ready!
 */
void rtl8812_query_rx_phy_status(
	struct recv_frame	*precvframe,
	uint8_t 				*pphy_status)
{
	struct rtl_priv *			rtlpriv = precvframe->rtlpriv;
	struct rx_pkt_attrib	*pattrib = &precvframe->attrib;
	 struct _rtw_hal		*pHalData = GET_HAL_DATA(rtlpriv);
	struct _ODM_Phy_Status_Info_ *pPHYInfo  = (struct _ODM_Phy_Status_Info_ *) (&pattrib->phy_info);
	uint8_t					*wlanhdr;
	struct _ODM_Per_Pkt_Info_ pkt_info;
	uint8_t *sa;
	struct sta_priv *pstapriv;
	struct sta_info *psta;
	//_irqL		irqL;

	pkt_info.bPacketMatchBSSID =false;
	pkt_info.bPacketToSelf = false;
	pkt_info.bPacketBeacon = false;

	wlanhdr = get_recvframe_data(precvframe);

	pkt_info.bPacketMatchBSSID = ((!IsFrameTypeCtrl(wlanhdr)) &&
		!pattrib->icv_err && !pattrib->crc_err &&
		memcmp(get_hdr_bssid(wlanhdr), get_bssid(&rtlpriv->mlmepriv), ETH_ALEN) == 0);

	pkt_info.bPacketToSelf = pkt_info.bPacketMatchBSSID &&
				(memcmp(get_da(wlanhdr), rtlpriv->mac80211.mac_addr, ETH_ALEN) == 0);

	pkt_info.bPacketBeacon = pkt_info.bPacketMatchBSSID && (GetFrameSubType(wlanhdr) == WIFI_BEACON);

	if(pkt_info.bPacketBeacon){
		if(check_fwstate(&rtlpriv->mlmepriv, WIFI_STATION_STATE) == true){
			sa = rtlpriv->mlmepriv.cur_network.network.MacAddress;
			#if 0
			{
				DBG_8192C("==> rx beacon from AP[%02x:%02x:%02x:%02x:%02x:%02x]\n",
					sa[0],sa[1],sa[2],sa[3],sa[4],sa[5]);
			}
			#endif
		}
		//to do Ad-hoc
	}
	else{
		sa = get_sa(wlanhdr);
	}

	pstapriv = &rtlpriv->stapriv;
	pkt_info.StationID = 0xFF;
	psta = rtw_get_stainfo(pstapriv, sa);
	if (psta)
	{
		pkt_info.StationID = psta->mac_id;
		//DBG_8192C("%s ==> StationID(%d)\n",__FUNCTION__,pkt_info.StationID);
	}
	pkt_info.DataRate = pattrib->data_rate;
	//rtl8188e_query_rx_phy_status(precvframe, pphy_status);

	//spin_lock_bh(&pHalData->odm_stainfo_lock, &irqL);
	query_rxphystatus(&pHalData->odmpriv,pPHYInfo,pphy_status,&(pkt_info));
	//spin_unlock_bh(&pHalData->odm_stainfo_lock, &irqL);

	precvframe->psta = NULL;
	if (pkt_info.bPacketMatchBSSID &&
		(check_fwstate(&rtlpriv->mlmepriv, WIFI_AP_STATE) == true))
	{
		if (psta)
		{
			precvframe->psta = psta;
			process_phy_info(rtlpriv, precvframe);

		}
	}
	else if (pkt_info.bPacketToSelf || pkt_info.bPacketBeacon)
	{
		if (check_fwstate(&rtlpriv->mlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE) == true)
		{
			if (psta)
			{
				precvframe->psta = psta;
			}
		}
		process_phy_info(rtlpriv, precvframe);
	}
}



static void _ConfigChipOutEP_8812(struct rtl_priv *rtlpriv, uint8_t NumOutPipe)
{
	struct rtl_usb  *rtlusb = rtl_usbdev(rtlpriv);

	rtlusb->out_queue_sel = 0;

	switch (NumOutPipe) {
	case 	4:
		rtlusb->out_queue_sel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		break;
	case 	3:
		rtlusb->out_queue_sel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		break;
	case 	2:
		rtlusb->out_queue_sel = TX_SELE_HQ | TX_SELE_NQ;
		break;
	case 	1:
		rtlusb->out_queue_sel = TX_SELE_HQ;
		break;
	default:
		break;

	}
	RT_TRACE(rtlpriv, COMP_INIT, DBG_EMERG, "Tx queue select :0x%02x..\n",
		 rtlusb->out_queue_sel);
}


/* endpoint mapping */

static void _OneOutEpMapping(struct rtl_ep_map *ep_map)
{
	/* typical setting */

	ep_map->ep_mapping[RTL_TXQ_BE]	= 2;
	ep_map->ep_mapping[RTL_TXQ_BK]	= 2;
	ep_map->ep_mapping[RTL_TXQ_VI]	= 2;
	ep_map->ep_mapping[RTL_TXQ_VO] = 2;
	ep_map->ep_mapping[RTL_TXQ_MGT] = 2;
	ep_map->ep_mapping[RTL_TXQ_BCN] = 2;
	ep_map->ep_mapping[RTL_TXQ_HI]	= 2;
}

static void _TwoOutEpMapping(struct rtl_ep_map *ep_map)
{
	/* typical setting */

	ep_map->ep_mapping[RTL_TXQ_BE]	= 3;
	ep_map->ep_mapping[RTL_TXQ_BK]	= 3;
	ep_map->ep_mapping[RTL_TXQ_VI]	= 2;
	ep_map->ep_mapping[RTL_TXQ_VO]	= 2;
	ep_map->ep_mapping[RTL_TXQ_MGT] = 2;
	ep_map->ep_mapping[RTL_TXQ_BCN] = 2;
	ep_map->ep_mapping[RTL_TXQ_HI]	= 2;
}

static void _ThreeOutEpMapping(struct rtl_ep_map *ep_map)
{
	/* typical setting */

	ep_map->ep_mapping[RTL_TXQ_BE]	= 4;
	ep_map->ep_mapping[RTL_TXQ_BK]	= 4;
	ep_map->ep_mapping[RTL_TXQ_VI]	= 3;
	ep_map->ep_mapping[RTL_TXQ_VO]	= 2;
	ep_map->ep_mapping[RTL_TXQ_MGT] = 2;
	ep_map->ep_mapping[RTL_TXQ_BCN] = 2;
	ep_map->ep_mapping[RTL_TXQ_HI]	= 2;
}

static void _FourOutEpMapping(struct rtl_ep_map *ep_map)
{
	/* typical setting */

	ep_map->ep_mapping[RTL_TXQ_BE]	= 8;
	ep_map->ep_mapping[RTL_TXQ_BK]	= 8;
	ep_map->ep_mapping[RTL_TXQ_VI]	= 6;
	ep_map->ep_mapping[RTL_TXQ_VO]	= 5;
	ep_map->ep_mapping[RTL_TXQ_MGT] = 5;
	ep_map->ep_mapping[RTL_TXQ_BCN] = 5;
	ep_map->ep_mapping[RTL_TXQ_HI]	= 5;
}

static bool Hal_MappingOutPipe(struct rtl_priv *rtlpriv, uint8_t NumOutPipe)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	struct rtl_ep_map *ep_map = &(rtlusb->ep_map);
	bool result = true;

	switch(NumOutPipe) {
	case 2:
		_TwoOutEpMapping(ep_map);
		break;
	case 3:
		_ThreeOutEpMapping(ep_map);
		break;
	case 4:
		_FourOutEpMapping(ep_map);
		break;
	case 1:
		_OneOutEpMapping(ep_map);
		break;
	default:
		result = false;
		break;
	}

	return result;

}


int rtl8821au_endpoint_mapping(struct rtl_priv *rtlpriv)
{
	struct rtl_usb	*rtlusb = rtl_usbdev(rtlpriv);
	bool		result		= false;

	_ConfigChipOutEP_8812(rtlpriv, rtlusb->RtNumOutPipes);

	/* Normal chip with one IN and one OUT doesn't have interrupt IN EP. */
	if (1 == rtlusb->RtNumOutPipes) {
		if (1 != rtlusb->RtNumInPipes) {
			return result;
		}
	}

	/*
	 * All config other than above support one Bulk IN and one Interrupt IN.
	 * if (2 != NumInPipe){
	 * 	return result;
	 * }
	 */

	result = Hal_MappingOutPipe(rtlpriv, rtlusb->RtNumOutPipes);

	return result;

}

