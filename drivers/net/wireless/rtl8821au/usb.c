#include <linux/etherdevice.h>

#include <drv_types.h>
#include <usb_ops.h>
#include <rtw_ap.h>
#include <rtl8812a_recv.h>
#include <../rtl8821au/trx.h>
#include <../rtl8821au/fw.h>
#include "debug.h"

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

#undef DBG_8192C
static inline void DBG_8192C(const char *fmt, ...)
{
}

/*
* Increase and check if the continual_urb_error of this @param dvobjprive is larger than MAX_CONTINUAL_URB_ERR
* @return true:
* @return false:
*/

/* ULLI : currently wrong functions for this prototypes, we change this (maybe) later */

static uint32_t _rtl_usb_receive(struct rtl_priv *rtlpriv, uint32_t cnt, uint8_t *rmem);



static inline int rtw_inc_and_chk_continual_urb_error(struct rtl_usb *dvobj)
{
	int ret = false;
	int value;
	if( (value=atomic_inc_return(&dvobj->continual_urb_error)) > MAX_CONTINUAL_URB_ERR) {
		DBG_871X("[dvobj:%p][ERROR] continual_urb_error:%d > %d\n", dvobj, value, MAX_CONTINUAL_URB_ERR);
		ret = true;
	} else {
		//DBG_871X("[dvobj:%p] continual_urb_error:%d\n", dvobj, value);
	}
	return ret;
}

int rtw_resume_process(struct rtl_priv *rtlpriv);

static int usbctrl_vendorreq(struct rtl_priv *rtlpriv, uint8_t request, u16 value, u16 index, void *pdata, u16 len, uint8_t requesttype)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	struct usb_device *udev = rtlusb->udev;
	int _unused;

	unsigned int pipe;
	int status = 0;
	u32 tmp_buflen=0;
	uint8_t reqtype;
	uint8_t *pIo_buf;
	int vendorreq_times = 0;

	uint8_t tmp_buf[MAX_USB_IO_CTL_SIZE];

	/* DBG_871X("%s %s:%d\n",__FUNCTION__, current->comm, current->pid); */

	if((rtlpriv->bSurpriseRemoved) ||(rtlpriv->pwrctrlpriv.pnp_bstop_trx)){
		status = -EPERM;
		goto exit;
	}

	if(len>MAX_VENDOR_REQ_CMD_SIZE){
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "[%s] Buffer len error ,vendor request failed\n", __FUNCTION__ );
		status = -EINVAL;
		goto exit;
	}

#ifdef CONFIG_USB_VENDOR_REQ_MUTEX
	_unused = mutex_lock_interruptible(&rtlusb->usb_vendor_req_mutex);
#endif


	/* Acquire IO memory for vendorreq */
#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_PREALLOC
	pIo_buf = rtlusb->usb_vendor_req_buf;
#else
	tmp_buflen = MAX_USB_IO_CTL_SIZE;

	/*
	 * Added by Albert 2010/02/09
	 * For mstar platform, mstar suggests the address for USB IO should be 16 bytes alignment.
	 * Trying to fix it here.
	 */
	pIo_buf = (tmp_buf==NULL)?NULL:tmp_buf + ALIGNMENT_UNIT -((__kernel_size_t)(tmp_buf) & 0x0f );
#endif

	if ( pIo_buf== NULL) {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "[%s] pIo_buf == NULL \n", __FUNCTION__ );
		status = -ENOMEM;
		goto release_mutex;
	}

	while (++vendorreq_times<= MAX_USBCTRL_VENDORREQ_TIMES) {
		memset(pIo_buf, 0, len);

		if (requesttype == 0x01) {
			pipe = usb_rcvctrlpipe(udev, 0);	/* read_in */
			reqtype =  REALTEK_USB_VENQT_READ;
		} else {
			pipe = usb_sndctrlpipe(udev, 0);	/* write_out */
			reqtype =  REALTEK_USB_VENQT_WRITE;
			memcpy( pIo_buf, pdata, len);
		}

		status = usb_control_msg(udev, pipe, request, reqtype, value, index, pIo_buf, len, RTW_USB_CONTROL_MSG_TIMEOUT);

		if (status == len) {   // Success this control transfer. */
			rtw_reset_continual_urb_error(rtlusb);
			if (requesttype == 0x01) {
				/* For Control read transfer, we have to copy the read data from pIo_buf to pdata. */
				memcpy(pdata, pIo_buf,  len);
			}
		} else {
			/* error cases */
			RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "reg 0x%x, usb %s %u fail, status:%d value=0x%x, vendorreq_times:%d\n"
				, value,(requesttype == 0x01)?"read":"write" , len, status, *(u32*)pdata, vendorreq_times);

			if (status < 0) {
				if(status == (-ESHUTDOWN) || status == -ENODEV) {
					rtlpriv->bSurpriseRemoved = true;
				} else {
					;
				}
			} else {
				/* status != len && status >= 0 */

				if(status > 0) {
					if ( requesttype == 0x01 ) {
						/* For Control read transfer, we have to copy the read data from pIo_buf to pdata. */
						memcpy( pdata, pIo_buf,  len );
					}
				}
			}

			if(rtw_inc_and_chk_continual_urb_error(rtlusb) == true ){
				rtlpriv->bSurpriseRemoved = true;
				break;
			}

		}

		/* firmware download is checksumed, don't retry */
		if( (value >= FW_8821AU_START_ADDRESS ) || status == len )
			break;

	}

	/* release IO memory used by vendorreq */

release_mutex:
#ifdef CONFIG_USB_VENDOR_REQ_MUTEX
	mutex_unlock(&rtlusb->usb_vendor_req_mutex);
#endif
exit:
	return status;

}



static uint8_t usb_read8(struct rtl_priv *rtlpriv, uint32_t addr)
{
	uint8_t request;
	uint8_t requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	uint8_t data = 0;

	request = 0x05;
	requesttype = 0x01;	/* read_in */
	index = 0;		/* n/a */

	wvalue = (u16) (addr&0x0000ffff);
	len = 1;

	usbctrl_vendorreq(rtlpriv, request, wvalue, index, &data, len, requesttype);

	return data;

}

static u16 usb_read16(struct rtl_priv *rtlpriv, uint32_t addr)
{
	uint8_t request;
	uint8_t requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u16 data = 0;

	request = 0x05;
	requesttype = 0x01;	/* read_in */
	index = 0;		/* n/a */

	wvalue = (u16)(addr&0x0000ffff);
	len = 2;

	usbctrl_vendorreq(rtlpriv, request, wvalue, index, &data, len, requesttype);

	return data;

}

static uint32_t usb_read32(struct rtl_priv *rtlpriv, uint32_t addr)
{
	uint8_t request;
	uint8_t requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	uint32_t data = 0;

	request = 0x05;
	requesttype = 0x01;	/* read_in */
	index = 0;		/* n/a */

	wvalue = (u16)(addr&0x0000ffff);
	len = 4;

	usbctrl_vendorreq(rtlpriv, request, wvalue, index, &data, len, requesttype);


	return data;

}

static void usb_write8(struct rtl_priv *rtlpriv, uint32_t addr, uint8_t val)
{
	uint8_t request;
	uint8_t requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	uint8_t data;

	request = 0x05;
	requesttype = 0x00;		/* write_out */
	index = 0;			/* n/a */

	wvalue = (u16)(addr&0x0000ffff);
	len = 1;

	data = val;

	usbctrl_vendorreq(rtlpriv, request, wvalue, index, &data, len, requesttype);
}

static void usb_write16(struct rtl_priv *rtlpriv, uint32_t addr, u16 val)
{
	uint8_t request;
	uint8_t requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	u16 data;

	request = 0x05;
	requesttype = 0x00;	/* write_out */
	index = 0;		/* n/a */

	wvalue = (u16)(addr&0x0000ffff);
	len = 2;

	data = val;

	usbctrl_vendorreq(rtlpriv, request, wvalue, index, &data, len, requesttype);
}

static void usb_write32(struct rtl_priv *rtlpriv, uint32_t addr, uint32_t val)
{
	uint8_t request;
	uint8_t requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	uint32_t data;

	request = 0x05;
	requesttype = 0x00;	/* write_out */
	index = 0;		/* n/a */

	wvalue = (u16)(addr&0x0000ffff);
	len = 4;
	data = val;

	usbctrl_vendorreq(rtlpriv, request, wvalue, index, &data, len, requesttype);
}

static void usb_writeN(struct rtl_priv *rtlpriv, uint32_t addr, void *pdata, u16 length)
{
	uint8_t request;
	uint8_t requesttype;
	u16 wvalue;
	u16 index;
	u16 len;
	uint8_t buf[VENDOR_CMD_MAX_DATA_LEN] = {0};

	request = 0x05;
	requesttype = 0x00;	/* write_out */
	index = 0;		/* n/a */

	wvalue = (u16)(addr&0x0000ffff);
	len = length;
	memcpy(buf, pdata, len);

	usbctrl_vendorreq(rtlpriv, request, wvalue, index, buf, len, requesttype);
}

static void _rtl_usb_io_handler_init(struct device *dev,
				     struct rtl_priv *rtlpriv)
{
	rtlpriv->io.dev = dev;
	mutex_init(&rtlpriv->io.bb_mutex);
	rtlpriv->io.write8_async	= usb_write8;
	rtlpriv->io.write16_async	= usb_write16;
	rtlpriv->io.write32_async	= usb_write32;
	rtlpriv->io.read8_sync		= usb_read8;
	rtlpriv->io.read16_sync		= usb_read16;
	rtlpriv->io.read32_sync		= usb_read32;
	rtlpriv->io.writeN_sync		= usb_writeN;
}

static void _rtl_usb_io_handler_release(struct rtl_priv *rtlpriv)
{
	mutex_destroy(&rtlpriv->io.bb_mutex);
}


static void _rtl_tx_complete(struct urb *purb)
{
	unsigned long flags;
	int i;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)purb->context;
	/* struct xmit_frame *pxmitframe = (struct xmit_frame *)pxmitbuf->priv_data; */
	/* struct rtl_priv			*rtlpriv = pxmitframe->rtlpriv; */
	struct rtl_priv	*rtlpriv = pxmitbuf->rtlpriv;
	struct xmit_priv	*pxmitpriv = &rtlpriv->xmitpriv;
	/* struct pkt_attrib *pattrib = &pxmitframe->attrib; */

	switch(pxmitbuf->flags) {
		case RTL_TXQ_VO:
			break;
		case RTL_TXQ_VI:
			break;
		case RTL_TXQ_BE:
			break;
		case RTL_TXQ_BK:
			break;
		case RTL_TXQ_HI:
#ifdef CONFIG_AP_MODE
			rtw_chk_hi_queue_cmd(rtlpriv);
#endif
			break;
		default:
			break;
	}


        /* rtw_free_xmitframe(pxmitpriv, pxmitframe); */

	if (rtlpriv->bSurpriseRemoved || rtlpriv->bDriverStopped ||rtlpriv->bWritePortCancel) {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "%s(): TX Warning! bDriverStopped(%d) OR bSurpriseRemoved(%d) bWritePortCancel(%d) pxmitbuf->buf_tag(%x) \n",
		__FUNCTION__,rtlpriv->bDriverStopped, rtlpriv->bSurpriseRemoved,rtlpriv->bReadPortCancel,pxmitbuf->buf_tag);

		goto check_completion;
	}


	if (purb->status==0) {

	} else {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD,  "###=> urb_write_port_complete status(%d)\n",purb->status);
		if((purb->status==-EPIPE)||(purb->status==-EPROTO)) {
			/*
			 * usb_clear_halt(pusbdev, purb->pipe);
			 * msleep(10);
			 */
		} else if (purb->status == -EINPROGRESS) {
			goto check_completion;

		} else if (purb->status == -ENOENT) {
			RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "%s: -ENOENT\n", __func__);
			goto check_completion;

		} else if (purb->status == -ECONNRESET) {
			RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD,  "%s: -ECONNRESET\n", __func__);
			goto check_completion;

		} else if (purb->status == -ESHUTDOWN) {
			rtlpriv->bDriverStopped=true;

			goto check_completion;
		} else 	{
			rtlpriv->bSurpriseRemoved=true;
			RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD,  "bSurpriseRemoved=true\n");
			/* rtl8192cu_trigger_gpio_0(rtlpriv); */

			goto check_completion;
		}
	}

check_completion:
	spin_lock_irqsave(&pxmitpriv->lock_sctx, flags);
	rtw_sctx_done_err(&pxmitbuf->sctx,
		purb->status ? RTW_SCTX_DONE_WRITE_PORT_ERR : RTW_SCTX_DONE_SUCCESS);
	spin_unlock_irqrestore(&pxmitpriv->lock_sctx, flags);

	rtw_free_xmitbuf(pxmitpriv, pxmitbuf);

	/* if(rtw_txframes_pending(rtlpriv)) */
	{
		tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
	}
}


/* ULLI : _rtlw* prefix because of rtlwifi namespace issues */

u32 _rtlw_usb_transmit(struct rtl_priv *rtlpriv, u32 queue_idx, u32 cnt, struct xmit_buf *pxmitbuf)
{
	u32 ep_num;

	unsigned long flags;
	unsigned int pipe;
	int status;
	u32 ret = _FAIL, bwritezero = false;
	struct urb *purb = NULL;
	struct rtl_usb	*rtlusb = rtl_usbdev(rtlpriv);
	struct xmit_priv *pxmitpriv = &rtlpriv->xmitpriv;
	struct xmit_frame *pxmitframe = (struct xmit_frame *)pxmitbuf->priv_data;
	struct usb_device *pusbd = rtlusb->udev;
	struct tx_pkt_attrib *pattrib = &pxmitframe->tx_attrib;

	if ((rtlpriv->bDriverStopped) || (rtlpriv->bSurpriseRemoved) ||(rtlpriv->pwrctrlpriv.pnp_bstop_trx)) {
		#ifdef DBG_TX
		DBG_871X(" DBG_TX %s:%d bDriverStopped%d, bSurpriseRemoved:%d, pnp_bstop_trx:%d\n",__FUNCTION__, __LINE__
			,rtlpriv->bDriverStopped, rtlpriv->bSurpriseRemoved, rtlpriv->pwrctrlpriv.pnp_bstop_trx );
		#endif
		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_TX_DENY);
		goto exit;
	}

	spin_lock_irqsave(&pxmitpriv->lock, flags);

	switch(queue_idx) {
		case RTL_TXQ_VO:
			pxmitbuf->flags = RTL_TXQ_VO;
			break;
		case RTL_TXQ_VI:
			pxmitbuf->flags = RTL_TXQ_VI;
			break;
		case RTL_TXQ_BE:
			pxmitbuf->flags = RTL_TXQ_BE;
			break;
		case RTL_TXQ_BK:
			pxmitbuf->flags = RTL_TXQ_BK;
			break;
		case RTL_TXQ_HI:
			pxmitbuf->flags = RTL_TXQ_HI;
			break;
		default:
			pxmitbuf->flags = RTL_TXQ_MGT;
			break;
	}

	spin_unlock_irqrestore(&pxmitpriv->lock, flags);

	purb	= pxmitbuf->pxmit_urb[0];

	/* translate DMA FIFO addr to pipehandle */
	ep_num = rtlusb->ep_map.ep_mapping[queue_idx];
	pipe = usb_sndbulkpipe(pusbd, ep_num);

#ifdef CONFIG_REDUCE_USB_TX_INT
	if ((pxmitpriv->free_xmitbuf_cnt%NR_XMITBUFF == 0) ||
	   (pxmitbuf->buf_tag > XMITBUF_DATA)) {
		purb->transfer_flags  &=  (~URB_NO_INTERRUPT);
	} else {
		purb->transfer_flags  |=  URB_NO_INTERRUPT;
		/* DBG_8192C("URB_NO_INTERRUPT "); */
	}
#endif

	usb_fill_bulk_urb(purb, pusbd, pipe,
       				pxmitframe->buf_addr, 	/* = pxmitbuf->pbuf */
              			cnt,
              			_rtl_tx_complete,
              			pxmitbuf);		/* context is pxmitbuf */
#if 0
	if (bwritezero) {
            purb->transfer_flags |= URB_ZERO_PACKET;
        }
#endif

	status = usb_submit_urb(purb, GFP_ATOMIC);
	if (!status) {
	} else {
		rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_WRITE_PORT_ERR);
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, " _rtlw_usb_transmit, status=%d\n", status);
		switch (status) {
		case -ENODEV:
			rtlpriv->bDriverStopped=true;
			break;
		default:
			break;
		}
		goto exit;
	}

	ret= _SUCCESS;

/*
 *  Commented by Albert 2009/10/13
 *  We add the URB_ZERO_PACKET flag to urb so that the host will send the zero packet automatically.
 */
/*
	if(bwritezero == true)
	{
		usb_bulkout_zero(pintfhdl, addr);
	}
*/

exit:
	if (ret != _SUCCESS)
		rtw_free_xmitbuf(pxmitpriv, pxmitbuf);

	return ret;
}

void usb_write_port_cancel(struct rtl_priv *rtlpriv)
{
	int i, j;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)rtlpriv->xmitpriv.pxmitbuf;

	rtlpriv->bWritePortCancel = true;

	for (i=0; i<NR_XMITBUFF; i++) {
		for (j=0; j<8; j++) {
			if (pxmitbuf->pxmit_urb[j]) {
				usb_kill_urb(pxmitbuf->pxmit_urb[j]);
			}
		}
		pxmitbuf++;
	}

	pxmitbuf = (struct xmit_buf*)rtlpriv->xmitpriv.pxmit_extbuf;
	for (i = 0; i < NR_XMIT_EXTBUFF; i++) {
		for (j=0; j<8; j++) {
			if(pxmitbuf->pxmit_urb[j]) {
				usb_kill_urb(pxmitbuf->pxmit_urb[j]);
			}
		}
		pxmitbuf++;
	}
}

static void _rtl_rx_complete(struct urb *purb)
{
	uint isevt, *pbuf;
	struct recv_buf	*precvbuf = (struct recv_buf *) purb->context;
	struct rtl_priv 		*rtlpriv = (struct rtl_priv *) precvbuf->rtlpriv;
	struct recv_priv	*precvpriv = &rtlpriv->recvpriv;

	/*
	 * spin_lock_irqsave(&precvpriv->lock, &irqL);
	 * precvbuf->irp_pending=false;
	 * precvpriv->rx_pending_cnt --;
	 * spin_unlock_irqrestore(&precvpriv->lock, &irqL);
	 */

	precvpriv->rx_pending_cnt--;

	/*
	 * if(precvpriv->rx_pending_cnt== 0) {
	 * 	RT_TRACE(_module_hci_ops_os_c_,_drv_err_,("usb_read_port_complete: rx_pending_cnt== 0, set allrxreturnevt!\n"));
	 * 	up(&precvpriv->allrxreturnevt);
	 * }
	 */

	if (rtlpriv->bSurpriseRemoved || rtlpriv->bDriverStopped
	|| rtlpriv->bReadPortCancel) {
		precvbuf->reuse = true;
		DBG_8192C("%s() RX Warning! bDriverStopped(%d) OR bSurpriseRemoved(%d) bReadPortCancel(%d)\n",
		__FUNCTION__, rtlpriv->bDriverStopped, rtlpriv->bSurpriseRemoved, rtlpriv->bReadPortCancel);
		goto exit;
	}

	if (purb->status == 0) {
		/* SUCCESS */
		if ((purb->actual_length > MAX_RECVBUF_SZ)
		 || (purb->actual_length < RXDESC_SIZE)) {
			precvbuf->reuse = true;
			_rtl_usb_receive(rtlpriv, 0, (unsigned char *)precvbuf);
			DBG_8192C("%s()-%d: RX Warning!\n", __FUNCTION__, __LINE__);
		} else {
			rtw_reset_continual_urb_error(rtl_usbdev(rtlpriv));

			skb_put(precvbuf->skb, purb->actual_length);
			skb_queue_tail(&precvpriv->rx_skb_queue, precvbuf->skb);

			if (skb_queue_len(&precvpriv->rx_skb_queue) <= 1)
				tasklet_schedule(&precvpriv->recv_tasklet);

			precvbuf->skb = NULL;
			precvbuf->reuse = false;
			_rtl_usb_receive(rtlpriv, 0, (unsigned char *)precvbuf);
		}
	} else {
		DBG_8192C("###=> usb_read_port_complete => urb status(%d)\n", purb->status);

		if (rtw_inc_and_chk_continual_urb_error(rtl_usbdev(rtlpriv)) == true) {
			rtlpriv->bSurpriseRemoved = true;
		}

		switch (purb->status) {
		case -EINVAL:
		case -EPIPE:
		case -ENODEV:
		case -ESHUTDOWN:
			/* rtlpriv->bSurpriseRemoved=true; */
		case -ENOENT:
			rtlpriv->bDriverStopped = true;
			break;
		case -EPROTO:
		case -EILSEQ:
		case -ETIME:
		case -ECOMM:
		case -EOVERFLOW:
			precvbuf->reuse = true;
			_rtl_usb_receive(rtlpriv, 0, (unsigned char *)precvbuf);
			break;
		case -EINPROGRESS:
			DBG_8192C("ERROR: URB IS IN PROGRESS!/n");
			break;
		default:
			break;
		}

	}

exit:
	;
}

void rtw_os_read_port(struct rtl_priv *rtlpriv, struct recv_buf *precvbuf)
{
	struct recv_priv *precvpriv = &rtlpriv->recvpriv;

	precvbuf->ref_cnt--;

	/* free skb in recv_buf */
	dev_kfree_skb_any(precvbuf->skb);

	precvbuf->skb = NULL;
	precvbuf->reuse = false;

	if (precvbuf->irp_pending == false) {
		_rtl_usb_receive(rtlpriv, 0, (unsigned char *)precvbuf);
	}



}

static uint32_t _rtl_usb_receive (struct rtl_priv *rtlpriv, uint32_t cnt, uint8_t *rmem)
{
	int err;
	unsigned int pipe;
	__kernel_size_t tmpaddr = 0;
	__kernel_size_t alignment = 0;
	uint32_t ret = _SUCCESS;
	struct urb *purb = NULL;
	struct recv_buf	*precvbuf = (struct recv_buf *) rmem;
	struct rtl_usb	*rtlusb = rtl_usbdev(rtlpriv);
	struct recv_priv	*precvpriv = &rtlpriv->recvpriv;
	struct usb_device	*pusbd = rtlusb->udev;
	uint32_t addr = RECV_BULK_IN_ADDR;

	if (rtlpriv->bDriverStopped || rtlpriv->bSurpriseRemoved
	 || rtlpriv->pwrctrlpriv.pnp_bstop_trx) {
		return _FAIL;
	}

	if ((precvbuf->reuse == false) || (precvbuf->skb == NULL)) {
		precvbuf->skb = skb_dequeue(&precvpriv->free_recv_skb_queue);
		if (precvbuf->skb != NULL) {
			precvbuf->reuse = true;
		}
	}

	if (precvbuf != NULL) {
		precvbuf->len = 0;
		precvbuf->ref_cnt = 0;

		/* re-assign for linux based on skb */
		if ((precvbuf->reuse == false) || (precvbuf->skb == NULL)) {
			/* precvbuf->pskb = alloc_skb(MAX_RECVBUF_SZ, GFP_ATOMIC);//don't use this after v2.6.25 */
			precvbuf->skb = netdev_alloc_skb(rtlpriv->ndev, MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ);
			if (precvbuf->skb == NULL) {
				DBG_8192C("#### usb_read_port() alloc_skb fail!#####\n");
				return _FAIL;
			}

			tmpaddr = (__kernel_size_t) precvbuf->skb->data;
			alignment = tmpaddr & (RECVBUFF_ALIGN_SZ-1);
			skb_reserve(precvbuf->skb, (RECVBUFF_ALIGN_SZ - alignment));

		} else {
			/* reuse skb */

			precvbuf->reuse = false;
		}

		/*
		 * spin_lock_irqsave(&precvpriv->lock, &irqL);
		 * precvpriv->rx_pending_cnt++;
		 * precvbuf->irp_pending = true;
		 * spin_unlock_irqrestore(&precvpriv->lock, &irqL);
		 */
		precvpriv->rx_pending_cnt++;

		purb = precvbuf->purb;

		/* translate DMA FIFO addr to pipehandle */
		pipe = usb_rcvbulkpipe(pusbd, rtlusb->RtInPipe[0]);

		usb_fill_bulk_urb(purb, pusbd, pipe,
						precvbuf->skb->data,
						MAX_RECVBUF_SZ,
						_rtl_rx_complete,
						precvbuf);	/* context is precvbuf */

		err = usb_submit_urb(purb, GFP_ATOMIC);
		if ((err) && (err != (-EPERM))) {
			DBG_8192C("cannot submit rx in-token(err = 0x%08x),urb_status = %d\n", err, purb->status);
			ret = _FAIL;
		}
	} else {
		ret = _FAIL;
	}

	return ret;
}


static inline int RT_usb_endpoint_dir_in(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN);
}

static inline int RT_usb_endpoint_dir_out(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT);
}

static inline int RT_usb_endpoint_xfer_int(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT);
}

static inline int RT_usb_endpoint_xfer_bulk(const struct usb_endpoint_descriptor *epd)
{
 	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK);
}

static inline int RT_usb_endpoint_is_bulk_in(const struct usb_endpoint_descriptor *epd)
{
	return (RT_usb_endpoint_xfer_bulk(epd) && RT_usb_endpoint_dir_in(epd));
}

static inline int RT_usb_endpoint_is_bulk_out(const struct usb_endpoint_descriptor *epd)
{
	return (RT_usb_endpoint_xfer_bulk(epd) && RT_usb_endpoint_dir_out(epd));
}

static inline int RT_usb_endpoint_num(const struct usb_endpoint_descriptor *epd)
{
	return epd->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
}

static uint8_t rtw_init_intf_priv(struct rtl_usb *dvobj)
{
	uint8_t rst = _SUCCESS;

#ifdef CONFIG_USB_VENDOR_REQ_MUTEX
	mutex_init(&dvobj->usb_vendor_req_mutex);
#endif


#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_PREALLOC
	dvobj->usb_alloc_vendor_req_buf = rtw_zmalloc(MAX_USB_IO_CTL_SIZE);
	if (dvobj->usb_alloc_vendor_req_buf == NULL) {
		DBG_871X("alloc usb_vendor_req_buf failed... /n");
		rst = _FAIL;
		goto exit;
	}
	dvobj->usb_vendor_req_buf  =
		(uint8_t *)N_BYTE_ALIGMENT((__kernel_size_t)(dvobj->usb_alloc_vendor_req_buf ), ALIGNMENT_UNIT);
exit:
#endif

	return rst;

}

static uint8_t rtw_deinit_intf_priv(struct rtl_usb *dvobj)
{
	uint8_t rst = _SUCCESS;

#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_PREALLOC
	if(dvobj->usb_vendor_req_buf)
		rtw_mfree(dvobj->usb_alloc_vendor_req_buf);
#endif

#ifdef CONFIG_USB_VENDOR_REQ_MUTEX
	mutex_destroy(&dvobj->usb_vendor_req_mutex);
#endif

	return rst;
}

static struct rtl_usb *usb_dvobj_init(struct usb_interface *usb_intf, struct rtl_usb *rtlusb)
{
	int	i;
	uint8_t	val8;
	int	status = _FAIL;
	struct usb_device_descriptor 	*pdev_desc;
	struct usb_host_config		*phost_conf;
	struct usb_config_descriptor	*pconf_desc;
	struct usb_host_interface	*phost_iface;
	struct usb_interface_descriptor	*piface_desc;
	struct usb_host_endpoint	*phost_endp;
	struct usb_endpoint_descriptor	*pendp_desc;
	struct usb_device			*pusbd;

	mutex_init(&rtlusb->hw_init_mutex);
	mutex_init(&rtlusb->h2c_fwcmd_mutex);
	mutex_init(&rtlusb->setch_mutex);
	mutex_init(&rtlusb->setbw_mutex);

	spin_lock_init(&rtlusb->lock);

	rtlusb->macid[1] = true; 	/* macid=1 for bc/mc stainfo */


	rtlusb->intf = usb_intf ;
	pusbd = rtlusb->udev = interface_to_usbdev(usb_intf);
	usb_set_intfdata(usb_intf, rtlusb);

	rtlusb->RtNumInPipes = 0;
	rtlusb->RtNumOutPipes = 0;

	/*
	 * rtlpriv->EepromAddressSize = 6;
	 * rtlusb->nr_endpoint = 6;
	 */

	pdev_desc = &pusbd->descriptor;


	phost_conf = pusbd->actconfig;
	pconf_desc = &phost_conf->desc;


	/*
	 * DBG_871X("\n****** num of altsetting = (%d) ******\n", pusb_interface->num_altsetting);
	 */


	phost_iface = &usb_intf->altsetting[0];
	piface_desc = &phost_iface->desc;


	rtlusb->nr_endpoint = piface_desc->bNumEndpoints;

	/* DBG_871X("\ndump usb_endpoint_descriptor:\n"); */

	for (i = 0; i < rtlusb->nr_endpoint; i++) {
		phost_endp = phost_iface->endpoint + i;
		if (phost_endp) 		{
			pendp_desc = &phost_endp->desc;

			DBG_871X("\nusb_endpoint_descriptor(%d):\n", i);
			DBG_871X("bLength=%x\n",pendp_desc->bLength);
			DBG_871X("bDescriptorType=%x\n",pendp_desc->bDescriptorType);
			DBG_871X("bEndpointAddress=%x\n",pendp_desc->bEndpointAddress);
			/* DBG_871X("bmAttributes=%x\n",pendp_desc->bmAttributes); */
			DBG_871X("wMaxPacketSize=%d\n",le16_to_cpu(pendp_desc->wMaxPacketSize));
			DBG_871X("bInterval=%x\n",pendp_desc->bInterval);
			/* DBG_871X("bRefresh=%x\n",pendp_desc->bRefresh); */
			/* DBG_871X("bSynchAddress=%x\n",pendp_desc->bSynchAddress); */

			if (RT_usb_endpoint_is_bulk_in(pendp_desc)) {
				DBG_871X("RT_usb_endpoint_is_bulk_in = %x\n", RT_usb_endpoint_num(pendp_desc));
				rtlusb->RtInPipe[rtlusb->RtNumInPipes] = RT_usb_endpoint_num(pendp_desc);
				rtlusb->RtNumInPipes++;
			} else if (RT_usb_endpoint_is_bulk_out(pendp_desc)) {
				DBG_871X("RT_usb_endpoint_is_bulk_out = %x\n", RT_usb_endpoint_num(pendp_desc));
				rtlusb->RtNumOutPipes++;
			}
			rtlusb->ep_num[i] = RT_usb_endpoint_num(pendp_desc);
		}
	}

	DBG_871X("nr_endpoint=%d, in_num=%d, out_num=%d\n\n", rtlusb->nr_endpoint, rtlusb->RtNumInPipes, rtlusb->RtNumOutPipes);

	switch(pusbd->speed) {
	case USB_SPEED_LOW:
		DBG_871X("USB_SPEED_LOW\n");
		rtlusb->usb_speed = RTW_USB_SPEED_1_1;
		break;
	case USB_SPEED_FULL:
		DBG_871X("USB_SPEED_FULL\n");
		rtlusb->usb_speed = RTW_USB_SPEED_1_1;
		break;
	case USB_SPEED_HIGH:
		DBG_871X("USB_SPEED_HIGH\n");
		rtlusb->usb_speed = RTW_USB_SPEED_2;
		break;
	case USB_SPEED_SUPER:
		DBG_871X("USB_SPEED_SUPER\n");
		rtlusb->usb_speed = RTW_USB_SPEED_3;
		break;
	default:
		DBG_871X("USB_SPEED_UNKNOWN(%x)\n",pusbd->speed);
		rtlusb->usb_speed = RTW_USB_SPEED_UNKNOWN;
		break;
	}

	if (rtlusb->usb_speed == RTW_USB_SPEED_UNKNOWN) {
		DBG_871X("UNKNOWN USB SPEED MODE, ERROR !!!\n");
		goto free_dvobj;
	}

	if (rtw_init_intf_priv(rtlusb) == _FAIL) {
		goto free_dvobj;
	}

	/* .3 misc */
	rtw_reset_continual_urb_error(rtlusb);

	usb_get_dev(pusbd);

	status = _SUCCESS;

free_dvobj:
	if (status != _SUCCESS && rtlusb) {
		usb_set_intfdata(usb_intf, NULL);
		mutex_destroy(&rtlusb->hw_init_mutex);
		mutex_destroy(&rtlusb->h2c_fwcmd_mutex);
		mutex_destroy(&rtlusb->setch_mutex);
		mutex_destroy(&rtlusb->setbw_mutex);
		rtlusb = NULL;
	}
exit:
	return rtlusb;
}

void usb_dvobj_deinit(struct usb_interface *usb_intf)
{
	struct rtl_usb *rtlusb = usb_get_intfdata(usb_intf);

	usb_set_intfdata(usb_intf, NULL);
	if (rtlusb) {
		if (interface_to_usbdev(usb_intf)->state != USB_STATE_NOTATTACHED) {
			/*
			 * If we didn't unplug usb dongle and remove/insert modlue, driver fails on sitesurvey for the first time when device is up .
			 * Reset usb port for sitesurvey fail issue. 2009.8.13, by Thomas
			 */
			DBG_871X("usb attached..., try to reset usb device\n");
			usb_reset_device(interface_to_usbdev(usb_intf));
		}
		rtw_deinit_intf_priv(rtlusb);
		mutex_destroy(&rtlusb->hw_init_mutex);
		mutex_destroy(&rtlusb->h2c_fwcmd_mutex);
		mutex_destroy(&rtlusb->setch_mutex);
		mutex_destroy(&rtlusb->setbw_mutex);
	}

	/* DBG_871X("%s %d\n", __func__, atomic_read(&usb_intf->dev.kobj.kref.refcount)); */
	usb_put_dev(interface_to_usbdev(usb_intf));


}



void usb_intf_start(struct rtl_priv *rtlpriv)
{
	uint8_t i;
	struct recv_buf *precvbuf;
	uint	status;
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	struct recv_priv *precvpriv = &(rtlpriv->recvpriv);

	status = _SUCCESS;

	/* issue Rx irp to receive data */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	for (i = 0; i < NR_RECVBUFF; i++) {
		if (_rtl_usb_receive(rtlpriv, 0, (unsigned char *) precvbuf) == false) {
			status = _FAIL;
			goto exit;
		}

		precvbuf++;
		precvpriv->free_recv_buf_queue_cnt--;
	}

exit:
	;
}

void usb_intf_stop(struct rtl_priv *rtlpriv)
{
	/* disabel_hw_interrupt */
	if (rtlpriv->bSurpriseRemoved == false) {
		/* device still exists, so driver can do i/o operation */
		/* TODO: */
	}

	/* cancel in irp */
	rtlpriv->cfg->ops->inirp_deinit(rtlpriv);

	/* cancel out irp */
	usb_write_port_cancel(rtlpriv);

	/* todo:cancel other irps */

}

static void rtw_dev_unload(struct rtl_priv *rtlpriv)
{
	struct net_device *ndev= (struct net_device*)rtlpriv->ndev;
	uint8_t val8;

	if (rtlpriv->initialized == true) {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "===> rtw_dev_unload\n");

		rtlpriv->bDriverStopped = true;
		/* s3. */
		usb_intf_stop(rtlpriv);

		/* s4. */
		if(!rtlpriv->pwrctrlpriv.bInternalAutoSuspend )
		rtw_stop_drv_threads(rtlpriv);


		/* s5. */
		if(rtlpriv->bSurpriseRemoved == false) {
			rtw_hal_deinit(rtlpriv);
			rtlpriv->bSurpriseRemoved = true;
		}

		rtlpriv->initialized = false;
	} else {
		;
	}

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "<=== rtw_dev_unload\n");
}

static void rtw_cancel_all_timer(struct rtl_priv *rtlpriv)
{
	del_timer_sync_ex(&rtlpriv->mlmepriv.assoc_timer);

	/*
	 * del_timer_sync_ex(&rtlpriv->securitypriv.tkip_timer);
	 * RT_TRACE(_module_os_intfs_c_,_drv_info_,("rtw_cancel_all_timer:cancel tkip_timer! \n"));
	 */

	del_timer_sync_ex(&rtlpriv->mlmepriv.scan_to_timer);
	del_timer_sync_ex(&rtlpriv->mlmepriv.dynamic_chk_timer);

	/* cancel sw led timer */
	rtlpriv->cfg->ops->deinit_sw_leds(rtlpriv);

	del_timer_sync_ex(&rtlpriv->pwrctrlpriv.pwr_state_check_timer);


#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	del_timer_sync_ex(&rtlpriv->recvpriv.signal_stat_timer);
#endif
}

static int rtl8821au_suspend(struct usb_interface *pusb_intf, pm_message_t message)
{
	struct rtl_usb *rtlusb = usb_get_intfdata(pusb_intf);
	struct rtl_priv *rtlpriv = rtlusb->rtlpriv;
	struct net_device *ndev = rtlpriv->ndev;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	struct usb_device *usb_dev = interface_to_usbdev(pusb_intf);

	int ret = 0;
	u32 start_time = jiffies;

	DBG_871X("==> %s (%s:%d)\n",__FUNCTION__, current->comm, current->pid);

	if((!rtlpriv->initialized) || (rtlpriv->bDriverStopped)||(rtlpriv->bSurpriseRemoved)) {
		DBG_871X("rtlpriv->initialized=%d bDriverStopped=%d bSurpriseRemoved = %d\n",
			rtlpriv->initialized, rtlpriv->bDriverStopped,rtlpriv->bSurpriseRemoved);
		goto exit;
	}

	if(pwrpriv->bInternalAutoSuspend )
	{
	}
	pwrpriv->bInSuspend = true;
	rtw_cancel_all_timer(rtlpriv);
	LeaveAllPowerSaveMode(rtlpriv);

	down(&pwrpriv->lock);
	/*
	 * rtlpriv->net_closed = true;
	 * s1.
	 */
	if (ndev) {
		netif_carrier_off(ndev);
		netif_tx_stop_all_queues(ndev);
	}

	/* s2. */
	rtw_disassoc_cmd(rtlpriv, 0, false);

	/* s2-2.  indicate disconnect to os */
	rtw_indicate_disconnect(rtlpriv);
	/* s2-3. */
	rtw_free_assoc_resources(rtlpriv, 1);
#ifdef CONFIG_AUTOSUSPEND
	if(!pwrpriv->bInternalAutoSuspend )
#endif
	/* s2-4. */
	rtw_free_network_queue(rtlpriv, true);

	rtw_dev_unload(rtlpriv);
#ifdef CONFIG_AUTOSUSPEND
	pwrpriv->rf_pwrstate = ERFOFF;
	pwrpriv->bips_processing = false;
#endif
	up(&pwrpriv->lock);

	if (check_fwstate(pmlmepriv, _FW_UNDER_SURVEY))
		rtw_indicate_scan_done(rtlpriv, 1);

	if (check_fwstate(pmlmepriv, _FW_UNDER_LINKING))
		rtw_indicate_disconnect(rtlpriv);

exit:
	DBG_871X("<===  %s return %d.............. in %dms\n", __FUNCTION__
		, ret, rtw_get_passing_time_ms(start_time));

	return ret;
}

static int rtl8821au_resume(struct usb_interface *pusb_intf)
{
	struct rtl_usb *rtlusb = usb_get_intfdata(pusb_intf);
	struct rtl_priv *rtlpriv = rtlusb->rtlpriv;

	return rtw_resume_process(rtlpriv);
}


int _netdev_open(struct net_device *ndev)
{
	uint status;
	struct rtl_priv *rtlpriv =  rtl_priv(ndev);
	struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;

	DBG_871X("+871x_drv - drv_open, bup=%d\n", rtlpriv->initialized);

	if (pwrctrlpriv->ps_flag == true) {
		rtlpriv->net_closed = false;
		goto netdev_open_normal_process;
	}

	if (rtlpriv->initialized == false) {
		rtlpriv->bDriverStopped = false;
		rtlpriv->bSurpriseRemoved = false;
		rtlpriv->bCardDisableWOHSM = false;

		status = rtw_hal_init(rtlpriv);
		if (status == _FAIL) {
			goto netdev_open_error;
		}

		DBG_871X("MAC Address = "MAC_FMT"\n", MAC_ARG(ndev->dev_addr));

		status = rtw_start_drv_threads(rtlpriv);
		if (status == _FAIL) {
			DBG_871X("Initialize driver software resource Failed!\n");
			goto netdev_open_error;
		}

		if (init_hw_mlme_ext(rtlpriv) == _FAIL) {
			DBG_871X("can't init mlme_ext_priv\n");
			goto netdev_open_error;
		}

		usb_intf_start(rtlpriv);

		rtlpriv->initialized = true;
	}
	rtlpriv->cfg->ops->led_control(rtlpriv, LED_CTL_POWER_ON);
	rtlpriv->net_closed = false;

	_set_timer(&rtlpriv->mlmepriv.dynamic_chk_timer, 2000);

	rtlpriv->pwrctrlpriv.bips_processing = false;
	rtw_set_pwr_state_check_timer(&rtlpriv->pwrctrlpriv);

	/*
	 * netif_carrier_on(ndev);//call this func when rtw_joinbss_event_callback return success
	 */
	if (!rtw_netif_queue_stopped(ndev))
		netif_tx_start_all_queues(ndev);
	else
		netif_tx_wake_all_queues(ndev);

netdev_open_normal_process:

	DBG_871X("-871x_drv - drv_open, initialized=%d\n", rtlpriv->initialized);

	return 0;

netdev_open_error:

	rtlpriv->initialized = false;

	netif_carrier_off(ndev);
	netif_tx_stop_all_queues(ndev);

	DBG_871X("-871x_drv - drv_open fail, initialized=%d\n", rtlpriv->initialized);

	return (-1);

}

static int netdev_open(struct net_device *ndev)
{
	int ret;
	int _unused;

	struct rtl_priv *rtlpriv =  rtl_priv(ndev);

	/* ULLI: orignal driver doesn't use the return value */
	_unused = mutex_lock_interruptible(&(rtl_usbdev(rtlpriv)->hw_init_mutex));
	ret = _netdev_open(ndev);
	mutex_unlock(&(rtl_usbdev(rtlpriv)->hw_init_mutex));

	return ret;
}

int  ips_netdrv_open(struct rtl_priv *rtlpriv)
{
	int status = _SUCCESS;
	rtlpriv->net_closed = false;

	DBG_871X("===> %s.........\n", __FUNCTION__);

	rtlpriv->bDriverStopped = false;
	rtlpriv->bSurpriseRemoved = false;
	rtlpriv->bCardDisableWOHSM = false;
	/* rtlpriv->bup = true; */

	status = rtw_hal_init(rtlpriv);
	if (status == _FAIL) {
		goto netdev_open_error;
	}

	usb_intf_start(rtlpriv);

	rtw_set_pwr_state_check_timer(&rtlpriv->pwrctrlpriv);
	_set_timer(&rtlpriv->mlmepriv.dynamic_chk_timer, 5000);

	 return _SUCCESS;

netdev_open_error:
	/* rtlpriv->bup = false; */
	DBG_871X("-ips_netdrv_open - drv_open failure, initialized=%d\n", rtlpriv->initialized);

	return _FAIL;
}

static uint8_t rtw_reset_drv_sw(struct rtl_priv *rtlpriv)
{
	uint8_t	ret8 = _SUCCESS;
	struct mlme_priv *pmlmepriv = &rtlpriv->mlmepriv;
	struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;

	/* hal_priv */
	rtlpriv->cfg->ops->init_default_value(rtlpriv);
	rtlpriv->bReadPortCancel = false;
	rtlpriv->bWritePortCancel = false;
	pmlmepriv->scan_interval = SCAN_INTERVAL;	/* 30*2 sec = 60sec */

	rtlpriv->xmitpriv.tx_pkts = 0;
	rtlpriv->recvpriv.rx_pkts = 0;

	pmlmepriv->LinkDetectInfo.bBusyTraffic = false;

	_clr_fwstate_(pmlmepriv, _FW_UNDER_SURVEY | _FW_UNDER_LINKING);

#ifdef CONFIG_AUTOSUSPEND
#endif

	pwrctrlpriv->pwr_state_check_cnts = 0;

	/* mlmeextpriv */
	rtlpriv->mlmeextpriv.sitesurvey_res.state = SCAN_DISABLE;

#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	rtw_set_signal_stat_timer(&rtlpriv->recvpriv);
#endif

	return ret8;
}

int rtw_ips_pwr_up(struct rtl_priv *rtlpriv)
{
	int result;
	u32 start_time = jiffies;

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "===>  rtw_ips_pwr_up..............\n");
	rtw_reset_drv_sw(rtlpriv);

	result = ips_netdrv_open(rtlpriv);

	rtlpriv->cfg->ops->led_control(rtlpriv, LED_CTL_NO_LINK);

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "<===  rtw_ips_pwr_up.............. in %dms\n", rtw_get_passing_time_ms(start_time));
	return result;

}

void rtw_ips_pwr_down(struct rtl_priv *rtlpriv)
{
	u32 start_time = jiffies;

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "===> rtw_ips_pwr_down...................\n");

	rtlpriv->bCardDisableWOHSM = true;
	rtlpriv->net_closed = true;

	rtw_ips_dev_unload(rtlpriv);
	rtlpriv->bCardDisableWOHSM = false;
	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "<=== rtw_ips_pwr_down..................... in %dms\n", rtw_get_passing_time_ms(start_time));
}
void rtw_ips_dev_unload(struct rtl_priv *rtlpriv)
{
	struct net_device *ndev = (struct net_device *) rtlpriv->ndev;
	struct xmit_priv	*pxmitpriv = &(rtlpriv->xmitpriv);

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "====> %s...\n", __FUNCTION__);

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_FIFO_CLEARN_UP, 0);

	usb_intf_stop(rtlpriv);

	/* s5. */
	if (rtlpriv->bSurpriseRemoved == false) {
		rtw_hal_deinit(rtlpriv);
	}

}

static int netdev_close(struct net_device *ndev)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	if (rtlpriv->pwrctrlpriv.bInternalAutoSuspend == true) {
		/*rtw_pwr_wakeup(rtlpriv); */
		if (rtlpriv->pwrctrlpriv.rf_pwrstate == ERFOFF)
			rtlpriv->pwrctrlpriv.ps_flag = true;
	}
	rtlpriv->net_closed = true;

/*	if (!rtlpriv->hw_init_completed)
	{
		DBG_871X("(1)871x_drv - drv_close, bup=%d, hw_init_completed=%d\n", rtlpriv->bup, rtlpriv->hw_init_completed);

		rtlpriv->bDriverStopped = true;

		rtw_dev_unload(rtlpriv);
	}
	else*/
	if (rtlpriv->pwrctrlpriv.rf_pwrstate == ERFON) {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "(2)871x_drv - drv_close, initialized=%d, hw_init_completed=%d\n", rtlpriv->initialized, rtlpriv->hw_init_completed);

		/* s1. */
		if (ndev) {
			if (!rtw_netif_queue_stopped(ndev))
				netif_tx_stop_all_queues(ndev);
		}

		/* s2. */
		LeaveAllPowerSaveMode(rtlpriv);
		rtw_disassoc_cmd(rtlpriv, 500, false);
		/* s2-2.  indicate disconnect to os */
		rtw_indicate_disconnect(rtlpriv);
		/* s2-3. */
		rtw_free_assoc_resources(rtlpriv, 1);
		/* s2-4. */
		rtw_free_network_queue(rtlpriv, true);
		/* Close LED */
	}
	rtlpriv->cfg->ops->led_control(rtlpriv, LED_CTL_POWER_OFF);
	rtlpriv->cfg->ops->deinit_sw_vars(rtlpriv);

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "-871x_drv - drv_close, initialized=%d\n", rtlpriv->initialized);

	return 0;

}

void rtw_ndev_destructor(struct net_device *ndev)
{
	DBG_871X(FUNC_NDEV_FMT"\n", FUNC_NDEV_ARG(ndev));

	free_netdev(ndev);
}


static int pm_netdev_open(struct net_device *ndev, uint8_t bnormal)
{
	int status;

	if (true == bnormal)
		status = netdev_open(ndev);
	else
		status =  (_SUCCESS == ips_netdrv_open(rtl_priv(ndev)))?(0):(-1);

	return status;
}

int rtw_resume_process(struct rtl_priv *rtlpriv)
{
	struct net_device *ndev;
	struct pwrctrl_priv *pwrpriv;
	int ret = -1;
	u32 start_time = jiffies;

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "==> %s (%s:%d)\n",__FUNCTION__, current->comm, current->pid);

	if(rtlpriv) {
		ndev= rtlpriv->ndev;
		pwrpriv = &rtlpriv->pwrctrlpriv;
	} else
		goto exit;

	down(&pwrpriv->lock);
	rtw_reset_drv_sw(rtlpriv);

	if(pm_netdev_open(ndev,true) != 0){
		up(&pwrpriv->lock);
		goto exit;
	}

	netif_device_attach(ndev);
	netif_carrier_on(ndev);

#ifdef CONFIG_AUTOSUSPEND
	if (pwrpriv->bInternalAutoSuspend) {
		pwrpriv->bInternalAutoSuspend = false;
		pwrpriv->brfoffbyhw = false;
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "enc_algorithm(%x),wepkeymask(%x)\n",
			rtlpriv->securitypriv.dot11PrivacyAlgrthm,pwrpriv->wepkeymask);
		if  ((WEP40_ENCRYPTION == rtlpriv->securitypriv.dot11PrivacyAlgrthm) ||
			(WEP104_ENCRYPTION == rtlpriv->securitypriv.dot11PrivacyAlgrthm)) {
			int keyid;

			for (keyid = 0; keyid < 4; keyid++) {
				if (pwrpriv->wepkeymask & BIT(keyid)) {
					if (keyid == rtlpriv->securitypriv.dot11PrivacyKeyIndex)
						rtw_set_key(rtlpriv,&rtlpriv->securitypriv, keyid, 1);
					else
						rtw_set_key(rtlpriv,&rtlpriv->securitypriv, keyid, 0);
				}
			}
		}
	}
#endif
	up(&pwrpriv->lock);

	ret = 0;
exit:
	pwrpriv->bInSuspend = false;
	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "<===  %s return %d.............. in %dms\n", __FUNCTION__
		, ret, rtw_get_passing_time_ms(start_time));

	return ret;
}

#ifdef CONFIG_AUTOSUSPEND
void autosuspend_enter(struct rtl_priv* rtlpriv)
{
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "==>autosuspend_enter...........\n");

	pwrpriv->bInternalAutoSuspend = true;
	pwrpriv->bips_processing = true;

	if (ERFOFF == pwrpriv->change_rfpwrstate) {
		usb_enable_autosuspend(dvobj->pusbdev);

			usb_autopm_put_interface(dvobj->pusbintf);
	}
	RT_TRACE(rtlpriv, COMP_USB, DBG_DMESG, "...pm_usage_cnt(%d).....\n", atomic_read(&(dvobj->pusbintf->pm_usage_cnt)));

}

int autoresume_enter(struct rtl_priv* rtlpriv)
{
	int result = _SUCCESS;
	struct pwrctrl_priv *pwrpriv = &rtlpriv->pwrctrlpriv;
	struct security_priv* psecuritypriv=&(rtlpriv->securitypriv);
	struct mlme_ext_priv	*pmlmeext = &rtlpriv->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);

	RT_TRACE(rtlpriv, COMP_USB, DBG_DMESG, "====> autoresume_enter \n");

	if (ERFOFF == pwrpriv->rf_pwrstate) {
		pwrpriv->ps_flag = false;
			if (usb_autopm_get_interface(dvobj->pusbintf) < 0) {
				RT_TRACE(rtlpriv, COMP_USB, DBG_DMESG,  "can't get autopm: %d\n", result);
				result = _FAIL;
				goto error_exit;
			}

		RT_TRACE(rtlpriv, COMP_USB, DBG_DMESG, "...pm_usage_cnt(%d).....\n", atomic_read(&(dvobj->pusbintf->pm_usage_cnt)));
	}
	RT_TRACE(rtlpriv, COMP_USB, DBG_DMESG, "<==== autoresume_enter \n");
error_exit:

	return result;
}
#endif


static int rtw_net_set_mac_address(struct net_device *ndev, void *p)
{
	struct rtl_priv *rtlpriv = rtl_priv(ndev);
	struct sockaddr *addr = p;

	if (rtlpriv->initialized == false) {
		/*
		 * DBG_871X("r8711_net_set_mac_address(), MAC=%x:%x:%x:%x:%x:%x\n", addr->sa_data[0], addr->sa_data[1], addr->sa_data[2], addr->sa_data[3],
		 * addr->sa_data[4], addr->sa_data[5]);
		 */
		memcpy(rtlpriv->mac80211.mac_addr, addr->sa_data, ETH_ALEN);
		/*
		 * memcpy(ndev->dev_addr, addr->sa_data, ETH_ALEN);
		 * rtlpriv->bset_hwaddr = true;
		 */
	}

	return 0;
}

/*
 * ULLI messy but needed
 */

struct net_device_stats *rtw_net_get_stats(struct net_device *ndev);
uint loadparam(struct rtl_priv *rtlpriv, struct net_device *ndev);

static const struct net_device_ops rtw_netdev_ops = {
	.ndo_open = netdev_open,
	.ndo_stop = netdev_close,
	.ndo_start_xmit = rtw_xmit_entry,
	.ndo_set_mac_address = rtw_net_set_mac_address,
	.ndo_get_stats = rtw_net_get_stats,
	.ndo_do_ioctl = rtw_ioctl,
};


/*
 * drv_init() - a device potentially for us
 *
 * notes: drv_init() is called when the bus driver has located a card for us to support.
 *        We accept the new device by returning 0.
*/

struct rtl_priv  *rtw_sw_export = NULL;

static char *ifname = "wlan%d";

static int rtw_init_netdev_name(struct net_device *ndev, const char *ifname)
{
	if (dev_alloc_name(ndev, ifname) < 0)
		/*
		 * RT_TRACE(_module_os_intfs_c_, _drv_err_, ("dev_alloc_name, fail!\n"));
		 */

	netif_carrier_off(ndev);
	return 0;
}


static void _rtl_usb_init_tx(struct rtl_priv *rtlpriv)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct _rtw_hal	*pHalData	= GET_HAL_DATA(rtlpriv);

	if (IS_SUPER_SPEED_USB(rtlpriv))
		rtlusb->max_bulk_out_size = USB_SUPER_SPEED_BULK_SIZE;	/* 1024 bytes */
	else if (IS_HIGH_SPEED_USB(rtlpriv))
		rtlusb->max_bulk_out_size = USB_HIGH_SPEED_BULK_SIZE;	/* 512 bytes */
	else
		rtlusb->max_bulk_out_size = USB_FULL_SPEED_BULK_SIZE; 	/*64 bytes */

	pHalData->UsbTxAggDescNum	= 6;	/* only 4 bits */

	if (IS_HARDWARE_TYPE_8812AU(rtlhal))	/* page added for Jaguar */
		pHalData->UsbTxAggDescNum = 3;
}

static void _rtl_usb_init_rx(struct rtl_priv *rtlpriv)
{
	struct _rtw_hal	*pHalData	= GET_HAL_DATA(rtlpriv);
	struct rtl_usb	*rtlusb = rtl_usbdev(rtlpriv);

	pHalData->UsbRxAggBlockCount	= 8; 			/* unit : 512b */
	pHalData->UsbRxAggBlockTimeout	= 0x6;
	pHalData->UsbRxAggPageCount	= 16; 			/* uint :128 b //0x0A;	// 10 = MAX_RX_DMA_BUFFER_SIZE/2/pHalData->UsbBulkOutSize */
	pHalData->UsbRxAggPageTimeout = 0x6; 			/* 6, absolute time = 34ms/(2^6) */

	pHalData->RegAcUsbDmaSize = 4;
	pHalData->RegAcUsbDmaTime = 8;

	rtlpriv->cfg->usb_interface_cfg->usb_endpoint_mapping(rtlpriv);
}

/* ULLI : ugly but currently we do this here */

/***************************************************/

static void _rtl_init_deferred_work(struct rtl_priv *rtlpriv)
{
#if 0
	/* <1> timer */
	setup_timer(&rtlpriv->works.watchdog_timer,
		    rtl_watch_dog_timer_callback, (unsigned long)hw);
	setup_timer(&rtlpriv->works.dualmac_easyconcurrent_retrytimer,
		    rtl_easy_concurrent_retrytimer_callback, (unsigned long)hw);
	/* <2> work queue */
	rtlpriv->works.hw = hw;
	rtlpriv->works.rtl_wq = alloc_workqueue("%s", 0, 0, rtlpriv->cfg->name);
	INIT_DELAYED_WORK(&rtlpriv->works.watchdog_wq,
			  (void *)rtl_watchdog_wq_callback);
	INIT_DELAYED_WORK(&rtlpriv->works.ips_nic_off_wq,
			  (void *)rtl_ips_nic_off_wq_callback);
	INIT_DELAYED_WORK(&rtlpriv->works.ps_work,
			  (void *)rtl_swlps_wq_callback);
	INIT_DELAYED_WORK(&rtlpriv->works.ps_rfon_wq,
			  (void *)rtl_swlps_rfon_wq_callback);
	INIT_DELAYED_WORK(&rtlpriv->works.fwevt_wq,
			  (void *)rtl_fwevt_wq_callback);
#endif
}

static void rtl_deinit_deferred_work(struct rtl_priv *rtlpriv)
{
#if 0
	del_timer_sync(&rtlpriv->works.watchdog_timer);

	cancel_delayed_work(&rtlpriv->works.watchdog_wq);
	cancel_delayed_work(&rtlpriv->works.ips_nic_off_wq);
	cancel_delayed_work(&rtlpriv->works.ps_work);
	cancel_delayed_work(&rtlpriv->works.ps_rfon_wq);
	cancel_delayed_work(&rtlpriv->works.fwevt_wq);
#endif
}

static int rtl_init_core(struct rtl_priv *rtlpriv)
{
	struct rtl_mac *rtlmac = rtl_mac(rtlpriv);

	/* <1> init mac80211 */
#if 0
	_rtl_init_mac80211(hw);
	rtlmac->hw = hw;

	/* <2> rate control register */
	hw->rate_control_algorithm = "rtl_rc";

	/*
	 * <3> init CRDA must come after init
	 * mac80211 hw  in _rtl_init_mac80211.
	 */
	if (rtl_regd_init(hw, rtl_reg_notifier)) {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_EMERG, "REGD init failed\n");
		return 1;
	}
#endif
	/* <4> locks */
#if 0
	mutex_init(&rtlpriv->locks.conf_mutex);
	spin_lock_init(&rtlpriv->locks.ips_lock);
	spin_lock_init(&rtlpriv->locks.irq_th_lock);
	spin_lock_init(&rtlpriv->locks.h2c_lock);
	spin_lock_init(&rtlpriv->locks.rf_ps_lock);
	spin_lock_init(&rtlpriv->locks.rf_lock);
	spin_lock_init(&rtlpriv->locks.waitq_lock);
	spin_lock_init(&rtlpriv->locks.entry_list_lock);
	spin_lock_init(&rtlpriv->locks.cck_and_rw_pagea_lock);
	spin_lock_init(&rtlpriv->locks.check_sendpkt_lock);
	spin_lock_init(&rtlpriv->locks.fw_ps_lock);
	spin_lock_init(&rtlpriv->locks.lps_lock);
	spin_lock_init(&rtlpriv->locks.iqk_lock);
	/* <5> init list */
	INIT_LIST_HEAD(&rtlpriv->entry_list);
#endif
	rtlmac->link_state = MAC80211_NOLINK;

	/* <6> init deferred work */
	_rtl_init_deferred_work(rtlpriv);

	return 0;
}

/***************************************************/





int rtw_usb_probe(struct usb_interface *pusb_intf, const struct usb_device_id *pdid,
	struct rtl_hal_cfg *rtl_hal_cfg)
{
	struct rtl_usb *rtlusb;
	struct usb_device *udev;
	struct rtl_priv *rtlpriv = NULL;
	struct net_device *ndev = NULL;
	int status = _FAIL;

	ndev = alloc_etherdev_mq(sizeof(*rtlpriv), 4);
	if (!ndev)
		goto exit;

	rtlpriv = netdev_priv(ndev);
	rtlusb = rtl_usbdev(rtlpriv);
	usb_dvobj_init(pusb_intf, rtlusb);

	rtlpriv = netdev_priv(ndev);
	rtlpriv->ndev = ndev;

	rtlusb->rtlpriv = rtlpriv;
	udev = rtlusb->udev;

	/* this spin lock must be initialized early */
	spin_lock_init(&rtlpriv->locks.usb_lock);
#if 0
	INIT_WORK(&rtlpriv->works.fill_h2c_cmd,
		  rtl_fill_h2c_cmd_work_callback);
	INIT_WORK(&rtlpriv->works.lps_change_work,
		  rtl_lps_change_work_callback);
#endif


	/* ULLI must clean/reorder this mess */

	rtw_dbgp_flag_init(rtlpriv);

	/* ULLI: Init IO handler */
	_rtl_usb_io_handler_init(&udev->dev, rtlpriv);

	rtlpriv->bDriverStopped=true;

	rtlpriv->rtlhal.interface = INTF_USB;

	if (pdid->driver_info == RTL8812) {
		rtlpriv->rtlhal.hw_type = HARDWARE_TYPE_RTL8812AU;
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "CHIP TYPE: RTL8812\n");
	} else if (pdid->driver_info == RTL8821) {
		/* rtlpriv->HardwareType = HARDWARE_TYPE_RTL8811AU; */
		rtlpriv->rtlhal.hw_type = HARDWARE_TYPE_RTL8821U;
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "CHIP TYPE: RTL8811AU or RTL8821U\n");
	}

	/* ndev->init = NULL; */

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "register rtw_netdev_ops to netdev_ops\n");
	ndev->netdev_ops = &rtw_netdev_ops;

#ifdef CONFIG_TCP_CSUM_OFFLOAD_TX
	ndev->features |= NETIF_F_IP_CSUM;
#endif
	/* ndev->tx_timeout = NULL; */
	ndev->watchdog_timeo = HZ*3; /* 3 second timeout */
#ifdef CONFIG_WIRELESS_EXT
	ndev->wireless_handlers = (struct iw_handler_def *)&rtw_handlers_def;
#endif


	/* step 2. */
	loadparam(rtlpriv, ndev);

	SET_NETDEV_DEV(ndev, dvobj_to_dev(rtlusb));
	rtlpriv = rtl_priv(ndev);

	/* step 2. hook cfg->ops, allocate HalData */
	/* hal_set_hal_ops(rtlpriv); */
	rtlpriv->HalData = rtw_zmalloc(sizeof( struct _rtw_hal));
	if (rtlpriv->HalData == NULL) {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "cant not alloc memory for HAL DATA \n");
	}

	rtlpriv->cfg = rtl_hal_cfg;

	/* step read_chip_version */
	rtlpriv->cfg->ops->read_chip_version(rtlpriv);

	/* step usb endpoint mapping */
	 _rtl_usb_init_tx(rtlpriv);
	 _rtl_usb_init_rx(rtlpriv);

	/* step read efuse/eeprom data and get mac_addr */
	rtlpriv->cfg->ops->read_adapter_info(rtlpriv);

	/* step 5. */
	if(rtlpriv->cfg->ops->init_sw_vars(rtlpriv) ==_FAIL) {
		goto free_hal_data;
	}

#ifdef CONFIG_PM
	if (rtlpriv->pwrctrlpriv.bSupportRemoteWakeup) {
		rtlusb->udev->do_remote_wakeup=1;
		pusb_intf->needs_remote_wakeup = 1;
		device_init_wakeup(&pusb_intf->dev, 1);
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "\n  rtlpriv->pwrctrlpriv.bSupportRemoteWakeup~~~~~~\n");
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "\n  rtlpriv->pwrctrlpriv.bSupportRemoteWakeup~~~[%d]~~~\n",device_may_wakeup(&pusb_intf->dev));
	}
#endif

#ifdef CONFIG_AUTOSUSPEND
	if (rtlpriv->registrypriv.power_mgnt != PS_MODE_ACTIVE) {
		if(rtlpriv->registrypriv.usbss_enable ){ 	/* autosuspend (2s delay) */
			dvobj->pusbdev->dev.power.autosuspend_delay = 0 * HZ;//15 * HZ; idle-delay time

			usb_enable_autosuspend(dvobj->pusbdev);

			/* usb_autopm_get_interface(rtl_usbdev(rtlpriv)->pusbintf );//init pm_usage_cnt ,let it start from 1 */

			RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "%s...pm_usage_cnt(%d).....\n",__FUNCTION__,atomic_read(&(dvobj->pusbintf ->pm_usage_cnt)));
		}
	}
#endif
	/* 2012-07-11 Move here to prevent the 8723AS-VAU BT auto suspend influence */
	if (usb_autopm_get_interface(pusb_intf) < 0) {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD,  "can't get autopm: \n");
	}

	/*  set mac addr */
	rtw_init_netdev_name(ndev, ifname);
	rtw_macaddr_cfg(rtlpriv->mac80211.mac_addr);

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "bDriverStopped:%d, bSurpriseRemoved:%d, initialized:%d, hw_init_completed:%d\n"
		, rtlpriv->bDriverStopped
		, rtlpriv->bSurpriseRemoved
		, rtlpriv->initialized
		, rtlpriv->hw_init_completed
	);

	status = _SUCCESS;

	netif_carrier_off(rtlpriv->ndev);
	/* rtw_netif_stop_queue(ndev); */

/*
 * 	ULLI : for support older kernel < 3.14,
 *	ether_addr_copy(ndev->dev_addr, rtlpriv->eeprompriv.mac_addr);
 */
	memcpy(ndev->dev_addr, rtlpriv->mac80211.mac_addr, ETH_ALEN);

	/* Tell the network stack we exist */
	if (register_netdev(rtlpriv->ndev) != 0) {
		RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD, "register_netdev failed\n");
		status = _FAIL;
		goto free_hal_data;
	}


	return 0;

free_dvobj:
	usb_dvobj_deinit(pusb_intf);

free_hal_data:
	if (status != _SUCCESS && rtlpriv->HalData)
		rtw_mfree(rtlpriv->HalData);

free_adapter:
	if (status != _SUCCESS) {
		if (ndev)
			free_netdev(ndev);

		rtlpriv = NULL;
	}
exit:
	return -ENODEV;
}


static uint8_t rtw_free_drv_sw(struct rtl_priv *rtlpriv)
{
	struct net_device *ndev = (struct net_device *)rtlpriv->ndev;

	/*
	 * we can call rtw_p2p_enable here, but:
	 * 1. rtw_p2p_enable may have IO operation
	 * 2. rtw_p2p_enable is bundled with wext interface
	 */

	free_mlme_ext_priv(&rtlpriv->mlmeextpriv);


	rtw_free_cmd_priv(&rtlpriv->cmdpriv);

	rtw_free_evt_priv(&rtlpriv->evtpriv);

	rtw_free_mlme_priv(&rtlpriv->mlmepriv);

	/* free_io_queue(rtlpriv); */

	_rtw_free_xmit_priv(&rtlpriv->xmitpriv);

	_rtw_free_sta_priv(&rtlpriv->stapriv); /* will free bcmc_stainfo here */

	_rtw_free_recv_priv(&rtlpriv->recvpriv);

	/* rtw_mfree(rtlpriv); */

	rtlpriv->cfg->ops->free_hal_data(rtlpriv);

	return _SUCCESS;

}

void rtw_usb_if1_deinit(struct rtl_priv *rtlpriv)
{
	struct net_device *ndev = rtlpriv->ndev;
	struct mlme_priv *pmlmepriv= &rtlpriv->mlmepriv;

	if(check_fwstate(pmlmepriv, _FW_LINKED))
		rtw_disassoc_cmd(rtlpriv, 0, false);


#ifdef CONFIG_AP_MODE
	free_mlme_ap_info(rtlpriv);
#endif

	if (ndev) {
		unregister_netdev(ndev); /* will call netdev_close() */
	}

	rtw_cancel_all_timer(rtlpriv);
	rtw_dev_unload(rtlpriv);

	RT_TRACE(rtlpriv, COMP_USB, DBG_LOUD,  "+r871xu_dev_remove, hw_init_completed=%d\n", rtlpriv->hw_init_completed);

	rtw_free_drv_sw(rtlpriv);

	if(ndev)
		free_netdev(ndev);
}


static void dump_usb_interface(struct usb_interface *usb_intf)
{
	int	i;
	uint8_t	val8;

	struct usb_device				*udev = interface_to_usbdev(usb_intf);
	struct usb_device_descriptor 	*dev_desc = &udev->descriptor;

	struct usb_host_config			*act_conf = udev->actconfig;
	struct usb_config_descriptor	*act_conf_desc = &act_conf->desc;

	struct usb_host_interface		*host_iface;
	struct usb_interface_descriptor	*iface_desc;
	struct usb_host_endpoint		*host_endp;
	struct usb_endpoint_descriptor	*endp_desc;

#if 1 /* The usb device this usb interface belongs to */
	DBG_871X("usb_interface:%p, usb_device:%p(num:%d, path:%s), usb_device_descriptor:%p\n", usb_intf, udev, udev->devnum, udev->devpath, dev_desc);
	DBG_871X("bLength:%u\n", dev_desc->bLength);
	DBG_871X("bDescriptorType:0x%02x\n", dev_desc->bDescriptorType);
	DBG_871X("bcdUSB:0x%04x\n", le16_to_cpu(dev_desc->bcdUSB));
	DBG_871X("bDeviceClass:0x%02x\n", dev_desc->bDeviceClass);
	DBG_871X("bDeviceSubClass:0x%02x\n", dev_desc->bDeviceSubClass);
	DBG_871X("bDeviceProtocol:0x%02x\n", dev_desc->bDeviceProtocol);
	DBG_871X("bMaxPacketSize0:%u\n", dev_desc->bMaxPacketSize0);
	DBG_871X("idVendor:0x%04x\n", le16_to_cpu(dev_desc->idVendor));
	DBG_871X("idProduct:0x%04x\n", le16_to_cpu(dev_desc->idProduct));
	DBG_871X("bcdDevice:0x%04x\n", le16_to_cpu(dev_desc->bcdDevice));
	DBG_871X("iManufacturer:0x02%x\n", dev_desc->iManufacturer);
	DBG_871X("iProduct:0x%02x\n", dev_desc->iProduct);
	DBG_871X("iSerialNumber:0x%02x\n", dev_desc->iSerialNumber);
	DBG_871X("bNumConfigurations:%u\n", dev_desc->bNumConfigurations);
#endif


#if 1 /* The acting usb_config_descriptor */
	DBG_871X("\nact_conf_desc:%p\n", act_conf_desc);
	DBG_871X("bLength:%u\n", act_conf_desc->bLength);
	DBG_871X("bDescriptorType:0x%02x\n", act_conf_desc->bDescriptorType);
	DBG_871X("wTotalLength:%u\n", le16_to_cpu(act_conf_desc->wTotalLength));
	DBG_871X("bNumInterfaces:%u\n", act_conf_desc->bNumInterfaces);
	DBG_871X("bConfigurationValue:0x%02x\n", act_conf_desc->bConfigurationValue);
	DBG_871X("iConfiguration:0x%02x\n", act_conf_desc->iConfiguration);
	DBG_871X("bmAttributes:0x%02x\n", act_conf_desc->bmAttributes);
	DBG_871X("bMaxPower=%u\n", act_conf_desc->bMaxPower);
#endif


	DBG_871X("****** num of altsetting = (%d) ******/\n", usb_intf->num_altsetting);
	/* Get he host side alternate setting (the current alternate setting) for this interface*/
	host_iface = usb_intf->cur_altsetting;
	iface_desc = &host_iface->desc;

#if 1 /* The current alternate setting*/
	DBG_871X("\nusb_interface_descriptor:%p:\n", iface_desc);
	DBG_871X("bLength:%u\n", iface_desc->bLength);
	DBG_871X("bDescriptorType:0x%02x\n", iface_desc->bDescriptorType);
	DBG_871X("bInterfaceNumber:0x%02x\n", iface_desc->bInterfaceNumber);
	DBG_871X("bAlternateSetting=%x\n", iface_desc->bAlternateSetting);
	DBG_871X("bNumEndpoints=%x\n", iface_desc->bNumEndpoints);
	DBG_871X("bInterfaceClass=%x\n", iface_desc->bInterfaceClass);
	DBG_871X("bInterfaceSubClass=%x\n", iface_desc->bInterfaceSubClass);
	DBG_871X("bInterfaceProtocol=%x\n", iface_desc->bInterfaceProtocol);
	DBG_871X("iInterface=%x\n", iface_desc->iInterface);
#endif


#if 1
	/* DBG_871X("\ndump usb_endpoint_descriptor:\n"); */

	for (i = 0; i < iface_desc->bNumEndpoints; i++) {
		host_endp = host_iface->endpoint + i;
		if (host_endp) {
			endp_desc = &host_endp->desc;

			DBG_871X("\nusb_endpoint_descriptor(%d):\n", i);
			DBG_871X("bLength=%x\n",endp_desc->bLength);
			DBG_871X("bDescriptorType=%x\n",endp_desc->bDescriptorType);
			DBG_871X("bEndpointAddress=%x\n",endp_desc->bEndpointAddress);
			DBG_871X("bmAttributes=%x\n",endp_desc->bmAttributes);
			DBG_871X("wMaxPacketSize=%x\n",endp_desc->wMaxPacketSize);
			DBG_871X("wMaxPacketSize=%x\n",le16_to_cpu(endp_desc->wMaxPacketSize));
			DBG_871X("bInterval=%x\n",endp_desc->bInterval);
			/* DBG_871X("bRefresh=%x\n",pendp_desc->bRefresh); */
			/* DBG_871X("bSynchAddress=%x\n",pendp_desc->bSynchAddress); */

			if (RT_usb_endpoint_is_bulk_in(endp_desc)) {
				DBG_871X("RT_usb_endpoint_is_bulk_in = %x\n", RT_usb_endpoint_num(endp_desc));
				/* rtlusb->RtNumInPipes++; */
			} else if (RT_usb_endpoint_is_bulk_out(endp_desc)) {
				DBG_871X("RT_usb_endpoint_is_bulk_out = %x\n", RT_usb_endpoint_num(endp_desc));
				/* rtlusb->RtNumOutPipes++; */
			}
			/* rtlusb->ep_num[i] = RT_usb_endpoint_num(pendp_desc); */
		}
	}

	/*
	 * DBG_871X("nr_endpoint=%d, in_num=%d, out_num=%d\n\n", rtlusb->nr_endpoint, rtlusb->RtNumInPipes, rtlusb->RtNumOutPipes);
	 */
#endif

	if (udev->speed == USB_SPEED_HIGH)
		DBG_871X("USB_SPEED_HIGH\n");
	else
		DBG_871X("NON USB_SPEED_HIGH\n");

}

/*
 * dev_remove() - our device is being removed
*/
/*
 * rmmod module & unplug(SurpriseRemoved) will call r871xu_dev_remove() => how to recognize both
 */
void rtw_usb_disconnect(struct usb_interface *pusb_intf)
{
	struct rtl_usb *rtlusb = usb_get_intfdata(pusb_intf);
	struct rtl_priv *rtlpriv = rtlusb->rtlpriv;
	struct net_device *ndev = rtlpriv->ndev;
	struct mlme_priv *pmlmepriv= &rtlpriv->mlmepriv;

	DBG_871X("+rtw_dev_remove\n");

	rtw_pm_set_ips(rtlpriv, IPS_NONE);
	rtw_pm_set_lps(rtlpriv, PS_MODE_ACTIVE);

	LeaveAllPowerSaveMode(rtlpriv);

	rtw_usb_if1_deinit(rtlpriv);

	usb_dvobj_deinit(pusb_intf);

	DBG_871X("-r871xu_dev_remove, done\n");

	return;
}

/*  11/16/2008 MH Read one byte from real Efuse. */
uint8_t
efuse_OneByteRead(
		struct rtl_priv *rtlpriv,
		u16			addr,
		uint8_t			*data)
{
	uint8_t	tmpidx = 0;
	uint8_t	bResult;

	//DBG_871X("===> EFUSE_OneByteRead(), addr = %x\n", addr);
	//DBG_871X("===> EFUSE_OneByteRead() start, 0x34 = 0x%X\n", rtl_read_dword(rtlpriv, EFUSE_TEST));

	// -----------------e-fuse reg ctrl ---------------------------------
	//address
	rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+1, (uint8_t)(addr&0xff));
	rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2, ((uint8_t)((addr>>8) &0x03) ) |
	(rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2)&0xFC ));

	rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3,  0x72);//read cmd

	while(!(0x80 &rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3))&&(tmpidx<1000))
	{
		mdelay(1);
		tmpidx++;
	}
	if(tmpidx<1000)
	{
		*data=rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]);
		bResult = true;
	}
	else
	{
		*data = 0xff;
		bResult = 0;
	}

	return bResult;
}


/*
 * ULLI need this for rtlwifi-lib
 * void efuse_write_1byte(struct ieee80211_hw *hw, u16 address, u8 value)
 */
/*  11/16/2008 MH Write one byte to reald Efuse. */
uint8_t
efuse_OneByteWrite(
		struct rtl_priv *rtlpriv,
		u16			addr,
		uint8_t			data)
{
	uint8_t	tmpidx = 0;
	uint8_t	bResult= 0;
	uint32_t efuseValue = 0;

	//DBG_871X("===> EFUSE_OneByteWrite(), addr = %x data=%x\n", addr, data);
	//DBG_871X("===> EFUSE_OneByteWrite() start, 0x34 = 0x%X\n", rtl_read_dword(rtlpriv, EFUSE_TEST));

	// -----------------e-fuse reg ctrl ---------------------------------
	//address


	efuseValue = rtl_read_dword(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]);
	efuseValue |= (BIT(21)|BIT(31));
	efuseValue &= ~(0x3FFFF);
	efuseValue |= ((addr<<8 | data) & 0x3FFFF);

	{
		rtl_write_dword(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL], efuseValue);
	}

	while((0x80 &  rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3)) && (tmpidx<100) ){
		mdelay(1);
		tmpidx++;
	}

	if(tmpidx<100)
	{
		bResult = true;
	}
	else
	{
		bResult = 0;
	}

	return bResult;
}
