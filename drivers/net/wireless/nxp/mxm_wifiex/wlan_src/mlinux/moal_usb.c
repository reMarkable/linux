/** @file moal_usb.c
 *
 * @brief This file contains the interfaceing to USB bus
 * driver.
 *
 *
 * Copyright 2014-2020 NXP
 *
 * This software file (the File) is distributed by NXP
 * under the terms of the GNU General Public License Version 2, June 1991
 * (the License).  You may use, redistribute and/or modify the File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 *
 */

/********************************************************
Change log:
    10/21/2008: initial version
********************************************************/

#include "moal_main.h"
#include "moal_usb.h"
extern struct semaphore AddRemoveCardSem;

/********************************************************
		Local Variables
********************************************************/

#if defined(USB8997) || defined(USB9098) || defined(USB9097) || defined(USB8978)
/** Card-type detection frame response */
typedef struct {
	/** 32-bit ACK+WINNER field */
	t_u32 ack_winner;
	/** 32-bit Sequence number */
	t_u32 seq;
	/** 32-bit extend */
	t_u32 extend;
	/** 32-bit chip-revision code */
	t_u32 chip_rev;
	/** 32-bit strap setting */
	t_u32 strap;
} usb_ack_pkt;
#endif

/** NXP USB device */
#define NXP_USB_DEVICE(vid, pid, name)                                         \
	USB_DEVICE(vid, pid), .driver_info = (t_ptr)name

/** Name of the USB driver */
const char usbdriver_name[] = "usbxxx";

/** This structure contains the device signature */
struct usb_device_id woal_usb_table[] = {
/* Enter the device signature inside */
#ifdef USB8897
	{NXP_USB_DEVICE(USB8897_VID_1, USB8897_PID_1, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8897_VID_1, USB8897_PID_2, "NXP WLAN USB Adapter")},
#endif
#ifdef USB8997
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997_PID_1, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997V2_PID_1,
			"NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997_PID_2, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997_PID_3, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997_PID_4, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997_PID_5, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997_PID_6, "NXP WLAN USB Adapter")},
#endif
#ifdef USB8978
	{NXP_USB_DEVICE(USB8978_VID_1, USB8978_PID_1, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8978_VID_1, USB8978_PID_1_BT,
			"NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8978_VID_1, USB8978_PID_2, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8978_VID_1, USB8978_PID_2_BT,
			"NXP WLAN USB Adapter")},
#endif
#ifdef USB9098
	{NXP_USB_DEVICE(USB9098_VID_1, USB9098_PID_1, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB9098_VID_1, USB9098_PID_2, "NXP WLAN USB Adapter")},
#endif
#ifdef USB9097
	{NXP_USB_DEVICE(USB9097_VID_1, USB9097_PID_1, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB9097_VID_1, USB9097_PID_2, "NXP WLAN USB Adapter")},
#endif
	/* Terminating entry */
	{},
};

/** This structure contains the device signature */
struct usb_device_id woal_usb_table_skip_fwdnld[] = {
/* Enter the device signature inside */
#ifdef USB8897
	{NXP_USB_DEVICE(USB8897_VID_1, USB8897_PID_2, "NXP WLAN USB Adapter")},
#endif
#ifdef USB8997
	{NXP_USB_DEVICE(USB8997_VID_1, USB8997_PID_2, "NXP WLAN USB Adapter")},
#endif
#ifdef USB8978
	{NXP_USB_DEVICE(USB8978_VID_1, USB8978_PID_2, "NXP WLAN USB Adapter")},
	{NXP_USB_DEVICE(USB8978_VID_1, USB8978_PID_2_BT,
			"NXP WLAN USB Adapter")},
#endif
#ifdef USB9098
	{NXP_USB_DEVICE(USB9098_VID_1, USB9098_PID_2, "NXP WLAN USB Adapter")},
#endif
#ifdef USB9097
	{NXP_USB_DEVICE(USB9097_VID_1, USB9097_PID_2, "NXP WLAN USB Adapter")},
#endif
	/* Terminating entry */
	{},
};

static mlan_status woal_usb_submit_rx_urb(urb_context *ctx, int size);
static int woal_usb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id);
static void woal_usb_disconnect(struct usb_interface *intf);
static mlan_status woal_usb_write_data_sync(moal_handle *handle,
					    mlan_buffer *pmbuf, t_u32 endpoint,
					    t_u32 timeout);
static mlan_status woal_usb_read_data_sync(moal_handle *handle,
					   mlan_buffer *pmbuf, t_u32 endpoint,
					   t_u32 timeout);
#ifdef CONFIG_PM
static int woal_usb_suspend(struct usb_interface *intf, pm_message_t message);
static int woal_usb_resume(struct usb_interface *intf);
#endif /* CONFIG_PM */

/** woal_usb_driver */
static struct usb_driver REFDATA woal_usb_driver = {
	/* Driver name */
	.name = usbdriver_name,

	/* Probe function name */
	.probe = woal_usb_probe,

	/* Disconnect function name */
	.disconnect = woal_usb_disconnect,

	/* Device signature table */
	.id_table = woal_usb_table,
#ifdef CONFIG_PM
	/* Suspend function name */
	.suspend = woal_usb_suspend,

	/* Resume function name */
	.resume = woal_usb_resume,

	/* Reset resume function name */
	.reset_resume = woal_usb_resume,

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
	/* Driver supports autosuspend */
	.supports_autosuspend = 1,
#endif
#endif /* CONFIG_PM */
};

MODULE_DEVICE_TABLE(usb, woal_usb_table);
MODULE_DEVICE_TABLE(usb, woal_usb_table_skip_fwdnld);

/* moal interface ops */
static moal_if_ops usb_ops;

/********************************************************
		Global Variables
********************************************************/

extern int skip_fwdnld;
extern int max_tx_buf;

/********************************************************
		Local Functions
********************************************************/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
/**
 *  @brief This function receive packet of the data/cmd/event packet
 *         and pass to MLAN
 *
 *  @param urb		Pointer to struct urb
 *  @param regs		Registers
 *
 *  @return 	   	N/A
 */
static void woal_usb_receive(struct urb *urb, struct pt_regs *regs)
#else
/**
 * @brief This function receive packet of the data/cmd/event packet
 *         and pass to MLAN
 *
 *  @param urb		Pointer to struct urb
 *
 *  @return 	   	N/A
 */
static void woal_usb_receive(struct urb *urb)
#endif
{
	urb_context *context = NULL;
	moal_handle *handle = NULL;
	mlan_buffer *pmbuf = NULL;
	struct usb_card_rec *cardp = NULL;
	int recv_length;
	int size;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!urb || !urb->context) {
		PRINTM(MERROR, "URB or URB context is not valid in USB Rx\n");
		LEAVE();
		return;
	}
	context = (urb_context *)urb->context;
	handle = context->handle;
	pmbuf = context->pmbuf;
	recv_length = urb->actual_length;

	if (!handle || !handle->card || !pmbuf) {
		PRINTM(MERROR,
		       "moal handle, card structure or mlan_buffer is not valid in USB Rx\n");
		LEAVE();
		return;
	}
	cardp = (struct usb_card_rec *)handle->card;
	if (cardp->rx_cmd_ep == context->ep)
		atomic_dec(&cardp->rx_cmd_urb_pending);
	else
		atomic_dec(&cardp->rx_data_urb_pending);

	if (recv_length) {
		if (urb->status || (handle->surprise_removed == MTRUE)) {
			if (handle->surprise_removed || handle->is_suspended) {
				woal_free_mlan_buffer(handle, pmbuf);
				context->pmbuf = NULL;
				goto rx_exit;
			} else {
				PRINTM(MERROR,
				       "EP %d Rx URB status failure: %d\n",
				       context->ep, urb->status);
				/* Do not free mlan_buffer in case of command ep
				 */
				if (cardp->rx_cmd_ep != context->ep)
					woal_free_mlan_buffer(handle, pmbuf);
				goto setup_for_next;
			}
		}
		pmbuf->data_len = recv_length;
		pmbuf->flags |= MLAN_BUF_FLAG_RX_DEAGGR;
		/* Send packet to MLAN */
		atomic_inc(&handle->rx_pending);
		status = mlan_recv(handle->pmlan_adapter, pmbuf, context->ep);
		PRINTM(MINFO, "Receive length = 0x%x, status=%d\n", recv_length,
		       status);
		if (status == MLAN_STATUS_PENDING) {
			queue_work(handle->workqueue, &handle->main_work);
			/* urb for data_ep is re-submitted now, unless we reach
			 * HIGH_RX_PENDING */
			/* urb for cmd_ep will be re-submitted in callback
			 * moal_recv_complete */
			if (cardp->rx_cmd_ep == context->ep)
				goto rx_exit;
			else if (atomic_read(&handle->rx_pending) >=
				 HIGH_RX_PENDING) {
				context->pmbuf = NULL;
				goto rx_exit;
			}
		} else {
			atomic_dec(&handle->rx_pending);
			if (status == MLAN_STATUS_FAILURE) {
				PRINTM(MERROR,
				       "MLAN fail to process the receive data\n");
			} else if ((status == MLAN_STATUS_SUCCESS) &&
				   (pmbuf->flags &
				    MLAN_BUF_FLAG_SLEEPCFM_RESP)) {
				pmbuf->flags &= ~MLAN_BUF_FLAG_SLEEPCFM_RESP;
				queue_work(handle->workqueue,
					   &handle->main_work);
			}
			/* Do not free mlan_buffer in case of command ep */
			if (cardp->rx_cmd_ep != context->ep)
				woal_free_mlan_buffer(handle, pmbuf);
		}
	} else if (urb->status) {
		if (!((cardp->rx_data_ep == context->ep) &&
		      (cardp->resubmit_urbs == 1))) {
			if (!handle->is_suspended) {
				PRINTM(MMSG, "Card is removed: %d\n",
				       urb->status);
				handle->surprise_removed = MTRUE;
			}
		}
		woal_free_mlan_buffer(handle, pmbuf);
		context->pmbuf = NULL;
		goto rx_exit;
	} else {
		/* Do not free mlan_buffer in case of command ep */
		if (cardp->rx_cmd_ep != context->ep)
			woal_free_mlan_buffer(handle, pmbuf);
		goto setup_for_next;
	}

setup_for_next:
	if (cardp->rx_cmd_ep == context->ep) {
		size = MLAN_RX_CMD_BUF_SIZE;
	} else {
		if (cardp->rx_deaggr_ctrl.enable) {
			size = cardp->rx_deaggr_ctrl.aggr_max;
			if (cardp->rx_deaggr_ctrl.aggr_mode ==
			    MLAN_USB_AGGR_MODE_NUM) {
				size *= MAX(MLAN_USB_MAX_PKT_SIZE,
					    cardp->rx_deaggr_ctrl.aggr_align);
				size = MAX(size, MLAN_RX_DATA_BUF_SIZE);
			}
		} else
			size = MLAN_RX_DATA_BUF_SIZE;
	}
	woal_usb_submit_rx_urb(context, size);

rx_exit:
	LEAVE();
	return;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
/**
 *  @brief  Call back function to handle the status of the Tx data URB
 *
 *  @param urb      Pointer to urb structure
 *  @param regs     Registers
 *
 *  @return         N/A
 */
static void woal_usb_tx_complete(struct urb *urb, struct pt_regs *regs)
#else
/**
 * @brief  Call back function to handle the status of the Tx data URB
 *
 * @param urb      Pointer to urb structure
 *
 * @return         N/A
 */
static void woal_usb_tx_complete(struct urb *urb)
#endif
{
	urb_context *context = NULL;
	moal_handle *handle = NULL;
	struct usb_card_rec *cardp = NULL;

	ENTER();

	if (!urb || !urb->context) {
		PRINTM(MERROR,
		       "URB or URB context is not valid in USB Tx complete\n");
		LEAVE();
		return;
	}
	context = (urb_context *)urb->context;
	handle = context->handle;

	if (!handle || !handle->card || !context->pmbuf) {
		PRINTM(MERROR,
		       "moal handle, card structure or mlan_buffer is not valid in USB Tx complete\n");
		LEAVE();
		return;
	}
	cardp = handle->card;

	/* Handle the transmission complete validations */
	if (urb->status) {
		PRINTM(MERROR, "EP %d Tx URB status failure: %d\n", context->ep,
		       urb->status);
		mlan_write_data_async_complete(handle->pmlan_adapter,
					       context->pmbuf, context->ep,
					       MLAN_STATUS_FAILURE);
	} else {
		mlan_write_data_async_complete(handle->pmlan_adapter,
					       context->pmbuf, context->ep,
					       MLAN_STATUS_SUCCESS);
	}

	/* Decrease pending URB counter */
	if (context->ep == cardp->tx_cmd_ep)
		atomic_dec(&cardp->tx_cmd_urb_pending);
	else if (context->ep == cardp->tx_data_ep)
		atomic_dec(&cardp->tx_data_urb_pending);

	queue_work(handle->workqueue, &handle->main_work);

	LEAVE();
	return;
}

/**
 *  @brief This function sets up the data to receive
 *
 *  @param ctx		Pointer to urb_context structure
 *  @param size	        Skb size
 *
 *  @return 	   	MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_usb_submit_rx_urb(urb_context *ctx, int size)
{
	moal_handle *handle = ctx->handle;
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;
	mlan_status ret = MLAN_STATUS_FAILURE;
	t_u8 *data = NULL;

	ENTER();

	if (handle->surprise_removed || handle->is_suspended) {
		if ((cardp->rx_cmd_ep == ctx->ep) && ctx->pmbuf) {
			woal_free_mlan_buffer(handle, ctx->pmbuf);
			ctx->pmbuf = NULL;
		}
		PRINTM(MERROR,
		       "Card removed/suspended, EP %d Rx URB submit skipped\n",
		       ctx->ep);
		goto rx_ret;
	}

	if (cardp->rx_cmd_ep != ctx->ep) {
		ctx->pmbuf = woal_alloc_mlan_buffer(handle, size);
		if (!ctx->pmbuf) {
			PRINTM(MERROR,
			       "Fail to submit Rx URB due to no memory/skb\n");
			goto rx_ret;
		}
		ctx->pmbuf->data_offset = MLAN_RX_HEADER_LEN;
		data = ctx->pmbuf->pbuf + ctx->pmbuf->data_offset;
	} else {
		ctx->pmbuf->data_offset = 0;
		data = ctx->pmbuf->pbuf + ctx->pmbuf->data_offset;
	}

	if (cardp->rx_cmd_ep == ctx->ep &&
	    cardp->rx_cmd_ep_type == USB_ENDPOINT_XFER_INT)
		usb_fill_int_urb(ctx->urb, cardp->udev,
				 usb_rcvintpipe(cardp->udev, ctx->ep), data,
				 size - ctx->pmbuf->data_offset,
				 woal_usb_receive, (void *)ctx,
				 cardp->rx_cmd_interval);
	else
		usb_fill_bulk_urb(ctx->urb, cardp->udev,
				  usb_rcvbulkpipe(cardp->udev, ctx->ep), data,
				  size - ctx->pmbuf->data_offset,
				  woal_usb_receive, (void *)ctx);
	if (cardp->rx_cmd_ep == ctx->ep)
		atomic_inc(&cardp->rx_cmd_urb_pending);
	else
		atomic_inc(&cardp->rx_data_urb_pending);
	if (usb_submit_urb(ctx->urb, GFP_ATOMIC)) {
		/* Submit URB failure */
		PRINTM(MERROR, "Submit EP %d Rx URB failed: %d\n", ctx->ep,
		       ret);
		woal_free_mlan_buffer(handle, ctx->pmbuf);
		if (cardp->rx_cmd_ep == ctx->ep)
			atomic_dec(&cardp->rx_cmd_urb_pending);
		else
			atomic_dec(&cardp->rx_data_urb_pending);
		ctx->pmbuf = NULL;
		ret = MLAN_STATUS_FAILURE;
	} else {
		ret = MLAN_STATUS_SUCCESS;
	}
rx_ret:
	LEAVE();
	return ret;
}

/********************************************************
		Global Functions
********************************************************/

#if defined(USB8997) || defined(USB9098) || defined(USB9097) || defined(USB8978)
/**
 *  @brief  Check chip revision
 *
 *  @param handle        A pointer to moal_handle structure
 *  @param usb_chip_rev  A pointer to usb_chip_rev variable
 *  @param usb_strap     A pointer to usb_strap
 *
 *  @return 	   	 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_check_chip_revision(moal_handle *handle, t_u32 *usb_chip_rev,
				     t_u32 *usb_strap)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_buffer mbuf;
	t_u8 *tx_buff = 0;
	t_u8 *recv_buff = 0;
	usb_ack_pkt ack_pkt;
	t_u32 extend_ver;
	t_u8 tx_size = CHIP_REV_TX_BUF_SIZE;
	struct usb_card_rec *cardp = handle->card;

	ENTER();

	/* Allocate memory for transmit */
	tx_buff = kzalloc(tx_size, GFP_ATOMIC | GFP_DMA);
	if (tx_buff == NULL) {
		PRINTM(MERROR,
		       "Could not allocate buffer for chip revision check frame transmission\n");
		ret = MLAN_STATUS_FAILURE;
		goto cleanup;
	}

	/* Allocate memory for receive */
	recv_buff = kzalloc(CHIP_REV_RX_BUF_SIZE, GFP_ATOMIC | GFP_DMA);
	if (recv_buff == NULL) {
		PRINTM(MERROR,
		       "Could not allocate buffer for chip revision check frame response\n");
		ret = MLAN_STATUS_FAILURE;
		goto cleanup;
	}

	/* The struct is initialised to all zero */
	memset(&ack_pkt, 0, sizeof(usb_ack_pkt));

	/* Send pseudo data to check winner status first */
	memset(&mbuf, 0, sizeof(mlan_buffer));
	mbuf.pbuf = (t_u8 *)tx_buff;
	mbuf.data_len = tx_size;

	/* Send the chip revision check frame */
	ret = woal_usb_write_data_sync(handle, &mbuf, cardp->tx_cmd_ep,
				       MLAN_USB_BULK_MSG_TIMEOUT);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR,
		       "Chip revision check frame dnld: write_data failed, ret %d\n",
		       ret);
		ret = MLAN_STATUS_FAILURE;
		goto cleanup;
	}

	memset(&mbuf, 0, sizeof(mlan_buffer));
	mbuf.pbuf = (t_u8 *)recv_buff;
	mbuf.data_len = CHIP_REV_RX_BUF_SIZE;

	/* Receive the chip revision check frame response */
	ret = woal_usb_read_data_sync(handle, &mbuf, cardp->rx_cmd_ep,
				      MLAN_USB_BULK_MSG_TIMEOUT);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR,
		       "Chip revision check frame response: read_data failed, ret %d\n",
		       ret);
		ret = MLAN_STATUS_FAILURE;
		goto cleanup;
	}
	moal_memcpy_ext(handle, &ack_pkt, recv_buff, sizeof(usb_ack_pkt),
			sizeof(ack_pkt));
	ack_pkt.ack_winner = woal_le32_to_cpu(ack_pkt.ack_winner);
	ack_pkt.seq = woal_le32_to_cpu(ack_pkt.seq);
	ack_pkt.extend = woal_le32_to_cpu(ack_pkt.extend);
	ack_pkt.chip_rev = woal_le32_to_cpu(ack_pkt.chip_rev);
	ack_pkt.strap = woal_le32_to_cpu(ack_pkt.strap);

	if ((ack_pkt.extend & 0xffff0000) == EXTEND_HDR) {
		extend_ver = ack_pkt.extend & 0x0000ffff;
		*usb_chip_rev = ack_pkt.chip_rev & 0x000000ff;
		if (extend_ver >= EXTEND_V2) {
			PRINTM(MINFO, "chip_rev=0x%x, strap=0x%x\n",
			       *usb_chip_rev, ack_pkt.strap);
			*usb_strap = ack_pkt.strap & 0x7;
		} else
			PRINTM(MINFO, "chip_rev=0x%x\n", *usb_chip_rev);
	}
cleanup:
	kfree(recv_buff);
	kfree(tx_buff);

	LEAVE();
	return ret;
}
#endif

/**
 *  @brief This function unlink urb
 *
 *  @param handle A pointer to moal_handle structure
 *  @return 	  N/A
 */
static void woal_usb_unlink_urb(void *card_desc)
{
	struct usb_card_rec *cardp = (struct usb_card_rec *)card_desc;
	int i;
	ENTER();
	if (cardp) {
		/* Unlink Rx cmd URB */
		if (atomic_read(&cardp->rx_cmd_urb_pending) &&
		    cardp->rx_cmd.urb) {
			usb_kill_urb(cardp->rx_cmd.urb);
		}
		/* Unlink Rx data URBs */
		if (atomic_read(&cardp->rx_data_urb_pending)) {
			for (i = 0; i < MVUSB_RX_DATA_URB; i++) {
				if (cardp->rx_data_list[i].urb)
					usb_kill_urb(
						cardp->rx_data_list[i].urb);
			}
		}
		/* Unlink Tx cmd URB */
		if (atomic_read(&cardp->tx_cmd_urb_pending) &&
		    cardp->tx_cmd.urb) {
			usb_kill_urb(cardp->tx_cmd.urb);
		}
		/* Unlink Tx data URBs */
		if (atomic_read(&cardp->tx_data_urb_pending)) {
			for (i = 0; i < MVUSB_TX_HIGH_WMARK; i++) {
				if (cardp->tx_data_list[i].urb) {
					usb_kill_urb(
						cardp->tx_data_list[i].urb);
				}
			}
		}
	}
	LEAVE();
}

/**
 *  @brief  Free Tx/Rx urb, skb and Rx buffer
 *
 *  @param cardp	Pointer usb_card_rec
 *
 *  @return 	   	N/A
 */
void woal_usb_free(struct usb_card_rec *cardp)
{
	int i;

	ENTER();

	woal_usb_unlink_urb(cardp);
	/* Free Rx data URBs */
	for (i = 0; i < MVUSB_RX_DATA_URB; i++) {
		if (cardp->rx_data_list[i].urb) {
			usb_free_urb(cardp->rx_data_list[i].urb);
			cardp->rx_data_list[i].urb = NULL;
		}
	}
	/* Free Rx cmd URB */
	if (cardp->rx_cmd.urb) {
		usb_free_urb(cardp->rx_cmd.urb);
		cardp->rx_cmd.urb = NULL;
	}

	/* Free Tx data URBs */
	for (i = 0; i < MVUSB_TX_HIGH_WMARK; i++) {
		if (cardp->tx_data_list[i].urb) {
			usb_free_urb(cardp->tx_data_list[i].urb);
			cardp->tx_data_list[i].urb = NULL;
		}
	}
	/* Free Tx cmd URB */
	if (cardp->tx_cmd.urb) {
		usb_free_urb(cardp->tx_cmd.urb);
		cardp->tx_cmd.urb = NULL;
	}

	LEAVE();
	return;
}

static t_u16 woal_update_card_type(t_void *card)
{
	struct usb_card_rec *cardp_usb = (struct usb_card_rec *)card;
	t_u16 card_type = 0;

	/* Update card type */
#ifdef USB8897
	if (woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8897_PID_1 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8897_PID_2) {
		card_type = CARD_TYPE_USB8897;
		moal_memcpy_ext(NULL, driver_version, CARD_USB8897,
				strlen(CARD_USB8897), strlen(driver_version));
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V15, strlen(V15),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef USB8997
	if (woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8997_PID_1 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8997_PID_2 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8997_PID_3 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8997_PID_4 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8997_PID_5 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8997_PID_6 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8997V2_PID_1) {
		card_type = CARD_TYPE_USB8997;
		moal_memcpy_ext(NULL, driver_version, CARD_USB8997,
				strlen(CARD_USB8997), strlen(driver_version));
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V16, strlen(V16),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef USB8978
	if (woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8978_PID_1 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB8978_PID_2) {
		card_type = CARD_TYPE_USB8978;
		moal_memcpy_ext(NULL, driver_version, CARD_USB8978,
				strlen(CARD_USB8978), strlen(driver_version));
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V16, strlen(V16),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef USB9098
	if (woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB9098_PID_1 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB9098_PID_2) {
		card_type = CARD_TYPE_USB9098;
		moal_memcpy_ext(NULL, driver_version, CARD_USB9098,
				strlen(CARD_USB9098), strlen(driver_version));
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V17, strlen(V17),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
#ifdef USB9097
	if (woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB9097_PID_1 ||
	    woal_cpu_to_le16(cardp_usb->udev->descriptor.idProduct) ==
		    USB9097_PID_2) {
		card_type = CARD_TYPE_USB9097;
		moal_memcpy_ext(NULL, driver_version, CARD_USB9097,
				strlen(CARD_USB9097), strlen(driver_version));
		moal_memcpy_ext(NULL,
				driver_version + strlen(INTF_CARDTYPE) +
					strlen(KERN_VERSION),
				V17, strlen(V17),
				strlen(driver_version) - strlen(INTF_CARDTYPE) -
					strlen(KERN_VERSION));
	}
#endif
	return card_type;
}

/**
 *  @brief Sets the configuration values
 *
 *  @param intf		Pointer to usb_interface
 *  @param id		Pointer to usb_device_id
 *
 *  @return 	   	Address of variable usb_cardp, error code otherwise
 */
static int woal_usb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct usb_device *udev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	struct usb_card_rec *usb_cardp = NULL;
	t_u16 card_type = 0;

	ENTER();

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	PRINTM(MMSG,
	       "USB probe: idVendor=%x idProduct=%x bInterfaceNumber=%d\n",
	       id->idVendor, id->idProduct, id->bInterfaceNumber);
#endif

	udev = interface_to_usbdev(intf);
	usb_cardp = kzalloc(sizeof(struct usb_card_rec), GFP_KERNEL);
	if (!usb_cardp) {
		LEAVE();
		return -ENOMEM;
	}

	/* Check probe is for our device */
	for (i = 0; woal_usb_table[i].idVendor; i++) {
		if (woal_cpu_to_le16(udev->descriptor.idVendor) ==
			    woal_usb_table[i].idVendor &&
		    woal_cpu_to_le16(udev->descriptor.idProduct) ==
			    woal_usb_table[i].idProduct) {
			PRINTM(MMSG, "VID/PID = %X/%X, Boot2 version = %X\n",
			       woal_cpu_to_le16(udev->descriptor.idVendor),
			       woal_cpu_to_le16(udev->descriptor.idProduct),
			       woal_cpu_to_le16(udev->descriptor.bcdDevice));
			switch (woal_cpu_to_le16(udev->descriptor.idProduct)) {
#ifdef USB8897
			case USB8897_PID_1:
#endif /* USB8897 */
#ifdef USB8997
			case USB8997_PID_1:
			case USB8997V2_PID_1:
#endif /* USB8997 */
#ifdef USB8978
			case USB8978_PID_1:
			case USB8978_PID_1_BT:
#endif /* USB8978 */
#ifdef USB9098
			case USB9098_PID_1:
#endif /* USB9098 */
#ifdef USB9097
			case USB9097_PID_1:
#endif /* USB9097 */
				/* If skip FW is set, we must return error so
				 * the next driver can download the FW */
				if (skip_fwdnld)
					goto error;
				else
					usb_cardp->boot_state = USB_FW_DNLD;
				break;
#ifdef USB8897
			case USB8897_PID_2:
#endif /* USB8897 */
#ifdef USB8997
			case USB8997_PID_2:
#endif /* USB8997 */
#ifdef USB8978
			case USB8978_PID_2:
			case USB8978_PID_2_BT:
#endif /* USB8978 */
#ifdef USB9098
			case USB9098_PID_2:
#endif /* USB9098 */
#ifdef USB9097
			case USB9097_PID_2:
#endif /* USB9097 */
				usb_cardp->boot_state = USB_FW_READY;
				break;
			}
			/*To do, get card type*/
			/*			if
			   (woal_cpu_to_le16(udev->descriptor.idProduct) ==
			   USB8897_PID_2) usb_cardp->card_type =
			   CARD_TYPE_USB8897; else if
			   (woal_cpu_to_le16(udev->descriptor.idProduct) ==
			   USB8997_PID_2) usb_cardp->card_type =
			   CARD_TYPE_USB997;
			*/
			break;
		}
	}

	if (woal_usb_table[i].idVendor) {
		usb_cardp->udev = udev;
		iface_desc = intf->cur_altsetting;
		usb_cardp->intf = intf;

		PRINTM(MINFO,
		       "bcdUSB = 0x%X bDeviceClass = 0x%X"
		       " bDeviceSubClass = 0x%X, bDeviceProtocol = 0x%X\n",
		       woal_cpu_to_le16(udev->descriptor.bcdUSB),
		       udev->descriptor.bDeviceClass,
		       udev->descriptor.bDeviceSubClass,
		       udev->descriptor.bDeviceProtocol);

		for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
			endpoint = &iface_desc->endpoint[i].desc;
			if ((usb_endpoint_is_bulk_in(endpoint) ||
			     usb_endpoint_is_int_in(endpoint)) &&
			    (usb_endpoint_num(endpoint) ==
				     MLAN_USB_EP_CMD_EVENT ||
			     usb_endpoint_num(endpoint) ==
				     MLAN_USB_EP_CMD_EVENT_IF2)) {
				usb_cardp->rx_cmd_ep_type =
					usb_endpoint_type(endpoint);
				usb_cardp->rx_cmd_interval =
					endpoint->bInterval;
				/* We found a bulk in command/event endpoint */
				PRINTM(MCMND,
				       "Rx CMD/EVT: max packet size = %d, address = %d ep_type=%d\n",
				       woal_le16_to_cpu(
					       endpoint->wMaxPacketSize),
				       endpoint->bEndpointAddress,
				       usb_cardp->rx_cmd_ep_type);
				usb_cardp->rx_cmd_ep =
					(endpoint->bEndpointAddress &
					 USB_ENDPOINT_NUMBER_MASK);

				atomic_set(&usb_cardp->rx_cmd_urb_pending, 0);
				if (usb_endpoint_num(endpoint) ==
				    MLAN_USB_EP_CMD_EVENT_IF2)
					usb_cardp->second_mac = MTRUE;
			}
			if (usb_endpoint_is_bulk_in(endpoint) &&
			    (usb_endpoint_num(endpoint) == MLAN_USB_EP_DATA ||
			     usb_endpoint_num(endpoint) ==
				     MLAN_USB_EP_DATA_IF2)) {
				/* We found a bulk in data endpoint */
				PRINTM(MINFO,
				       "Bulk IN: max packet size = %d, address = %d\n",
				       woal_le16_to_cpu(
					       endpoint->wMaxPacketSize),
				       endpoint->bEndpointAddress);
				usb_cardp->rx_data_ep =
					(endpoint->bEndpointAddress &
					 USB_ENDPOINT_NUMBER_MASK);
				atomic_set(&usb_cardp->rx_data_urb_pending, 0);
			}
			if (usb_endpoint_is_bulk_out(endpoint) &&
			    (usb_endpoint_num(endpoint) == MLAN_USB_EP_DATA ||
			     usb_endpoint_num(endpoint) ==
				     MLAN_USB_EP_DATA_IF2)) {
				/* We found a bulk out data endpoint */
				PRINTM(MCMND,
				       "Bulk OUT: max packet size = %d, address = %d\n",
				       woal_le16_to_cpu(
					       endpoint->wMaxPacketSize),
				       endpoint->bEndpointAddress);
				usb_cardp->tx_data_ep =
					endpoint->bEndpointAddress;
				atomic_set(&usb_cardp->tx_data_urb_pending, 0);
				usb_cardp->tx_data_maxpktsize =
					woal_le16_to_cpu(
						endpoint->wMaxPacketSize);
			}

			if ((usb_endpoint_is_bulk_out(endpoint) ||
			     usb_endpoint_is_int_out(endpoint)) &&
			    (usb_endpoint_num(endpoint) ==
				     MLAN_USB_EP_CMD_EVENT ||
			     usb_endpoint_num(endpoint) ==
				     MLAN_USB_EP_CMD_EVENT_IF2)) {
				usb_cardp->tx_cmd_ep_type =
					usb_endpoint_type(endpoint);
				usb_cardp->tx_cmd_interval =
					endpoint->bInterval;
				/* We found a bulk out command/event endpoint */
				PRINTM(MCMND,
				       "Tx CMD: max packet size = %d, address = %d ep_type=%d\n",
				       woal_le16_to_cpu(
					       endpoint->wMaxPacketSize),
				       endpoint->bEndpointAddress,
				       usb_cardp->tx_cmd_ep_type);
				usb_cardp->tx_cmd_ep =
					endpoint->bEndpointAddress;
				atomic_set(&usb_cardp->tx_cmd_urb_pending, 0);
				usb_cardp->tx_cmd_maxpktsize = woal_le16_to_cpu(
					endpoint->wMaxPacketSize);
			}
		}

		if (usb_cardp->boot_state == USB_FW_DNLD) {
			if (!usb_cardp->tx_cmd_ep || !usb_cardp->rx_cmd_ep)
				goto error;
		} else if (usb_cardp->boot_state == USB_FW_READY) {
			if (!usb_cardp->tx_cmd_ep || !usb_cardp->tx_data_ep ||
			    !usb_cardp->rx_cmd_ep || !usb_cardp->rx_data_ep) {
				PRINTM(MERROR,
				       "%s: invalid endpoint assignment\n",
				       __FUNCTION__);
				goto error;
			}
		}

		usb_cardp->tx_aggr_ctrl.enable = MFALSE;
		usb_cardp->tx_aggr_ctrl.aggr_mode = MLAN_USB_AGGR_MODE_NUM;
		usb_cardp->tx_aggr_ctrl.aggr_align =
			MAX(max_tx_buf, MLAN_USB_TX_AGGR_ALIGN);
		usb_cardp->tx_aggr_ctrl.aggr_max = MLAN_USB_TX_MAX_AGGR_NUM;
		usb_cardp->tx_aggr_ctrl.aggr_tmo =
			MLAN_USB_TX_AGGR_TIMEOUT_MSEC * 1000;
		usb_cardp->rx_deaggr_ctrl.enable = MFALSE;
		usb_cardp->rx_deaggr_ctrl.aggr_mode = MLAN_USB_AGGR_MODE_NUM;
		usb_cardp->rx_deaggr_ctrl.aggr_align = MLAN_USB_RX_ALIGN_SIZE;
		usb_cardp->rx_deaggr_ctrl.aggr_max = MLAN_USB_RX_MAX_AGGR_NUM;
		usb_cardp->rx_deaggr_ctrl.aggr_tmo =
			MLAN_USB_RX_DEAGGR_TIMEOUT_USEC;
		usb_set_intfdata(intf, usb_cardp);

		card_type = woal_update_card_type(usb_cardp);
		if (!card_type) {
			PRINTM(MERROR,
			       "usb probe: woal_update_card_type() failed\n");
			goto error;
		}

		/* At this point wlan_add_card() will be called */
		if (!(woal_add_card(usb_cardp, &usb_cardp->udev->dev, &usb_ops,
				    card_type))) {
			PRINTM(MERROR, "%s: woal_add_card failed\n",
			       __FUNCTION__);
			goto error;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
		/* Note: From 2.6.34.2 onwards, remote wakeup is NOT enabled by
		 * default. So drivers wanting remote wakeup will have to enable
		 * this using -
		 * device_set_wakeup_enable(&udev->dev, 1);
		 * It has been observed that some cards having device attr =
		 * 0xa0 do not support remote wakeup. These cards come
		 * immediately out of the suspend when power/wakeup file is set
		 * to 'enabled'. To support all types of cards i.e. with/without
		 * remote wakeup, we are NOT setting the 'power/wakeup' file
		 * from here. Also in principle, we are not supposed to change
		 * the wakeup policy, which is purely a userspace decision.
		 */
		/* if (udev->actconfig->desc.bmAttributes &
		   USB_CONFIG_ATT_WAKEUP) intf->needs_remote_wakeup = 1; */
#endif
		usb_get_dev(udev);
		LEAVE();
		return 0;
	} else {
		PRINTM(MINFO, "Discard the Probe request\n");
		PRINTM(MINFO, "VID = 0x%X PID = 0x%X\n",
		       woal_cpu_to_le16(udev->descriptor.idVendor),
		       woal_cpu_to_le16(udev->descriptor.idProduct));
	}
error:
	kfree(usb_cardp);
	usb_cardp = NULL;
	LEAVE();
	return -ENXIO;
}

/**
 *  @brief Free resource and cleanup
 *
 *  @param intf		Pointer to usb_interface
 *
 *  @return 	   	N/A
 */
static void woal_usb_disconnect(struct usb_interface *intf)
{
	struct usb_card_rec *cardp = usb_get_intfdata(intf);
	moal_handle *phandle = NULL;
	ENTER();
	if (!cardp || !cardp->phandle) {
		PRINTM(MERROR, "Card or phandle is not valid\n");
		LEAVE();
		return;
	}
	phandle = (moal_handle *)cardp->phandle;

	/*
	 * Update Surprise removed to TRUE
	 *  Free all the URB's allocated
	 */
	phandle->surprise_removed = MTRUE;

	/* Card is removed and we can call wlan_remove_card */
	PRINTM(MINFO, "Call remove card\n");
	woal_remove_card(cardp);

	usb_set_intfdata(intf, NULL);
	usb_put_dev(interface_to_usbdev(intf));
	kfree(cardp);
	LEAVE();
	return;
}

/**
 *  @brief killall pending urbs
 *
 *  @param handle	  Pointer to moal_handle
 *
 *
 *  @return 	   	  N/A
 */
void woal_kill_urbs(moal_handle *handle)
{
	ENTER();
	handle->is_suspended = MTRUE;
	woal_usb_unlink_urb(handle->card);
	LEAVE();
}

/**
 *  @brief resubmit urbs
 *
 *  @param handle	  Pointer to moal_handle
 *
 *
 *  @return 	   	  N/A
 */
void woal_resubmit_urbs(moal_handle *handle)
{
	struct usb_card_rec *cardp = handle->card;

	ENTER();
	handle->is_suspended = MFALSE;

	if (!atomic_read(&cardp->rx_data_urb_pending)) {
		/* Submit multiple Rx data URBs */
		woal_usb_submit_rx_data_urbs(handle);
	}
	if (!atomic_read(&cardp->rx_cmd_urb_pending)) {
		cardp->rx_cmd.pmbuf =
			woal_alloc_mlan_buffer(handle, MLAN_RX_CMD_BUF_SIZE);
		if (cardp->rx_cmd.pmbuf)
			woal_usb_submit_rx_urb(&cardp->rx_cmd,
					       MLAN_RX_CMD_BUF_SIZE);
	}
	LEAVE();
}

#ifdef CONFIG_PM
/**
 *  @brief Handle suspend
 *
 *  @param intf		  Pointer to usb_interface
 *  @param message	  Pointer to pm_message_t structure
 *
 *  @return 	   	  MLAN_STATUS_SUCCESS
 */
static int woal_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_card_rec *cardp = usb_get_intfdata(intf);
	moal_handle *handle = NULL;
	int i;
	int ret = 0;

	ENTER();

	PRINTM(MCMND, "<--- Enter woal_usb_suspend --->\n");
	if (!cardp || !cardp->phandle) {
		PRINTM(MERROR, "Card or moal_handle structure is not valid\n");
		ret = 0;
		goto done;
	}
	handle = cardp->phandle;
	if (handle->is_suspended == MTRUE) {
		PRINTM(MWARN, "Device already suspended\n");
		ret = 0;
		goto done;
	}
#ifdef STA_SUPPORT
	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i] &&
		    (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_STA))
			woal_cancel_scan(handle->priv[i], MOAL_IOCTL_WAIT);
	}
#endif
	/* Enable Host Sleep */
	woal_enable_hs(woal_get_priv(handle, MLAN_BSS_ROLE_ANY));

	/* Indicate device suspended */
	/* The flag must be set here before the usb_kill_urb() calls.
	 * Reason: In the complete handlers, urb->status(= -ENOENT) and
	 * 'is_suspended' flag is used in combination to distinguish
	 * between a suspended state and a 'disconnect' one.
	 */
	handle->is_suspended = MTRUE;
	for (i = 0; i < handle->priv_num; i++)
		netif_carrier_off(handle->priv[i]->netdev);

	/* Unlink Rx cmd URB */
	if (atomic_read(&cardp->rx_cmd_urb_pending) && cardp->rx_cmd.urb) {
		usb_kill_urb(cardp->rx_cmd.urb);
	}
	/* Unlink Rx data URBs */
	if (atomic_read(&cardp->rx_data_urb_pending)) {
		for (i = 0; i < MVUSB_RX_DATA_URB; i++) {
			if (cardp->rx_data_list[i].urb) {
				usb_kill_urb(cardp->rx_data_list[i].urb);
				usb_init_urb(cardp->rx_data_list[i].urb);
			}
		}
	}

	/* Unlink Tx data URBs */
	for (i = 0; i < MVUSB_TX_HIGH_WMARK; i++) {
		if (cardp->tx_data_list[i].urb) {
			usb_kill_urb(cardp->tx_data_list[i].urb);
		}
	}
	/* Unlink Tx cmd URB */
	if (cardp->tx_cmd.urb) {
		usb_kill_urb(cardp->tx_cmd.urb);
	}

	handle->suspend_wait_q_woken = MTRUE;
	wake_up_interruptible(&handle->suspend_wait_q);

done:
	PRINTM(MCMND, "<--- Leave woal_usb_suspend --->\n");
	LEAVE();
	return ret;
}

/**
 *  @brief Handle resume
 *
 *  @param intf		  Pointer to usb_interface
 *
 *  @return 	   	  MLAN_STATUS_SUCCESS
 */
static int woal_usb_resume(struct usb_interface *intf)
{
	struct usb_card_rec *cardp = usb_get_intfdata(intf);
	moal_handle *handle = NULL;
	int i;
	int ret = 0;

	ENTER();

	PRINTM(MCMND, "<--- Enter woal_usb_resume --->\n");
	if (!cardp || !cardp->phandle) {
		PRINTM(MERROR, "Card or adapter structure is not valid\n");
		ret = 0;
		goto done;
	}
	handle = cardp->phandle;

	if (handle->is_suspended == MFALSE) {
		PRINTM(MWARN, "Device already resumed\n");
		ret = 0;
		goto done;
	}

	/* Indicate device resumed.
	 * The netdev queue will be resumed only after the urbs
	 * have been resubmitted */
	handle->is_suspended = MFALSE;

	if (!atomic_read(&cardp->rx_data_urb_pending)) {
		/* Submit multiple Rx data URBs */
		woal_usb_submit_rx_data_urbs(handle);
	}
	if (!atomic_read(&cardp->rx_cmd_urb_pending)) {
		cardp->rx_cmd.pmbuf =
			woal_alloc_mlan_buffer(handle, MLAN_RX_CMD_BUF_SIZE);
		if (cardp->rx_cmd.pmbuf)
			woal_usb_submit_rx_urb(&cardp->rx_cmd,
					       MLAN_RX_CMD_BUF_SIZE);
	}

	for (i = 0; i < handle->priv_num; i++)
		if (handle->priv[i]->media_connected == MTRUE)
			netif_carrier_on(handle->priv[i]->netdev);

	/* Disable Host Sleep */
	if (handle->hs_activated)
		woal_cancel_hs(woal_get_priv(handle, MLAN_BSS_ROLE_ANY),
			       MOAL_NO_WAIT);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
		/* Resume handler may be called due to remote wakeup,
		   force to exit suspend anyway */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	cardp->udev->autosuspend_disabled = 1;
#else
#ifdef CONFIG_PM_RUNTIME
	cardp->udev->dev.power.runtime_auto = 0;
#endif
#endif /* < 2.6.35 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
	cardp->udev->autoresume_disabled = 0;
#endif /* < 2.6.33 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
#ifdef CONFIG_PM_RUNTIME
	atomic_inc(&(cardp->udev)->dev.power.usage_count);
#endif
#endif /* >= 2.6.34 */
#endif /* >= 2.6.24 */

done:
	PRINTM(MCMND, "<--- Leave woal_usb_resume --->\n");
	LEAVE();
	return 0;
}
#endif /* CONFIG_PM */

/**
 *  @brief This function initialize the tx URBs
 *
 *  @param handle 	Pointer to moal_handle structure
 *
 *  @return 	   	MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_usb_tx_init(moal_handle *handle)
{
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;
	int i;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	cardp->tx_cmd.handle = handle;
	cardp->tx_cmd.ep = cardp->tx_cmd_ep;
	/* Allocate URB for command */
	cardp->tx_cmd.urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!cardp->tx_cmd.urb) {
		PRINTM(MERROR, "Tx command URB allocation failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto init_exit;
	}

	cardp->tx_data_ix = 0;
	for (i = 0; i < MVUSB_TX_HIGH_WMARK; i++) {
		cardp->tx_data_list[i].handle = handle;
		cardp->tx_data_list[i].ep = cardp->tx_data_ep;
		/* Allocate URB for data */
		cardp->tx_data_list[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!cardp->tx_data_list[i].urb) {
			PRINTM(MERROR, "Tx data URB allocation failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto init_exit;
		}
	}

init_exit:
	LEAVE();
	return ret;
}

/**
 *  @brief This function submits the rx data URBs
 *
 *  @param handle 	Pointer to moal_handle structure
 *
 *  @return 	   	MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_usb_submit_rx_data_urbs(moal_handle *handle)
{
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;
	int i;
	mlan_status ret = MLAN_STATUS_FAILURE;
	t_u32 buffer_len = MLAN_RX_DATA_BUF_SIZE;

	ENTER();

	if (cardp->rx_deaggr_ctrl.enable) {
		buffer_len = cardp->rx_deaggr_ctrl.aggr_max;
		if (cardp->rx_deaggr_ctrl.aggr_mode == MLAN_USB_AGGR_MODE_NUM) {
			buffer_len *= MAX(MLAN_USB_MAX_PKT_SIZE,
					  cardp->rx_deaggr_ctrl.aggr_align);
			buffer_len = MAX(buffer_len, MLAN_RX_DATA_BUF_SIZE);
		}
	}

	for (i = 0; i < MVUSB_RX_DATA_URB; i++) {
		/* Submit Rx data URB */
		if (!cardp->rx_data_list[i].pmbuf) {
			if (woal_usb_submit_rx_urb(&cardp->rx_data_list[i],
						   buffer_len))
				continue;
		}
		ret = MLAN_STATUS_SUCCESS;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function initialize the rx URBs and submit them
 *
 *  @param handle 	Pointer to moal_handle structure
 *
 *  @return 	   	MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_usb_rx_init(moal_handle *handle)
{
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;
	int i;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	cardp->rx_cmd.handle = handle;
	cardp->rx_cmd.ep = cardp->rx_cmd_ep;
	/* Allocate URB for command/event */
	cardp->rx_cmd.urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!cardp->rx_cmd.urb) {
		PRINTM(MERROR, "Rx command URB allocation failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto init_exit;
	}

	cardp->rx_cmd.pmbuf =
		woal_alloc_mlan_buffer(handle, MLAN_RX_CMD_BUF_SIZE);
	if (cardp->rx_cmd.pmbuf) {
		/* Submit Rx command URB */
		if (woal_usb_submit_rx_urb(&cardp->rx_cmd,
					   MLAN_RX_CMD_BUF_SIZE)) {
			ret = MLAN_STATUS_FAILURE;
			goto init_exit;
		}
	}
	for (i = 0; i < MVUSB_RX_DATA_URB; i++) {
		cardp->rx_data_list[i].handle = handle;
		cardp->rx_data_list[i].ep = cardp->rx_data_ep;
		/* Allocate URB for data */
		cardp->rx_data_list[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!cardp->rx_data_list[i].urb) {
			PRINTM(MERROR, "Rx data URB allocation failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto init_exit;
		}
	}
	ret = woal_usb_submit_rx_data_urbs(handle);
	if (ret) {
		PRINTM(MERROR, "Rx data URB submission failed\n");
		goto init_exit;
	}

init_exit:
	LEAVE();
	return ret;
}

/**
 *  @brief  This function downloads data blocks to device
 *
 *  @param handle	Pointer to moal_handle structure
 *  @param pmbuf	Pointer to mlan_buffer structure
 *  @param ep		Endpoint to send
 *  @param timeout 	Timeout value in milliseconds (if 0 the wait is forever)
 *
 *  @return 	   	MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_usb_write_data_sync(moal_handle *handle,
					    mlan_buffer *pmbuf, t_u32 endpoint,
					    t_u32 timeout)
{
	struct usb_card_rec *cardp = handle->card;
	t_u8 *data = (t_u8 *)(pmbuf->pbuf + pmbuf->data_offset);
	t_u8 ep = endpoint;
	t_u32 length = pmbuf->data_len;
	int actual_length;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int bulk_out_maxpktsize = 512;

	if (ep == cardp->tx_cmd_ep)
		bulk_out_maxpktsize = cardp->tx_cmd_maxpktsize;
	else if (ep == cardp->tx_data_ep)
		bulk_out_maxpktsize = cardp->tx_data_maxpktsize;

	if (length % bulk_out_maxpktsize == 0)
		length++;

	/* Send the data block */
	ret = usb_bulk_msg(cardp->udev, usb_sndbulkpipe(cardp->udev, ep),
			   (t_u8 *)data, length, &actual_length, timeout);
	if (ret) {
		PRINTM(MERROR, "usb_blk_msg for send failed, ret %d\n", ret);
		ret = MLAN_STATUS_FAILURE;
	}

	pmbuf->data_len = actual_length;
	DBG_HEXDUMP(MIF_D, "write sync", data, actual_length);

	return ret;
}

/**
 *  @brief  This function read data blocks to device
 *
 *  @param handle	Pointer to moal_handle structure
 *  @param pmbuf	Pointer to mlan_buffer structure
 *  @param ep		Endpoint to receive
 *  @param timeout 	Timeout value in milliseconds (if 0 the wait is forever)
 *
 *  @return 	   	MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_usb_read_data_sync(moal_handle *handle,
					   mlan_buffer *pmbuf, t_u32 endpoint,
					   t_u32 timeout)
{
	struct usb_card_rec *cardp = handle->card;
	t_u8 *data = (t_u8 *)(pmbuf->pbuf + pmbuf->data_offset);
	t_u8 ep = endpoint;
	t_u32 buf_len = pmbuf->data_len;
	int actual_length;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();
	/* Receive the data response */
	ret = usb_bulk_msg(cardp->udev, usb_rcvbulkpipe(cardp->udev, ep), data,
			   buf_len, &actual_length, timeout);
	if (ret) {
		PRINTM(MERROR, "usb_bulk_msg failed: %d\n", ret);
		ret = MLAN_STATUS_FAILURE;
	}
	pmbuf->data_len = actual_length;
	DBG_HEXDUMP(MIF_D, "read sync", data, actual_length);
	LEAVE();
	return ret;
}

/**
 *  @brief  This function downloads data/command packet to device
 *
 *  @param handle	Pointer to moal_handle structure
 *  @param pmbuf	Pointer to mlan_buffer structure
 *  @param ep		Endpoint to send
 *
 *  @return 	   	MLAN_STATUS_PENDING or MLAN_STATUS_FAILURE or
 * MLAN_STATUS_RESOURCE
 */
mlan_status woal_write_data_async(moal_handle *handle, mlan_buffer *pmbuf,
				  t_u8 ep)
{
	struct usb_card_rec *cardp = handle->card;
	urb_context *context = NULL;
	t_u8 *data = (t_u8 *)(pmbuf->pbuf + pmbuf->data_offset);
	struct urb *tx_urb = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 data_len = pmbuf->data_len;
	int bulk_out_maxpktsize = 512;

	ENTER();

	/* Check if device is removed */
	if (handle->surprise_removed) {
		PRINTM(MERROR, "Device removed\n");
		ret = MLAN_STATUS_FAILURE;
		goto tx_ret;
	}

	if ((ep == cardp->tx_data_ep) &&
	    (atomic_read(&cardp->tx_data_urb_pending) >= MVUSB_TX_HIGH_WMARK)) {
		ret = MLAN_STATUS_RESOURCE;
		goto tx_ret;
	}
	PRINTM(MINFO, "woal_write_data_async: ep=%d\n", ep);

	if (ep == cardp->tx_cmd_ep) {
		context = &cardp->tx_cmd;
		bulk_out_maxpktsize = cardp->tx_cmd_maxpktsize;
	} else {
		if (ep == cardp->tx_data_ep) {
			bulk_out_maxpktsize = cardp->tx_data_maxpktsize;
			if (cardp->tx_data_ix >= MVUSB_TX_HIGH_WMARK)
				cardp->tx_data_ix = 0;
			context = &cardp->tx_data_list[cardp->tx_data_ix++];
		}
	}

	if (!context) {
		PRINTM(MERROR, "Cannot allocate Tx urb_context\n");
		ret = MLAN_STATUS_FAILURE;
		goto tx_ret;
	}
	context->handle = handle;
	context->ep = ep;
	context->pmbuf = pmbuf;

	tx_urb = context->urb;

	if (data_len % bulk_out_maxpktsize == 0)
		data_len++;

	/*
	 * Use USB API usb_fill_bulk_urb() to set the
	 * configuration information of the Tx bulk URB
	 * and initialize the Tx callback
	 */

	if (ep == cardp->tx_cmd_ep &&
	    cardp->tx_cmd_ep_type == USB_ENDPOINT_XFER_INT) {
		usb_fill_int_urb(tx_urb, cardp->udev,
				 usb_sndintpipe(cardp->udev, ep), data,
				 data_len, woal_usb_tx_complete,
				 (void *)context, cardp->tx_cmd_interval);
	} else
		usb_fill_bulk_urb(tx_urb, cardp->udev,
				  usb_sndbulkpipe(cardp->udev, ep), data,
				  data_len, woal_usb_tx_complete,
				  (void *)context);
	/* We find on Ubuntu 12.10 this flag does not work */
	// tx_urb->transfer_flags |= URB_ZERO_PACKET;

	if (ep == cardp->tx_cmd_ep)
		atomic_inc(&cardp->tx_cmd_urb_pending);
	else if (ep == cardp->tx_data_ep)
		atomic_inc(&cardp->tx_data_urb_pending);
	if (usb_submit_urb(tx_urb, GFP_ATOMIC)) {
		/* Submit URB failure */
		PRINTM(MERROR, "Submit EP %d Tx URB failed: %d\n", ep, ret);
		if (ep == cardp->tx_cmd_ep)
			atomic_dec(&cardp->tx_cmd_urb_pending);
		else {
			if (ep == cardp->tx_data_ep) {
				atomic_dec(&cardp->tx_data_urb_pending);
				if (cardp->tx_data_ix)
					cardp->tx_data_ix--;
				else
					cardp->tx_data_ix = MVUSB_TX_HIGH_WMARK;
			}
		}
		ret = MLAN_STATUS_FAILURE;
	} else {
		if (ep == cardp->tx_data_ep &&
		    (atomic_read(&cardp->tx_data_urb_pending) ==
		     MVUSB_TX_HIGH_WMARK))
			ret = MLAN_STATUS_PRESOURCE;
		else
			ret = MLAN_STATUS_SUCCESS;
	}

tx_ret:

	if (!ret)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 *  @brief  This function register usb device and initialize parameter
 *
 *  @param handle	Pointer to moal_handle structure
 *
 *  @return 	   	MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_usb_register_dev(moal_handle *handle)
{
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	cardp->phandle = handle;
	LEAVE();
	return ret;
}

static void woal_usb_unregister_dev(moal_handle *handle)
{
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;
	PRINTM(MMSG, "USB: unregister device\n");
	woal_usb_free(cardp);
	cardp->phandle = NULL;
	return;
}

/**
 *  @brief This function registers driver.
 *
 *  @return 	 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_usb_bus_register(void)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	ENTER();

	if (skip_fwdnld) {
		woal_usb_driver.id_table = woal_usb_table_skip_fwdnld;
	}
	/*
	 * API registers the NXP USB driver
	 * to the USB system
	 */
	if (usb_register(&woal_usb_driver)) {
		PRINTM(MFATAL, "USB Driver Registration Failed \n");
		ret = MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function removes usb driver.
 *
 *  @return 	   	N/A
 */
void woal_usb_bus_unregister(void)
{
	ENTER();
	/* API unregisters the driver from USB subsystem */
	usb_deregister(&woal_usb_driver);
	LEAVE();
	return;
}

/**
 *  @brief This function check if this is second mac
 *
 *  @param handle   A pointer to moal_handle structure
 *  @return         MTRUE/MFALSE
 *
 */
static t_u8 woal_usb_is_second_mac(moal_handle *handle)
{
	return ((struct usb_card_rec *)(handle->card))->second_mac;
}

#ifdef CONFIG_USB_SUSPEND
/**
 *  @brief This function makes USB device to suspend.
 *
 *  @param handle  A pointer to moal_handle structure
 *
 *  @return             0 --success, otherwise fail
 */
int woal_enter_usb_suspend(moal_handle *handle)
{
	struct usb_device *udev = ((struct usb_card_rec *)(handle->card))->udev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
	struct usb_interface *intf =
		((struct usb_card_rec *)(handle->card))->intf;
#endif /* < 2.6.34 */
#endif /* >= 2.6.24 */

	ENTER();

	PRINTM(MIOCTL, "USB suspend ioctl (state %d)\n", udev->state);

	if (handle->is_suspended == MTRUE) {
		PRINTM(MERROR, "Device already suspended\n");
		LEAVE();
		return -EFAULT;
	}

	handle->suspend_wait_q_woken = MFALSE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
	/* Enter into USB suspend */
	usb_lock_device(udev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
	udev->autosuspend_delay = 0; /* Autosuspend delay in jiffies */
#else
	pm_runtime_set_autosuspend_delay(&udev->dev, 0); /* Autosuspend delay in
							    jiffies */
#endif /* < 2.6.38 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
	udev->autosuspend_disabled = 0; /* /sys/bus/usb/devices/.../power/level
					   < auto */
#endif /* < 2.6.34 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
	udev->autoresume_disabled = 0;
#endif /* < 2.6.33 */
	usb_unlock_device(udev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
	intf->pm_usage_cnt = 1;
#else
	atomic_set(&intf->pm_usage_cnt, 1);
#endif /* < 2.6.32 */
	usb_autopm_put_interface(intf);
#else
	usb_lock_device(udev);
	atomic_set(&udev->dev.power.usage_count, 1);
	usb_enable_autosuspend(udev);
	usb_unlock_device(udev);
#endif /* < 2.6.34 */
#endif /* >= 2.6.24 */

	/* Wait for suspend to complete */
	wait_event_interruptible(handle->suspend_wait_q,
				 handle->suspend_wait_q_woken);

	LEAVE();
	return 0;
}

/**
 *  @brief This function makes USB device to resume.
 *
 *  @param handle  A pointer to moal_handle structure
 *
 *  @return             0 --success, otherwise fail
 */
int woal_exit_usb_suspend(moal_handle *handle)
{
	struct usb_device *udev = ((struct usb_card_rec *)(handle->card))->udev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
	struct usb_interface *intf =
		((struct usb_card_rec *)(handle->card))->intf;
#endif /* < 2.6.34 */
#endif /* >= 2.6.24 */

	ENTER();

	PRINTM(MIOCTL, "USB resume ioctl (state %d)\n", udev->state);

	if (handle->is_suspended == MFALSE ||
	    udev->state != USB_STATE_SUSPENDED) {
		PRINTM(MERROR, "Device already resumed\n");
		LEAVE();
		return -EFAULT;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
	/* Exit from USB suspend */
	usb_lock_device(udev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
	udev->autosuspend_disabled = 1; /* /sys/bus/usb/devices/.../power/level
					   < on */
#endif /* < 2.6.34 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
	udev->autoresume_disabled = 0;
#endif /* < 2.6.33 */
	usb_unlock_device(udev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
	intf->pm_usage_cnt = 0;
#else
	atomic_set(&intf->pm_usage_cnt, 0);
#endif /* < 2.6.32 */
	usb_autopm_get_interface(intf);
#else
	usb_lock_device(udev);
	atomic_set(&udev->dev.power.usage_count, 0);
	usb_disable_autosuspend(udev);
	usb_unlock_device(udev);
#endif /* < 2.6.34 */
#endif /* >= 2.6.24 */

	LEAVE();
	return 0;
}
#endif /* CONFIG_USB_SUSPEND */

/**
 *  @brief This function will submit rx urb.
 *
 *  @param handle   Pointer to moal_handle
 *  @param ep       Endpoint to re-submit urb
 *
 *  @return 	   	N/A
 */
void woal_submit_rx_urb(moal_handle *handle, t_u8 ep)
{
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;

	ENTER();

	if ((ep == cardp->rx_cmd_ep) &&
	    (!atomic_read(&cardp->rx_cmd_urb_pending))) {
		woal_usb_submit_rx_urb(&cardp->rx_cmd, MLAN_RX_CMD_BUF_SIZE);
	}

	LEAVE();
	return;
}

/**
 *  @brief This function dump firmware memory to file
 *
 *  @param phandle   A pointer to moal_handle
 *
 *  @return         N/A
 */
void woal_usb_dump_fw_info(moal_handle *phandle)
{
	moal_private *priv = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *pcfg_misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	priv = woal_get_priv(phandle, MLAN_BSS_ROLE_ANY);
	if (!priv) {
		PRINTM(MERROR, "woal_dump_firmware_info get priv is NULL!\n");
		goto done;
	}
	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MERROR, "woal_dump_firmware_info alloc req fail!\n");
		goto done;
	}

	/* Fill request buffer */
	pcfg_misc = (mlan_ds_misc_cfg *)req->pbuf;
	pcfg_misc->sub_command = MLAN_OID_MISC_FW_DUMP_EVENT;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

	if (status != MLAN_STATUS_PENDING)
		kfree(req);

done:
	LEAVE();
	return;
}

static mlan_status woal_usb_get_fw_name(moal_handle *handle)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
#if defined(USB8997) || defined(USB9098) || defined(USB9097) || defined(USB8978)
	t_u32 revision_id = 0;
	t_u32 strap = 0;
#endif
	struct usb_card_rec *cardp = (struct usb_card_rec *)handle->card;
#if defined(USB9098)
	moal_handle *ref_handle = NULL;
#endif

	ENTER();
	if (handle->params.fw_name)
		goto done;
	if (cardp->boot_state == USB_FW_READY)
		goto done;
#if defined(USB8997) || defined(USB9098) || defined(USB9097) || defined(USB8978)
	ret = woal_check_chip_revision(handle, &revision_id, &strap);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MFATAL, "Chip revision check failure!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	PRINTM(MCMND, "revision=0x%x, strap=0x%x\n", revision_id, strap);
#endif

#ifdef USB8997
	if (IS_USB8997(handle->card_type)) {
		if (strap == CARD_TYPE_USB_UART)
			strcpy(handle->card_info->fw_name,
			       USBUART8997_DEFAULT_COMBO_FW_NAME);
		else if (strap != 0)
			strcpy(handle->card_info->fw_name,
			       USBUSB8997_DEFAULT_COMBO_FW_NAME);
	}
#endif

#ifdef USB8978
	if (IS_USB8978(handle->card_type)) {
		if (strap == CARD_TYPE_USB_UART)
			strcpy(handle->card_info->fw_name,
			       USBUART8978_DEFAULT_COMBO_FW_NAME);
		else if (strap != 0)
			strcpy(handle->card_info->fw_name,
			       USBUSB8978_DEFAULT_COMBO_FW_NAME);
	}
#endif

#ifdef USB9098
	if (IS_USB9098(handle->card_type)) {
		if (cardp->second_mac) {
			ref_handle = (moal_handle *)handle->pref_mac;
			if (ref_handle) {
				strcpy(handle->card_info->fw_name,
				       ref_handle->card_info->fw_name);
				strcpy(handle->card_info->fw_name_wlan,
				       ref_handle->card_info->fw_name_wlan);
			}
			goto done;
		}
		switch (revision_id) {
		case USB9098_Z1Z2:
			if (strap != 0) {
				if (strap == CARD_TYPE_USB_UART)
					strcpy(handle->card_info->fw_name,
					       USBUART9098_DEFAULT_COMBO_FW_NAME);
				else
					strcpy(handle->card_info->fw_name,
					       USBUSB9098_DEFAULT_COMBO_FW_NAME);
			}
			strcpy(handle->card_info->fw_name_wlan,
			       USB9098_DEFAULT_WLAN_FW_NAME);
			break;
		case USB9098_A0:
		case USB9098_A1:
		case USB9098_A2:
			if (strap != 0) {
				if (strap == CARD_TYPE_USB_UART)
					strcpy(handle->card_info->fw_name,
					       USBUART9098_COMBO_V1_FW_NAME);
				else
					strcpy(handle->card_info->fw_name,
					       USBUSB9098_COMBO_V1_FW_NAME);
			}
			strcpy(handle->card_info->fw_name_wlan,
			       USB9098_WLAN_V1_FW_NAME);
			break;
		}
	}
#endif
#ifdef USB9097
	if (IS_USB9097(handle->card_type)) {
		switch (revision_id) {
		case USB9097_B0:
		case USB9097_B1:
			if (strap != 0) {
				if (strap == CARD_TYPE_USB_UART)
					strcpy(handle->card_info->fw_name,
					       USBUART9097_COMBO_V1_FW_NAME);
				else
					strcpy(handle->card_info->fw_name,
					       USBUSB9097_COMBO_V1_FW_NAME);
			}
			strcpy(handle->card_info->fw_name_wlan,
			       USB9097_WLAN_V1_FW_NAME);
			break;
		}
	}
#endif
done:
	PRINTM(MCMND, "combo fw:%s wlan fw:%s \n", handle->card_info->fw_name,
	       handle->card_info->fw_name_wlan);
	LEAVE();
	return ret;
}

static moal_if_ops usb_ops = {
	.register_dev = woal_usb_register_dev,
	.unregister_dev = woal_usb_unregister_dev,
	.read_data_sync = woal_usb_read_data_sync,
	.write_data_sync = woal_usb_write_data_sync,
	.get_fw_name = woal_usb_get_fw_name,
	.dump_fw_info = woal_usb_dump_fw_info,
	.is_second_mac = woal_usb_is_second_mac,
};
