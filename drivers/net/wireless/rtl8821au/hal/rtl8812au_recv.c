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
#define _RTL8812AU_RECV_C_

#include <drv_types.h>
#include <rtl8812a_hal.h>

int	rtl8812au_init_recv_priv(struct rtl_priv *rtlpriv)
{
	struct recv_priv	*precvpriv = &rtlpriv->recvpriv;
	int	i, res = _SUCCESS;
	struct recv_buf *precvbuf;

	tasklet_init(&precvpriv->recv_tasklet,
	     (void(*)(unsigned long))rtl8812au_recv_tasklet,
	     (unsigned long)rtlpriv);

	/* init recv_buf */
	_rtw_init_queue(&precvpriv->free_recv_buf_queue);

	precvpriv->pallocated_recv_buf = rtw_zmalloc(NR_RECVBUFF * sizeof(struct recv_buf) + 4);
	if (precvpriv->pallocated_recv_buf == NULL) {
		res = _FAIL;
		goto exit;
	}
	memset(precvpriv->pallocated_recv_buf, 0, NR_RECVBUFF * sizeof(struct recv_buf) + 4);

	precvpriv->precv_buf = (uint8_t *) N_BYTE_ALIGMENT((__kernel_size_t)(precvpriv->pallocated_recv_buf), 4);
	/*
	 * precvpriv->precv_buf = precvpriv->pallocated_recv_buf + 4 -
	 * 	((uint) (precvpriv->pallocated_recv_buf) &(4-1));
	*/


	precvbuf = (struct recv_buf *) precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF; i++) {
		INIT_LIST_HEAD(&precvbuf->list);

		spin_lock_init(&precvbuf->recvbuf_lock);

		precvbuf->alloc_sz = MAX_RECVBUF_SZ;

		res = rtw_os_recvbuf_resource_alloc(rtlpriv, precvbuf);
		if (res == _FAIL)
			break;

		precvbuf->ref_cnt = 0;
		precvbuf->rtlpriv = rtlpriv;

		/* list_add_tail(&precvbuf->list, &(precvpriv->free_recv_buf_queue.queue)); */

		precvbuf++;

	}

	precvpriv->free_recv_buf_queue_cnt = NR_RECVBUFF;

	skb_queue_head_init(&precvpriv->rx_skb_queue);

	{
		int i;
		__kernel_size_t tmpaddr = 0;
		__kernel_size_t alignment = 0;
		struct sk_buff *skb = NULL;

		skb_queue_head_init(&precvpriv->free_recv_skb_queue);

		for (i = 0; i < NR_PREALLOC_RECV_SKB; i++) {

			skb = __netdev_alloc_skb(rtlpriv->ndev, MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ, GFP_KERNEL);

			if (skb) {
				skb->dev = rtlpriv->ndev;

				tmpaddr = (__kernel_size_t) skb->data;
				alignment = tmpaddr & (RECVBUFF_ALIGN_SZ-1);
				skb_reserve(skb, (RECVBUFF_ALIGN_SZ - alignment));
				skb_queue_tail(&precvpriv->free_recv_skb_queue, skb);
			}

		}
	}

exit:
	return res;
}

void rtl8812au_free_recv_priv (struct rtl_priv *rtlpriv)
{
	int	i;
	struct recv_buf	*precvbuf;
	struct recv_priv	*precvpriv = &rtlpriv->recvpriv;

	precvbuf = (struct recv_buf *)precvpriv->precv_buf;

	for (i = 0; i < NR_RECVBUFF ; i++) {
		rtw_os_recvbuf_resource_free(rtlpriv, precvbuf);
		precvbuf++;
	}

	if (precvpriv->pallocated_recv_buf)
		rtw_mfree(precvpriv->pallocated_recv_buf);

	if (skb_queue_len(&precvpriv->rx_skb_queue)) {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "rx_skb_queue not empty\n");
	}

	skb_queue_purge(&precvpriv->rx_skb_queue);

	if (skb_queue_len(&precvpriv->free_recv_skb_queue)) {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "free_recv_skb_queue not empty, %d\n", skb_queue_len(&precvpriv->free_recv_skb_queue));
	}

	skb_queue_purge(&precvpriv->free_recv_skb_queue);
}


