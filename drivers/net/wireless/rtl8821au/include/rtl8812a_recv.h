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
#ifndef __RTL8812A_RECV_H__
#define __RTL8812A_RECV_H__

#define MAX_RECVBUF_SZ (24576) // 24k

// Rx smooth factor
#define Rx_Smooth_Factor (20)


#define INTERRUPT_MSG_FORMAT_LEN 60
int32_t rtl8812au_init_recv_priv(struct rtl_priv *rtlpriv);
void rtl8812au_free_recv_priv(struct rtl_priv *rtlpriv);
void rtl8812au_recv_hdl(struct rtl_priv *rtlpriv, struct recv_buf *precvbuf);
void rtl8812au_recv_tasklet(void *priv);

#endif

