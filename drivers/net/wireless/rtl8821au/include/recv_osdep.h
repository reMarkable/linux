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
#ifndef __RECV_OSDEP_H_
#define __RECV_OSDEP_H_


extern int _rtw_init_recv_priv(struct recv_priv *precvpriv, struct rtl_priv *rtlpriv);
extern void _rtw_free_recv_priv (struct recv_priv *precvpriv);


extern int32_t  rtw_recv_entry(struct recv_frame *precv_frame);
extern int rtw_recv_indicatepkt(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);
extern void rtw_recv_returnpacket(struct  net_device *cnxt, struct sk_buff *preturnedpkt);

extern void rtw_handle_tkip_mic_err(struct rtl_priv *rtlpriv,u8 bgroup);


int	rtw_init_recv_priv(struct recv_priv *precvpriv, struct rtl_priv *rtlpriv);
void rtw_free_recv_priv (struct recv_priv *precvpriv);

int rtw_os_alloc_recvframe(struct rtl_priv *rtlpriv, struct recv_frame *precvframe, u8 *pdata, struct sk_buff *pskb);
void rtw_os_free_recvframe(struct recv_frame *precvframe);


int rtw_os_recvbuf_resource_alloc(struct rtl_priv *rtlpriv, struct recv_buf *precvbuf);
int rtw_os_recvbuf_resource_free(struct rtl_priv *rtlpriv, struct recv_buf *precvbuf);

struct sk_buff *rtw_os_alloc_msdu_pkt(struct recv_frame *prframe, u16 nSubframe_Length, u8 *pdata);
void rtw_os_recv_indicate_pkt(struct rtl_priv *rtlpriv, struct sk_buff *pkt, struct rx_pkt_attrib *pattrib);

void rtw_init_recv_timer(struct recv_reorder_ctrl *preorder_ctrl);


#endif //

