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

#ifndef __OSDEP_INTF_H_
#define __OSDEP_INTF_H_

u32 rtw_start_drv_threads(struct rtl_priv *rtlpriv);
void rtw_stop_drv_threads (struct rtl_priv *rtlpriv);

int rtw_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);

u16 rtw_recv_select_queue(struct sk_buff *skb);

void rtw_ips_dev_unload(struct rtl_priv *rtlpriv);
int rtw_ips_pwr_up(struct rtl_priv *rtlpriv);
void rtw_ips_pwr_down(struct rtl_priv *rtlpriv);

void rtw_ndev_destructor(struct  net_device *ndev);

#endif	//_OSDEP_INTF_H_

