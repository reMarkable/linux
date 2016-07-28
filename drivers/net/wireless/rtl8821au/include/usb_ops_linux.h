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
#ifndef __USB_OPS_LINUX_H__
#define __USB_OPS_LINUX_H__

#define VENDOR_CMD_MAX_DATA_LEN	254

#define RTW_USB_CONTROL_MSG_TIMEOUT_TEST	10//ms
#define RTW_USB_CONTROL_MSG_TIMEOUT	500//ms

#define RECV_BULK_IN_ADDR		0x80//assign by drv,not real address


#if defined(CONFIG_VENDOR_REQ_RETRY) && defined(CONFIG_USB_VENDOR_REQ_MUTEX)
/* vendor req retry should be in the situation when each vendor req is atomically submitted from others */
#define MAX_USBCTRL_VENDORREQ_TIMES	10
#else
#define MAX_USBCTRL_VENDORREQ_TIMES	1
#endif

#define RTW_USB_BULKOUT_TIMEOUT	5000//ms

#endif

