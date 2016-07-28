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
#define _RTL8812A_CMD_C_

#include <drv_types.h>
#include <rtl8812a_hal.h>
#include <../rtl8821au/trx.h>
#include <../rtl8821au/fw.h>







void rtl8812_Add_RateATid(struct rtl_priv *rtlpriv, uint32_t bitmap, uint8_t * arg, uint8_t rssi_level)
{
	uint8_t	macid;

	macid = arg[0];

	if(rssi_level != DM_RATR_STA_INIT)
		bitmap = ODM_Get_Rate_Bitmap(rtlpriv, macid, bitmap, rssi_level);

	rtl8812_set_raid_cmd(rtlpriv, bitmap, arg);
}



