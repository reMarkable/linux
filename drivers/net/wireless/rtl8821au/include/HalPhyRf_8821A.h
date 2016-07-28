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

#ifndef __HAL_PHY_RF_8821A_H__
#define __HAL_PHY_RF_8821A_H__

/*--------------------------Define Parameters-------------------------------*/
#define	IQK_DELAY_TIME_8821A		10		//ms
#define	index_mapping_NUM_8821A	15
#define AVG_THERMAL_NUM_8821A	4
#define RF_T_METER_8821A 		0x42

//1 7.	IQK

void PHY_SetRFPathSwitch_8821A(struct rtl_priv *rtlpriv, bool bMain);


#endif	// #ifndef __HAL_PHY_RF_8821A_H__

