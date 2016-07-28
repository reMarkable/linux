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
#ifndef __INC_HAL8812PHYCFG_H__
#define __INC_HAL8812PHYCFG_H__


/*--------------------------Define Parameters-------------------------------*/
#define LOOP_LIMIT				5
#define MAX_STALL_TIME			50		//us
#define AntennaDiversityValue	0x80	//(rtlpriv->bSoftwareAntennaDiversity ? 0x00:0x80)
#define MAX_TXPWR_IDX_NMODE_92S	63
#define Reset_Cnt_Limit			3


#define MAX_AGGR_NUM	0x07


/*--------------------------Define Parameters-------------------------------*/

/*------------------------------Define structure----------------------------*/


/* BB/RF related */

/*------------------------------Define structure----------------------------*/


/*------------------------Export global variable----------------------------*/
/*------------------------Export global variable----------------------------*/


/*------------------------Export Marco Definition---------------------------*/
/*------------------------Export Marco Definition---------------------------*/


/*--------------------------Exported Function prototype---------------------*/
//
// BB and RF register read/write
//
//
// Initialization related function
//
/* MAC/BB/RF HAL config */

/* RF config */

//
// BB TX Power R/W
//

u32 PHY_GetTxBBSwing_8812A(struct rtl_priv *rtlpriv, enum band_type Band, uint8_t RFPath);

//
// Switch bandwidth for 8192S
//
void PHY_SetBWMode8812(struct rtl_priv *rtlpriv, enum CHANNEL_WIDTH Bandwidth,
	uint8_t Offset);

//
// channel switch related funciton
//
void PHY_SwChnl8812(struct rtl_priv *rtlpriv, uint8_t channel);

void PHY_SetSwChnlBWMode8812(struct rtl_priv *rtlpriv, uint8_t channel,
	enum CHANNEL_WIDTH Bandwidth, uint8_t Offset40, uint8_t	Offset80);

//
// BB/MAC/RF other monitor API
//


/*--------------------------Exported Function prototype---------------------*/
#endif	// __INC_HAL8192CPHYCFG_H

