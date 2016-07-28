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
#ifndef __HAL_COMMON_H__
#define __HAL_COMMON_H__

#include <../rtl8821au/def.h>
#include "hal_phy.h"
#include "hal_phy_reg.h"
#include "hal_com_reg.h"
#include "hal_com_phycfg.h"


enum{
	UP_LINK,
	DOWN_LINK,
};
typedef enum _RT_MEDIA_STATUS {
	RT_MEDIA_DISCONNECT = 0,
	RT_MEDIA_CONNECT       = 1
} RT_MEDIA_STATUS;

#define MAX_DLFW_PAGE_SIZE			4096	// @ page : 4k bytes

// BK, BE, VI, VO, HCCA, MANAGEMENT, COMMAND, HIGH, BEACON.
//#define MAX_TX_QUEUE		9

#define TX_SELE_HQ			BIT(0)		// High Queue
#define TX_SELE_LQ			BIT(1)		// Low Queue
#define TX_SELE_NQ			BIT(2)		// Normal Queue
#define TX_SELE_EQ			BIT(3)		// Extern Queue

#define PageNum_128(_Len)		(u32)(((_Len)>>7) + ((_Len)&0x7F ? 1:0))
#define PageNum_256(_Len)		(u32)(((_Len)>>8) + ((_Len)&0xFF ? 1:0))
#define PageNum_512(_Len)		(u32)(((_Len)>>9) + ((_Len)&0x1FF ? 1:0))
#define PageNum(_Len, _Size)		(u32)(((_Len)/(_Size)) + ((_Len)&((_Size) - 1) ? 1:0))


uint8_t	//return the final channel plan decision
hal_com_get_channel_plan(
	struct rtl_priv *rtlpriv,
	uint8_t			hw_channel_plan,	//channel plan from HW (efuse/eeprom)
	uint8_t			sw_channel_plan,	//channel plan from SW (registry/module param)
	uint8_t			def_channel_plan,	//channel plan used when the former two is invalid
	bool			AutoLoadFail
	);

bool HAL_IsLegalChannel(struct rtl_priv *rtlpriv, u32 Channel);

void	HalSetBrateCfg(
	struct rtl_priv *	rtlpriv,
	uint8_t			*mBratesOS,
	u16			*pBrateCfg);


uint8_t rtw_hal_networktype_to_raid(struct rtl_priv *rtlpriv,unsigned char network_type);
uint8_t rtw_get_mgntframe_raid(struct rtl_priv *rtlpriv,unsigned char network_type);
#endif //__HAL_COMMON_H__

