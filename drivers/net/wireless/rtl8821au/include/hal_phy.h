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
#ifndef __HAL_PHY_H__
#define __HAL_PHY_H__


#define	RF6052_MAX_TX_PWR			0x3F
#define	RF6052_MAX_REG_88E			0xFF
#define	RF6052_MAX_REG_92C			0x7F

#define	RF6052_MAX_REG	\
		(RF6052_MAX_REG_88E > RF6052_MAX_REG_92C) ? RF6052_MAX_REG_88E: RF6052_MAX_REG_92C

#define GET_RF6052_REAL_MAX_REG(_Adapter)	\
		IS_HARDWARE_TYPE_8188E(_Adapter) ? RF6052_MAX_REG_88E : RF6052_MAX_REG_92C

#define	RF6052_MAX_PATH				2

/*--------------------------Define Parameters-------------------------------*/

#define	TX_1S			0
#define	TX_2S			1
#define	TX_3S			2
#define	TX_4S			3

#define	RF_PATH_MAX_92C_88E 		2
#define	RF_PATH_MAX_90_8812		4	//Max RF number 90 support

typedef enum _ANTENNA_PATH{
       ANTENNA_NONE 	= 0,
	ANTENNA_D		= 1,
	ANTENNA_C		= 2,
	ANTENNA_CD	= 3,
	ANTENNA_B		= 4,
	ANTENNA_BD	= 5,
	ANTENNA_BC	= 6,
	ANTENNA_BCD	= 7,
	ANTENNA_A		= 8,
	ANTENNA_AD	= 9,
	ANTENNA_AC	= 10,
	ANTENNA_ACD	= 11,
	ANTENNA_AB	= 12,
	ANTENNA_ABD	= 13,
	ANTENNA_ABC	= 14,
	ANTENNA_ABCD	= 15
} ANTENNA_PATH;

typedef enum _RF_CONTENT{
	radioa_txt = 0x1000,
	radiob_txt = 0x1001,
	radioc_txt = 0x1002,
	radiod_txt = 0x1003
} RF_CONTENT;

typedef enum _BaseBand_Config_Type{
	BaseBand_Config_PHY_REG = 0,			//Radio Path A
	BaseBand_Config_AGC_TAB = 1,			//Radio Path B
	BaseBand_Config_AGC_TAB_2G = 2,
	BaseBand_Config_AGC_TAB_5G = 3,
	BaseBand_Config_PHY_REG_PG
}BaseBand_Config_Type, *PBaseBand_Config_Type;

typedef enum _HW_BLOCK{
	HW_BLOCK_MAC = 0,
	HW_BLOCK_PHY0 = 1,
	HW_BLOCK_PHY1 = 2,
	HW_BLOCK_RF = 3,
	HW_BLOCK_MAXIMUM = 4, // Never use this
}HW_BLOCK_E, *PHW_BLOCK_E;

typedef enum _SwChnlCmdID{
	CmdID_End,
	CmdID_SetTxPowerLevel,
	CmdID_BBRegWrite10,
	CmdID_WritePortUlong,
	CmdID_WritePortu16,
	CmdID_WritePortUchar,
	CmdID_RF_WriteReg,
}SwChnlCmdID;

typedef struct _SwChnlCmd{
	SwChnlCmdID	CmdID;
	u32				Para1;
	u32				Para2;
	u32				msDelay;
}SwChnlCmd;

typedef struct _R_ANTENNA_SELECT_OFDM{
	u32			r_tx_antenna:4;
	u32			r_ant_l:4;
	u32			r_ant_non_ht:4;
	u32			r_ant_ht1:4;
	u32			r_ant_ht2:4;
	u32			r_ant_ht_s1:4;
	u32			r_ant_non_ht_s1:4;
	u32			OFDM_TXSC:2;
	u32			Reserved:2;
}R_ANTENNA_SELECT_OFDM;

typedef struct _R_ANTENNA_SELECT_CCK{
	uint8_t			r_cckrx_enable_2:2;
	uint8_t			r_cckrx_enable:2;
	uint8_t			r_ccktx_enable:4;
}R_ANTENNA_SELECT_CCK;

typedef struct RF_Shadow_Compare_Map {
	// Shadow register value
	u32		Value;
	// Compare or not flag
	uint8_t		Compare;
	// Record If it had ever modified unpredicted
	uint8_t		ErrorOrNot;
	// Recorver Flag
	uint8_t		Recorver;
	//
	uint8_t		Driver_Write;
}RF_SHADOW_T;

/*--------------------------Exported Function prototype---------------------*/

u32 PHY_RFShadowRead(struct rtl_priv *rtlpriv, uint8_t eRFPath, u32 Offset);

void PHY_RFShadowWrite(struct rtl_priv *rtlpriv, uint8_t eRFPath,
	u32 Offset, u32	Data);

bool PHY_RFShadowCompare(struct rtl_priv *rtlpriv, uint8_t eRFPath, u32	Offset);

void PHY_RFShadowRecorver(struct rtl_priv *rtlpriv, uint8_t eRFPath, u32 Offset);

void PHY_RFShadowCompareAll(struct rtl_priv *rtlpriv);

void PHY_RFShadowRecorverAll(struct rtl_priv *rtlpriv);

void PHY_RFShadowCompareFlagSet(struct rtl_priv *rtlpriv, uint8_t eRFPath,
	u32 Offset, uint8_t Type);

void PHY_RFShadowRecorverFlagSet(struct rtl_priv *rtlpriv, uint8_t eRFPath,
	u32 Offset, uint8_t Type);

void PHY_RFShadowCompareFlagSetAll(struct rtl_priv *rtlpriv);

void PHY_RFShadowRecorverFlagSetAll(struct rtl_priv *rtlpriv);

void PHY_RFShadowRefresh(struct rtl_priv *rtlpriv);

#endif //__HAL_COMMON_H__

