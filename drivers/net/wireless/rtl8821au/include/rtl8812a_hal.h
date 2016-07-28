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
#ifndef __RTL8812A_HAL_H__
#define __RTL8812A_HAL_H__

//#include "hal_com.h"
#include <odm_precomp.h>

//include HAL Related header after HAL Related compiling flags
#include "rtl8812a_spec.h"
#include "rtl8812a_rf.h"
#include "rtl8812a_dm.h"
#include "rtl8812a_recv.h"
#include "rtl8812a_xmit.h"
#include "rtl8812a_cmd.h"
#include "Hal8812PwrSeq.h"
#include "Hal8821APwrSeq.h" //for 8821A/8811A
#include "Hal8812PhyReg.h"
#include "Hal8812PhyCfg.h"
#ifdef DBG_CONFIG_ERROR_DETECT
#include "rtl8812a_sreset.h"
#endif

#define Rtl8812_NIC_PWR_ON_FLOW				rtl8812_power_on_flow
#define Rtl8812_NIC_RF_OFF_FLOW				rtl8812_radio_off_flow
#define Rtl8812_NIC_DISABLE_FLOW				rtl8812_card_disable_flow
#define Rtl8812_NIC_ENABLE_FLOW				rtl8812_card_enable_flow
#define Rtl8812_NIC_SUSPEND_FLOW				rtl8812_suspend_flow
#define Rtl8812_NIC_RESUME_FLOW				rtl8812_resume_flow
#define Rtl8812_NIC_PDN_FLOW					rtl8812_hwpdn_flow
#define Rtl8812_NIC_LPS_ENTER_FLOW			rtl8812_enter_lps_flow
#define Rtl8812_NIC_LPS_LEAVE_FLOW				rtl8812_leave_lps_flow

//---------------------------------------------------------------------
//		RTL8821 Power Configuration CMDs for PCIe interface
//---------------------------------------------------------------------
#define Rtl8821A_NIC_PWR_ON_FLOW				rtl8821A_power_on_flow
#define Rtl8821A_NIC_RF_OFF_FLOW				rtl8821A_radio_off_flow
#define Rtl8821A_NIC_DISABLE_FLOW				rtl8821A_card_disable_flow
#define Rtl8821A_NIC_ENABLE_FLOW				rtl8821A_card_enable_flow
#define Rtl8821A_NIC_SUSPEND_FLOW				rtl8821A_suspend_flow
#define Rtl8821A_NIC_RESUME_FLOW				rtl8821A_resume_flow
#define Rtl8821A_NIC_PDN_FLOW					rtl8821A_hwpdn_flow
#define Rtl8821A_NIC_LPS_ENTER_FLOW			rtl8821A_enter_lps_flow
#define Rtl8821A_NIC_LPS_LEAVE_FLOW			rtl8821A_leave_lps_flow


#if 1 // download firmware related data structure

#endif // download firmware related data structure


#define DRIVER_EARLY_INT_TIME_8812		0x05
#define BCN_DMA_ATIME_INT_TIME_8812		0x02

#define TX_TOTAL_PAGE_NUMBER_8812		0xF8

#define TX_PAGE_BOUNDARY_8812			(TX_TOTAL_PAGE_NUMBER_8812 + 1)
#define TX_PAGE_LOAD_FW_BOUNDARY_8812		0x47 //0xA5
#define TX_PAGE_BOUNDARY_WOWLAN_8812		0xE0

// For Normal Chip Setting
// (HPQ + LPQ + NPQ + PUBQ) shall be TX_TOTAL_PAGE_NUMBER_92C
#define NORMAL_PAGE_NUM_PUBQ_8812			0xD8
#define NORMAL_PAGE_NUM_LPQ_8812				0x10
#define NORMAL_PAGE_NUM_HPQ_8812			0x10
#define NORMAL_PAGE_NUM_NPQ_8812				0x00

//Note: For WMM Normal Chip Setting ,modify later
#define WMM_NORMAL_TX_TOTAL_PAGE_NUMBER_8812	0xFB
#define WMM_NORMAL_TX_PAGE_BOUNDARY_8812		(WMM_NORMAL_TX_TOTAL_PAGE_NUMBER_8812 + 1)

#define WMM_NORMAL_PAGE_NUM_PUBQ_8812		0x8B
#define WMM_NORMAL_PAGE_NUM_HPQ_8812		0x30
#define WMM_NORMAL_PAGE_NUM_LPQ_8812		0x20
#define WMM_NORMAL_PAGE_NUM_NPQ_8812		0x20


// for 8821A
// TX 64K, RX 16K, Page size 256B for TX, 128B for RX
#define PAGE_SIZE_TX_8821A					256
#define PAGE_SIZE_RX_8821A					128

// For Normal Chip Setting
#define TX_TOTAL_PAGE_NUMBER_8821			0xF7
#define TX_PAGE_BOUNDARY_8821				(TX_TOTAL_PAGE_NUMBER_8821 + 1)
//#define TX_PAGE_BOUNDARY_WOWLAN_8821		0xE0

// (HPQ + LPQ + NPQ + PUBQ) shall be TX_TOTAL_PAGE_NUMBER
#define NORMAL_PAGE_NUM_PUBQ_8821			0xE7
#define NORMAL_PAGE_NUM_LPQ_8821			0x08
#define NORMAL_PAGE_NUM_HPQ_8821			0x08
#define NORMAL_PAGE_NUM_NPQ_8821			0x00

// For WMM Normal Chip Setting
#define WMM_NORMAL_TX_TOTAL_PAGE_NUMBER_8821	0xFB
#define WMM_NORMAL_TX_PAGE_BOUNDARY_8821		(WMM_NORMAL_TX_TOTAL_PAGE_NUMBER_8821 + 1)

#define WMM_NORMAL_PAGE_NUM_PUBQ_8821		0x8B
#define WMM_NORMAL_PAGE_NUM_HPQ_8821		0x30
#define WMM_NORMAL_PAGE_NUM_LPQ_8821		0x20
#define WMM_NORMAL_PAGE_NUM_NPQ_8821		0x20


#define	EFUSE_HIDDEN_812AU					0
#define	EFUSE_HIDDEN_812AU_VS				1
#define	EFUSE_HIDDEN_812AU_VL				2
#define	EFUSE_HIDDEN_812AU_VN				3

#define EFUSE_REAL_CONTENT_LEN_JAGUAR		512
#define HWSET_MAX_SIZE_JAGUAR					512

#define EFUSE_MAX_BANK_8812A					2
#define EFUSE_MAP_LEN_JAGUAR					512
#define EFUSE_MAX_SECTION_JAGUAR				64
#define EFUSE_MAX_WORD_UNIT_JAGUAR			4
#define EFUSE_IC_ID_OFFSET_JAGUAR				506	//For some inferiority IC purpose. added by Roger, 2009.09.02.
#define AVAILABLE_EFUSE_ADDR_8812(addr) 	(addr < EFUSE_REAL_CONTENT_LEN_JAGUAR)
// <Roger_Notes> To prevent out of boundary programming case, leave 1byte and program full section
// 9bytes + 1byt + 5bytes and pre 1byte.
// For worst case:
// | 2byte|----8bytes----|1byte|--7bytes--| //92D
#define EFUSE_OOB_PROTECT_BYTES_JAGUAR		18 	// PG data exclude header, dummy 7 bytes frome CP test and reserved 1byte.
#define EFUSE_PROTECT_BYTES_BANK_JAGUAR		16

//#define RT_IS_FUNC_DISABLED(rtlpriv, __FuncBits) ( (rtlpriv)->DisabledFunctions & (__FuncBits) )

// rtl8812_hal_init.c
void	InitializeFirmwareVars8812(struct rtl_priv *rtlpriv);


// EFuse
void UpdateHalRAMask8812A(struct rtl_priv *rtlpriv, uint32_t mac_id, uint8_t rssi_level);
// register
void SetBcnCtrlReg(struct rtl_priv *rtlpriv, uint8_t SetBits, uint8_t ClearBits);
void rtw_set_sta_info(struct rtl_priv *rtlpriv, struct sta_info *psta, bool bSet);

#endif //__RTL8188E_HAL_H__

