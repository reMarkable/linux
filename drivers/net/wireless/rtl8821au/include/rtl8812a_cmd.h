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
#ifndef __RTL8812A_CMD_H__
#define __RTL8812A_CMD_H__

typedef enum _RTL8812_H2C_CMD
{
	H2C_8812_RSVDPAGE = 0,
	H2C_8812_MSRRPT = 1,
	H2C_8812_SCAN = 2,
	H2C_8812_KEEP_ALIVE_CTRL = 3,
	H2C_8812_DISCONNECT_DECISION = 4,

	H2C_8812_INIT_OFFLOAD = 6,
	H2C_8812_AP_OFFLOAD = 8,
	H2C_8812_BCN_RSVDPAGE = 9,
	H2C_8812_PROBERSP_RSVDPAGE = 10,

	H2C_8812_SETPWRMODE = 0x20,
	H2C_8812_PS_TUNING_PARA = 0x21,
	H2C_8812_PS_TUNING_PARA2 = 0x22,
	H2C_8812_PS_LPS_PARA = 0x23,
	H2C_8812_P2P_PS_OFFLOAD = 0x24,
	H2C_8812_RA_MASK = 0x40,
	H2C_8812_RSSI_REPORT = 0x42,

	H2C_8812_WO_WLAN = 0x80,
	H2C_8812_REMOTE_WAKE_CTRL = 0x81,
	H2C_8812_AOAC_GLOBAL_INFO = 0x82,
	H2C_8812_AOAC_RSVDPAGE = 0x83,

	H2C_8812_TSF_RESET = 0xC0,

	MAX_8812_H2CCMD
}RTL8812_H2C_CMD;


typedef enum _RTL8812_C2H_EVT
{
	C2H_8812_DBG = 0,
	C2H_8812_LB = 1,
	C2H_8812_TXBF = 2,
	C2H_8812_TX_REPORT = 3,
	C2H_8812_BT_INFO = 9,
	C2H_8812_BT_MP = 11,
	C2H_8812_RA_RPT=12,

	C2H_8812_FW_SWCHNL = 0x10,
	C2H_8812_IQK_FINISH = 0x11,
	MAX_8812_C2HEVENT
}RTL8812_C2H_EVT;


struct cmd_msg_parm {
	uint8_t eid; //element id
	uint8_t sz; // sz
	uint8_t buf[6];
};

enum{
	PWRS
};

struct H2C_SS_RFOFF_PARAM{
	uint8_t ROFOn; // 1: on, 0:off
	u16 gpio_period; // unit: 1024 us
}__attribute__ ((packed));




void	Set_RA_LDPC_8812(struct sta_info	*psta, bool bLDPC);

// host message to firmware cmd
void rtl8812au_set_fw_pwrmode_cmd(struct rtl_priv *rtlpriv, uint8_t PSMode);
void rtl8812_set_raid_cmd(struct rtl_priv *rtlpriv, u32 bitmap, u8* arg);
void rtl8812_Add_RateATid(struct rtl_priv *rtlpriv, u32 bitmap, u8* arg, uint8_t rssi_level);

void rtl8812_set_FwMediaStatus_cmd(struct rtl_priv *rtlpriv, u16 mstatus_rpt );

#endif//__RTL8188E_CMD_H__


