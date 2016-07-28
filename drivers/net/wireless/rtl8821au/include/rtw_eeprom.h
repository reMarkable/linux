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
#ifndef __RTW_EEPROM_H__
#define __RTW_EEPROM_H__


#define	RTL8712_EEPROM_ID			0x8712
//#define	EEPROM_MAX_SIZE			256

#define	HWSET_MAX_SIZE_128		128
#define	HWSET_MAX_SIZE_256		256
#define	HWSET_MAX_SIZE_512		512

#define	EEPROM_MAX_SIZE			HWSET_MAX_SIZE_512

#define	CLOCK_RATE					50			//100us

//- EEPROM opcodes
#define EEPROM_READ_OPCODE		06
#define EEPROM_WRITE_OPCODE		05
#define EEPROM_ERASE_OPCODE		07
#define EEPROM_EWEN_OPCODE		19      // Erase/write enable
#define EEPROM_EWDS_OPCODE		16      // Erase/write disable

//Country codes
#define USA							0x555320
#define EUROPE						0x1 //temp, should be provided later
#define JAPAN						0x2 //temp, should be provided later


//
// Customer ID, note that:
// This variable is initiailzed through EEPROM or registry,
// however, its definition may be different with that in EEPROM for
// EEPROM size consideration. So, we have to perform proper translation between them.
// Besides, CustomerID of registry has precedence of that of EEPROM.
// defined below. 060703, by rcnjko.
//
typedef enum _RT_CUSTOMER_ID
{
	RT_CID_DEFAULT = 0,
	RT_CID_Sercomm_Belkin = 22,
	RT_CID_Edimax_ASUS = 35,
	RT_CID_NETGEAR = 36,
	RT_CID_Sercomm_Netgear = 43,
	RT_CID_ALPHA_Dlink = 44,//add by ylb 20121012 for customer led for alpha
}RT_CUSTOMER_ID, *PRT_CUSTOMER_ID;

extern void eeprom_write16(struct rtl_priv *rtlpriv, u16 reg, u16 data);
extern u16 eeprom_read16(struct rtl_priv *rtlpriv, u16 reg);
extern void read_eeprom_content(struct rtl_priv *rtlpriv);
extern void eeprom_read_sz(struct rtl_priv * rtlpriv, u16 reg,u8* data, u32 sz);

extern void read_eeprom_content_by_attrib(struct rtl_priv *	rtlpriv	);
#endif  //__RTL871X_EEPROM_H__

