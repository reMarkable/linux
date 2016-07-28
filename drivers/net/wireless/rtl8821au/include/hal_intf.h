/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
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
#ifndef __HAL_INTF_H__
#define __HAL_INTF_H__


enum RTL871X_HCI_TYPE {
	RTW_USB 	= BIT(1),
};

enum _CHIP_TYPE {
	NULL_CHIP_TYPE,
	RTL8812,
	RTL8821, //RTL8811
	MAX_CHIP_TYPE
};

typedef enum _HAL_ODM_VARIABLE{
	HAL_ODM_STA_INFO,
}HAL_ODM_VARIABLE;

typedef enum _HAL_INTF_PS_FUNC{
	HAL_USB_SELECT_SUSPEND,
	HAL_MAX_ID,
}HAL_INTF_PS_FUNC;

typedef int32_t (*c2h_id_filter)(uint8_t id);


typedef	enum _RT_EEPROM_TYPE{
	EEPROM_93C46,
	EEPROM_93C56,
	EEPROM_BOOT_EFUSE,
}RT_EEPROM_TYPE,*PRT_EEPROM_TYPE;



#define RF_CHANGE_BY_INIT	0
#define RF_CHANGE_BY_IPS 	BIT28
#define RF_CHANGE_BY_PS 	BIT29
#define RF_CHANGE_BY_HW 	BIT30
#define RF_CHANGE_BY_SW 	BIT31

uint rtw_hal_init(struct rtl_priv *rtlpriv);
uint rtw_hal_deinit(struct rtl_priv *rtlpriv);
void rtw_hal_stop(struct rtl_priv *rtlpriv);

void	rtw_hal_get_odm_var(struct rtl_priv *rtlpriv, HAL_ODM_VARIABLE eVariable, void *pValue1,bool bSet);

int32_t	rtw_hal_mgnt_xmit(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe);

void rtw_hal_update_ra_mask(struct rtl_priv *rtlpriv, struct sta_info *psta, uint8_t rssi_level);

#endif //__HAL_INTF_H__

