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

#define _HAL_INTF_C_

#include <drv_types.h>
#include <rtw_ap.h>
#include <rtl8812a_hal.h>

uint rtw_hal_init(struct rtl_priv *rtlpriv)
{
	uint status = _SUCCESS;

	rtlpriv->hw_init_completed = false;

	status = rtlpriv->cfg->ops->hw_init(rtlpriv);

	if (status == _SUCCESS) {
		rtlpriv->hw_init_completed = true;
	} else {
		rtlpriv->hw_init_completed = false;
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "rtw_hal_init: hal__init fail\n");
	}

	return status;

}

uint rtw_hal_deinit(struct rtl_priv *rtlpriv)
{
	uint	status = _SUCCESS;

	status = rtlpriv->cfg->ops->hal_deinit(rtlpriv);

	if (status == _SUCCESS) {
		rtlpriv->hw_init_completed = false;
	} else {
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "\n rtw_hal_deinit: hal_init fail\n");
	}

	return status;
}

int32_t	rtw_hal_mgnt_xmit(struct rtl_priv *rtlpriv, struct xmit_frame *pmgntframe)
{
	int32_t ret = _FAIL;

	update_mgntframe_attrib_addr(rtlpriv, pmgntframe);

	if (rtlpriv->cfg->ops->mgnt_xmit)
		ret = rtlpriv->cfg->ops->mgnt_xmit(rtlpriv, pmgntframe);
	return ret;
}

void rtw_hal_update_ra_mask(struct rtl_priv *rtlpriv, struct sta_info *psta, 
			    uint8_t rssi_level)
{
	struct mlme_priv *pmlmepriv;

	if (!psta)
		return;

	pmlmepriv = &(rtlpriv->mlmepriv);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) == true) {
		add_RATid(rtlpriv, psta, rssi_level);
	} else {
		UpdateHalRAMask8812A(rtlpriv, psta->mac_id, rssi_level);
	}
}
