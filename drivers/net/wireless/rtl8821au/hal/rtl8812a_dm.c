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

#define _RTL8812A_DM_C_

#include <drv_types.h>
#include <rtl8812a_hal.h>
#include <../rtl8821au/dm.h>


/*
 * Initialize GPIO setting registers
 */


/*
 * ============================================================
 * functions
 * ============================================================
 */

static void Init_ODM_ComInfo_8812(struct rtl_priv *rtlpriv)
{
	struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct _rtw_dm *pDM_Odm = &(pHalData->odmpriv);

	/*
	 * Init Value
	 */

	memset(pDM_Odm,0,sizeof(*pDM_Odm));

	pDM_Odm->rtlpriv = rtlpriv;


	/* 1 ============== End of BoardType ============== */


	/* if(pHalData->AntDivCfg)
	 *	pdmpriv->InitODMFlag |= ODM_BB_ANT_DIV;
	 */
}

void rtl8812_init_dm_priv(struct rtl_priv *rtlpriv)
{
	struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;
	memset(pdmpriv, 0, sizeof(struct dm_priv));

	/* spin_lock_init(&(pHalData->odm_stainfo_lock)); */

	Init_ODM_ComInfo_8812(rtlpriv);
}

