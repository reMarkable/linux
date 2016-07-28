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
#define _HAL_COM_C_

#include <drv_types.h>
#include <odm_precomp.h>

#define	EEPROM_CHANNEL_PLAN_BY_HW_MASK	0x80

/*
 * 	u8 hw_channel_plan,	channel plan from HW (efuse/eeprom)
 *	u8 sw_channel_plan,	channel plan from SW (registry/module param)
 *	u8 def_channel_plan,	channel plan used when the former two is invalid
 *	bool AutoLoadFail
 */

#define rtw_is_channel_plan_valid(chplan) (chplan<RT_CHANNEL_DOMAIN_MAX || \
					   chplan == RT_CHANNEL_DOMAIN_REALTEK_DEFINE)


u8 hal_com_get_channel_plan(struct rtl_priv *rtlpriv, u8 hw_channel_plan,	
			    u8 sw_channel_plan, u8 def_channel_plan,
			    bool AutoLoadFail)
{
	uint8_t swConfig;
	uint8_t chnlPlan;

	swConfig = true;
	if (!AutoLoadFail) {
		if (!rtw_is_channel_plan_valid(sw_channel_plan))
			swConfig = false;
		if (hw_channel_plan & EEPROM_CHANNEL_PLAN_BY_HW_MASK)
			swConfig = false;
	}

	if (swConfig == true)
		chnlPlan = sw_channel_plan;
	else
		chnlPlan = hw_channel_plan & (~EEPROM_CHANNEL_PLAN_BY_HW_MASK);

	if (!rtw_is_channel_plan_valid(chnlPlan))
		chnlPlan = def_channel_plan;

	return chnlPlan;
}

bool HAL_IsLegalChannel(struct rtl_priv *rtlpriv, uint32_t Channel)
{
	bool bLegalChannel = true;

	if (Channel > 14) {
		if(IsSupported5G(WIRELESS_MODE_MAX) == false) {
			bLegalChannel = false;
			RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "Channel > 14 but wireless_mode do not support 5G\n");
		}
	} else if ((Channel <= 14) && (Channel >=1)){
		if(IsSupported24G(WIRELESS_MODE_MAX) == false) {
			bLegalChannel = false;
			RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "(Channel <= 14) && (Channel >=1) but wireless_mode do not support 2.4G\n");
		}
	} else {
		bLegalChannel = false;
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "Channel is Invalid !!!\n");
	}

	return bLegalChannel;
}

void HalSetBrateCfg(struct rtl_priv *rtlpriv, uint8_t *mBratesOS,
	u16	*pBrateCfg)
{
	uint8_t	i, is_brate, brate;

	for(i=0;i<NDIS_802_11_LENGTH_RATES_EX;i++)
	{
		is_brate = mBratesOS[i] & IEEE80211_BASIC_RATE_MASK;
		brate = mBratesOS[i] & 0x7f;

		if( is_brate )
		{
			switch(brate)
			{
				case IEEE80211_CCK_RATE_1MB:	*pBrateCfg |= RATE_1M;	break;
				case IEEE80211_CCK_RATE_2MB:	*pBrateCfg |= RATE_2M;	break;
				case IEEE80211_CCK_RATE_5MB:	*pBrateCfg |= RATE_5_5M;break;
				case IEEE80211_CCK_RATE_11MB:	*pBrateCfg |= RATE_11M;	break;
				case IEEE80211_OFDM_RATE_6MB:	*pBrateCfg |= RATE_6M;	break;
				case IEEE80211_OFDM_RATE_9MB:	*pBrateCfg |= RATE_9M;	break;
				case IEEE80211_OFDM_RATE_12MB:	*pBrateCfg |= RATE_12M;	break;
				case IEEE80211_OFDM_RATE_18MB:	*pBrateCfg |= RATE_18M;	break;
				case IEEE80211_OFDM_RATE_24MB:	*pBrateCfg |= RATE_24M;	break;
				case IEEE80211_OFDM_RATE_36MB:	*pBrateCfg |= RATE_36M;	break;
				case IEEE80211_OFDM_RATE_48MB:	*pBrateCfg |= RATE_48M;	break;
				case IEEE80211_OFDM_RATE_54MB:	*pBrateCfg |= RATE_54M;	break;
			}
		}
	}
}

uint8_t  rtw_hal_networktype_to_raid(struct rtl_priv *rtlpriv,unsigned char network_type)
{
	return networktype_to_raid_ex(rtlpriv,network_type);
}

uint8_t rtw_get_mgntframe_raid(struct rtl_priv *rtlpriv,unsigned char network_type)
{

	uint8_t raid;
	raid = (network_type & WIRELESS_11B) ?RATEID_IDX_B :RATEID_IDX_G;
	return raid;
}


