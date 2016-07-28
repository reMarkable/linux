#ifndef __RTL8821AU_PHY_H__
#define __RTL8821AU_PHY_H__

u32 rtl8821au_phy_query_bb_reg(struct rtl_priv *rtlpriv, u32 RegAddr, u32 BitMask);
void rtl8821au_phy_set_bb_reg(struct rtl_priv *	rtlpriv, u32 RegAddr, u32 BitMask, u32 Data);
u32 rtl8821au_phy_query_rf_reg(struct rtl_priv *rtlpriv, u32 eRFPath, u32 RegAddr, u32 BitMask);
void rtl8821au_phy_set_rf_reg(struct rtl_priv *rtlpriv, u32 eRFPath, u32 RegAddr, 
	u32 BitMask, u32 Data);
bool _rtl8821au_phy_config_bb_with_headerfile(struct rtl_priv *rtlpriv,
						       u8 configtype);
bool _rtl8821au_phy_config_bb_with_pgheaderfile(struct rtl_priv *rtlpriv,
							u8 configtype);
void rtl8821au_phy_set_bw_mode_callback(struct rtl_priv *rtlpriv);
void rtl8812au_phy_iq_calibrate(struct rtl_priv *rtlpriv, bool bReCovery);

							
void rtl8821au_phy_iq_calibrate(struct rtl_priv *rtlpriv, bool bReCovery);


void _rtl8821au_phy_config_mac_with_headerfile(struct rtl_priv *rtlpriv);

bool rtl8821au_phy_config_rf_with_headerfile(struct rtl_priv *rtlpriv, enum radio_path eRFPath);
bool rtl8812au_phy_config_rf_with_headerfile(struct rtl_priv *rtlpriv, enum radio_path eRFPath);
void rtl8821au_phy_switch_wirelessband(struct rtl_priv *rtlpriv, u8 Band);
void rtl8821au_phy_sw_chnl_callback(struct rtl_priv *rtlpriv);
int rtl8821au_phy_bb_config(struct rtl_priv *rtlpriv);


uint32_t phy_get_tx_swing_8821au(struct rtl_priv *rtlpriv, enum band_type Band,
	uint8_t	RFPath);
	
enum _ANT_DIV_TYPE {
	NO_ANTDIV		= 0xFF,
	CG_TRX_HW_ANTDIV	= 0x01,
	CGCS_RX_HW_ANTDIV 	= 0x02,
	FIXED_HW_ANTDIV		= 0x03,
	CG_TRX_SMART_ANTDIV	= 0x04,
	CGCS_RX_SW_ANTDIV	= 0x05,
	S0S1_HW_ANTDIV          = 0x06, //8723B intrnal switch S0 S1
};

enum baseband_config_type {
	BASEBAND_CONFIG_PHY_REG = 0,
	BASEBAND_CONFIG_AGC_TAB = 1,
};
	
/* Not in rtlwifi */

void	PHY_SetTxPowerLevel8812(struct rtl_priv *rtlpriv, uint8_t Channel);
bool phy_SwBand8812(struct rtl_priv *rtlpriv, uint8_t channelToSW);

#endif
