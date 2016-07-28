#ifndef __RTL8821AU_PR_H__
#define __RTL8821AU_PR_H__

void rtl8821au_phy_rf6052_set_bandwidth(struct rtl_priv *rtlpriv, enum CHANNEL_WIDTH	Bandwidth);
bool rtl8821au_phy_rf6052_config(struct rtl_priv *rtlpriv);
void rtl8821au_phy_rf6052_set_cck_txpower(struct rtl_priv *rtlpriv, uint8_t *pPowerlevel);

void rtl8821au_phy_rf6052_set_ofdm_txpower(struct rtl_priv *rtlpriv, uint8_t *pPowerLevelOFDM,
	uint8_t *pPowerLevelBW20, uint8_t *pPowerLevelBW40, uint8_t Channel);

#endif
