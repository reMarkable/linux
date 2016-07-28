#ifndef __RTL8821AU_HW_H__
#define __RTL8821AU_HW_H__


#include <rtl8812a_hal.h>

void rtl8821au_read_chip_version(struct rtl_priv *rtlpriv);
void rtl8821au_set_hw_reg(struct rtl_priv *rtlpriv, u8 variable,u8 *val);
void rtl8821au_get_hw_reg(struct rtl_priv *rtlpriv, u8 variable,u8 *val);

void rtl8821au_set_beacon_related_registers(struct rtl_priv *rtlpriv);
uint32_t rtl8812au_hw_init(struct rtl_priv *rtlpriv);
void rtl8821au_init_beacon_parameters(struct rtl_priv *rtlpriv);
int32_t	 _rtl8821au_llt_table_init(struct rtl_priv *rtlpriv, uint8_t txpktbuf_bndy);
void rtl8821au_enable_interrupt(struct rtl_priv *rtlpriv);
void rtl8821au_disable_interrupt(struct rtl_priv *rtlpriv);

/* temporaly prototypes for transition */

void Set_MSR(struct rtl_priv *rtlpriv, uint8_t type);
bool rtl8821au_gpio_radio_on_off_checking(struct rtl_priv *rtlpriv, u8 *valid);
int rtl8821au_set_network_type(struct rtl_priv *rtlpriv, uint8_t mode);

#endif
