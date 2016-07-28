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

#ifdef CONFIG_RTLWIFI

#include <../drivers/net/wireless/realtek/rtlwifi/wifi.h>
#include <../drivers/net/wireless/realtek/rtlwifi/use.h>

#else

#include <drv_types.h>

#endif

/*
 * ULLI led.c is a mess, we have two different paths for
 * rtl8812au and rtl8821au why ??
 *
 * And up too three leds ...
 */

/* for RTL8812AU */
/* TP-LINK : LED_PIN_LED0 */
/* LINKSYS : LED_PIN_LED0 -> traffic */
/* LINKSYS : LED_PIN_LED2 -> WPS ?? or whatever */

/* for RTL8821AU see LED PATH for HW info */
/* For Digitus Wireless AC433 */
/* DLINK DWA */

/* 
 * Better rewrite the mess for rtlwifi
 * rtlwifi has (currently) no timer for the led blinkinbg thing
 * only static on/off switching.
 * For secure led :
 * We *must* check ieee80211 for secure connection,
 * but we can spoof user(space) width this. The other issue is, if 
 * somebody changes the PCB layout. tought.
 * Secure led is *currently* disabled
 */

static void _rtl8821au_init_led(struct rtl_priv *rtlpriv, 
				struct rtl_led *pled, enum rtl_led_pin ledpin)
{
	pled->rtlpriv = rtlpriv;
	pled->LedPin = ledpin;
	pled->bLedOn = false;
}

static void _rtl8821au_deinit_led(struct rtl_led *pled)
{
}

static void rtl8812au_sw_led_off(struct rtl_priv *rtlpriv, struct rtl_led *pLed)
{
	struct rtl_usb_priv *usbpriv = rtl_usbpriv(rtlpriv);
	struct rtl_led_ctl *pledpriv = &(usbpriv->ledpriv);

	uint8_t	LedCfg;
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);

	if (rtlpriv->bSurpriseRemoved == true) {
		return;
	}

	{
		switch (pLed->LedPin) {
		case LED_PIN_GPIO0:
			break;

		case LED_PIN_LED0:
			 if (pHalData->AntDivCfg == 0) {
				LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG0);
				LedCfg &= 0x70; 	/* Set to software control. */
				rtl_write_byte(rtlpriv, REG_LEDCFG0, (LedCfg|BIT(3)|BIT(5)));
			} else {
				LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG2);
				LedCfg &= 0xe0; 	/* Set to software control. */
				rtl_write_byte(rtlpriv, REG_LEDCFG2, (LedCfg|BIT(3)|BIT(7)|BIT(6)|BIT(5)));
			}
			break;

		case LED_PIN_LED1:
			LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG1);
			LedCfg &= 0x70; 	/* Set to software control. */
			rtl_write_byte(rtlpriv, REG_LEDCFG1, (LedCfg|BIT(3)|BIT(5)));
			break;

		case LED_PIN_LED2:
			LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG2);
			LedCfg &= 0x70; 	/* Set to software control. */
			rtl_write_byte(rtlpriv, REG_LEDCFG2, (LedCfg|BIT(3)|BIT(5)));
			break;

		default:
			break;
		}
	}

	pLed->bLedOn = false;
}

static void rtl8821au_sw_led_off(struct rtl_priv *rtlpriv, struct rtl_led *pLed)
{
	struct rtl_usb_priv *usbpriv = rtl_usbpriv(rtlpriv);
	struct rtl_led_ctl *pledpriv = &(usbpriv->ledpriv);

	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	uint8_t	LedCfg;

	if (rtlpriv->bSurpriseRemoved == true) {
		return;
	}

	{
		switch (pLed->LedPin) {
		case LED_PIN_GPIO0:
			break;

		case LED_PIN_LED0:
		case LED_PIN_LED1:
		case LED_PIN_LED2:
			 if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
				LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG2);
				LedCfg &= 0x20; 	/* Set to software control. */
				rtl_write_byte(rtlpriv, REG_LEDCFG2, (LedCfg|BIT(3)|BIT(5)));
			 }

			break;


		default:
			break;
		}
	}

	pLed->bLedOn = false;
}

static void rtl8812au_sw_led_on(struct rtl_priv *rtlpriv, struct rtl_led *pLed)
{
	uint8_t	LedCfg;
	 struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);

	if ((rtlpriv->bSurpriseRemoved == true) || (rtlpriv->bDriverStopped == true)) {
		return;
	}

	{
		switch (pLed->LedPin) {
		case LED_PIN_GPIO0:
			break;

		case LED_PIN_LED0:
			if (pHalData->AntDivCfg == 0) {
				LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG0);
				rtl_write_byte(rtlpriv, REG_LEDCFG0, (LedCfg&0x70)|BIT(5)); /* SW control led0 on. */
			} else {
				LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG2);
				rtl_write_byte(rtlpriv, REG_LEDCFG2, (LedCfg&0xe0)|BIT(7)|BIT(6)|BIT(5)); /* SW control led0 on. */
			}
			break;

		case LED_PIN_LED1:
			LedCfg = rtl_read_byte(rtlpriv, (REG_LEDCFG1));
			rtl_write_byte(rtlpriv, (REG_LEDCFG1), (LedCfg&0x70)|BIT(5)); /* SW control led1 on. */
			break;

		case LED_PIN_LED2:
			LedCfg = rtl_read_byte(rtlpriv, (REG_LEDCFG2));
			rtl_write_byte(rtlpriv, (REG_LEDCFG2), (LedCfg&0x70)|BIT(5)); /* SW control led1 on. */
			break;

		default:
			break;
		}
	}

	pLed->bLedOn = true;
}

static void rtl8821au_sw_led_on(struct rtl_priv *rtlpriv, struct rtl_led *pLed)
{
	uint8_t	LedCfg;
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	if ((rtlpriv->bSurpriseRemoved == true) || (rtlpriv->bDriverStopped == true)) {
		return;
	}

	 {
		switch (pLed->LedPin) {
		case LED_PIN_GPIO0:
			break;

		case LED_PIN_LED0:
		case LED_PIN_LED1:
		case LED_PIN_LED2:
			if (IS_HARDWARE_TYPE_8821U(rtlhal)) {
				LedCfg = rtl_read_byte(rtlpriv, REG_LEDCFG2);
				rtl_write_byte(rtlpriv, REG_LEDCFG2, ((LedCfg&0x20) & (~BIT(3)))|BIT(5)); /* SW control led0 on. */
			}

			break;

		default:
			break;
		}
	}
	pLed->bLedOn = true;
}

void rtl8821au_init_sw_leds(struct rtl_priv *rtlpriv)
{
	struct rtl_usb_priv *usbpriv = rtl_usbpriv(rtlpriv);
		
	_rtl8821au_init_led(rtlpriv, &(usbpriv->ledpriv.SwLed0), LED_PIN_LED0);
	_rtl8821au_init_led(rtlpriv, &(usbpriv->ledpriv.SwLed1), LED_PIN_LED1);
}

void rtl8821au_deinit_sw_leds(struct rtl_priv *rtlpriv)
{
	struct rtl_usb_priv *usbpriv = rtl_usbpriv(rtlpriv);

	_rtl8821au_deinit_led(&(usbpriv->ledpriv.SwLed0));
	_rtl8821au_deinit_led(&(usbpriv->ledpriv.SwLed1));

}

static void _rtl8821au_sw_led_control(struct rtl_priv *rtlpriv,
				      enum led_ctl_mode ledaction)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct rtl_usb_priv *usbpriv = rtl_usbpriv(rtlpriv);
	
	if (ledaction == LED_CTL_TX ||
	    ledaction == LED_CTL_RX ||
	    ledaction == LED_CTL_SITE_SURVEY ||
	    ledaction == LED_CTL_LINK ||
	    ledaction == LED_CTL_NO_LINK ||
	    ledaction == LED_CTL_START_TO_LINK ||
	    ledaction == LED_CTL_POWER_ON) {
		if (IS_HARDWARE_TYPE_8812(rtlhal))
			rtl8812au_sw_led_on(rtlpriv, &(usbpriv->ledpriv.SwLed0));
		else
			rtl8821au_sw_led_on(rtlpriv, &(usbpriv->ledpriv.SwLed0));
	} else {
		if (IS_HARDWARE_TYPE_8812(rtlhal))
			rtl8812au_sw_led_off(rtlpriv, &(usbpriv->ledpriv.SwLed0));
		else
			rtl8821au_sw_led_off(rtlpriv, &(usbpriv->ledpriv.SwLed0));
	}
}

void rtl8821au_led_control(struct rtl_priv *rtlpriv,
			   enum led_ctl_mode ledaction)
{
#if 0	/* currently no power saving */	
	struct rtl_ps_ctl *ppsc = rtl_psc(rtl_priv(hw));

	if ((ppsc->rfoff_reason > RF_CHANGE_BY_PS) &&
	    (ledaction == LED_CTL_TX ||
	     ledaction == LED_CTL_RX ||
	     ledaction == LED_CTL_SITE_SURVEY ||
	     ledaction == LED_CTL_LINK ||
	     ledaction == LED_CTL_NO_LINK ||
	     ledaction == LED_CTL_START_TO_LINK ||
	     ledaction == LED_CTL_POWER_ON)) {
		return;
	}
#endif	
	RT_TRACE(rtlpriv, COMP_LED, DBG_LOUD, "ledaction %d\n", ledaction);
	_rtl8821au_sw_led_control(rtlpriv, ledaction);
}

