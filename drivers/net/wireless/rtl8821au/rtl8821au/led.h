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

#ifndef __RTL8821AU_LED_H__
#define __RTL8821AU_LED_H__

//================================================================================
//	LED Behavior Constant.
//================================================================================
// Default LED behavior.
//
#define LED_BLINK_NORMAL_INTERVAL	100
#define LED_BLINK_SLOWLY_INTERVAL	200
#define LED_BLINK_LONG_INTERVAL	400
#define LED_INITIAL_INTERVAL		1800

// LED Customerization

//ALPHA
#define LED_BLINK_NO_LINK_INTERVAL_ALPHA	1000
#define LED_BLINK_NO_LINK_INTERVAL_ALPHA_500MS 500 //add by ylb 20121012 for customer led for alpha
#define LED_BLINK_LINK_INTERVAL_ALPHA		500	//500
#define LED_BLINK_SCAN_INTERVAL_ALPHA		180 	//150
#define LED_BLINK_FASTER_INTERVAL_ALPHA		50
#define LED_BLINK_WPS_SUCESS_INTERVAL_ALPHA	5000

// 111122 by hpfan: Customized for Xavi
#define LED_CM11_BLINK_INTERVAL			300
#define LED_CM11_LINK_ON_INTERVEL		3000

//Netgear
#define LED_BLINK_LINK_INTERVAL_NETGEAR		500
#define LED_BLINK_LINK_SLOWLY_INTERVAL_NETGEAR		1000

#define LED_WPS_BLINK_OFF_INTERVAL_NETGEAR		100
#define LED_WPS_BLINK_ON_INTERVAL_NETGEAR		500

//Belkin AC950
#define LED_BLINK_LINK_INTERVAL_ON_BELKIN		200
#define LED_BLINK_LINK_INTERVAL_OFF_BELKIN		100
#define LED_BLINK_ERROR_INTERVAL_BELKIN		100

//================================================================================
// LED object.
//================================================================================

enum led_ctl_mode {
	LED_CTL_POWER_ON = 1,
	LED_CTL_LINK = 2,
	LED_CTL_NO_LINK = 3,
	LED_CTL_TX = 4,
	LED_CTL_RX = 5,
	LED_CTL_SITE_SURVEY = 6,
	LED_CTL_POWER_OFF = 7,
	LED_CTL_START_TO_LINK = 8,
	LED_CTL_START_WPS = 9,
	LED_CTL_STOP_WPS = 10,
};

enum rtl_led_pin {
	LED_PIN_GPIO0,
	LED_PIN_LED0,
	LED_PIN_LED1,
	LED_PIN_LED2
};


//================================================================================
// PCIE LED Definition.
//================================================================================

struct rtl_led {
	struct rtl_priv *		rtlpriv;

	enum rtl_led_pin		LedPin;	// Identify how to implement this SW led.

	bool				bLedOn; // true if LED is ON, false if LED is OFF.

	bool				bSWLedCtrl;
};

struct rtl_led_ctl {
	/* add for led controll */
	bool led_opendrain;
	struct rtl_led SwLed0;
	struct rtl_led SwLed1;
	struct rtl_led SwLed2;
	void (*SwLedOn)(struct rtl_priv *rtlpriv, struct rtl_led *pLed);
	void (*SwLedOff)(struct rtl_priv *rtlpriv, struct rtl_led *pLed);
	/* add for led controll */
};

//hal...

//================================================================================
// Interface to manipulate LED objects.
//================================================================================

void rtl8821au_init_sw_leds(struct rtl_priv *rtlpriv);
void rtl8821au_deinit_sw_leds(struct rtl_priv *rtlpriv);
void rtl8821au_led_control(struct rtl_priv *rtlpriv, enum led_ctl_mode LedAction);

#endif
