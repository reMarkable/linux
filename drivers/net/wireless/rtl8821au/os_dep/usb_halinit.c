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
#define _HCI_HAL_INIT_C_

#include <drv_types.h>
#include <rtl8812a_hal.h>
#include <../rtl8821au/phy.h>
#include <../rtl8821au/reg.h>
#include <../rtl8821au/trx.h>
#include <../rtl8821au/hw.h>
#include <../rtl8821au/fw.h>
#include <hal_intf.h>

#undef DBG_8192C
static inline void DBG_8192C(const char *fmt, ...)
{
}

static void _dbg_dump_macreg(struct rtl_priv *rtlpriv)
{
	uint32_t offset = 0;
	uint32_t val32 = 0;
	uint32_t index = 0;

	for (index = 0; index < 64; index++) {
		offset = index*4;
		val32 = rtl_read_dword(rtlpriv, offset);
		DBG_8192C("offset : 0x%02x ,val:0x%08x\n", offset, val32);
	}
}




/*
---------------------------------------------------------------

	MAC init functions

---------------------------------------------------------------
*/


static void _InitHardwareDropIncorrectBulkOut_8812A(struct rtl_priv *rtlpriv)
{
	uint32_t value32 = rtl_read_dword(rtlpriv, REG_TXDMA_OFFSET_CHK);
	value32 |= DROP_DATA_EN;
	rtl_write_dword(rtlpriv, REG_TXDMA_OFFSET_CHK, value32);
}

static void _InitRxSetting_8812AU(struct rtl_priv *rtlpriv)
{
	rtl_write_dword(rtlpriv, REG_MACID, 0x87654321);
	/* ULLI unknown register */
	rtl_write_dword(rtlpriv, 0x0700, 0x87654321);
}


/* Set CCK and OFDM Block "ON" */
static void _BBTurnOnBlock(struct rtl_priv *rtlpriv)
{
	rtl_set_bbreg(rtlpriv, RFPGA0_RFMOD, bCCKEn, 0x1);
	rtl_set_bbreg(rtlpriv, RFPGA0_RFMOD, bOFDMEn, 0x1);
}

enum {
	Antenna_Lfet = 1,
	Antenna_Right = 2,
};


static void _rtl8821au_poweroff_adapter(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	uint8_t	u1bTmp;
	uint8_t 	val8;
	u16	val16;
	uint32_t	val32;

	/* DBG_871X("CardDisableRTL8188EU\n"); */

	/* Stop Tx Report Timer. 0x4EC[Bit1]=b'0 */
	u1bTmp = rtl_read_byte(rtlpriv, REG_TX_RPT_CTRL);
	rtl_write_byte(rtlpriv, REG_TX_RPT_CTRL, val8&(~BIT(1)));

	/* stop rx */
	rtl_write_byte(rtlpriv, REG_CR, 0x0);

	/* Run LPS WL RFOFF flow */
	if (IS_HARDWARE_TYPE_8821U(rtlhal))
		rtw_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, Rtl8821A_NIC_LPS_ENTER_FLOW);
	else
		rtw_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, Rtl8812_NIC_LPS_ENTER_FLOW);

	if ((rtl_read_byte(rtlpriv, REG_MCUFWDL)&RAM_DL_SEL) && rtlhal->fw_ready) {
		  /* 8051 RAM code */
		rtl8821au_firmware_selfreset(rtlpriv);
	}

	/* Reset MCU. Suggested by Filen. 2011.01.26. by tynli. */
	u1bTmp = rtl_read_byte(rtlpriv, REG_SYS_FUNC_EN+1);
	rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN+1, (u1bTmp&(~BIT(2))));

	/* MCUFWDL 0x80[1:0]=0
	 * reset MCU ready status
	 */
	rtl_write_byte(rtlpriv, REG_MCUFWDL, 0x00);

	/* Card disable power action flow */
	if (IS_HARDWARE_TYPE_8821U(rtlhal))
		rtw_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, Rtl8821A_NIC_DISABLE_FLOW);
	else
		rtw_hal_pwrseqcmdparsing(rtlpriv, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, Rtl8812_NIC_DISABLE_FLOW);
}

#if 0	/* ULLI : we have no HW power down here ->pwrctrlpriv.bHWPowerdown = 0 */
static void rtl8812au_hw_power_down(struct rtl_priv *rtlpriv)
{
	/*
	 *  2010/-8/09 MH For power down module, we need to enable register block contrl reg at 0x1c.
	 * Then enable power down control bit of register 0x04 BIT4 and BIT(15) as 1.
	 */

	/* Enable register area 0x0-0xc. */
	rtl_write_byte(rtlpriv, REG_RSV_CTRL, 0x0);
	rtl_write_word(rtlpriv, REG_APS_FSMCO, 0x8812);
}
#endif
uint32_t rtl8812au_hal_deinit(struct rtl_priv *rtlpriv)
{
	DBG_8192C("==> %s \n", __FUNCTION__);

	rtl_write_word(rtlpriv, REG_GPIO_MUXCFG, rtl_read_word(rtlpriv, REG_GPIO_MUXCFG)&(~BIT(12)));

	if (IF_RTL8821AU_USB3_MODE(rtlpriv->rtlhal.version) == true) {
		/*
		 * set Reg 0xf008[3:4] to 2'11 to eable U1/U2 Mode in USB3.0. added by page, 20120712
		 */
		rtl_write_byte(rtlpriv, 0xf008, rtl_read_byte(rtlpriv, 0xf008)|0x18);
	}

	rtl_write_dword(rtlpriv, REG_HISR0_8812, 0xFFFFFFFF);
	rtl_write_dword(rtlpriv, REG_HISR1_8812, 0xFFFFFFFF);
	rtlpriv->cfg->ops->disable_interrupt(rtlpriv);
	
	if (rtlpriv->hw_init_completed == true) {
		_rtl8821au_poweroff_adapter(rtlpriv);
#if 0	/* ULLI : we have no HW power down here ->pwrctrlpriv.bHWPowerdown = 0 */
		if ((rtlpriv->pwrctrlpriv.bHWPwrPindetect) && (rtlpriv->pwrctrlpriv.bHWPowerdown))
			rtl8812au_hw_power_down(rtlpriv);
#endif
	}
	return _SUCCESS;
}

unsigned int rtl8812au_inirp_deinit(struct rtl_priv *rtlpriv)
{
	usb_read_port_cancel(rtlpriv);
	return _SUCCESS;
}

/*
 * -------------------------------------------------------------------
 *
 * 	EEPROM/EFUSE Content Parsing
 *
 * -------------------------------------------------------------------
 */

void UpdateInterruptMask8812AU(struct rtl_priv *rtlpriv, uint8_t bHIMR0, uint32_t AddMSR, uint32_t RemoveMSR)
{
	struct rtl_usb *rtlusb = rtl_usbdev(rtlpriv);

	uint32_t *himr;

	if (bHIMR0)
		himr = &(rtlusb->irq_mask[0]);
	else
		himr = &(rtlusb->irq_mask[1]);

	if (AddMSR)
		*himr |= AddMSR;

	if (RemoveMSR)
		*himr &= (~RemoveMSR);

	if (bHIMR0)
		rtl_write_dword(rtlpriv, REG_HIMR0_8812, *himr);
	else
		rtl_write_dword(rtlpriv, REG_HIMR1_8812, *himr);

}

void _update_response_rate(struct rtl_priv *rtlpriv, unsigned int mask)
{
	uint8_t	RateIndex = 0;
	/* Set RRSR rate table. */
	rtl_write_byte(rtlpriv, REG_RRSR, mask&0xff);
	rtl_write_byte(rtlpriv, REG_RRSR+1, (mask>>8)&0xff);

	/* Set RTS initial rate */
	while (mask > 0x1) {
		mask = (mask >> 1);
		RateIndex++;
	}
	rtl_write_byte(rtlpriv, REG_INIRTS_RATE_SEL, RateIndex);
}
