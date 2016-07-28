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
#define _RTW_EEPROM_C_

#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>

void up_clk(struct rtl_priv *rtlpriv, u16 *x)
{
	*x = *x | _EESK;
	usb_write8(rtlpriv, EE_9346CR, (uint8_t)*x);
	rtw_udelay_os(CLOCK_RATE);
}

void down_clk(struct rtl_priv *rtlpriv, u16 *x	)
{
	*x = *x & ~_EESK;
	usb_write8(rtlpriv, EE_9346CR, (uint8_t)*x);
	rtw_udelay_os(CLOCK_RATE);
}

void shift_out_bits(struct rtl_priv *rtlpriv, u16 data, u16 count)
{
	u16 x,mask;

	if(rtlpriv->bSurpriseRemoved==true){
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}

	mask = 0x01 << (count - 1);
	x = usb_read8(rtlpriv, EE_9346CR);

	x &= ~(_EEDO | _EEDI);

	do {
		x &= ~_EEDI;
		if (data & mask)
			x |= _EEDI;

		if (rtlpriv->bSurpriseRemoved==true) {
			RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
			goto out;
		}

		usb_write8(rtlpriv, EE_9346CR, (uint8_t)x);
		rtw_udelay_os(CLOCK_RATE);
		up_clk(rtlpriv, &x);
		down_clk(rtlpriv, &x);
		mask = mask >> 1;
	} while (mask);

	if (rtlpriv->bSurpriseRemoved==true){
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}

	x &= ~_EEDI;
	usb_write8(rtlpriv, EE_9346CR, (uint8_t)x);
out:
}

u16 shift_in_bits (struct rtl_priv *rtlpriv)
{
	u16 x,d=0,i;

	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}
	x = usb_read8(rtlpriv, EE_9346CR);

	x &= ~( _EEDO | _EEDI);
	d = 0;

	for(i = 0; i < 16; i++) {
		d = d << 1;
		up_clk(rtlpriv, &x);
		if(rtlpriv->bSurpriseRemoved == true) {
			RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
			goto out;
		}

		x = usb_read8(rtlpriv, EE_9346CR);

		x &= ~(_EEDI);
		if(x & _EEDO)
		d |= 1;

		down_clk(rtlpriv, &x);
	}
out:
	return d;
}

void standby(struct rtl_priv *rtlpriv)
{
	uint8_t   x;

	x = usb_read8(rtlpriv, EE_9346CR);

	x &= ~(_EECS | _EESK);
	usb_write8(rtlpriv, EE_9346CR,x);

	rtw_udelay_os(CLOCK_RATE);
	x |= _EECS;
	usb_write8(rtlpriv, EE_9346CR, x);
	rtw_udelay_os(CLOCK_RATE);
}

u16 wait_eeprom_cmd_done(struct rtl_priv* rtlpriv)
{
	uint8_t 	x;
	u16	i,res=false;

	standby(rtlpriv );

	for (i = 0; i < 200; i++) {
		x = usb_read8(rtlpriv, EE_9346CR);
		if (x & _EEDO){
			res=true;
			goto exit;
			}
		rtw_udelay_os(CLOCK_RATE);
	}
exit:

	return res;
}

void eeprom_clean(struct rtl_priv *rtlpriv)
{
	u16 x;

	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}
	x = usb_read8(rtlpriv, EE_9346CR);
	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}
	x &= ~(_EECS | _EEDI);
	usb_write8(rtlpriv, EE_9346CR, (uint8_t)x);

	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}

	up_clk(rtlpriv, &x);
	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}
	down_clk(rtlpriv, &x);
out:

}

void eeprom_write16(struct rtl_priv * rtlpriv, u16 reg, u16 data)
{
	uint8_t x;

	x = usb_read8(rtlpriv, EE_9346CR);

	x &= ~(_EEDI | _EEDO | _EESK | _EEM0);
	x |= _EEM1 | _EECS;
	usb_write8(rtlpriv, EE_9346CR, x);

	shift_out_bits(rtlpriv, EEPROM_EWEN_OPCODE, 5);

	if (rtlpriv->EepromAddressSize==8)	//CF+ and SDIO
		shift_out_bits(rtlpriv, 0, 6);
	else									//USB
		shift_out_bits(rtlpriv, 0, 4);

	standby( rtlpriv);

// Commented out by rcnjko, 2004.0
//	// Erase this particular word.  Write the erase opcode and register
//	// number in that order. The opcode is 3bits in length; reg is 6 bits long.
//	shift_out_bits(rtlpriv, EEPROM_ERASE_OPCODE, 3);
//	shift_out_bits(rtlpriv, reg, rtlpriv->EepromAddressSize);
//
//	if (wait_eeprom_cmd_done(rtlpriv ) == false)
//	{
//		return;
//	}


	standby(rtlpriv );

	// write the new word to the EEPROM

	// send the write opcode the EEPORM
	shift_out_bits(rtlpriv, EEPROM_WRITE_OPCODE, 3);

	// select which word in the EEPROM that we are writing to.
	shift_out_bits(rtlpriv, reg, rtlpriv->EepromAddressSize);

	// write the data to the selected EEPROM word.
	shift_out_bits(rtlpriv, data, 16);

	if (wait_eeprom_cmd_done(rtlpriv ) == false) {
		goto exit;
	}

	standby(rtlpriv );

	shift_out_bits(rtlpriv, EEPROM_EWDS_OPCODE, 5);
	shift_out_bits(rtlpriv, reg, 4);

	eeprom_clean(rtlpriv );
exit:

	return;
}

u16 eeprom_read16(struct rtl_priv * rtlpriv, u16 reg) //ReadEEprom
{

	u16 x;
	u16 data=0;

	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}
	// select EEPROM, reset bits, set _EECS
	x = usb_read8(rtlpriv, EE_9346CR);

	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}

	x &= ~(_EEDI | _EEDO | _EESK | _EEM0);
	x |= _EEM1 | _EECS;

	usb_write8(rtlpriv, EE_9346CR, (unsigned char)x);

	// write the read opcode and register number in that order
	// The opcode is 3bits in length, reg is 6 bits long

	shift_out_bits(rtlpriv, EEPROM_READ_OPCODE, 3);
	shift_out_bits(rtlpriv, reg, rtlpriv->EepromAddressSize);

	// Now read the data (16 bits) in from the selected EEPROM word
	data = shift_in_bits(rtlpriv);

	eeprom_clean(rtlpriv);
out:

	return data;
}




//From even offset
void eeprom_read_sz(struct rtl_priv * rtlpriv, u16 reg, uint8_t * data, uint32_t	 sz)
{

	u16 x, data16;
	uint32_t	 i;

	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}
	// select EEPROM, reset bits, set _EECS
	x = usb_read8(rtlpriv, EE_9346CR);

	if (rtlpriv->bSurpriseRemoved == true) {
		RT_TRACE(_module_rtl871x_eeprom_c_,_drv_err_,("rtlpriv->bSurpriseRemoved==true"));
		goto out;
	}

	x &= ~(_EEDI | _EEDO | _EESK | _EEM0);
	x |= _EEM1 | _EECS;
	usb_write8(rtlpriv, EE_9346CR, (unsigned char)x);

	// write the read opcode and register number in that order
	// The opcode is 3bits in length, reg is 6 bits long
	shift_out_bits(rtlpriv, EEPROM_READ_OPCODE, 3);
	shift_out_bits(rtlpriv, reg, rtlpriv->EepromAddressSize);


	for (i = 0; i < sz; i += 2) {
		data16 = shift_in_bits(rtlpriv);
		data[i] = data16 & 0xff;
		data[i+1] = data16 >>8;
	}

	eeprom_clean(rtlpriv);
out:
}


//addr_off : address offset of the entry in eeprom (not the tuple number of eeprom (reg); that is addr_off !=reg)
uint8_t eeprom_read(struct rtl_priv *rtlpriv, uint32_t	addr_off, uint8_t sz, uint8_t * rbuf)
{
	uint8_t quotient, remainder, addr_2align_odd;
	u16 reg, stmp , i=0, idx = 0;

	reg = (u16)(addr_off >> 1);
	addr_2align_odd = (uint8_t)(addr_off & 0x1);

	if(addr_2align_odd) { /* read that start at high part: e.g  1,3,5,7,9,... */
		stmp = eeprom_read16(rtlpriv, reg);
		rbuf[idx++] = (uint8_t) ((stmp>>8)&0xff); //return hogh-part of the short
		reg++; sz--;
	}

	quotient = sz >> 1;
	remainder = sz & 0x1;

	for (i = 0 ; i < quotient; i++) {
		stmp = eeprom_read16(rtlpriv, reg+i);
		rbuf[idx++] = (uint8_t) (stmp&0xff);
		rbuf[idx++] = (uint8_t) ((stmp>>8)&0xff);
	}

	reg = reg+i;
	if(remainder){ //end of read at lower part of short : 0,2,4,6,...
		stmp = eeprom_read16(rtlpriv, reg);
		rbuf[idx] = (uint8_t)(stmp & 0xff);
	}

	return true;
}



void read_eeprom_content(struct rtl_priv *rtlpriv)
{
}

