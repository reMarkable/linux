#include <drv_types.h>
#include "wifi.h"
#include <rtl8812a_spec.h>
#include <rtl8812a_hal.h>

static void efuse_ShadowRead1Byte(struct rtl_priv *rtlpriv, u16	Offset,
	 	u8 *Value)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	*Value = rtlefuse->efuse_map[0][Offset];
}

static void efuse_ShadowRead2Byte(struct rtl_priv *rtlpriv, u16	Offset,
		u16 *Value)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	*Value = rtlefuse->efuse_map[0][Offset];
	*Value |= rtlefuse->efuse_map[0][Offset+1] << 8;
}

//---------------Read Four Bytes
static void efuse_ShadowRead4Byte(struct rtl_priv *rtlpriv, u16	Offset,
	 	u32 *Value)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	*Value = rtlefuse->efuse_map[0][Offset];
	*Value |= rtlefuse->efuse_map[0][Offset + 1] << 8;
	*Value |= rtlefuse->efuse_map[0][Offset + 2] << 16;
	*Value |= rtlefuse->efuse_map[0][Offset + 3] << 24;
}

static void efuse_ShadowWrite1Byte(struct rtl_priv *rtlpriv, u16 Offset,
		u8 Value)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	rtlefuse->efuse_map[0][Offset] = Value;

}

static void efuse_ShadowWrite2Byte(struct rtl_priv *rtlpriv, u16 Offset,
		u16 Value)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	rtlefuse->efuse_map[0][Offset] = Value & 0x0ff;
	rtlefuse->efuse_map[0][Offset+1] = Value >> 8;
}

static void efuse_ShadowWrite4Byte(struct rtl_priv *rtlpriv, u16 Offset,
		u32 Value)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	rtlefuse->efuse_map[0][Offset] = (u8) (Value & 0x0000000ff);
	rtlefuse->efuse_map[0][Offset+1] = (u8) ((Value >> 8) & 0x0000ff);
	rtlefuse->efuse_map[0][Offset+2] = (u8) ((Value >> 16) & 0x00ff);
	rtlefuse->efuse_map[0][Offset+3] = (u8) ((Value >> 24) & 0xff);
}

void EFUSE_ShadowRead(struct rtl_priv *rtlpriv, u8 Type,
		u16 Offset, u32 *Value)
{
	if (Type == 1)
		efuse_ShadowRead1Byte(rtlpriv, Offset, (uint8_t *)Value);
	else if (Type == 2)
		efuse_ShadowRead2Byte(rtlpriv, Offset, (u16 *)Value);
	else if (Type == 4)
		efuse_ShadowRead4Byte(rtlpriv, Offset, (uint32_t *)Value);

}	// EFUSE_ShadowRead

static void efuse_power_switch(struct rtl_priv *rtlpriv, u8 write, u8 pwrstate)
{
	uint8_t	tempval;
	u16	tmpV16;
#define EFUSE_ACCESS_ON_JAGUAR 0x69
#define EFUSE_ACCESS_OFF_JAGUAR 0x00
	if (pwrstate) {
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_ACCESS], EFUSE_ACCESS_ON_JAGUAR);

		/* 1.2V Power: From VDDON with Power Cut(0x0000h[15]), defualt valid */
		tmpV16 = rtl_read_word(rtlpriv, REG_SYS_ISO_CTRL);
		if (!(tmpV16 & rtlpriv->cfg->maps[EFUSE_PWC_EV12V])) {
			tmpV16 |= rtlpriv->cfg->maps[EFUSE_PWC_EV12V];
			/* rtl_write_word(rtlpriv,REG_SYS_ISO_CTRL,tmpV16); */
		}
		/* Reset: 0x0000h[28], default valid */
		tmpV16 =  rtl_read_word(rtlpriv, REG_SYS_FUNC_EN);
		if (!(tmpV16 & rtlpriv->cfg->maps[EFUSE_FEN_ELDR])) {
			tmpV16 |= rtlpriv->cfg->maps[EFUSE_FEN_ELDR] ;
			rtl_write_word(rtlpriv, REG_SYS_FUNC_EN, tmpV16);
		}

		/* Clock: Gated(0x0008h[5]) 8M(0x0008h[1]) clock from ANA, default valid */
		tmpV16 = rtl_read_word(rtlpriv, REG_SYS_CLKR);
		if ((!(tmpV16 & rtlpriv->cfg->maps[EFUSE_LOADER_CLK_EN])) || 
		    (!(tmpV16 & rtlpriv->cfg->maps[EFUSE_ANA8M]))) {
			tmpV16 |= (rtlpriv->cfg->maps[EFUSE_LOADER_CLK_EN] | 
			           rtlpriv->cfg->maps[EFUSE_ANA8M]);
			rtl_write_word(rtlpriv, REG_SYS_CLKR, tmpV16);
		}

		if (write) {
			/* Enable LDO 2.5V before read/write action */
			tempval = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_TEST]+3);
			tempval &= ~(BIT(3) | BIT(4) | BIT(5) | BIT(6));
			tempval |= (VOLTAGE_V25 << 3);
			tempval |= BIT(7);
			rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_TEST] + 3, tempval);
		}
	} else {
		rtl_write_byte(rtlpriv, REG_EFUSE_BURN_GNT_8812, EFUSE_ACCESS_OFF_JAGUAR);

		if (write) {
			/* Disable LDO 2.5V after read/write action */
			tempval = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_TEST] + 3);
			rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_TEST] + 3, (tempval & 0x7F));
		}
	}
}


static void read_efuse(struct rtl_priv *rtlpriv, u16 _offset,  u16 _size_byte, u8 *pbuf)
{
	uint8_t	*efuseTbl = NULL;
	u16	eFuse_Addr = 0;
	uint8_t	offset = 0, wden = 0;
	u16	i, j;
	u16	**eFuseWord = NULL;
	u16	efuse_utilized = 0;
	uint8_t	efuse_usage = 0;
	uint8_t	offset_2_0 = 0;
	uint8_t	efuseHeader = 0, efuseExtHdr = 0, efuseData = 0;
	const u16 efuse_max_section =
		rtlpriv->cfg->maps[EFUSE_MAX_SECTION_MAP];
	const u32 efuse_len =
		rtlpriv->cfg->maps[EFUSE_REAL_CONTENT_SIZE];

	/*
	 * Do NOT excess total size of EFuse table. Added by Roger, 2008.11.10.
	 */
	if ((_offset + _size_byte) > rtlpriv->cfg->maps[EFUSE_HWSET_MAX_SIZE]) {
		/* total E-Fuse table is 512bytes */
		dev_err(&(rtlpriv->ndev->dev), "Hal_EfuseReadEFuse8812A(): Invalid offset(%#x) with read bytes(%#x)!!\n", _offset, _size_byte);
		goto exit;
	}

	efuseTbl = (uint8_t *) rtw_zmalloc(rtlpriv->cfg->maps[EFUSE_HWSET_MAX_SIZE]);
	if (efuseTbl == NULL) {
		dev_err(&(rtlpriv->ndev->dev), "%s: alloc efuseTbl fail!\n", __FUNCTION__);
		goto exit;
	}

	eFuseWord = (u16 **) rtw_malloc2d(efuse_max_section, EFUSE_MAX_WORD_UNIT, sizeof(u16));
	if (eFuseWord == NULL) {
		dev_err(&(rtlpriv->ndev->dev), "%s: alloc eFuseWord fail!\n", __FUNCTION__);
		goto exit;
	}

	/* 0. Refresh efuse init map as all oxFF. */
	for (i = 0; i < efuse_max_section; i++)
		for (j = 0; j < EFUSE_MAX_WORD_UNIT; j++)
			eFuseWord[i][j] = 0xFFFF;

	/*
	 * 1. Read the first byte to check if efuse is empty!!!
	 */
	efuse_OneByteRead(rtlpriv, eFuse_Addr++, &efuseHeader);

	if (efuseHeader != 0xFF) {
		efuse_utilized++;
	} else {
		dev_err(&(rtlpriv->ndev->dev), "EFUSE is empty\n");
		efuse_utilized = 0;
		goto exit;
	}
	/* RT_DISP(FEEPROM, EFUSE_READ_ALL, ("Hal_EfuseReadEFuse8812A(): efuse_utilized: %d\n", efuse_utilized)); */

	/*
	 * 2. Read real efuse content. Filter PG header and every section data.
	 */
	while ((efuseHeader != 0xFF) && AVAILABLE_EFUSE_ADDR_8812(eFuse_Addr)) {
		/* RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse_Addr-%d efuse_data=%x\n", eFuse_Addr-1, *rtemp8)); */

		/* Check PG header for section num. */
		if (EXT_HEADER(efuseHeader)) {	/* extended header */
			offset_2_0 = GET_HDR_OFFSET_2_0(efuseHeader);
			/* RT_DISP(FEEPROM, EFUSE_READ_ALL, ("extended header offset_2_0=%X\n", offset_2_0)); */

			efuse_OneByteRead(rtlpriv, eFuse_Addr++, &efuseExtHdr);

			/* RT_DISP(FEEPROM, EFUSE_READ_ALL, ("efuse[%X]=%X\n", eFuse_Addr-1, efuseExtHdr)); */

			if (efuseExtHdr != 0xff) {
				efuse_utilized++;
				if (ALL_WORDS_DISABLED(efuseExtHdr)) {
					efuse_OneByteRead(rtlpriv, eFuse_Addr++, &efuseHeader);
					if (efuseHeader != 0xff) {
						efuse_utilized++;
					}
					break;
				} else {
					offset = ((efuseExtHdr & 0xF0) >> 1) | offset_2_0;
					wden = (efuseExtHdr & 0x0F);
				}
			} else 	{
				dev_err(&(rtlpriv->ndev->dev), "Error condition, extended = 0xff\n");
				/* We should handle this condition. */
				break;
			}
		} else {
			offset = ((efuseHeader >> 4) & 0x0f);
			wden = (efuseHeader & 0x0f);
		}

		if (offset < efuse_max_section) {
			/* Get word enable value from PG header */
			/* RT_DISP(FEEPROM, EFUSE_READ_ALL, ("Offset-%X Worden=%X\n", offset, wden)); */

			for (i = 0; i < EFUSE_MAX_WORD_UNIT; i++) {
				/* Check word enable condition in the section */
				if (!(wden & (0x01 << i))) {
					efuse_OneByteRead(rtlpriv, eFuse_Addr++, &efuseData);
					/* RT_DISP(FEEPROM, EFUSE_READ_ALL, ("efuse[%X]=%X\n", eFuse_Addr-1, efuseData)); */
					efuse_utilized++;
					eFuseWord[offset][i] = (efuseData & 0xff);

					if (!AVAILABLE_EFUSE_ADDR_8812(eFuse_Addr))
						break;

					efuse_OneByteRead(rtlpriv, eFuse_Addr++, &efuseData);
					/* RT_DISP(FEEPROM, EFUSE_READ_ALL, ("efuse[%X]=%X\n", eFuse_Addr-1, efuseData)); */
					efuse_utilized++;
					eFuseWord[offset][i] |= (((u16)efuseData << 8) & 0xff00);

					if (!AVAILABLE_EFUSE_ADDR_8812(eFuse_Addr))
						break;
				}
			}
		}

		/* Read next PG header */
		efuse_OneByteRead(rtlpriv, eFuse_Addr++, &efuseHeader);
		/* RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Addr=%d rtemp 0x%x\n", eFuse_Addr, *rtemp8)); */

		if (efuseHeader != 0xFF) {
			efuse_utilized++;
		}
	}

	/*
	 * 3. Collect 16 sections and 4 word unit into Efuse map.
	 */
	for (i = 0; i < efuse_max_section; i++) {
		for (j = 0; j < EFUSE_MAX_WORD_UNIT; j++) {
			efuseTbl[(i*8)+(j*2)] = (eFuseWord[i][j] & 0xff);
			efuseTbl[(i*8)+((j*2)+1)] = ((eFuseWord[i][j] >> 8) & 0xff);
		}
	}

	/* RT_DISP(FEEPROM, EFUSE_READ_ALL, ("Hal_EfuseReadEFuse8812A(): efuse_utilized: %d\n", efuse_utilized)); */

	/*
	 * 4. Copy from Efuse map to output pointer memory!!!
	 */
	for (i = 0; i < _size_byte; i++) {
		pbuf[i] = efuseTbl[_offset+i];
	}

	/*
	 * 5. Calculate Efuse utilization.
	 */
	efuse_usage = (u8)((eFuse_Addr*100)/efuse_len);
	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_EFUSE_BYTES, (uint8_t *)&eFuse_Addr);

exit:
	if (efuseTbl)
		rtw_mfree(efuseTbl);

	if (eFuseWord)
		rtw_mfree2d((void *)eFuseWord, efuse_max_section, EFUSE_MAX_WORD_UNIT, sizeof(u16));
}

static void efuse_read_all_map(struct rtl_priv *rtlpriv, uint8_t *Efuse)
{
	efuse_power_switch(rtlpriv, false, true);
	read_efuse(rtlpriv, 0, rtlpriv->cfg->maps[EFUSE_HWSET_MAX_SIZE], Efuse);
	efuse_power_switch(rtlpriv, false, false);
}

uint8_t rtw_efuse_map_read(struct rtl_priv *rtlpriv, u16 addr, u16 cnts, uint8_t *data)
{
	if ((addr + cnts) > rtlpriv->cfg->maps[EFUSE_HWSET_MAX_SIZE])
		return _FAIL;

	efuse_power_switch(rtlpriv, false, true);
	read_efuse(rtlpriv, addr, cnts, data);
	efuse_power_switch(rtlpriv, false, false);

	return _SUCCESS;
}

/* ULLI rtw is used for not messing with rtlwifi */

void rtw_efuse_shadow_map_update(struct rtl_priv *rtlpriv)
{
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);

	if (efuse->autoload_failflag == true)
		memset(&efuse->efuse_map[0][0], 0xFF, rtlpriv->cfg->maps[EFUSE_HWSET_MAX_SIZE]);
	else
		efuse_read_all_map(rtlpriv, efuse->efuse_map[0]);
}

uint8_t
EFUSE_Read1Byte(
		struct rtl_priv *rtlpriv,
		u16		Address)
{
	uint8_t	data;
	uint8_t	Bytetemp = {0x00};
	uint8_t	temp = {0x00};
	uint32_t	k=0;
	u16	contentLen = rtlpriv->cfg->maps[EFUSE_REAL_CONTENT_SIZE];

	if (Address < contentLen)	//E-fuse 512Byte
	{
		//Write E-fuse Register address BIT(0)~7
		temp = Address & 0xFF;
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+1, temp);
		Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2);
		//Write E-fuse Register address bit8~9
		temp = ((Address >> 8) & 0x03) | (Bytetemp & 0xFC);
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2, temp);

		//Write 0x30[31]=0
		Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3);
		temp = Bytetemp & 0x7F;
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3, temp);

		//Wait Write-ready (0x30[31]=1)
		Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3);
		while(!(Bytetemp & 0x80))
		{
			Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3);
			k++;
			if(k==1000)
			{
				k=0;
				break;
			}
		}
		data=rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]);
		return data;
	}
	else
		return 0xFF;

}/* EFUSE_Read1Byte */

void
EFUSE_Write1Byte(
		struct rtl_priv *rtlpriv,
		u16		Address,
		uint8_t		Value)
{
	uint8_t	Bytetemp = {0x00};
	uint8_t	temp = {0x00};
	uint32_t	k=0;
	u16	contentLen = rtlpriv->cfg->maps[EFUSE_REAL_CONTENT_SIZE];

	if( Address < contentLen)	//E-fuse 512Byte
	{
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL], Value);

		//Write E-fuse Register address BIT(0)~7
		temp = Address & 0xFF;
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+1, temp);
		Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2);

		//Write E-fuse Register address bit8~9
		temp = ((Address >> 8) & 0x03) | (Bytetemp & 0xFC);
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2, temp);

		//Write 0x30[31]=1
		Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3);
		temp = Bytetemp | 0x80;
		rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3, temp);

		//Wait Write-ready (0x30[31]=0)
		Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3);
		while(Bytetemp & 0x80)
		{
			Bytetemp = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3);
			k++;
			if(k==100)
			{
				k=0;
				break;
			}
		}
	}
}/* EFUSE_Write1Byte */

static u8 Efuse_CalculateWordCnts(u8 word_en)
{
	uint8_t word_cnts = 0;
	if(!(word_en & BIT(0)))	word_cnts++; // 0 : write enable
	if(!(word_en & BIT(1)))	word_cnts++;
	if(!(word_en & BIT(2)))	word_cnts++;
	if(!(word_en & BIT(3)))	word_cnts++;
	return word_cnts;
}

static u16 efuse_get_current_size(struct rtl_priv *rtlpriv)
{
	int	bContinual = true;
	u16	efuse_addr = 0;
	uint8_t	hoffset = 0, hworden = 0;
	uint8_t	efuse_data, word_cnts = 0;

	rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_EFUSE_BYTES, (uint8_t *)&efuse_addr);

	/* RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfuseGetCurrentSize_8723A(), start_efuse_addr = %d\n", efuse_addr)); */

	while (bContinual && efuse_OneByteRead(rtlpriv, efuse_addr, &efuse_data)
	     && (efuse_addr  < EFUSE_REAL_CONTENT_LEN_JAGUAR)) {
		if (efuse_data != 0xFF) {
			if ((efuse_data & 0x1F) == 0x0F) {	/* extended header */
				hoffset = efuse_data;
				efuse_addr++;
				efuse_OneByteRead(rtlpriv, efuse_addr, &efuse_data);
				if ((efuse_data & 0x0F) == 0x0F) {
					efuse_addr++;
					continue;
				} else {
					hoffset = ((hoffset & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
					hworden = efuse_data & 0x0F;
				}
			} else {
				hoffset = (efuse_data >> 4) & 0x0F;
				hworden =  efuse_data & 0x0F;
			}
			word_cnts = Efuse_CalculateWordCnts(hworden);
			/* read next header */
			efuse_addr = efuse_addr + (word_cnts*2)+1;
		} else {
			bContinual = false ;
		}
	}

	rtlpriv->cfg->ops->set_hw_reg(rtlpriv, HW_VAR_EFUSE_BYTES, (uint8_t *)&efuse_addr);

	return efuse_addr;
}

static void efuse_WordEnableDataRead(u8 word_en, u8 *sourdata,
					u8 *targetdata)
{
	if (!(word_en&BIT(0)))
	{
		targetdata[0] = sourdata[0];
		targetdata[1] = sourdata[1];
	}
	if (!(word_en&BIT(1)))
	{
		targetdata[2] = sourdata[2];
		targetdata[3] = sourdata[3];
	}
	if (!(word_en&BIT(2)))
	{
		targetdata[4] = sourdata[4];
		targetdata[5] = sourdata[5];
	}
	if (!(word_en&BIT(3)))
	{
		targetdata[6] = sourdata[6];
		targetdata[7] = sourdata[7];
	}
}

static int efuse_pg_packet_read(struct rtl_priv *rtlpriv,
	uint8_t offset, uint8_t *data)
{
	uint8_t	ReadState = PG_STATE_HEADER;

	int	bContinual = true;
	int	bDataEmpty = true ;

	uint8_t	efuse_data, word_cnts = 0;
	u16 efuse_addr = 0;
	uint8_t	hoffset = 0, hworden = 0;
	uint8_t	tmpidx = 0;
	uint8_t	tmpdata[8];
	uint8_t	max_section = 0;
	uint8_t	tmp_header = 0;

	if (data == NULL)
		return false;
	if (offset > EFUSE_MAX_SECTION_JAGUAR)
		return false;

	memset((void *)data, 0xff, sizeof(uint8_t) * PGPKT_DATA_SIZE);
	memset((void *)tmpdata, 0xff, sizeof(uint8_t) * PGPKT_DATA_SIZE);


	/*
	 * <Roger_TODO> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP.
	 * Skip dummy parts to prevent unexpected data read from Efuse.
	 *  By pass right now. 2009.02.19.
	 */
	while (bContinual && (efuse_addr  < EFUSE_REAL_CONTENT_LEN_JAGUAR)) {
		/* -------  Header Read ------------- */
		if (ReadState & PG_STATE_HEADER) {
			if (efuse_OneByteRead(rtlpriv, efuse_addr, &efuse_data)
			    && (efuse_data != 0xFF)) {
				if (EXT_HEADER(efuse_data)) {
					tmp_header = efuse_data;
					efuse_addr++;
					efuse_OneByteRead(rtlpriv, efuse_addr, &efuse_data);
					if (!ALL_WORDS_DISABLED(efuse_data)) {
						hoffset = ((tmp_header & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
						hworden = efuse_data & 0x0F;
					} else {
#if 0	/* ULLI : temporary disabled */					
						DBG_8192C("Error, All words disabled\n");
#endif						
						efuse_addr++;
						break;
					}
				} else {
					hoffset = (efuse_data >> 4) & 0x0F;
					hworden =  efuse_data & 0x0F;
				}
				word_cnts = Efuse_CalculateWordCnts(hworden);
				bDataEmpty = true ;

				if (hoffset == offset) {
					for (tmpidx = 0; tmpidx < word_cnts*2; tmpidx++) {
						if (efuse_OneByteRead(rtlpriv, efuse_addr+1+tmpidx, &efuse_data)) {
							tmpdata[tmpidx] = efuse_data;
							if (efuse_data != 0xff) {
								bDataEmpty = false;
							}
						}
					}
					if (bDataEmpty == false) {
						ReadState = PG_STATE_DATA;
					} else {	/* read next header */
						efuse_addr = efuse_addr + (word_cnts*2)+1;
						ReadState = PG_STATE_HEADER;
					}
				} else {	/*read next header */
					efuse_addr = efuse_addr + (word_cnts*2)+1;
					ReadState = PG_STATE_HEADER;
				}

			} else{
				bContinual = false ;
			}
		} else if (ReadState & PG_STATE_DATA) {
			/* -------  Data section Read ------------- */
			efuse_WordEnableDataRead(hworden, tmpdata, data);
			efuse_addr = efuse_addr + (word_cnts*2)+1;
			ReadState = PG_STATE_HEADER;
		}

	}

	if ((data[0] == 0xff) && (data[1] == 0xff) && (data[2] == 0xff) &&
	    (data[3] == 0xff) && (data[4] == 0xff) && (data[5] == 0xff) &&
	    (data[6] == 0xff) && (data[7] == 0xff))
		return false;
	else
		return true;

}

static u8 efuse_word_enable_data_write(struct rtl_priv *rtlpriv,
	u16 efuse_addr, uint8_t word_en, uint8_t *data);


static int efuse_pg_packet_write(struct rtl_priv *rtlpriv, uint8_t offset,
	uint8_t	word_en, uint8_t *data)
{
	uint8_t WriteState = PG_STATE_HEADER;

	int bContinual = true, bDataEmpty = true;
	/* int bResult = true; */
	u16 efuse_addr = 0;
	uint8_t	efuse_data;

	uint8_t	pg_header = 0, pg_header_temp = 0;

	uint8_t	tmp_word_cnts = 0, target_word_cnts = 0;
	uint8_t	tmp_header, match_word_en, tmp_word_en;

	struct pgpkt_struct target_pkt;
	struct pgpkt_struct tmp_pkt;

	uint8_t	originaldata[sizeof(uint8_t) * 8];
	uint8_t	tmpindex = 0, badworden = 0x0F;

	static int repeat_times = 0;

	bool	bExtendedHeader = false;

	/*
	 * <Roger_Notes> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP.
	 * So we have to prevent unexpected data string connection, which will cause
	 * incorrect data auto-load from HW. The total size is equal or smaller than 498bytes
	 * (i.e., offset 0~497, and dummy 1bytes) expected after CP test.
	 * 2009.02.19.
	 */
#if 0	 /* ULLI : temporary disabled */
	if (rtl8812_EfuseGetCurrentSize(rtlpriv) >= (EFUSE_REAL_CONTENT_LEN_JAGUAR-EFUSE_OOB_PROTECT_BYTES_JAGUAR)) {
		DBG_871X("hal_EfusePgPacketWrite_8812A() error: %x >= %x\n", rtl8812_EfuseGetCurrentSize(rtlpriv), (EFUSE_REAL_CONTENT_LEN_JAGUAR-EFUSE_OOB_PROTECT_BYTES_JAGUAR));
		return false;
	}
#endif
	/* Init the 8 bytes content as 0xff */
	target_pkt.offset = offset;
	target_pkt.word_en = word_en;
	/* Initial the value to avoid compile warning */
	tmp_pkt.offset = 0;
	tmp_pkt.word_en = 0;

	/* DBG_871X("hal_EfusePgPacketWrite_8812A target offset 0x%x word_en 0x%x \n", target_pkt.offset, target_pkt.word_en); */

	memset((void *)target_pkt.data, 0xFF, sizeof(uint8_t)*8);

	efuse_WordEnableDataRead(word_en, data, target_pkt.data);
	target_word_cnts = Efuse_CalculateWordCnts(target_pkt.word_en);

	/*
	 * <Roger_Notes> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP.
	 * So we have to prevent unexpected data string connection, which will cause
	 * incorrect data auto-load from HW. Dummy 1bytes is additional.
	 * 2009.02.19.
	 */
	while (bContinual && (efuse_addr  < (EFUSE_REAL_CONTENT_LEN_JAGUAR-EFUSE_OOB_PROTECT_BYTES_JAGUAR))) {
		if (WriteState == PG_STATE_HEADER) {
			bDataEmpty = true;
			badworden = 0x0F;
			/* ************	so ******************* */
			/* DBG_871X("EFUSE PG_STATE_HEADER\n"); */
			if (efuse_OneByteRead(rtlpriv, efuse_addr, &efuse_data) &&
			   (efuse_data != 0xFF)) {
				if ((efuse_data&0x1F) == 0x0F) {		/* extended header */
					tmp_header = efuse_data;
					efuse_addr++;
					efuse_OneByteRead(rtlpriv, efuse_addr, &efuse_data);
					if ((efuse_data & 0x0F) == 0x0F) {	/* wren fail */
						uint8_t next = 0, next_next = 0, data = 0, i = 0;
						uint8_t s = ((tmp_header & 0xF0) >> 4);
						efuse_OneByteRead(rtlpriv, efuse_addr+1, &next);
						efuse_OneByteRead(rtlpriv, efuse_addr+2, &next_next);
						if (next == 0xFF && next_next == 0xFF) {
							/* Have enough space to make fake data to recover bad header. */
							switch (s) {
							case 0x0:
							case 0x2:
							case 0x4:
							case 0x6:
							case 0x8:
							case 0xA:
							case 0xC:
								for (i = 0; i < 3; ++i) {
								efuse_OneByteWrite(rtlpriv, efuse_addr, 0x27);
									efuse_OneByteRead(rtlpriv, efuse_addr, &data);
									if (data == 0x27)
										break;
								}
								break;
							case 0xE:
								for (i = 0; i < 3; ++i) {
								efuse_OneByteWrite(rtlpriv, efuse_addr, 0x17);
									efuse_OneByteRead(rtlpriv, efuse_addr, &data);
									if (data == 0x17)
										break;
								}
								break;
							default:
								break;
							}
							efuse_OneByteWrite(rtlpriv, efuse_addr+1, 0xFF);
							efuse_OneByteWrite(rtlpriv, efuse_addr+2, 0xFF);
							efuse_addr += 3;
						} else {
							efuse_addr++;
						}
						continue;
					} else {
						tmp_pkt.offset = ((tmp_header & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
						tmp_pkt.word_en = efuse_data & 0x0F;
					}
				} else {
					uint8_t i = 0, data = 0;
					tmp_header	=  efuse_data;
					tmp_pkt.offset	= (tmp_header>>4) & 0x0F;
					tmp_pkt.word_en = tmp_header & 0x0F;

					if (tmp_pkt.word_en == 0xF) {
						uint8_t next = 0;
						efuse_OneByteRead(rtlpriv, efuse_addr+1, &next);
						if (next == 0xFF) { 	/* Have enough space to make fake data to recover bad header. */
							tmp_header = (tmp_header & 0xF0) | 0x7;
							for (i = 0; i < 3; ++i) {
							efuse_OneByteWrite(rtlpriv, efuse_addr, tmp_header);
								efuse_OneByteRead(rtlpriv, efuse_addr, &data);
								if (data == tmp_header)
									break;
							}
							efuse_OneByteWrite(rtlpriv, efuse_addr+1, 0xFF);
							efuse_OneByteWrite(rtlpriv, efuse_addr+2, 0xFF);
							efuse_addr += 2;
						}
					}
				}
				tmp_word_cnts =  Efuse_CalculateWordCnts(tmp_pkt.word_en);

				/* DBG_871X("section offset 0x%x worden 0x%x\n", tmp_pkt.offset, tmp_pkt.word_en); */

				/* ************	so-1 ******************* */
				if (tmp_pkt.offset  != target_pkt.offset) {
					efuse_addr = efuse_addr + (tmp_word_cnts * 2) + 1; /* Next pg_packet */
#if (EFUSE_ERROE_HANDLE == 1)
					WriteState = PG_STATE_HEADER;
#endif
				} else {
					/* write the same offset */
					/* DBG_871X("hal_EfusePgPacketWrite_8812A section offset the same\n"); */
					/* ************	so-2 ******************* */
					for (tmpindex = 0; tmpindex < (tmp_word_cnts * 2) ; tmpindex++) {
						if (efuse_OneByteRead(rtlpriv, (efuse_addr + 1 + tmpindex), &efuse_data) && (efuse_data != 0xFF)) {
							bDataEmpty = false;
						}
					}
					/* ***********	so-2-1 ******************* */
					if (bDataEmpty == false) {
						/* DBG_871X("hal_EfusePgPacketWrite_8812A section offset the same and data is NOT empty\n"); */
						efuse_addr = efuse_addr + (tmp_word_cnts*2) + 1; /* Next pg_packet */
#if (EFUSE_ERROE_HANDLE == 1)
						WriteState = PG_STATE_HEADER;
#endif
					} else {
						/* ************  so-2-2 ******************* */
						/* DBG_871X("hal_EfusePgPacketWrite_8812A section data empty\n"); */
						match_word_en = 0x0F;			/* same bit as original wren */
						if (!((target_pkt.word_en & BIT(0)) | (tmp_pkt.word_en & BIT(0)))) {
							 match_word_en &= (~BIT(0));
						}
						if (!((target_pkt.word_en & BIT(1)) | (tmp_pkt.word_en & BIT(1)))) {
							 match_word_en &= (~BIT(1));
						}
						if (!((target_pkt.word_en & BIT(2)) | (tmp_pkt.word_en & BIT(2)))) {
							 match_word_en &= (~BIT(2));
						}
						if (!((target_pkt.word_en & BIT(3)) | (tmp_pkt.word_en & BIT(3)))) {
							 match_word_en &= (~BIT(3));
						}

						/* ***********	so-2-2-A ******************* */
						if ((match_word_en&0x0F) != 0x0F) {
							badworden = efuse_word_enable_data_write(rtlpriv, efuse_addr + 1, tmp_pkt.word_en, target_pkt.data);

							/************	so-2-2-A-1 ******************* */
							/* ############################ */
							if (0x0F != (badworden & 0x0F)) {
								uint8_t	reorg_offset = offset;
								uint8_t	reorg_worden = badworden;
								efuse_pg_packet_write(rtlpriv, reorg_offset, reorg_worden, target_pkt.data);
							}
							/* ############################ */

							tmp_word_en = 0x0F;		/* not the same bit as original wren */
							if ((target_pkt.word_en&BIT(0)) ^ (match_word_en&BIT(0))) {
								tmp_word_en &= (~BIT(0));
							}
							if ((target_pkt.word_en&BIT(1)) ^ (match_word_en&BIT(1))) {
								tmp_word_en &=	(~BIT(1));
							}
							if ((target_pkt.word_en&BIT(2)) ^ (match_word_en&BIT(2))) {
								tmp_word_en &= (~BIT(2));
							}
							if ((target_pkt.word_en&BIT(3)) ^ (match_word_en&BIT(3))) {
								tmp_word_en &= (~BIT(3));
							}

							/* ************	so-2-2-A-2 ******************* */
							if ((tmp_word_en & 0x0F) != 0x0F) {
								/* reorganize other pg packet */
								/* efuse_addr = efuse_addr + (2*tmp_word_cnts) +1;//next pg packet addr */
								efuse_addr = efuse_get_current_size(rtlpriv);
								/* =========================== */
								target_pkt.offset = offset;
								target_pkt.word_en = tmp_word_en;
								/* =========================== */
							} else {
								bContinual = false;
							}
#if (EFUSE_ERROE_HANDLE == 1)
							WriteState = PG_STATE_HEADER;
							repeat_times++;
							if (repeat_times > EFUSE_REPEAT_THRESHOLD_) {
								bContinual = false;
								/* bResult = false; */
							}
#endif
						} else{
							/* ************  so-2-2-B ******************* */
							/* reorganize other pg packet */
							efuse_addr = efuse_addr + (2 * tmp_word_cnts) + 1;	/* next pg packet addr */
							/* =========================== */
							target_pkt.offset = offset;
							target_pkt.word_en = target_pkt.word_en;
							/* =========================== */
#if (EFUSE_ERROE_HANDLE == 1)
							WriteState = PG_STATE_HEADER;
#endif
						}
					}
				}
#if 0	/* ULLI : temporary disabled */	
				DBG_871X("EFUSE PG_STATE_HEADER-1\n");
#endif				
			} else	{
				/* ***********	s1: header == oxff	******************* */
				bExtendedHeader = false;

				if (target_pkt.offset >= EFUSE_MAX_SECTION_BASE) {
					pg_header = ((target_pkt.offset & 0x07) << 5) | 0x0F;

					/* DBG_871X("hal_EfusePgPacketWrite_8812A extended pg_header[2:0] |0x0F 0x%x \n", pg_header); */

					efuse_OneByteWrite(rtlpriv, efuse_addr, pg_header);
					efuse_OneByteRead(rtlpriv, efuse_addr, &tmp_header);

					while (tmp_header == 0xFF) {
						/* DBG_871X("hal_EfusePgPacketWrite_8812A extended pg_header[2:0] wirte fail \n"); */

						repeat_times++;

						if (repeat_times > EFUSE_REPEAT_THRESHOLD_) {
							bContinual = false;
							/* bResult = false; */
							efuse_addr++;
							break;
						}
						efuse_OneByteWrite(rtlpriv, efuse_addr, pg_header);
						efuse_OneByteRead(rtlpriv, efuse_addr, &tmp_header);
					}

					if (!bContinual)
						break;

					if (tmp_header == pg_header) {
						efuse_addr++;
						pg_header_temp = pg_header;
						pg_header = ((target_pkt.offset & 0x78) << 1) | target_pkt.word_en;

						/* DBG_871X("hal_EfusePgPacketWrite_8812A extended pg_header[6:3] | worden 0x%x word_en 0x%x \n", pg_header, target_pkt.word_en); */

						efuse_OneByteWrite(rtlpriv, efuse_addr, pg_header);
						efuse_OneByteRead(rtlpriv, efuse_addr, &tmp_header);

						while (tmp_header == 0xFF) {
							repeat_times++;

							if (repeat_times > EFUSE_REPEAT_THRESHOLD_) {
								bContinual = false;
								/* bResult = false; */
								break;
							}
							efuse_OneByteWrite(rtlpriv, efuse_addr, pg_header);
							efuse_OneByteRead(rtlpriv, efuse_addr, &tmp_header);
						}

						if (!bContinual)
							break;

						if ((tmp_header & 0x0F) == 0x0F) {
							/* wren PG fail */
							repeat_times++;

							if (repeat_times > EFUSE_REPEAT_THRESHOLD_) {
								bContinual = false;
								/* bResult = false; */
								break;
							} else {
								efuse_addr++;
								continue;
							}
						} else if (pg_header != tmp_header) {	/* offset PG fail */
							bExtendedHeader = true;
							tmp_pkt.offset = ((pg_header_temp & 0xE0) >> 5) | ((tmp_header & 0xF0) >> 1);
							tmp_pkt.word_en =  tmp_header & 0x0F;
							tmp_word_cnts =  Efuse_CalculateWordCnts(tmp_pkt.word_en);
						}
					} else if ((tmp_header & 0x1F) == 0x0F) {	/* wrong extended header */
						efuse_addr += 2;
						continue;
					}
				} else {
					pg_header = ((target_pkt.offset << 4) & 0xf0) | target_pkt.word_en;
					efuse_OneByteWrite(rtlpriv, efuse_addr, pg_header);
					efuse_OneByteRead(rtlpriv, efuse_addr, &tmp_header);
				}

				if (tmp_header == pg_header) {
					/* ***********  s1-1******************* */
					WriteState = PG_STATE_DATA;
				}
#if (EFUSE_ERROE_HANDLE == 1)
				else if (tmp_header == 0xFF) {
					/************	s1-3: if Write or read func doesn't work ******************* */
					/* efuse_addr doesn't change */
					WriteState = PG_STATE_HEADER;
					repeat_times++;
					if (repeat_times > EFUSE_REPEAT_THRESHOLD_) {
						bContinual = false;
						/* bResult = false; */
					}
				}
#endif
				else {
					/* ***********  s1-2 : fixed the header procedure ******************* */
					if (!bExtendedHeader) {
						tmp_pkt.offset = (tmp_header >> 4) & 0x0F;
						tmp_pkt.word_en =  tmp_header & 0x0F;
						tmp_word_cnts =  Efuse_CalculateWordCnts(tmp_pkt.word_en);
					}

					/* ************	s1-2-A :cover the exist data ******************* */
					memset(originaldata, 0xff, sizeof(uint8_t) * 8);

					if (efuse_pg_packet_read(rtlpriv, tmp_pkt.offset, originaldata)) {
						/* check if data exist */
						badworden = efuse_word_enable_data_write(rtlpriv, efuse_addr+1, tmp_pkt.word_en, originaldata);
						/* ############################ */
						if (0x0F != (badworden & 0x0F)) {
							uint8_t	reorg_offset = tmp_pkt.offset;
							uint8_t	reorg_worden = badworden;
							efuse_pg_packet_write(rtlpriv, reorg_offset, reorg_worden, originaldata);
							efuse_addr = efuse_get_current_size(rtlpriv);
						} else {
							/* ############################ */
							efuse_addr = efuse_addr + (tmp_word_cnts*2) + 1; /* Next pg_packet */
						}
					} else {
						/* ************  s1-2-B: wrong address******************* */
						efuse_addr = efuse_addr + (tmp_word_cnts*2) + 1; /* Next pg_packet */
					}

#if (EFUSE_ERROE_HANDLE == 1)
					WriteState = PG_STATE_HEADER;
					repeat_times++;
					if (repeat_times > EFUSE_REPEAT_THRESHOLD_) {
						bContinual = false;
						/* bResult = false; */
					}
#endif

					/* DBG_871X("EFUSE PG_STATE_HEADER-2\n"); */
				}

			}

		}
		/* write data state */
		else if (WriteState == PG_STATE_DATA) {
			/* ************	s1-1  ******************* */
			/* DBG_871X("EFUSE PG_STATE_DATA\n"); */
			badworden = 0x0f;
			badworden = efuse_word_enable_data_write(rtlpriv, efuse_addr+1, target_pkt.word_en, target_pkt.data);
			if ((badworden & 0x0F) == 0x0F) {
				/* ************  s1-1-A ******************* */
				bContinual = false;
			} else {
				/* reorganize other pg packet  */
				/* ***********  s1-1-B ******************* */
				efuse_addr = efuse_addr + (2 * target_word_cnts) + 1;	/* next pg packet addr */

				/* =========================== */
				target_pkt.offset = offset;
				target_pkt.word_en = badworden;
				target_word_cnts = Efuse_CalculateWordCnts(target_pkt.word_en);
				/* =========================== */
#if (EFUSE_ERROE_HANDLE == 1)
				WriteState = PG_STATE_HEADER;
				repeat_times++;
				if (repeat_times > EFUSE_REPEAT_THRESHOLD_) {
					bContinual = false;
					/* bResult = false; */
				}
#endif
				/* DBG_871X("EFUSE PG_STATE_HEADER-3\n"); */
			}
		}
	}

#if 0	/* ULLI : temporary disabled */
	if (efuse_addr >= (EFUSE_REAL_CONTENT_LEN_JAGUAR-EFUSE_OOB_PROTECT_BYTES_JAGUAR)) {
		DBG_871X("hal_EfusePgPacketWrite_8812A(): efuse_addr(%#x) Out of size!!\n", efuse_addr);
	}
#endif	
	return true;
}

static u8 efuse_word_enable_data_write(struct rtl_priv *rtlpriv,
	u16 efuse_addr, uint8_t word_en, uint8_t *data)
{
	u16 tmpaddr = 0;
	u16 start_addr = efuse_addr;
	uint8_t	badworden = 0x0F;
	uint8_t	tmpdata[8];

	memset((void *)tmpdata, 0xff, PGPKT_DATA_SIZE);

	/*
	 * RT_TRACE(COMP_EFUSE, DBG_LOUD, ("word_en = %x efuse_addr=%x\n", word_en, efuse_addr));
	 */

	if (!(word_en & BIT(0))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(rtlpriv, start_addr++, data[0]);
		efuse_OneByteWrite(rtlpriv, start_addr++, data[1]);

		efuse_OneByteRead(rtlpriv, tmpaddr, &tmpdata[0]);
		efuse_OneByteRead(rtlpriv, tmpaddr+1, &tmpdata[1]);
		if ((data[0] != tmpdata[0]) || (data[1] != tmpdata[1])) {
			badworden &= (~BIT(0));
		}
	}
	if (!(word_en & BIT(1))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(rtlpriv, start_addr++, data[2]);
		efuse_OneByteWrite(rtlpriv, start_addr++, data[3]);

		efuse_OneByteRead(rtlpriv, tmpaddr    , &tmpdata[2]);
		efuse_OneByteRead(rtlpriv, tmpaddr+1, &tmpdata[3]);
		if ((data[2] != tmpdata[2]) || (data[3] != tmpdata[3])) {
			badworden &= (~BIT(1));
		}
	}
	if (!(word_en & BIT(2))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(rtlpriv, start_addr++, data[4]);
		efuse_OneByteWrite(rtlpriv, start_addr++, data[5]);

		efuse_OneByteRead(rtlpriv, tmpaddr, &tmpdata[4]);
		efuse_OneByteRead(rtlpriv, tmpaddr+1, &tmpdata[5]);
		if ((data[4] != tmpdata[4]) || (data[5] != tmpdata[5])) {
			badworden &= (~BIT(2));
		}
	}
	if (!(word_en & BIT(3))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(rtlpriv, start_addr++, data[6]);
		efuse_OneByteWrite(rtlpriv, start_addr++, data[7]);

		efuse_OneByteRead(rtlpriv, tmpaddr, &tmpdata[6]);
		efuse_OneByteRead(rtlpriv, tmpaddr+1, &tmpdata[7]);
		if ((data[6] != tmpdata[6]) || (data[7] != tmpdata[7])) {
			badworden &= (~BIT(3));
		}
	}
	return badworden;
}

void
ReadEFuseByte(
		struct rtl_priv *rtlpriv,
		u16 			_offset,
		uint8_t 			*pbuf)
{
	uint32_t	value32;
	uint8_t	readbyte;
	u16	retry;
	//uint32_t start=jiffies;

	//Write Address
	rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+1, (_offset & 0xff));
	readbyte = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2);
	rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+2, ((_offset >> 8) & 0x03) | (readbyte & 0xfc));

	//Write bit 32 0
	readbyte = rtl_read_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3);
	rtl_write_byte(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]+3, (readbyte & 0x7f));

	//Check bit 32 read-ready
	retry = 0;
	value32 = rtl_read_dword(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]);
	//while(!(((value32 >> 24) & 0xff) & 0x80)  && (retry<10))
	while(!(((value32 >> 24) & 0xff) & 0x80)  && (retry<10000))
	{
		value32 = rtl_read_dword(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]);
		retry++;
	}

	// 20100205 Joseph: Add delay suggested by SD1 Victor.
	// This fix the problem that Efuse read error in high temperature condition.
	// Designer says that there shall be some delay after ready bit is set, or the
	// result will always stay on last data we read.
	udelay(50);
	value32 = rtl_read_dword(rtlpriv, rtlpriv->cfg->maps[EFUSE_CTRL]);

	*pbuf = (uint8_t)(value32 & 0xff);
	//DBG_871X("ReadEFuseByte _offset:%08u, in %d ms\n",_offset ,rtw_get_passing_time_ms(start));

}
