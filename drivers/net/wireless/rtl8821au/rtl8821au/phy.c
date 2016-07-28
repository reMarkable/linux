#ifdef CONFIG_RTLWIFI

#include <../drivers/net/wireless/realtek/rtlwifi/wifi.h>
#include <../drivers/net/wireless/realtek/rtlwifi/ps.h>
#include <../drivers/net/wireless/realtek/rtlwifi/base.h>

#else

#include <drv_types.h>
#include <rtl8812a_hal.h>
#include "phy.h"
#include "dm.h"
#include "table.h"
#include "rf.h"
#include "def.h"
#include "reg.h"

#endif

/*
 * 1. BB register R/W API
 */

#define TX_PWR_BY_RATE_NUM_RATE			84

u8 _rtl8821au_get_txpower_index_base(struct rtl_priv *rtlpriv,
				     u8 RFPath, u8 Rate,
				     enum CHANNEL_WIDTH Bandwidth,
				     u8 Channel,
				     bool *bIn24G);

static u8 _rtl8821au_phy_get_txpower_by_rate_base(struct rtl_priv *rtlpriv,
					     u8 band,
					     u8 path,
					     u8 txnum, u8 rate_section);
					     
static s8 _rtl8821au_phy_get_txpower_by_rate(struct rtl_priv *rtlpriv,
					     u8 Band, u8 RFPath,
					     u8 TxNum, u8 Rate);

static bool CheckPositive(struct rtl_priv *rtlpriv,
			  uint32_t Condition1, uint32_t Condition2)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	u8 _BoardType = ((rtlhal->board_type & ODM_BOARD_EXT_LNA) ? 0x01 : 0) |
			((rtlhal->board_type & ODM_BOARD_EXT_PA) ?  0x02 : 0) |
			((rtlhal->board_type & ODM_BOARD_EXT_LNA_5G) ? 0x04 : 0) |
			((rtlhal->board_type & ODM_BOARD_EXT_PA_5G) ? 0x10 : 0) |
			/* ULLI : would be zero, but curently here for consistence */
			((rtlhal->board_type & ODM_BOARD_BT) ? 0x10 : 0);
#if 0
	/* ULLI : From AWUSB driver
	 * pDM_Odm->SupportInterface : RTW_USB 	 BIT(1),
	 * pDM_Odm->SupportPlatform  : ODM_CE     0x04
	 */
#endif
	uint32_t cond1   = Condition1, cond2 = Condition2;
	uint32_t driver1 = NUM_CUT_VERSION(rtlhal->version) << 24 |
		           ODM_CE << 16 |
		           /* ULLI : pDM_Odm->PackageType      << 12 |  is zero */
		           RTW_USB << 8  |
		           _BoardType;
	uint32_t driver2 = rtlhal->lna_type_2g <<  0 |
		           rtlhal->pa_type_2g  <<  8 |
		           rtlhal->lna_type_5g << 16 |
		           rtlhal->lna_type_5g  << 24;

	/*
	 * ============== Value Defined Check ===============
	 * QFN Type [15:12] and Cut Version [27:24] need to do value check
	 */

	if (((cond1 & 0x0000F000) != 0) &&
	    ((cond1 & 0x0000F000) != (driver1 & 0x0000F000)))
		return false;
		
	if (((cond1 & 0x0F000000) != 0) &&
	    ((cond1 & 0x0F000000) != (driver1 & 0x0F000000)))
		return false;

	/*
	 * =============== Bit Defined Check ================
	 * We don't care [31:28] and [23:20]
	 */

	cond1   &= 0x000F0FFF;
	driver1 &= 0x000F0FFF;

	if ((cond1 & driver1) == cond1) {
		uint32_t bitMask = 0;
		if ((cond1 & 0x0F) == 0) /* BoardType is DONTCARE */
			return true;

		if ((cond1 & BIT(0)) != 0) /* GLNA */
			bitMask |= 0x000000FF;
		if ((cond1 & BIT(1)) != 0) /* GPA */
			bitMask |= 0x0000FF00;
		if ((cond1 & BIT(2)) != 0) /* ALNA */
			bitMask |= 0x00FF0000;
		if ((cond1 & BIT(3)) != 0) /* APA */
			bitMask |= 0xFF000000;

		if ((cond2 & bitMask) == (driver2 & bitMask)) /* BoardType of each RF path is matched */
			return false;
		else
			return false;
	} else {
		return false;
	}

	return false;
}

#define READ_NEXT_PAIR(array_table, v1, v2, i) \
	do { \
		i += 2; \
		v1 = array_table[i]; \
		v2 = array_table[i+1]; \
	} while (0)

static u32 _rtl8821au_phy_calculate_bit_shift(uint32_t BitMask)
{
	uint32_t i;

	for (i = 0; i <= 31; i++) {
		if (((BitMask >> i) & 0x1) == 1)
			break;
	}

	return i;
}

u32 rtl8821au_phy_query_bb_reg(struct rtl_priv *rtlpriv, uint32_t RegAddr, uint32_t BitMask)
{
	uint32_t ReturnValue = 0, OriginalValue, BitShift;

	/* DBG_871X("--->PHY_QueryBBReg8812(): RegAddr(%#x), BitMask(%#x)\n", RegAddr, BitMask); */


	OriginalValue = rtl_read_dword(rtlpriv, RegAddr);
	BitShift = _rtl8821au_phy_calculate_bit_shift(BitMask);
	ReturnValue = (OriginalValue & BitMask) >> BitShift;

	/* DBG_871X("BBR MASK=0x%x Addr[0x%x]=0x%x\n", BitMask, RegAddr, OriginalValue); */
	return ReturnValue;
}


void rtl8821au_phy_set_bb_reg(struct rtl_priv *rtlpriv, u32 RegAddr, u32 BitMask, u32 Data)
{
	uint32_t OriginalValue, BitShift;

	if (BitMask != bMaskDWord) {	/* if not "double word" write */
		OriginalValue = rtl_read_dword(rtlpriv, RegAddr);
		BitShift = _rtl8821au_phy_calculate_bit_shift(BitMask);
		Data = ((OriginalValue) & (~BitMask)) | (((Data << BitShift)) & BitMask);
	}

	rtl_write_dword(rtlpriv, RegAddr, Data);

	/* DBG_871X("BBW MASK=0x%x Addr[0x%x]=0x%x\n", BitMask, RegAddr, Data); */
}

static u32 _rtl8821au_phy_rf_serial_read(struct rtl_priv *rtlpriv, uint8_t eRFPath,
	uint32_t Offset)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct bb_reg_def *pphyreg = &(rtlpriv->phy.phyreg_def[eRFPath]);

	uint32_t			retValue = 0;
	bool				bIsPIMode = false;


	/*
	 * 2009/06/17 MH We can not execute IO for power save or other accident mode.
	 * if(RT_CANNOT_IO(rtlpriv)) {
	 * 	RT_DISP(FPHY, PHY_RFR, ("phy_RFSerialRead return all one\n"));
	 * 	return	0xFFFFFFFF;
	 * }
	 */

	/* <20120809, Kordan> CCA OFF(when entering), asked by James to avoid reading the wrong value. */
	/* <20120828, Kordan> Toggling CCA would affect RF 0x0, skip it! */
	if (Offset != 0x0 &&  !(IS_VENDOR_8812A_C_CUT(rtlhal->version) || IS_HARDWARE_TYPE_8821(rtlhal)))
		rtl_set_bbreg(rtlpriv, rCCAonSec_Jaguar, 0x8, 1);

	Offset &= 0xff;

	if (eRFPath == RF90_PATH_A)
		bIsPIMode = (bool)rtl_get_bbreg(rtlpriv, 0xC00, 0x4);
	else if (eRFPath == RF90_PATH_B)
		bIsPIMode = (bool)rtl_get_bbreg(rtlpriv, 0xE00, 0x4);

	if (IS_VENDOR_8812A_TEST_CHIP(rtlhal->version))
		rtl_set_bbreg(rtlpriv, pphyreg->rfhssi_para2, bMaskDWord, 0);

	rtl_set_bbreg(rtlpriv, pphyreg->rfhssi_para2, bHSSIRead_addr_Jaguar, Offset);

	if (IS_VENDOR_8812A_TEST_CHIP(rtlhal->version) )
		rtl_set_bbreg(rtlpriv, pphyreg->rfhssi_para2, bMaskDWord, Offset|BIT(8));

	if (IS_VENDOR_8812A_C_CUT(rtlhal->version) || IS_HARDWARE_TYPE_8821(rtlhal))
		udelay(20);

	if (bIsPIMode) {
		retValue = rtl_get_bbreg(rtlpriv, pphyreg->rf_rbpi, rRead_data_Jaguar);
		/* DBG_871X("[PI mode] RFR-%d Addr[0x%x]=0x%x\n", eRFPath, pPhyReg->rfLSSIReadBackPi, retValue); */
	} else {
		retValue = rtl_get_bbreg(rtlpriv, pphyreg->rf_rb, rRead_data_Jaguar);
		/* DBG_871X("[SI mode] RFR-%d Addr[0x%x]=0x%x\n", eRFPath, pPhyReg->rfLSSIReadBack, retValue); */
	}

	/* <20120809, Kordan> CCA ON(when exiting), asked by James to avoid reading the wrong value. */
	/* <20120828, Kordan> Toggling CCA would affect RF 0x0, skip it! */
	if (Offset != 0x0 &&  ! (IS_VENDOR_8812A_C_CUT(rtlhal->version) || IS_HARDWARE_TYPE_8821(rtlhal)))
		rtl_set_bbreg(rtlpriv, rCCAonSec_Jaguar, 0x8, 0);

	return retValue;
}

static void _rtl8821au_phy_rf_serial_write(struct rtl_priv *rtlpriv, uint8_t eRFPath,
	uint32_t Offset, uint32_t Data)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct bb_reg_def *pphyreg = &(rtlpriv->phy.phyreg_def[eRFPath]);
	uint32_t		DataAndAddr = 0;

	/*
	 * 2009/06/17 MH We can not execute IO for power save or other accident mode.
	 * if(RT_CANNOT_IO(rtlpriv)) {
	 * 	RTPRINT(FPHY, PHY_RFW, ("phy_RFSerialWrite stop\n"));
	 * 	return;
	 * }
	 */

	Offset &= 0xff;

	// Shadow Update
	//PHY_RFShadowWrite(rtlpriv, eRFPath, Offset, Data);

	// Put write addr in [27:20]  and write data in [19:00]
	DataAndAddr = ((Offset<<20) | (Data&0x000fffff)) & 0x0fffffff;

	/*
	 * 3 <Note> This is a workaround for 8812A test chips.
	 * <20120427, Kordan> MAC first moves lower 16 bits and then upper 16 bits of a 32-bit data.
	 * BaseBand doesn't know the two actions is actually only one action to access 32-bit data,
	 * so that the lower 16 bits is overwritten by the upper 16 bits. (Asked by ynlin.)
	 * (Unfortunately, the protection mechanism has not been implemented in 8812A yet.)
	 * 2012/10/26 MH Revise V3236 Lanhsin check in, if we do not enable the function
	 * for 8821, then it can not scan.
	 */
	if ((!IF_RTL8821AU_USB3_MODE(rtlpriv->rtlhal.version)) &&
	    (!IS_NORMAL_CHIP(rtlhal->version))) {	/* USB 2.0 or older */
		/* if (IS_VENDOR_8812A_TEST_CHIP(rtlpriv) || IS_HARDWARE_TYPE_8821(rtlpriv) is) */
		{
			rtl_write_dword(rtlpriv, 0x1EC, DataAndAddr);
			if (eRFPath == RF90_PATH_A)
				rtl_write_dword(rtlpriv, 0x1E8, 0x4000F000|0xC90);
			else
				rtl_write_dword(rtlpriv, 0x1E8, 0x4000F000|0xE90);
		}
	} else {
		/* USB 3.0 */
		/* Write Operation */
		/* TODO: Dynamically determine whether using PI or SI to write RF registers. */
		rtl_set_bbreg(rtlpriv, pphyreg->rf3wire_offset, bMaskDWord, DataAndAddr);
		/* DBG_871X("RFW-%d Addr[0x%x]=0x%x\n", eRFPath, pPhyReg->rf3wireOffset, DataAndAddr); */
	}

}


void rtl8821au_phy_set_rf_reg(struct rtl_priv *rtlpriv, u32 eRFPath, u32 RegAddr,
	u32 BitMask, u32 Data)
{
	if (BitMask == 0)
		return;

	/* RF data is 20 bits only */
	if (BitMask != bLSSIWrite_data_Jaguar) {
		uint32_t	Original_Value, BitShift;
		Original_Value =  _rtl8821au_phy_rf_serial_read(rtlpriv, eRFPath, RegAddr);
		BitShift =  _rtl8821au_phy_calculate_bit_shift(BitMask);
		Data = ((Original_Value) & (~BitMask)) | (Data<< BitShift);
	}

	 _rtl8821au_phy_rf_serial_write(rtlpriv, eRFPath, RegAddr, Data);

}

u32 rtl8821au_phy_query_rf_reg(struct rtl_priv *rtlpriv, u32 eRFPath, u32 RegAddr,
	u32 BitMask)
{
	u32 Original_Value, Readback_Value, BitShift;

	Original_Value =  _rtl8821au_phy_rf_serial_read(rtlpriv, eRFPath, RegAddr);

	BitShift =  _rtl8821au_phy_calculate_bit_shift(BitMask);
	Readback_Value = (Original_Value & BitMask) >> BitShift;

	return (Readback_Value);
}

/* ****************************************************************************** */
/*									*/
/*  from HalPhyRf_8812A.c						*/
/*									*/
/* ****************************************************************************** */

static void _rtl8812au_iqk_rx_fill_iqc(struct rtl_priv *rtlpriv, enum radio_path Path,
	unsigned int RX_X, unsigned int RX_Y)
{
	switch (Path) {
	case RF90_PATH_A:
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
		if (RX_X>>1 == 0x112 || RX_Y>>1 == 0x3ee) {
			rtl_set_bbreg(rtlpriv, 0xc10, 0x000003ff, 0x100);
			rtl_set_bbreg(rtlpriv, 0xc10, 0x03ff0000, 0);
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X = %x;;RX_Y = %x ====>fill to IQC\n", RX_X>>1&0x000003ff, RX_Y>>1&0x000003ff);
		} else {
			rtl_set_bbreg(rtlpriv, 0xc10, 0x000003ff, RX_X>>1);
			rtl_set_bbreg(rtlpriv, 0xc10, 0x03ff0000, RX_Y>>1);
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X = %x;;RX_Y = %x ====>fill to IQC\n", RX_X>>1&0x000003ff, RX_Y>>1&0x000003ff);
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "0xc10 = %x ====>fill to IQC\n", rtl_read_dword(rtlpriv, 0xc10));
		}
		break;
	case RF90_PATH_B:
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /*  [31] = 0 --> Page C */
			if (RX_X>>1 == 0x112 || RX_Y>>1 == 0x3ee) {
				rtl_set_bbreg(rtlpriv, 0xe10, 0x000003ff, 0x100);
				rtl_set_bbreg(rtlpriv, 0xe10, 0x03ff0000, 0);
				RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X = %x;;RX_Y = %x ====>fill to IQC\n", RX_X>>1&0x000003ff, RX_Y>>1&0x000003ff);
			} else {
				rtl_set_bbreg(rtlpriv, 0xe10, 0x000003ff, RX_X>>1);
				rtl_set_bbreg(rtlpriv, 0xe10, 0x03ff0000, RX_Y>>1);
				RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X = %x;;RX_Y = %x====>fill to IQC\n ", RX_X>>1&0x000003ff, RX_Y>>1&0x000003ff);
				RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "0xe10 = %x====>fill to IQC\n", rtl_read_dword(rtlpriv, 0xe10));
			}
		break;
	default:
		break;
	};
}

static void _rtl8821au_iqk_rx_fill_iqc(struct rtl_priv *rtlpriv, enum radio_path Path,
	unsigned int RX_X, unsigned int RX_Y)
{
	switch (Path) {
	case RF90_PATH_A:
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
		rtl_set_bbreg(rtlpriv, 0xc10, 0x000003ff, RX_X>>1);
		rtl_set_bbreg(rtlpriv, 0xc10, 0x03ff0000, RX_Y>>1);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X = %x;;RX_Y = %x ====>fill to IQC\n", RX_X>>1, RX_Y>>1);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "0xc10 = %x ====>fill to IQC\n", rtl_read_dword(rtlpriv, 0xc10));
		break;
	default:
		break;
	};
}

static void _rtl8812au_iqk_tx_fill_iqc(struct rtl_priv *rtlpriv, enum radio_path  Path,
	unsigned int TX_X, unsigned int TX_Y)
{
	switch (Path) {
	case RF90_PATH_A:
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */
		rtl_write_dword(rtlpriv, 0xc90, 0x00000080);
		rtl_write_dword(rtlpriv, 0xcc4, 0x20040000);
		rtl_write_dword(rtlpriv, 0xcc8, 0x20000000);
		rtl_set_bbreg(rtlpriv, 0xccc, 0x000007ff, TX_Y);
		rtl_set_bbreg(rtlpriv, 0xcd4, 0x000007ff, TX_X);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TX_X = %x;;TX_Y = %x =====> fill to IQC\n", TX_X&0x000007ff, TX_Y&0x000007ff);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "0xcd4 = %x;;0xccc = %x ====>fill to IQC\n", rtl_get_bbreg(rtlpriv, 0xcd4, 0x000007ff), rtl_get_bbreg(rtlpriv, 0xccc, 0x000007ff));
		break;
	case RF90_PATH_B:
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */
		rtl_write_dword(rtlpriv, 0xe90, 0x00000080);
		rtl_write_dword(rtlpriv, 0xec4, 0x20040000);
		rtl_write_dword(rtlpriv, 0xec8, 0x20000000);
		rtl_set_bbreg(rtlpriv, 0xecc, 0x000007ff, TX_Y);
		rtl_set_bbreg(rtlpriv, 0xed4, 0x000007ff, TX_X);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TX_X = %x;;TX_Y = %x =====> fill to IQC\n", TX_X&0x000007ff, TX_Y&0x000007ff);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "0xed4 = %x;;0xecc = %x ====>fill to IQC\n", rtl_get_bbreg(rtlpriv, 0xed4, 0x000007ff), rtl_get_bbreg(rtlpriv, 0xecc, 0x000007ff));
		break;
	default:
		break;
	};
}

static void _rtl8821au_iqk_tx_fill_iqc(struct rtl_priv *rtlpriv, enum radio_path Path,
	unsigned int TX_X, unsigned int TX_Y)
{
	switch (Path) {
	case RF90_PATH_A:
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */
		rtl_write_dword(rtlpriv, 0xc90, 0x00000080);
		rtl_write_dword(rtlpriv, 0xcc4, 0x20040000);
		rtl_write_dword(rtlpriv, 0xcc8, 0x20000000);
		rtl_set_bbreg(rtlpriv, 0xccc, 0x000007ff, TX_Y);
		rtl_set_bbreg(rtlpriv, 0xcd4, 0x000007ff, TX_X);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TX_X = %x;;TX_Y = %x =====> fill to IQC\n", TX_X, TX_Y);
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "0xcd4 = %x;;0xccc = %x ====>fill to IQC\n", rtl_get_bbreg(rtlpriv, 0xcd4, 0x000007ff), rtl_get_bbreg(rtlpriv, 0xccc, 0x000007ff));
		break;
	default:
		break;
	};
}

#define cal_num 3

/* ULLI this function needs a complete rewrite (or we cantake code form rtlwifi-lib */

static void _rtl8812au_iqk_tx(struct rtl_priv *rtlpriv, enum radio_path Path)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

	uint32_t 	TX_fail, RX_fail, delay_count, IQK_ready, cal_retry, cal = 0, temp_reg65;
	int		TX_X = 0, TX_Y = 0, RX_X = 0, RX_Y = 0, TX_Average = 0, RX_Average = 0;
	int 		TX_X0[cal_num], TX_Y0[cal_num], RX_X0[cal_num], RX_Y0[cal_num];
	bool 	TX0IQKOK = false, RX0IQKOK = false;
	int 		TX_X1[cal_num], TX_Y1[cal_num], RX_X1[cal_num], RX_Y1[cal_num];
	bool  	TX1IQKOK = false, RX1IQKOK = false, VDF_enable = false;
	int 			i, k, VDF_Y[3], VDF_X[3], Tx_dt[3], Rx_dt[3], ii, dx = 0, dy = 0, TX_finish = 0, RX_finish = 0, dt = 0;

	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BandWidth = %d, ExtPA5G = %d, ExtPA2G = %d\n", rtlpriv->phy.current_chan_bw, rtlhal->external_pa_5g, rtlhal->external_pa_2g);
	if (rtlpriv->phy.current_chan_bw == 2) {
		VDF_enable = true;
	}
	VDF_enable = false;
	temp_reg65 = rtl_get_rfreg(rtlpriv, Path, 0x65, bMaskDWord);

	switch (Path) {
	case RF90_PATH_A:
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);	/* [31] = 0 --> Page C */
		/*  ========Path-A AFE all on======== */
		/* Port 0 DAC/ADC on */
		rtl_write_dword(rtlpriv, 0xc60, 0x77777777);
		rtl_write_dword(rtlpriv, 0xc64, 0x77777777);

		/* Port 1 DAC/ADC off */
		rtl_write_dword(rtlpriv, 0xe60, 0x00000000);
		rtl_write_dword(rtlpriv, 0xe64, 0x00000000);

		rtl_write_dword(rtlpriv, 0xc68, 0x19791979);

		rtl_set_bbreg(rtlpriv, 0xc00, 0xf, 0x4);		/* hardware 3-wire off */

		/* DAC/ADC sampling rate (160 MHz) */
		rtl_set_bbreg(rtlpriv, 0xc5c, BIT(26)|BIT(25)|BIT(24), 0x7);
		rtl_set_bbreg(rtlpriv, 0x8c4, BIT(30), 0x1);
		/* rtl_set_bbreg(rtlpriv, 0xcb0, 0x00ff0000, 0x77); */
		/* rtl_set_bbreg(rtlpriv, 0xcb4, 0x03000000, 0x0); */
		break;
	case RF90_PATH_B:
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
		/* ========Path-B AFE all on======== */
		/* Port 0 DAC/ADC off */
		rtl_write_dword(rtlpriv, 0xc60, 0x00000000);
		rtl_write_dword(rtlpriv, 0xc64, 0x00000000);

		/* Port 1 DAC/ADC on */
		rtl_write_dword(rtlpriv, 0xe60, 0x77777777);
		rtl_write_dword(rtlpriv, 0xe64, 0x77777777);

		rtl_write_dword(rtlpriv, 0xe68, 0x19791979);

		rtl_set_bbreg(rtlpriv, 0xe00, 0xf, 0x4);		/* hardware 3-wire off */

		/* DAC/ADC sampling rate (160 MHz) */
		rtl_set_bbreg(rtlpriv, 0xe5c, BIT(26)|BIT(25)|BIT(24), 0x7);
		rtl_set_bbreg(rtlpriv, 0x8c4, BIT(30), 0x1);
		/* rtl_set_bbreg(rtlpriv, 0xeb0, 0x00ff0000, 0x77); */
		/* rtl_set_bbreg(rtlpriv, 0xeb4, 0x03000000, 0x0); */
		break;
	default:
		break;
	}

	switch (Path) {
	case RF90_PATH_A:
	    {
		/* ====== TX IQK ====== */
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /*  [31] = 0 --> Page C */
		rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80002);
		rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x20000);
		rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x3fffd);
		rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfe83f);
		rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d5);
		rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x8a001);
		rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
		rtl_write_dword(rtlpriv, 0xb00, 0x03000100);
		rtl_set_bbreg(rtlpriv, 0xc94, BIT(0), 0x1);
		rtl_write_dword(rtlpriv, 0x978, 0x29002000);	/* TX (X,Y) */
		rtl_write_dword(rtlpriv, 0x97c, 0xa9002000);	/* RX (X,Y) */
		rtl_write_dword(rtlpriv, 0x984, 0x00462910);	/* [0]:AGC_en, [15]:idac_K_Mask */

		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);	/* [31] = 1 --> Page C1 */

		if (rtlhal->external_pa_5g)
			rtl_write_dword(rtlpriv, 0xc88, 0x821403f7);
		else
			rtl_write_dword(rtlpriv, 0xc88, 0x821403f1);

		if (rtlhal->current_bandtype)
			rtl_write_dword(rtlpriv, 0xc8c, 0x68163e96);
		else {
			rtl_write_dword(rtlpriv, 0xc8c, 0x28163e96);
			if (rtlhal->rfe_type == 3) {
				if (rtlhal->external_pa_2g)
					rtl_write_dword(rtlpriv, 0xc88, 0x821403e3);
				else
					rtl_write_dword(rtlpriv, 0xc88, 0x821403f7);
			}

		}

		if (VDF_enable == 1) {
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TXVDF Start\n");
			for (k = 0; k <= 2; k++) {
				switch (k) {
				case 0:
					rtl_write_dword(rtlpriv, 0xc80, 0x18008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xc84, 0x38008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_write_dword(rtlpriv, 0x984, 0x00462910);	/* [0]:AGC_en, [15]:idac_K_Mask */
					rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x0);
					break;
				case 1:
					rtl_set_bbreg(rtlpriv, 0xc80, BIT(28), 0x0);
					rtl_set_bbreg(rtlpriv, 0xc84, BIT(28), 0x0);
					rtl_write_dword(rtlpriv, 0x984, 0x0046a910);	/* [0]:AGC_en, [15]:idac_K_Mask */
					break;
				case 2:
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_Y[1] = %x;;;VDF_Y[0] = %x\n", VDF_Y[1]>>21 & 0x00007ff, VDF_Y[0]>>21 & 0x00007ff);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_X[1] = %x;;;VDF_X[0] = %x\n", VDF_X[1]>>21 & 0x00007ff, VDF_X[0]>>21 & 0x00007ff);
					Tx_dt[cal] = (VDF_Y[1]>>20)-(VDF_Y[0]>>20);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Tx_dt = %d\n", Tx_dt[cal]);
					Tx_dt[cal] = ((16*Tx_dt[cal])*10000/15708);
					Tx_dt[cal] = (Tx_dt[cal] >> 1) + (Tx_dt[cal] & BIT(0));
					rtl_write_dword(rtlpriv, 0xc80, 0x18008c20);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xc84, 0x38008c20);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x1);
					rtl_set_bbreg(rtlpriv, 0xce8, 0x3fff0000, Tx_dt[cal] & 0x00003fff);
					break;
				default:
					break;
				}
				rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);		/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
				cal_retry = 0;
				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); /* Delay 10ms */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {							/* If 20ms No Result, then cal_retry++ */
						/* ============TXIQK Check============== */
						TX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(12));

						if (~TX_fail) {
							rtl_write_dword(rtlpriv, 0xcb8, 0x02000000);
							VDF_X[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xcb8, 0x04000000);
							VDF_Y[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							TX0IQKOK = true;
							break;
						} else {
							TX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10) {
								break;
							}
						}
					} else {
						TX0IQKOK = false;
						cal_retry++;
						if (cal_retry == 10) {
							break;
						}
					}
				}
			}
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TXA_VDF_cal_retry = %d\n", cal_retry);
			TX_X0[cal] = VDF_X[k-1] ;
			TX_Y0[cal] = VDF_Y[k-1];
		} else {
			rtl_write_dword(rtlpriv, 0xc80, 0x18008c10);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
			rtl_write_dword(rtlpriv, 0xc84, 0x38008c10);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
			rtl_write_dword(rtlpriv, 0xce8, 0x00000000);

			for (cal = 0; cal < cal_num; cal++) {
				cal_retry = 0;
				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);	/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); 				/* Delay 25ms */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {				/* If 20ms No Result, then cal_retry++ */
						/* ============TXIQK Check============== */
						TX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(12));

						if (~TX_fail) {
							rtl_write_dword(rtlpriv, 0xcb8, 0x02000000);
							TX_X0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xcb8, 0x04000000);
							TX_Y0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							TX0IQKOK = true;
							/*
							rtl_write_dword(rtlpriv, 0xcb8, 0x01000000);
							reg1 = rtl_get_bbreg(rtlpriv, 0xd00, 0xffffffff);
							rtl_write_dword(rtlpriv, 0xcb8, 0x02000000);
							reg2 = rtl_get_bbreg(rtlpriv, 0xd00, 0x0000001f);
							Image_Power = (reg2<<32)+reg1;
							DbgPrint("Before PW = %d\n", Image_Power);
							rtl_write_dword(rtlpriv, 0xcb8, 0x03000000);
							reg1 = rtl_get_bbreg(rtlpriv, 0xd00, 0xffffffff);
							rtl_write_dword(rtlpriv, 0xcb8, 0x04000000);
							reg2 = rtl_get_bbreg(rtlpriv, 0xd00, 0x0000001f);
							Image_Power = (reg2<<32)+reg1;
							DbgPrint("After PW = %d\n", Image_Power);
							*/
							break;
						} else {
							TX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10) {
								break;
							}
						}
					} else {
						TX0IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
							break;
					}
				}
				RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TXA_cal_retry = %d\n", cal_retry);
				if (TX0IQKOK)
					TX_Average++;
			}
		}

		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
		rtl_set_rfreg(rtlpriv, Path, 0x58, 0x7fe00, rtl_get_rfreg(rtlpriv, Path, 0x8, 0xffc00)); /* Load LOK */
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */

		if (TX0IQKOK == false)
			break;				/* TXK fail, Don't do RXK */

		if (VDF_enable == 1) {
			rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x0);    /*  TX VDF Disable */
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXVDF Start\n");

			/* ====== RX IQK ====== */
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
			rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
			rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x3f7ff);
			rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfe7bf);
			rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x88001);
			rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d0);
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);
			rtl_set_bbreg(rtlpriv, 0x978, BIT(31), 0x1);
			rtl_set_bbreg(rtlpriv, 0x97c, BIT(31), 0x0);
			rtl_write_dword(rtlpriv, 0x984, 0x0046a911);

			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */
			rtl_write_dword(rtlpriv, 0xc88, 0x02140119);
			rtl_write_dword(rtlpriv, 0xc8c, 0x28161420);

			for (k = 0; k <= 2; k++) {
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); 	/* [31] = 0 --> Page C */
				rtl_set_bbreg(rtlpriv, 0x978, 0x03FF8000, (VDF_X[k])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x978, 0x000007FF, (VDF_Y[k])>>21&0x000007ff);

				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); 	/* [31] = 1 --> Page C1 */
				switch (k) {
				case 0:
					rtl_write_dword(rtlpriv, 0xc80, 0x38008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xc84, 0x18008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_set_bbreg(rtlpriv, 0xce8, BIT(30), 0x0);
					break;
				case 1:
					rtl_write_dword(rtlpriv, 0xc80, 0x28008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xc84, 0x08008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					break;
				case 2:
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_Y[1] = %x;;;VDF_Y[0] = %x\n", VDF_Y[1]>>21 & 0x00007ff, VDF_Y[0]>>21 & 0x00007ff);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_X[1] = %x;;;VDF_X[0] = %x\n", VDF_X[1]>>21 & 0x00007ff, VDF_X[0]>>21 & 0x00007ff);
					Rx_dt[cal] = (VDF_Y[1]>>20)-(VDF_Y[0]>>20);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Rx_dt = %d\n", Rx_dt[cal]);
					Rx_dt[cal] = ((16*Rx_dt[cal])*10000/13823);
					Rx_dt[cal] = (Rx_dt[cal] >> 1) + (Rx_dt[cal] & BIT(0));
					rtl_write_dword(rtlpriv, 0xc80, 0x38008c20);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xc84, 0x18008c20);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_set_bbreg(rtlpriv, 0xce8, 0x00003fff, Rx_dt[cal] & 0x00003fff);
					break;
				default:
					break;
				}


				if (k == 2) {
					rtl_set_bbreg(rtlpriv, 0xce8, BIT(30), 0x1);  /* RX VDF Enable */
				}
				rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */

				cal_retry = 0;
				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); /* Delay 10ms */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
					delay_count = 0;

					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {	/* If 20ms No Result, then cal_retry++ */
						/* ============RXIQK Check============== */
						RX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(11));
						if (RX_fail == 0) {
							rtl_write_dword(rtlpriv, 0xcb8, 0x06000000);
							VDF_X[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xcb8, 0x08000000);
							VDF_Y[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							RX0IQKOK = true;
							break;
						} else {
							rtl_set_bbreg(rtlpriv, 0xc10, 0x000003ff, 0x200>>1);
							rtl_set_bbreg(rtlpriv, 0xc10, 0x03ff0000, 0x0>>1);
							RX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10)
								break;
						}
					} else {
						RX0IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
							break;
					}
				}
			}
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXA_VDF_cal_retry = %d\n", cal_retry);
			RX_X0[cal] = VDF_X[k-1] ;
			RX_Y0[cal] = VDF_Y[k-1];
			rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x1);    /* TX VDF Enable */
		} else {
			/* ====== RX IQK ====== */
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			/*  1. RX RF Setting */
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
			rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
			rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x3f7ff);
			rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfe7bf);
			rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x88001);
			rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d0);
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);

			rtl_set_bbreg(rtlpriv, 0x978, BIT(31), 0x1);
			rtl_set_bbreg(rtlpriv, 0x97c, BIT(31), 0x0);
			rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
			/* rtl_write_dword(rtlpriv, 0x984, 0x0046a911); */
			rtl_write_dword(rtlpriv, 0x984, 0x0046a891);

			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); 	/* [31] = 1 --> Page C1 */
			rtl_write_dword(rtlpriv, 0xc80, 0x38008c10);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
			rtl_write_dword(rtlpriv, 0xc84, 0x18008c10);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
			rtl_write_dword(rtlpriv, 0xc88, 0x02140119);
			rtl_write_dword(rtlpriv, 0xc8c, 0x28160d40);

			for (cal = 0; cal < cal_num; cal++) {
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);		/* [31] = 0 --> Page C */
				rtl_set_bbreg(rtlpriv, 0x978, 0x03FF8000, (TX_X0[cal])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x978, 0x000007FF, (TX_Y0[cal])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);		/* [31] = 1 --> Page C1 */
				cal_retry = 0;
				while (1) {
					/*  one shot */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10);			/* Delay 10ms */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

				if (delay_count < 20) {	/* If 20ms No Result, then cal_retry++ */
					/* ============RXIQK Check============== */
					RX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(11));
					if (RX_fail == 0) {
						rtl_write_dword(rtlpriv, 0xcb8, 0x06000000);
						RX_X0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
						rtl_write_dword(rtlpriv, 0xcb8, 0x08000000);
						RX_Y0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
						RX0IQKOK = true;
						/*
						rtl_write_dword(rtlpriv, 0xcb8, 0x05000000);
						reg1 = rtl_get_bbreg(rtlpriv, 0xd00, 0xffffffff);
						rtl_write_dword(rtlpriv, 0xcb8, 0x06000000);
						reg2 = rtl_get_bbreg(rtlpriv, 0xd00, 0x0000001f);
						DbgPrint("reg1 = %d, reg2 = %d", reg1, reg2);
						Image_Power = (reg2<<32)+reg1;
						DbgPrint("Before PW = %d\n", Image_Power);
						rtl_write_dword(rtlpriv, 0xcb8, 0x07000000);
						reg1 = rtl_get_bbreg(rtlpriv, 0xd00, 0xffffffff);
						rtl_write_dword(rtlpriv, 0xcb8, 0x08000000);
						reg2 = rtl_get_bbreg(rtlpriv, 0xd00, 0x0000001f);
						Image_Power = (reg2<<32)+reg1;
						DbgPrint("After PW = %d\n", Image_Power);
						*/

						break;
					} else {
						rtl_set_bbreg(rtlpriv, 0xc10, 0x000003ff, 0x200>>1);
						rtl_set_bbreg(rtlpriv, 0xc10, 0x03ff0000, 0x0>>1);
						RX0IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
						break;
					}
				} else {
					RX0IQKOK = false;
					cal_retry++;
					if (cal_retry == 10)
						break;
					}
				}
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXA_cal_retry = %d\n", cal_retry);
			if (RX0IQKOK)
				RX_Average++;
			}
		}
	    }
		break; /* MARK */
	case RF90_PATH_B:
	    {
		/* Path-B TX/RX IQK */
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);		/* [31] = 0 --> Page C */
		rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80002);
		rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x20000);
		rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x3fffd);
		rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfe83f);
		rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d5);
		rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x8a001);
		rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
		rtl_write_dword(rtlpriv, 0xb00, 0x03000100);
		rtl_set_bbreg(rtlpriv, 0xe94, BIT(0), 0x1);
		rtl_write_dword(rtlpriv, 0x978, 0x29002000);		/* TX (X,Y) */
		rtl_write_dword(rtlpriv, 0x97c, 0xa9002000);		/* RX (X,Y) */
		rtl_write_dword(rtlpriv, 0x984, 0x00462910);		/* [0]:AGC_en, [15]:idac_K_Mask */

		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);		/* [31] = 1 --> Page C1 */

		if (rtlhal->external_pa_5g)
			rtl_write_dword(rtlpriv, 0xe88, 0x821403f7);
		else
			rtl_write_dword(rtlpriv, 0xe88, 0x821403f1);

		if (rtlhal->current_bandtype)
			rtl_write_dword(rtlpriv, 0xe8c, 0x68163e96);
		else
			rtl_write_dword(rtlpriv, 0xe8c, 0x28163e96);

		if (VDF_enable == 1) {
			for (k = 0; k <= 2; k++) {
				switch (k) {
				case 0:
					rtl_write_dword(rtlpriv, 0xe80, 0x18008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xe84, 0x38008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_write_dword(rtlpriv, 0x984, 0x00462910);
					rtl_set_bbreg(rtlpriv, 0xee8, BIT(31), 0x0);
					break;
				case 1:
					rtl_set_bbreg(rtlpriv, 0xe80, BIT(28), 0x0);
					rtl_set_bbreg(rtlpriv, 0xe84, BIT(28), 0x0);
					rtl_write_dword(rtlpriv, 0x984, 0x0046a910);
					rtl_set_bbreg(rtlpriv, 0xee8, BIT(31), 0x0);
					break;
				case 2:
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_Y[1] = %x;;;VDF_Y[0] = %x\n", VDF_Y[1]>>21 & 0x00007ff, VDF_Y[0]>>21 & 0x00007ff);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_X[1] = %x;;;VDF_X[0] = %x\n", VDF_X[1]>>21 & 0x00007ff, VDF_X[0]>>21 & 0x00007ff);
					Tx_dt[cal] = (VDF_Y[1]>>20)-(VDF_Y[0]>>20);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Tx_dt = %d\n", Tx_dt[cal]);
					Tx_dt[cal] = ((16*Tx_dt[cal])*10000/15708);
					Tx_dt[cal] = (Tx_dt[cal] >> 1) + (Tx_dt[cal] & BIT(0));
					rtl_write_dword(rtlpriv, 0xe80, 0x18008c20);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xe84, 0x38008c20);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_set_bbreg(rtlpriv, 0xee8, BIT(31), 0x1);
					rtl_set_bbreg(rtlpriv, 0xee8, 0x3fff0000, Tx_dt[cal] & 0x00003fff);
					break;
				default:
					break;
				}


				rtl_write_dword(rtlpriv, 0xeb8, 0x00100000);/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
				cal_retry = 0;
				while (1) {
					/*  one shot */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); /* Delay 10ms */
					rtl_write_dword(rtlpriv, 0xeb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd40, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {		/* If 20ms No Result, then cal_retry++ */
						/* ============TXIQK Check============== */
						TX_fail = rtl_get_bbreg(rtlpriv, 0xd40, BIT(12));

						if (~TX_fail) {
							rtl_write_dword(rtlpriv, 0xeb8, 0x02000000);
							VDF_X[k] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xeb8, 0x04000000);
							VDF_Y[k] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							TX1IQKOK = true;
							break;
						} else {
							TX1IQKOK = false;
							cal_retry++;
							if (cal_retry == 10) {
								break;
							}
						}
					} else {
						TX1IQKOK = false;
						cal_retry++;
						if (cal_retry == 10) {
							break;
						}
					}
				}
			}
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TXB_VDF_cal_retry = %d\n", cal_retry);
			TX_X1[cal] = VDF_X[k-1] ;
			TX_Y1[cal] = VDF_Y[k-1];
		} else {
			rtl_write_dword(rtlpriv, 0xe80, 0x18008c10);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
			rtl_write_dword(rtlpriv, 0xe84, 0x38008c10);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
			rtl_write_dword(rtlpriv, 0xee8, 0x00000000);

			for (cal = 0; cal < cal_num; cal++) {
				cal_retry = 0;

				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0xeb8, 0x00100000);/*  cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); /* Delay 25ms */
					rtl_write_dword(rtlpriv, 0xeb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd40, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {							/* If 20ms No Result, then cal_retry++ */
						/* ============TXIQK Check============== */
						TX_fail = rtl_get_bbreg(rtlpriv, 0xd40, BIT(12));
						if (~TX_fail) {
							rtl_write_dword(rtlpriv, 0xeb8, 0x02000000);
							TX_X1[cal] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xeb8, 0x04000000);
							TX_Y1[cal] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							TX1IQKOK = true;
							/*
							int			reg1 = 0, reg2 = 0, Image_Power = 0;
							rtl_write_dword(rtlpriv, 0xeb8, 0x01000000);
							reg1 = rtl_get_bbreg(rtlpriv, 0xd40, 0xffffffff);
							rtl_write_dword(rtlpriv, 0xeb8, 0x02000000);
							reg2 = rtl_get_bbreg(rtlpriv, 0xd40, 0x0000001f);
							Image_Power = (reg2<<32)+reg1;
							DbgPrint("Before PW = %d\n", Image_Power);
							rtl_write_dword(rtlpriv, 0xeb8, 0x03000000);
							reg1 = rtl_get_bbreg(rtlpriv, 0xd40, 0xffffffff);
							rtl_write_dword(rtlpriv, 0xeb8, 0x04000000);
							reg2 = rtl_get_bbreg(rtlpriv, 0xd40, 0x0000001f);
							Image_Power = (reg2<<32)+reg1;
							DbgPrint("After PW = %d\n", Image_Power);
							*/
							break;
						} else {
							TX1IQKOK = false;
							cal_retry++;
							if (cal_retry == 10) {
								break;
							}
						}
					}  else {
						TX1IQKOK = false;
						cal_retry++;
						if (cal_retry == 10) {
							break;
						}
					}
				}
				RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TXB_cal_retry = %d\n", cal_retry);
				if (TX1IQKOK)
					TX_Average++;
			}
		}

		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);		/* [31] = 0 --> Page C */
		rtl_set_rfreg(rtlpriv, Path, 0x58, 0x7fe00, rtl_get_rfreg(rtlpriv, Path, 0x8, 0xffc00));	/* Load LOK */
		rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);		/* [31] = 1 --> Page C1 */

		if (TX1IQKOK == false)
			break;				/* TXK fail, Don't do RXK */

		if (VDF_enable == 1) {
			rtl_set_bbreg(rtlpriv, 0xee8, BIT(31), 0x0);    /* TX VDF Disable */
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXVDF Start\n");

			/* ====== RX IQK ====== */
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);		/* [31] = 0 --> Page C */
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
			rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
			rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x3f7ff);
			rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfe7bf);
			rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x88001);
			rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d0);
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);

			rtl_set_bbreg(rtlpriv, 0x978, BIT(31), 0x1);
			rtl_set_bbreg(rtlpriv, 0x97c, BIT(31), 0x0);
			rtl_write_dword(rtlpriv, 0x984, 0x0046a911);
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);		/* [31] = 1 --> Page C1 */
			rtl_write_dword(rtlpriv, 0xe88, 0x02140119);
			rtl_write_dword(rtlpriv, 0xe8c, 0x28161420);

			for (k = 0; k <= 2; k++) {
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);		/* [31] = 0 --> Page C */
				rtl_set_bbreg(rtlpriv, 0x978, 0x03FF8000, (VDF_X[k])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x978, 0x000007FF, (VDF_Y[k])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);		/* [31] = 1 --> Page C1 */

				switch (k) {
				case 0:
					rtl_write_dword(rtlpriv, 0xe80, 0x38008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xe84, 0x18008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_set_bbreg(rtlpriv, 0xee8, BIT(30), 0x0);
					break;
				case 1:
					rtl_write_dword(rtlpriv, 0xe80, 0x28008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xe84, 0x08008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_set_bbreg(rtlpriv, 0xee8, BIT(30), 0x0);
					break;
				case 2:
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_Y[1] = %x;;;VDF_Y[0] = %x\n", VDF_Y[1]>>21 & 0x00007ff, VDF_Y[0]>>21 & 0x00007ff);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_X[1] = %x;;;VDF_X[0] = %x\n", VDF_X[1]>>21 & 0x00007ff, VDF_X[0]>>21 & 0x00007ff);
					Rx_dt[cal] = (VDF_Y[1]>>20)-(VDF_Y[0]>>20);
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Rx_dt = %d\n", Rx_dt[cal]);
					Rx_dt[cal] = ((16*Rx_dt[cal])*10000/13823);
					Rx_dt[cal] = (Rx_dt[cal] >> 1) + (Rx_dt[cal] & BIT(0));
					rtl_write_dword(rtlpriv, 0xe80, 0x38008c20);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
					rtl_write_dword(rtlpriv, 0xe84, 0x18008c20);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
					rtl_set_bbreg(rtlpriv, 0xee8, 0x00003fff, Rx_dt[cal] & 0x00003fff);
					break;
				default:
					break;
				}


				if (k == 2) {
					rtl_set_bbreg(rtlpriv, 0xee8, BIT(30), 0x1);	/* RX VDF Enable */
				}

				rtl_write_dword(rtlpriv, 0xeb8, 0x00100000);		/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */

				cal_retry = 0;

				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); /* Delay 10ms */
					rtl_write_dword(rtlpriv, 0xeb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd40, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {	/* If 20ms No Result, then cal_retry++ */
						/* ============RXIQK Check============== */
						RX_fail = rtl_get_bbreg(rtlpriv, 0xd40, BIT(11));
						if (RX_fail == 0) {
							rtl_write_dword(rtlpriv, 0xeb8, 0x06000000);
							VDF_X[k] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xeb8, 0x08000000);
							VDF_Y[k] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							RX1IQKOK = true;
							break;
						} else {
							rtl_set_bbreg(rtlpriv, 0xe10, 0x000003ff, 0x200>>1);
							rtl_set_bbreg(rtlpriv, 0xe10, 0x03ff0000, 0x0>>1);
							RX1IQKOK = false;
							cal_retry++;
							if (cal_retry == 10)
								break;

						}
					} else {
						RX1IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
							break;
					}
				}
			}


			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXB_VDF_cal_retry = %d\n", cal_retry);
			RX_X1[cal] = VDF_X[k-1] ;
			RX_Y1[cal] = VDF_Y[k-1];
			rtl_set_bbreg(rtlpriv, 0xee8, BIT(31), 0x1);	/* TX VDF Enable */
		} else {
			/* ====== RX IQK ====== */
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
			rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
			rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x3f7ff);
			rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfe7bf);
			rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x88001);
			rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d0);
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);

			rtl_set_bbreg(rtlpriv, 0x978, BIT(31), 0x1);
			rtl_set_bbreg(rtlpriv, 0x97c, BIT(31), 0x0);
			rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
			/* rtl_write_dword(rtlpriv, 0x984, 0x0046a911); */
			rtl_write_dword(rtlpriv, 0x984, 0x0046a891);

			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);	/* [31] = 1 --> Page C1 */
			rtl_write_dword(rtlpriv, 0xe80, 0x38008c10);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
			rtl_write_dword(rtlpriv, 0xe84, 0x18008c10);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
			rtl_write_dword(rtlpriv, 0xe88, 0x02140119);
			rtl_write_dword(rtlpriv, 0xe8c, 0x28161180);

			for (cal = 0; cal < cal_num; cal++) {
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);		/* [31] = 0 --> Page C */
				rtl_set_bbreg(rtlpriv, 0x978, 0x03FF8000, (TX_X1[cal])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x978, 0x000007FF, (TX_Y1[cal])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);		/* [31] = 1 --> Page C1 */
				cal_retry = 0;

				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0xeb8, 0x00100000);	/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10);	/*Delay 10ms */
					rtl_write_dword(rtlpriv, 0xeb8, 0x00000000);
					delay_count = 0;

					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd40, BIT(10));
						if ((IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {				/* If 20ms No Result, then cal_retry++ */
						/* ============RXIQK Check============== */
						RX_fail = rtl_get_bbreg(rtlpriv, 0xd40, BIT(11));
						if (RX_fail == 0) {
							rtl_write_dword(rtlpriv, 0xeb8, 0x06000000);
							RX_X1[cal] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xeb8, 0x08000000);
							RX_Y1[cal] = rtl_get_bbreg(rtlpriv, 0xd40, 0x07ff0000)<<21;
							RX1IQKOK = true;
							/*
							rtl_write_dword(rtlpriv, 0xeb8, 0x05000000);
							reg1 = rtl_get_bbreg(rtlpriv, 0xd40, 0xffffffff);
							rtl_write_dword(rtlpriv, 0xeb8, 0x06000000);
							reg2 = rtl_get_bbreg(rtlpriv, 0xd40, 0x0000001f);
							DbgPrint("reg1 = %d, reg2 = %d", reg1, reg2);
							Image_Power = (reg2<<32)+reg1;
							DbgPrint("Before PW = %d\n", Image_Power);
							rtl_write_dword(rtlpriv, 0xeb8, 0x07000000);
							reg1 = rtl_get_bbreg(rtlpriv, 0xd40, 0xffffffff);
							rtl_write_dword(rtlpriv, 0xeb8, 0x08000000);
							reg2 = rtl_get_bbreg(rtlpriv, 0xd40, 0x0000001f);
							Image_Power = (reg2<<32)+reg1;
							DbgPrint("After PW = %d\n", Image_Power);
							*/
							break;
						} else {
							rtl_set_bbreg(rtlpriv, 0xe10, 0x000003ff, 0x200>>1);
							rtl_set_bbreg(rtlpriv, 0xe10, 0x03ff0000, 0x0>>1);
							RX1IQKOK = false;
							cal_retry++;
							if (cal_retry == 10)
								break;
						}
					} else {
						RX1IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
							break;
					}

				}

			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXB_cal_retry = %d\n", cal_retry);
			if (RX1IQKOK)
				RX_Average++;
			}
		}
	    }
		break;
	default:
		break;
	}

	/* FillIQK Result */
	switch (Path) {
	case RF90_PATH_A:
	    {
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "========Path_A =======\n");
		if (TX_Average == 0) {
			_rtl8812au_iqk_tx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
			break;
		}
		for (i = 0; i < TX_Average; i++) {
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TX_X0[%d] = %x ;; TX_Y0[%d] = %x\n", i, (TX_X0[i])>>21&0x000007ff, i, (TX_Y0[i])>>21&0x000007ff);
		}

		for (i = 0; i < TX_Average; i++) {
			for (ii = i+1; ii < TX_Average; ii++) {
				dx = (TX_X0[i]>>21) - (TX_X0[ii]>>21);
				if (dx < 4 && dx > -4) {
					dy = (TX_Y0[i]>>21) - (TX_Y0[ii]>>21);
					if (dy < 4 && dy > -4) {
						TX_X = ((TX_X0[i]>>21) + (TX_X0[ii]>>21))/2;
						TX_Y = ((TX_Y0[i]>>21) + (TX_Y0[ii]>>21))/2;
						if (rtlpriv->phy.current_chan_bw == 2) {
							Tx_dt[0] = (Tx_dt[i] + Tx_dt[ii])/2;
						}
						TX_finish = 1;
						break;
					}
				}
			}
			if (TX_finish == 1)
				break;
		}

		if (rtlpriv->phy.current_chan_bw == 2) {
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);		/* [31] = 0 --> Page C */
			rtl_set_bbreg(rtlpriv, 0xce8, 0x3fff0000, Tx_dt[0] & 0x00003fff);
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);		/* [31] = 0 --> Page C */
		}

		if (TX_finish == 1) {
			_rtl8812au_iqk_tx_fill_iqc(rtlpriv, Path, TX_X, TX_Y);
		} else {
			_rtl8812au_iqk_tx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
		}

		if (RX_Average == 0) {
			_rtl8812au_iqk_rx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
			break;
		}

		for (i = 0; i < RX_Average; i++) {
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X0[%d] = %x ;; RX_Y0[%d] = %x\n", i, (RX_X0[i])>>21&0x000007ff, i, (RX_Y0[i])>>21&0x000007ff);
		}

		for (i = 0; i < RX_Average; i++) {
			for (ii = i+1; ii < RX_Average; ii++) {
				dx = (RX_X0[i]>>21) - (RX_X0[ii]>>21);
				if (dx < 4 && dx > -4) {
					dy = (RX_Y0[i]>>21) - (RX_Y0[ii]>>21);
					if (dy < 4 && dy > -4) {
						RX_X = ((RX_X0[i]>>21) + (RX_X0[ii]>>21))/2;
						RX_Y = ((RX_Y0[i]>>21) + (RX_Y0[ii]>>21))/2;
						if (rtlpriv->phy.current_chan_bw == 2) {
							Rx_dt[0] = (Rx_dt[i] + Rx_dt[ii])/2;
						}
						RX_finish = 1;
						break;
					}
				}
			}
			if (RX_finish == 1)
				break;
		}

		if (rtlpriv->phy.current_chan_bw == 2) {
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 0 --> Page C */
			rtl_set_bbreg(rtlpriv, 0xce8, 0x00003fff, Rx_dt[0] & 0x00003fff);
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
		}

		if (RX_finish == 1) {
			_rtl8812au_iqk_rx_fill_iqc(rtlpriv, Path, RX_X, RX_Y);
		} else {
			_rtl8812au_iqk_rx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
		}

		/*
		 * ULLI check with rtl8821ae source, if we can remove this
		 * look as the commit 5856f7384c22bf6364ecb77635e95b4858567080
		 * from #if 1
		 */
#if 1
		if (TX_finish && RX_finish) {
			rtlpriv->phy.need_iqk = false;

			if (rtlpriv->phy.current_chan_bw == 2) {
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 0 --> Page C */
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			}
		}
#endif

	    }
		break;
	case RF90_PATH_B:
	    {
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "========Path_B =======\n");
		if (TX_Average == 0) {
			_rtl8812au_iqk_tx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
			break;
		}

		for (i = 0; i < TX_Average; i++) {
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TX_X1[%d] = %x ;; TX_Y1[%d] = %x\n", i, (TX_X1[i])>>21&0x000007ff, i, (TX_Y1[i])>>21&0x000007ff);
		}

		for (i = 0; i < TX_Average; i++) {
			for (ii = i+1; ii < TX_Average; ii++) {
				dx = (TX_X1[i]>>21) - (TX_X1[ii]>>21);
				if (dx < 4 && dx > -4) {
					dy = (TX_Y1[i]>>21) - (TX_Y1[ii]>>21);
					if (dy < 4 && dy > -4) {
						TX_X = ((TX_X1[i]>>21) + (TX_X1[ii]>>21))/2;
						TX_Y = ((TX_Y1[i]>>21) + (TX_Y1[ii]>>21))/2;
						if (rtlpriv->phy.current_chan_bw == 2) {
							Tx_dt[0] = (Tx_dt[i] + Tx_dt[ii])/2;
						}
						TX_finish = 1;
						break;
					}
				}
			}
			if (TX_finish == 1)
				break;
		}

		if (rtlpriv->phy.current_chan_bw == 2) {
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 0 --> Page C */
			rtl_set_bbreg(rtlpriv, 0xee8, 0x3fff0000, Tx_dt[0] & 0x00003fff);
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
		}
		if (TX_finish == 1) {
			_rtl8812au_iqk_tx_fill_iqc(rtlpriv, Path, TX_X, TX_Y);
		} else {
			_rtl8812au_iqk_tx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
		}

		if (RX_Average == 0) {
			_rtl8812au_iqk_rx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
			break;
		}

		for (i = 0; i < RX_Average; i++) {
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X1[%d] = %x ;; RX_Y1[%d] = %x\n", i, (RX_X1[i])>>21&0x000007ff, i, (RX_Y1[i])>>21&0x000007ff);
		}

		for (i = 0; i < RX_Average; i++) {
			for (ii = i+1; ii < RX_Average; ii++) {
				dx = (RX_X1[i]>>21) - (RX_X1[ii]>>21);
				if (dx < 4 && dx > -4) {
					dy = (RX_Y1[i]>>21) - (RX_Y1[ii]>>21);
					if (dy < 4 && dy > -4) {
						RX_X = ((RX_X1[i]>>21) + (RX_X1[ii]>>21))/2;
						RX_Y = ((RX_Y1[i]>>21) + (RX_Y1[ii]>>21))/2;
						if (rtlpriv->phy.current_chan_bw == 2) {
							Rx_dt[0] = (Rx_dt[i] + Rx_dt[ii])/2;
						}
						RX_finish = 1;
						break;
					}
				}
			}
			if (RX_finish == 1)
				break;
		}

		if (rtlpriv->phy.current_chan_bw == 2) {
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 0 --> Page C */
			rtl_set_bbreg(rtlpriv, 0xee8, 0x00003fff, Rx_dt[0] & 0x00003fff);
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
		}

		if (RX_finish == 1) {
			_rtl8812au_iqk_rx_fill_iqc(rtlpriv, Path, RX_X, RX_Y);
		} else{
			_rtl8812au_iqk_rx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
		}


		/*
		 * ULLI check with rtl8821ae source, if we can remove this
		 * look as the commit 5856f7384c22bf6364ecb77635e95b4858567080
		 * from #if 1
		 */
#if 1
		if (TX_finish && RX_finish) {
/* pRFCalibrateInfo->IQKMatrixRegSetting[chnlIdx].bIQKDone= true; */
			rtlpriv->phy.need_iqk = false;

			if (rtlpriv->phy.current_chan_bw == 2) {
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 0 --> Page C */
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			}
		}
#endif

	    }
		break;
	default:
		break;
	}
}



static void _rtl8812au_iqk_backup_macbb(struct rtl_priv *rtlpriv,
					u32 *macbb_backup,
					u32 *backup_macbb_reg, u32 mac_bb_num)
{
	u32 i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* save MACBB default value */
	for (i = 0; i < mac_bb_num; i++) {
		macbb_backup[i] = rtl_read_dword(rtlpriv, backup_macbb_reg[i]);
	}

	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BackupMacBB Success!!!!\n");
}

static void _rtl8821au_iqk_backup_macbb(struct rtl_priv *rtlpriv,
					u32 *macbb_backup,
					u32 *backup_macbb_reg, u32 mac_bb_num)
{
	u32 i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* save MACBB default value */
	for (i = 0; i < mac_bb_num; i++) {
		macbb_backup[i] = rtl_read_dword(rtlpriv, backup_macbb_reg[i]);
	}

	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BackupMacBB Success!!!!\n");
}

static void _rtl8812au_iqk_backup_rf(struct rtl_priv *rtlpriv,
	uint32_t *RFA_backup, uint32_t *RFB_backup,
	uint32_t *Backup_RF_REG, uint32_t RF_NUM)
{
	uint32_t i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* Save RF Parameters */
	for (i = 0; i < RF_NUM; i++) {
		RFA_backup[i] = rtl_get_rfreg(rtlpriv, RF90_PATH_A, Backup_RF_REG[i], bMaskDWord);
		RFB_backup[i] = rtl_get_rfreg(rtlpriv, RF90_PATH_B, Backup_RF_REG[i], bMaskDWord);
	}
	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BackupRF Success!!!!\n");
}

static void _rtl8821au_iqk_backup_rf(struct rtl_priv *rtlpriv, u32 *rfa_backup,
				     u32 *rfb_backup, u32 *backup_rf_reg,
				     u32 rf_num)
{
	u32 i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* Save RF Parameters */
	for (i = 0; i < rf_num; i++) {
		rfa_backup[i] = rtl_get_rfreg(rtlpriv, RF90_PATH_A, backup_rf_reg[i], bMaskDWord);
	}

	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BackupRF Success!!!!\n");
}

static void _rtl8812au_iqk_backup_afe(struct rtl_priv *rtlpriv,
	uint32_t *AFE_backup, uint32_t *Backup_AFE_REG, uint32_t AFE_NUM)
{
	uint32_t i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* Save AFE Parameters */
	for (i = 0; i < AFE_NUM; i++) {
		AFE_backup[i] = rtl_read_dword(rtlpriv, Backup_AFE_REG[i]);
	}
	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BackupAFE Success!!!!\n");
}

static void _rtl8821au_iqk_backup_afe(struct rtl_priv *rtlpriv, u32 *afe_backup,
				      u32 *backup_afe_REG, u32 afe_num)
{
	u32  i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* Save AFE Parameters */
	for (i = 0; i < afe_num; i++) {
		afe_backup[i] = rtl_read_dword(rtlpriv, backup_afe_REG[i]);
	}
	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BackupAFE Success!!!!\n");
}

static void _rtl8812au_iqk_restore_macbb(struct rtl_priv *rtlpriv,
	uint32_t *MACBB_backup, uint32_t *Backup_MACBB_REG, uint32_t MACBB_NUM)
{
	uint32_t i;
	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);     /* [31] = 0 --> Page C */
	/* Reload MacBB Parameters */
	for (i = 0; i < MACBB_NUM; i++) {
		rtl_write_dword(rtlpriv, Backup_MACBB_REG[i], MACBB_backup[i]);
	}
	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RestoreMacBB Success!!!!\n");
}

static void _rtl8821au_iqk_restore_macbb(struct rtl_priv *rtlpriv,
					 u32 *macbb_backup,
					 u32 *backup_macbb_reg,
					 u32 macbb_num)
{
	u32 i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0);     /* [31] = 0 --> Page C */
	/* Reload MacBB Parameters */
	for (i = 0; i < macbb_num; i++) {
		rtl_write_dword(rtlpriv, backup_macbb_reg[i], macbb_backup[i]);
	}
	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RestoreMacBB Success!!!!\n");
}

static void _rtl8812au_iqk_restore_rf(struct rtl_priv *rtlpriv,
	enum radio_path Path, uint32_t *Backup_RF_REG, uint32_t *RF_backup, uint32_t RF_REG_NUM)
{
	uint32_t i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /*  [31] = 0 --> Page C */
	for (i = 0; i < RF_REG_NUM; i++)
		rtl_set_rfreg(rtlpriv, Path, Backup_RF_REG[i], bRFRegOffsetMask, RF_backup[i]);

	rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x0);

	switch (Path) {
	case RF90_PATH_A:
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RestoreRF Path A Success!!!!\n");
		break;
	case RF90_PATH_B:
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RestoreRF Path B Success!!!!\n");
		break;
	default:
		break;
	}
}

static void _rtl8821au_iqk_restore_rf(struct rtl_priv *rtlpriv,
				      enum radio_path Path,
				      u32 *backup_rf_reg,
				      u32 *rf_backup, u32 rf_reg_num)
{
	u32 i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /*  [31] = 0 --> Page C */
	for (i = 0; i < rf_reg_num; i++)
		rtl_set_rfreg(rtlpriv, Path, backup_rf_reg[i], bRFRegOffsetMask, rf_backup[i]);

	switch (Path) {
	case RF90_PATH_A:
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RestoreRF Path A Success!!!!\n");
		break;
	default:
		break;
	}
}

static void _rtl8812au_iqk_restore_afe(struct rtl_priv *rtlpriv, uint32_t *AFE_backup,
	uint32_t *Backup_AFE_REG, uint32_t AFE_NUM)
{
	uint32_t i;

	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* Reload AFE Parameters */
	for (i = 0; i < AFE_NUM; i++) {
		rtl_write_dword(rtlpriv, Backup_AFE_REG[i], AFE_backup[i]);
	}
	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /*  [31] = 1 --> Page C1 */
	rtl_write_dword(rtlpriv, 0xc80, 0x0);
	rtl_write_dword(rtlpriv, 0xc84, 0x0);
	rtl_write_dword(rtlpriv, 0xc88, 0x0);
	rtl_write_dword(rtlpriv, 0xc8c, 0x3c000000);
	rtl_write_dword(rtlpriv, 0xcb8, 0x0);
	rtl_write_dword(rtlpriv, 0xe80, 0x0);
	rtl_write_dword(rtlpriv, 0xe84, 0x0);
	rtl_write_dword(rtlpriv, 0xe88, 0x0);
	rtl_write_dword(rtlpriv, 0xe8c, 0x3c000000);
	rtl_write_dword(rtlpriv, 0xeb8, 0x0);
	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RestoreAFE Success!!!!\n");
}

static void _rtl8821au_iqk_restore_afe(struct rtl_priv *rtlpriv,
				       u32 *afe_backup, u32 *backup_afe_reg,
				       u32 afe_num)
{
	uint32_t i;
	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	/* Reload AFE Parameters */
	for (i = 0; i < afe_num; i++) {
		rtl_write_dword(rtlpriv, backup_afe_reg[i], afe_backup[i]);
	}
	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */
	rtl_write_dword(rtlpriv, 0xc80, 0x0);
	rtl_write_dword(rtlpriv, 0xc84, 0x0);
	rtl_write_dword(rtlpriv, 0xc88, 0x0);
	rtl_write_dword(rtlpriv, 0xc8c, 0x3c000000);
	rtl_write_dword(rtlpriv, 0xcb8, 0x0);
	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RestoreAFE Success!!!!\n");
}


static void _rtl8812au_iqk_configure_mac(struct rtl_priv *rtlpriv)
{
	/* ========MAC register setting======== */
	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); 	/* [31] = 0 --> Page C */
	rtl_write_byte(rtlpriv, 0x522, 0x3f);
	rtl_set_bbreg(rtlpriv, 0x550, BIT(11)|BIT(3), 0x0);
	rtl_set_bbreg(rtlpriv, 0x808, BIT(28), 0x0);	/* CCK Off */
	rtl_write_byte(rtlpriv, 0x808, 0x00);		/* RX ante off */
	rtl_set_bbreg(rtlpriv, 0x838, 0xf, 0xc);		/* CCA off */
}

static void _rtl8821au_iqk_configure_mac(struct rtl_priv *rtlpriv)
{
	/* ========MAC register setting======== */
	rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
	rtl_write_byte(rtlpriv, 0x522, 0x3f);
	rtl_set_bbreg(rtlpriv, 0x550, BIT(3), 0x0);
	rtl_set_bbreg(rtlpriv, 0x551, BIT(3), 0x0);
	rtl_set_bbreg(rtlpriv, 0x808, BIT(28), 0x0);	/* CCK Off */
	rtl_write_byte(rtlpriv, 0x808, 0x00);		/* RX ante off */
	rtl_set_bbreg(rtlpriv, 0x838, 0xf, 0xc);		/* CCA off */
}


#undef MACBB_REG_NUM
#undef AFE_REG_NUM
#undef RF_REG_NUM
#define MACBB_REG_NUM 10
#define AFE_REG_NUM 14
#define RF_REG_NUM 3


/* Maintained by BB James. */
static void _rtl8812au_phy_iq_calibrate(struct rtl_priv *rtlpriv)
{
	uint32_t MACBB_backup[MACBB_REG_NUM], AFE_backup[AFE_REG_NUM], RFA_backup[RF_REG_NUM], RFB_backup[RF_REG_NUM];
	uint32_t Backup_MACBB_REG[MACBB_REG_NUM] = { 0xb00, 0x520, 0x550, 0x808, 0x90c, 0xc00, 0xe00, 0x8c4, 0x838, 0x82c };
	uint32_t Backup_AFE_REG[AFE_REG_NUM] = { 0xc5c, 0xc60, 0xc64, 0xc68, 0xcb8, 0xcb0, 0xcb4,
						 0xe5c, 0xe60, 0xe64, 0xe68, 0xeb8, 0xeb0, 0xeb4 };
	uint32_t Backup_RF_REG[RF_REG_NUM] = { 0x65, 0x8f, 0x0 };


	_rtl8812au_iqk_backup_macbb(rtlpriv, MACBB_backup, Backup_MACBB_REG, MACBB_REG_NUM);
	_rtl8812au_iqk_backup_afe(rtlpriv, AFE_backup, Backup_AFE_REG, AFE_REG_NUM);
	_rtl8812au_iqk_backup_rf(rtlpriv, RFA_backup, RFB_backup, Backup_RF_REG, RF_REG_NUM);

	_rtl8812au_iqk_configure_mac(rtlpriv);
	_rtl8812au_iqk_tx(rtlpriv, RF90_PATH_A);
	_rtl8812au_iqk_restore_rf(rtlpriv, RF90_PATH_A, Backup_RF_REG, RFA_backup, RF_REG_NUM);

	_rtl8812au_iqk_tx(rtlpriv, RF90_PATH_B);
	_rtl8812au_iqk_restore_rf(rtlpriv, RF90_PATH_B, Backup_RF_REG, RFB_backup, RF_REG_NUM);

	_rtl8812au_iqk_restore_afe(rtlpriv, AFE_backup, Backup_AFE_REG, AFE_REG_NUM);
	_rtl8812au_iqk_restore_macbb(rtlpriv, MACBB_backup, Backup_MACBB_REG, MACBB_REG_NUM);
}



void rtl8812au_phy_iq_calibrate(struct rtl_priv *rtlpriv, bool bReCovery)
{
	_rtl8812au_phy_iq_calibrate(rtlpriv);
}



/* ****************************************************************************** */
/*									*/
/*  from HalPhyRf_8821A.c						*/
/*									*/
/* ****************************************************************************** */








#define cal_num 3

static void _rtl8821au_iqk_tx(struct rtl_priv *rtlpriv, enum radio_path Path)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

	uint32_t TX_fail, RX_fail, delay_count, IQK_ready, cal_retry, cal = 0, temp_reg65;
	int 	TX_X = 0, TX_Y = 0, RX_X = 0, RX_Y = 0, TX_Average = 0, RX_Average = 0;
	int 	TX_X0[cal_num], TX_Y0[cal_num], TX_X0_RXK[cal_num], TX_Y0_RXK[cal_num], RX_X0[cal_num], RX_Y0[cal_num];
	bool TX0IQKOK = false, RX0IQKOK = false;
	bool VDF_enable = false;
	int 	i, k, VDF_Y[3], VDF_X[3], Tx_dt[3], Rx_dt[3], ii, dx = 0, dy = 0, TX_finish = 0, RX_finish = 0;

	RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "BandWidth = %d\n", rtlpriv->phy.current_chan_bw);
	if (rtlpriv->phy.current_chan_bw == 2) {
		VDF_enable = true;
	}

	while (cal < cal_num) {
		switch (Path) {
		case RF90_PATH_A:
		    {
			temp_reg65 = rtl_get_rfreg(rtlpriv, Path, 0x65, bMaskDWord);

			if (rtlhal->external_pa_2g) {
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
				rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d5);
			}

			/* Path-A LOK */
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			/* ========Path-A AFE all on======== */
			/* Port 0 DAC/ADC on */
			rtl_write_dword(rtlpriv, 0xc60, 0x77777777);
			rtl_write_dword(rtlpriv, 0xc64, 0x77777777);

			rtl_write_dword(rtlpriv, 0xc68, 0x19791979);
			rtl_write_dword(rtlpriv, 0xc6c, 0x19791979);
			rtl_write_dword(rtlpriv, 0xc70, 0x19791979);
			rtl_write_dword(rtlpriv, 0xc74, 0x19791979);
			rtl_write_dword(rtlpriv, 0xc78, 0x19791979);
			rtl_write_dword(rtlpriv, 0xc7c, 0x19791979);
			rtl_write_dword(rtlpriv, 0xc80, 0x19791979);
			rtl_write_dword(rtlpriv, 0xc84, 0x19791979);

			rtl_set_bbreg(rtlpriv, 0xc00, 0xf, 0x4);	/* 	hardware 3-wire off */

			/* LOK Setting */
			/* ====== LOK ====== */
			/* 1. DAC/ADC sampling rate (160 MHz) */
			rtl_set_bbreg(rtlpriv, 0xc5c, BIT(26)|BIT(25)|BIT(24), 0x7);

			/* 2. LoK RF Setting (at BW = 20M) */
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80002);
			rtl_set_rfreg(rtlpriv, Path, 0x18, 0x00c00, 0x3);     /* BW 20M */
			rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x20000);
			rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x0003f);
			rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xf3fc3);
			rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d5);
			rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x8a001);
			rtl_set_bbreg(rtlpriv, 0xcb8, 0xf, 0xd);
			rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
			rtl_write_dword(rtlpriv, 0xb00, 0x03000100);
			rtl_set_bbreg(rtlpriv, 0xc94, BIT(0), 0x1);
			rtl_write_dword(rtlpriv, 0x978, 0x29002000);	/* TX (X,Y) */
			rtl_write_dword(rtlpriv, 0x97c, 0xa9002000);	/* RX (X,Y) */
			rtl_write_dword(rtlpriv, 0x984, 0x00462910);	/* [0]:AGC_en, [15]:idac_K_Mask */

			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); 	/* [31] = 1 --> Page C1 */

			if (rtlhal->external_pa_2g)
				rtl_write_dword(rtlpriv, 0xc88, 0x821403f7);
			else
				rtl_write_dword(rtlpriv, 0xc88, 0x821403f4);

			if (rtlhal->current_bandtype)
				rtl_write_dword(rtlpriv, 0xc8c, 0x68163e96);
			else
				rtl_write_dword(rtlpriv, 0xc8c, 0x28163e96);

			rtl_write_dword(rtlpriv, 0xc80, 0x18008c10);/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
			rtl_write_dword(rtlpriv, 0xc84, 0x38008c10);/* RX_Tone_idx[9:0], RxK_Mask[29] */
			rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
			rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
			rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

			mdelay(10); /* Delay 10ms */
			rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);

			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			rtl_set_rfreg(rtlpriv, Path, 0x58, 0x7fe00, rtl_get_rfreg(rtlpriv, Path, 0x8, 0xffc00)); /* Load LOK */
			switch (rtlpriv->phy.current_chan_bw) {
			case 1:
				rtl_set_rfreg(rtlpriv, Path, 0x18, 0x00c00, 0x1);
				break;
			case 2:
				rtl_set_rfreg(rtlpriv, Path, 0x18, 0x00c00, 0x0);
				break;
			default:
				break;
			}
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */

			/* 3. TX RF Setting */
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
			rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x20000);
			rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x0003f);
			rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xf3fc3);
			rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d5);
			rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x8a001);
			rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);
			rtl_set_bbreg(rtlpriv, 0xcb8, 0xf, 0xd);
			rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
			rtl_write_dword(rtlpriv, 0xb00, 0x03000100);
			rtl_set_bbreg(rtlpriv, 0xc94, BIT(0), 0x1);
			rtl_write_dword(rtlpriv, 0x978, 0x29002000);/* TX (X,Y) */
			rtl_write_dword(rtlpriv, 0x97c, 0xa9002000);/* RX (X,Y) */
			rtl_write_dword(rtlpriv, 0x984, 0x0046a910);/* [0]:AGC_en, [15]:idac_K_Mask */

			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */

			if (rtlhal->external_pa_2g)
				rtl_write_dword(rtlpriv, 0xc88, 0x821403f7);
			else
				rtl_write_dword(rtlpriv, 0xc88, 0x821403f1);

			if (rtlhal->current_bandtype)
				rtl_write_dword(rtlpriv, 0xc8c, 0x40163e96);
			else
				rtl_write_dword(rtlpriv, 0xc8c, 0x00163e96);

			if (VDF_enable == 1) {
				RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_enable\n");
				for (k = 0; k <= 2; k++) {
					switch (k) {
					case 0:
						rtl_write_dword(rtlpriv, 0xc80, 0x18008c38);/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
						rtl_write_dword(rtlpriv, 0xc84, 0x38008c38);/* RX_Tone_idx[9:0], RxK_Mask[29] */
						rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x0);
						break;
					case 1:
						rtl_set_bbreg(rtlpriv, 0xc80, BIT(28), 0x0);
						rtl_set_bbreg(rtlpriv, 0xc84, BIT(28), 0x0);
						rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x0);
						break;
					case 2:
						RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_Y[1] = %x;;;VDF_Y[0] = %x\n", VDF_Y[1]>>21 & 0x00007ff, VDF_Y[0]>>21 & 0x00007ff);
						RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_X[1] = %x;;;VDF_X[0] = %x\n", VDF_X[1]>>21 & 0x00007ff, VDF_X[0]>>21 & 0x00007ff);
						Tx_dt[cal] = (VDF_Y[1]>>20)-(VDF_Y[0]>>20);
						Tx_dt[cal] = ((16*Tx_dt[cal])*10000/15708);
						Tx_dt[cal] = (Tx_dt[cal] >> 1)+(Tx_dt[cal] & BIT(0));
						rtl_write_dword(rtlpriv, 0xc80, 0x18008c20);/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
						rtl_write_dword(rtlpriv, 0xc84, 0x38008c20);/* RX_Tone_idx[9:0], RxK_Mask[29] */
						rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x1);
						rtl_set_bbreg(rtlpriv, 0xce8, 0x3fff0000, Tx_dt[cal] & 0x00003fff);
						break;
					default:
						break;
					}
					rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);	/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
					cal_retry = 0;
					while (1) {
						/* one shot */
						rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
						rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

						mdelay(10); 	/* Delay 10ms */
						rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
						delay_count = 0;
						while (1) {
							IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
							if ((~IQK_ready) || (delay_count > 20)) {
								break;
							} else {
								mdelay(1);
								delay_count++;
							}
						}

						if (delay_count < 20) {			/* If 20ms No Result, then cal_retry++ */
						/* ============TXIQK Check============== */
							TX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(12));

							if (~TX_fail) {
								rtl_write_dword(rtlpriv, 0xcb8, 0x02000000);
								VDF_X[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
								rtl_write_dword(rtlpriv, 0xcb8, 0x04000000);
								VDF_Y[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
								TX0IQKOK = true;
								break;
							} else {
								rtl_set_bbreg(rtlpriv, 0xccc, 0x000007ff, 0x0);
								rtl_set_bbreg(rtlpriv, 0xcd4, 0x000007ff, 0x200);
								TX0IQKOK = false;
								cal_retry++;
								if (cal_retry == 10) {
									break;
								}
							}
						} else {
							TX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10) {
								break;
							}
						}
					}
				}

				if (k == 3) {
					TX_X0[cal] = VDF_X[k-1] ;
					TX_Y0[cal] = VDF_Y[k-1];
				}
			} else {
				rtl_write_dword(rtlpriv, 0xc80, 0x18008c10);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
				rtl_write_dword(rtlpriv, 0xc84, 0x38008c10);	/* RX_Tone_idx[9:0], RxK_Mask[29]  */
				rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);	/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
				cal_retry = 0;
				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); /*  Delay 10ms */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
						if ((~IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {		/* If 20ms No Result, then cal_retry++ */
						/* ============TXIQK Check============== */
						TX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(12));
						if (~TX_fail) {
							rtl_write_dword(rtlpriv, 0xcb8, 0x02000000);
							TX_X0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xcb8, 0x04000000);
							TX_Y0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							TX0IQKOK = true;
							break;
						} else {
							rtl_set_bbreg(rtlpriv, 0xccc, 0x000007ff, 0x0);
							rtl_set_bbreg(rtlpriv, 0xcd4, 0x000007ff, 0x200);
							TX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10) {
								break;
							}
						}
					} else {
						TX0IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
							break;
					}
				}
			}

			if (TX0IQKOK == false)
				break;					/* TXK fail, Don't do RXK */

			if (VDF_enable == 1) {
				rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x0);    /* TX VDF Disable */
				RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXVDF Start\n");
				for (k = 0; k <= 2; k++) {
					/* ====== RX mode TXK (RXK Step 1) ====== */
					rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
					/*  1. TX RF Setting */
					rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
					rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
					rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x00029);
					rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xd7ffb);
					rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, temp_reg65);
					rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x8a001);
					rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);

					rtl_set_bbreg(rtlpriv, 0xcb8, 0xf, 0xd);
					rtl_write_dword(rtlpriv, 0x978, 0x29002000);/* TX (X,Y) */
					rtl_write_dword(rtlpriv, 0x97c, 0xa9002000);/* RX (X,Y) */
					rtl_write_dword(rtlpriv, 0x984, 0x0046a910);/* [0]:AGC_en, [15]:idac_K_Mask */
					rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
					rtl_write_dword(rtlpriv, 0xb00, 0x03000100);
					rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */
					switch (k) {
					case 0:
						rtl_write_dword(rtlpriv, 0xc80, 0x18008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
						rtl_write_dword(rtlpriv, 0xc84, 0x38008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
						rtl_set_bbreg(rtlpriv, 0xce8, BIT(30), 0x0);
						break;
					case 1:
						rtl_write_dword(rtlpriv, 0xc80, 0x08008c38);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
						rtl_write_dword(rtlpriv, 0xc84, 0x28008c38);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
						rtl_set_bbreg(rtlpriv, 0xce8, BIT(30), 0x0);
						break;
					case 2:
						RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_Y[1] = %x;;;VDF_Y[0] = %x\n", VDF_Y[1]>>21 & 0x00007ff, VDF_Y[0]>>21 & 0x00007ff);
						RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "VDF_X[1] = %x;;;VDF_X[0] = %x\n", VDF_X[1]>>21 & 0x00007ff, VDF_X[0]>>21 & 0x00007ff);
						Rx_dt[cal] = (VDF_Y[1]>>20)-(VDF_Y[0]>>20);
						RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Rx_dt = %d\n", Rx_dt[cal]);
						Rx_dt[cal] = ((16*Rx_dt[cal])*10000/13823);
						Rx_dt[cal] = (Rx_dt[cal] >> 1)+(Rx_dt[cal] & BIT(0));
						rtl_write_dword(rtlpriv, 0xc80, 0x18008c20);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
						rtl_write_dword(rtlpriv, 0xc84, 0x38008c20);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
						rtl_set_bbreg(rtlpriv, 0xce8, 0x00003fff, Rx_dt[cal] & 0x00003fff);
						break;
					default:
						break;
					}

					rtl_write_dword(rtlpriv, 0xc88, 0x821603e0);
					rtl_write_dword(rtlpriv, 0xc8c, 0x68163e96);
					rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);	/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
					cal_retry = 0;
					while (1) {
						/* one shot */
						rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
						rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

						mdelay(10); 	/* Delay 10ms */
						rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
						delay_count = 0;
						while (1) {
							IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
							if ((~IQK_ready) || (delay_count > 20)) {
								break;
							} else {
								mdelay(1);
								delay_count++;
							}
						}

						if (delay_count < 20) {		/* If 20ms No Result, then cal_retry++ */
							/* ============TXIQK Check============== */
							TX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(12));

							if (~TX_fail) {
								rtl_write_dword(rtlpriv, 0xcb8, 0x02000000);
								TX_X0_RXK[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
								rtl_write_dword(rtlpriv, 0xcb8, 0x04000000);
								TX_Y0_RXK[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
								TX0IQKOK = true;
								break;
							} else {
								TX0IQKOK = false;
								cal_retry++;
								if (cal_retry == 10)
									break;
							}
						} else {
							TX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10)
								break;
						}
					}

					if (TX0IQKOK == false) {   /* If RX mode TXK fail, then take TXK Result */
						TX_X0_RXK[cal] = TX_X0[cal];
						TX_Y0_RXK[cal] = TX_Y0[cal];
						TX0IQKOK = true;
						RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RXK Step 1 fail\n");
					}

					/* ====== RX IQK ====== */
					rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
					/* 1. RX RF Setting */
					rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
					rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
					rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x0002f);
					rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfffbb);
					rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x88001);
					rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d8);
					rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);

					rtl_set_bbreg(rtlpriv, 0x978, 0x03FF8000, (TX_X0_RXK[cal])>>21&0x000007ff);
					rtl_set_bbreg(rtlpriv, 0x978, 0x000007FF, (TX_Y0_RXK[cal])>>21&0x000007ff);
					rtl_set_bbreg(rtlpriv, 0x978, BIT(31), 0x1);
					rtl_set_bbreg(rtlpriv, 0x97c, BIT(31), 0x0);
					rtl_set_bbreg(rtlpriv, 0xcb8, 0xF, 0xe);
					rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
					rtl_write_dword(rtlpriv, 0x984, 0x0046a911);

					rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); /* [31] = 1 --> Page C1 */
					rtl_set_bbreg(rtlpriv, 0xc80, BIT(29), 0x1);
					rtl_set_bbreg(rtlpriv, 0xc84, BIT(29), 0x0);
					rtl_write_dword(rtlpriv, 0xc88, 0x02140119);
					rtl_write_dword(rtlpriv, 0xc8c, 0x28161420);

					if (k == 2) {
						rtl_set_bbreg(rtlpriv, 0xce8, BIT(30), 0x1);  /* RX VDF Enable */
					}
					rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);	/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */

					cal_retry = 0;
					while (1) {
						/* one shot */
						rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
						rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

						mdelay(10); /* Delay 10ms */
						rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
						delay_count = 0;
						while (1) {
							IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
							if ((~IQK_ready) || (delay_count > 20)) {
								break;
							} else {
								mdelay(1);
								delay_count++;
							}
						}

						if (delay_count < 20) {	/* If 20ms No Result, then cal_retry++ */
							/* ============RXIQK Check============== */
							RX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(11));
							if (RX_fail == 0) {
								rtl_write_dword(rtlpriv, 0xcb8, 0x06000000);
								VDF_X[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
								rtl_write_dword(rtlpriv, 0xcb8, 0x08000000);
								VDF_Y[k] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
								RX0IQKOK = true;
								break;
							} else {
								rtl_set_bbreg(rtlpriv, 0xc10, 0x000003ff, 0x200>>1);
								rtl_set_bbreg(rtlpriv, 0xc10, 0x03ff0000, 0x0>>1);
								RX0IQKOK = false;
								cal_retry++;
								if (cal_retry == 10)
									break;
							}
						} else {
							RX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10)
								break;
						}
					}
				}

				if (k == 3) {
					RX_X0[cal] = VDF_X[k-1] ;
					RX_Y0[cal] = VDF_Y[k-1];
				}

				rtl_set_bbreg(rtlpriv, 0xce8, BIT(31), 0x1);    /* TX VDF Enable */
			} else {
				/* ====== RX mode TXK (RXK Step 1) ====== */
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
				/* 1. TX RF Setting */
				rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
				rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
				rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x00029);
				rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xd7ffb);
				rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, temp_reg65);
				rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x8a001);
				rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);
				rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
				rtl_write_dword(rtlpriv, 0xb00, 0x03000100);
				rtl_write_dword(rtlpriv, 0x984, 0x0046a910);/* [0]:AGC_en, [15]:idac_K_Mask */

				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1);	/* [31] = 1 --> Page C1 */
				rtl_write_dword(rtlpriv, 0xc80, 0x18008c10);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
				rtl_write_dword(rtlpriv, 0xc84, 0x38008c10);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
				rtl_write_dword(rtlpriv, 0xc88, 0x821603e0);
				/* rtl_write_dword(rtlpriv, 0xc8c, 0x68163e96); */
				rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */
				cal_retry = 0;
				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); 	/* Delay 10ms */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
						if ((~IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {							/* If 20ms No Result, then cal_retry++ */
						/* ============TXIQK Check============== */
						TX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(12));

						if (~TX_fail) {
							rtl_write_dword(rtlpriv, 0xcb8, 0x02000000);
							TX_X0_RXK[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xcb8, 0x04000000);
							TX_Y0_RXK[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							TX0IQKOK = true;
							break;
						} else {
							TX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10)
								break;
						}
					} else {
						TX0IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
							break;
					}
				}

				if (TX0IQKOK == false) {	/* If RX mode TXK fail, then take TXK Result */
					TX_X0_RXK[cal] = TX_X0[cal];
					TX_Y0_RXK[cal] = TX_Y0[cal];
					TX0IQKOK = true;
					RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "1");
				}

				/* ====== RX IQK ====== */
				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); /* [31] = 0 --> Page C */
				/* 1. RX RF Setting */
				rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x80000);
				rtl_set_rfreg(rtlpriv, Path, 0x30, bRFRegOffsetMask, 0x30000);
				rtl_set_rfreg(rtlpriv, Path, 0x31, bRFRegOffsetMask, 0x0002f);
				rtl_set_rfreg(rtlpriv, Path, 0x32, bRFRegOffsetMask, 0xfffbb);
				rtl_set_rfreg(rtlpriv, Path, 0x8f, bRFRegOffsetMask, 0x88001);
				rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, 0x931d8);
				rtl_set_rfreg(rtlpriv, Path, 0xef, bRFRegOffsetMask, 0x00000);

				rtl_set_bbreg(rtlpriv, 0x978, 0x03FF8000, (TX_X0_RXK[cal])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x978, 0x000007FF, (TX_Y0_RXK[cal])>>21&0x000007ff);
				rtl_set_bbreg(rtlpriv, 0x978, BIT(31), 0x1);
				rtl_set_bbreg(rtlpriv, 0x97c, BIT(31), 0x0);
				rtl_set_bbreg(rtlpriv, 0xcb8, 0xF, 0xe);
				rtl_write_dword(rtlpriv, 0x90c, 0x00008000);
				rtl_write_dword(rtlpriv, 0x984, 0x0046a911);

				rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x1); 	/* [31] = 1 --> Page C1 */
				rtl_write_dword(rtlpriv, 0xc80, 0x38008c10);	/* TX_Tone_idx[9:0], TxK_Mask[29] TX_Tone = 16 */
				rtl_write_dword(rtlpriv, 0xc84, 0x18008c10);	/* RX_Tone_idx[9:0], RxK_Mask[29] */
				rtl_write_dword(rtlpriv, 0xc88, 0x02140119);
				rtl_write_dword(rtlpriv, 0xc8c, 0x28161440);
				rtl_write_dword(rtlpriv, 0xcb8, 0x00100000);	/* cb8[20] ±N SI/PI ¨Ï¥ÎÅv¤Áµ¹ iqk_dpk module */

				cal_retry = 0;
				while (1) {
					/* one shot */
					rtl_write_dword(rtlpriv, 0x980, 0xfa000000);
					rtl_write_dword(rtlpriv, 0x980, 0xf8000000);

					mdelay(10); /* Delay 10ms */
					rtl_write_dword(rtlpriv, 0xcb8, 0x00000000);
					delay_count = 0;
					while (1) {
						IQK_ready = rtl_get_bbreg(rtlpriv, 0xd00, BIT(10));
						if ((~IQK_ready) || (delay_count > 20)) {
							break;
						} else {
							mdelay(1);
							delay_count++;
						}
					}

					if (delay_count < 20) {	/* If 20ms No Result, then cal_retry++ */
						/* ============RXIQK Check============== */
						RX_fail = rtl_get_bbreg(rtlpriv, 0xd00, BIT(11));
						if (RX_fail == 0) {
							rtl_write_dword(rtlpriv, 0xcb8, 0x06000000);
							RX_X0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							rtl_write_dword(rtlpriv, 0xcb8, 0x08000000);
							RX_Y0[cal] = rtl_get_bbreg(rtlpriv, 0xd00, 0x07ff0000)<<21;
							RX0IQKOK = true;
							break;
						} else {
							rtl_set_bbreg(rtlpriv, 0xc10, 0x000003ff, 0x200>>1);
							rtl_set_bbreg(rtlpriv, 0xc10, 0x03ff0000, 0x0>>1);
							RX0IQKOK = false;
							cal_retry++;
							if (cal_retry == 10)
								break;

						}
					} else {
						RX0IQKOK = false;
						cal_retry++;
						if (cal_retry == 10)
							break;
					}
				}
			}
			if (TX0IQKOK)
				TX_Average++;
			if (RX0IQKOK)
				RX_Average++;
			rtl_set_bbreg(rtlpriv, 0x82c, BIT(31), 0x0); 	/* [31] = 0 --> Page C */
			rtl_set_rfreg(rtlpriv, Path, 0x65, bRFRegOffsetMask, temp_reg65);
		    }
			break;
		default:
			break;
		}
		cal++;
	}
	/* FillIQK Result */
	switch (Path) {
	case RF90_PATH_A:
	    {
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "========Path_A =======\n");
		if (TX_Average == 0)
			break;

		for (i = 0; i < TX_Average; i++) {
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, " TX_X0_RXK[%d] = %x ;; TX_Y0_RXK[%d] = %x\n", i, (TX_X0_RXK[i])>>21&0x000007ff, i, (TX_Y0_RXK[i])>>21&0x000007ff);
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "TX_X0[%d] = %x ;; TX_Y0[%d] = %x\n", i, (TX_X0[i])>>21&0x000007ff, i, (TX_Y0[i])>>21&0x000007ff);
		}

		for (i = 0; i < TX_Average; i++) {
			for (ii = i+1; ii < TX_Average; ii++) {
				dx = (TX_X0[i]>>21) - (TX_X0[ii]>>21);
				if (dx < 3 && dx > -3) {
					dy = (TX_Y0[i]>>21) - (TX_Y0[ii]>>21);
					if (dy < 3 && dy > -3) {
						TX_X = ((TX_X0[i]>>21) + (TX_X0[ii]>>21))/2;
						TX_Y = ((TX_Y0[i]>>21) + (TX_Y0[ii]>>21))/2;
						TX_finish = 1;
						break;
					}
				}
			}
			if (TX_finish == 1)
				break;
		}

		if (TX_finish == 1) {
			_rtl8821au_iqk_tx_fill_iqc(rtlpriv, Path, TX_X, TX_Y);
		} else {
			_rtl8821au_iqk_tx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
		}

		if (RX_Average == 0)
			break;

		for (i = 0; i < RX_Average; i++) {
			RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "RX_X0[%d] = %x ;; RX_Y0[%d] = %x\n", i, (RX_X0[i])>>21&0x000007ff, i, (RX_Y0[i])>>21&0x000007ff);
		}

		for (i = 0; i < RX_Average; i++) {
			for (ii = i+1; ii < RX_Average; ii++) {
				dx = (RX_X0[i]>>21) - (RX_X0[ii]>>21);
				if (dx < 3 && dx > -3) {
					dy = (RX_Y0[i]>>21) - (RX_Y0[ii]>>21);
					if (dy < 3 && dy > -3) {
						RX_X = ((RX_X0[i]>>21) + (RX_X0[ii]>>21))/2;
						RX_Y = ((RX_Y0[i]>>21) + (RX_Y0[ii]>>21))/2;
						RX_finish = 1;
						break;
					}
				}
			}
			if (RX_finish == 1)
				break;
		}

		if (RX_finish == 1) {
			 _rtl8821au_iqk_rx_fill_iqc(rtlpriv, Path, RX_X, RX_Y);
		} else {
			 _rtl8821au_iqk_rx_fill_iqc(rtlpriv, Path, 0x200, 0x0);
		}
	    }
		break;
	default:
		break;
	}
}

#undef MACBB_REG_NUM
#undef AFE_REG_NUM
#undef RF_REG_NUM
#define MACBB_REG_NUM 11
#define AFE_REG_NUM 12
#define RF_REG_NUM 3


static void _rtl8821au_phy_iq_calibrate(struct rtl_priv *rtlpriv)
{
	u32 macbb_backup[MACBB_REG_NUM];
	u32 afe_backup[AFE_REG_NUM];
	u32 rfa_backup[RF_REG_NUM];
	u32 rfb_backup[RF_REG_NUM];
	u32 backup_macbb_reg[MACBB_REG_NUM] = {
		0xb00, 0x520, 0x550, 0x808, 0x90c, 0xc00, 0xc50,
		0xe00, 0xe50, 0x838, 0x82c
	};
	u32 backup_afe_reg[AFE_REG_NUM] = {
		0xc5c, 0xc60, 0xc64, 0xc68, 0xc6c, 0xc70, 0xc74,
		0xc78, 0xc7c, 0xc80, 0xc84, 0xcb8
	};
	u32 backup_rf_reg[RF_REG_NUM] = { 0x65, 0x8f, 0x0 };

	_rtl8821au_iqk_backup_macbb(rtlpriv, macbb_backup, backup_macbb_reg,
				    MACBB_REG_NUM);
	_rtl8821au_iqk_backup_afe(rtlpriv, afe_backup, backup_afe_reg, AFE_REG_NUM);
	_rtl8821au_iqk_backup_rf(rtlpriv, rfa_backup, rfb_backup, backup_rf_reg,
				 RF_REG_NUM);

	_rtl8821au_iqk_configure_mac(rtlpriv);
	_rtl8821au_iqk_tx(rtlpriv, RF90_PATH_A);
	_rtl8821au_iqk_restore_rf(rtlpriv, RF90_PATH_A, backup_rf_reg, rfa_backup,
				 RF_REG_NUM);

	_rtl8821au_iqk_restore_afe(rtlpriv, afe_backup, backup_afe_reg, AFE_REG_NUM);
	_rtl8821au_iqk_restore_macbb(rtlpriv, macbb_backup, backup_macbb_reg,
				     MACBB_REG_NUM);

	/* _IQK_Exit_8821A(pDM_Odm); */
	/* _IQK_TX_CheckResult_8821A */
}

void rtl8821au_phy_iq_calibrate(struct rtl_priv *rtlpriv, bool bReCovery)
{
	_rtl8821au_phy_iq_calibrate(rtlpriv);
}


/* ********************************************************** */

/* OLD functions need complete ? rewrite */
bool Getu8IntegerFromStringInDecimal(s8 *Str, uint8_t *pInt)
{
	u16 i = 0;
	*pInt = 0;

	while (Str[i] != '\0') {
		if (Str[i] >= '0' && Str[i] <= '9') {
			*pInt *= 10;
			*pInt += (Str[i] - '0');
		} else {
			return false;
		}
		++i;
	}

	return true;
}


static bool eqNByte(uint8_t *str1, uint8_t *str2, uint32_t num)
{
	if (num == 0)
		return false;

	while (num > 0) {
		num--;
		if (str1[num] != str2[num])
			return false;
	}

	return true;
}


static s8 phy_GetChannelGroup(struct rtl_priv *rtlpriv, enum band_type Band, uint8_t Channel)
{
	s8 channelGroup = -1;

	if (Channel <= 14 && Band == BAND_ON_2_4G) {
		if (1 <= Channel && Channel <= 2)
			channelGroup = 0;
		else if (3  <= Channel && Channel <= 5)
			channelGroup = 1;
		else if (6  <= Channel && Channel <= 8)
			channelGroup = 2;
		else if (9  <= Channel && Channel <= 11)
			channelGroup = 3;
		else if (12 <= Channel && Channel <= 14)
			channelGroup = 4;
		else {
			RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "==> phy_GetChannelGroup() in 2.4 G, but Channel %d in Group not found \n", Channel);
			channelGroup = -1;
		}
	} else if (Band == BAND_ON_5G) {
		if (36 <= Channel && Channel <=  42)
			channelGroup = 0;
		else if (44 <= Channel && Channel <= 48)
			channelGroup = 1;
		else if (50 <= Channel && Channel <= 58)
			channelGroup = 2;
		else if (60 <= Channel && Channel <= 64)
			channelGroup = 3;
		else if (100 <= Channel && Channel <= 106)
			channelGroup = 4;
		else if (108 <= Channel && Channel <= 114)
			channelGroup = 5;
		else if (116 <= Channel && Channel <= 122)
			channelGroup = 6;
		else if (124 <= Channel && Channel <= 130)
			channelGroup = 7;
		else if (132 <= Channel && Channel <= 138)
			channelGroup = 8;
		else if (140 <= Channel && Channel <= 144)
			channelGroup = 9;
		else if (149 <= Channel && Channel <= 155)
			channelGroup = 10;
		else if (157 <= Channel && Channel <= 161)
			channelGroup = 11;
		else if (165 <= Channel && Channel <= 171)
			channelGroup = 12;
		else if (173 <= Channel && Channel <= 177)
			channelGroup = 13;
		else {
			RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "==>phy_GetChannelGroup() in 5G, but Channel %d in Group not found \n", Channel);
			channelGroup = -1;
		}
	} else {
		RT_TRACE(rtlpriv, COMP_EFUSE, DBG_LOUD, "==>phy_GetChannelGroup() in unsupported band %d\n", Band);
		channelGroup = -1;
	}

	return channelGroup;
}

s8 phy_GetWorldWideLimit(s8* LimitTable)
{
	s8	min = LimitTable[0];
	u8	i = 0;
	
	for (i = 0; i < MAX_REGULATION_NUM; ++i) {
		if (LimitTable[i] < min)
			min = LimitTable[i];
	}

	return min;
}

static char _rtl8821au_get_chnl_idx_of_txpwr_limit(struct rtl_priv *rtlpriv,
				     u8 Band, u8 Channel)
{
	s8 channelIndex = -1;
	u8 channel5G[CHANNEL_MAX_NUMBER_5G] =
				 {36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,100,102,104,106,108,110,112,
				114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,149,151,
				153,155,157,159,161,163,165,167,168,169,171,173,175,177};
	u8 i = 0;
	if (Band == BAND_ON_2_4G) {
		channelIndex = Channel - 1;
	} else if ( Band == BAND_ON_5G ) {
		for ( i = 0; i < sizeof(channel5G)/sizeof(u8); ++i ) {
			if ( channel5G[i] == Channel )
				channelIndex = i;
		}
	} else {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid Band %d in %s",
			 Band, __func__);
	}

	if (channelIndex == -1)
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid Channel %d of Band %d in %s",
			 Channel, Band, __func__);

	return channelIndex;
}

static char _rtl8821au_phy_get_txpower_limit(struct rtl_priv *rtlpriv,
		       enum band_type band,
		       enum CHANNEL_WIDTH bandwidth,
		       enum radio_path rf_path,
		       u8 rate, u8 channel)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	short band_temp = -1, regulation = -1, bandwidth_temp = -1,
		rate_section = -1, channel_temp = 0-1;
	char powerLimit = MAX_POWER_INDEX;

	/* ULLI : I'm lazy taken form rtl8821ae driver */
	
	if (rtlefuse->eeprom_regulatory == 2)
		return MAX_POWER_INDEX;

	regulation = TXPWR_LMT_WW;

	//DBG_871X("pMgntInfo->RegPwrTblSel %d, final regulation %d\n", Adapter->registrypriv.RegPwrTblSel, regulation );


	if (band == BAND_ON_2_4G)
		band_temp = 0;
	else if (band == BAND_ON_5G)
		band_temp = 1;

	if (bandwidth == CHANNEL_WIDTH_20)
		bandwidth_temp = 0;
	else if (bandwidth == CHANNEL_WIDTH_40)
		bandwidth_temp = 1;
	else if (bandwidth == CHANNEL_WIDTH_80)
		bandwidth_temp = 2;
	else if (bandwidth == CHANNEL_WIDTH_160)
		bandwidth_temp = 3;

	switch (rate) {
	case MGN_1M:
	case MGN_2M:
	case MGN_5_5M:
	case MGN_11M:
		rate_section = 0;
		break;

	case MGN_6M:
	case MGN_9M:
	case MGN_12M:
	case MGN_18M:
	case MGN_24M:
	case MGN_36M:
	case MGN_48M:
	case MGN_54M:
		rate_section = 1;
		break;

	case MGN_MCS0:
	case MGN_MCS1:
	case MGN_MCS2:
	case MGN_MCS3:
	case MGN_MCS4:
	case MGN_MCS5:
	case MGN_MCS6:
	case MGN_MCS7:
		rate_section = 2;
		break;

	case MGN_MCS8:
	case MGN_MCS9:
	case MGN_MCS10:
	case MGN_MCS11:
	case MGN_MCS12:
	case MGN_MCS13:
	case MGN_MCS14:
	case MGN_MCS15:
		rate_section = 3;
		break;

	case MGN_MCS16:
	case MGN_MCS17:
	case MGN_MCS18:
	case MGN_MCS19:
	case MGN_MCS20:
	case MGN_MCS21:
	case MGN_MCS22:
	case MGN_MCS23:
		rate_section = 4;
		break;

	case MGN_MCS24:
	case MGN_MCS25:
	case MGN_MCS26:
	case MGN_MCS27:
	case MGN_MCS28:
	case MGN_MCS29:
	case MGN_MCS30:
	case MGN_MCS31:
		rate_section = 5;
		break;

	case MGN_VHT1SS_MCS0:
	case MGN_VHT1SS_MCS1:
	case MGN_VHT1SS_MCS2:
	case MGN_VHT1SS_MCS3:
	case MGN_VHT1SS_MCS4:
	case MGN_VHT1SS_MCS5:
	case MGN_VHT1SS_MCS6:
	case MGN_VHT1SS_MCS7:
	case MGN_VHT1SS_MCS8:
	case MGN_VHT1SS_MCS9:
		rate_section = 6;
		break;

	case MGN_VHT2SS_MCS0:
	case MGN_VHT2SS_MCS1:
	case MGN_VHT2SS_MCS2:
	case MGN_VHT2SS_MCS3:
	case MGN_VHT2SS_MCS4:
	case MGN_VHT2SS_MCS5:
	case MGN_VHT2SS_MCS6:
	case MGN_VHT2SS_MCS7:
	case MGN_VHT2SS_MCS8:
	case MGN_VHT2SS_MCS9:
		rate_section = 7;
		break;

	case MGN_VHT3SS_MCS0:
	case MGN_VHT3SS_MCS1:
	case MGN_VHT3SS_MCS2:
	case MGN_VHT3SS_MCS3:
	case MGN_VHT3SS_MCS4:
	case MGN_VHT3SS_MCS5:
	case MGN_VHT3SS_MCS6:
	case MGN_VHT3SS_MCS7:
	case MGN_VHT3SS_MCS8:
	case MGN_VHT3SS_MCS9:
		rate_section = 8;
		break;

	case MGN_VHT4SS_MCS0:
	case MGN_VHT4SS_MCS1:
	case MGN_VHT4SS_MCS2:
	case MGN_VHT4SS_MCS3:
	case MGN_VHT4SS_MCS4:
	case MGN_VHT4SS_MCS5:
	case MGN_VHT4SS_MCS6:
	case MGN_VHT4SS_MCS7:
	case MGN_VHT4SS_MCS8:
	case MGN_VHT4SS_MCS9:
		rate_section = 9;
		break;

	default:
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, 
			 "Wrong rate 0x%x\n", rate );
		break;
	}

	if (band == BAND_ON_5G  && rate_section == 0)
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, 
			 "Wrong rate 0x%x: No CCK in 5G Band\n", rate);

	// workaround for wrong index combination to obtain tx power limit, 
	// OFDM only exists in BW 20M
	if (rate_section == 1 )
		bandwidth_temp = 0;

	// workaround for wrong index combination to obtain tx power limit, 
	// CCK table will only be given in BW 20M
	if (rate_section == 0)
		bandwidth_temp = 0;

	// workaround for wrong indxe combination to obtain tx power limit, 
	// HT on 80M will reference to HT on 40M
	if ((rate_section == 2 || rate_section == 3) && band == BAND_ON_5G && bandwidth_temp == 2 ) {
		bandwidth_temp = 1;
	}
	
	if (band == BAND_ON_2_4G)
		channel_temp = 
			_rtl8821au_get_chnl_idx_of_txpwr_limit(rtlpriv,
							       BAND_ON_2_4G,
							       channel);
	else if (band == BAND_ON_5G)
		channel_temp = 
			_rtl8821au_get_chnl_idx_of_txpwr_limit(rtlpriv,
							       BAND_ON_5G,
							       channel);
	else if (band == BAND_ON_BOTH) {
		// BAND_ON_BOTH don't care temporarily 
	}
	
	if (band_temp == -1 || regulation == -1 || bandwidth_temp == -1 || 
	     rate_section == -1 || channel_temp == -1 ) {
		//DBG_871X("Wrong index value to access power limit table [band %d][regulation %d][bandwidth %d][rf_path %d][rate_section %d][chnlGroup %d]\n",
		//	  band, regulation, bandwidth, RfPath, rateSection, channelGroup );

		return MAX_POWER_INDEX;
	}

	if (band == BAND_ON_2_4G) {
		s8 limits[10] = {0}; u8 i = 0;
		for (i = 0; i < MAX_REGULATION_NUM; ++i)
			limits[i] = rtlphy->txpwr_limit_2_4g[i][bandwidth_temp]
					[rate_section][channel_temp][rf_path];

		powerLimit = (regulation == TXPWR_LMT_WW) ?
			phy_GetWorldWideLimit(limits) :
			rtlphy->txpwr_limit_2_4g[regulation][bandwidth_temp]
					[rate_section][channel_temp][rf_path];

	} else if (band == BAND_ON_5G) {
		s8 limits[10] = {0}; u8 i = 0;
		for (i = 0; i < MAX_REGULATION_NUM; ++i)
			limits[i] = rtlphy->txpwr_limit_5g[i][bandwidth_temp]
					[rate_section][channel_temp][rf_path];
		
		powerLimit = (regulation == TXPWR_LMT_WW) ? 
			phy_GetWorldWideLimit(limits) :
			rtlphy->txpwr_limit_5g[regulation][bandwidth_temp]
					[rate_section][channel_temp][rf_path];
	} else
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, 
			 "No power limit table of the specified band\n");

	// combine 5G VHT & HT rate
	// 5G 20M and 40M HT and VHT can cross reference
	/*
	if ( Band == BAND_ON_5G && powerLimit == MAX_POWER_INDEX ) {
		if ( bandwidth == 0 || bandwidth == 1 ) { 
			RT_TRACE( COMP_INIT, DBG_LOUD, ( "No power limit table of the specified band %d, bandwidth %d, ratesection %d, rf path %d\n", 
					  band, bandwidth, rateSection, RfPath ) );
			if ( rateSection == 2 )
				powerLimit = pHalData->TxPwrLimit_5G[regulation]
										[bandwidth][4][channelGroup][RfPath];
			else if ( rateSection == 4 )
				powerLimit = pHalData->TxPwrLimit_5G[regulation]
										[bandwidth][2][channelGroup][RfPath];
			else if ( rateSection == 3 )
				powerLimit = pHalData->TxPwrLimit_5G[regulation]
										[bandwidth][5][channelGroup][RfPath];
			else if ( rateSection == 5 )
				powerLimit = pHalData->TxPwrLimit_5G[regulation]
										[bandwidth][3][channelGroup][RfPath];
		}
	}
	*/
	//DBG_871X("TxPwrLmt[Regulation %d][Band %d][BW %d][RFPath %d][Rate 0x%x][Chnl %d] = %d\n", 
	//		regulation, pHalData->CurrentBandType, Bandwidth, RfPath, DataRate, Channel, powerLimit);
	return powerLimit;
}

static bool _rtl8821au_phy_get_chnl_index(uint8_t Channel, uint8_t *ChannelIdx)
{
	uint8_t channel5G[CHANNEL_MAX_NUMBER_5G] = {
		36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
		100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120,
		122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142,
		144, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167,
		168, 169, 171, 173, 175, 177
		};
	uint8_t	i = 0;
	bool bIn24G = true;

	if (Channel <= 14) {
		bIn24G = true;
		*ChannelIdx = Channel - 1;
	} else {
		bIn24G = false;

		for (i = 0; i < sizeof(channel5G)/sizeof(uint8_t); ++i) {
			if (channel5G[i] == Channel) {
				*ChannelIdx = i;
				return bIn24G;
			}
		}
	}
	return bIn24G;

}

/*
 * For VHT series, we will use a new TX pwr by rate array to meet new spec.
 */
 
u8 _rtl8821au_get_txpower_index_base(struct rtl_priv *rtlpriv,
				     u8 RFPath, u8 Rate,
				     enum CHANNEL_WIDTH BandWidth,
				     u8 Channel,
				     bool *bIn24G)
{
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);

	u8					i = 0;	//default set to 1S
	u8					txPower = 0;
	u8					chnlIdx = (Channel-1);

	*bIn24G = _rtl8821au_phy_get_chnl_index(Channel, &chnlIdx);

	//DBG_871X("[%s] Channel Index: %d\n", (*bIn24G?"2.4G":"5G"), chnlIdx);

	if (*bIn24G) { //3 ============================== 2.4 G ==============================
		if (IS_CCK_RATE(Rate)) {
			txPower = rtlefuse->txpwrlevel_cck[RFPath][chnlIdx];
		} else if (MGN_6M <= Rate) {
			txPower = rtlefuse->txpwrlevel_ht40_1s[RFPath][chnlIdx];
		} else {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_WARNING,
				 "INVALID Rate.\n");
		}

		//DBG_871X("Base Tx power(RF-%c, Rate #%d, Channel Index %d) = 0x%X\n", 
		//		((RFPath==0)?'A':'B'), Rate, chnlIdx, txPower);
		
		// OFDM-1T
		if ((MGN_6M <= Rate && Rate <= MGN_54M) && ! IS_CCK_RATE(Rate)) {
			txPower += rtlefuse->txpwr_legacyhtdiff[RFPath][TX_1S];
			//DBG_871X("+PowerDiff 2.4G (RF-%c): (OFDM-1T) = (%d)\n", ((RFPath==0)?'A':'B'), pHalData->OFDM_24G_Diff[RFPath][TX_1S]);
		}

		if (BandWidth == CHANNEL_WIDTH_20) {		// BW20-1S, BW20-2S
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht20diff[RFPath][TX_1S];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht20diff[RFPath][TX_2S];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht20diff[RFPath][TX_3S];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht20diff[RFPath][TX_4S];

			//DBG_871X("+PowerDiff 2.4G (RF-%c): (BW20-1S, BW20-2S, BW20-3S, BW20-4S) = (%d, %d, %d, %d)\n", ((RFPath==0)?'A':(RFPath==1)?'B':(RFPath==2)?'C':'D'), 
			//	pHalData->BW20_24G_Diff[RFPath][TX_1S], pHalData->BW20_24G_Diff[RFPath][TX_2S], 
			//	pHalData->BW20_24G_Diff[RFPath][TX_3S], pHalData->BW20_24G_Diff[RFPath][TX_4S]);
		} else if (BandWidth == CHANNEL_WIDTH_40) {	// BW40-1S, BW40-2S
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_1S];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_2S];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_3S];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_4S];

			//DBG_871X("+PowerDiff 2.4G (RF-%c): (BW40-1S, BW40-2S, BW40-3S, BW40-4S) = (%d, %d, %d, %d)\n", ((RFPath==0)?'A':(RFPath==1)?'B':(RFPath==2)?'C':'D'), 
			//	pHalData->BW40_24G_Diff[RFPath][TX_1S], pHalData->BW40_24G_Diff[RFPath][TX_2S],
			//	pHalData->BW40_24G_Diff[RFPath][TX_3S], pHalData->BW40_24G_Diff[RFPath][TX_4S]);
		} else if (BandWidth == CHANNEL_WIDTH_80) {	// Willis suggest adopt BW 40M power index while in BW 80 mode
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_1S];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_2S];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_3S];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_ht40diff[RFPath][TX_4S];

			//DBG_871X("+PowerDiff 2.4G (RF-%c): (BW40-1S, BW40-2S, BW40-3S, BW40-4T) = (%d, %d, %d, %d) P.S. Current is in BW 80MHz\n", ((RFPath==0)?'A':(RFPath==1)?'B':(RFPath==2)?'C':'D'), 
			//	pHalData->BW40_24G_Diff[RFPath][TX_1S], pHalData->BW40_24G_Diff[RFPath][TX_2S],
			//	pHalData->BW40_24G_Diff[RFPath][TX_3S], pHalData->BW40_24G_Diff[RFPath][TX_4S]);
		}
	} else { //3 ============================== 5 G ==============================
		if (MGN_6M <= Rate) {				
			txPower = rtlefuse->txpwr_5g_bw40base[RFPath][chnlIdx];
		} else {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_WARNING,
				 "INVALID Rate.\n");
		}

		//DBG_871X("Base Tx power(RF-%c, Rate #%d, Channel Index %d) = 0x%X\n", 
		//	((RFPath==0)?'A':'B'), Rate, chnlIdx, txPower);

		// OFDM-1T
		if ((MGN_6M <= Rate && Rate <= MGN_54M) && ! IS_CCK_RATE(Rate)) {
			txPower += rtlefuse->txpwr_5g_ofdmdiff[RFPath][TX_1S];
			//DBG_871X("+PowerDiff 5G (RF-%c): (OFDM-1T) = (%d)\n", ((RFPath==0)?'A':'B'), pHalData->OFDM_5G_Diff[RFPath][TX_1S]);
		}
		
		if (BandWidth == CHANNEL_WIDTH_20) {		// BW20-1S, BW20-2S
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31)  ||
			    (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw20diff[RFPath][TX_1S];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw20diff[RFPath][TX_2S];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw20diff[RFPath][TX_3S];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw20diff[RFPath][TX_4S];

			//DBG_871X("+PowerDiff 5G (RF-%c): (BW20-1S, BW20-2S, BW20-3S, BW20-4S) = (%d, %d, %d, %d)\n", ((RFPath==0)?'A':(RFPath==1)?'B':(RFPath==2)?'C':'D'), 
			//	pHalData->BW20_5G_Diff[RFPath][TX_1S], pHalData->BW20_5G_Diff[RFPath][TX_2S],
			//	pHalData->BW20_5G_Diff[RFPath][TX_3S], pHalData->BW20_5G_Diff[RFPath][TX_4S]);
		} else if (BandWidth == CHANNEL_WIDTH_40) {	// BW40-1S, BW40-2S
			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31)  ||
			    (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw40diff[RFPath][TX_1S];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw40diff[RFPath][TX_2S];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw40diff[RFPath][TX_3S];
			if ((MGN_MCS24 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw40diff[RFPath][TX_4S];

			//DBG_871X("+PowerDiff 5G(RF-%c): (BW40-1S, BW40-2S) = (%d, %d, %d, %d)\n", ((RFPath==0)?'A':(RFPath==1)?'B':(RFPath==2)?'C':'D'), 
			//	pHalData->BW40_5G_Diff[RFPath][TX_1S], pHalData->BW40_5G_Diff[RFPath][TX_2S],
			//	pHalData->BW40_5G_Diff[RFPath][TX_3S], pHalData->BW40_5G_Diff[RFPath][TX_4S]);
		} else if (BandWidth== CHANNEL_WIDTH_80) {	// BW80-1S, BW80-2S
			// <20121220, Kordan> Get the index of array "Index5G_BW80_Base".
			u8	channel5G_80M[CHANNEL_MAX_NUMBER_5G_80M] = {42, 58, 106, 122, 138, 155, 171};
			for (i = 0; i < sizeof(channel5G_80M)/sizeof(u8); ++i)
				if ( channel5G_80M[i] == Channel) 
					chnlIdx = i;

			txPower = rtlefuse->txpwr_5g_bw80base[RFPath][chnlIdx];

			if ((MGN_MCS0 <= Rate && Rate <= MGN_MCS31)  ||
			    (MGN_VHT1SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += + rtlefuse->txpwr_5g_bw80diff[RFPath][TX_1S];
			if ((MGN_MCS8 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT2SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw80diff[RFPath][TX_2S];
			if ((MGN_MCS16 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT3SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw80diff[RFPath][TX_3S];
			if ((MGN_MCS23 <= Rate && Rate <= MGN_MCS31) ||
			    (MGN_VHT4SS_MCS0 <= Rate && Rate <= MGN_VHT4SS_MCS9))
				txPower += rtlefuse->txpwr_5g_bw80diff[RFPath][TX_4S];

			//DBG_871X("+PowerDiff 5G(RF-%c): (BW80-1S, BW80-2S, BW80-3S, BW80-4S) = (%d, %d, %d, %d)\n",((RFPath==0)?'A':(RFPath==1)?'B':(RFPath==2)?'C':'D'), 
			//	pHalData->BW80_5G_Diff[RFPath][TX_1S], pHalData->BW80_5G_Diff[RFPath][TX_2S],
			//	pHalData->BW80_5G_Diff[RFPath][TX_3S], pHalData->BW80_5G_Diff[RFPath][TX_4S]);
		}
	}

	return txPower;	
}

u8 phy_GetCurrentTxNum_8812A(struct rtl_priv *rtlpriv, u8 Rate)
{
	u8	tx_num = 0;
	
	if ((Rate >= MGN_MCS8 && Rate <= MGN_MCS15) || 
		 (Rate >= MGN_VHT2SS_MCS0 && Rate <= MGN_VHT2SS_MCS9))
		tx_num = RF_2TX;
	else
		tx_num = RF_1TX;
		
	return tx_num;
}

static u8 PHY_GetRateIndexOfTxPowerByRate(u8 Rate)
{
	u8 index = 0;
	switch (Rate) {
	case MGN_1M:
		index = 0;
		break;
	case MGN_2M:
		index = 1;
		break;
	case MGN_5_5M:
		index = 2;
		break;
	case MGN_11M:
		index = 3;
		break;
	case MGN_6M:
		index = 4;
		break;
	case MGN_9M:
		index = 5;
		break;
	case MGN_12M:
		index = 6;
		break;
	case MGN_18M:
		index = 7;
		break;
	case MGN_24M:
		index = 8;
		break;
	case MGN_36M:
		index = 9;
		break;
	case MGN_48M:
		index = 10;
		break;
	case MGN_54M:
		index = 11;
		break;
	case MGN_MCS0:
		index = 12;
		break;
	case MGN_MCS1:
		index = 13;
		break;
	case MGN_MCS2:
		index = 14;
		break;
	case MGN_MCS3:
		index = 15;
		break;
	case MGN_MCS4:
		index = 16;
		break;
	case MGN_MCS5:
		index = 17;
		break;
	case MGN_MCS6:
		index = 18;
		break;
	case MGN_MCS7:
		index = 19;
		break;
	case MGN_MCS8:
		index = 20;
		break;
	case MGN_MCS9:
		index = 21;
		break;
	case MGN_MCS10:
		index = 22;
		break;
	case MGN_MCS11:
		index = 23;
		break;
	case MGN_MCS12:
		index = 24;
		break;
	case MGN_MCS13:
		index = 25;
		break;
	case MGN_MCS14:
		index = 26;
		break;
	case MGN_MCS15:
		index = 27;
		break;
	case MGN_MCS16:
		index = 28;
		break;
	case MGN_MCS17:
		index = 29;
		break;
	case MGN_MCS18:
		index = 30;
		break;
	case MGN_MCS19:
		index = 31;
		break;
	case MGN_MCS20:
		index = 32;
		break;
	case MGN_MCS21:
		index = 33;
		break;
	case MGN_MCS22:
		index = 34;
		break;
	case MGN_MCS23:
		index = 35;
		break;
	case MGN_MCS24:
		index = 36;
		break;
	case MGN_MCS25:
		index = 37;
		break;
	case MGN_MCS26:
		index = 38;
		break;
	case MGN_MCS27:
		index = 39;
		break;
	case MGN_MCS28:
		index = 40;
		break;
	case MGN_MCS29:
		index = 41;
		break;
	case MGN_MCS30:
		index = 42;
		break;
	case MGN_MCS31:
		index = 43;
		break;
	case MGN_VHT1SS_MCS0:
		index = 44;
		break;
	case MGN_VHT1SS_MCS1:
		index = 45;
		break;
	case MGN_VHT1SS_MCS2:
		index = 46;
		break;
	case MGN_VHT1SS_MCS3:
		index = 47;
		break;
	case MGN_VHT1SS_MCS4:
		index = 48;
		break;
	case MGN_VHT1SS_MCS5:
		index = 49;
		break;
	case MGN_VHT1SS_MCS6:
		index = 50;
		break;
	case MGN_VHT1SS_MCS7:
		index = 51;
		break;
	case MGN_VHT1SS_MCS8:
		index = 52;
		break;
	case MGN_VHT1SS_MCS9:
		index = 53;
		break;
	case MGN_VHT2SS_MCS0:
		index = 54;
		break;
	case MGN_VHT2SS_MCS1:
		index = 55;
		break;
	case MGN_VHT2SS_MCS2:
		index = 56;
		break;
	case MGN_VHT2SS_MCS3:
		index = 57;
		break;
	case MGN_VHT2SS_MCS4:
		index = 58;
		break;
	case MGN_VHT2SS_MCS5:
		index = 59;
		break;
	case MGN_VHT2SS_MCS6:
		index = 60;
		break;
	case MGN_VHT2SS_MCS7:
		index = 61;
		break;
	case MGN_VHT2SS_MCS8:
		index = 62;
		break;
	case MGN_VHT2SS_MCS9:
		index = 63;
		break;
	case MGN_VHT3SS_MCS0:
		index = 64;
		break;
	case MGN_VHT3SS_MCS1:
		index = 65;
		break;
	case MGN_VHT3SS_MCS2:
		index = 66;
		break;
	case MGN_VHT3SS_MCS3:
		index = 67;
		break;
	case MGN_VHT3SS_MCS4:
		index = 68;
		break;
	case MGN_VHT3SS_MCS5:
		index = 69;
		break;
	case MGN_VHT3SS_MCS6:
		index = 70;
		break;
	case MGN_VHT3SS_MCS7:
		index = 71;
		break;
	case MGN_VHT3SS_MCS8:
		index = 72;
		break;
	case MGN_VHT3SS_MCS9:
		index = 73;
		break;
	case MGN_VHT4SS_MCS0:
		index = 74;
		break;
	case MGN_VHT4SS_MCS1:
		index = 75;
		break;
	case MGN_VHT4SS_MCS2:
		index = 76;
		break;
	case MGN_VHT4SS_MCS3:
		index = 77;
		break;
	case MGN_VHT4SS_MCS4:
		index = 78;
		break;
	case MGN_VHT4SS_MCS5:
		index = 79;
		break;
	case MGN_VHT4SS_MCS6:
		index = 80;
		break;
	case MGN_VHT4SS_MCS7:
		index = 81;
		break;
	case MGN_VHT4SS_MCS8:
		index = 82;
		break;
	case MGN_VHT4SS_MCS9:
		index = 83;
		break;

	default:
#if 0	
		DBG_871X("Invalid rate 0x%x in %s\n", Rate, __FUNCTION__ );
#endif		
		break;
	};

	return index;
}

static s8 _rtl8821au_phy_get_txpower_tracking_offset(struct rtl_priv *rtlpriv,
						     u8 rfpath, u8 rate)
{
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);
	s8 offset = 0;
	
	if( rtldm->txpower_track_control  == false)
		return offset;
	
	if ((rate == MGN_1M) || (rate == MGN_2M) ||
	    (rate == MGN_5_5M)||(rate == MGN_11M)) {
		offset = rtldm->remnant_cck_idx;
		//DBG_871X("+Remnant_CCKSwingIdx = 0x%x\n", RFPath, Rate, pDM_Odm->Remnant_CCKSwingIdx);
	} else {
		offset = rtldm->remnant_ofdm_swing_idx[rfpath];
		//DBG_871X("+Remanant_OFDMSwingIdx[RFPath %u][Rate 0x%x] = 0x%x\n", RFPath, Rate, pDM_Odm->Remnant_OFDMSwingIdx[RFPath]);		
		
	}

	return offset;
}

static u8 _rtl8821au_get_txpower_index(struct rtl_priv *rtlpriv, u8 RFPath,
			       u8 Rate, enum CHANNEL_WIDTH BandWidth,
			       u8 Channel)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	s8 txpower = 0, limit = 0;
	bool in_24g = false;
	s8 powerdiff_byrate = 0;

	u8 tx_num = phy_GetCurrentTxNum_8812A(rtlpriv, Rate);

	//DBG_871X("===> PHY_GetTxPowerIndex_8812A\n");

	txpower = (s8) _rtl8821au_get_txpower_index_base(rtlpriv, RFPath, Rate, 
							  BandWidth, Channel, &in_24g);

	powerdiff_byrate = _rtl8821au_phy_get_txpower_by_rate(rtlpriv, (u8)(!in_24g), RFPath, tx_num, Rate );

	limit = _rtl8821au_phy_get_txpower_limit(rtlpriv, (u8)(!in_24g),
						 rtlpriv->phy.current_chan_bw,
						 RFPath, Rate,
						 rtlpriv->phy.current_channel);

	powerdiff_byrate = (powerdiff_byrate > limit) ? limit : powerdiff_byrate;
	//DBG_871X("Rate-0x%x: (TxPower, PowerDiffByRate Path-%c) = (0x%X, %d)\n", Rate, ((RFPath==0)?'A':'B'), txPower, powerDiffByRate);

	// We need to reduce power index for VHT MCS 8 & 9.
	if (Rate == MGN_VHT1SS_MCS8 || Rate == MGN_VHT1SS_MCS9 ||
		Rate == MGN_VHT2SS_MCS8 || Rate == MGN_VHT2SS_MCS9) {
		txpower += powerdiff_byrate;
	} else {
#ifdef CONFIG_USB_HCI
		//
		// 2013/01/29 MH For preventing VHT rate of 8812AU to be used in USB 2.0 mode
		// and the current will be more than 500mA and card disappear. We need to limit 
		// TX power with any power by rate for VHT in U2.
		// 2013/01/30 MH According to power current test compare with BCM AC NIC, we
		// decide to use host hub = 2.0 mode to enable tx power limit behavior.
		//
		if (adapter_to_dvobj(pAdapter)->usb_speed == RTW_USB_SPEED_2 && IS_HARDWARE_TYPE_8812AU(pAdapter))
		{
			powerdiff_byrate = 0;
		}
#endif
		txpower += powerdiff_byrate;
#if 0
		//
		// 2013/02/06 MH Add for ASUS requiremen for adjusting TX power limit.
		// This is a temporarily dirty fix for asus , neeed to revise later!
		// 2013/03/07 MH Asus add more request.
		// 2013/03/14 MH Asus add one more request for the power control.
		//
		if (Channel >= 36) {			
			txPower += pMgntInfo->RegTPCLvl5g;

			if (txPower > pMgntInfo->RegTPCLvl5gD)
				txPower -= pMgntInfo->RegTPCLvl5gD;
		} else {		
			txPower += pMgntInfo->RegTPCLvl;

			if (txPower > pMgntInfo->RegTPCLvlD)
				txPower -= pMgntInfo->RegTPCLvlD;
		}
#endif
	}	
	
	txpower += _rtl8821au_phy_get_txpower_tracking_offset(rtlpriv, RFPath, Rate );
	
	// 2012/09/26 MH We need to take care high power device limiation to prevent destroy EXT_PA.
	// This case had ever happened in CU/SU high power module. THe limitation = 0x20.
	// But for 8812, we still not know the value.
#if 0	
	phy_TxPwrAdjInPercentage(pAdapter, (u8 *)&txPower);
#endif	

	if(txpower > MAX_POWER_INDEX)
		txpower = MAX_POWER_INDEX;

	if ( txpower % 2 == 1 && !IS_NORMAL_CHIP(rtlhal->version))
		--txpower;

	//DBG_871X("Final Tx Power(RF-%c, Channel: %d) = %d(0x%X)\n", ((RFPath==0)?'A':'B'), Channel,txPower, txPower);

	return (u8) txpower;
}

static void PHY_SetPowerLimitTableValue(struct rtl_priv *rtlpriv,
	s8 *Regulation, s8 *Band, s8 *Bandwidth, s8 *RateSection,
	s8 *RfPath, s8 *Channel, s8 *PowerLimit)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	uint8_t		regulation = 0, bandwidth = 0, rateSection = 0,
			channel, powerLimit, channelGroup;

	RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Index of power limit table \
		  [band %s][regulation %s][bw %s][rate section %s][rf path %s][chnl %s][val %s]\n",
		  Band, Regulation, Bandwidth, RateSection, RfPath, Channel, PowerLimit);

	if (!Getu8IntegerFromStringInDecimal(Channel, &channel) ||
		 !Getu8IntegerFromStringInDecimal(PowerLimit, &powerLimit)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Illegal index of power limit table [chnl %s][val %s]\n", Channel, PowerLimit);
	}

	powerLimit = powerLimit > MAX_POWER_INDEX ? MAX_POWER_INDEX : powerLimit;

	if (eqNByte(Regulation, "FCC", 3))
		regulation = 0;
	else if (eqNByte(Regulation, "MKK", 3))
		regulation = 1;
	else if (eqNByte(Regulation, "ETSI", 4))
		regulation = 2;

	if (eqNByte(RateSection, "CCK", 3))
		rateSection = 0;
	else if (eqNByte(RateSection, "OFDM", 4))
		rateSection = 1;
	else if (eqNByte(RateSection, "HT", 2) && eqNByte(RfPath, "1T", 2))
		rateSection = 2;
	else if (eqNByte(RateSection, "HT", 2) && eqNByte(RfPath, "2T", 2))
		rateSection = 3;
	else if (eqNByte(RateSection, "VHT", 3) && eqNByte(RfPath, "1T", 2))
		rateSection = 4;
	else if (eqNByte(RateSection, "VHT", 3) && eqNByte(RfPath, "2T", 2))
		rateSection = 5;


	if (eqNByte(Bandwidth, "20M", 3))
		bandwidth = 0;
	else if (eqNByte(Bandwidth, "40M", 3))
		bandwidth = 1;
	else if (eqNByte(Bandwidth, "80M", 3))
		bandwidth = 2;
	else if (eqNByte(Bandwidth, "160M", 4))
		bandwidth = 3;

	if (eqNByte(Band, "2.4G", 4)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "2.4G Band value : [regulation %d][bw %d][rate_section %d][chnl %d][val %d]\n",
			regulation, bandwidth, rateSection, channel, powerLimit);

		channelGroup = phy_GetChannelGroup(rtlpriv, BAND_ON_2_4G, channel);
		rtlphy->txpwr_limit_2_4g[regulation][bandwidth][rateSection][channelGroup][RF90_PATH_A] = powerLimit;
	} else if (eqNByte(Band, "5G", 2)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "5G Band value : [regulation %d][bw %d][rate_section %d][chnl %d][val %d]\n",
			  regulation, bandwidth, rateSection, channel, powerLimit);

		channelGroup = phy_GetChannelGroup(rtlpriv, BAND_ON_5G, channel);
		rtlphy->txpwr_limit_5g[regulation][bandwidth][rateSection][channelGroup][RF90_PATH_A] = powerLimit;
	} else {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Cannot recognize the band info in %s\n", Band);
		return;
	}
}

static void odm_ConfigBB_TXPWR_LMT_8821A(struct rtl_priv *rtlpriv,
	u8 *Regulation, u8 *Band, u8 *Bandwidth,
	u8 * RateSection, u8 *RfPath, u8 *Channel,
	u8 *PowerLimit
    )
{
	PHY_SetPowerLimitTableValue(rtlpriv, Regulation, Band,
		Bandwidth, RateSection, RfPath, Channel, PowerLimit);
}

static void _rtl8821au_phy_set_txpower_limit(struct rtl_priv *rtlpriv,
	s8 *Regulation, s8 *Band, s8 *Bandwidth, s8 *RateSection,
	s8 *RfPath, s8 *Channel, s8 *PowerLimit)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	uint8_t		regulation = 0, bandwidth = 0, rateSection = 0,
			channel, powerLimit, channelGroup;

	RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Index of power limit table \
		  [band %s][regulation %s][bw %s][rate section %s][rf path %s][chnl %s][val %s]\n",
		  Band, Regulation, Bandwidth, RateSection, RfPath, Channel, PowerLimit);

	if (!Getu8IntegerFromStringInDecimal(Channel, &channel) ||
		 !Getu8IntegerFromStringInDecimal(PowerLimit, &powerLimit)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Illegal index of power limit table [chnl %s][val %s]\n", Channel, PowerLimit);
	}

	powerLimit = powerLimit > MAX_POWER_INDEX ? MAX_POWER_INDEX : powerLimit;

	if (eqNByte(Regulation, "FCC", 3))
		regulation = 0;
	else if (eqNByte(Regulation, "MKK", 3))
		regulation = 1;
	else if (eqNByte(Regulation, "ETSI", 4))
		regulation = 2;

	if (eqNByte(RateSection, "CCK", 3))
		rateSection = 0;
	else if (eqNByte(RateSection, "OFDM", 4))
		rateSection = 1;
	else if (eqNByte(RateSection, "HT", 2) && eqNByte(RfPath, "1T", 2))
		rateSection = 2;
	else if (eqNByte(RateSection, "HT", 2) && eqNByte(RfPath, "2T", 2))
		rateSection = 3;
	else if (eqNByte(RateSection, "VHT", 3) && eqNByte(RfPath, "1T", 2))
		rateSection = 4;
	else if (eqNByte(RateSection, "VHT", 3) && eqNByte(RfPath, "2T", 2))
		rateSection = 5;


	if (eqNByte(Bandwidth, "20M", 3))
		bandwidth = 0;
	else if (eqNByte(Bandwidth, "40M", 3))
		bandwidth = 1;
	else if (eqNByte(Bandwidth, "80M", 3))
		bandwidth = 2;
	else if (eqNByte(Bandwidth, "160M", 4))
		bandwidth = 3;

	if (eqNByte(Band, "2.4G", 4)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "2.4G Band value : [regulation %d][bw %d][rate_section %d][chnl %d][val %d]\n",
			regulation, bandwidth, rateSection, channel, powerLimit);

		channelGroup = phy_GetChannelGroup(rtlpriv, BAND_ON_2_4G, channel);
		rtlphy->txpwr_limit_2_4g[regulation][bandwidth][rateSection][channelGroup][RF90_PATH_A] = powerLimit;
	} else if (eqNByte(Band, "5G", 2)) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "5G Band value : [regulation %d][bw %d][rate_section %d][chnl %d][val %d]\n",
			  regulation, bandwidth, rateSection, channel, powerLimit);

		channelGroup = phy_GetChannelGroup(rtlpriv, BAND_ON_5G, channel);
		rtlphy->txpwr_limit_5g[regulation][bandwidth][rateSection][channelGroup][RF90_PATH_A] = powerLimit;
	} else {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Cannot recognize the band info in %s\n", Band);
		return;
	}
}

static void _rtl8821au_phy_config_bb_txpwr_lmt(struct rtl_priv *rtlpriv,
					       u8 *regulation, u8 *band,
					       u8 *bandwidth, u8 *rate_selection,
					       u8 *rf_path, u8 *channel,
					       u8 *power_limit)
{
	_rtl8821au_phy_set_txpower_limit(rtlpriv, regulation, band, bandwidth,
					 rate_selection, rf_path, channel,
					 power_limit);
}

static void _rtl8821au_phy_read_and_config_txpwr_lmt(struct rtl_priv *rtlpriv)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);
	uint32_t i		= 0;
	uint32_t ArrayLen       = RTL8821AU_TXPWR_LMT_ARRAY_LEN;
	u8 **Array		= RTL8821AU_TXPWR_LMT;

	if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
		ArrayLen = RTL8821AU_TXPWR_LMT_ARRAY_LEN;
		Array = RTL8812AU_TXPWR_LMT;
	} else {
		ArrayLen = RTL8821AU_TXPWR_LMT_ARRAY_LEN;
		Array = RTL8821AU_TXPWR_LMT;

	}

	for (i = 0; i < ArrayLen; i += 7) {
		u8 *regulation = Array[i];
		u8 *band = Array[i+1];
		u8 *bandwidth = Array[i+2];
		u8 *rate = Array[i+3];
		u8 *rf_path = Array[i+4];
		u8 *chnl = Array[i+5];
		u8 *val = Array[i+6];

		_rtl8821au_phy_config_bb_txpwr_lmt(rtlpriv, regulation, band,
						   bandwidth, rate, rf_path,
						   chnl, val);
	}

}

static bool CheckCondition(const uint32_t  Condition, const uint32_t  Hex)
{
	uint32_t _board     = (Hex & 0x000000FF);
	uint32_t _interface = (Hex & 0x0000FF00) >> 8;
	uint32_t _platform  = (Hex & 0x00FF0000) >> 16;
	uint32_t cond = Condition;

	if (Condition == 0xCDCDCDCD)
		return true;

	cond = Condition & 0x000000FF;
	if ((_board != cond) && (cond != 0xFF))
		return false;

	cond = Condition & 0x0000FF00;
	cond = cond >> 8;
	if (((_interface & cond) == 0) && (cond != 0x07))
		return false;

	cond = Condition & 0x00FF0000;
	cond = cond >> 16;
	if (((_platform & cond) == 0) && (cond != 0x0F))
		return false;

	return true;
}


void _rtl8821au_phy_config_mac_with_headerfile(struct rtl_priv *rtlpriv)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

	uint32_t     hex         = 0;
	uint32_t     i           = 0;

	/* ULLI : fixed values ?? */
	u8  platform = ODM_CE;
	u8 _interface = RTW_USB;
	u8 board = rtlhal->board_type;
	uint32_t ArrayLen;
	uint32_t *Array;

	if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
		ArrayLen    = RTL8812AUMAC_1T_ARRAYLEN;
		Array       = RTL8812AU_MAC_REG_ARRAY;
	} else {
		ArrayLen    = RTL8821AUMAC_1T_ARRAYLEN;
		Array       = RTL8821AU_MAC_REG_ARRAY;
	}

	hex += board;
	hex += _interface << 8;
	hex += platform << 16;
	hex += 0xFF000000;

	RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ReadAndConfig_MP_8821A_MAC_REG, hex = 0x%X\n", hex);

	for (i = 0; i < ArrayLen; i += 2) {
		uint32_t v1 = Array[i];
		uint32_t v2 = Array[i+1];

		/* This (offset, data) pair meets the condition. */
		if (v1 < 0xCDCDCDC) {
			rtl_write_byte(rtlpriv, v1, (u8)v2);
			continue;
		} else {
			/* This line is the start line of branch. */
			if (!CheckCondition(Array[i], hex)) {
				/* Discard the following (offset, data) pairs. */
				READ_NEXT_PAIR(Array, v1, v2, i);
				while (v2 != 0xDEAD &&
					v2 != 0xCDEF &&
					v2 != 0xCDCD && i < ArrayLen - 2) {
						READ_NEXT_PAIR(Array, v1, v2, i);
				}
				i -= 2; /* prevent from for-loop += 2 */
			} else {
				/* Configure matched pairs and skip to end of if-else. */
				READ_NEXT_PAIR(Array, v1, v2, i);
				while (v2 != 0xDEAD &&
					v2 != 0xCDEF &&
					v2 != 0xCDCD && i < ArrayLen - 2) {
						rtl_write_byte(rtlpriv, v1, (u8)v2);
						READ_NEXT_PAIR(Array, v1, v2, i);
				}

				while (v2 != 0xDEAD && i < ArrayLen - 2) {
					READ_NEXT_PAIR(Array, v1, v2, i);
				}

			}
		}
	}

}

/* *****  */

/******************************************************************************
*                           RadioA.TXT
******************************************************************************/
static void _rtl8821au_config_rf_reg(struct rtl_priv *rtlpriv, uint32_t Addr,
	uint32_t Data, enum radio_path path, uint32_t RegAddr)
{
	if (Addr == 0xfe || Addr == 0xffe) {
		msleep(50);
	} else {
		rtl_set_rfreg(rtlpriv, path, RegAddr, bRFRegOffsetMask, Data);
		/* Add 1us delay between BB/RF register setting. */
		udelay(1);
	}
}

static void _rtl8821au_config_rf_radio_a(struct rtl_priv *rtlpriv, uint32_t Addr,
	uint32_t Data)
{
	uint32_t content = 0x1000;		/* RF_Content: radioa_txt */
	uint32_t maskforPhySet = (uint32_t)(content&0xE000);

	_rtl8821au_config_rf_reg(rtlpriv, Addr, Data, RF90_PATH_A, Addr|maskforPhySet);

	RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ConfigRFWithHeaderFile: [RadioA] %08X %08X\n", Addr, Data);
}

static void _rtl8821au_config_rf_radio_b(struct rtl_priv *rtlpriv, uint32_t Addr,
	uint32_t Data)
{
	uint32_t  content = 0x1001;		/* RF_Content: radiob_txt */
	uint32_t maskforPhySet = (uint32_t)(content&0xE000);

	_rtl8821au_config_rf_reg(rtlpriv, Addr, Data, RF90_PATH_B, Addr|maskforPhySet);

	RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ConfigRFWithHeaderFile: [RadioB] %08X %08X\n", Addr, Data);
}



bool rtl8812au_phy_config_rf_with_headerfile(struct rtl_priv *rtlpriv,
				enum radio_path eRFPath)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

	int i;
	bool rtstatus = true;
	u32 *radioa_array_table_a, *radioa_array_table_b;
	u16 radioa_arraylen_a, radioa_arraylen_b;
	u32 v1 = 0, v2 = 0;

	uint32_t	hex         = 0;
	u16	count       = 0;
	uint32_t	*ptr_array   = NULL;

	/* ULLI : fixed values ?? */
	u8  platform = ODM_CE;
	u8 _interface = RTW_USB;
	u8 board = rtlhal->board_type;

	radioa_arraylen_a = RTL8812AU_RADIOA_1TARRAYLEN;
	radioa_array_table_a = RTL8812AU_RADIOA_ARRAY;
	radioa_arraylen_b = RTL8812AU_RADIOB_1TARRAYLEN;
	radioa_array_table_b = RTL8812AU_RADIOB_ARRAY;
	/*

	RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
		 "Radio_A:RTL8821AE_RADIOA_ARRAY %d\n", radioa_arraylen_a);
	RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD, "Radio No %x\n", rfpath);
	*/
	rtstatus = true;
	switch (eRFPath) {
	case RF90_PATH_A:
		hex += board;
		hex += _interface << 8;
		hex += platform << 16;
		hex += 0xFF000000;
		RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ReadAndConfig_MP_8821A_RadioA, hex = 0x%X\n", hex);

		for (i = 0; i < radioa_arraylen_a; i += 2) {
			uint32_t v1 = radioa_array_table_a[i];
			uint32_t v2 = radioa_array_table_a[i+1];

			/* This (offset, data) pair meets the condition. */
			if (v1 < 0xCDCDCDCD) {
				_rtl8821au_config_rf_radio_a(rtlpriv, v1, v2);
				continue;
			} else {
				/* This line is the start line of branch. */
				if (!CheckCondition(radioa_array_table_a[i], hex)) {
					/* Discard the following (offset, data) pairs. */
					READ_NEXT_PAIR(radioa_array_table_a, v1, v2, i);
					while (v2 != 0xDEAD &&
					    v2 != 0xCDEF &&
					    v2 != 0xCDCD && i < radioa_arraylen_a-2) {
						READ_NEXT_PAIR(radioa_array_table_a, v1, v2, i);
					}
					i -= 2; /* prevent from for-loop += 2 */
				} else {
					/* Configure matched pairs and skip to end of if-else. */
					READ_NEXT_PAIR(radioa_array_table_a, v1, v2, i);
					while (v2 != 0xDEAD &&
					    v2 != 0xCDEF &&
					    v2 != 0xCDCD && i < radioa_arraylen_a-2) {
						_rtl8821au_config_rf_radio_a(rtlpriv, v1, v2);
						READ_NEXT_PAIR(radioa_array_table_a, v1, v2, i);
					}

					while (v2 != 0xDEAD && i < radioa_arraylen_a-2) {
						READ_NEXT_PAIR(radioa_array_table_a, v1, v2, i);
					}
				}
			}
		}
		break;
	case RF90_PATH_B:
		hex += board;
		hex += _interface << 8;
		hex += platform << 16;
		hex += 0xFF000000;
		RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ReadAndConfig_MP_8812A_RadioB, hex = 0x%X\n", hex);

		for (i = 0; i < radioa_arraylen_b; i += 2) {
			uint32_t v1 = radioa_array_table_b[i];
			uint32_t v2 = radioa_array_table_b[i+1];

			/* This (offset, data) pair meets the condition. */
			if (v1 < 0xCDCDCDCD) {
				_rtl8821au_config_rf_radio_b(rtlpriv, v1, v2);
				continue;
			} else {
				/* This line is the start line of branch. */
				if (!CheckCondition(radioa_array_table_b[i], hex)) {
					/* Discard the following (offset, data) pairs. */
					READ_NEXT_PAIR(radioa_array_table_b, v1, v2, i);
					while (v2 != 0xDEAD &&
					    v2 != 0xCDEF &&
					    v2 != 0xCDCD && i < radioa_arraylen_b-2) {
						READ_NEXT_PAIR(radioa_array_table_b, v1, v2, i);
					}
					i -= 2; /* prevent from for-loop += 2 */
				} else {
					/* Configure matched pairs and skip to end of if-else. */
					READ_NEXT_PAIR(radioa_array_table_b, v1, v2, i);
					while (v2 != 0xDEAD &&
					    v2 != 0xCDEF &&
					    v2 != 0xCDCD && i < radioa_arraylen_b-2) {
						_rtl8821au_config_rf_radio_b(rtlpriv, v1, v2);
						READ_NEXT_PAIR(radioa_array_table_b, v1, v2, i);
					}

					while (v2 != 0xDEAD && i < radioa_arraylen_b-2) {
						READ_NEXT_PAIR(radioa_array_table_b, v1, v2, i);
					}
				}
			}
		}

	case RF90_PATH_C:
		RT_TRACE(rtlpriv, COMP_ERR, DBG_EMERG,
			 "switch case not process\n");
		break;
	case RF90_PATH_D:
		RT_TRACE(rtlpriv, COMP_ERR, DBG_EMERG, 
			 "switch case not process\n");
		break;
	}

	return rtstatus;
}


/******************************************************************************
*                           RadioA.TXT
******************************************************************************/

bool rtl8821au_phy_config_rf_with_headerfile(struct rtl_priv *rtlpriv, enum radio_path eRFPath)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

	uint32_t	hex         = 0;
	uint32_t	i           = 0;
	bool rtstatus = true;
	u16	count       = 0;
	uint32_t	*ptr_array   = NULL;

	/* ULLI : fixed values ?? */
	u8  platform = ODM_CE;
	u8 _interface = RTW_USB;
	u8 board = rtlhal->board_type;

	uint32_t	ArrayLen    =  RTL8821AU_RADIOA_1TARRAYLEN;
	uint32_t	*Array       = RTL8821AU_RADIOA_ARRAY;


	hex += board;
	hex += _interface << 8;
	hex += platform << 16;
	hex += 0xFF000000;
	RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ReadAndConfig_MP_8821A_RadioA, hex = 0x%X\n", hex);

	for (i = 0; i < ArrayLen; i += 2) {
		uint32_t v1 = Array[i];
		uint32_t v2 = Array[i+1];

		/* This (offset, data) pair meets the condition. */
		if (v1 < 0xCDCDCDCD) {
			_rtl8821au_config_rf_radio_a(rtlpriv, v1, v2);
			continue;
		} else {
			/* This line is the start line of branch. */
			if (!CheckCondition(Array[i], hex)) {
				/* Discard the following (offset, data) pairs. */
				READ_NEXT_PAIR(Array, v1, v2, i);
				while (v2 != 0xDEAD &&
				    v2 != 0xCDEF &&
				    v2 != 0xCDCD && i < ArrayLen-2) {
					READ_NEXT_PAIR(Array, v1, v2, i);
				}
				i -= 2; /* prevent from for-loop += 2 */
			} else {
				/* Configure matched pairs and skip to end of if-else. */
				READ_NEXT_PAIR(Array, v1, v2, i);
				while (v2 != 0xDEAD &&
				    v2 != 0xCDEF &&
				    v2 != 0xCDCD && i < ArrayLen-2) {
					_rtl8821au_config_rf_radio_a(rtlpriv, v1, v2);
					READ_NEXT_PAIR(Array, v1, v2, i);
				}

				while (v2 != 0xDEAD && i < ArrayLen-2) {
					READ_NEXT_PAIR(Array, v1, v2, i);
				}
			}
		}
	}

	return rtstatus;
}

static void _rtl8821au_phy_set_reg_bw(struct rtl_priv *rtlpriv, enum CHANNEL_WIDTH bw)
{
	u16 reg_rf_mode_bw, tmp = 0;

	reg_rf_mode_bw = rtl_read_word(rtlpriv, REG_WMAC_TRXPTCL_CTL);
	switch (bw) {
	case CHANNEL_WIDTH_20:
		rtl_write_word(rtlpriv, REG_WMAC_TRXPTCL_CTL, reg_rf_mode_bw & 0xFE7F);
		break;
	case CHANNEL_WIDTH_40:
		tmp = reg_rf_mode_bw | BIT(7);
		rtl_write_word(rtlpriv, REG_WMAC_TRXPTCL_CTL, tmp & 0xFEFF);
		break;
	case CHANNEL_WIDTH_80:
		tmp = reg_rf_mode_bw | BIT(8);
		rtl_write_word(rtlpriv, REG_WMAC_TRXPTCL_CTL, tmp & 0xFF7F);
		break;
	default:
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "phy_PostSetBWMode8812():	unknown Bandwidth: %#X\n", bw);
		break;
	}
}

void rtl8812au_fixspur(struct rtl_priv *rtlpriv, enum CHANNEL_WIDTH Bandwidth,
	u8 Channel)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	/* C cut Item12 ADC FIFO CLOCK */
	if(IS_VENDOR_8812A_C_CUT(rtlhal->version)) {
		if(Bandwidth == CHANNEL_WIDTH_40 && Channel == 11)
			rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0xC00, 0x3);		/* 0x8AC[11:10] = 2'b11 */
		else
			rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0xC00, 0x2);		/* 0x8AC[11:10] = 2'b10 */

		/*
		 *  <20120914, Kordan> A workarould to resolve 2480Mhz spur by setting ADC clock as 160M. (Asked by Binson)
		 */
		if (Bandwidth == CHANNEL_WIDTH_20 && (Channel == 13 || Channel == 14)) {
			rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x300, 0x3);  		/* 0x8AC[9:8] = 2'b11 */
			rtl_set_bbreg(rtlpriv, rADC_Buf_Clk_Jaguar, BIT(30), 1);  	/* 0x8C4[30] = 1 */
		} else if (Bandwidth == CHANNEL_WIDTH_40 && Channel == 11) {
			rtl_set_bbreg(rtlpriv, rADC_Buf_Clk_Jaguar, BIT(30), 1);  	/* 0x8C4[30] = 1 */
		} else if (Bandwidth != CHANNEL_WIDTH_80) {
			rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x300, 0x2);  		/* 0x8AC[9:8] = 2'b10 */
			rtl_set_bbreg(rtlpriv, rADC_Buf_Clk_Jaguar, BIT(30), 0);  	/* 0x8C4[30] = 0 */

		}
	} else if (IS_HARDWARE_TYPE_8812(rtlhal)) {
		/* <20120914, Kordan> A workarould to resolve 2480Mhz spur by setting ADC clock as 160M. (Asked by Binson) */
		if (Bandwidth == CHANNEL_WIDTH_20 && (Channel == 13 || Channel == 14))
			rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x300, 0x3);  /* 0x8AC[9:8] = 11 */
		else if (Channel <= 14) /* 2.4G only */
			rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x300, 0x2);  /* 0x8AC[9:8] = 10 */
	}

}
static u8 _rtl8821au_phy_get_secondary_chnl(struct rtl_priv *rtlpriv)
{
	struct rtl_mac *mac = &(rtlpriv->mac80211);
	uint8_t	SCSettingOf40 = 0, SCSettingOf20 = 0;

	/*
	 * DBG_871X("SCMapping: VHT Case: pHalData->CurrentChannelBW %d, pHalData->nCur80MhzPrimeSC %d, pHalData->nCur40MhzPrimeSC %d \n",pHalData->CurrentChannelBW,pHalData->nCur80MhzPrimeSC,pHalData->nCur40MhzPrimeSC);
	 */
	if(rtlpriv->phy.current_chan_bw== CHANNEL_WIDTH_80) {
		if(mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER)
			SCSettingOf40 = VHT_DATA_SC_40_LOWER_OF_80MHZ;
		else if(mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER)
			SCSettingOf40 = VHT_DATA_SC_40_UPPER_OF_80MHZ;
		else
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "SCMapping: Not Correct Primary40MHz Setting \n");

		if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER))
			SCSettingOf20 = VHT_DATA_SC_20_LOWEST_OF_80MHZ;
		else if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER))
			SCSettingOf20 = VHT_DATA_SC_20_LOWER_OF_80MHZ;
		else if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER))
			SCSettingOf20 = VHT_DATA_SC_20_UPPER_OF_80MHZ;
		else if((mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER) && (mac->cur_80_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER))
			SCSettingOf20 = VHT_DATA_SC_20_UPPERST_OF_80MHZ;
		else
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "SCMapping: Not Correct Primary40MHz Setting \n");
	} else if(rtlpriv->phy.current_chan_bw == CHANNEL_WIDTH_40) {
		/*
		 * DBG_871X("SCMapping: VHT Case: pHalData->CurrentChannelBW %d, pHalData->nCur40MhzPrimeSC %d \n",pHalData->CurrentChannelBW,pHalData->nCur40MhzPrimeSC);
		 */

		if(mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_UPPER)
			SCSettingOf20 = VHT_DATA_SC_20_UPPER_OF_80MHZ;
		else if(mac->cur_40_prime_sc == HAL_PRIME_CHNL_OFFSET_LOWER)
			SCSettingOf20 = VHT_DATA_SC_20_LOWER_OF_80MHZ;
		else
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "SCMapping: Not Correct Primary40MHz Setting \n");
	}

	/*
	 * DBG_871X("SCMapping: SC Value %x \n", ( (SCSettingOf40 << 4) | SCSettingOf20));
	 */
	return  ( (SCSettingOf40 << 4) | SCSettingOf20);
}


void rtl8821au_phy_set_bw_mode_callback(struct rtl_priv *rtlpriv)
{
	uint8_t			SubChnlNum = 0;
	uint8_t			L1pkVal = 0;

	/* 3 Set Reg668 Reg440 BW */
	_rtl8821au_phy_set_reg_bw(rtlpriv, rtlpriv->phy.current_chan_bw);

	/* 3 Set Reg483 */
	SubChnlNum = _rtl8821au_phy_get_secondary_chnl(rtlpriv);
	rtl_write_byte(rtlpriv, REG_DATA_SC_8812, SubChnlNum);

	/* DBG_871X("[BW:CHNL], phy_PostSetBwMode8812(), set BW=%s !!\n", GLBwSrc[pHalData->CurrentChannelBW]); */

	/* 3 Set Reg848 Reg864 Reg8AC Reg8C4 RegA00 */
	switch (rtlpriv->phy.current_chan_bw) {
	case CHANNEL_WIDTH_20:
		rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x003003C3, 0x00300200); /* 0x8ac[21,20,9:6,1,0]=8'b11100000 */
		rtl_set_bbreg(rtlpriv, rADC_Buf_Clk_Jaguar, BIT(30), 0);			// 0x8c4[30] = 1'b0

		rtl_set_bbreg(rtlpriv, rFPGA0_XB_RFInterfaceOE, 0x001C0000, 4);	/* 0x864[20:18] = 3'b4 */

		if(rtlpriv->phy.rf_type == RF_2T2R)
			rtl_set_bbreg(rtlpriv, rL1PeakTH_Jaguar, 0x03C00000, 7);	/* 2R 0x848[25:22] = 0x7 */
		else
			rtl_set_bbreg(rtlpriv, rL1PeakTH_Jaguar, 0x03C00000, 8);	/* 1R 0x848[25:22] = 0x8 */

		break;

	case CHANNEL_WIDTH_40:
		rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x003003C3, 0x00300201);	/* 0x8ac[21,20,9:6,1,0]=8'b11100000 */
		rtl_set_bbreg(rtlpriv, rADC_Buf_Clk_Jaguar, BIT(30), 0);		/* 0x8c4[30] = 1'b0 */
		rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x3C, SubChnlNum);
		rtl_set_bbreg(rtlpriv, rCCAonSec_Jaguar, 0xf0000000, SubChnlNum);

		rtl_set_bbreg(rtlpriv, rFPGA0_XB_RFInterfaceOE, 0x001C0000, 2);	/* 0x864[20:18] = 3'b2 */

		if(rtlpriv->phy.reg_837 & BIT(2))
			L1pkVal = 6;
		else {
			if(rtlpriv->phy.rf_type == RF_2T2R)
				L1pkVal = 7;
			else
				L1pkVal = 8;
		}

		rtl_set_bbreg(rtlpriv, rL1PeakTH_Jaguar, 0x03C00000, L1pkVal);	/* 0x848[25:22] = 0x6 */

		if(SubChnlNum == VHT_DATA_SC_20_UPPER_OF_80MHZ)
			rtl_set_bbreg(rtlpriv, rCCK_System_Jaguar, bCCK_System_Jaguar, 1);
		else
			rtl_set_bbreg(rtlpriv, rCCK_System_Jaguar, bCCK_System_Jaguar, 0);
		break;

	case CHANNEL_WIDTH_80:
		rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x003003C3, 0x00300202);	/* 0x8ac[21,20,9:6,1,0]=8'b11100010 */
		rtl_set_bbreg(rtlpriv, rADC_Buf_Clk_Jaguar, BIT(30), 1);		/* 0x8c4[30] = 1 */
		rtl_set_bbreg(rtlpriv, rRFMOD_Jaguar, 0x3C, SubChnlNum);
		rtl_set_bbreg(rtlpriv, rCCAonSec_Jaguar, 0xf0000000, SubChnlNum);

		rtl_set_bbreg(rtlpriv, rFPGA0_XB_RFInterfaceOE, 0x001C0000, 2);	/* 0x864[20:18] = 3'b2 */

		if(rtlpriv->phy.reg_837 & BIT(2))
			L1pkVal = 5;
		else {
			if(rtlpriv->phy.rf_type == RF_2T2R)
				L1pkVal = 6;
			else
				L1pkVal = 7;
		}
		rtl_set_bbreg(rtlpriv, rL1PeakTH_Jaguar, 0x03C00000, L1pkVal);	/* 0x848[25:22] = 0x5 */

		break;

	default:
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "phy_PostSetBWMode8812():	unknown Bandwidth: %#X\n",rtlpriv->phy.current_chan_bw);
		break;
	}

	/* <20121109, Kordan> A workaround for 8812A only. */
	rtl8812au_fixspur(rtlpriv, rtlpriv->phy.current_chan_bw, rtlpriv->phy.current_channel);

	/*
	 * DBG_871X("phy_PostSetBwMode8812(): Reg483: %x\n", rtl_read_byte(rtlpriv, 0x483));
	 * DBG_871X("phy_PostSetBwMode8812(): Reg668: %x\n", rtl_read_dword(rtlpriv, 0x668));
	 * DBG_871X("phy_PostSetBwMode8812(): Reg8AC: %x\n", rtl_get_bbreg(rtlpriv, rRFMOD_Jaguar, 0xffffffff));
	 */

	/* 3 Set RF related register */
	rtl8821au_phy_rf6052_set_bandwidth(rtlpriv, rtlpriv->phy.current_chan_bw);
}

uint32_t phy_get_tx_swing_8821au(struct rtl_priv *rtlpriv, enum band_type Band,
	uint8_t	RFPath)
{
	struct rtl_dm *rtldm = &(rtlpriv->dm);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);
	char reg_swing_2g = -1;	/* 0xff */
	char reg_swing_5g = -1;	/* 0xff */
	char swing_2g = -1 * reg_swing_2g;
	char swing_5g = -1 * reg_swing_5g;
	uint32_t	out = 0x200;
	const char auto_temp = -1;


	if (efuse->autoload_failflag) {
		if (Band == BAND_ON_2_4G) {
			rtldm->swing_diff_2g = swing_2g;
			if      (swing_2g == 0)
				out = 0x200; /*  0 dB */
		        else if (swing_2g == -3)
				out = 0x16A; /* -3 dB */
		        else if (swing_2g == -6)
				out = 0x101; /* -6 dB */
		        else if (swing_2g == -9)
				out = 0x0B6; /* -9 dB */
		        else {
				if (rtlhal->external_pa_2g) {
					rtldm->swing_diff_2g = -3;
					out = 0x16A;
				} else  {
					rtldm->swing_diff_2g = 0;
					out = 0x200;
				}
			}
		} else if (Band == BAND_ON_5G) {
			rtldm->swing_diff_5g = swing_5g;
			if      (swing_5g == 0)
				out = 0x200; /*  0 dB */
			else if (swing_5g == -3)
				out = 0x16A; /* -3 dB */
			else if (swing_5g == -6)
				out = 0x101; /* -6 dB */
			else if (swing_5g == -9)
				out = 0x0B6; /* -9 dB */
			else {
				if (rtlhal->external_pa_5g) {
					rtldm->swing_diff_5g = -3;
					out = 0x16A;
				} else  {
					rtldm->swing_diff_5g = 0;
					out = 0x200;
				}
			}
		} else  {
			rtldm->swing_diff_2g = -3;
			rtldm->swing_diff_5g = -3;
			out = 0x16A; /* -3 dB */
		}
	} else {
		uint32_t swing = 0, swingA = 0, swingB = 0;

		if (Band == BAND_ON_2_4G) {
			if (reg_swing_2g == auto_temp) {
				EFUSE_ShadowRead(rtlpriv, 1, EEPROM_TX_BBSWING_2G_8812, (uint32_t *)&swing);
				swing = (swing == 0xFF) ? 0x00 : swing;
			} else if (swing_2g ==  0)
				swing = 0x00; /*  0 dB */
			else if (swing_2g == -3)
				swing = 0x05; /* -3 dB */
			else if (swing_2g == -6)
				swing = 0x0A; // -6 dB */
			else if (swing_2g == -9)
				swing = 0xFF; // -9 dB */
			else swing = 0x00;
		} else {
			if (reg_swing_5g == auto_temp) {
				EFUSE_ShadowRead(rtlpriv, 1, EEPROM_TX_BBSWING_5G_8812, (uint32_t *)&swing);
				swing = (swing == 0xFF) ? 0x00 : swing;
			} else if (swing_5g ==  0)
				swing = 0x00; /*  0 dB */
			else if (swing_5g == -3)
				swing = 0x05; /* -3 dB */
			else if (swing_5g == -6)
				swing = 0x0A; /* -6 dB */
			else if (swing_5g == -9)
				swing = 0xFF; /* -9 dB */
			else swing = 0x00;
		}

		swingA = (swing & 0x3) >> 0; /* 0xC6/C7[1:0] */
		swingB = (swing & 0xC) >> 2; /* 0xC6/C7[3:2] */

		/* DBG_871X("===> PHY_GetTxBBSwing_8812A, swingA: 0x%X, swingB: 0x%X\n", swingA, swingB); */

		/* 3 Path-A */
		if (swingA == 0x00) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = 0;
			else
				rtldm->swing_diff_5g = 0;
			out = 0x200; /* 0 dB */
		} else if (swingA == 0x01) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = -3;
			else
				rtldm->swing_diff_5g = -3;
			out = 0x16A; /*  -3 dB */
		} else if (swingA == 0x10) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = -6;
			else
				rtldm->swing_diff_5g = -6;
			out = 0x101; /* -6 dB */
		} else if (swingA == 0x11) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = -9;
			else
				rtldm->swing_diff_5g = -9;
			out = 0x0B6; /* -9 dB */
		}

		/* 3 Path-B */
		if (swingB == 0x00) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = 0;
			else
				rtldm->swing_diff_5g = 0;
			out = 0x200; /* 0 dB */
		} else if (swingB == 0x01) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = -3;
			else
				rtldm->swing_diff_5g = -3;
			out = 0x16A; /* -3 dB */
		} else if (swingB == 0x10) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = -6;
			else
				rtldm->swing_diff_5g = -6;
			out = 0x101; /* -6 dB */
		} else if (swingB == 0x11) {
			if (Band == BAND_ON_2_4G)
				rtldm->swing_diff_2g = -9;
			else
				rtldm->swing_diff_5g = -9;
			out = 0x0B6; /* -9 dB */
		}
	}

	/* DBG_871X("<=== PHY_GetTxBBSwing_8812A, out = 0x%X\n", out); */

	return out;
}


/*
 * ULLI : in rtlwifi check is needed for swithing band
 * ULLI : is in rtl8821au_phy_switch_wirelessband()
 * */

bool phy_SwBand8812(struct rtl_priv *rtlpriv, uint8_t channelToSW)
{
	uint8_t			u1Btmp;
	bool		ret_value = true;
	uint8_t			Band = BAND_ON_5G, BandToSW;

	u1Btmp = rtl_read_byte(rtlpriv, REG_CCK_CHECK_8812);
	if(u1Btmp & BIT(7))
		Band = BAND_ON_5G;
	else
		Band = BAND_ON_2_4G;

	/* Use current channel to judge Band Type and switch Band if need. */
	if(channelToSW > 14) {
		BandToSW = BAND_ON_5G;
	} else {
		BandToSW = BAND_ON_2_4G;
	}

	if(BandToSW != Band)
		rtl8821au_phy_switch_wirelessband(rtlpriv,BandToSW);

	return ret_value;
}
static void phy_SetRFEReg8812(struct rtl_priv *rtlpriv,uint8_t Band)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	u8			u1tmp = 0;

	if(Band == BAND_ON_2_4G) {
		switch(rtlhal->rfe_type){
		case 0: case 1: case 2:
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77777777);
			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77777777);
			rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x000);
			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x000);
			break;
		case 3:
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x54337770);
			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x54337770);
			rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			rtl_set_bbreg(rtlpriv, r_ANTSEL_SW_Jaguar,0x00000303, 0x1);
			break;
		case 4:
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77777777);
			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77777777);
			rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x001);
			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x001);
			break;
		case 5:
			/* if(BT_IsBtExist(rtlpriv)) */
			{
				/* rtl_write_word(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x7777); */
				rtl_write_byte(rtlpriv, rA_RFE_Pinmux_Jaguar+2, 0x77);
			}
			/* else */
				/* rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77777777); */

			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77777777);

			/* if(BT_IsBtExist(rtlpriv)) */
			{
				/*
				 * u1tmp = rtl_read_byte(rtlpriv, rA_RFE_Inv_Jaguar+2);
				 * rtl_write_byte(rtlpriv, rA_RFE_Inv_Jaguar+2,  (u1tmp &0x0f));
				 */
				u1tmp = rtl_read_byte(rtlpriv, rA_RFE_Inv_Jaguar+3);
				rtl_write_byte(rtlpriv, rA_RFE_Inv_Jaguar+3,  (u1tmp &= ~BIT(0)));
			}
			/* else */
				/* rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000); */

			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x000);
			break;
		default:
			break;
		}
	} else {
		switch(rtlhal->rfe_type){
		case 0:
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337717);
			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337717);
			rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			break;
		case 1:
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337717);
			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337717);
			rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x000);
			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x000);
			break;
		case 2: case 4:
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337777);
			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337777);
			rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			break;
		case 3:
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x54337717);
			rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x54337717);
			rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar,bMask_RFEInv_Jaguar, 0x010);
			rtl_set_bbreg(rtlpriv, r_ANTSEL_SW_Jaguar,0x00000303, 0x1);
			break;
		case 5:
			//if(BT_IsBtExist(rtlpriv))
			{
				//rtl_write_word(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x7777);
				if(rtlhal->external_pa_5g)
					rtl_write_byte(rtlpriv, rA_RFE_Pinmux_Jaguar+2, 0x33);
				else
					rtl_write_byte(rtlpriv, rA_RFE_Pinmux_Jaguar+2, 0x73);
			}
#if 0
			else
			{
				if (rtlhal->external_pa_5g)
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337777);
				else
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar,bMaskDWord, 0x77737777);
			}
#endif

			if (rtlhal->external_pa_5g)
				rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77337777);
			else
				rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar,bMaskDWord, 0x77737777);

			/* if(BT_IsBtExist(rtlpriv)) */
			{
				/*
				 * u1tmp = rtl_read_byte(rtlpriv, rA_RFE_Inv_Jaguar+2);
				 * rtl_write_byte(rtlpriv, rA_RFE_Inv_Jaguar+2,  (u1tmp &0x0f));
				 */
				u1tmp = rtl_read_byte(rtlpriv, rA_RFE_Inv_Jaguar+3);
				rtl_write_byte(rtlpriv, rA_RFE_Inv_Jaguar+3,  (u1tmp |= BIT(0)));
			}
			/* else */
				/* rtl_set_bbreg(rtlpriv, rA_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010); */

			rtl_set_bbreg(rtlpriv, rB_RFE_Inv_Jaguar, bMask_RFEInv_Jaguar, 0x010);
			break;
		default:
			break;
		}
	}
}

void rtl8821au_phy_switch_wirelessband(struct rtl_priv *rtlpriv, u8 Band)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	uint8_t				currentBand = rtlhal->current_bandtype;

	/* DBG_871X("==>rtl8821au_phy_switch_wirelessband() %s\n", ((Band==0)?"2.4G":"5G")); */

	rtlhal->current_bandtype =(enum band_type)Band;

	if(Band == BAND_ON_2_4G) {	/* 2.4G band */

		/* STOP Tx/Rx */
		rtl_set_bbreg(rtlpriv, rOFDMCCKEN_Jaguar, bOFDMEN_Jaguar|bCCKEN_Jaguar, 0x00);

		if (IS_HARDWARE_TYPE_8821(rtlhal)) {
			/* Turn off RF PA and LNA */
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0xF000, 0x7);	/* 0xCB0[15:12] = 0x7 (LNA_On) */
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0xF0, 0x7);	/* 0xCB0[7:4] = 0x7 (PAPE_A) */
		}

		/* AGC table select */
		if(IS_VENDOR_8821A_MP_CHIP(rtlhal->version))
			rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xF00, 0); // 0xC1C[11:8] = 0
		else
			rtl_set_bbreg(rtlpriv, rAGC_table_Jaguar, 0x3, 0);

		if(IS_VENDOR_8812A_TEST_CHIP(rtlhal->version)) {
			/* r_select_5G for path_A/B */
			rtl_set_bbreg(rtlpriv, rA_RFE_Jaguar, BIT(12), 0x0);
			rtl_set_bbreg(rtlpriv, rB_RFE_Jaguar, BIT(12), 0x0);

			/* LANON (5G uses external LNA) */
			rtl_set_bbreg(rtlpriv, rA_RFE_Jaguar, BIT(15), 0x1);
			rtl_set_bbreg(rtlpriv, rB_RFE_Jaguar, BIT(15), 0x1);
		} else if(IS_VENDOR_8812A_MP_CHIP(rtlhal->version)) {
			if(0 /* GetRegbENRFEType(rtlpriv) */)
			
				phy_SetRFEReg8812(rtlpriv, Band);
			else {
				/* PAPE_A (bypass RFE module in 2G) */
				rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x000000F0, 0x7);
				rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, 0x000000F0, 0x7);

				/* PAPE_G (bypass RFE module in 5G) */
				if (rtlhal->external_pa_2g) {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x0000000F, 0x0);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, 0x0000000F, 0x0);
				} else {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x0000000F, 0x7);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, 0x0000000F, 0x7);
				}

				/* TRSW bypass RFE moudle in 2G */
				if (rtlhal->external_lna_2g) {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, MASKBYTE2, 0x54);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, MASKBYTE2, 0x54);
				} else {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, MASKBYTE2, 0x77);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, MASKBYTE2, 0x77);
				}
			}
		}

		update_tx_basic_rate(rtlpriv, WIRELESS_11BG);

		/* cck_enable */
		rtl_set_bbreg(rtlpriv, rOFDMCCKEN_Jaguar, bOFDMEN_Jaguar|bCCKEN_Jaguar, 0x3);

		/* SYN Setting */
		if(IS_VENDOR_8812A_TEST_CHIP(rtlhal->version)) 	{
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0xEF, bLSSIWrite_data_Jaguar, 0x40000);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0x3E, bLSSIWrite_data_Jaguar, 0x00000);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0x3F, bLSSIWrite_data_Jaguar, 0x0001c);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0xEF, bLSSIWrite_data_Jaguar, 0x00000);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0xB5, bLSSIWrite_data_Jaguar, 0x16BFF);
		}

		/* CCK_CHECK_en */
		rtl_write_byte(rtlpriv, REG_CCK_CHECK_8812, 0x0);
	} else {		/* 5G band */
		u16	count = 0, reg41A = 0;

		if (IS_HARDWARE_TYPE_8821(rtlhal)) {
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0xF000, 0x5);	/* 0xCB0[15:12] = 0x5 (LNA_On) */
			rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0xF0, 0x4);	/* 0xCB0[7:4] = 0x4 (PAPE_A) */
		}

		/* CCK_CHECK_en */
		rtl_write_byte(rtlpriv, REG_CCK_CHECK_8812, 0x80);

		count = 0;
		reg41A = rtl_read_word(rtlpriv, REG_TXPKT_EMPTY);
		/* DBG_871X("Reg41A value %d", reg41A); */
		reg41A &= 0x30;
		while((reg41A!= 0x30) && (count < 50)) {
			udelay(50);
			/* DBG_871X("Delay 50us \n"); */

			reg41A = rtl_read_word(rtlpriv, REG_TXPKT_EMPTY);
			reg41A &= 0x30;
			count++;
			/* DBG_871X("Reg41A value %d", reg41A); */
		}
		if(count != 0)
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "rtl8821au_phy_switch_wirelessband(): Switch to 5G Band. Count = %d reg41A=0x%x\n", count, reg41A);

		/* STOP Tx/Rx */
		rtl_set_bbreg(rtlpriv, rOFDMCCKEN_Jaguar, bOFDMEN_Jaguar|bCCKEN_Jaguar, 0x00);

		/* AGC table select */
		if (IS_VENDOR_8821A_MP_CHIP(rtlhal->version))
			rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xF00, 1); /* 0xC1C[11:8] = 1 */
		else
			rtl_set_bbreg(rtlpriv, rAGC_table_Jaguar, 0x3, 1);

		if(IS_VENDOR_8812A_TEST_CHIP(rtlhal->version)) 	{
			/* r_select_5G for path_A/B */
			rtl_set_bbreg(rtlpriv, rA_RFE_Jaguar, BIT(12), 0x1);
			rtl_set_bbreg(rtlpriv, rB_RFE_Jaguar, BIT(12), 0x1);

			/* LANON (5G uses external LNA) */
			rtl_set_bbreg(rtlpriv, rA_RFE_Jaguar, BIT(15), 0x0);
			rtl_set_bbreg(rtlpriv, rB_RFE_Jaguar, BIT(15), 0x0);
		} else if(IS_VENDOR_8812A_MP_CHIP(rtlhal->version)) {
			if(0 /* GetRegbENRFEType(rtlpriv) */)
				phy_SetRFEReg8812(rtlpriv, Band);
			else {
				/* PAPE_A (bypass RFE module in 2G) */
				if (rtlhal->external_pa_5g) {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x000000F0, 0x1);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, 0x000000F0, 0x1);
				} else {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x000000F0, 0x0);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, 0x000000F0, 0x0);
				}

				/* PAPE_G (bypass RFE module in 5G) */
				rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, 0x0000000F, 0x7);
				rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, 0x0000000F, 0x7);

				/* TRSW bypass RFE moudle in 2G */
				if (rtlhal->external_lna_5g) {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, MASKBYTE2, 0x54);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, MASKBYTE2, 0x54);
				} else {
					rtl_set_bbreg(rtlpriv, rA_RFE_Pinmux_Jaguar, MASKBYTE2, 0x77);
					rtl_set_bbreg(rtlpriv, rB_RFE_Pinmux_Jaguar, MASKBYTE2, 0x77);
				}
			}
		}

		/*
		 * avoid using cck rate in 5G band
		 * Set RRSR rate table.
		 */
		update_tx_basic_rate(rtlpriv, WIRELESS_11A);

		/* cck_enable */
		rtl_set_bbreg(rtlpriv, rOFDMCCKEN_Jaguar, bOFDMEN_Jaguar|bCCKEN_Jaguar, 0x2);

		/* SYN Setting */
		if(IS_VENDOR_8812A_TEST_CHIP(rtlhal->version)) 	{
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0xEF, bLSSIWrite_data_Jaguar, 0x40000);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0x3E, bLSSIWrite_data_Jaguar, 0x00000);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0x3F, bLSSIWrite_data_Jaguar, 0x00017);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0xEF, bLSSIWrite_data_Jaguar, 0x00000);
			rtl_set_rfreg(rtlpriv, RF90_PATH_A, 0xB5, bLSSIWrite_data_Jaguar, 0x04BFF);
		}

		/* DBG_871X("==>rtl8821au_phy_switch_wirelessband() BAND_ON_5G settings OFDM index 0x%x\n", pHalData->OFDM_index[RF90_PATH_A]); */
	}

	/* <20120903, Kordan> Tx BB swing setting for RL6286, asked by Ynlin. */
	if (IS_NORMAL_CHIP(rtlhal->version) || IS_HARDWARE_TYPE_8821(rtlhal)) {
		s8	BBDiffBetweenBand = 0;
		struct rtl_dm	*rtldm = rtl_dm(rtlpriv);

		rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000,
					 phy_get_tx_swing_8821au(rtlpriv, (enum band_type)Band, RF90_PATH_A)); // 0xC1C[31:21]
		rtl_set_bbreg(rtlpriv, rB_TxScale_Jaguar, 0xFFE00000,
					 phy_get_tx_swing_8821au(rtlpriv, (enum band_type)Band, RF90_PATH_B)); // 0xE1C[31:21]

		/*
		 *  <20121005, Kordan> When TxPowerTrack is ON, we should take care of the change of BB swing.
		 *  That is, reset all info to trigger Tx power tracking.
		 */
		{
			if (Band != currentBand) {
				BBDiffBetweenBand = (rtldm->swing_diff_2g - rtldm->swing_diff_5g);
				BBDiffBetweenBand = (Band == BAND_ON_2_4G) ? BBDiffBetweenBand : (-1 * BBDiffBetweenBand);
				rtldm->default_ofdm_index += BBDiffBetweenBand*2;
			}

			rtl8821au_dm_clean_txpower_tracking_state(rtlpriv);
		}
	}

	/* DBG_871X("<==rtl8821au_phy_switch_wirelessband():Switch Band OK.\n"); */
}

static void _rtl8821au_phy_set_txpower_index(struct rtl_priv *rtlpriv, uint32_t power_index,
	u8 path, u8 rate)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	/*
	 *  <20120928, Kordan> A workaround in 8812A/8821A testchip, to fix the bug of odd Tx power indexes.
	 */
	if ((power_index % 2 == 1) && !IS_NORMAL_CHIP(rtlhal->version))
		power_index -= 1;

	/* ULLI check register names as in rtlwifi-lib */

	if (path == RF90_PATH_A) {
		switch (rate) {
		case MGN_1M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_CCK11_CCK1, MASKBYTE0, power_index);
			break;
		case MGN_2M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_CCK11_CCK1, MASKBYTE1, power_index);
			break;
		case MGN_5_5M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_CCK11_CCK1, MASKBYTE2, power_index);
			break;
		case MGN_11M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_CCK11_CCK1, MASKBYTE3, power_index);
			break;

		case MGN_6M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM18_OFDM6, MASKBYTE0, power_index);
			break;
		case MGN_9M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM18_OFDM6, MASKBYTE1, power_index);
			break;
		case MGN_12M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM18_OFDM6, MASKBYTE2, power_index);
			break;
		case MGN_18M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM18_OFDM6, MASKBYTE3, power_index);
			break;

		case MGN_24M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM54_OFDM24, MASKBYTE0, power_index);
			break;
		case MGN_36M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM54_OFDM24, MASKBYTE1, power_index);
			break;
		case MGN_48M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM54_OFDM24, MASKBYTE2, power_index);
			break;
		case MGN_54M:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_OFDM54_OFDM24, MASKBYTE3, power_index);
			break;

		case MGN_MCS0:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS03_MCS00, MASKBYTE0, power_index);
			break;
		case MGN_MCS1:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS03_MCS00, MASKBYTE1, power_index);
			break;
		case MGN_MCS2:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS03_MCS00, MASKBYTE2, power_index);
			break;
		case MGN_MCS3:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS03_MCS00, MASKBYTE3, power_index);
			break;

		case MGN_MCS4:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS07_MCS04, MASKBYTE0, power_index);
			break;
		case MGN_MCS5:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS07_MCS04, MASKBYTE1, power_index);
			break;
		case MGN_MCS6:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS07_MCS04, MASKBYTE2, power_index);
			break;
		case MGN_MCS7:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS07_MCS04, MASKBYTE3, power_index);
			break;

		case MGN_MCS8:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS11_MCS08, MASKBYTE0, power_index);
			break;
		case MGN_MCS9:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS11_MCS08, MASKBYTE1, power_index);
			break;
		case MGN_MCS10:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS11_MCS08, MASKBYTE2, power_index);
			break;
		case MGN_MCS11:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS11_MCS08, MASKBYTE3, power_index);
			break;

		case MGN_MCS12:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS15_MCS12, MASKBYTE0, power_index);
			break;
		case MGN_MCS13:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS15_MCS12, MASKBYTE1, power_index);
			break;
		case MGN_MCS14:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS15_MCS12, MASKBYTE2, power_index);
			break;
		case MGN_MCS15:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_MCS15_MCS12, MASKBYTE3, power_index);
			break;

		case MGN_VHT1SS_MCS0:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX3_NSS1INDEX0, MASKBYTE0, power_index);
			break;
		case MGN_VHT1SS_MCS1:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX3_NSS1INDEX0, MASKBYTE1, power_index);
			break;
		case MGN_VHT1SS_MCS2:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX3_NSS1INDEX0, MASKBYTE2, power_index);
			break;
		case MGN_VHT1SS_MCS3:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX3_NSS1INDEX0, MASKBYTE3, power_index);
			break;

		case MGN_VHT1SS_MCS4:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX7_NSS1INDEX4, MASKBYTE0, power_index);
			break;
		case MGN_VHT1SS_MCS5:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX7_NSS1INDEX4, MASKBYTE1, power_index);
			break;
		case MGN_VHT1SS_MCS6:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX7_NSS1INDEX4, MASKBYTE2, power_index);
			break;
		case MGN_VHT1SS_MCS7:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS1INDEX7_NSS1INDEX4, MASKBYTE3, power_index);
			break;

		case MGN_VHT1SS_MCS8:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX1_NSS1INDEX8, MASKBYTE0, power_index);
			break;
		case MGN_VHT1SS_MCS9:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX1_NSS1INDEX8, MASKBYTE1, power_index);
			break;
		case MGN_VHT2SS_MCS0:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX1_NSS1INDEX8, MASKBYTE2, power_index);
			break;
		case MGN_VHT2SS_MCS1:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX1_NSS1INDEX8, MASKBYTE3, power_index);
			break;

		case MGN_VHT2SS_MCS2:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX5_NSS2INDEX2, MASKBYTE0, power_index);
			break;
		case MGN_VHT2SS_MCS3:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX5_NSS2INDEX2, MASKBYTE1, power_index);
			break;
		case MGN_VHT2SS_MCS4:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX5_NSS2INDEX2, MASKBYTE2, power_index);
			break;
		case MGN_VHT2SS_MCS5:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX5_NSS2INDEX2, MASKBYTE3, power_index);
			break;

		case MGN_VHT2SS_MCS6:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX9_NSS2INDEX6, MASKBYTE0, power_index);
			break;
		case MGN_VHT2SS_MCS7:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX9_NSS2INDEX6, MASKBYTE1, power_index);
			break;
		case MGN_VHT2SS_MCS8:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX9_NSS2INDEX6, MASKBYTE2, power_index);
			break;
		case MGN_VHT2SS_MCS9:
			rtl_set_bbreg(rtlpriv, RTXAGC_A_NSS2INDEX9_NSS2INDEX6, MASKBYTE3, power_index);
			break;

		default:
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid Rate!!\n");
			break;
		}
	} else if (path == RF90_PATH_B) {
		switch (rate) {
		case MGN_1M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_CCK11_CCK1, MASKBYTE0, power_index);
			break;
		case MGN_2M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_CCK11_CCK1, MASKBYTE1, power_index);
			break;
		case MGN_5_5M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_CCK11_CCK1, MASKBYTE2, power_index);
			break;
		case MGN_11M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_CCK11_CCK1, MASKBYTE3, power_index);
			break;

		case MGN_6M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM18_OFDM6, MASKBYTE0, power_index);
			break;
		case MGN_9M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM18_OFDM6, MASKBYTE1, power_index);
			break;
		case MGN_12M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM18_OFDM6, MASKBYTE2, power_index);
			break;
		case MGN_18M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM18_OFDM6, MASKBYTE3, power_index);
			break;

		case MGN_24M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM54_OFDM24, MASKBYTE0, power_index);
			break;
		case MGN_36M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM54_OFDM24, MASKBYTE1, power_index);
			break;
		case MGN_48M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM54_OFDM24, MASKBYTE2, power_index);
			break;
		case MGN_54M:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_OFDM54_OFDM24, MASKBYTE3, power_index);
			break;

		case MGN_MCS0:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS03_MCS00, MASKBYTE0, power_index);
			break;
		case MGN_MCS1:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS03_MCS00, MASKBYTE1, power_index);
			break;
		case MGN_MCS2:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS03_MCS00, MASKBYTE2, power_index);
			break;
		case MGN_MCS3:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS03_MCS00, MASKBYTE3, power_index);
			break;

		case MGN_MCS4:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS07_MCS04, MASKBYTE0, power_index);
			break;
		case MGN_MCS5:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS07_MCS04, MASKBYTE1, power_index);
			break;
		case MGN_MCS6:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS07_MCS04, MASKBYTE2, power_index);
			break;
		case MGN_MCS7:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS07_MCS04, MASKBYTE3, power_index);
			break;

		case MGN_MCS8:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS11_MCS08, MASKBYTE0, power_index);
			break;
		case MGN_MCS9:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS11_MCS08, MASKBYTE1, power_index);
			break;
		case MGN_MCS10:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS11_MCS08, MASKBYTE2, power_index);
			break;
		case MGN_MCS11:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS11_MCS08, MASKBYTE3, power_index);
			break;

		case MGN_MCS12:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS15_MCS12, MASKBYTE0, power_index);
			break;
		case MGN_MCS13:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS15_MCS12, MASKBYTE1, power_index);
			break;
		case MGN_MCS14:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS15_MCS12, MASKBYTE2, power_index);
			break;
		case MGN_MCS15:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_MCS15_MCS12, MASKBYTE3, power_index);
			break;

		case MGN_VHT1SS_MCS0:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX3_NSS1INDEX0, MASKBYTE0, power_index);
			break;
		case MGN_VHT1SS_MCS1:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX3_NSS1INDEX0, MASKBYTE1, power_index);
			break;
		case MGN_VHT1SS_MCS2:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX3_NSS1INDEX0, MASKBYTE2, power_index);
			break;
		case MGN_VHT1SS_MCS3:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX3_NSS1INDEX0, MASKBYTE3, power_index);
			break;

		case MGN_VHT1SS_MCS4:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX7_NSS1INDEX4, MASKBYTE0, power_index);
			break;
		case MGN_VHT1SS_MCS5:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX7_NSS1INDEX4, MASKBYTE1, power_index);
			break;
		case MGN_VHT1SS_MCS6:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX7_NSS1INDEX4, MASKBYTE2, power_index);
			break;
		case MGN_VHT1SS_MCS7:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS1INDEX7_NSS1INDEX4, MASKBYTE3, power_index);
			break;

		case MGN_VHT1SS_MCS8:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX1_NSS1INDEX8, MASKBYTE0, power_index);
			break;
		case MGN_VHT1SS_MCS9:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX1_NSS1INDEX8, MASKBYTE1, power_index);
			break;
		case MGN_VHT2SS_MCS0:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX1_NSS1INDEX8, MASKBYTE2, power_index);
			break;
		case MGN_VHT2SS_MCS1:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX1_NSS1INDEX8, MASKBYTE3, power_index);
			break;

		case MGN_VHT2SS_MCS2:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX5_NSS2INDEX2, MASKBYTE0, power_index);
			break;
		case MGN_VHT2SS_MCS3:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX5_NSS2INDEX2, MASKBYTE1, power_index);
			break;
		case MGN_VHT2SS_MCS4:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX5_NSS2INDEX2, MASKBYTE2, power_index);
			break;
		case MGN_VHT2SS_MCS5:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX5_NSS2INDEX2, MASKBYTE3, power_index);
			break;

		case MGN_VHT2SS_MCS6:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX9_NSS2INDEX6, MASKBYTE0, power_index);
			break;
		case MGN_VHT2SS_MCS7:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX9_NSS2INDEX6, MASKBYTE1, power_index);
			break;
		case MGN_VHT2SS_MCS8:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX9_NSS2INDEX6, MASKBYTE2, power_index);
			break;
		case MGN_VHT2SS_MCS9:
			rtl_set_bbreg(rtlpriv, RTXAGC_B_NSS2INDEX9_NSS2INDEX6, MASKBYTE3, power_index);
			break;

		default:
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid Rate!!\n");
			break;
		}
	} else {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid RFPath!!\n");
	}
}

static void _rtl8821au_phy_set_txpower_level_by_path(struct rtl_priv *rtlpriv, uint8_t RFPath,
	enum CHANNEL_WIDTH BandWidth, uint8_t Channel, uint8_t *Rates,
	uint8_t	RateArraySize)
{
	uint32_t power_index = 0;
	int	i = 0;

	for (i = 0; i < RateArraySize; ++i) {
		power_index = _rtl8821au_get_txpower_index(rtlpriv, RFPath, Rates[i], BandWidth, Channel);
		_rtl8821au_phy_set_txpower_index(rtlpriv, power_index, RFPath, Rates[i]);
	}

}
static void _rtl8821au_phy_txpower_training_by_path(struct rtl_priv *rtlpriv,
	enum CHANNEL_WIDTH BandWidth, uint8_t Channel, uint8_t RfPath)
{
	uint8_t	i;
	uint32_t	PowerLevel, writeData, writeOffset;

	if(RfPath >=  rtlpriv->phy.num_total_rfpath)
		return;

	writeData = 0;

	if (RfPath == RF90_PATH_A) {
		PowerLevel = _rtl8821au_get_txpower_index(rtlpriv, RF90_PATH_A, MGN_MCS7, BandWidth, Channel);
		writeOffset =  rA_TxPwrTraing_Jaguar;
	} else {
		PowerLevel = _rtl8821au_get_txpower_index(rtlpriv, RF90_PATH_B, MGN_MCS7, BandWidth, Channel);
		writeOffset =  rB_TxPwrTraing_Jaguar;
	}

	for (i = 0; i < 3; i++) {
		if(i == 0)
			PowerLevel = PowerLevel - 10;
		else if(i == 1)
			PowerLevel = PowerLevel - 8;
		else
			PowerLevel = PowerLevel - 6;

		writeData |= (((PowerLevel > 2)?(PowerLevel):2) << (i * 8));
	}

	rtl_set_bbreg(rtlpriv, writeOffset, 0xffffff, writeData);
}


static void rtl8821au_phy_set_txpower_level_by_path(struct rtl_priv *rtlpriv,
	uint8_t	channel, uint8_t path)
{
	uint8_t	cckRates[]   = {MGN_1M, MGN_2M, MGN_5_5M, MGN_11M};
	uint8_t	ofdmRates[]  = {MGN_6M, MGN_9M, MGN_12M, MGN_18M, MGN_24M, MGN_36M, MGN_48M, MGN_54M};
	uint8_t	htRates1T[]  = {MGN_MCS0, MGN_MCS1, MGN_MCS2, MGN_MCS3, MGN_MCS4, MGN_MCS5, MGN_MCS6, MGN_MCS7};
	uint8_t	htRates2T[]  = {MGN_MCS8, MGN_MCS9, MGN_MCS10, MGN_MCS11, MGN_MCS12, MGN_MCS13, MGN_MCS14, MGN_MCS15};
	uint8_t	vhtRates1T[] = {MGN_VHT1SS_MCS0, MGN_VHT1SS_MCS1, MGN_VHT1SS_MCS2, MGN_VHT1SS_MCS3, MGN_VHT1SS_MCS4,
				MGN_VHT1SS_MCS5, MGN_VHT1SS_MCS6, MGN_VHT1SS_MCS7, MGN_VHT1SS_MCS8, MGN_VHT1SS_MCS9};
	uint8_t	vhtRates2T[] = {MGN_VHT2SS_MCS0, MGN_VHT2SS_MCS1, MGN_VHT2SS_MCS2, MGN_VHT2SS_MCS3, MGN_VHT2SS_MCS4,
				MGN_VHT2SS_MCS5, MGN_VHT2SS_MCS6, MGN_VHT2SS_MCS7, MGN_VHT2SS_MCS8, MGN_VHT2SS_MCS9};

	//DBG_871X("==>PHY_SetTxPowerLevelByPath8812()\n");

	//if(pMgntInfo->RegNByteAccess == 0)
	if(rtlpriv->rtlhal.current_bandtype == BAND_ON_2_4G)
		_rtl8821au_phy_set_txpower_level_by_path(rtlpriv, path, rtlpriv->phy.current_chan_bw, channel,
								  cckRates, sizeof(cckRates)/sizeof(u8));

	_rtl8821au_phy_set_txpower_level_by_path(rtlpriv, path, rtlpriv->phy.current_chan_bw, channel,
								  ofdmRates, sizeof(ofdmRates)/sizeof(u8));
	_rtl8821au_phy_set_txpower_level_by_path(rtlpriv, path, rtlpriv->phy.current_chan_bw, channel,
								  htRates1T, sizeof(htRates1T)/sizeof(u8));
	_rtl8821au_phy_set_txpower_level_by_path(rtlpriv, path, rtlpriv->phy.current_chan_bw, channel,
							  	  vhtRates1T, sizeof(vhtRates1T)/sizeof(u8));

	if ( rtlpriv->phy.num_total_rfpath >= 2) {
		_rtl8821au_phy_set_txpower_level_by_path(rtlpriv, path, rtlpriv->phy.current_chan_bw, channel,
							  htRates2T, sizeof(htRates2T)/sizeof(u8));
		_rtl8821au_phy_set_txpower_level_by_path(rtlpriv, path, rtlpriv->phy.current_chan_bw, channel,
							  vhtRates2T, sizeof(vhtRates2T)/sizeof(u8));
	}

	_rtl8821au_phy_txpower_training_by_path(rtlpriv, rtlpriv->phy.current_chan_bw, channel, path);

	/* DBG_871X("<==PHY_SetTxPowerLevelByPath8812()\n"); */
}



void PHY_SetTxPowerLevel8812(struct rtl_priv *rtlpriv, uint8_t	Channel)
{
	uint8_t	path = 0;

	/* DBG_871X("==>PHY_SetTxPowerLevel8812()\n"); */

	for (path = RF90_PATH_A; path < rtlpriv->phy.num_total_rfpath; ++path) {
		rtl8821au_phy_set_txpower_level_by_path(rtlpriv, Channel, path);
	}

	/* DBG_871X("<==PHY_SetTxPowerLevel8812()\n"); */
}

#undef READ_NEXT_PAIR
#define READ_NEXT_PAIR(v1, v2, i) do { i += 2; v1 = Array[i]; v2 = Array[i+1]; } while(0)


static void _rtl8821au_config_bb_reg(struct rtl_priv *rtlpriv, uint32_t Addr,
	uint32_t Bitmask, uint32_t Data)
{
	if (Addr == 0xfe)
		msleep(50);
	else if (Addr == 0xfd)
		mdelay(5);
	else if (Addr == 0xfc)
		mdelay(1);
	else if (Addr == 0xfb)
		udelay(50);
	else if (Addr == 0xfa)
		udelay(5);
	else if (Addr == 0xf9)
		udelay(1);

	rtl_set_bbreg(rtlpriv, Addr, Bitmask, Data);

	/* Add 1us delay between BB/RF register setting. */
	udelay(1);
	RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ConfigBBWithHeaderFile: [PHY_REG] %08X %08X\n", Addr, Data);
}


/******************** */


static void _phy_convert_txpower_dbm_to_relative_value(u32 *data, u8 start,
						       u8 end, u8 base_val)
{
	s8 i = 0;
	u8 temp_value = 0;
	u32 temp_data = 0;

	/* BaseValue = ( BaseValue & 0xf ) + ( ( BaseValue >> 4 ) & 0xf ) * 10; */
	/* RT_TRACE(COMP_INIT, DBG_LOUD, ("Corrected BaseValue %u\n", BaseValue ) ); */

	for (i = 3; i >= 0; --i) {
		if (i >= start && i <= end) {
			/* Get the exact value */
			temp_value = (u8) (*data >> (i * 8) ) & 0xF;
			temp_value += (( u8) (( *data >> (i * 8 + 4)) & 0xF)) * 10;

			/* Change the value to a relative value */
			temp_value = (temp_value > base_val) ?
				(temp_value - base_val) :
				(base_val - temp_value);
		} else {
			temp_value = (u8) (*data >> (i * 8)) & 0xFF;
		}

		temp_data <<= 8;
		temp_data |= temp_value;
	}

	*data = temp_data;
}

/*
 * 2012/10/18
 */
 
static s8 _rtl8821au_phy_get_txpower_by_rate(struct rtl_priv *rtlpriv,
					     u8 Band, u8 RFPath,
					     u8 TxNum, u8 Rate)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	s8 			value = 0, limit = 0;
	u8			rateIndex = PHY_GetRateIndexOfTxPowerByRate( Rate );

#if 0
	if ( ( pAdapter->registrypriv.RegEnableTxPowerByRate == 2 && pHalData->EEPROMRegulatory == 2 ) ||
		   pAdapter->registrypriv.RegEnableTxPowerByRate == 0 )
		return 0;
#endif
	if (Band != BAND_ON_2_4G && Band != BAND_ON_5G ) {
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Invalid band %d in %s\n", Band, __func__);
		return value;
	}
	if (RFPath > RF90_PATH_D) {
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Invalid RfPath %d in %s\n", RFPath, __func__);
		return value;
	}
	if (TxNum >= RF_MAX_TX_NUM) {
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Invalid TxNum %d in %s\n", TxNum, __func__);
		return value;
	}
	if (rateIndex >= TX_PWR_BY_RATE_NUM_RATE) {
		RT_TRACE(rtlpriv, COMP_IQK, DBG_LOUD, "Invalid RateIndex %d in %s\n", rateIndex, __func__);
		return value;
	}

	value = rtlphy->tx_power_by_rate_offset[Band][RFPath][TxNum][rateIndex];

	return value;

}


static u8 _rtl8821au_phy_get_txpower_by_rate_base(struct rtl_priv *rtlpriv,
					     u8 band,
					     u8 path,
					     u8 txnum, u8 rate_section)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	u8 value = 0;

	if (path > RF90_PATH_D) {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
			 "Invalid Rf Path %d in PHY_GetTxPowerByRateBase()\n", path);
		return 0;
	}

	if (band == BAND_ON_2_4G) {
		switch (rate_section) {
		case CCK:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][0];
			break;
		case OFDM:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][1];
			break;
		case HT_MCS0_MCS7:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][2];
			break;
		case HT_MCS8_MCS15:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][3];
			break;
		case HT_MCS16_MCS23:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][4];
			break;
		case HT_MCS24_MCS31:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][5];
			break;
		case VHT_1SSMCS0_1SSMCS9:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][6];
			break;
		case VHT_2SSMCS0_2SSMCS9:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][7];
			break;
		case VHT_3SSMCS0_3SSMCS9:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][8];
			break;
		case VHT_4SSMCS0_4SSMCS9:
			value = rtlphy->txpwr_by_rate_base_24g[path][txnum][9];
			break;
		default:
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
				 "Invalid RateSection %d in Band 2.4G, Rf Path %d, %dTx in PHY_GetTxPowerByRateBase()\n",
				 rate_section, path, txnum);
			break;
		};
	} else if (band == BAND_ON_5G) {
		switch (rate_section) {
		case OFDM:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][0];
			break;
		case HT_MCS0_MCS7:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][1];
			break;
		case HT_MCS8_MCS15:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][2];
			break;
		case HT_MCS16_MCS23:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][3];
			break;
		case HT_MCS24_MCS31:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][4];
			break;
		case VHT_1SSMCS0_1SSMCS9:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][5];
			break;
		case VHT_2SSMCS0_2SSMCS9:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][6];
			break;
		case VHT_3SSMCS0_3SSMCS9:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][7];
			break;
		case VHT_4SSMCS0_4SSMCS9:
			value = rtlphy->txpwr_by_rate_base_5g[path][txnum][8];
			break;
		default:
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
				 "Invalid RateSection %d in Band 5G, Rf Path %d, %dTx in PHY_GetTxPowerByRateBase()\n",
				 rate_section, path, txnum );
			break;
		};
	} else {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
			 "Invalid Band %d in PHY_GetTxPowerByRateBase()\n",
			 band);
	}

	return value;
}

static void _rtl8821au_phy_set_txpower_by_rate_base(struct rtl_priv *rtlpriv,
						    u8 band, u8 path, 
						    u8 rate_section,
						    u8 txnum, u8 value)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);

	if (path > RF90_PATH_D) {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
			 "Invalid Rf Path %d in phy_SetTxPowerByRatBase()\n",
			 path);
		return;
	}

	if (band == BAND_ON_2_4G) {
		switch (rate_section) {
		case CCK:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][0] = value;
			break;
		case OFDM:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][1] = value;
			break;
		case HT_MCS0_MCS7:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][2] = value;
			break;
		case HT_MCS8_MCS15:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][3] = value;
			break;
		case HT_MCS16_MCS23:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][4] = value;
			break;
		case HT_MCS24_MCS31:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][5] = value;
			break;
		case VHT_1SSMCS0_1SSMCS9:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][6] = value;
			break;
		case VHT_2SSMCS0_2SSMCS9:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][7] = value;
			break;
		case VHT_3SSMCS0_3SSMCS9:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][8] = value;
			break;
		case VHT_4SSMCS0_4SSMCS9:
			rtlphy->txpwr_by_rate_base_24g[path][txnum][9] = value;
			break;
		default:
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
				 "Invalid RateSection %d in Band 2.4G, Rf Path %d, %dTx in phy_SetTxPowerByRateBase()\n",
				 rate_section, path, txnum );
			break;
		};
	} else if (band == BAND_ON_5G) {
		switch (rate_section) {
		case OFDM:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][0] = value;
			break;
		case HT_MCS0_MCS7:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][1] = value;
			break;
		case HT_MCS8_MCS15:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][2] = value;
			break;
		case HT_MCS16_MCS23:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][3] = value;
			break;
		case HT_MCS24_MCS31:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][4] = value;
			break;
		case VHT_1SSMCS0_1SSMCS9:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][5] = value;
			break;
		case VHT_2SSMCS0_2SSMCS9:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][6] = value;
			break;
		case VHT_3SSMCS0_3SSMCS9:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][7] = value;
			break;
		case VHT_4SSMCS0_4SSMCS9:
			rtlphy->txpwr_by_rate_base_5g[path][txnum][8] = value;
			break;
		default:
			RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
				 "Invalid RateSection %d in Band 5G, Rf Path %d, %dTx in phy_SetTxPowerByRateBase()\n",
				 rate_section, path, txnum);
			break;
		};
	} else {
		RT_TRACE(rtlpriv, COMP_INIT, DBG_LOUD,
			 "Invalid Band %d in phy_SetTxPowerByRateBase()\n",
			 band);
	}
}

static void _rtl8821au_get_values_of_txpwr_by_rate(struct rtl_priv *rtlpriv,
						   u32 RegAddr, u32 BitMask,
						   u32 Value, u8 *rate_idx,
						   s8 *PwrByRateVal, u8 *RateNum)
{
	u8 index = 0, i;

	switch (RegAddr) {
	case rTxAGC_A_Rate18_06:
	case rTxAGC_B_Rate18_06:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_6M);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_9M);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_12M);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_18M);
		for (i = 0; i < 4; ++ i ) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case rTxAGC_A_Rate54_24:
	case rTxAGC_B_Rate54_24:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_24M);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_36M);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_48M);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_54M);
		for ( i = 0; i < 4; ++ i ) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case rTxAGC_A_CCK1_Mcs32:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_1M);
		PwrByRateVal[0] = (s8) ((((Value >> (8 + 4)) & 0xF)) * 10 +
					(( Value >> 8 ) & 0xF));
		*RateNum = 1;
		break;

	case rTxAGC_B_CCK11_A_CCK2_11:
		if (BitMask == 0xffffff00) {
			rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_2M);
			rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_5_5M);
			rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_11M);
			for (i = 1; i < 4; ++ i) {
				PwrByRateVal[i - 1] = (s8)
						      ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
						       (( Value >> (i * 8)) & 0xF ));
			}
			*RateNum = 3;
		} else if ( BitMask == 0x000000ff ) {
			rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_11M);
			PwrByRateVal[0] = (s8) ((((Value >> 4 ) & 0xF )) * 10 +
						(Value & 0xF));
			*RateNum = 1;
		}
		break;

	case rTxAGC_A_Mcs03_Mcs00:
	case rTxAGC_B_Mcs03_Mcs00:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS0);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS1);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS2);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS3);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   (( Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case rTxAGC_A_Mcs07_Mcs04:
	case rTxAGC_B_Mcs07_Mcs04:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS4);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS5);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS6);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS7);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case rTxAGC_A_Mcs11_Mcs08:
	case rTxAGC_B_Mcs11_Mcs08:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS8);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS9);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS10);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS11);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case rTxAGC_A_Mcs15_Mcs12:
	case rTxAGC_B_Mcs15_Mcs12:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS12);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS13);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS14);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS15);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8) ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
						((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case rTxAGC_B_CCK1_55_Mcs32:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_1M);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_2M);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_5_5M);
		for (i = 1; i < 4; ++ i) {
			PwrByRateVal[i - 1] = (s8)
					      ((((Value >> (i * 8 + 4)) & 0xF )) * 10 +
					       (( Value >> ( i * 8) ) & 0xF ) );
		}
		*RateNum = 3;
		break;

	case 0xC20:
	case 0xE20:
	case 0x1820:
	case 0x1a20:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_1M);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_2M);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_5_5M);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_11M);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   (( Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xC24:
	case 0xE24:
	case 0x1824:
	case 0x1a24:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_6M);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_9M);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_12M);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_18M);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xC28:
	case 0xE28:
	case 0x1828:
	case 0x1a28:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_24M);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_36M);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_48M);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_54M);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   (( Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xC2C:
	case 0xE2C:
	case 0x182C:
	case 0x1a2C:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS0);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS1);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS2);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS3);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xC30:
	case 0xE30:
	case 0x1830:
	case 0x1a30:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS4);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS5);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS6);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS7);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xC34:
	case 0xE34:
	case 0x1834:
	case 0x1a34:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS8);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS9);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS10);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS11);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xC38:
	case 0xE38:
	case 0x1838:
	case 0x1a38:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS12);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS13);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS14);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS15);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xC3C:
	case 0xE3C:
	case 0x183C:
	case 0x1a3C:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS0);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS1);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS2);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS3);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xC40:
	case 0xE40:
	case 0x1840:
	case 0x1a40:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS4);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS5);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS6);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS7);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xC44:
	case 0xE44:
	case 0x1844:
	case 0x1a44:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS8);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT1SS_MCS9);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS0);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS1);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF )) * 10 +
					   (( Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xC48:
	case 0xE48:
	case 0x1848:
	case 0x1a48:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS2);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS3);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS4);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS5);
		for (i = 0; i < 4; ++ i) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xC4C:
	case 0xE4C:
	case 0x184C:
	case 0x1a4C:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS6);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS7);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS8);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT2SS_MCS9);
		for ( i = 0; i < 4; ++ i ) {
			PwrByRateVal[i] = (s8)
					  (((( Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   (( Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xCD8:
	case 0xED8:
	case 0x18D8:
	case 0x1aD8:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS16);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS17);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS18);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS19);
		for ( i = 0; i < 4; ++ i ) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF )) * 10 +
					   ((Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xCDC:
	case 0xEDC:
	case 0x18DC:
	case 0x1aDC:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS20);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS21);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS22);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_MCS23);
		for ( i = 0; i < 4; ++ i ) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF));
		}
		*RateNum = 4;
		break;

	case 0xCE0:
	case 0xEE0:
	case 0x18E0:
	case 0x1aE0:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS0);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS1);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS2);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS3);
		for ( i = 0; i < 4; ++ i ) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF)) * 10 +
					   ((Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xCE4:
	case 0xEE4:
	case 0x18E4:
	case 0x1aE4:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS4);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS5);
		rate_idx[2] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS6);
		rate_idx[3] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS7);
		for ( i = 0; i < 4; ++ i ) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF )) * 10 +
					   (( Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	case 0xCE8:
	case 0xEE8:
	case 0x18E8:
	case 0x1aE8:
		rate_idx[0] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS8);
		rate_idx[1] = PHY_GetRateIndexOfTxPowerByRate(MGN_VHT3SS_MCS9);
		for ( i = 0; i < 2; ++ i ) {
			PwrByRateVal[i] = (s8)
					  ((((Value >> (i * 8 + 4)) & 0xF )) * 10 +
					   (( Value >> (i * 8)) & 0xF ));
		}
		*RateNum = 4;
		break;

	default:
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD,
			 "Invalid RegAddr 0x%x in %s()\n", RegAddr, __FUNCTION__);
		break;
	};
}

void _rtl8821au_phy_init_tx_power(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	u8 band, rfpath, txnum = 0, rate_section;

	for (band = BAND_ON_2_4G; band <= BAND_ON_5G; ++band)
		for (rfpath = 0; rfpath < TX_PWR_BY_RATE_NUM_RF; ++rfpath)
			for (txnum = 0; txnum < TX_PWR_BY_RATE_NUM_RF; ++txnum)
				for (rate_section = 0;
				     rate_section < TX_PWR_BY_RATE_NUM_RATE;
				     ++rate_section)
					rtlphy->tx_power_by_rate_offset[band]
					   [rfpath][txnum][rate_section] = 0;

}

static void _rtl8821au_store_tx_power_by_rate(struct rtl_priv *rtlpriv, 
					      u32 band, u32 rfpath, u32 txnum,
					      u32 regaddr, u32 bitmask, u32 data)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);

	u8 i, rate_idx[4], rate_num = 0;
	s8 pwr_by_rate_val[4];

	_rtl8821au_get_values_of_txpwr_by_rate(rtlpriv, regaddr, bitmask,
					       data, rate_idx, 
					       pwr_by_rate_val,
					       &rate_num);

	if (band != BAND_ON_2_4G && band != BAND_ON_5G ) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid RFPath %d\n", band);
		return;
	}

	if (rfpath > RF90_PATH_D) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid RFPath %d\n", rfpath);
		return;
	}

	if (txnum > RF90_PATH_D) {
		RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Invalid RFPath %d\n", txnum);
		return;
	}

	for (i = 0; i < rate_num; ++i) {
		if (rate_idx[i] == PHY_GetRateIndexOfTxPowerByRate( MGN_VHT2SS_MCS0) ||
			 rate_idx[i] == PHY_GetRateIndexOfTxPowerByRate( MGN_VHT2SS_MCS1)) {
			txnum = RF_2TX;
		}
		rtlphy->tx_power_by_rate_offset[band][rfpath][txnum][rate_idx[i]] = 
				pwr_by_rate_val[i];
	}
}

bool _rtl8821au_phy_config_bb_with_pgheaderfile(struct rtl_priv *rtlpriv,
							u8 configtype)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

	uint32_t hex = 0;
	uint32_t i = 0;
	u16 count = 0;
	uint32_t    *ptr_array   = NULL;

	/* ULLI : fixed values ?? */
	u8  platform = ODM_CE;
	u8 _interface = RTW_USB;
	u8 board = rtlhal->board_type;

	uint32_t ArrayLen;
	uint32_t *Array;
	uint32_t v1 ,v2 ,v3 , v4, v5, v6;

	if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
		if (rtlhal->rfe_type == 3 && IS_NORMAL_CHIP(rtlhal->version)) {
			ArrayLen = RTL8812AU_PHY_REG_PG_ASUS_ARRAY_LEN;
			Array = RTL8812AU_PHY_REG_PG_ASUS_ARRAY;
		} else {
			ArrayLen = RTL8812AU_PHY_REG_PG_ARRAY_LEN;
			Array = RTL8812AU_PHY_REG_PG_ARRAY;
		}
	} else {
		ArrayLen = RTL8821AU_PHY_REG_PG_ARRAY_LEN;
		Array = RTL8821AU_PHY_REG_PG_ARRAY;
	}

	if (configtype != BASEBAND_CONFIG_PHY_REG) {
		RT_TRACE(rtlpriv, COMP_SEND, DBG_TRACE,
			 "configtype != BaseBand_Config_PHY_REG\n");
		return true;
	}
	
	for (i = 0; i < ArrayLen; i += 6 ) {
		v1 = Array[i];
		v2 = Array[i+1];
		v3 = Array[i+2];
		v4 = Array[i+3];
		v5 = Array[i+4];
		v6 = Array[i+5];

		if (v4 == 0xfe)
			mdelay(50);
		else if (v4 == 0xfd)
			mdelay(5);
		else if (v4 == 0xfc)
			mdelay(1);
		else if (v4 == 0xfb)
			udelay(50);
		else if (v4 == 0xfa)
			udelay(5);
		else if (v4 == 0xf9)
			udelay(1);

		_rtl8821au_store_tx_power_by_rate(rtlpriv, v1, v2, v3, v4, v5, v6);
	}
	return true;
}

/* **************** */

static void _rtl8821au_phy_init_txpower_limit(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	uint8_t		i, j, k, l, m;

	/* DBG_871X( "=====> PHY_InitPowerLimitTable()!\n" ); */

	for (i = 0; i < MAX_REGULATION_NUM; ++i) {
		for (j = 0; j < MAX_2_4G_BANDWITH_NUM; ++j)
			for (k = 0; k < MAX_2_4G_RATE_SECTION_NUM; ++k)
				for (m = 0; m < MAX_2_4G_CHANNEL_NUM; ++m)
					for (l = 0; l <  rtlpriv->phy.num_total_rfpath ;++l)
						rtlphy->txpwr_limit_2_4g[i][j][k][m][l] = MAX_POWER_INDEX;
	}

	for (i = 0; i < MAX_REGULATION_NUM; ++i) {
		for (j = 0; j < MAX_5G_BANDWITH_NUM; ++j)
			for (k = 0; k < MAX_5G_RATE_SECTION_NUM; ++k)
				for (m = 0; m < MAX_5G_CHANNEL_NUM; ++m)
					for (l = 0; l <   rtlpriv->phy.num_total_rfpath ; ++l)
						rtlphy->txpwr_limit_5g[i][j][k][m][l] = MAX_POWER_INDEX;
	}

	/* DBG_871X("<===== PHY_InitPowerLimitTable()!\n" ); */
}
static u8 _rtl8812au_phy_get_txpower_by_rate_base_index(struct rtl_priv *rtlpriv, enum band_type Band, uint8_t Rate)
{
	uint8_t	index = 0;
	if (Band == BAND_ON_2_4G) {
		switch (Rate) {
		case MGN_1M:
		case MGN_2M:
		case MGN_5_5M:
		case MGN_11M:
			index = 0;
			break;

		case MGN_6M:
		case MGN_9M:
		case MGN_12M:
		case MGN_18M:
		case MGN_24M:
		case MGN_36M:
		case MGN_48M:
		case MGN_54M:
			index = 1;
			break;

		case MGN_MCS0:
		case MGN_MCS1:
		case MGN_MCS2:
		case MGN_MCS3:
		case MGN_MCS4:
		case MGN_MCS5:
		case MGN_MCS6:
		case MGN_MCS7:
			index = 2;
			break;

		case MGN_MCS8:
		case MGN_MCS9:
		case MGN_MCS10:
		case MGN_MCS11:
		case MGN_MCS12:
		case MGN_MCS13:
		case MGN_MCS14:
		case MGN_MCS15:
			index = 3;
			break;

		default:
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Wrong rate 0x%x to obtain index in 2.4G in phy_getPowerByRateBaseIndex()\n", Rate );
			break;
		}
	} else if (Band == BAND_ON_5G) {
		switch (Rate) {
		case MGN_6M:
		case MGN_9M:
		case MGN_12M:
		case MGN_18M:
		case MGN_24M:
		case MGN_36M:
		case MGN_48M:
		case MGN_54M:
			index = 0;
			break;

		case MGN_MCS0:
		case MGN_MCS1:
		case MGN_MCS2:
		case MGN_MCS3:
		case MGN_MCS4:
		case MGN_MCS5:
		case MGN_MCS6:
		case MGN_MCS7:
			index = 1;
			break;

		case MGN_MCS8:
		case MGN_MCS9:
		case MGN_MCS10:
		case MGN_MCS11:
		case MGN_MCS12:
		case MGN_MCS13:
		case MGN_MCS14:
		case MGN_MCS15:
			index = 2;
			break;

		case MGN_VHT1SS_MCS0:
		case MGN_VHT1SS_MCS1:
		case MGN_VHT1SS_MCS2:
		case MGN_VHT1SS_MCS3:
		case MGN_VHT1SS_MCS4:
		case MGN_VHT1SS_MCS5:
		case MGN_VHT1SS_MCS6:
		case MGN_VHT1SS_MCS7:
		case MGN_VHT1SS_MCS8:
		case MGN_VHT1SS_MCS9:
			index = 3;
			break;

		case MGN_VHT2SS_MCS0:
		case MGN_VHT2SS_MCS1:
		case MGN_VHT2SS_MCS2:
		case MGN_VHT2SS_MCS3:
		case MGN_VHT2SS_MCS4:
		case MGN_VHT2SS_MCS5:
		case MGN_VHT2SS_MCS6:
		case MGN_VHT2SS_MCS7:
		case MGN_VHT2SS_MCS8:
		case MGN_VHT2SS_MCS9:
			index = 4;
			break;

		default:
			RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "Wrong rate 0x%x to obtain index in 5G in phy_getPowerByRateBaseIndex()\n", Rate );
			break;
		}
	}

	return index;
}

void _rtl8812au_phy_cross_reference_ht_and_vht_txpower_limit(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	u8 regulation, bw, channel, rateSection;
	s8 tempPwrLmt = 0;
	
	for (regulation = 0; regulation < MAX_REGULATION_NUM; ++regulation) {
		for (bw = 0; bw < MAX_5G_BANDWITH_NUM; ++bw) {
			for (channel = 0; channel < CHANNEL_MAX_NUMBER_5G; ++channel) {
				for (rateSection = 0; rateSection < MAX_RATE_SECTION_NUM; ++rateSection) {
					tempPwrLmt = rtlphy->txpwr_limit_5g[regulation][bw][rateSection][channel][RF90_PATH_A];
					if (tempPwrLmt == MAX_POWER_INDEX ) {
						u8	baseSection = 2, refSection = 6;
						if (bw == 0 || bw == 1) { // 5G 20M 40M VHT and HT can cross reference
							//DBG_871X("No power limit table of the specified band %d, bandwidth %d, ratesection %d, channel %d, rf path %d\n",
							//			1, bw, rateSection, channel, ODM_RF_PATH_A );
							if (rateSection >= 2 && rateSection <= 9) {
								if (rateSection == 2) {
									baseSection = 2;
									refSection = 6;
								} else if (rateSection == 3) {
									baseSection = 3;
									refSection = 7;
								} else if (rateSection == 4) {
									baseSection = 4;
									refSection = 8;
								} else if (rateSection == 5) {
									baseSection = 5;
									refSection = 9;
								} else if (rateSection == 6) {
									baseSection = 6;
									refSection = 2;
								} else if (rateSection == 7) {
									baseSection = 7;
									refSection = 3;
								} else if (rateSection == 8) {
									baseSection = 8;
									refSection = 4;
								} else if (rateSection == 9) {
									baseSection = 9;
									refSection = 5;
								}
								rtlphy->txpwr_limit_5g[regulation][bw][baseSection][channel][RF90_PATH_A] = 
									rtlphy->txpwr_limit_5g[regulation][bw][refSection][channel][RF90_PATH_A];
							}

							//DBG_871X("use other value %d", tempPwrLmt );
						}
					}
				}
			}
		}
	}
}

static void _rtl8821au_phy_convert_txpower_limit_to_power_index(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = rtl_phy(rtlpriv);
	u8 BW40PwrBasedBm2_4G = 0x2E, BW40PwrBasedBm5G = 0x2E;
	u8 regulation, bw, channel, rateSection;	
	u8 baseIndex2_4G;
	u8 baseIndex5G;
	s8 tempValue = 0, tempPwrLmt = 0;
	u8 rf_path = 0;

	//DBG_871X("=====> PHY_ConvertTxPowerLimitToPowerIndex()\n" );

	_rtl8812au_phy_cross_reference_ht_and_vht_txpower_limit(rtlpriv);

	for (regulation = 0; regulation < MAX_REGULATION_NUM; ++regulation) {
		for (bw = 0; bw < MAX_2_4G_BANDWITH_NUM; ++bw) {
			for (channel = 0; channel < CHANNEL_MAX_NUMBER_2G; ++channel) {
				for (rateSection = 0; rateSection < MAX_RATE_SECTION_NUM; ++rateSection) {
					tempPwrLmt = rtlphy->txpwr_limit_2_4g[regulation][bw][rateSection][channel][RF90_PATH_A];

					for (rf_path = RF90_PATH_A; rf_path < MAX_RF_PATH_NUM; ++rf_path) {
						if (rateSection == 5) // HT 4T
							BW40PwrBasedBm2_4G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_4TX, HT_MCS24_MCS31 );
						else if (rateSection == 4) // HT 3T
							BW40PwrBasedBm2_4G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_3TX, HT_MCS16_MCS23 );
						else if (rateSection == 3) // HT 2T
							BW40PwrBasedBm2_4G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_2TX, HT_MCS8_MCS15 );
						else if (rateSection == 2) // HT 1T
							BW40PwrBasedBm2_4G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_1TX, HT_MCS0_MCS7 );
						else if (rateSection == 1) // OFDM
							BW40PwrBasedBm2_4G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_1TX, OFDM );
						else if (rateSection == 0 ) // CCK
							BW40PwrBasedBm2_4G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_1TX, CCK );

						if (tempPwrLmt != MAX_POWER_INDEX) {
							tempValue = tempPwrLmt - BW40PwrBasedBm2_4G;
							rtlphy->txpwr_limit_2_4g[regulation][bw][rateSection][channel][rf_path] = tempValue;
						}
					}
				}
			}
		}
	}
	
	for (regulation = 0; regulation < MAX_REGULATION_NUM; ++regulation) {
		for (bw = 0; bw < MAX_5G_BANDWITH_NUM; ++bw) {
			for (channel = 0; channel < CHANNEL_MAX_NUMBER_5G; ++channel) {
				for (rateSection = 0; rateSection < MAX_RATE_SECTION_NUM; ++rateSection) {	
					tempPwrLmt = rtlphy->txpwr_limit_5g[regulation][bw][rateSection][channel][RF90_PATH_A];

					for (rf_path = RF90_PATH_A; rf_path < MAX_RF_PATH_NUM; ++rf_path) {
						if ( rateSection == 9 ) // VHT 4SS
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_4TX, VHT_4SSMCS0_4SSMCS9);
						else if ( rateSection == 8 ) // VHT 3SS
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_3TX, VHT_3SSMCS0_3SSMCS9 );
						else if ( rateSection == 7 ) // VHT 2SS
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_2TX, VHT_2SSMCS0_2SSMCS9 );
						else if ( rateSection == 6 ) // VHT 1SS
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_1TX, VHT_1SSMCS0_1SSMCS9 );
						else if ( rateSection == 5 ) // HT 4T
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_4TX, HT_MCS24_MCS31 );
						else if ( rateSection == 4 ) // HT 3T
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_3TX, HT_MCS16_MCS23 );
						else if ( rateSection == 3 ) // HT 2T
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_2TX, HT_MCS8_MCS15 );
						else if ( rateSection == 2 ) // HT 1T
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_1TX, HT_MCS0_MCS7 );
						else if ( rateSection == 1 ) // OFDM 
							BW40PwrBasedBm5G = _rtl8821au_phy_get_txpower_by_rate_base(rtlpriv, BAND_ON_2_4G, rf_path, RF_1TX, OFDM );

						if (tempPwrLmt != MAX_POWER_INDEX) {
							tempValue = tempPwrLmt - BW40PwrBasedBm5G;
							rtlphy->txpwr_limit_5g[regulation][bw][rateSection][channel][rf_path] = tempValue;
						}
					}
				}
			}
		}
	}
	//DBG_871X("<===== PHY_ConvertTxPowerLimitToPowerIndex()\n" );
}

static void phy_InitBBRFRegisterDefinition(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = &(rtlpriv->phy);

	/* RF Interface Sowrtware Control */
	rtlphy->phyreg_def[RF90_PATH_A].rfintfs = rFPGA0_XAB_RFInterfaceSW;	/* 16 LSBs if read 32-bit from 0x870 */
	rtlphy->phyreg_def[RF90_PATH_B].rfintfs = rFPGA0_XAB_RFInterfaceSW;	/* 16 MSBs if read 32-bit from 0x870 (16-bit for 0x872) */

	/* RF Interface Output (and Enable) */
	rtlphy->phyreg_def[RF90_PATH_A].rfintfo = rFPGA0_XA_RFInterfaceOE;	/* 16 LSBs if read 32-bit from 0x860 */
	rtlphy->phyreg_def[RF90_PATH_B].rfintfo = rFPGA0_XB_RFInterfaceOE;	/* 16 LSBs if read 32-bit from 0x864 */

	/* RF Interface (Output and)  Enable */
	rtlphy->phyreg_def[RF90_PATH_A].rfintfe = rFPGA0_XA_RFInterfaceOE;	/* 16 MSBs if read 32-bit from 0x860 (16-bit for 0x862) */
	rtlphy->phyreg_def[RF90_PATH_B].rfintfe = rFPGA0_XB_RFInterfaceOE; 	/* 16 MSBs if read 32-bit from 0x864 (16-bit for 0x866) */

	rtlphy->phyreg_def[RF90_PATH_A].rf3wire_offset = rA_LSSIWrite_Jaguar; 	/* LSSI Parameter */
	rtlphy->phyreg_def[RF90_PATH_B].rf3wire_offset = rB_LSSIWrite_Jaguar;

	rtlphy->phyreg_def[RF90_PATH_A].rfhssi_para2 = rHSSIRead_Jaguar;		/* wire control parameter2 */
	rtlphy->phyreg_def[RF90_PATH_B].rfhssi_para2 = rHSSIRead_Jaguar;		/* wire control parameter2 */

	/* Tranceiver Readback LSSI/HSPI mode */
	rtlphy->phyreg_def[RF90_PATH_A].rf_rb = rA_SIRead_Jaguar;
	rtlphy->phyreg_def[RF90_PATH_B].rf_rb = rB_SIRead_Jaguar;
	rtlphy->phyreg_def[RF90_PATH_A].rf_rbpi = rA_PIRead_Jaguar;
	rtlphy->phyreg_def[RF90_PATH_B].rf_rbpi = rB_PIRead_Jaguar;

	/* pHalData->bPhyValueInitReady=true; */
}


/* ULLI : check with _rtl8821ae_phy_config_bb_with_headerfile () */

bool _rtl8821au_phy_config_bb_with_headerfile(struct rtl_priv *rtlpriv,
						       u8 configtype)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

	#define READ_NEXT_PAIR(v1, v2, i) do { i += 2; v1 = Array[i]; v2 = Array[i+1]; } while(0)

	uint32_t     hex         = 0;
	uint32_t     i           = 0;

	/* ULLI : fixed values ?? */
	u8  platform = ODM_CE;
	u8 _interface = RTW_USB;
	u8 board = rtlhal->board_type;

	uint32_t ArrayLen;
	uint32_t *Array;

	if (configtype == BASEBAND_CONFIG_PHY_REG) {
		if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
			ArrayLen = RTL8812AU_PHY_REG_ARRAY_LEN;
			Array = RTL8812AU_PHY_REG_ARRAY;
		} else {
			ArrayLen = RTL8821AU_PHY_REG_ARRAY_LEN;
			Array = RTL8821AU_PHY_REG_ARRAY;
		}

		hex += board;
		hex += _interface << 8;
		hex += platform << 16;
		hex += 0xFF000000;
		RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ReadAndConfig_MP_8821A_PHY_REG, hex = 0x%X\n", hex);

		for (i = 0; i < ArrayLen; i += 2) {
			uint32_t v1 = Array[i];
			uint32_t v2 = Array[i+1];

			// This (offset, data) pair meets the condition.
			if (v1 < 0xCDCDCDCD) {
				_rtl8821au_config_bb_reg(rtlpriv, v1, bMaskDWord, v2);
				continue;
			} else {
				// This line is the start line of branch.
				if (!CheckCondition(Array[i], hex)) {
					// Discard the following (offset, data) pairs.
					READ_NEXT_PAIR(v1, v2, i);
				        while (v2 != 0xDEAD && v2 != 0xCDEF &&
				               v2 != 0xCDCD && i < ArrayLen -2) {
							READ_NEXT_PAIR(v1, v2, i);
					}
					i -= 2; // prevent from for-loop += 2
				} else {
					// Configure matched pairs and skip to end of if-else.
				        READ_NEXT_PAIR(v1, v2, i);
				        while (v2 != 0xDEAD && v2 != 0xCDEF &&
				               v2 != 0xCDCD && i < ArrayLen -2) {
							_rtl8821au_config_bb_reg(rtlpriv, v1, bMaskDWord, v2);
							READ_NEXT_PAIR(v1, v2, i);
				        }

				        while (v2 != 0xDEAD && i < ArrayLen -2) {
						READ_NEXT_PAIR(v1, v2, i);
				        }
				}
			}
		}
	} else if (configtype == BASEBAND_CONFIG_AGC_TAB) {
		struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);

		uint32_t hex = 0;
		uint32_t i = 0;

		/* ULLI : fixed values ?? */
		u8  platform = ODM_CE;
		u8 _interface = RTW_USB;
		u8 board = rtlhal->board_type;

		uint32_t ArrayLen;
		uint32_t *Array;

		if (IS_HARDWARE_TYPE_8812AU(rtlhal)) {
			ArrayLen = RTL8812AU_AGC_TAB_ARRAY_LEN;
			Array = RTL8812AU_AGC_TAB_ARRAY;
		} else {
			ArrayLen = RTL8821AU_AGC_TAB_ARRAY_LEN;
			Array = RTL8821AU_AGC_TAB_ARRAY;
		}

		hex += board;
		hex += _interface << 8;
		hex += platform << 16;
		hex += 0xFF000000;

		RT_TRACE(rtlpriv, COMP_INIT, DBG_TRACE, "===> ODM_ReadAndConfig_MP_8821A_AGC_TAB, hex = 0x%X\n", hex);

		for (i = 0; i < ArrayLen; i += 2) {
			uint32_t v1 = Array[i];
			uint32_t v2 = Array[i+1];

			// This (offset, data) pair meets the condition.
			if (v1 < 0xCDCDCDCD ) {
				rtl_set_bbreg(rtlpriv, v1, bMaskDWord, v2);
				/* Add 1us delay between BB/RF register setting. */
				udelay(1);

				continue;
			} else {
				// This line is the start line of branch.
				if (!CheckCondition(Array[i], hex)) {
					// Discard the following (offset, data) pairs.
					READ_NEXT_PAIR(v1, v2, i);
					while (v2 != 0xDEAD && v2 != 0xCDEF &&
						v2 != 0xCDCD && i < ArrayLen -2) {
							READ_NEXT_PAIR(v1, v2, i);
					}
					i -= 2; // prevent from for-loop += 2
				} else {
					// Configure matched pairs and skip to end of if-else.
					READ_NEXT_PAIR(v1, v2, i);
					while (v2 != 0xDEAD && v2 != 0xCDEF &&
						v2 != 0xCDCD && i < ArrayLen -2) {
							rtl_set_bbreg(rtlpriv, v1, bMaskDWord, v2);
							/* Add 1us delay between BB/RF register setting. */
							udelay(1);

							READ_NEXT_PAIR(v1, v2, i);
					}

					while (v2 != 0xDEAD && i < ArrayLen -2) {
						READ_NEXT_PAIR(v1, v2, i);
					}
				}
			}
		}
	}

	return true;
}

static int _rtl8821au_phy_bb_with_headerfile(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = &(rtlpriv->phy);
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);
	int			rtStatus = _SUCCESS;

	/* DBG_871X("==>phy_BB8812_Config_ParaFile\n"); */

	RT_TRACE(rtlpriv, COMP_POWER, DBG_LOUD, "===> phy_BB8812_Config_ParaFile() EEPROMRegulatory %d\n", efuse->eeprom_regulatory);

	_rtl8821au_phy_init_txpower_limit(rtlpriv);

	if (efuse->eeprom_regulatory == 1) {
		_rtl8821au_phy_read_and_config_txpwr_lmt(rtlpriv);
	}

	/* Read PHY_REG.TXT BB INIT!! */
	rtlpriv->cfg->ops->config_bb_with_headerfile(rtlpriv,
						     BASEBAND_CONFIG_PHY_REG);

	/* If EEPROM or EFUSE autoload OK, We must config by PHY_REG_PG.txt */
	/* 1 TODO */
	if (efuse->autoload_failflag == false) {
		rtlphy->pwrgroup_cnt = 0;

		rtlpriv->cfg->ops->config_bb_with_pgheaderfile(rtlpriv,
							       BASEBAND_CONFIG_PHY_REG);

		if (efuse->eeprom_regulatory == 1 )
			_rtl8821au_phy_convert_txpower_limit_to_power_index(rtlpriv);
	}


	/* BB AGC table Initialization */
	rtlpriv->cfg->ops->config_bb_with_headerfile(rtlpriv,
						     BASEBAND_CONFIG_AGC_TAB);
	return rtStatus;
}

int rtl8821au_phy_bb_config(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct rtl_efuse *rtlefuse = rtl_efuse(rtlpriv);
	int	rtStatus = _SUCCESS;
	uint8_t	TmpU1B=0;
	uint8_t	crystal_cap;

	phy_InitBBRFRegisterDefinition(rtlpriv);

    	/* tangw check start 20120412 */
	/* . APLL_EN,,APLL_320_GATEB,APLL_320BIAS,  auto config by hw fsm after pfsm_go (0x4 bit 8) set */
	TmpU1B = rtl_read_byte(rtlpriv, REG_SYS_FUNC_EN);

	/* ULLI some PCIe code ?? */

	TmpU1B |= FEN_USBA;

	rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN, TmpU1B);

	rtl_write_byte(rtlpriv, REG_SYS_FUNC_EN, (TmpU1B|FEN_BB_GLB_RSTn|FEN_BBRSTB));	/* same with 8812 */
	/* 6. 0x1f[7:0] = 0x07 PathA RF Power On */
	rtl_write_byte(rtlpriv, REG_RF_CTRL, 0x07);		/* RF_SDMRSTB,RF_RSTB,RF_EN same with 8723a */
	/* 7.  PathB RF Power On */
	rtl_write_byte(rtlpriv, REG_OPT_CTRL_8812+2, 0x7);	/* RF_SDMRSTB,RF_RSTB,RF_EN same with 8723a */
	/* tangw check end 20120412 */

	/*
	 * Config BB and AGC
	 */
	rtStatus = _rtl8821au_phy_bb_with_headerfile(rtlpriv);

	if (IS_HARDWARE_TYPE_8812(rtlhal)) {
		/* write 0x2C[30:25] = 0x2C[24:19] = CrystalCap */
		crystal_cap = rtlefuse->crystalcap & 0x3F;
		rtl_set_bbreg(rtlpriv, REG_MAC_PHY_CTRL, 0x7FF80000, (crystal_cap | (crystal_cap << 6)));
	} else if (IS_HARDWARE_TYPE_8821(rtlhal)) {
		/* 0x2C[23:18] = 0x2C[17:12] = CrystalCap */
		crystal_cap = rtlefuse->crystalcap & 0x3F;
		rtl_set_bbreg(rtlpriv, REG_MAC_PHY_CTRL, 0xFFF000, (crystal_cap | (crystal_cap << 6)));
	}

	rtlpriv->phy.reg_837 = rtl_read_byte(rtlpriv, 0x837);

	return rtStatus;

}

void rtl8821au_phy_sw_chnl_callback(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	uint8_t	path = 0;
	uint8_t	channel = rtlpriv->phy.current_channel;
	u32 data;

	if(phy_SwBand8812(rtlpriv, channel) == false)
		RT_TRACE(rtlpriv, COMP_ERR, DBG_LOUD, "error Chnl %d !\n", channel);

	/* DBG_871X("[BW:CHNL], phy_SwChnl8812(), switch to channel %d !!\n", channel); */

	/* fc_area */
	if (36 <= channel && channel <= 48)
		data = 0x494;
	else if (50 <= channel && channel <= 64)
		data = 0x453;
	else if (100 <= channel && channel <= 116)
		data = 0x452;
	else if (118 <= channel)
		data = 0x412;
	else
		data = 0x96a;

	rtl_set_bbreg(rtlpriv, rFc_area_Jaguar, 0x1ffe0000, data);

	for (path = 0; path <  rtlpriv->phy.num_total_rfpath; path++) {
		/* [2.4G] LC Tank */
		if (IS_VENDOR_8812A_TEST_CHIP(rtlhal->version)) {
			if (1 <= channel && channel <= 7)
				rtl_set_rfreg(rtlpriv, path, RF_TxLCTank_Jaguar, bLSSIWrite_data_Jaguar, 0x0017e);
			else if (8 <= channel && channel <= 14)
				rtl_set_rfreg(rtlpriv, path, RF_TxLCTank_Jaguar, bLSSIWrite_data_Jaguar, 0x0013e);
		}

		/* RF_MOD_AG */
		if (36 <= channel && channel <= 64)
			data = 0x101; //5'b00101);
		else if (100 <= channel && channel <= 140)
			data = 0x301; //5'b01101);
		else if (140 < channel)
			data = 0x501; //5'b10101);
		else
			data = 0x000; //5'b00000);

		rtl_set_rfreg(rtlpriv, path, RF_CHNLBW_Jaguar,
			      BIT(18)|BIT(17)|BIT(16)|BIT(9)|BIT(8), data);

		/* <20121109, Kordan> A workaround for 8812A only. */
		rtl8812au_fixspur(rtlpriv, rtlpriv->phy.current_chan_bw, channel);

		rtl_set_rfreg(rtlpriv, path, RF_CHNLBW_Jaguar, MASKBYTE0, channel);

		/* <20130104, Kordan> APK for MP chip is done on initialization from folder. */
		if (IS_HARDWARE_TYPE_8811AU(rtlhal) &&
		    (!IS_NORMAL_CHIP(rtlhal->version)) && channel > 14 ) {
			/* <20121116, Kordan> For better result of APK. Asked by AlexWang. */
			if (36 <= channel && channel <= 64)
				rtl_set_rfreg(rtlpriv, path, RF_APK_Jaguar, bRFRegOffsetMask, 0x710E7);
			else if (100 <= channel && channel <= 140)
				rtl_set_rfreg(rtlpriv, path, RF_APK_Jaguar, bRFRegOffsetMask, 0x716E9);
			else
				rtl_set_rfreg(rtlpriv, path, RF_APK_Jaguar, bRFRegOffsetMask, 0x714E9);
		}
	}
}

