#ifndef __RTL8821AU_DEF_H__
#define __RTL8821AU_DEF_H__

#include <linux/types.h>
#include <linux/bitops.h>

enum MGN_RATE{
	MGN_1M		= 0x02,
	MGN_2M		= 0x04,
	MGN_5_5M 	= 0x0B,
	MGN_6M	 	= 0x0C,
	MGN_9M		= 0x12,
	MGN_11M 	= 0x16,
	MGN_12M	= 0x18,
	MGN_18M	= 0x24,
	MGN_24M	= 0x30,
	MGN_36M	= 0x48,
	MGN_48M	= 0x60,
	MGN_54M	= 0x6C,
	MGN_MCS32	= 0x7F,
	MGN_MCS0,
	MGN_MCS1,
	MGN_MCS2,
	MGN_MCS3,
	MGN_MCS4,
	MGN_MCS5,
	MGN_MCS6,
	MGN_MCS7,
	MGN_MCS8,
	MGN_MCS9,
	MGN_MCS10,
	MGN_MCS11,
	MGN_MCS12,
	MGN_MCS13,
	MGN_MCS14,
	MGN_MCS15,
	MGN_MCS16,
	MGN_MCS17,
	MGN_MCS18,
	MGN_MCS19,
	MGN_MCS20,
	MGN_MCS21,
	MGN_MCS22,
	MGN_MCS23,
	MGN_MCS24,
	MGN_MCS25,
	MGN_MCS26,
	MGN_MCS27,
	MGN_MCS28,
	MGN_MCS29,
	MGN_MCS30,
	MGN_MCS31,
	MGN_VHT1SS_MCS0,
	MGN_VHT1SS_MCS1,
	MGN_VHT1SS_MCS2,
	MGN_VHT1SS_MCS3,
	MGN_VHT1SS_MCS4,
	MGN_VHT1SS_MCS5,
	MGN_VHT1SS_MCS6,
	MGN_VHT1SS_MCS7,
	MGN_VHT1SS_MCS8,
	MGN_VHT1SS_MCS9,
	MGN_VHT2SS_MCS0,
	MGN_VHT2SS_MCS1,
	MGN_VHT2SS_MCS2,
	MGN_VHT2SS_MCS3,
	MGN_VHT2SS_MCS4,
	MGN_VHT2SS_MCS5,
	MGN_VHT2SS_MCS6,
	MGN_VHT2SS_MCS7,
	MGN_VHT2SS_MCS8,
	MGN_VHT2SS_MCS9,
	MGN_VHT3SS_MCS0,
	MGN_VHT3SS_MCS1,
	MGN_VHT3SS_MCS2,
	MGN_VHT3SS_MCS3,
	MGN_VHT3SS_MCS4,
	MGN_VHT3SS_MCS5,
	MGN_VHT3SS_MCS6,
	MGN_VHT3SS_MCS7,
	MGN_VHT3SS_MCS8,
	MGN_VHT3SS_MCS9,
	MGN_VHT4SS_MCS0,
	MGN_VHT4SS_MCS1,
	MGN_VHT4SS_MCS2,
	MGN_VHT4SS_MCS3,
	MGN_VHT4SS_MCS4,
	MGN_VHT4SS_MCS5,
	MGN_VHT4SS_MCS6,
	MGN_VHT4SS_MCS7,
	MGN_VHT4SS_MCS8,
	MGN_VHT4SS_MCS9,
	MGN_UNKNOWN
};

enum rtl_desc_qsel {
	QSLT_BK = 0x2,
	QSLT_BE = 0x0,
	QSLT_VI = 0x5,
	QSLT_VO = 0x7,
	QSLT_BEACON = 0x10,
	QSLT_HIGH = 0x11,
	QSLT_MGNT = 0x12,
	QSLT_CMD = 0x13,
};

//VERSION_8192C			VersionID;
//HAL_VERSION			VersionID;

enum version_8821au {
	CHIP_8812 =	BIT(2),
	CHIP_8821 = 	(BIT(0)|BIT(2)),
	NORMAL_CHIP =	BIT(3),
	RF_TYPE_1T1R =	(~(BIT(4)|BIT(5)|BIT(6))),
	RF_TYPE_1T2R =	BIT(4),
	RF_TYPE_2T2R =	BIT(5),
	CHIP_VENDOR_UMC = BIT(7),
	B_CUT_VERSION =	BIT(12),
	C_CUT_VERSION = BIT(13),
	D_CUT_VERSION =	((BIT(12)|BIT(13))),
	E_CUT_VERSION = BIT(14),

	/* ULLI : our own bitfield, form pHalData->bSupportUSB3 */

	RTL8821AU_USB3_MODE = BIT(15),
	
	VERSION_UNKNOWN = 0x0,
};

#define IF_RTL8821AU_USB3_MODE(version)		((version) & RTL8821AU_USB3_MODE)


/* MASK */
#define IC_TYPE_MASK			(BIT(0)|BIT(1)|BIT(2))
#define CHIP_TYPE_MASK			BIT(3)
#define RF_TYPE_MASK			(BIT(4)|BIT(5)|BIT(6))
#define MANUFACTUER_MASK		BIT(7)
#define CUT_VERSION_MASK		(BIT(15)|BIT(14)|BIT(13)|BIT(12))


// Get element
#define GET_CVID_IC_TYPE(version)	((version) & IC_TYPE_MASK)
#define GET_CVID_CHIP_TYPE(version)	((version) & CHIP_TYPE_MASK)
#define GET_CVID_RF_TYPE(version)	((version) & RF_TYPE_MASK)
#define GET_CVID_MANUFACTUER(version)	((version) & MANUFACTUER_MASK)
#define GET_CVID_CUT_VERSION(version)	((version) & CUT_VERSION_MASK)
#define NUM_CUT_VERSION(version)	(GET_CVID_CUT_VERSION(version) >> 12)
#define GET_CVID_ROM_VERSION(version)	(((version)) & ROM_VERSION_MASK)

//----------------------------------------------------------------------------
//Common Macro. --
//----------------------------------------------------------------------------
//HAL_VERSION VersionID

// HAL_IC_TYPE_E
#define IS_8812_SERIES(version)			((GET_CVID_IC_TYPE(version) == CHIP_8812)? true : false)
#define IS_8821_SERIES(version)			((GET_CVID_IC_TYPE(version) == CHIP_8821)? true : false)
#define IS_NORMAL_CHIP(version)			((GET_CVID_CHIP_TYPE(version))? true: false)

//HAL_CUT_VERSION_E
#define IS_B_CUT(version)				((GET_CVID_CUT_VERSION(version) == B_CUT_VERSION) ? true : false)

//HAL_VENDOR_E
#define IS_CHIP_VENDOR_TSMC(version)	((GET_CVID_MANUFACTUER(version) == CHIP_VENDOR_TSMC)? true: false)
#define IS_CHIP_VENDOR_UMC(version)	((GET_CVID_MANUFACTUER(version) == CHIP_VENDOR_UMC)? true: false)
#define IS_CHIP_VENDOR_SMIC(version)	((GET_CVID_MANUFACTUER(version) == CHIP_VENDOR_SMIC)? true: false)

//HAL_RF_TYPE_E
#define IS_1T1R(version)					((GET_CVID_RF_TYPE(version) == RF_TYPE_1T1R)? true : false )
#define IS_1T2R(version)					((GET_CVID_RF_TYPE(version) == RF_TYPE_1T2R)? true : false )
#define IS_2T2R(version)					((GET_CVID_RF_TYPE(version) == RF_TYPE_2T2R)? true : false )


//----------------------------------------------------------------------------
//Chip version Macro. --
//----------------------------------------------------------------------------
#define IS_81XXC_TEST_CHIP(version)		((IS_81XXC(version) && (!IS_NORMAL_CHIP(version)))? true: false)

#define IS_92C_SERIAL(version)   					((IS_81XXC(version) && IS_2T2R(version)) ? true : false)
#define IS_81xxC_VENDOR_UMC_B_CUT(version)	(IS_81XXC(version)?(IS_CHIP_VENDOR_UMC(version) ? (IS_B_CUT(version) ? true : false) : false): false)
#define IS_81xxC_VENDOR_UMC_C_CUT(version)	(IS_81XXC(version)?(IS_CHIP_VENDOR_UMC(version) ? (IS_C_CUT(version) ? true : false) : false): false)

#define IS_VENDOR_8812A_TEST_CHIP(version)		((IS_8812_SERIES(version)) ? ((IS_NORMAL_CHIP(version)) ? false : true) : false)
#define IS_VENDOR_8812A_MP_CHIP(version)		((IS_8812_SERIES(version)) ? ((IS_NORMAL_CHIP(version)) ? true : false) : false)
#define IS_VENDOR_8812A_C_CUT(version)			((IS_8812_SERIES(version)) ? ((GET_CVID_CUT_VERSION(version) == C_CUT_VERSION) ? true : false) : false)

#define IS_VENDOR_8821A_TEST_CHIP(version)	((IS_8821_SERIES(version)) ? ((IS_NORMAL_CHIP(version) ? false : true) : false)
#define IS_VENDOR_8821A_MP_CHIP(version)		((IS_8821_SERIES(version)) ? ((IS_NORMAL_CHIP(version)) ? true : false) : false)

#define MAX_RX_DMA_BUFFER_SIZE_8821                    0x3E80  // RX 16K


#endif
