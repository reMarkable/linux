#ifndef __RTL8821AU_FW_H__
#define __RTL8821AU_FW_H__

#include <rtl8812a_hal.h>
#include "../wifi.h"


//_RSVDPAGE_LOC_CMD0
#define SET_8812_H2CCMD_RSVDPAGE_LOC_PROBE_RSP(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE(__pH2CCmd, 0, 8, __Value)
#define SET_8812_H2CCMD_RSVDPAGE_LOC_PSPOLL(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE((__pH2CCmd)+1, 0, 8, __Value)
#define SET_8812_H2CCMD_RSVDPAGE_LOC_NULL_DATA(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE((__pH2CCmd)+2, 0, 8, __Value)
#define SET_8812_H2CCMD_RSVDPAGE_LOC_QOS_NULL_DATA(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE((__pH2CCmd)+3, 0, 8, __Value)
#define SET_8812_H2CCMD_RSVDPAGE_LOC_BT_QOS_NULL_DATA(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE((__pH2CCmd)+4, 0, 8, __Value)

//_MEDIA_STATUS_RPT_PARM_CMD1
#define SET_8812_H2CCMD_MSRRPT_PARM_OPMODE(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE(__pH2CCmd, 0, 1, __Value)
#define SET_8812_H2CCMD_MSRRPT_PARM_MACID_IND(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE(__pH2CCmd, 1, 1, __Value)
#define SET_8812_H2CCMD_MSRRPT_PARM_MACID(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd+1, 0, 8, __Value)
#define SET_8812_H2CCMD_MSRRPT_PARM_MACID_END(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd+2, 0, 8, __Value)

//_SETPWRMODE_PARM
#define SET_8812_H2CCMD_PWRMODE_PARM_MODE(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd, 0, 8, __Value)
#define SET_8812_H2CCMD_PWRMODE_PARM_RLBM(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE((__pH2CCmd)+1, 0, 4, __Value)
#define SET_8812_H2CCMD_PWRMODE_PARM_SMART_PS(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE((__pH2CCmd)+1, 4, 4, __Value)
#define SET_8812_H2CCMD_PWRMODE_PARM_BCN_PASS_TIME(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE((__pH2CCmd)+2, 0, 8, __Value)
#define SET_8812_H2CCMD_PWRMODE_PARM_ALL_QUEUE_UAPSD(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE((__pH2CCmd)+3, 0, 8, __Value)
#define SET_8812_H2CCMD_PWRMODE_PARM_PWR_STATE(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE((__pH2CCmd)+4, 0, 8, __Value)

#define GET_8812_H2CCMD_PWRMODE_PARM_MODE(__pH2CCmd)					LE_BITS_TO_1BYTE(__pH2CCmd, 0, 8)

//_P2P_PS_OFFLOAD
#define SET_8812_H2CCMD_P2P_PS_OFFLOAD_ENABLE(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd, 0, 1, __Value)
#define SET_8812_H2CCMD_P2P_PS_OFFLOAD_ROLE(__pH2CCmd, __Value)			SET_BITS_TO_LE_1BYTE(__pH2CCmd, 1, 1, __Value)
#define SET_8812_H2CCMD_P2P_PS_OFFLOAD_CTWINDOW_EN(__pH2CCmd, __Value)		SET_BITS_TO_LE_1BYTE(__pH2CCmd, 2, 1, __Value)
#define SET_8812_H2CCMD_P2P_PS_OFFLOAD_NOA0_EN(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE(__pH2CCmd, 3, 1, __Value)
#define SET_8812_H2CCMD_P2P_PS_OFFLOAD_NOA1_EN(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE(__pH2CCmd, 4, 1, __Value)
#define SET_8812_H2CCMD_P2P_PS_OFFLOAD_ALLSTASLEEP(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE(__pH2CCmd, 5, 1, __Value)
#define SET_8812_H2CCMD_P2P_PS_OFFLOAD_DISCOVERY(__pH2CCmd, __Value)	SET_BITS_TO_LE_1BYTE(__pH2CCmd, 6, 1, __Value)


void rtl8812au_set_fw_pwrmode_cmd(struct rtl_priv *rtlpriv, uint8_t PSMode);
int32_t rtl8821au_download_fw(struct rtl_priv *rtlpriv, bool bUsedWoWLANFw);
void rtl8821au_firmware_selfreset(struct rtl_priv *rtlpriv);
void rtl8821au_fill_h2c_cmd(struct rtl_priv *rtlpriv, u8 element_id, 
			   u32 cmd_len, u8 *cmdbuffer);
uint8_t rtl8812_set_rssi_cmd(struct rtl_priv *rtlpriv, uint8_t *param);
void rtl8821au_set_fw_joinbss_report_cmd(struct rtl_priv *rtlpriv, uint8_t mstatus);

#define FW_8821AU_START_ADDRESS    	0x1000
#define FW_8821AU_END_ADDRESS         	0x5FFF
#define FW_SIZE_8812			0x8000 // Compatible with RTL8723 Maximal RAM code size 24K.   modified to 32k, TO compatible with 92d maximal fw size 32k

//
// This structure must be cared byte-ordering
//
// Added by tynli. 2009.12.04.
#define IS_FW_HEADER_EXIST_8812(_pFwHdr)	((GET_FIRMWARE_HDR_SIGNATURE_8812(_pFwHdr) &0xFFF0) ==  0x9500)

#define IS_FW_HEADER_EXIST_8821(_pFwHdr)	((GET_FIRMWARE_HDR_SIGNATURE_8812(_pFwHdr) &0xFFF0) ==  0x2100)
//=====================================================
//					Firmware Header(8-byte alinment required)
//=====================================================
//--- LONG WORD 0 ----
#define GET_FIRMWARE_HDR_SIGNATURE_8812(__FwHdr)		LE_BITS_TO_4BYTE(__FwHdr, 0, 16) // 92C0: test chip; 92C, 88C0: test chip; 88C1: MP A-cut; 92C1: MP A-cut
#define GET_FIRMWARE_HDR_CATEGORY_8812(__FwHdr)		LE_BITS_TO_4BYTE(__FwHdr, 16, 8) // AP/NIC and USB/PCI
#define GET_FIRMWARE_HDR_FUNCTION_8812(__FwHdr)		LE_BITS_TO_4BYTE(__FwHdr, 24, 8) // Reserved for different FW function indcation, for further use when driver needs to download different FW in different conditions
#define GET_FIRMWARE_HDR_VERSION_8812(__FwHdr)		LE_BITS_TO_4BYTE(__FwHdr+4, 0, 16)// FW Version
#define GET_FIRMWARE_HDR_SUB_VER_8812(__FwHdr)		LE_BITS_TO_4BYTE(__FwHdr+4, 16, 8) // FW Subversion, default 0x00
#define GET_FIRMWARE_HDR_RSVD1_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+4, 24, 8)

//--- LONG WORD 1 ----
#define GET_FIRMWARE_HDR_MONTH_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+8, 0, 8) // Release time Month field
#define GET_FIRMWARE_HDR_DATE_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+8, 8, 8) // Release time Date field
#define GET_FIRMWARE_HDR_HOUR_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+8, 16, 8)// Release time Hour field
#define GET_FIRMWARE_HDR_MINUTE_8812(__FwHdr)		LE_BITS_TO_4BYTE(__FwHdr+8, 24, 8)// Release time Minute field
#define GET_FIRMWARE_HDR_ROMCODE_SIZE_8812(__FwHdr)	LE_BITS_TO_4BYTE(__FwHdr+12, 0, 16)// The size of RAM code
#define GET_FIRMWARE_HDR_RSVD2_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+12, 16, 16)

//--- LONG WORD 2 ----
#define GET_FIRMWARE_HDR_SVN_IDX_8812(__FwHdr)		LE_BITS_TO_4BYTE(__FwHdr+16, 0, 32)// The SVN entry index
#define GET_FIRMWARE_HDR_RSVD3_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+20, 0, 32)

//--- LONG WORD 3 ----
#define GET_FIRMWARE_HDR_RSVD4_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+24, 0, 32)
#define GET_FIRMWARE_HDR_RSVD5_8812(__FwHdr)			LE_BITS_TO_4BYTE(__FwHdr+28, 0, 32)


#endif
