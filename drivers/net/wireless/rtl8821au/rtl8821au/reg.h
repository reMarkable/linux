#ifndef __RTL8821AU_REG_H__
#define __RTL8821AU_REG_H__

/* ULLI : look in wifi.h -> MASKLWORD */


//for PutRegsetting & GetRegSetting BitMask
#define bMaskHWord                	0xffff0000
#define bMaskLWord                	0x0000ffff
#define bMaskDWord                	0xffffffff
#define bMask12Bits			0xfff
#define bMaskH4Bits			0xf0000000
#define bMaskOFDM_D			0xffc00000
#define bMaskCCK			0x3f3f3f3f



//
// 3. Page8(0x800)
//
#define RFPGA0_RFMOD			0x800

#define RFPGA0_TXINFO			0x804
#define RFPGA0_PSDFUNCTION		0x808

#define RFPGA0_TXGAINSTAGE		0x80c

#define RFPGA0_XA_HSSIPARAMETER1	0x820
#define RFPGA0_XA_HSSIPARAMETER2	0x824
#define RFPGA0_XB_HSSIPARAMETER1	0x828
#define RFPGA0_XB_HSSIPARAMETER2	0x82c

#define RFPGA0_XAB_SWITCHCONTROL	0x858
#define RFPGA0_XCD_SWITCHCONTROL	0x85c

#define RFPGA0_XAB_RFPARAMETER		0x878	// RF Parameter
#define RFPGA0_XCD_RFPARAMETER		0x87c

#define RFPGA0_ANALOGPARAMETER1		0x880	// Crystal cap setting RF-R/W protection for parameter4??
#define RFPGA0_ANALOGPARAMETER2		0x884
#define RFPGA0_ANALOGPARAMETER3		0x888
#define rFPGA0_AdDaClockEn		0x888	// enable ad/da clock1 for dual-phy
#define RFPGA0_ANALOGPARAMETER4		0x88c
#define rFPGA0_XB_LSSIReadBack		0x8a4
//
// 4. Page9(0x900)
//
#define RFPGA1_RFMOD			0x900	//RF mode & OFDM TxSC // RF BW Setting??

#define RFPGA1_TXBLOCK			0x904	// Useless now
#define RFPGA1_DEBUGSELECT		0x908	// Useless now
#define RFPGA1_TXINFO			0x90c	// Useless now // Status report??

//
// PageA(0xA00)
//
#define rCCK0_System			0xa00
#define rCCK0_AFESetting		0xa04	// Disable init gain now // Select RX path by RSSI
#define rCCK0_TxFilter1			0xa20
#define rCCK0_TxFilter2			0xa24
#define rCCK0_DebugPort			0xa28	//debug port and Tx filter3

//
// PageB(0xB00)
//
#define RPDP_ANTA      			0xb00  
#define RPDP_ANTA_4    			0xb04
#define rConfig_Pmpd_AntA 		0xb28
#define rConfig_AntA 			0xb68
#define rConfig_AntB 			0xb6c
#define RPDP_ANTB 			0xb70
#define RPDP_ANTB_4 			0xb74
#define rConfig_Pmpd_AntB		0xb98
#define rAPK				0xbd8


//
// 6. PageC(0xC00)
//
#define ROFDM0_LSTF				0xc00

#define ROFDM0_TRXPATHENABLE			0xc04
#define ROFDM0_TRMUXPAR				0xc08
#define ROFDM0_TRSWISOLATION			0xc0c

#define ROFDM0_XARXAFE				0xc10  //RxIQ DC offset, Rx digital filter, DC notch filter
#define ROFDM0_XARXIQIMBALANCE    		0xc14  //RxIQ imblance matrix
#define ROFDM0_XBRXAFE            		0xc18
#define ROFDM0_XBRXIQIMBALANCE    		0xc1c
#define ROFDM0_XCRXAFE            		0xc20
#define ROFDM0_XCRXIQIMBALANCE    		0xc24			/* ULLI Typo in rtl8821ae */
#define ROFDM0_XDRXAFE            		0xc28
#define ROFDM0_XDRXIQIMBALANCE    		0xc2c

#define ROFDM0_RXDETECTOR1			0xc30  //PD,BW & SBD	// DM tune init gain
#define ROFDM0_RXDETECTOR2			0xc34  //SBD & Fame Sync. 
#define ROFDM0_RXDETECTOR3			0xc38  //Frame Sync.
#define ROFDM0_RXDETECTOR4			0xc3c  //PD, SBD, Frame Sync & Short-GI

#define rOFDM0_RxDSP				0xc40  //Rx Sync Path
#define rOFDM0_CFOandDAGC			0xc44  //CFO & DAGC
#define rOFDM0_CCADropThreshold	0xc48 //CCA Drop threshold
#define rOFDM0_ECCAThreshold		0xc4c // energy CCA

#define rOFDM0_XAAGCCore1			0xc50	// DIG
#define rOFDM0_XAAGCCore2			0xc54
#define rOFDM0_XBAGCCore1			0xc58
#define rOFDM0_XBAGCCore2			0xc5c
#define rOFDM0_XCAGCCore1			0xc60
#define rOFDM0_XCAGCCore2			0xc64
#define rOFDM0_XDAGCCore1			0xc68
#define rOFDM0_XDAGCCore2			0xc6c

#define ROFDM0_AGCPARAMETER1		0xc70
#define ROFDM0_AGCPARAMETER2		0xc74
#define rOFDM0_AGCRSSITable		0xc78
#define rOFDM0_HTSTFAGC			0xc7c

#define rOFDM0_XATxIQImbalance		0xc80	// TX PWR TRACK and DIG
#define rOFDM0_XATxAFE				0xc84
#define rOFDM0_XBTxIQImbalance		0xc88
#define rOFDM0_XBTxAFE				0xc8c
#define rOFDM0_XCTxIQImbalance		0xc90
#define rOFDM0_XCTxAFE            		0xc94
#define rOFDM0_XDTxIQImbalance		0xc98
#define rOFDM0_XDTxAFE				0xc9c

#define rOFDM0_RxIQExtAnta			0xca0
#define ROFDM0_TXCOEFF1				0xca4
#define ROFDM0_TXCOEFF2				0xca8
#define ROFDM0_TXCOEFF3				0xcac
#define ROFDM0_TXCOEFF4				0xcb0
#define ROFDM0_TXCOEFF5				0xcb4
#define ROFDM0_TXCOEFF6				0xcb8
#define rOFDM0_RxHPParameter		0xce0
#define rOFDM0_TxPseudoNoiseWgt	0xce4
#define rOFDM0_FrameSync			0xcf0
#define rOFDM0_DFSReport			0xcf4

//
// 7. PageD(0xD00)
//
#define rOFDM1_LSTF					0xd00
#define rOFDM1_TRxPathEnable		0xd04

//
// 8. PageE(0xE00)
//
#define rTxAGC_A_Rate18_06			0xe00
#define rTxAGC_A_Rate54_24			0xe04
#define rTxAGC_A_CCK1_Mcs32		0xe08
#define rTxAGC_A_Mcs03_Mcs00		0xe10
#define rTxAGC_A_Mcs07_Mcs04		0xe14
#define rTxAGC_A_Mcs11_Mcs08		0xe18
#define rTxAGC_A_Mcs15_Mcs12		0xe1c

#define rTxAGC_B_Rate18_06			0x830
#define rTxAGC_B_Rate54_24			0x834
#define rTxAGC_B_CCK1_55_Mcs32	0x838
#define rTxAGC_B_Mcs03_Mcs00		0x83c
#define rTxAGC_B_Mcs07_Mcs04		0x848
#define rTxAGC_B_Mcs11_Mcs08		0x84c
#define rTxAGC_B_Mcs15_Mcs12		0x868
#define rTxAGC_B_CCK11_A_CCK2_11	0x86c

#define rFPGA0_IQK					0xe28
#define rTx_IQK_Tone_A				0xe30
#define rRx_IQK_Tone_A				0xe34
#define rTx_IQK_PI_A				0xe38
#define rRx_IQK_PI_A				0xe3c

#define rTx_IQK 						0xe40
#define rRx_IQK						0xe44
#define rIQK_AGC_Pts					0xe48
#define rIQK_AGC_Rsp				0xe4c
#define rTx_IQK_Tone_B				0xe50
#define rRx_IQK_Tone_B				0xe54
#define rTx_IQK_PI_B					0xe58
#define rRx_IQK_PI_B					0xe5c
#define rIQK_AGC_Cont				0xe60

#define rBlue_Tooth					0xe6c
#define rRx_Wait_CCA				0xe70
#define rTx_CCK_RFON				0xe74
#define rTx_CCK_BBON				0xe78
#define rTx_OFDM_RFON				0xe7c
#define rTx_OFDM_BBON				0xe80
#define rTx_To_Rx					0xe84
#define rTx_To_Tx					0xe88
#define rRx_CCK						0xe8c

#define RTX_POWER_BEFORE_IQK_A		0xe94
#define RTX_POWER_AFTER_IQK_A		0xe9c

#define RRX_POWER_BEFORE_IQK_A		0xea0
#define RRX_POWER_BEFORE_IQK_A_2	0xea4
#define RRX_POWER_AFTER_IQK_A		0xea8
#define RRX_POWER_AFTER_IQK_A_2		0xeac

#define RTX_POWER_BEFORE_IQK_B		0xeb4
#define RTX_POWER_AFTER_IQK_B		0xebc

#define RRX_POWER_BEFORE_IQK_B		0xec0
#define RRX_POWER_BEFORE_IQK_B_2	0xec4
#define RRX_POWER_AFTER_IQK_B		0xec8
#define RRX_POWER_AFTER_IQK_B_2		0xecc

#define rRx_OFDM				0xed0
#define rRx_Wait_RIFS 				0xed4
#define rRx_TO_Rx 				0xed8
#define rStandby 				0xedc
#define rSleep 					0xee0
#define rPMPD_ANAEN				0xeec



// TX AGC 
#define RTXAGC_A_CCK11_CCK1			0xc20
#define RTXAGC_A_OFDM18_OFDM6			0xc24
#define RTXAGC_A_OFDM54_OFDM24			0xc28
#define RTXAGC_A_MCS03_MCS00			0xc2c
#define RTXAGC_A_MCS07_MCS04			0xc30
#define RTXAGC_A_MCS11_MCS08			0xc34
#define RTXAGC_A_MCS15_MCS12			0xc38
#define RTXAGC_A_NSS1INDEX3_NSS1INDEX0		0xc3c
#define RTXAGC_A_NSS1INDEX7_NSS1INDEX4		0xc40
#define RTXAGC_A_NSS2INDEX1_NSS1INDEX8		0xc44
#define RTXAGC_A_NSS2INDEX5_NSS2INDEX2		0xc48
#define RTXAGC_A_NSS2INDEX9_NSS2INDEX6		0xc4c


#define RTXAGC_B_CCK11_CCK1			0xe20
#define RTXAGC_B_OFDM18_OFDM6			0xe24
#define RTXAGC_B_OFDM54_OFDM24			0xe28
#define RTXAGC_B_MCS03_MCS00			0xe2c
#define RTXAGC_B_MCS07_MCS04			0xe30
#define RTXAGC_B_MCS11_MCS08			0xe34
#define RTXAGC_B_MCS15_MCS12			0xe38
#define RTXAGC_B_NSS1INDEX3_NSS1INDEX0		0xe3c
#define RTXAGC_B_NSS1INDEX7_NSS1INDEX4		0xe40
#define RTXAGC_B_NSS2INDEX1_NSS1INDEX8		0xe44
#define RTXAGC_B_NSS2INDEX5_NSS2INDEX2		0xe48
#define RTXAGC_B_NSS2INDEX9_NSS2INDEX6		0xe4c

#define MASKBYTE0                		0xff	// Reg 0xc50 rOFDM0_XAAGCCore~0xC6f
#define MASKBYTE1                		0xff00
#define MASKBYTE2                		0xff0000
#define MASKBYTE3                		0xff000000

#define bTxAGC_byte0_Jaguar							0xff
#define bTxAGC_byte1_Jaguar							0xff00
#define bTxAGC_byte2_Jaguar							0xff0000
#define bTxAGC_byte3_Jaguar							0xff000000

#define RF_T_METER_8812A 		0x42

// RXIQC
#define rA_RxIQC_AB_Jaguar    	0xc10  //RxIQ imblance matrix coeff. A & B
#define rA_RxIQC_CD_Jaguar    	0xc14  //RxIQ imblance matrix coeff. C & D
#define rA_TxScale_Jaguar 		0xc1c  // Pah_A TX scaling factor
#define rB_TxScale_Jaguar 		0xe1c  // Path_B TX scaling factor
#define rB_RxIQC_AB_Jaguar    	0xe10  //RxIQ imblance matrix coeff. A & B
#define rB_RxIQC_CD_Jaguar    	0xe14  //RxIQ imblance matrix coeff. C & D
#define b_RxIQC_AC_Jaguar		0x02ff  // bit mask for IQC matrix element A & C
#define b_RxIQC_BD_Jaguar		0x02ff0000 // bit mask for IQC matrix element A & C

// Misc functions
#define rEDCCA_Jaguar				0x8a4 // EDCCA
#define bEDCCA_Jaguar				0xffff
#define rAGC_table_Jaguar			0x82c   // AGC tabel select
#define bAGC_table_Jaguar			0x3
#define b_sel5g_Jaguar    				0x1000 // sel5g
#define b_LNA_sw_Jaguar				0x8000 // HW/WS control for LNA
#define rFc_area_Jaguar				0x860   // fc_area 
#define bFc_area_Jaguar				0x1ffe000
#define rSingleTone_ContTx_Jaguar	0x914


// AFE-related
#define rA_AFEPwr1_Jaguar					0xc60 // dynamic AFE power control
#define rA_AFEPwr2_Jaguar					0xc64 // dynamic AFE power control
#define rA_Rx_WaitCCA_Tx_CCKRFON_Jaguar	0xc68
#define rA_Tx_CCKBBON_OFDMRFON_Jaguar	0xc6c
#define rA_Tx_OFDMBBON_Tx2Rx_Jaguar		0xc70
#define rA_Tx2Tx_RXCCK_Jaguar				0xc74
#define rA_Rx_OFDM_WaitRIFS_Jaguar			0xc78
#define rA_Rx2Rx_BT_Jaguar					0xc7c
#define rA_sleep_nav_Jaguar 					0xc80
#define rA_pmpd_Jaguar 						0xc84
#define rB_AFEPwr1_Jaguar					0xe60 // dynamic AFE power control
#define rB_AFEPwr2_Jaguar					0xe64 // dynamic AFE power control
#define rB_Rx_WaitCCA_Tx_CCKRFON_Jaguar	0xe68
#define rB_Tx_CCKBBON_OFDMRFON_Jaguar	0xe6c
#define rB_Tx_OFDMBBON_Tx2Rx_Jaguar		0xe70
#define rB_Tx2Tx_RXCCK_Jaguar				0xe74
#define rB_Rx_OFDM_WaitRIFS_Jaguar			0xe78
#define rB_Rx2Rx_BT_Jaguar					0xe7c
#define rB_sleep_nav_Jaguar 					0xe80
#define rB_pmpd_Jaguar 						0xe84

// RSSI Dump
#define rA_RSSIDump_Jaguar 			0xBF0
#define rB_RSSIDump_Jaguar 			0xBF1
#define rS1_RXevmDump_Jaguar		0xBF4 
#define rS2_RXevmDump_Jaguar 		0xBF5
#define rA_RXsnrDump_Jaguar		0xBF6
#define rB_RXsnrDump_Jaguar		0xBF7
#define rA_CfoShortDump_Jaguar		0xBF8 
#define rB_CfoShortDump_Jaguar		0xBFA
#define rA_CfoLongDump_Jaguar		0xBEC
#define rB_CfoLongDump_Jaguar		0xBEE
 

// RF Register
//
#define RF_AC_Jaguar				0x00	// 
#define RF_RF_Top_Jaguar			0x07	// 
#define RF_TXLOK_Jaguar				0x08	// 
#define RF_TXAPK_Jaguar				0x0B
#define RF_CHNLBW_Jaguar 			0x18	// RF channel and BW switch
#define RF_TxLCTank_Jaguar          	0x54
#define RF_APK_Jaguar				0x63
#define bRF_CHNLBW_MOD_AG_Jaguar	0x70300
#define bRF_CHNLBW_BW 				0xc00
#define RF_RCK1_Jaguar				0x1c	// 
#define RF_RCK2_Jaguar				0x1d
#define RF_RCK3_Jaguar   			0x1e
#define RF_LCK						0xB4


//
// RL6052 Register definition
//
#define RF_AC						0x00	// 
#define RF_IPA_A					0x0C	// 
#define RF_TXBIAS_A					0x0D
#define RF_BS_PA_APSET_G9_G11		0x0E
#define RF_MODE1					0x10	// 
#define RF_MODE2					0x11	// 
#define RF_CHNLBW					0x18	// RF channel and BW switch
#define RF_RCK_OS					0x30	// RF TX PA control
#define RF_TXPA_G1					0x31	// RF TX PA control
#define RF_TXPA_G2					0x32	// RF TX PA control
#define RF_TXPA_G3					0x33	// RF TX PA control
#define RF_0x52 						0x52
#define RF_WE_LUT					0xEF

// Security
#define REG_CAMCMD						0x0670
#define REG_CAMWRITE					0x0674
#define REG_CAMREAD					0x0678
#define REG_CAMDBG						0x067C
#define REG_SECCFG						0x0680

//----------------------------------------------------------------------------
//       CAM Config Setting (offset 0x680, 1 byte)
//----------------------------------------------------------------------------
#define CAM_NOTVALID			0x0000
#define CAM_USEDK				BIT5

#define CAM_CONTENT_COUNT 	8

#define CAM_NONE				0x0
#define CAM_WEP40				0x01
#define CAM_TKIP				0x02
#define CAM_AES					0x04
#define CAM_WEP104				0x05

#define HALF_CAM_ENTRY			16

#define CAM_CONFIG_USEDK		true
#define CAM_CONFIG_NO_USEDK	false

#define CAM_READ				0x00000000

#define SCR_UseDK				0x01
#define SCR_TxSecEnable			0x02
#define SCR_RxSecEnable			0x04


//====================================================
//			EEPROM/Efuse PG Offset for 8812AE/8812AU/8812AS
//====================================================
// 0x10 ~ 0x63 = TX power area.
#define EEPROM_USB_MODE_8812					0x08
#define EEPROM_TX_PWR_INX_8812				0x10

#define EEPROM_ChannelPlan_8812				0xB8
#define EEPROM_XTAL_8812						0xB9
#define EEPROM_THERMAL_METER_8812			0xBA
#define EEPROM_IQK_LCK_8812					0xBB
#define EEPROM_2G_5G_PA_TYPE_8812			0xBC
#define EEPROM_2G_LNA_TYPE_GAIN_SEL_8812	0xBD
#define EEPROM_5G_LNA_TYPE_GAIN_SEL_8812	0xBF

#define EEPROM_RF_BOARD_OPTION_8812			0xC1
#define EEPROM_RF_FEATURE_OPTION_8812		0xC2
#define EEPROM_RF_BT_SETTING_8812				0xC3
#define EEPROM_VERSION_8812					0xC4
#define EEPROM_CustomID_8812					0xC5
#define EEPROM_TX_BBSWING_2G_8812			0xC6
#define EEPROM_TX_BBSWING_5G_8812			0xC7
#define EEPROM_TX_PWR_CALIBRATE_RATE_8812	0xC8
#define EEPROM_RF_ANTENNA_OPT_8812			0xC9
#define EEPROM_RFE_OPTION_8812				0xCA

// RTL8812AE
#define EEPROM_MAC_ADDR_8812AE				0xD0
#define EEPROM_VID_8812AE						0xD6
#define EEPROM_DID_8812AE						0xD8
#define EEPROM_SVID_8812AE						0xDA
#define EEPROM_SMID_8812AE					0xDC

//RTL8812AU
#define EEPROM_MAC_ADDR_8812AU				0xD7
#define EEPROM_VID_8812AU						0xD0
#define EEPROM_PID_8812AU						0xD2
#define EEPROM_PA_TYPE_8812AU					0xBC
#define EEPROM_LNA_TYPE_2G_8812AU			0xBD
#define EEPROM_LNA_TYPE_5G_8812AU			0xBF

//====================================================
//			EEPROM/Efuse PG Offset for 8821AE/8821AU/8821AS
//====================================================
#define EEPROM_TX_PWR_INX_8821				0x10

#define EEPROM_ChannelPlan_8821				0xB8
#define EEPROM_XTAL_8821						0xB9
#define EEPROM_THERMAL_METER_8821			0xBA
#define EEPROM_IQK_LCK_8821					0xBB


#define EEPROM_RF_BOARD_OPTION_8821			0xC1
#define EEPROM_RF_FEATURE_OPTION_8821		0xC2
#define EEPROM_RF_BT_SETTING_8821				0xC3
#define EEPROM_VERSION_8821					0xC4
#define EEPROM_CustomID_8821					0xC5
#define EEPROM_RF_ANTENNA_OPT_8821			0xC9

// RTL8821AE
#define EEPROM_MAC_ADDR_8821AE				0xD0
#define EEPROM_VID_8821AE						0xD6
#define EEPROM_DID_8821AE						0xD8
#define EEPROM_SVID_8821AE						0xDA
#define EEPROM_SMID_8821AE					0xDC

//RTL8821AU
#define EEPROM_PA_TYPE_8821AU					0xBC
#define EEPROM_LNA_TYPE_8821AU				0xBF

// RTL8821AS
#define EEPROM_MAC_ADDR_8821AS				0x11A

//RTL8821AU
#define EEPROM_MAC_ADDR_8821AU				0x107
#define EEPROM_VID_8821AU						0x100
#define EEPROM_PID_8821AU						0x102

#define EEPROM_Default_PID						0x1234
#define EEPROM_Default_VID						0x5678
#define EEPROM_Default_CustomerID				0xAB
#define EEPROM_Default_CustomerID_8188E		0x00
#define EEPROM_Default_SubCustomerID			0xCD
#define EEPROM_Default_Version					0

#define EEPROM_Default_CrystalCap_8812			0x20

//New EFUSE deafult value
#define EEPROM_DEFAULT_24G_INDEX			0x2A
#define EEPROM_DEFAULT_24G_HT20_DIFF		0X02
#define EEPROM_DEFAULT_24G_OFDM_DIFF		0X04

#define EEPROM_DEFAULT_5G_INDEX			0X2A
#define EEPROM_DEFAULT_5G_HT20_DIFF		0X00
#define EEPROM_DEFAULT_5G_OFDM_DIFF		0X04

#define EEPROM_DEFAULT_DIFF				0XFE
#define EEPROM_DEFAULT_CHANNEL_PLAN		0x7F
#define EEPROM_DEFAULT_BOARD_OPTION		0x00
#define EEPROM_DEFAULT_RFE_OPTION		0x04
#define EEPROM_DEFAULT_FEATURE_OPTION	0x00
#define EEPROM_DEFAULT_BT_OPTION			0x10

#define EEPROM_Default_ThermalMeter_8812		0x18

#define EEPROM_Default_PAType						0
#define EEPROM_Default_LNAType						0

#define RTL_EEPROM_ID							0x8129

#define EEPROM_CID_DEFAULT					0x0



#endif
