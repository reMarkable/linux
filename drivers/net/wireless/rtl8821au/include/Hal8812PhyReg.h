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
#ifndef __INC_HAL8812PHYREG_H__
#define __INC_HAL8812PHYREG_H__
/*--------------------------Define Parameters-------------------------------*/
//
// BB-PHY register PMAC 0x100 PHY 0x800 - 0xEFF
// 1. PMAC duplicate register due to connection: RF_Mode, TRxRN, NumOf L-STF
// 2. 0x800/0x900/0xA00/0xC00/0xD00/0xE00
// 3. RF register 0x00-2E
// 4. Bit Mask for BB/RF register
// 5. Other defintion for BB/RF R/W
//


// BB Register Definition

#define rCCAonSec_Jaguar		0x838

// BW and sideband setting
#define rBWIndication_Jaguar		0x834
#define rL1PeakTH_Jaguar		0x848
#define rRFMOD_Jaguar			0x8ac	//RF mode 
#define rADC_Buf_Clk_Jaguar		0x8c4
#define rRFECTRL_Jaguar			0x900
#define bRFMOD_Jaguar			0xc3
#define rCCK_System_Jaguar		0xa00   // for cck sideband
#define bCCK_System_Jaguar		0x10

// Block & Path enable
#define rOFDMCCKEN_Jaguar 		0x808 // OFDM/CCK block enable
#define bOFDMEN_Jaguar			0x20000000
#define bCCKEN_Jaguar			0x10000000
#define rRxPath_Jaguar			0x808	// Rx antenna
#define bRxPath_Jaguar			0xff
#define rTxPath_Jaguar			0x80c	// Tx antenna
#define bTxPath_Jaguar			0x0fffffff
#define rCCK_RX_Jaguar			0xa04	// for cck rx path selection
#define bCCK_RX_Jaguar			0x0c000000 
#define rVhtlen_Use_Lsig_Jaguar	0x8c3	// Use LSIG for VHT length

// RF read/write-related
#define rHSSIRead_Jaguar			0x8b0  // RF read addr
#define bHSSIRead_addr_Jaguar		0xff
#define bHSSIRead_trigger_Jaguar	0x100
#define rA_PIRead_Jaguar			0xd04 // RF readback with PI
#define rB_PIRead_Jaguar			0xd44 // RF readback with PI
#define rA_SIRead_Jaguar			0xd08 // RF readback with SI
#define rB_SIRead_Jaguar			0xd48 // RF readback with SI
#define rRead_data_Jaguar			0xfffff
#define rA_LSSIWrite_Jaguar			0xc90 // RF write addr
#define rB_LSSIWrite_Jaguar			0xe90 // RF write addr
#define bLSSIWrite_data_Jaguar		0x000fffff
#define bLSSIWrite_addr_Jaguar		0x0ff00000



// YN: mask the following register definition temporarily 
#define rFPGA0_XA_RFInterfaceOE			0x860	// RF Channel switch
#define rFPGA0_XB_RFInterfaceOE			0x864

#define rFPGA0_XAB_RFInterfaceSW		0x870	// RF Interface Software Control
#define rFPGA0_XCD_RFInterfaceSW		0x874

//#define RFPGA0_XAB_RFPARAMETER		0x878	// RF Parameter
//#define RFPGA0_XCD_RFPARAMETER		0x87c

//#define RFPGA0_ANALOGPARAMETER1		0x880	// Crystal cap setting RF-R/W protection for parameter4??
//#define RFPGA0_ANALOGPARAMETER2		0x884
//#define RFPGA0_ANALOGPARAMETER3		0x888
//#define rFPGA0_AdDaClockEn			0x888	// enable ad/da clock1 for dual-phy
//#define RFPGA0_ANALOGPARAMETER4		0x88c


// CCK TX scaling
#define rCCK_TxFilter1_Jaguar		0xa20
#define bCCK_TxFilter1_C0_Jaguar   	0x00ff0000
#define bCCK_TxFilter1_C1_Jaguar		0xff000000
#define rCCK_TxFilter2_Jaguar		0xa24
#define bCCK_TxFilter2_C2_Jaguar		0x000000ff
#define bCCK_TxFilter2_C3_Jaguar		0x0000ff00
#define bCCK_TxFilter2_C4_Jaguar		0x00ff0000
#define bCCK_TxFilter2_C5_Jaguar		0xff000000
#define rCCK_TxFilter3_Jaguar		0xa28
#define bCCK_TxFilter3_C6_Jaguar		0x000000ff
#define bCCK_TxFilter3_C7_Jaguar		0x0000ff00


// YN: mask the following register definition temporarily
//#define RPDP_ANTA      					0xb00  
//#define RPDP_ANTA_4    				0xb04
//#define rConfig_Pmpd_AntA 			0xb28
//#define rConfig_AntA 					0xb68
//#define rConfig_AntB 					0xb6c
//#define RPDP_ANTB 					0xb70
//#define RPDP_ANTB_4 					0xb74
//#define rConfig_Pmpd_AntB			0xb98
//#define rAPK							0xbd8



// DIG-related
#define rA_IGI_Jaguar				0xc50	// Initial Gain for path-A
#define rB_IGI_Jaguar				0xe50	// Initial Gain for path-B
#define rOFDM_FalseAlarm1_Jaguar	0xf48  // counter for break
#define rOFDM_FalseAlarm2_Jaguar	0xf4c  // counter for spoofing
#define rCCK_FalseAlarm_Jaguar        	0xa5c // counter for cck false alarm
#define b_FalseAlarm_Jaguar			0xffff
#define rCCK_CCA_Jaguar				0xa08	// cca threshold
#define bCCK_CCA_Jaguar				0x00ff0000

// Tx Power Ttraining-related
#define rA_TxPwrTraing_Jaguar		0xc54
#define rB_TxPwrTraing_Jaguar		0xe54

// Report-related
#define rOFDM_ShortCFOAB_Jaguar	0xf60  
#define rOFDM_LongCFOAB_Jaguar		0xf64
#define rOFDM_EndCFOAB_Jaguar		0xf70
#define rOFDM_AGCReport_Jaguar		0xf84
#define rOFDM_RxSNR_Jaguar			0xf88
#define rOFDM_RxEVMCSI_Jaguar		0xf8c
#define rOFDM_SIGReport_Jaguar		0xf90


// RFE
#define rA_RFE_Pinmux_Jaguar	0xcb0  // Path_A RFE cotrol pinmux
#define rB_RFE_Pinmux_Jaguar	0xeb0 // Path_B RFE control pinmux
#define rA_RFE_Inv_Jaguar		0xcb4  // Path_A RFE cotrol   
#define rB_RFE_Inv_Jaguar		0xeb4 // Path_B RFE control
#define rA_RFE_Jaguar			0xcb8  // Path_A RFE cotrol   
#define rB_RFE_Jaguar			0xeb8 // Path_B RFE control
#define r_ANTSEL_SW_Jaguar		0x900 // ANTSEL SW Control
#define bMask_RFEInv_Jaguar		0x3ff00000
#define bMask_AntselPathFollow_Jaguar 0x00030000

// IQK YN: temporaily mask this part
//#define rFPGA0_IQK					0xe28
//#define rTx_IQK_Tone_A				0xe30
//#define rRx_IQK_Tone_A				0xe34
//#define rTx_IQK_PI_A					0xe38
//#define rRx_IQK_PI_A					0xe3c

//#define rTx_IQK 						0xe40
//#define rRx_IQK						0xe44
//#define rIQK_AGC_Pts					0xe48
//#define rIQK_AGC_Rsp					0xe4c
//#define rTx_IQK_Tone_B				0xe50
//#define rRx_IQK_Tone_B				0xe54
//#define rTx_IQK_PI_B					0xe58
//#define rRx_IQK_PI_B					0xe5c
//#define rIQK_AGC_Cont				0xe60


//
//Bit Mask
//
// 1. Page1(0x100)
#define bBBResetB					0x100	// Useless now?
#define bGlobalResetB				0x200
#define bOFDMTxStart				0x4
#define bCCKTxStart					0x8
#define bCRC32Debug					0x100
#define bPMACLoopback				0x10
#define bTxLSIG						0xffffff
#define bOFDMTxRate					0xf
#define bOFDMTxReserved			0x10
#define bOFDMTxLength				0x1ffe0
#define bOFDMTxParity				0x20000
#define bTxHTSIG1					0xffffff
#define bTxHTMCSRate				0x7f
#define bTxHTBW						0x80
#define bTxHTLength					0xffff00
#define bTxHTSIG2					0xffffff
#define bTxHTSmoothing				0x1
#define bTxHTSounding				0x2
#define bTxHTReserved				0x4
#define bTxHTAggreation				0x8
#define bTxHTSTBC					0x30
#define bTxHTAdvanceCoding			0x40
#define bTxHTShortGI					0x80
#define bTxHTNumberHT_LTF			0x300
#define bTxHTCRC8					0x3fc00
#define bCounterReset				0x10000
#define bNumOfOFDMTx				0xffff
#define bNumOfCCKTx					0xffff0000
#define bTxIdleInterval				0xffff
#define bOFDMService				0xffff0000
#define bTxMACHeader				0xffffffff
#define bTxDataInit					0xff
#define bTxHTMode					0x100
#define bTxDataType					0x30000
#define bTxRandomSeed				0xffffffff
#define bCCKTxPreamble				0x1
#define bCCKTxSFD					0xffff0000
#define bCCKTxSIG					0xff
#define bCCKTxService				0xff00
#define bCCKLengthExt				0x8000
#define bCCKTxLength				0xffff0000
#define bCCKTxCRC16					0xffff
#define bCCKTxStatus					0x1
#define bOFDMTxStatus				0x2


//
// 1. PMAC duplicate register due to connection: RF_Mode, TRxRN, NumOf L-STF
// 1. Page1(0x100)
//
#define rPMAC_Reset					0x100
#define rPMAC_TxStart				0x104
#define rPMAC_TxLegacySIG			0x108
#define rPMAC_TxHTSIG1				0x10c
#define rPMAC_TxHTSIG2				0x110
#define rPMAC_PHYDebug				0x114
#define rPMAC_TxPacketNum			0x118
#define rPMAC_TxIdle					0x11c
#define rPMAC_TxMACHeader0			0x120
#define rPMAC_TxMACHeader1			0x124
#define rPMAC_TxMACHeader2			0x128
#define rPMAC_TxMACHeader3			0x12c
#define rPMAC_TxMACHeader4			0x130
#define rPMAC_TxMACHeader5			0x134
#define rPMAC_TxDataType			0x138
#define rPMAC_TxRandomSeed		0x13c
#define rPMAC_CCKPLCPPreamble		0x140
#define rPMAC_CCKPLCPHeader		0x144
#define rPMAC_CCKCRC16				0x148
#define rPMAC_OFDMRxCRC32OK		0x170
#define rPMAC_OFDMRxCRC32Er		0x174
#define rPMAC_OFDMRxParityEr		0x178
#define rPMAC_OFDMRxCRC8Er			0x17c
#define rPMAC_CCKCRxRC16Er			0x180
#define rPMAC_CCKCRxRC32Er			0x184
#define rPMAC_CCKCRxRC32OK			0x188
#define rPMAC_TxStatus				0x18c





// 2. Page8(0x800)
#define bRFMOD						0x1	// Reg 0x800 RFPGA0_RFMOD
#define bJapanMode					0x2
#define bCCKTxSC					0x30
#define bCCKEn						0x1000000
#define bOFDMEn						0x2000000
#define bXBTxAGC                  			0xf00	// Reg 80c rFPGA0_TxGainStage
#define bXCTxAGC                  			0xf000
#define bXDTxAGC                  			0xf0000

// 4. PageA(0xA00)
#define bCCKBBMode                			0x3	// Useless
#define bCCKTxPowerSaving         		0x80
#define bCCKRxPowerSaving         		0x40

#define bCCKSideBand              		0x10	// Reg 0xa00 rCCK0_System 20/40 switch

#define bCCKScramble              		0x8	// Useless
#define bCCKAntDiversity    		      	0x8000
#define bCCKCarrierRecovery   	    	0x4000
#define bCCKTxRate           		     	0x3000
#define bCCKDCCancel             	 		0x0800
#define bCCKISICancel             			0x0400
#define bCCKMatchFilter           		0x0200
#define bCCKEqualizer             			0x0100
#define bCCKPreambleDetect       	 	0x800000
#define bCCKFastFalseCCA          		0x400000
#define bCCKChEstStart            		0x300000
#define bCCKCCACount              		0x080000
#define bCCKcs_lim                			0x070000
#define bCCKBistMode              			0x80000000
#define bCCKCCAMask             	  		0x40000000
#define bCCKTxDACPhase         	   	0x4
#define bCCKRxADCPhase         	   	0x20000000   //r_rx_clk
#define bCCKr_cp_mode0         	   	0x0100
#define bCCKTxDCOffset           	 	0xf0
#define bCCKRxDCOffset           	 	0xf
#define bCCKCCAMode              	 		0xc000
#define bCCKFalseCS_lim           		0x3f00
#define bCCKCS_ratio              			0xc00000
#define bCCKCorgBit_sel           		0x300000
#define bCCKPD_lim                			0x0f0000
#define bCCKNewCCA                		0x80000000
#define bCCKRxHPofIG              		0x8000
#define bCCKRxIG                  			0x7f00
#define bCCKLNAPolarity           		0x800000
#define bCCKRx1stGain             		0x7f0000
#define bCCKRFExtend              		0x20000000 //CCK Rx Iinital gain polarity
#define bCCKRxAGCSatLevel        	 	0x1f000000
#define bCCKRxAGCSatCount       	  	0xe0
#define bCCKRxRFSettle            		0x1f       //AGCsamp_dly
#define bCCKFixedRxAGC           	 	0x8000
//#define bCCKRxAGCFormat         	 	0x4000   //remove to HSSI register 0x824
#define bCCKAntennaPolarity      	 	0x2000
#define bCCKTxFilterType          		0x0c00
#define bCCKRxAGCReportType   	   	0x0300
#define bCCKRxDAGCEn              		0x80000000
#define bCCKRxDAGCPeriod        	  	0x20000000
#define bCCKRxDAGCSatLevel     	   	0x1f000000
#define bCCKTimingRecovery       	 	0x800000
#define bCCKTxC0                  			0x3f0000
#define bCCKTxC1                  			0x3f000000
#define bCCKTxC2                  			0x3f
#define bCCKTxC3                  			0x3f00
#define bCCKTxC4                  			0x3f0000
#define bCCKTxC5                  			0x3f000000
#define bCCKTxC6                  			0x3f
#define bCCKTxC7                  			0x3f00
#define bCCKDebugPort             		0xff0000
#define bCCKDACDebug              		0x0f000000
#define bCCKFalseAlarmEnable      		0x8000
#define bCCKFalseAlarmRead        		0x4000
#define bCCKTRSSI                 			0x7f
#define bCCKRxAGCReport           		0xfe
#define bCCKRxReport_AntSel       		0x80000000
#define bCCKRxReport_MFOff        		0x40000000
#define bCCKRxRxReport_SQLoss     	0x20000000
#define bCCKRxReport_Pktloss      		0x10000000
#define bCCKRxReport_Lockedbit    	0x08000000
#define bCCKRxReport_RateError    	0x04000000
#define bCCKRxReport_RxRate       		0x03000000
#define bCCKRxFACounterLower      	0xff
#define bCCKRxFACounterUpper      	0xff000000
#define bCCKRxHPAGCStart          		0xe000
#define bCCKRxHPAGCFinal          		0x1c00       		
#define bCCKRxFalseAlarmEnable    	0x8000
#define bCCKFACounterFreeze       		0x4000       		
#define bCCKTxPathSel             		0x10000000
#define bCCKDefaultRxPath         		0xc000000
#define bCCKOptionRxPath          		0x3000000

// 6. PageE(0xE00)
#define bSTBCEn                  			0x4	// Useless
#define bAntennaMapping          		0x10
#define bNss                     				0x20
#define bCFOAntSumD              		0x200
#define bPHYCounterReset         		0x8000000
#define bCFOReportGet            			0x4000000
#define bOFDMContinueTx          		0x10000000
#define bOFDMSingleCarrier       		0x20000000
#define bOFDMSingleTone          		0x40000000


//
// Other Definition
//

#define bEnable                   0x1	// Useless
#define bDisable                  0x0

//byte endable for srwrite
#define bByte0                    		0x1	// Useless
#define bByte1                    		0x2
#define bByte2                    		0x4
#define bByte3                    		0x8
#define bWord0                    		0x3
#define bWord1                    		0xc
#define bDWord                    		0xf

/*--------------------------Define Parameters-------------------------------*/


#endif

