#ifndef	__RTL8821AU_DM_H__
#define __RTL8821AU_DM_H__

#include <odm_types.h>
#include <odm.h>


//3===========================================================
//3 Dynamic Tx Power
//3===========================================================
//Dynamic Tx Power Control Threshold
#define		TX_POWER_NEAR_FIELD_THRESH_LVL2	74
#define		TX_POWER_NEAR_FIELD_THRESH_LVL1	67
#define		TX_POWER_NEAR_FIELD_THRESH_AP		0x3F

#define		TxHighPwrLevel_Normal		0
#define		TxHighPwrLevel_Level1		1
#define		TxHighPwrLevel_Level2		2
#define		TxHighPwrLevel_BT1			3
#define		TxHighPwrLevel_BT2			4
#define		TxHighPwrLevel_15			5
#define		TxHighPwrLevel_35			6
#define		TxHighPwrLevel_50			7
#define		TxHighPwrLevel_70			8
#define		TxHighPwrLevel_100			9



#define CCK_TABLE_SIZE				33

void rtl8812au_get_delta_swing_table(struct rtl_priv *rtlpriv,
					    u8 **up_a, u8 **down_a,
					    u8 **up_b, u8 **down_b);
void rtl8821au_get_delta_swing_table(struct rtl_priv *rtlpriv,
					    u8 **up_a, u8 **down_a,
					    u8 **up_b, u8 **down_b);
void rtl8821au_dm_watchdog(struct rtl_priv *rtlpriv);
void rtl8821au_dm_clean_txpower_tracking_state(struct rtl_priv *rtlpriv);
void rtl8812_dm_init(struct rtl_priv *rtlpriv);

/* Old prototypes */

void odm_Adaptivity(struct rtl_priv *rtlpriv, u8 IGI);
void odm_RefreshRateAdaptiveMask(struct rtl_priv *rtlpriv);

/*  ULLI : from odm_RegDefine11AC.h */

//2 BB REG LIST
//PAGE 8
#define	ODM_REG_CCK_RPT_FORMAT_11AC	0x804
#define	ODM_REG_BB_RX_PATH_11AC			0x808
//PAGE 9
#define	ODM_REG_OFDM_FA_RST_11AC		0x9A4
//PAGE A
#define	ODM_REG_CCK_CCA_11AC			0xA0A
#define	ODM_REG_CCK_FA_RST_11AC			0xA2C
#define	ODM_REG_CCK_FA_11AC				0xA5C
//PAGE C
#define	ODM_REG_IGI_A_11AC				0xC50
//PAGE E
#define	ODM_REG_IGI_B_11AC				0xE50
//PAGE F
#define	ODM_REG_OFDM_FA_11AC			0xF48


//2 MAC REG LIST




//DIG Related
#define	ODM_BIT_IGI_11AC					0xFFFFFFFF
#define	ODM_BIT_CCK_RPT_FORMAT_11AC		BIT(16)
#define	ODM_BIT_BB_RX_PATH_11AC			0xF

/* ULLI : From odm_reg.h */
//MAC REG
#define	ODM_BB_RESET					0x002
#define	ODM_DUMMY					0x4fe
#define	RF_T_METER_OLD				0x24
#define	RF_T_METER_NEW				0x42

#define	ODM_EDCA_VO_PARAM			0x500
#define	ODM_EDCA_VI_PARAM			0x504
#define	ODM_EDCA_BE_PARAM			0x508
#define	ODM_EDCA_BK_PARAM			0x50C
#define	ODM_TXPAUSE					0x522

//BB REG
#define	ODM_FPGA_PHY0_PAGE8			0x800
#define	ODM_PSD_SETTING				0x808
#define	ODM_AFE_SETTING				0x818
#define	ODM_TXAGC_B_6_18				0x830
#define	ODM_TXAGC_B_24_54			0x834
#define	ODM_TXAGC_B_MCS32_5			0x838
#define	ODM_TXAGC_B_MCS0_MCS3		0x83c
#define	ODM_TXAGC_B_MCS4_MCS7		0x848
#define	ODM_TXAGC_B_MCS8_MCS11		0x84c
#define	ODM_ANALOG_REGISTER			0x85c
#define	ODM_RF_INTERFACE_OUTPUT		0x860
#define	ODM_TXAGC_B_MCS12_MCS15	0x868
#define	ODM_TXAGC_B_11_A_2_11		0x86c
#define	ODM_AD_DA_LSB_MASK			0x874
#define	ODM_ENABLE_3_WIRE			0x88c
#define	ODM_PSD_REPORT				0x8b4
#define	ODM_R_ANT_SELECT				0x90c
#define	ODM_CCK_ANT_SELECT			0xa07
#define	ODM_CCK_PD_THRESH			0xa0a
#define	ODM_CCK_RF_REG1				0xa11
#define	ODM_CCK_MATCH_FILTER			0xa20
#define	ODM_CCK_RAKE_MAC				0xa2e
#define	ODM_CCK_CNT_RESET			0xa2d
#define	ODM_CCK_TX_DIVERSITY			0xa2f
#define	ODM_CCK_FA_CNT_MSB			0xa5b
#define	ODM_CCK_FA_CNT_LSB			0xa5c
#define	ODM_CCK_NEW_FUNCTION		0xa75
#define	ODM_OFDM_PHY0_PAGE_C		0xc00
#define	ODM_OFDM_RX_ANT				0xc04
#define	ODM_R_A_RXIQI					0xc14
#define	ODM_R_A_AGC_CORE1			0xc50
#define	ODM_R_A_AGC_CORE2			0xc54
#define	ODM_R_B_AGC_CORE1			0xc58
#define	ODM_R_AGC_PAR					0xc70
#define	ODM_R_HTSTF_AGC_PAR			0xc7c
#define	ODM_TX_PWR_TRAINING_A		0xc90
#define	ODM_TX_PWR_TRAINING_B		0xc98
#define	ODM_OFDM_FA_CNT1				0xcf0
#define	ODM_OFDM_PHY0_PAGE_D		0xd00
#define	ODM_OFDM_FA_CNT2				0xda0
#define	ODM_OFDM_FA_CNT3				0xda4
#define	ODM_OFDM_FA_CNT4				0xda8
#define	ODM_TXAGC_A_6_18				0xe00
#define	ODM_TXAGC_A_24_54			0xe04
#define	ODM_TXAGC_A_1_MCS32			0xe08
#define	ODM_TXAGC_A_MCS0_MCS3		0xe10
#define	ODM_TXAGC_A_MCS4_MCS7		0xe14
#define	ODM_TXAGC_A_MCS8_MCS11		0xe18
#define	ODM_TXAGC_A_MCS12_MCS15		0xe1c

//RF REG
#define	ODM_GAIN_SETTING				0x00
#define	ODM_CHANNEL					0x18

//Ant Detect Reg
#define	ODM_DPDT						0x300

//PSD Init
#define	ODM_PSDREG					0x808

//92D Path Div
#define	PATHDIV_REG					0xB30
#define	PATHDIV_TRI					0xBA0

enum pwr_track_control_method {
	BBSWING,
	TXAGC,
	MIX_MODE
};

void rtl8821au_dm_write_dig(struct rtl_priv *rtlpriv, u8 current_igi);

#endif
