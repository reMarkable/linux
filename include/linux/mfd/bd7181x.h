/**
 * @file bd7181x.h  ROHM BD71815GW/BD71817GW header file
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author yanglsh@embest-tech.com
 */

#ifndef __LINUX_MFD_BD7181X_H
#define __LINUX_MFD_BD7181X_H

#include <linux/regmap.h>

// LDO5VSEL_EQ_H
// define to 1 when LDO5VSEL connect to High
// define to 0 when LDO5VSEL connect to Low
#define LDO5VSEL_EQ_H		1

#ifndef LDO5VSEL_EQ_H
	#error define LDO5VSEL_EQ_H to 1 when connect to High, to 0 when connect to Low
#else
	#if LDO5VSEL_EQ_H == 1
		#define BD7181X_REG_LDO5_VOLT BD7181X_REG_LDO5_VOLT_H
	#elif LDO5VSEL_EQ_H == 0
		#define BD7181X_REG_LDO5_VOLT BD7181X_REG_LDO5_VOLT_L
	#else
		#error  Define LDO5VSEL_EQ_H only to 0 or 1
	#endif
#endif

enum {
	BD7181X_BUCK1	=	0,
	BD7181X_BUCK2,
	BD7181X_BUCK3,
	BD7181X_BUCK4,
	BD7181X_BUCK5,
	// General Purpose
	BD7181X_LDO1,
	BD7181X_LDO2,
	BD7181X_LDO3,
	// LDOs for SD Card and SD Card Interface
	BD7181X_LDO4,
	BD7181X_LDO5,
	// LDO for DDR Reference Voltage
	BD7181X_LDODVREF,
	// LDO for Secure Non-Volatile Storage
	// BD7181X_LDOSNVS,
	// LDO for Low-Power State Retention
	BD7181X_LDOLPSR,
	BD7181X_WLED,
	BD7181X_REGULATOR_CNT,
};

#define BD7181X_SUPPLY_STATE_ENABLED    0x1

enum {
	BD7181X_REG_DEVICE		=	0,
	BD7181X_REG_PWRCTRL,
	BD7181X_REG_BUCK1_MODE,
	BD7181X_REG_BUCK2_MODE,
	BD7181X_REG_BUCK3_MODE,
	BD7181X_REG_BUCK4_MODE,
	BD7181X_REG_BUCK5_MODE,
	BD7181X_REG_BUCK1_VOLT_H,
	// 0x08
	BD7181X_REG_BUCK1_VOLT_L,
	BD7181X_REG_BUCK2_VOLT_H,
	BD7181X_REG_BUCK2_VOLT_L,
	BD7181X_REG_BUCK3_VOLT,
	BD7181X_REG_BUCK4_VOLT,
	BD7181X_REG_BUCK5_VOLT,
	BD7181X_REG_LED_CTRL,
	BD7181X_REG_LED_DIMM,
	// 0x10
	BD7181X_REG_LDO_MODE1,
	BD7181X_REG_LDO_MODE2,
	BD7181X_REG_LDO_MODE3,
	BD7181X_REG_LDO_MODE4,
	BD7181X_REG_LDO1_VOLT,
	BD7181X_REG_LDO2_VOLT,
	BD7181X_REG_LDO3_VOLT,
	BD7181X_REG_LDO4_VOLT,
	// 0x18
	BD7181X_REG_LDO5_VOLT_H,
	BD7181X_REG_LDO5_VOLT_L,
	BD7181X_REG_BUCK_PD_DIS,
	BD7181X_REG_LDO_PD_DIS,
	BD7181X_REG_GPO,
	BD7181X_REG_OUT32K,
	BD7181X_REG_SEC,
	BD7181X_REG_MIN,
	// 0x20
	BD7181X_REG_HOUR,
	BD7181X_REG_WEEK,
	BD7181X_REG_DAY,
	BD7181X_REG_MONTH,
	BD7181X_REG_YEAR,
	BD7181X_REG_ALM0_SEC,

	// 0x2C
	BD7181X_REG_ALM1_SEC		=	0x2C,

	// 0x33
	BD7181X_REG_ALM0_MASK		=	0x33,
	BD7181X_REG_ALM1_MASK,
	BD7181X_REG_ALM2,
	BD7181X_REG_TRIM,
	BD7181X_REG_CONF,
	// 0x38
	BD7181X_REG_SYS_INIT,
	BD7181X_REG_CHG_STATE,
	BD7181X_REG_CHG_LAST_STATE,
	BD7181X_REG_BAT_STAT,
	BD7181X_REG_DCIN_STAT,
	BD7181X_REG_VSYS_STAT,
	BD7181X_REG_CHG_STAT,
	BD7181X_REG_CHG_WDT_STAT,
	// 0x40
	BD7181X_REG_BAT_TEMP,
	BD7181X_REG_IGNORE_0,
	BD7181X_REG_INHIBIT_0,
	BD7181X_REG_DCIN_CLPS,
	BD7181X_REG_VSYS_REG,
	BD7181X_REG_VSYS_MAX,
	BD7181X_REG_VSYS_MIN,
	BD7181X_REG_CHG_SET1,
	// 0x48
	BD7181X_REG_CHG_SET2,
	BD7181X_REG_CHG_WDT_PRE,
	BD7181X_REG_CHG_WDT_FST,
	BD7181X_REG_CHG_IPRE,
	BD7181X_REG_CHG_IFST,
	BD7181X_REG_CHG_IFST_TERM,
	BD7181X_REG_CHG_VPRE,
	BD7181X_REG_CHG_VBAT_1,
	// 0x50
	BD7181X_REG_CHG_VBAT_2,
	BD7181X_REG_CHG_VBAT_3,
	BD7181X_REG_CHG_LED_1,
	BD7181X_REG_VF_TH,
	BD7181X_REG_BAT_SET_1,
	BD7181X_REG_BAT_SET_2,
	BD7181X_REG_BAT_SET_3,
	BD7181X_REG_ALM_VBAT_TH_U,
	// 0x58
	BD7181X_REG_ALM_VBAT_TH_L,
	BD7181X_REG_ALM_DCIN_TH,
	BD7181X_REG_ALM_VSYS_TH,
	BD7181X_REG_VM_IBAT_U,
	BD7181X_REG_VM_IBAT_L,
	BD7181X_REG_VM_VBAT_U,
	BD7181X_REG_VM_VBAT_L,
	BD7181X_REG_VM_BTMP,
	// 0x60
	BD7181X_REG_VM_VTH,
	BD7181X_REG_VM_DCIN_U,
	BD7181X_REG_VM_DCIN_L,
	BD7181X_REG_VM_VSYS,
	BD7181X_REG_VM_VF,
	BD7181X_REG_VM_OCI_PRE_U,
	BD7181X_REG_VM_OCI_PRE_L,
	BD7181X_REG_VM_OCV_PRE_U,
	// 0x68
	BD7181X_REG_VM_OCV_PRE_L,
	BD7181X_REG_VM_OCI_PST_U,
	BD7181X_REG_VM_OCI_PST_L,
	BD7181X_REG_VM_OCV_PST_U,
	BD7181X_REG_VM_OCV_PST_L,
	BD7181X_REG_VM_SA_VBAT_U,
	BD7181X_REG_VM_SA_VBAT_L,
	BD7181X_REG_VM_SA_IBAT_U,
	// 0x70
	BD7181X_REG_VM_SA_IBAT_L,
	BD7181X_REG_CC_CTRL,
	BD7181X_REG_CC_BATCAP1_TH_U,
	BD7181X_REG_CC_BATCAP1_TH_L,
	BD7181X_REG_CC_BATCAP2_TH_U,
	BD7181X_REG_CC_BATCAP2_TH_L,
	BD7181X_REG_CC_BATCAP3_TH_U,
	BD7181X_REG_CC_BATCAP3_TH_L,
	// 0x78
	BD7181X_REG_CC_STAT,
	BD7181X_REG_CC_CCNTD_3,
	BD7181X_REG_CC_CCNTD_2,
	BD7181X_REG_CC_CCNTD_1,
	BD7181X_REG_CC_CCNTD_0,
	BD7181X_REG_CC_CURCD_U,
	BD7181X_REG_CC_CURCD_L,
	BD7181X_REG_VM_OCUR_THR_1,
	// 0x80
	BD7181X_REG_VM_OCUR_DUR_1,
	BD7181X_REG_VM_OCUR_THR_2,
	BD7181X_REG_VM_OCUR_DUR_2,
	BD7181X_REG_VM_OCUR_THR_3,
	BD7181X_REG_VM_OCUR_DUR_3,
	BD7181X_REG_VM_OCUR_MON,
	BD7181X_REG_VM_BTMP_OV_THR,
	BD7181X_REG_VM_BTMP_OV_DUR,
	// 0x88
	BD7181X_REG_VM_BTMP_LO_THR,
	BD7181X_REG_VM_BTMP_LO_DUR,
	BD7181X_REG_VM_BTMP_MON,
	BD7181X_REG_INT_EN_01,
	// 0x95
	BD7181X_REG_INT_EN_11		=	0x95,
	// 0x96
	BD7181X_REG_INT_EN_12		=	0x96,
	BD7181X_REG_INT_STAT,

	// 0x98
	BD7181X_REG_INT_STAT_01,
	BD7181X_REG_INT_STAT_02,
	BD7181X_REG_INT_STAT_03,
	BD7181X_REG_INT_STAT_04,
	BD7181X_REG_INT_STAT_05,
	BD7181X_REG_INT_STAT_06,
	BD7181X_REG_INT_STAT_07,
	BD7181X_REG_INT_STAT_08,

	// 0xA0
	BD7181X_REG_INT_STAT_09,
	BD7181X_REG_INT_STAT_10,
	BD7181X_REG_INT_STAT_11,
	BD7181X_REG_INT_STAT_12,
	BD7181X_REG_INT_UPDATE,

	// 0xC0
	BD7181X_REG_VM_VSYS_U		=	0xC0,
	BD7181X_REG_VM_VSYS_L,
	BD7181X_REG_VM_SA_VSYS_U,
	BD7181X_REG_VM_SA_VSYS_L,

	// 0xD0
	BD7181X_REG_VM_SA_IBAT_MIN_U	=	0xD0,
	BD7181X_REG_VM_SA_IBAT_MIN_L,
	BD7181X_REG_VM_SA_IBAT_MAX_U,
	BD7181X_REG_VM_SA_IBAT_MAX_L,
	BD7181X_REG_VM_SA_VBAT_MIN_U,
	BD7181X_REG_VM_SA_VBAT_MIN_L,
	BD7181X_REG_VM_SA_VBAT_MAX_U,
	BD7181X_REG_VM_SA_VBAT_MAX_L,
	BD7181X_REG_VM_SA_VSYS_MIN_U,
	BD7181X_REG_VM_SA_VSYS_MIN_L,
	BD7181X_REG_VM_SA_VSYS_MAX_U,
	BD7181X_REG_VM_SA_VSYS_MAX_L,
	BD7181X_REG_VM_SA_MINMAX_CLR,

	// 0xE0
	BD7181X_REG_REX_CCNTD_3		=	0xE0,
	BD7181X_REG_REX_CCNTD_2,
	BD7181X_REG_REX_CCNTD_1,
	BD7181X_REG_REX_CCNTD_0,
	BD7181X_REG_REX_SA_VBAT_U,
	BD7181X_REG_REX_SA_VBAT_L,
	BD7181X_REG_REX_CTRL_1,
	BD7181X_REG_REX_CTRL_2,
	BD7181X_REG_FULL_CCNTD_3,
	BD7181X_REG_FULL_CCNTD_2,
	BD7181X_REG_FULL_CCNTD_1,
	BD7181X_REG_FULL_CCNTD_0,
	BD7181X_REG_FULL_CTRL,

	// 0xF0
	BD7181X_REG_CCNTD_CHG_3		=	0xF0,
	BD7181X_REG_CCNTD_CHG_2,

	// 0xFE
	BD7181X_REG_TEST_MODE		=	0xFE,
	BD7181X_MAX_REGISTER,
};

/* BD7181X_REG_BUCK1_MODE bits */
#define BUCK1_RAMPRATE_MASK		0xC0
#define BUCK1_RAMPRATE_10P00MV	0x0
#define BUCK1_RAMPRATE_5P00MV	0x1
#define BUCK1_RAMPRATE_2P50MV	0x2
#define BUCK1_RAMPRATE_1P25MV	0x3

/* BD7181X_REG_BUCK1_VOLT_H bits */
#define BUCK1_DVSSEL			0x80
#define BUCK1_STBY_DVS			0x40
#define BUCK1_H_MASK			0x3F
#define BUCK1_H_DEFAULT			0x14

/* BD7181X_REG_BUCK1_VOLT_L bits */
#define BUCK1_L_MASK			0x3F
#define BUCK1_L_DEFAULT			0x14

/* BD7181X_REG_BUCK2_VOLT_H bits */
#define BUCK2_DVSSEL			0x80
#define BUCK2_STBY_DVS			0x40
#define BUCK2_H_MASK			0x3F
#define BUCK2_H_DEFAULT			0x14

/* BD7181X_REG_BUCK2_VOLT_L bits */
#define BUCK2_L_MASK			0x3F
#define BUCK2_L_DEFAULT			0x14

/* BD7181X_REG_LDO1_CTRL bits */
#define LDO1_EN					0x01
#define LDO2_EN					0x02
#define LDO3_EN					0x04
#define DVREF_EN				0x08
#define VOSNVS_SW_EN				0x10
#define VOLT_MASK				0x3F

/* BD7181X_REG_OUT32K bits */
#define OUT32K_EN				0x01
#define OUT32K_MODE				0x02

/* BD7181X_REG_BAT_STAT bits */
#define BAT_DET					0x20
#define BAT_DET_OFFSET				5
#define BAT_DET_DONE				0x10
#define VBAT_OV					0x08
#define DBAT_DET				0x01

/* BD7181X_REG_VBUS_STAT bits */
#define VBUS_DET				0x01

#define BUCK1_RAMPRATE_10MV_US			0x0
#define BUCK1_RAMPRATE_5MV_US			0x1
#define BUCK1_RAMPRATE_2P5MV_US			0x2
#define BUCK1_RAMPRATE_1P25MV_US		0x3a

/* BD7181X_REG_ALM0_MASK bits */
#define A0_ONESEC				0x80

/* BD7181X_REG_INT_EN_00 bits */
#define ALMALE					0x1

/* BD7181X_REG_INT_STAT_03 bits */
#define DCIN_MON_DET				0x02
#define DCIN_MON_RES				0x01
#define POWERON_LONG				0x04
#define POWERON_MID					0x08
#define POWERON_SHORT				0x10
#define POWERON_PRESS				0x20


/* BD71805_REG_INT_STAT_08 bits */
#define VBAT_MON_DET				0x02
#define VBAT_MON_RES				0x01

/* BD71805_REG_INT_STAT_11 bits */
#define	INT_STAT_11_VF_DET			0x80
#define	INT_STAT_11_VF_RES			0x40
#define	INT_STAT_11_VF125_DET		0x20
#define	INT_STAT_11_VF125_RES		0x10
#define	INT_STAT_11_OVTMP_DET		0x08
#define	INT_STAT_11_OVTMP_RES		0x04
#define	INT_STAT_11_LOTMP_DET		0x02
#define	INT_STAT_11_LOTMP_RES		0x01

#define VBAT_MON_DET				0x02
#define VBAT_MON_RES				0x01

/* BD7181X_REG_PWRCTRL bits */
#define RESTARTEN				0x01

/* BD7181X_REG_GPO bits */
#define READY_FORCE_LOW				0x04

/* BD7181X_REG_CHG_SET1 bits */
#define CHG_EN					0x01

/* BD7181X interrupt masks */
enum {
	BD7181X_INT_EN_01_BUCKAST_MASK	=	0x0F,
	BD7181X_INT_EN_02_DCINAST_MASK	=	0x3E,
	BD7181X_INT_EN_03_DCINAST_MASK	=	0x3F,
	BD7181X_INT_EN_04_VSYSAST_MASK	=	0xCF,
	BD7181X_INT_EN_05_CHGAST_MASK	=	0xFC,
	BD7181X_INT_EN_06_BATAST_MASK	=	0xF3,
	BD7181X_INT_EN_07_BMONAST_MASK	=	0xFE,
	BD7181X_INT_EN_08_BMONAST_MASK	=	0x03,
	BD7181X_INT_EN_09_BMONAST_MASK	=	0x07,
	BD7181X_INT_EN_10_BMONAST_MASK	=	0x3F,
	BD7181X_INT_EN_11_TMPAST_MASK	=	0xFF,
	BD7181X_INT_EN_12_ALMAST_MASK	=	0x07,
};
/* BD7181X interrupt irqs */
enum {
	BD7181X_IRQ_BUCK_01		=	0x0,
	BD7181X_IRQ_DCIN_02,
	BD7181X_IRQ_DCIN_03,
	BD7181X_IRQ_VSYS_04,
	BD7181X_IRQ_CHARGE_05,
	BD7181X_IRQ_BAT_06,
	BD7181X_IRQ_BAT_MON_07,
	BD7181X_IRQ_BAT_MON_08,
	BD7181X_IRQ_BAT_MON_09,
	BD7181X_IRQ_BAT_MON_10,
	BD7181X_IRQ_TEMPERATURE_11,
	BD7181X_IRQ_ALARM_12,
};

/* BD7181X_REG_INT_EN_12 bits */
#define ALM0					0x1

/* BD7181X_REG_HOUR bits */
#define HOUR_24HOUR				0x80

/* BD7181X_REG_CC_CTRL bits */
#define CCNTRST					0x80
#define CCNTENB					0x40
#define CCCALIB					0x20

/* BD7181X_REG_CHG_SET1 bits */
#define WDT_AUTO				0x40

/* BD7181X_REG_CC_CURCD */
#define CURDIR_Discharging			0x8000

/* BD7181X_REG_VM_SA_IBAT */
#define IBAT_SA_DIR_Discharging			0x8000

/* BD7181X_REG_VM_SA_MINMAX_CLR bits */
#define VSYS_SA_MIN_CLR				0x10
#define VBAT_SA_MIN_CLR				0x01

/* BD7181X_REG_REX_CTRL_1 bits */
#define REX_CLR					0x10

/* BD7181X_REG_REX_CTRL_1 bits */
#define REX_PMU_STATE_MASK			0x04

/* BD7181X_REG_FULL_CTRL bits */
#define FULL_CLR				0x10

/* BD7181X_REG_LED_CTRL bits */
#define CHGDONE_LED_EN				0x10

/** @brief charge state enumuration */
enum CHG_STATE {
	CHG_STATE_SUSPEND = 0x0,		/**< suspend state */
	CHG_STATE_TRICKLE_CHARGE,		/**< trickle charge state */
	CHG_STATE_PRE_CHARGE,			/**< precharge state */
	CHG_STATE_FAST_CHARGE,			/**< fast charge state */
	CHG_STATE_TOP_OFF,			/**< top off state */
	CHG_STATE_DONE,				/**< charge complete */
};

/** @brief rtc or alarm registers structure */
struct bd7181x_rtc_alarm {
	u8	sec;
	u8	min;
	u8	hour;
	u8	week;
	u8	day;
	u8	month;
	u8	year;
};

struct bd7181x;

/**
 * @brief Board platform data may be used to initialize regulators.
 */

struct bd7181x_board {
	struct regulator_init_data *init_data[BD7181X_REGULATOR_CNT];
	/**< regulator initialize data */
	int	gpio_intr;		/**< gpio connected to bd7181x INTB */
	int	irq_base;		/**< bd7181x sub irqs base #  */
};

/**
 * @brief bd7181x sub-driver chip access routines
 */

struct bd7181x {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct mutex io_mutex;
	unsigned int id;

	/* IRQ Handling */
	int 	chip_irq;		/**< bd7181x irq to host cpu */
	struct regmap_irq_chip_data *irq_data;

	/* Client devices */
	struct bd7181x_pmic *pmic;	/**< client device regulator */
	struct bd7181x_power *power;	/**< client device battery */

	struct bd7181x_board *of_plat_data;
	/**< Device node parsed board data */
};

static inline int bd7181x_chip_id(struct bd7181x *bd7181x)
{
	return bd7181x->id;
}


/**
 * @brief bd7181x_reg_read
 * read single register's value of bd7181x
 * @param bd7181x device to read
 * @param reg register address
 * @return register value if success
 *         error number if fail
 */
static inline int bd7181x_reg_read(struct bd7181x *bd7181x, u8 reg)
{
	int r, val;

	r = regmap_read(bd7181x->regmap, reg, &val);
	if (r < 0) {
		return r;
	}
	return val;
}

/**
 * @brief bd7181x_reg_write
 * write single register of bd7181x
 * @param bd7181x device to write
 * @param reg register address
 * @param val value to write
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd7181x_reg_write(struct bd7181x *bd7181x, u8 reg,
		unsigned int val)
{
	return regmap_write(bd7181x->regmap, reg, val);
}

/**
 * @brief bd7181x_set_bits
 * set bits in one register of bd7181x
 * @param bd7181x device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */
static inline int bd7181x_set_bits(struct bd7181x *bd7181x, u8 reg,
		u8 mask)
{
	return regmap_update_bits(bd7181x->regmap, reg, mask, mask);
}

/**
 * @brief bd7181x_clear_bits
 * clear bits in one register of bd7181x
 * @param bd7181x device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd7181x_clear_bits(struct bd7181x *bd7181x, u8 reg,
		u8 mask)
{
	return regmap_update_bits(bd7181x->regmap, reg, mask, 0);
}

/**
 * @brief bd7181x_update_bits
 * update bits in one register of bd7181x
 * @param bd7181x device to read
 * @param reg register address
 * @param mask mask bits
 * @param val value to update
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd7181x_update_bits(struct bd7181x *bd7181x, u8 reg,
					   u8 mask, u8 val)
{
	return regmap_update_bits(bd7181x->regmap, reg, mask, val);
}

/**
 * @brief bd7181x platform data type
 */
struct bd7181x_gpo_plat_data {
	u32 mode;		///< gpo output mode
	int gpio_base;		///< base gpio number in system
};

#define BD7181X_DBG0		0x0001
#define BD7181X_DBG1		0x0002
#define BD7181X_DBG2		0x0004
#define BD7181X_DBG3		0x0008

extern unsigned int bd7181x_debug_mask;
#define bd7181x_debug(debug, fmt, arg...) do { if(debug & bd7181x_debug_mask) printk("BD7181x:" fmt, ##arg);} while(0)

#endif /* __LINUX_MFD_BD7181X_H */
