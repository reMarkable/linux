/*
 * MAX77818 Driver Core
 *
 * Copyright (C) 2014 Maxim Integrated
 * TaiEup Kim <clark.kim@maximintegrated.com>
 *
 * Copyright and License statement to be determined with Customer.
 * GNU Public License version 2 requires software code to be
 * publically open source if the code is to be statically linked with
 * the Linux kernel binary object.
 */

#include <linux/mutex.h>

#ifndef __MAX77818_MFD_H__
#define __MAX77818_MFD_H__

#define MAX77818_REGULATOR_NAME		"max77818-regulator"
#define MAX77818_CHARGER_NAME		"max77818-charger"
#define MAX77818_FUELGAUGE_NAME		"max77818-fuelgauge"

#define REG_INTSRC			0x22
#define REG_INTSRCMASK			0x23
#define BIT_CHGR_INT			BIT (0)
#define BIT_FG_INT			BIT (1)
#define BIT_SYS_INT			BIT (2)

#define REG_SYSINTSRC     	 	0x24
#define REG_SYSINTMASK			0x26
#define BIT_SYSUVLO_INT			BIT (0)
#define BIT_SYSOVLO_INT			BIT (1)
#define BIT_TSHDN_INT			BIT (2)
#define BIT_TM_INT			BIT (7)

#define REG_CHARGER_INT			0xB0
#define REG_CHARGER_INT_MASK		0xB1

#define BIT_CHG_BYP_I			BIT (0)
#define BIT_CHG_BATP_I			BIT (2)
#define BIT_CHG_BAT_I			BIT (3)
#define BIT_CHG_CHG_I			BIT (4)
#define BIT_CHG_WCIN_I			BIT (5)
#define BIT_CHG_CHGIN_I			BIT (6)
#define BIT_CHG_AICL_I			BIT (7)

/* Chip Interrupts */
enum {
	MAX77818_CHGR_INT = 0,
	MAX77818_FG_INT,
	MAX77818_SYS_INT,

	MAX77818_SYS_IRQ_START,
	MAX77818_SYS_IRQ_UVLO = MAX77818_SYS_IRQ_START,
	MAX77818_SYS_IRQ_OVLO,
	MAX77818_SYS_IRQ_TSHDN,
	MAX77818_SYS_IRQ_TM,

	MAX77818_CHG_IRQ_START,
	MAX77818_CHG_IRQ_BYP_I = MAX77818_CHG_IRQ_START,
	MAX77818_CHG_IRQ_BATP_I,
	MAX77818_CHG_IRQ_BAT_I,
	MAX77818_CHG_IRQ_CHG_I,
	MAX77818_CHG_IRQ_WCIN_I,
	MAX77818_CHG_IRQ_CHGIN_I,
	MAX77818_CHG_IRQ_AICL_I,

	MAX77818_NUM_OF_INTS,
};

enum {
	SYS_IRQ_UVLO = 0,
	SYS_IRQ_OVLO,
	SYS_IRQ_TSHDN,
	SYS_IRQ_TM,

	CHG_IRQ_BYP_I = 0,
	CHG_IRQ_BATP_I,
	CHG_IRQ_BAT_I,
	CHG_IRQ_CHG_I,
	CHG_IRQ_WCIN_I,
	CHG_IRQ_CHGIN_I,
	CHG_IRQ_AICL_I,

	FG_IRQ_ALERT = 0,
};


struct max77818_dev {
	struct device *dev;
	int irq;

	struct regmap_irq_chip_data *irqc_intsrc;
	struct regmap_irq_chip_data *irqc_sys;
	struct regmap_irq_chip_data *irqc_chg;

	struct i2c_client *pmic;
	struct i2c_client *chg;
	struct i2c_client *fg;

	struct regmap *regmap_pmic;
	struct regmap *regmap_chg;
	struct regmap *regmap_fg;

	struct mutex lock;
};

#endif /* !__MAX77818_MFD_H__ */

