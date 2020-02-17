/*
* Maxim MAX77818 Charger Driver
*
* Copyright (C) 2014 Maxim Integrated Product
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This driver is based on max77843-charger.c
*/

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/power_supply.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/gpio/consumer.h>
#include <linux/suspend.h>

/* Register map */
#define REG_CHG_INT 			0xB0
#define REG_CHG_INT_MASK		0xB1
#define BIT_AICL			BIT(7)
#define BIT_CHGIN			BIT(6)
#define BIT_WCIN              		BIT(5)
#define BIT_CHG                 	BIT(4)
#define BIT_BAT                 	BIT(3)
#define BIT_BATP                	BIT(2)
#define BIT_BYP				BIT(0)

#define REG_CHG_INT_OK			0xB2
#define BIT_AICL_OK			BIT(7)
#define BIT_CHGIN_OK			BIT(6)
#define BIT_WCIN_OK			BIT(5)
#define BIT_CHG_OK			BIT(4)
#define BIT_BAT_OK			BIT(3)
#define BIT_BATP_OK			BIT(2)
#define BIT_BYP_OK			BIT(0)

#define REG_CHG_DTLS_00			0xB3
#define BIT_CHGIN_DTLS			GENMASK(6, 5)
#define BIT_WCIN_DTLS       		GENMASK(4, 3)
#define BIT_BATP_DTLS       		BIT(0)

#define REG_CHG_DTLS_01			0xB4
#define BIT_TREG        		BIT(7)
#define BIT_BAT_DTLS        		GENMASK(6, 4)
#define BIT_CHG_DTLS       	 	GENMASK(3, 0)

#define REG_CHG_DTLS_02			0xB5
#define BIT_BYP_DTLS			GENMASK(3, 0)
#define BIT_BCKNegILIM			BIT(2)
#define BIT_BSTILIM			BIT(1)
#define BIT_OTGILIM			BIT(0)

#define REG_CHG_CNFG_00			0xB7
#define BIT_OTG_CTRL			BIT(7)
#define BIT_DISIBS			BIT(6)
#define BIT_SPREAD			BIT(5)
#define BIT_WDTEN			BIT(4)
#define BIT_MODE			GENMASK(3, 0)
#define BIT_MODE_BOOST			BIT(3)
#define BIT_MODE_BUCK			BIT(2)
#define BIT_MODE_OTG			BIT(1)
#define BIT_MODE_CHARGER		BIT(0)
#define MODE_ALL_OFF			0x00
#define MODE_OTG_BUCK_BOOST		0x0F
#define MODE_CHARGER_BUCK		0x05

/* Read back charger status ranges */
#define MODE_ALL_OFF_MIN		0x00
#define MODE_ALL_OFF_MAX		0x03
#define MODE_CHARGER_MIN		0x04
#define MODE_CHARGER_MAX		0x07
#define MODE_OTG_MIN			0x0e
#define MODE_OTG_MAX			0x0f

#define REG_CHG_CNFG_01			0xB8
#define BIT_PQEN			BIT(7)
#define BIT_LSEL			BIT(6)
#define BIT_CHG_RSTRT			GENMASK(5, 4)
#define SHIFT_CHG_RSTRT			4
#define BIT_FSW				BIT(3)
#define BIT_FCHGTIME			GENMASK(2, 0)

#define REG_CHG_CNFG_02			0xB9
#define BIT_OTG_ILIM			GENMASK(7, 6)
#define BIT_CHG_CC			GENMASK(5, 0)

#define REG_CHG_CNFG_03			0xBA
#define BIT_ILIM			GENMASK(7, 6)
#define BIT_TO_TIME			GENMASK(5, 3)
#define SHIFT_TO_TIME			3
#define BIT_TO_ITH			GENMASK(2, 0)

#define REG_CHG_CNFG_04			0xBB
#define SHIFT_MINVSYS			6

#define REG_CHG_CNFG_06			0xBD
#define BIT_CHGPROT			GENMASK(3, 2)
#define BIT_WDTCLR			GENMASK(1, 0)

#define REG_CHG_CNFG_07			0xBE
#define BIT_REGTEMP			GENMASK(6, 5)

#define REG_CHG_CNFG_09			0xC0
#define BIT_CHGIN_ILIM			GENMASK(6, 0)

#define REG_CHG_CNFG_10			0xC1
#define BIT_WCIN_ILIM			GENMASK(5, 0)

#define REG_CHG_CNFG_11			0xC2
#define BIT_VBYPSET			GENMASK(6, 0)

#define REG_CHG_CNFG_12			0xC3
#define BIT_WCINSEL			BIT(6)
#define BIT_CHGINSEL			BIT(5)
#define BIT_VCHGIN_REG			GENMASK(4, 3)
#define BIT_B2SOVRC			GENMASK(2, 0)

enum {
	WCIN_DTLS_UVLO,
	WCIN_DTLS_INVALID_01,
	WCIN_DTLS_OVLO,
	WCIN_DTLS_VALID,
};

enum {
	CHGIN_DTLS_UVLO,
	CHGIN_DTLS_INVALID_01,
	CHGIN_DTLS_OVLO,
	CHGIN_DTLS_VALID,
};

enum {
	CHG_DTLS_PREQUAL,
	CHG_DTLS_FASTCHARGE_CC,
	CHG_DTLS_FASTCHARGE_CV,
	CHG_DTLS_TOPOFF,
	CHG_DTLS_DONE,
	CHG_DTLS_RESEVRED_05,
	CHG_DTLS_OFF_TIMER_FAULT,
	CHG_DTLS_OFF_SUSPEND,
	CHG_DTLS_OFF_INPUT_INVALID,
	CHG_DTLS_RESERVED_09,
	CHG_DTLS_OFF_JUCTION_TEMP,
	CHG_DTLS_OFF_WDT_EXPIRED,
};

enum {
	BAT_DTLS_NO_BATTERY,
	BAT_DTLS_RESERVED_01,
	BAT_DTLS_TIMER_FAULT,
	BAT_DTLS_OKAY,
	BAT_DTLS_OKAY_LOW,
	BAT_DTLS_OVERVOLTAGE,
	BAT_DTLS_OVERCURRENT,
	BAT_DTLS_RESERVED_07,
};

enum {
	CHG_INT_BYP_I,
	CHG_INT_BATP_I,
	CHG_INT_BAT_I,
	CHG_INT_CHG_I,
	CHG_INT_WCIN_I,
	CHG_INT_CHGIN_I,
	CHG_INT_AICL_I,
};

struct max77818_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy_chg;
	struct mutex lock;
	struct work_struct irq_work;

	int irq;
	int chgin_irq;
	int wcin_irq;
	int chg_int;

	u8 dtls[3];

	int health;
	int status;
	int charge_type;
	int charger_mode; /* OTG SUPPLY/CHARGER */

	int fast_charge_timer;
	int fast_charge_current;
	int topoff_current;
	int topoff_timer;
	int restart_threshold;
	int termination_voltage;
	int vsys_min;
	int input_current_limit_chgin;
	int input_current_limit_wcin;

	struct gpio_desc *chgin_stat_gpio;
	struct gpio_desc *wcin_stat_gpio;
};

static enum power_supply_property max77818_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGER_MODE,
	POWER_SUPPLY_PROP_STATUS_EX,
};

static bool max77818_charger_chgin_present(struct max77818_charger *chg)
{
	u32 chg_int_ok = 0;
	int ret;

	/* Try to read from the device first, but if the charger device
	 * is offline, try to read the chg status gpios */
	if(!IS_ERR_OR_NULL(chg->regmap)) {
		ret = regmap_read(chg->regmap, REG_CHG_INT_OK, &chg_int_ok);
		if (!ret) {
			if (chg_int_ok & BIT_CHGIN_OK) {
				return true;
			} else {
				/* check whether charging or not in the UVLO condition */
				if (((chg->dtls[0] & BIT_CHGIN_DTLS) == 0) &&
				    ((chg_int_ok & BIT_WCIN_OK) == 0) &&
				    (((chg->dtls[1] & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CC) ||
				     ((chg->dtls[1] & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CV))) {
					return true;
				} else {
					return false;
				}
			}
		}
	}

	/* chg->regmap is not initialized, or read from device failed
	 * Use GPIO
	 */
	if (chg->chgin_stat_gpio) {
		ret = gpiod_get_value_cansleep(chg->chgin_stat_gpio);
		if (ret < 0)
			dev_err(chg->dev,
				"failed to read chgin-stat-gpio: %d\n",
				ret);
		return (ret > 0);

	}
	else {
		dev_warn(chg->dev,
			"chgin-stat-gpio not configured, connection "
			"status not available\n");
		return false;
	}
}

static bool max77818_charger_wcin_present(struct max77818_charger *chg)
{
	u32 chg_int_ok = 0;
	int ret;

	/* Try to read from the device first, but if the charger device
	 * is offline, try to read the chg status gpios */
	if(!IS_ERR_OR_NULL(chg->regmap)) {
		ret = regmap_read(chg->regmap, REG_CHG_INT_OK, &chg_int_ok);
		if (!ret) {
			if (chg_int_ok & BIT_WCIN_OK) {
				return true;
			} else {
				/* check whether charging or not in the UVLO condition */
				if (((chg->dtls[0] & BIT_WCIN_DTLS) == 0) &&
				    ((chg_int_ok & BIT_CHGIN_OK) == 0) &&
				    (((chg->dtls[1] & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CC) ||
				     ((chg->dtls[1] & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CV))) {
					return true;
				} else {
					return false;
				}
			}
		}
	}

	/* chg->regmap is not initialized, or read from device failed
	 * Use GPIO
	 */
	if (chg->wcin_stat_gpio) {
		ret = gpiod_get_value_cansleep(chg->wcin_stat_gpio);
		if (ret < 0)
			dev_err(chg->dev,
				"failed to read wcin-stat-gpio: %d\n",
				ret);
		return (ret > 0);
	}
	else {
		dev_warn(chg->dev,
			"wcin-stat-gpio not configured, connection "
			"status not available\n");
		return false;
	}
}

static int max77818_charger_get_input_current(struct max77818_charger *chg,
					      int *input_current)
{
	int quotient, remainder;
	int steps[3] = { 0, 33, 67 };
	u32 val = 0;
	int ret;

	if(IS_ERR_OR_NULL(chg->regmap)) {
		dev_warn(chg->dev, "unable to read from charger device\n");
		return -ENODEV;
	}

	if (!max77818_charger_chgin_present(chg) &&
	    max77818_charger_wcin_present(chg)) {
		/*
		 * When the wireless charging input is the only one currently
		 * active, the configured max input current for the wireless
		 * charging input is returned
		 */
		ret = regmap_read(chg->regmap, REG_CHG_CNFG_10, &val);
		if (ret) {
			dev_warn(chg->dev,
				 "failed to read CNFG_10: %d\n",
				 ret);
			return ret;
		}

		if (val <= 3)
			*input_current = 60;
		else
			*input_current = 60 + (val - 3) * 20;
	} else {
		/*
		 * Just return the max wired charging input current in all
		 * cases where the wireless charging input is not the only
		 * current active charging input
		 */
		ret = regmap_read(chg->regmap, REG_CHG_CNFG_09, &val);
		if (ret) {
			dev_warn(chg->dev,
				 "failed to read CNFG_09: %d\n",
				 ret);
			return ret;
		}

		quotient = val / 3;
		remainder = val % 3;

		if ((val & BIT_CHGIN_ILIM) < 3)
			*input_current = 100;
		else if ((val & BIT_CHGIN_ILIM) > 0x78)
			*input_current = 4000;
		else
			*input_current = quotient * 100 + steps[remainder];
	}

	return 0;
}

static int
max77818_charger_set_chgin_current_limit(struct max77818_charger *chg,
					 int input_current)
{
	int quotient, remainder;
	u8 val = 0;

	if(IS_ERR_OR_NULL(chg->regmap)) {
		dev_warn(chg->dev, "unable to read from charger device\n");
		return -ENODEV;
	}

	if (input_current) {
		quotient = input_current / 100;
		remainder = input_current % 100;

		if (remainder >= 67)
			val |= (quotient * 3) + 2;
		else if (remainder >= 33)
			val |= (quotient * 3) + 1;
		else if (remainder < 33)
			val |= quotient * 3;
	}

	return regmap_update_bits(chg->regmap, REG_CHG_CNFG_09,
				  BIT_CHGIN_ILIM, val);
}

static int
max77818_charger_set_wcin_current_limit(struct max77818_charger *chg,
					int input_current)
{
	u8 val = 0;

	if(IS_ERR_OR_NULL(chg->regmap)) {
		dev_warn(chg->dev, "unable to read from charger device\n");
		return -ENODEV;
	}

	if (input_current) {
		if (input_current < 60)
			input_current = 60;
		else if (input_current > 1260)
			input_current = 1260;

		val = DIV_ROUND_UP(input_current - 60, 20) + 3;
	}

	return regmap_update_bits(chg->regmap, REG_CHG_CNFG_10,
				  BIT_WCIN_ILIM, val);
}

static int max77818_charger_set_enable(struct max77818_charger *chg, int en)
{
	u8 mode;
	int ret;

	if(IS_ERR_OR_NULL(chg->regmap)) {
		dev_warn(chg->dev, "unable to read from charger device\n");
		return -ENODEV;
	}

	/* If enable = 0, just shut off charging/otg power */
	if (!en) {
		mode = MODE_ALL_OFF;
		goto update_mode;
	}

	/*
	 * Depending on configured mode, turn on charging or OTG
	 * power.
	 */
	if (chg->charger_mode == POWER_SUPPLY_MODE_CHARGER) {
		mode = MODE_CHARGER_BUCK;
	} else if (chg->charger_mode == POWER_SUPPLY_MODE_OTG_SUPPLY) {
		mode = MODE_OTG_BUCK_BOOST;
	} else {
		dev_err(chg->dev, "invalid charger_mode %d\n",
			chg->charger_mode);
		return -EINVAL;
	}

update_mode:
	dev_info(chg->dev, "set MODE bits to 0x%x", mode);
	ret = regmap_update_bits(chg->regmap, REG_CHG_CNFG_00, BIT_MODE, mode);
	if (ret) {
		dev_err(chg->dev, "failed to update MODE bits: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77818_charger_get_charger_mode(struct max77818_charger *chg)
{
	int ret;
	u32 read_val;

	dev_dbg(chg->dev, "Trying to read current charger mode from device\n");

	ret = regmap_read(chg->regmap, REG_CHG_CNFG_00, &read_val);
	if (ret) {
		dev_err(chg->dev, "failed to read CNFG_00: %d\n", ret);
		return ret;
	}
	dev_dbg(chg->dev, "Read raw charger_mode register: 0x%02x\n", read_val);
	dev_dbg(chg->dev, "Read charger_mode 0x%02x\n", (read_val & BIT_MODE));

	switch(read_val & BIT_MODE)
	{
	case MODE_ALL_OFF_MIN ... MODE_ALL_OFF_MAX:
		chg->charger_mode = POWER_SUPPLY_MODE_ALL_OFF;
		break;
	case MODE_CHARGER_MIN ... MODE_CHARGER_MAX:
		chg->charger_mode = POWER_SUPPLY_MODE_CHARGER;
		break;
	case MODE_OTG_MIN ... MODE_OTG_MAX:
		chg->charger_mode = POWER_SUPPLY_MODE_OTG_SUPPLY;
		break;
	default:
		dev_warn(chg->dev,
			 "Unknown charger_mode read from device: 0x%02x\n",
			 (read_val & BIT_MODE));
	}

	return 0;
}

static int max77818_charger_initialize(struct max77818_charger *chg)
{
	struct device *dev = chg->dev;
	u8 val, tmpval;
	int ret;

	if(IS_ERR_OR_NULL(chg->regmap)) {
		dev_warn(chg->dev, "unable to read from charger device\n");
		return -ENODEV;
	}

	/* unlock charger register */
	ret = regmap_update_bits(chg->regmap, REG_CHG_CNFG_06,
				 BIT_CHGPROT, BIT_CHGPROT);
	if (ret) {
		dev_err(dev, "failed to unlock: %d\n", ret);
		return ret;
	}

	/* charge current (mA) */
	ret = regmap_update_bits(chg->regmap, REG_CHG_CNFG_02, BIT_CHG_CC,
				 chg->fast_charge_current / 50);
	if (ret) {
		dev_err(dev, "failed to set charge current: %d\n", ret);
		return ret;
	}

	/* input current limit (mA) for chgin */
	ret = max77818_charger_set_chgin_current_limit(chg,
					chg->input_current_limit_chgin);
	if (ret) {
		dev_err(dev, "failed to set chgin input current: %d\n", ret);
		return ret;
	}

	/* input current limit (mA) for wcin */
	ret = max77818_charger_set_wcin_current_limit(chg,
					chg->input_current_limit_wcin);
	if (ret) {
		dev_err(dev, "failed to set wcin input current: %d\n", ret);
		return ret;
	}

	/* topoff current (mA) */
	if (chg->topoff_current < 100)
		val = 0x00;
	else if (chg->topoff_current < 200)
		val = (chg->topoff_current - 100) / 25;
	else if (chg->topoff_current < 350)
		val = chg->topoff_current / 50;
	else
		val = 0x07;

	/* topoff timer (min) */
	val |= (chg->topoff_timer / 10) << SHIFT_TO_TIME;

	ret = regmap_update_bits(chg->regmap, REG_CHG_CNFG_03,
				 BIT_TO_ITH | BIT_TO_TIME, val);
	if (ret) {
		dev_err(dev, "failed to update topoff bits: %d\n", ret);
		return ret;
	}

	/* charge restart threshold(mV) */
	if (chg->restart_threshold <= 0)
		val = 0x03; /* disable */
	else if (chg->restart_threshold > 200)
		val = 0x02; /* 200 mV */
	else
		val = (chg->restart_threshold - 100) / 50;

	/* fast-charge timer(hr) */
	if (chg->fast_charge_timer <= 0 || chg->fast_charge_timer > 16)
		tmpval = 0x00; /* disable */
	else if (chg->fast_charge_timer > 16)
		tmpval = 0x07; /* 16 hours */
	else
		tmpval = chg->fast_charge_timer / 2 - 1;

	val = val << SHIFT_CHG_RSTRT | tmpval;

	/* Enable Low Battery Prequalification Mode */
	val |= BIT_PQEN;

	ret = regmap_update_bits(chg->regmap, REG_CHG_CNFG_01,
				 BIT_CHG_RSTRT | BIT_FCHGTIME, val);
	if (ret) {
		dev_err(dev, "failed to update CNFG_01: %d\n", ret);
		return ret;
	}

	/* charge termination voltage (mV) */
	if (chg->termination_voltage < 3650)
		val =  0x00;
	else if (chg->termination_voltage <= 4325)
		val = DIV_ROUND_UP(chg->termination_voltage - 3650, 25);
	else if (chg->termination_voltage <= 4340)
		val = 0x1C;
	else if (chg->termination_voltage <= 4700)
		val = DIV_ROUND_UP(chg->termination_voltage - 3650, 25) + 1;
	else
		val = 0x2B;

	/* vsys min voltage */
	if (chg->vsys_min < 3400)
		tmpval = 0x00;
	else if (chg->vsys_min <= 3700)
		tmpval = DIV_ROUND_UP(chg->vsys_min - 3400, 100);
	else
		tmpval = 0x03;

	val = val | tmpval << SHIFT_MINVSYS;

	ret = regmap_write(chg->regmap, REG_CHG_CNFG_04, val);
	if (ret) {
		dev_err(dev, "failed to update CNFG_04: %d\n", ret);
		return ret;;
	}


	/* do initial read from device, to initialize shadow value
	 * with real value and not assume charger_mode = 0
	 */
	ret = max77818_charger_get_charger_mode(chg);
	if (ret) {
		dev_err(dev, "failed to read charger_mode from device: %d\n",
			ret);
		return ret;
	}

	return 0;
}

struct max77818_charger_status_map {
	int health;
	int status;
	int charge_type;
};

#define STATUS_MAP(_chg_dtls, _health, _status, _charge_type) \
		[CHG_DTLS_##_chg_dtls] = { \
			.health = POWER_SUPPLY_HEALTH_##_health, \
			.status = POWER_SUPPLY_STATUS_##_status, \
			.charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type, \
		}

static struct max77818_charger_status_map max77818_charger_status_map[] = {
	//	   chg_details_xx	health			status       	charge_type
	STATUS_MAP(PREQUAL,     	GOOD,			CHARGING, 	TRICKLE),
	STATUS_MAP(FASTCHARGE_CC,    	GOOD,			CHARGING,     	FAST),
	STATUS_MAP(FASTCHARGE_CV,    	GOOD,			CHARGING,     	FAST),
	STATUS_MAP(TOPOFF,           	GOOD,  		     	CHARGING,     	FAST),
	STATUS_MAP(DONE,             	GOOD,  	    	 	FULL,         	NONE),
	STATUS_MAP(OFF_TIMER_FAULT,	SAFETY_TIMER_EXPIRE,	NOT_CHARGING, 	NONE),
	STATUS_MAP(OFF_SUSPEND,  	UNKNOWN,		NOT_CHARGING, 	NONE),
	STATUS_MAP(OFF_INPUT_INVALID,   UNKNOWN,		NOT_CHARGING,   NONE),
	STATUS_MAP(OFF_JUCTION_TEMP,    UNKNOWN,		NOT_CHARGING, 	UNKNOWN),
	STATUS_MAP(OFF_WDT_EXPIRED, 	WATCHDOG_TIMER_EXPIRE,  NOT_CHARGING, 	UNKNOWN),
};

static void max77818_charger_update(struct max77818_charger *chg)
{
	u32 dtls_01;
	u8 chg_dtls;
	int ret;

	chg->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	chg->status = POWER_SUPPLY_STATUS_UNKNOWN;
	chg->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	if (!max77818_charger_chgin_present(chg) &&
	    !max77818_charger_wcin_present(chg)) {
		/* no charger present */
		return;
	}

	if(!IS_ERR_OR_NULL(chg->regmap)) {
		ret = regmap_read(chg->regmap, REG_CHG_DTLS_01, &dtls_01);
		if (!ret) {
			chg_dtls = dtls_01 & BIT_CHG_DTLS;

			chg->health = max77818_charger_status_map[chg_dtls].health;
			chg->status = max77818_charger_status_map[chg_dtls].status;
			chg->charge_type = max77818_charger_status_map[chg_dtls].charge_type;

			if (chg->health != POWER_SUPPLY_HEALTH_UNKNOWN)
				return;

			/* override health by TREG */
			if ((dtls_01 & BIT_TREG) != 0)
				chg->health = POWER_SUPPLY_HEALTH_OVERHEAT;

			return;
		}
	}

	/* chg->regmap is not initialized, or read from device failed
	 * Just do simple GPIO based check when device is not present on
	 * the I2C bus, and set the status being used by ONLINE property,
	 * being used by ...am_i_supplied() call which determines the
	 * status of other supplies dependent on this supply
	 */
	if (chg->charger_mode == POWER_SUPPLY_MODE_ALL_OFF)
		chg->status = 0;
	else if(chg->charger_mode == POWER_SUPPLY_MODE_OTG_SUPPLY)
		chg->status = max77818_charger_wcin_present(chg);
	else
		chg->status = max77818_charger_chgin_present(chg) ||
			      max77818_charger_wcin_present(chg);
}

static int max77818_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max77818_charger *chg = (struct max77818_charger*) psy->drv_data;
	int ret = 0;
	bool chgin_connected, wcin_connected;

	mutex_lock(&chg->lock);

	max77818_charger_update(chg);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (chg->status == POWER_SUPPLY_STATUS_CHARGING ||
		    chg->status == POWER_SUPPLY_STATUS_FULL)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chg->health;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chg->status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = chg->charge_type;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77818_charger_get_input_current(chg, &val->intval);
		if (ret) {
			dev_warn(chg->dev, "failed to read max current from device: %d\n",
				 ret);
			ret = -ENODEV;
			goto out;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		ret = max77818_charger_get_charger_mode(chg);
		if (ret) {
			dev_warn(chg->dev, "failed to read charger_mode from device: %d\n",
				ret);
			ret = -ENODEV;
			goto out;
		}
		val->intval = chg->charger_mode;
		break;
	case POWER_SUPPLY_PROP_STATUS_EX:
		if (chg->charger_mode == POWER_SUPPLY_MODE_OTG_SUPPLY)
			/* Charger device reports CHGIN OK in OTG mode anyway,
			 * so just ignore this and report WCIN status only when
			 * in OTG mode (charging is off anyway)
			 */
			val->intval = (max77818_charger_wcin_present(chg) << 1);
		else {
			chgin_connected = max77818_charger_chgin_present(chg);
			wcin_connected = max77818_charger_wcin_present(chg);
			val->intval = chgin_connected | (wcin_connected << 1);
		}
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

out:
	mutex_unlock(&chg->lock);
	return ret;
}

static int max77818_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct max77818_charger *chg = (struct max77818_charger *) psy->drv_data;
	int ret = 0;

	mutex_lock(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		if (val->intval == POWER_SUPPLY_MODE_CHARGER ||
		    val->intval == POWER_SUPPLY_MODE_ALL_OFF ||
		    val->intval == POWER_SUPPLY_MODE_OTG_SUPPLY) {
			chg->charger_mode = val->intval;
		} else {
			ret = -EINVAL;
			goto out;
		}

		/*
		 * Disable charger, but only if it's requested.
		 */
		if (val->intval == POWER_SUPPLY_MODE_ALL_OFF)
			max77818_charger_set_enable(chg, 0);
		else
			max77818_charger_set_enable(chg, 1);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

out:
	mutex_unlock(&chg->lock);
	return ret;
}

static int max77818_charger_property_is_writeable(struct power_supply *psy,
						  enum power_supply_property psp)
{

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		return 1;
	default:
		return 0;
	}
}

static void max77818_do_irq(struct max77818_charger *chg)
{
	struct device *dev = chg->dev;
	bool chg_input, wc_input;

	if(IS_ERR_OR_NULL(chg->regmap)) {
		dev_warn(chg->dev, "unable to read from charger device\n");
		return;
	}

	regmap_bulk_read(chg->regmap, REG_CHG_DTLS_00, chg->dtls, 3);

	switch (chg->chg_int) {
	case CHG_INT_CHGIN_I:
		/*
		 * Ignore the interrupt in case of OTG mode, as we do not
		 * want to actively manage charger on/off for OTG mode.
		 */
		if (chg->charger_mode == POWER_SUPPLY_MODE_OTG_SUPPLY)
			break;

		chg_input = max77818_charger_chgin_present(chg);
		wc_input = max77818_charger_wcin_present(chg);

		dev_dbg(dev, "CHGIN input %s\n", chg_input ? "inserted" :
							     "removed");

		/* Enable charging whenever charge input is changed,
		 * and leave it on */
		max77818_charger_set_enable(chg, 1);
		break;
	case CHG_INT_WCIN_I:
		/*
		 * Ignore the interrupt in case of OTG mode, as we do not
		 * want to actively manage charger on/off for OTG mode.
		 */
		if (chg->charger_mode == POWER_SUPPLY_MODE_OTG_SUPPLY)
			break;

		chg_input = max77818_charger_chgin_present(chg);
		wc_input = max77818_charger_wcin_present(chg);

		dev_dbg(dev, "WCIN input %s\n", wc_input ? "inserted" :
							   "removed");

		/* Enable charging whenever charge input is changed,
		 * and leave it on */
		max77818_charger_set_enable(chg, 1);
		break;
	default:
		break;
	}

	/* notify psy changed */
	power_supply_changed(chg->psy_chg);
}

static void max77818_charger_irq_work(struct work_struct *work)
{
	struct max77818_charger *chg =
		container_of(work, struct max77818_charger, irq_work);

	mutex_lock(&chg->lock);
	max77818_do_irq(chg);
	mutex_unlock(&chg->lock);

	return;
}

static irqreturn_t max77818_charger_chgin_isr(int irq, void *data)
{
	struct max77818_charger *chg = data;

	chg->chg_int = CHG_INT_CHGIN_I;
	schedule_work(&chg->irq_work);

	return IRQ_HANDLED;
}

static irqreturn_t max77818_charger_wcin_isr(int irq, void *data)
{
	struct max77818_charger *chg = data;

	chg->chg_int = CHG_INT_WCIN_I;
	schedule_work(&chg->irq_work);

	return IRQ_HANDLED;
}

static irqreturn_t max77818_charger_handler(int irq, void *data)
{
	/*
	 * This IRQ handler needs to do nothing, as it's here only for
	 * manipulate top max77818 mfd irq_chip to handle BIT_CHGR_INT.
	 */
	return IRQ_HANDLED;
}

static int max77818_charger_parse_dt(struct max77818_charger *chg)
{
	struct device_node *np = chg->dev->of_node;
	struct gpio_desc *gdp;

	if (!np) {
		dev_err(chg->dev, "no charger OF node\n");
		return -ENODEV;
	}

	if (of_property_read_u32(np, "fast_charge_timer",
				 &chg->fast_charge_timer))
		chg->fast_charge_timer = 4; // 4 hours

	if (of_property_read_u32(np, "fast_charge_current",
				 &chg->fast_charge_current))
		chg->fast_charge_current = 450; // 450mA

	if (of_property_read_u32(np, "charge_termination_voltage",
				 &chg->termination_voltage))
		chg->termination_voltage = 4200; // 4200mV

	if (of_property_read_u32(np, "topoff_timer", &chg->topoff_timer))
		chg->topoff_timer = 30; // 30 min

	if (of_property_read_u32(np, "topoff_current",
				 &chg->topoff_current))
		chg->topoff_current = 150; // 150mA

	if (of_property_read_u32(np, "restart_threshold",
				 &chg->restart_threshold))
		chg->restart_threshold = 150; // 150mV

	if (of_property_read_u32(np, "input_current_limit_chgin",
				 &chg->input_current_limit_chgin))
		chg->input_current_limit_chgin = 500; // 500mA

	if (of_property_read_u32(np, "input_current_limit_wcin",
				 &chg->input_current_limit_wcin))
		chg->input_current_limit_wcin = 500; // 500mA

	gdp = devm_gpiod_get(chg->dev, "chgin-stat", GPIOD_IN);
	if (IS_ERR(gdp)) {
		if (PTR_ERR(gdp) != -ENOENT)
			dev_warn(chg->dev,
				"chgin-stat GPIO not given in DT, "
				"chgin connection status not available\n");

		if (PTR_ERR(gdp) != -ENOSYS)
			dev_warn(chg->dev,
				 "chgin-stat GPIO given is not valid, "
				 "chgin connection status not available\n");
	}
	else {
		chg->chgin_stat_gpio = gdp;
		dev_dbg(chg->dev,
			"chgin connection status gpio registered (gpio %d)\n",
			desc_to_gpio(chg->chgin_stat_gpio));
	}

	gdp = devm_gpiod_get(chg->dev, "wcin-stat", GPIOD_IN);
	if (IS_ERR(gdp)) {
		if (PTR_ERR(gdp) != -ENOENT)
			dev_warn(chg->dev,
				"wcin-stat GPIO not given in DT, "
				"wcin connection status not available\n");

		if (PTR_ERR(gdp) != -ENOSYS)
			dev_warn(chg->dev,
				"wcin-stat GPIO given is not valid, "
				"wcin connection status not available\n");
	}
	else {
		chg->wcin_stat_gpio = gdp;
		dev_dbg(chg->dev,
			"wcin connection status gpio registered (gpio %d)\n",
			desc_to_gpio(chg->wcin_stat_gpio));
	}
	return 0;
}

static const struct power_supply_desc psy_chg_desc = {
	.name = "max77818-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77818_charger_props,
	.num_properties = ARRAY_SIZE(max77818_charger_props),
	.get_property = max77818_charger_get_property,
	.set_property = max77818_charger_set_property,
	.property_is_writeable = max77818_charger_property_is_writeable,
};

static int max77818_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77818_dev *max77818 = dev_get_drvdata(dev->parent);
	struct power_supply_config psy_cfg = {};
	struct max77818_charger *chg;
	int ret;
	bool init_ok;

	if (IS_ERR_OR_NULL(max77818->regmap_chg)) {
		dev_warn(dev,
			 "charge device regmap not initialized by MFD parent\n");
	}

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	chg->regmap = max77818->regmap_chg;

	mutex_init(&chg->lock);

	ret = max77818_charger_parse_dt(chg);
	if (ret < 0)
		dev_warn(dev, "failed to parse charger dt: %d\n", ret);

	psy_cfg.drv_data = chg;
	psy_cfg.of_node = dev->of_node;

	init_ok = !max77818_charger_initialize(chg);
	if (!init_ok) {
		dev_warn(dev, "failed to init charger: %d\n", init_ok);
	}

	chg->psy_chg = devm_power_supply_register(dev, &psy_chg_desc, &psy_cfg);
	if (IS_ERR(chg->psy_chg)) {
		ret = PTR_ERR(chg->psy_chg);
		dev_err(dev, "failed to register supply: %d\n", ret);
		return ret;
	}

	if (!init_ok) {
		dev_warn(dev, "skipping IRQ initialization, "
			      "due to failed charger init\n");
		return 0;
	}


	INIT_WORK(&chg->irq_work, max77818_charger_irq_work);

	chg->irq = regmap_irq_get_virq(max77818->irqc_intsrc,
				       MAX77818_CHGR_INT);
	if (chg->irq <= 0) {
		dev_err(dev, "failed to get virq: %d\n", chg->irq);
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(dev, chg->irq, NULL,
					max77818_charger_handler, 0,
					"charger", chg);
	if (ret) {
		dev_err(dev, "failed to request charger irq: %d\n", ret);
		return ret;
	}

	chg->chgin_irq = regmap_irq_get_virq(max77818->irqc_chg,
					     CHG_IRQ_CHGIN_I);
	if (chg->chgin_irq <= 0) {
		dev_err(dev, "failed to get chgin virq: %d\n", chg->chgin_irq);
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(dev, chg->chgin_irq, NULL,
					max77818_charger_chgin_isr,
					0, "charger-chgin", chg);
	if (ret) {
		dev_err(dev, "failed to reqeust chgin irq: %d\n", ret);
		return ret;
	}

	chg->wcin_irq = regmap_irq_get_virq(max77818->irqc_chg,
					    CHG_IRQ_WCIN_I);
	if (chg->wcin_irq <= 0) {
		dev_err(dev, "failed to get wcin virq: %d\n", chg->wcin_irq);
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(dev, chg->wcin_irq, NULL,
					max77818_charger_wcin_isr,
					0, "charger-wcin", chg);
	if (ret) {
		dev_err(dev, "failed to reqeust wcin irq: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77818_charger_suspend(struct device *dev)
{
	if (pm_suspend_target_state == PM_SUSPEND_MEM) {
		dev_dbg(dev->parent, "Selecting sleep pinctrl state\n");
		pinctrl_pm_select_sleep_state(dev->parent);
	}

	return 0;
}

static int max77818_charger_resume(struct device *dev)
{
	if (pm_suspend_target_state == PM_SUSPEND_MEM) {
		dev_dbg(dev->parent, "Selecting default pinctrl state\n");
		pinctrl_pm_select_default_state(dev->parent);
	}

	return 0;
}

static struct dev_pm_ops max77818_pm_ops = {
	.suspend = max77818_charger_suspend,
	.resume = max77818_charger_resume,
};

static struct platform_driver max77818_charger_driver = {
	.driver = {
		.name = MAX77818_CHARGER_NAME,
		.owner = THIS_MODULE,
		.pm = &max77818_pm_ops,
	},
	.probe = max77818_charger_probe,
};
module_platform_driver(max77818_charger_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77818 Charger Driver");
MODULE_AUTHOR("TaiEup Kim <clark.kim@maximintegrated.com>");
