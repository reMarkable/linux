/*
 * Fuel gauge driver for Maxim 77818
 *  Note that Maxim 77818 is mfd and this is its subdevice.
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Steinar Bakkemo <steinar.bakkemo@remarkable.com>
 * Author: Shawn Guo  <shawn.guo@linaro.org>
 * Author: Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * It is based on max17042_battery driver.
 */

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/power/max77818_battery_utils.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/platform_device.h>
#include <linux/usb/phy.h>

/* Status register bits */
#define STATUS_POR_BIT         (1 << 1)
#define STATUS_BST_BIT         (1 << 3)
#define STATUS_VMN_BIT         (1 << 8)
#define STATUS_TMN_BIT         (1 << 9)
#define STATUS_SMN_BIT         (1 << 10)
#define STATUS_BI_BIT          (1 << 11)
#define STATUS_VMX_BIT         (1 << 12)
#define STATUS_TMX_BIT         (1 << 13)
#define STATUS_SMX_BIT         (1 << 14)
#define STATUS_BR_BIT          (1 << 15)

/* Interrupt mask bits */
#define CONFIG_ALRT_BIT_ENBL	(1 << 2)
#define STATUS_INTR_SOCMIN_BIT	(1 << 10)
#define STATUS_INTR_SOCMAX_BIT	(1 << 14)

/* Config2 register bits */
#define CONFIG2_LDMDL		(1 << 5)

#define VFSOC0_LOCK		0x0000
#define VFSOC0_UNLOCK		0x0080
#define MODEL_UNLOCK1		0X0059
#define MODEL_UNLOCK2		0X00C4
#define MODEL_LOCK1		0X0000
#define MODEL_LOCK2		0X0000

#define dQ_ACC_DIV		0x4
#define dP_ACC_100		0x1900
#define dP_ACC_200		0x3200

#define MAX77818_VMAX_TOLERANCE	50 /* 50 mV */

#define SYNC_SET_FLAG(flag, lock) ( \
	{ \
		mutex_lock(lock); \
		flag = true; \
		mutex_unlock(lock); \
	} \
)

#define SYNC_CLEAR_FLAG(flag, lock) ( \
	{ \
		mutex_lock(lock); \
		flag = false; \
		mutex_unlock(lock); \
	} \
)

#define SYNC_GET_FLAG(flag, lock) ( \
	{ \
		bool state; \
		mutex_lock(lock); \
		state = flag; \
		mutex_unlock(lock); \
		state; \
	} \
)

/* Parameter to be given from u-boot after doing update
 * in order to verify that all custom FG parameters
 * are configured according to DT */
static char config_update_param[255] = "verify";
module_param_string(config_update,
		    config_update_param,
		    sizeof(config_update_param),
		    0644);
MODULE_PARM_DESC(config_update,
		 "Optional parameter indicating the following config update mode:"
		 "- complete FG update (config_update=\"complete\")"
		 "- partial update (config_update=\"partial\")"
		 "- verification against current DT (config_update=\"verify\")"
		 "\n"
		 "By default, the verify mode is set."
		 "\n"
		 "Scripts may set the config_update param in order to re-config "
		 "the device if required, depending on versioning scheme or other "
		 "means of determining if a complete or partial update is required.");

struct max77818_chip {
	struct device *dev;
	struct max77818_dev *max77818_dev;

	int fg_irq;
	int chg_irq;
	int chg_chgin_irq;
	int chg_wcin_irq;
	struct regmap *regmap;
	struct power_supply *battery;
	struct max17042_platform_data *pdata;
	struct work_struct init_work;
	struct work_struct chg_isr_work;
	bool init_complete;
	struct power_supply *charger;
	struct usb_phy *usb_phy[2];
	struct notifier_block charger_detection_nb[2];
	struct work_struct charger_detection_work[2];
	int status_ex;
	int usb_safe_max_current;
	struct work_struct initial_charger_sync_work;
	struct completion init_completion;
	struct mutex lock;
};

static enum power_supply_property max77818_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_STATUS_EX,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CHARGER_MODE,
};

static const char * const max77818_status_ex_text[] = {
	"Charger not connected", "POGO connected", "USB-C connected",
	"POGO/USB-C connected", "Changing", "Unknown"
};

struct max77818_of_property {
	const char *property_name;
	u8 register_addr;
	int (*reg_write_op)(struct regmap *map,
			    unsigned int reg,
			    unsigned int value);
	bool skip_verify;
	bool require_lock;
};

static void max77818_do_init_completion(struct max77818_chip *chip)
{
	/* Clear flag used by power supply callbacks to check
	 * if it is safe to do property read/write */
	SYNC_SET_FLAG(chip->init_complete, &chip->lock);

	/* Signal to worker(s) waiting for initiation to be complete before
	 * doing final steps */
	complete(&chip->init_completion);
}

static bool max77818_do_complete_update(struct max77818_chip *chip)
{
	if (strncmp(config_update_param, "complete", 8) == 0) {
		dev_dbg(chip->dev, "config_update='complete'\n");
		return true;
	}
	else {
		return false;
	}
}

static bool max77818_do_partial_update(struct max77818_chip *chip)
{
	if (strncmp(config_update_param, "partial", 7) == 0) {
		dev_dbg(chip->dev, "config_update='partial'\n");
		return true;
	}
	else {
		return false;
	}
}

static bool max77818_do_param_verification(struct max77818_chip *chip)
{
	if (strncmp(config_update_param, "verify", 6) == 0) {
		dev_dbg(chip->dev, "config_update='verify'\n");
		return true;
	}
	else {
		return false;
	}
}

static int max77818_get_temperature(struct max77818_chip *chip, int *temp)
{
	struct regmap *map = chip->regmap;
	u32 data;
	int ret;

	ret = regmap_read(map, MAX17042_TEMP, &data);
	if (ret < 0)
		return ret;

	*temp = sign_extend32(data, 15);
	/* The value is converted into deci-centigrade scale */
	/* Units of LSB = 1 / 256 degree Celsius */
	*temp = *temp * 10 / 256;
	return 0;
}

static int max77818_get_status(struct max77818_chip *chip, int *status)
{
	int ret, charge_full, charge_now;

	ret = power_supply_am_i_supplied(chip->battery);
	if (ret < 0) {
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}
	if (ret == 0) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	/*
	 * The MAX170xx has builtin end-of-charge detection and will update
	 * FullCAP to match RepCap when it detects end of charging.
	 *
	 * When this cycle the battery gets charged to a higher (calculated)
	 * capacity then the previous cycle then FullCAP will get updated
	 * contineously once end-of-charge detection kicks in, so allow the
	 * 2 to differ a bit.
	 */

	ret = regmap_read(chip->regmap, MAX17042_FullCAP, &charge_full);
	if (ret < 0)
		return ret;

	ret = regmap_read(chip->regmap, MAX17042_RepCap, &charge_now);
	if (ret < 0)
		return ret;

	if ((charge_full - charge_now) <= MAX17042_FULL_THRESHOLD)
		*status = POWER_SUPPLY_STATUS_FULL;
	else
		*status = POWER_SUPPLY_STATUS_CHARGING;

	return 0;
}

static void max77818_update_status_ex(struct max77818_chip *chip)
{
	union power_supply_propval val;
	int ret = 0;

	ret = MAX77818_DO_NON_FGCC_OP(
			chip->max77818_dev,
			power_supply_get_property(chip->charger,
						  POWER_SUPPLY_PROP_STATUS_EX,
						  &val),
			"Requesting status_ex from charger");

	if (ret) {
		dev_err(chip->dev,
			"Failed to read status_ex from charger, setting "
			"status_ex = UNKNOWN\n");
		chip->status_ex = POWER_SUPPLY_STATUS_EX_UNKNOWN;
	}
	else {
		if (val.intval < (sizeof(max77818_status_ex_text)/
				  sizeof(max77818_status_ex_text[0])))
			dev_dbg(chip->dev,
				"Setting status_ex = %s\n",
				max77818_status_ex_text[val.intval]);
		else
			dev_dbg(chip->dev,
				"Unknown status_ex (%d) returned\n",
				val.intval);

		chip->status_ex = val.intval;
	}

	dev_dbg(chip->dev, "Sending status_ex change notification\n");
	sysfs_notify(&chip->battery->dev.kobj, NULL, "status_ex");
}

static int max77818_get_battery_health(struct max77818_chip *chip, int *health)
{
	int temp, vavg, vbatt, ret;
	u32 val;

	ret = regmap_read(chip->regmap, MAX17042_AvgVCELL, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vavg = val * 625 / 8;
	/* Convert to millivolts */
	vavg /= 1000;

	ret = regmap_read(chip->regmap, MAX17042_VCELL, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vbatt = val * 625 / 8;
	/* Convert to millivolts */
	vbatt /= 1000;

	if (vavg < chip->pdata->vmin) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
		goto out;
	}

	if (vbatt > chip->pdata->vmax + MAX77818_VMAX_TOLERANCE) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto out;
	}

	ret = max77818_get_temperature(chip, &temp);
	if (ret < 0)
		goto health_error;

	if (temp < chip->pdata->temp_min) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		goto out;
	}

	if (temp > chip->pdata->temp_max) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		goto out;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;

out:
	return 0;

health_error:
	return ret;
}

static int max77818_set_charger_mode(struct max77818_chip *chip,
				     const union power_supply_propval *val)
{
	int ret;

	if (!chip->charger)
		return -ENODEV;

	ret = MAX77818_DO_NON_FGCC_OP(
			chip->max77818_dev,
			power_supply_set_property(chip->charger,
						  POWER_SUPPLY_PROP_CHARGER_MODE,
						  val),
			"Setting charger mode through charger driver");
	if (ret)
		dev_err(chip->dev,
			"Failed to forward charger mode to charger driver\n");

	return ret;
}

static int max77818_get_charger_mode(struct max77818_chip *chip,
				     int *charger_mode)
{
	union power_supply_propval val;
	int ret;

	if (!chip->charger)
		return -ENODEV;


	ret = MAX77818_DO_NON_FGCC_OP(
			chip->max77818_dev,
			power_supply_get_property(chip->charger,
						  POWER_SUPPLY_PROP_CHARGER_MODE,
						  &val),
			"Reading charger mode from charger driver");
	if (ret)
		dev_err(chip->dev,
			"Failed to read charger mode from charger driver\n");

	else
		*charger_mode = val.intval;

	return ret;
}

static int max77818_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct max77818_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	int temp;
	int ret;
	u32 data;
	u64 data64;

	if (!SYNC_GET_FLAG(chip->init_complete, &chip->lock))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = max77818_get_status(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_STATUS_EX:
		/* status_ex is just a shadow value of the status_ex prop
		 * reported by the charger driver, updated upon receiving
		 * a connection change interrupt from the charger */
		val->intval = chip->status_ex;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		/*
		 * MAX17042_STATUS_BattAbsent bit is not working for some
		 * reason unknown yet. We are working around the issue here by
		 * reading temperature register, in which a negative value
		 * indicates absence of battery.
		 */
		ret = max77818_get_temperature(chip, &temp);
		if (ret < 0)
			return ret;

		if (temp < 0)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = regmap_read(map, MAX17042_Cycles, &data);
		if (ret < 0)
			return ret;

		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = regmap_read(map, MAX17042_MinMaxVolt, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		val->intval *= 20000; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		ret = regmap_read(map, MAX17042_MinMaxVolt, &data);
		if (ret < 0)
			return ret;

		val->intval = (data & 0xff) * 20000; /* Units of 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = regmap_read(map, MAX17047_V_empty, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 7;
		val->intval *= 10000; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(map, MAX17042_VCELL, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(map, MAX17042_AvgVCELL, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = regmap_read(map, MAX17042_OCVInternal, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 625 / 8;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = regmap_read(map, MAX17042_RepSOC, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = regmap_read(map, MAX17042_SALRT_Th, &data);
		if (ret < 0)
			return ret;

		val->intval = data & 0xff;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = regmap_read(map, MAX17042_SALRT_Th, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = regmap_read(map, MAX17042_DesignCap, &data);
		if (ret < 0)
			return ret;

		data64 = data * 5000000ll;
		do_div(data64, chip->pdata->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(map, MAX17042_FullCAP, &data);
		if (ret < 0)
			return ret;

		data64 = data * 5000000ll;
		do_div(data64, chip->pdata->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = regmap_read(map, MAX17042_RepCap, &data);
		if (ret < 0)
			return ret;

		data64 = data * 5000000ll;
		do_div(data64, chip->pdata->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = regmap_read(map, MAX17042_QH, &data);
		if (ret < 0)
			return ret;

		val->intval = data * 1000 / 2;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = max77818_get_temperature(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;
		/* LSB is Alert Minimum. In deci-centigrade */
		val->intval = sign_extend32(data & 0xff, 7) * 10;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;
		/* MSB is Alert Maximum. In deci-centigrade */
		val->intval = sign_extend32(data >> 8, 7) * 10;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max77818_get_battery_health(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = regmap_read(map, MAX17042_Current, &data);
		if (ret < 0)
			return ret;

		val->intval = sign_extend32(data, 15);
		val->intval *= 1562500 / chip->pdata->r_sns;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = regmap_read(map, MAX17042_AvgCurrent, &data);
		if (ret < 0)
			return ret;

		val->intval = sign_extend32(data, 15);
		val->intval *= 1562500 / chip->pdata->r_sns;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = regmap_read(map, MAX17042_TTE, &data);
		val->intval = data * 5625 / 1000;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = regmap_read(map, MAX77818_TTF, &data);
		val->intval = data * 5625 / 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		ret = max77818_get_charger_mode(chip, &val->intval);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max77818_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct max77818_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	int ret = 0;
	u32 data;
	int8_t temp;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;

		/* Input in deci-centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force min < max */
		if (temp >= (int8_t)(data >> 8))
			temp = (int8_t)(data >> 8) - 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff00) + temp;
		ret = regmap_write(map, MAX17042_TALRT_Th, data);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;

		/* Input in Deci-Centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force max > min */
		if (temp <= (int8_t)(data & 0xff))
			temp = (int8_t)(data & 0xff) + 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff) + (temp << 8);
		ret = regmap_write(map, MAX17042_TALRT_Th, data);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		if (val->intval < 0 || val->intval > 100) {
			ret = -EINVAL;
			break;
		}
		ret = regmap_update_bits(map, MAX17042_SALRT_Th,
					 0xff, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		if ((val->intval < 0 || val->intval > 100) &&
		    val->intval != 255) {
			ret = -EINVAL;
			break;
		}
		ret = regmap_update_bits(map, MAX17042_SALRT_Th,
					 0xff00, val->intval << 8);
		break;
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		if ((val->intval < 0) || (val->intval > 2)) {
			ret = -EINVAL;
			break;
		}

		ret = max77818_set_charger_mode(chip, val);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int max77818_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static void max77818_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static int max77818_write_and_verify_reg(struct regmap *map, unsigned int reg, unsigned int value)
{
	int retries = 8;
	u32 read_value;
	int ret;

	do {
		retries--;
		ret = regmap_write(map, reg, value);
		if (ret) {
			continue;
		}

		ret = regmap_read(map, reg, &read_value);
		if (ret) {
			continue;
		}

		if (read_value != value) {
			ret = -EIO;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}

static inline int max77818_unlock_model(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret;

	ret = regmap_write(map, MAX17042_MLOCKReg1, MODEL_UNLOCK1);
	if (ret)
		return ret;

	return regmap_write(map, MAX17042_MLOCKReg2, MODEL_UNLOCK2);
}

static inline int max77818_lock_model(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret;

	ret = regmap_write(map, MAX17042_MLOCKReg1, MODEL_LOCK1);
	if (ret)
		return ret;

	return regmap_write(map, MAX17042_MLOCKReg2, MODEL_LOCK2);
}

static inline int max77818_write_model_data(struct max77818_chip *chip,
					     u16 *data, int count)
{
	struct regmap *map = chip->regmap;
	int i, ret;

	for (i = 0; i < count; i++) {
		ret = regmap_write(map, MAX17042_MODELChrTbl + i, data[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static inline int max77818_read_model_data(struct max77818_chip *chip,
					    u16 *data, int count)
{
	struct regmap *map = chip->regmap;
	u32 tmp;
	int ret, i;

	for (i = 0; i < count; i++) {
		ret = regmap_read(map, MAX17042_MODELChrTbl + i, &tmp);
		if (ret)
			return ret;
		data[i] = (u16)tmp;
	}
	return 0;
}

static inline int max77818_model_data_compare(struct max77818_chip *chip,
					u16 *expected, u16 *compareto, int size)
{
	int i;

	if (memcmp(expected, compareto, size)) {
		dev_err(chip->dev, "%s compare failed\n", __func__);
		for (i = 0; i < size; i++)
			dev_info(chip->dev,
					"%03d: Expected: 0x%x, read: 0x%x\n",
					i,
					expected[i],
					compareto[i]);
		return -EINVAL;
	}
	return 0;
}

static int max77818_init_model(struct max77818_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	u16 *data, *rdata;
	int count;
	int ret;

	count = of_property_count_u16_elems(np, "maxim,cell-model-data");
	if (count < 0 || count != MAX17042_CHARACTERIZATION_DATA_SIZE) {
		dev_err(chip->dev, "invalid or missing maxim,cell-model-data: %d\n",
			count);
		return -EINVAL;
	}

	data = kcalloc(count, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(chip->dev, "failed to kcalloc for data\n");
		return -ENOMEM;
	}

	/* Read cell model data */
	of_property_read_u16_array(np, "maxim,cell-model-data", data, count);

	rdata = kcalloc(count, sizeof(*rdata), GFP_KERNEL);
	if (!rdata) {
		dev_err(chip->dev, "failed to kcalloc for rdata\n");
		ret = -ENOMEM;
		goto free_data;
	}

	ret = max77818_unlock_model(chip);
	if (ret) {
		dev_err(chip->dev, "unable to unlock model data: %d\n", ret);
		goto free_rdata;
	}

	ret = max77818_write_model_data(chip, data, count);
	if (ret) {
		dev_err(chip->dev, "unable to write model data: %d\n", ret);
		goto lock_model;
	}

	ret = max77818_read_model_data(chip, rdata, count);
	if (ret) {
		dev_err(chip->dev, "unable to read model data: %d\n", ret);
		goto lock_model;
	}

	ret = max77818_model_data_compare(chip, data, rdata, count);
	if (ret)
		dev_err(chip->dev, "model data compare failed: %d\n", ret);

lock_model:
	max77818_lock_model(chip);
free_rdata:
	kfree(rdata);
free_data:
	kfree(data);
	return ret;
}

static int max77818_verify_model_lock(struct max77818_chip *chip)
{
	int count = MAX17042_CHARACTERIZATION_DATA_SIZE;
	u16 *data;
	int ret = 0;
	int i;

	data = kcalloc(count, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = max77818_read_model_data(chip, data, count);
	if (ret)
		goto done;

	for (i = 0; i < count; i++) {
		if (data[i]) {
			ret = -EINVAL;
			goto done;
		}
	}

done:
	kfree(data);
	return ret;
}

static int max77818_model_loading(struct max77818_chip *chip)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(400);
	struct regmap *map = chip->regmap;
	u32 config2;
	int ret;

	/* Setting LdMdl bit */
	ret = regmap_read(map, MAX77818_Config2, &config2);
	if (ret)
		return ret;

	ret = regmap_write(map, MAX77818_Config2, config2 | CONFIG2_LDMDL);
	if (ret)
		return ret;

	/* Poll LdMdl bit to be 0 under 400ms */
	do {
		ret = regmap_read(map, MAX77818_Config2, &config2);
		if (ret)
			break;
		if ((config2 & CONFIG2_LDMDL) == 0)
			break;
		if (time_after(jiffies, timeout))
			break;
	} while (1);

	ret = regmap_read(map, MAX77818_Config2, &config2);
	if (ret)
		return ret;

	return (config2 & CONFIG2_LDMDL) ? -ETIMEDOUT : 0;
}

static inline int max77818_read_of_property(struct max77818_chip *chip,
				      const char *param,
				      u16 *value)
{
	struct device_node *np = chip->dev->of_node;
	int ret;

	u16 read_value = 0;

	ret = of_property_read_u16(np, param, &read_value);
	if (!ret)
		*value = read_value;

	return ret;
}

static int max77818_write_of_param_if_mismatch(struct max77818_chip *chip,
					   struct max77818_of_property *prop)
{
	u16 read_param = 0;
	int read_cur_value = 0;
	bool param_mismatch = false;
	int ret;

	ret = max77818_read_of_property(chip, prop->property_name, &read_param);
	if(!ret) {
		dev_dbg(chip->dev, "Verifying '%s' (reg 0x%02x) = 0x%04x\n",
			prop->property_name,
			prop->register_addr,
			read_param);

		if (prop->require_lock) {
			dev_dbg(chip->dev, "Applying lock\n");
			mutex_lock(&chip->max77818_dev->lock);
		}

		ret = regmap_read(chip->regmap,
				  prop->register_addr,
				  &read_cur_value);
		if(ret) {
			dev_warn(chip->dev,
				 "Failed to read '%s' from reg 0x%02x)\n",
				 prop->property_name,
				 prop->register_addr);
			param_mismatch = true;
		}

		if ((read_param != read_cur_value) || param_mismatch) {
			dev_dbg(chip->dev,
				"Read '%s' (reg 0x%02x) = 0x%04x from device, "
				"expected 0x%04x\n",
				prop->property_name,
				prop->register_addr,
				read_cur_value,
				read_param);
			param_mismatch = true;
		}

		if (param_mismatch) {
			dev_dbg(chip->dev, "Writing '%s' (reg 0x%02x): 0x%04x\n",
				prop->property_name,
				prop->register_addr,
				read_param);
			ret = prop->reg_write_op(chip->regmap,
						 prop->register_addr,
						 read_param);
			if (ret) {
				dev_err(chip->dev,
						"Failed to write '%s' (reg 0x%02x): 0x%04x\n",
						prop->property_name,
						prop->register_addr,
						read_param);
			}
		}

		if (prop->require_lock) {
			dev_dbg(chip->dev, "Releasing lock\n");
			mutex_unlock(&chip->max77818_dev->lock);
		}
	}
	else if (ret == -EINVAL)
		dev_warn(chip->dev,
			 "'%s' property not given in DT, using default value\n",
			 prop->property_name);
	else
		dev_warn(chip->dev,
			 "Failed to read '%s' param from DT, check value\n",
			 prop->property_name);

	return ret;
}

static int  max77818_read_of_param_and_write(struct max77818_chip *chip,
					  struct max77818_of_property *prop)
{
	u16 read_param;
	int ret;

	ret = max77818_read_of_property(chip, prop->property_name, &read_param);
	if (!ret) {
		dev_dbg(chip->dev, "Writing '%s' (reg 0x%02x): 0x%04x\n",
			prop->property_name,
			prop->register_addr,
			read_param);

		if (prop->require_lock) {
			dev_dbg(chip->dev, "Applying lock\n");
			mutex_lock(&chip->max77818_dev->lock);
		}

		ret = prop->reg_write_op(chip->regmap,
					 prop->register_addr,
					 read_param);
		if (ret) {
			dev_warn(chip->dev,
				 "Failed to write '%s' property read from DT\n",
				 prop->property_name);
		}

		if (prop->require_lock) {
			dev_dbg(chip->dev, "Releasing lock\n");
			mutex_unlock(&chip->max77818_dev->lock);
		}
	}
	else if (ret == -EINVAL)
		dev_warn(chip->dev,
			 "'%s' property not given in DT, using default value\n",
			 prop->property_name);
	else
		dev_warn(chip->dev,
			 "Failed to read '%s' param from DT, check value\n",
			 prop->property_name);

	return ret;
}

static int max77818_unlock_extra_config_registers(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	u32 value;
	int ret;

	ret = regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);
	if (ret) {
		dev_warn(chip->dev,
			 "Failed to write VFSOC0Enable to unlock: %d\n", ret);
		return ret;
	}

	ret = regmap_read(map, MAX17042_VFSOC, &value);
	if (ret) {
		dev_warn(chip->dev, "Failed to read VFSOC: %d\n", ret);
		return ret;
	}

	return max77818_write_and_verify_reg(map, MAX17042_VFSOC0, value);
}

static int max77818_lock_extra_config_registers(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret;

	ret = regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_LOCK);
	if (ret)
		dev_warn(chip->dev,
			 "Failed to write VFSOC0Enable to lock: %d\n", ret);

	return ret;
}

static struct max77818_of_property max77818_relax_cfg =
	{"maxim,relax-cfg", MAX17042_RelaxCFG, regmap_write, true, false };

static struct max77818_of_property max77818_custom_param_list [] = {
	{ "maxim,learn-cfg", MAX17042_LearnCFG, regmap_write, true, false },

	/* Verified and restored if required after reboot to ensure FGCC=1 */
	/* Also marked to require lock */
	{ "maxim,config", MAX17042_CONFIG, regmap_write, false, true},

	{ "maxim,config2", MAX77818_Config2, regmap_write, true },
	{ "maxim,full-soc-threshold", MAX17047_FullSOCThr, regmap_write, true, false },

	/* learned value, skipped during verify/write operation at boot */
	{ "maxim,fullcaprep", MAX17042_FullCAP0, max77818_write_and_verify_reg, true, false },

	{ "maxim,design-cap", MAX17042_DesignCap, regmap_write, true, false },

	/* learned values, skipped during verify/write operation at boot */
	{ "maxim,dpacc", MAX17042_dPacc, max77818_write_and_verify_reg, true, false },
	{ "maxim,dqacc", MAX17042_dQacc, max77818_write_and_verify_reg, true, false },
	{ "maxim,fullcapnom", MAX17042_FullCAPNom, max77818_write_and_verify_reg, true, false },

	{ "maxim,misc-cfg", MAX17042_MiscCFG, regmap_write, true, false },
	{ "maxim,v-empty", MAX17047_V_empty, regmap_write, true, false },
	{ "maxim,qresidual00", MAX17047_QRTbl00, max77818_write_and_verify_reg, true, false },
	{ "maxim,qresidual10", MAX17047_QRTbl10, max77818_write_and_verify_reg, true, false },
	{ "maxim,qresidual20", MAX17047_QRTbl20, max77818_write_and_verify_reg, true, false },
	{ "maxim,qresidual30", MAX17047_QRTbl30, max77818_write_and_verify_reg, true, false },

	/* learned value, skipped during verify/write operation at boot */
	{ "maxim,rcomp0", MAX17042_RCOMP0, max77818_write_and_verify_reg, true, false },

	{ "maxim,tempco", MAX17042_TempCo, max77818_write_and_verify_reg, true, false },
	{ "maxim,ichg-term", MAX17042_ICHGTerm, regmap_write, true, false },
	{ "maxim,filter-cfg", MAX17042_FilterCFG, regmap_write, true, false },

	/* learned value, skipped during verify/write operation at boot */
	{ "maxim,iavg-empty", MAX17042_LAvg_empty, regmap_write, true, false },

	{ "maxim,tgain", MAX17042_TGAIN, regmap_write, false, false },
	{ "maxim,toff", MAx17042_TOFF, regmap_write, false, false },
	{ "maxim,tcurve", MAX77818_TCURVE, regmap_write, false, false },
	{ "maxim,talrt-th", MAX17042_TALRT_Th, regmap_write, false, false },
	{ "maxim,talrt-th2", MAX77818_TALRT_Th2, regmap_write, false, false },
	{ "maxim,jeita-curr", MAX77818_JEITA_Curr, regmap_write, false, false },
	{ "maxim,jeita-volt", MAX77818_JEITA_Volt, regmap_write, false, false },
	{ "maxim,chargestate0", MAX77818_ChargeState0, regmap_write, false, false },
	{ "maxim,chargestate1", MAX77818_ChargeState1, regmap_write, false, false },
	{ "maxim,chargestate2", MAX77818_ChargeState2, regmap_write, false, false },
	{ "maxim,chargestate3", MAX77818_ChargeState3, regmap_write, false, false },
	{ "maxim,chargestate4", MAX77818_ChargeState4, regmap_write, false, false },
	{ "maxim,chargestate5", MAX77818_ChargeState5, regmap_write, false, false },
	{ "maxim,chargestate6", MAX77818_ChargeState6, regmap_write, false, false },
	{ "maxim,chargestate7", MAX77818_ChargeState7, regmap_write, false, false },

	/* The order of the following ones should be respected */
	{ "maxim,at-rate", MAX17042_AtRate, regmap_write, true, false },
	{ "maxim,smart-chg-cfg", MAX77818_SmartChgCfg, regmap_write, false, false },
	{ "maxim,convgcfg", MAX77818_ConvgCfg, regmap_write, true, false },
};

static bool max77818_write_mismatched_custom_params(struct max77818_chip *chip)
{
	int i;
	int ret;
	bool is_success = true;

	dev_dbg(chip->dev, "Verifying custom params\n");

	if (max77818_relax_cfg.skip_verify) {
		dev_dbg(chip->dev,
			"Skipping verify/write for register '%s'\n",
			max77818_relax_cfg.property_name);
	}
	else {
		ret = max77818_write_of_param_if_mismatch(chip, &max77818_relax_cfg);
		if (ret)
			is_success = false;
	}

	ret = max77818_unlock_extra_config_registers(chip);
	if (ret)
		is_success = false;

	for(i = 0; i < ARRAY_SIZE(max77818_custom_param_list); i++) {
		if (max77818_custom_param_list[i].skip_verify) {
			dev_dbg(chip->dev,
				"Skipping verify/write for register '%s'\n",
				max77818_custom_param_list[i].property_name);
			continue;
		}

		ret = max77818_write_of_param_if_mismatch(chip,
					       &max77818_custom_param_list[i]);
		if (ret)
			is_success = false;
	}

	max77818_lock_extra_config_registers(chip);

	return is_success;
}

static bool max77818_write_all_custom_params(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret, i;
	bool is_success = true;

	dev_dbg(chip->dev, "Writing custom params\n");

	ret = regmap_write(map, MAX17042_RepCap, 0);
	if (ret) {
		dev_warn(chip->dev, "Failed to write RepCap: %d\n", ret);
		is_success = false;
	}

	ret = max77818_read_of_param_and_write(chip, &max77818_relax_cfg);
	if (ret)
		is_success = false;

	ret = max77818_unlock_extra_config_registers(chip);
	if (ret)
		is_success = false;

	for(i = 0; i < ARRAY_SIZE(max77818_custom_param_list); i++) {
		ret = max77818_read_of_param_and_write(chip,
					      &max77818_custom_param_list[i]);
		if (ret)
			is_success = false;
	}

	max77818_lock_extra_config_registers(chip);

	return is_success;
}

static int max77818_init_chip(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret;
	bool is_success;

	/* Write cell characterization data */
	ret = max77818_init_model(chip);
	if (ret) {
		dev_err(chip->dev, "init model failed: %d\n", ret);
		return ret;
	}

	ret = max77818_verify_model_lock(chip);
	if (ret) {
		dev_err(chip->dev, "lock verify failed: %d\n", ret);
		return ret;
	}

	/* Write custom parameters from device tree */
	is_success = max77818_write_all_custom_params(chip);

	/* Initiate model loading for MAX77818 */
	ret = max77818_model_loading(chip);
	if (ret) {
		dev_err(chip->dev, "initiate model loading failed: %d\n", ret);
		return ret;
	}

	/* Wait 500 ms for SOC to be calculated from the new parameters */
	msleep(500);

	/* Init complete, Clear the POR bit if successful */
	if (is_success)
		regmap_update_bits(map, MAX17042_STATUS, STATUS_POR_BIT, 0x0);

	return 0;
}

static void max77818_do_initial_charger_sync_worker(struct work_struct *work)
{
	struct max77818_chip *chip =
	    container_of(work, struct max77818_chip, initial_charger_sync_work);
	struct max77818_dev *max77818 = chip->max77818_dev;
	struct device *dev = max77818->dev;
	union power_supply_propval val;
	int ret;

	/* Wait for other initiation to be completed before running these
	 * final steps */
	dev_dbg(dev, "Waiting for other initiation to complete before doing "
		     "final charger driver sync. steps \n");
	ret = wait_for_completion_interruptible_timeout(&chip->init_completion,
							10 * HZ);
	if (ret == 0) {
		dev_err(dev, "Timeout while waiting for other init to complete, "
			     "trying to do final charger driver sync anyway\n");
	}
	else {
		dev_dbg(dev, "Other init completed, starting final charger "
			     "driver sync stepd\n");
	}

	val.intval = 0;
	ret = MAX77818_DO_NON_FGCC_OP(
			max77818,
			power_supply_set_property(chip->charger,
						  POWER_SUPPLY_PROP_CURRENT_MAX,
						  &val),
			"Setting chgin max current\n");
	if (ret) {
		dev_err(dev,
			"Failed to set max current in charger driver\n");
	}

	ret = MAX77818_DO_NON_FGCC_OP(
			max77818,
			power_supply_get_property(chip->charger,
						  POWER_SUPPLY_PROP_STATUS_EX,
						  &val),
			"Reading initial status_ex from charger\n");
	if (ret) {
		dev_err(chip->dev,
			"Failed to read status_ex from charger driver while"
			"doing initial charger driver sync\n");
	}

	/* Do an initial max current adjustment according to max current
	 * currently configured in USB PHY, in case this was updated before
	 * this driver was loaded.
	 *
	 * As the charger detection handling is normally done i a worker,
	 * the initial calls done here are queued on the same work queue as the
	 * async event workers, in order to prevent conflict with possible
	 * ongoing event handling if called directly from here */
	dev_dbg(chip->dev,
		"Scheduling initial max current adjustment for chgin interface ");
	schedule_work(&chip->charger_detection_work[0]);

	dev_dbg(chip->dev,
		"Scheduling initial max current adjustment for wcin interface ");
	schedule_work(&chip->charger_detection_work[1]);
}

static irqreturn_t max77818_fg_isr(int id, void *dev)
{
	struct max77818_chip *chip = dev;
	u32 val;

	regmap_read(chip->regmap, MAX17042_STATUS, &val);

	if (val & STATUS_INTR_SOCMIN_BIT) {
		dev_info(chip->dev, "MIN SOC alert\n");
		sysfs_notify(&chip->battery->dev.kobj, NULL,
			     "capacity_alert_min");
	}

	if (val & STATUS_INTR_SOCMAX_BIT) {
		dev_info(chip->dev, "MAX SOC alert\n");
		sysfs_notify(&chip->battery->dev.kobj, NULL,
			     "capacity_alert_max");
	}

	power_supply_changed(chip->battery);

	return IRQ_HANDLED;
}

static void max77818_charger_isr_work(struct work_struct *work)
{
	struct max77818_chip *chip =
		container_of(work, struct max77818_chip, chg_isr_work);

	dev_dbg(chip->dev, "Changing status_ex -> CHANGING\n");
	chip->status_ex = POWER_SUPPLY_STATUS_EX_CHANGING;

	dev_dbg(chip->dev, "Sending status_ex change notification\n");
	sysfs_notify(&chip->battery->dev.kobj, NULL, "status_ex");

	dev_dbg(chip->dev, "Reading updated connection state from charger\n");
	max77818_update_status_ex(chip);
}

static irqreturn_t max77818_charger_connection_change_isr(int irq, void *data)
{
	struct max77818_chip *chip = data;

	schedule_work(&chip->chg_isr_work);

	return IRQ_HANDLED;
}

static irqreturn_t max77818_charger_isr(int irq, void *data)
{
	/*
	 * This IRQ handler needs to do nothing, as it's here only for
	 * manipulate top max77818 mfd irq_chip to handle BIT_CHGR_INT.
	 */
	return IRQ_HANDLED;
}

static void max77818_init_worker(struct work_struct *work)
{
	struct max77818_chip *chip = container_of(work, struct max77818_chip,
						  init_work);
	int ret;

	dev_dbg(chip->dev, "Doing complete re-config\n");
	ret = max77818_init_chip(chip);
	if (ret) {
		dev_err(chip->dev, "failed to init chip: %d\n", ret);
	}

	max77818_do_init_completion(chip);
}

static struct max17042_platform_data *
max77818_get_pdata(struct max77818_chip *chip)
{
	struct max17042_platform_data *pdata;
	struct device *dev = chip->dev;
	struct device_node *np = dev->of_node;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	if (of_property_read_u32(np, "maxim,rsns-microohm", &pdata->r_sns))
		pdata->r_sns = MAX17042_DEFAULT_SNS_RESISTOR;

	if (of_property_read_s32(np, "maxim,cold-temp", &pdata->temp_min))
		pdata->temp_min = INT_MIN;
	if (of_property_read_s32(np, "maxim,over-heat-temp", &pdata->temp_max))
		pdata->temp_max = INT_MAX;

	if (of_property_read_s32(np, "maxim,dead-volt", &pdata->vmin))
		pdata->vmin = INT_MIN;
	if (of_property_read_s32(np, "maxim,over-volt", &pdata->vmax))
		pdata->vmax = INT_MAX;

	return pdata;
}

static void max77818_get_driver_config(struct max77818_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	int read_value;
	int ret;

	ret = of_property_read_u32(np, "usb_safe_max_current", &read_value);
	if (ret) {
		dev_err(chip->dev,
			"Failed to read usb_safe_max_current from DT\n");
		chip->usb_safe_max_current = 100;
	}
	else {
		dev_dbg(chip->dev,
			"Read usb_safe_max_current: %d\n",
			read_value);
		chip->usb_safe_max_current = read_value;
	}
}

static const struct regmap_config max77818_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static const struct power_supply_desc max77818_psy_desc = {
	.name = "max77818_battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = max77818_get_property,
	.set_property = max77818_set_property,
	.property_is_writeable = max77818_property_is_writeable,
	.external_power_changed = max77818_external_power_changed,
	.properties = max77818_battery_props,
	.num_properties = ARRAY_SIZE(max77818_battery_props),
};

static void max77818_charger_detection_worker_chgin(struct work_struct *work)
{
	struct max77818_chip *chip = container_of(work,
						  struct max77818_chip,
						  charger_detection_work[0]);
	/*
	unsigned int min_current, max_current;
	*/

	union power_supply_propval val;
	int ret;

	mutex_lock(&chip->lock);

	dev_dbg(chip->dev, "Doing charger detection work for chgin interface..\n");

	if (!chip->charger) {
		dev_err(chip->dev,
			"Cannot access charger device, unable to set max current for chgin interface\n");
		goto done;
	}

	/*
	dev_dbg(chip->dev, "Getting max/min current configured for given USB PHY\n");
	usb_phy_get_charger_current(chip->usb_phy[0], &min_current, &max_current);
	if (max_current == 0)
		val.intval = chip->usb_safe_max_current;
	else
		val.intval = max_current;
	*/

	val.intval = 0;

	ret = MAX77818_DO_NON_FGCC_OP(
			chip->max77818_dev,
			power_supply_set_property(chip->charger,
						  POWER_SUPPLY_PROP_CURRENT_MAX,
						  &val),
			"Setting max chgin current through charger driver");
	if (ret)
		dev_err(chip->dev,
			"Failed to set max chgin current in charger driver\n");

	done:
	mutex_unlock(&chip->lock);
}

static void max77818_charger_detection_worker_wcin(struct work_struct *work)
{
	struct max77818_chip *chip = container_of(work,
						  struct max77818_chip,
						  charger_detection_work[1]);
	unsigned int min_current, max_current;
	union power_supply_propval val;
	int ret;

	mutex_lock(&chip->lock);

	dev_dbg(chip->dev, "Doing charger detection work for wcin interface..\n");

	if (!chip->charger) {
		dev_err(chip->dev,
			"Cannot access charger device, unable to set max current for wcin interface\n");
		goto done;
	}

	dev_dbg(chip->dev, "Getting max/min current configured for given USB PHY (wcin)\n");
	usb_phy_get_charger_current(chip->usb_phy[1], &min_current, &max_current);
	if (max_current == 0)
		val.intval = 500;
	else
		val.intval = max_current;

	ret = MAX77818_DO_NON_FGCC_OP(
			chip->max77818_dev,
			power_supply_set_property(chip->charger,
						  POWER_SUPPLY_PROP_CURRENT_MAX2,
						  &val),
				"Setting max wcin current through charger driver");
	if (ret)
		dev_err(chip->dev,
			"Failed to set max wcin current in charger driver\n");

	done:
	mutex_unlock(&chip->lock);
}

static int max77818_charger_detection_notifier_call_chgin(struct notifier_block *nb,
							 unsigned long val, void *v)
{
	struct max77818_chip *chip = container_of(nb,
						  struct max77818_chip,
						  charger_detection_nb[0]);

	dev_dbg(chip->dev,
		"Handling charger detection notification from chgin interface "
		"(max current: %lu)\n", val);

	schedule_work(&chip->charger_detection_work[0]);

	return NOTIFY_OK;
}

static int max77818_charger_detection_notifier_call_wcin(struct notifier_block *nb,
							 unsigned long val, void *v)
{
	struct max77818_chip *chip = container_of(nb,
						  struct max77818_chip,
						  charger_detection_nb[1]);

	dev_dbg(chip->dev,
		"Handling charger detection notification from wcin interface "
		"(max current: %lu)\n", val);

	schedule_work(&chip->charger_detection_work[1]);

	return NOTIFY_OK;
}

static int max77818_init_otg_supply(struct max77818_chip *chip)
{
	struct device *dev = chip->dev;

	dev_dbg(dev, "Trying to reference charger (expecting charger as"
		     "first supply in given list of supplies)\n");
	if (chip->battery->num_supplies > 0) {
		chip->charger = power_supply_get_by_name(
					chip->battery->supplied_from[0]);
		if (!chip->charger) {
			dev_warn(dev, "Failed to reference charger, "
				      "OTG/charger IRQ handling will not"
				      "be available - verify DT config\n");
		}
	}

	return 0;
}

static int max77818_init_charger_detection(struct max77818_chip *chip)
{
	struct device *dev = chip->dev;
	int ret;

	dev_dbg(dev,
		"Trying to reference usb-phy1 (for receiving charger "
		"detection notifications for chgin interface\n");
	chip->usb_phy[0]= devm_usb_get_phy_by_phandle(dev, "usb-phy1", 0);
	if (IS_ERR(chip->usb_phy[0])) {
		ret = PTR_ERR(chip->usb_phy[0]);
		dev_err(dev, "usb_get_phy failed: %d\n", ret);
		return ret;
	}

	dev_dbg(dev,
		"Trying to reference usb-phy2 (for receiving charger "
		"detection notifications for wcin interface\n");
	chip->usb_phy[1]= devm_usb_get_phy_by_phandle(dev, "usb-phy2", 0);
	if (IS_ERR(chip->usb_phy[1])) {
		ret = PTR_ERR(chip->usb_phy[1]);
		dev_err(dev, "usb_get_phy failed: %d\n", ret);
		return ret;
	}

	dev_dbg(dev,
		"Trying to register notification handler (worker) for "
		"chgin interface charger detection notifications \n");
	INIT_WORK(&chip->charger_detection_work[0],
		  max77818_charger_detection_worker_chgin);
	chip->charger_detection_nb[0].notifier_call =
			max77818_charger_detection_notifier_call_chgin;
	ret = usb_register_notifier(chip->usb_phy[0],
				    &chip->charger_detection_nb[0]);
	if (ret) {
		dev_err(dev, "usb_register_notifier failed: %d\n", ret);
		return ret;
	}

	dev_dbg(dev,
		"Trying to register notification handler (worker) for "
		"wcin interface charger detection notifications \n");
	INIT_WORK(&chip->charger_detection_work[1],
		  max77818_charger_detection_worker_wcin);
	chip->charger_detection_nb[1].notifier_call =
			max77818_charger_detection_notifier_call_wcin;
	ret = usb_register_notifier(chip->usb_phy[1],
				    &chip->charger_detection_nb[1]);
	if (ret) {
		dev_err(dev, "usb_register_notifier failed: %d\n", ret);
		goto unreg_usb_phy0_notifier;
	}

	return 0;

unreg_usb_phy0_notifier:
	usb_unregister_notifier(chip->usb_phy[0],
				&chip->charger_detection_nb[0]);

	return ret;
}

static int max77818_init_fg_interrupt_handling(struct max77818_dev *max77818,
						struct max77818_chip *chip)
{
	struct device *dev = chip->dev;
	int ret;

	/* Disable max SOC alert and set min SOC alert as 10% by default */
	regmap_write(chip->regmap, MAX17042_SALRT_Th, (0xff << 8) | 0x0a );

	/* Register irq handler for the FG interrupt */
	chip->fg_irq = regmap_irq_get_virq(max77818->irqc_intsrc,
					   MAX77818_FG_INT);
	if (chip->fg_irq <= 0) {
		dev_err(dev, "failed to get virq: %d\n", chip->fg_irq);
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(dev, chip->fg_irq, NULL,
					max77818_fg_isr, 0,
					chip->battery->desc->name,
					chip);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		return ret;
	}

	return 0;
}

static int max77818_init_chg_interrupt_handling(struct max77818_dev *max77818,
						struct max77818_chip *chip)
{
	struct device *dev = chip->dev;
	bool fgcc_restore_state;
	bool skip_final_fgcc_op_finish = false;
	int ret;

	/* Init worker to be used by the charger interrupt handler */
	INIT_WORK(&chip->chg_isr_work, max77818_charger_isr_work);

	/* Disable FGCC during charger device irq handler registration */
	ret = MAX77818_START_NON_FGCC_OP(
			max77818,
			fgcc_restore_state,
			"Starting registration of irq handlers for the charger "
			"interrupts\n");
	if (ret) {
		dev_err(dev,
			"Failed to disable FGCC before charger device irq "
			"handler registration\n");
		return ret;
	}

	/* Register irq handler for the charger interrupt */
	chip->chg_irq = regmap_irq_get_virq(max77818->irqc_intsrc,
					    MAX77818_CHGR_INT);
	if (chip->chg_irq <= 0) {
		dev_err(dev, "failed to get virq: %d\n", chip->chg_irq);
		ret = -ENODEV;
		goto finish_fgcc_op;
	}

	ret = devm_request_threaded_irq(dev, chip->chg_irq, NULL,
					max77818_charger_isr, 0,
					"charger", chip);
	if (ret) {
		dev_err(dev, "failed to request charger irq: %d\n", ret);
		goto finish_fgcc_op;
	}

	/* Register irq handler for the CHGIN interrupt */
	chip->chg_chgin_irq = regmap_irq_get_virq(max77818->irqc_chg,
						  CHG_IRQ_CHGIN_I);
	if (chip->chg_chgin_irq <= 0) {
		dev_err(dev, "failed to get chgin virq: %d\n",
			chip->chg_chgin_irq);
		ret = -ENODEV;
		goto free_chg_irq;
	}
	dev_dbg(dev, "chgin irq: %d\n", chip->chg_chgin_irq);

	ret = devm_request_threaded_irq(dev, chip->chg_chgin_irq, NULL,
					max77818_charger_connection_change_isr,
					0, "charger-chgin", chip);
	if (ret) {
		dev_err(dev, "failed to reqeust chgin irq: %d\n", ret);
		goto free_chg_irq;
	}

	/* Register irq handler for the WCIN interrupt */
	chip->chg_wcin_irq = regmap_irq_get_virq(max77818->irqc_chg,
					    CHG_IRQ_WCIN_I);
	if (chip->chg_wcin_irq <= 0) {
		dev_err(dev, "failed to get wcin virq: %d\n", chip->chg_wcin_irq);
		ret = -ENODEV;
		goto free_chgin_irq;
	}
	dev_dbg(dev, "wcin irq: %d\n", chip->chg_wcin_irq);

	ret = devm_request_threaded_irq(dev, chip->chg_wcin_irq, NULL,
					max77818_charger_connection_change_isr,
					0, "charger-wcin", chip);
	if (ret) {
		dev_err(dev, "failed to reqeust wcin irq: %d\n", ret);
		goto free_chgin_irq;
	}

	ret = MAX77818_FINISH_NON_FGCC_OP(
			max77818,
			fgcc_restore_state,
			"Finishing registration of irq handlers for the charger "
			"interrupts\n");
	if (ret) {
		dev_err(dev,
			"Failed to re-enable FGCC after charger device irq "
			"handler registration\n");
		skip_final_fgcc_op_finish = true;
		goto free_wcin_irq;
	}

	return 0;

free_wcin_irq:
	free_irq(chip->chg_wcin_irq, NULL);
free_chgin_irq:
	free_irq(chip->chg_chgin_irq, NULL);
free_chg_irq:
	free_irq(chip->chg_irq, NULL);
finish_fgcc_op:
	if (!skip_final_fgcc_op_finish)
		if (MAX77818_FINISH_NON_FGCC_OP(
				max77818,
				fgcc_restore_state,
				"Finishing (failed) registration of irq handlers "
				"for the charger interrupts\n"))
			dev_err(dev,
			"Failed to re-enable FGCC after failed charger device "
			"irq handler registration\n");

	return ret;
}

static int max77818_probe(struct platform_device *pdev)
{
	struct max77818_dev *max77818 = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	struct max77818_chip *chip;
	u32 val;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->lock);

	chip->max77818_dev = max77818;
	chip->dev = dev;
	chip->regmap = max77818->regmap_fg;

	chip->pdata = max77818_get_pdata(chip);
	if (!chip->pdata) {
		dev_err(dev, "no platform data provided\n");
		return -EINVAL;
	}

	/* Get non device related driver config from DT */
	max77818_get_driver_config(chip);

	platform_set_drvdata(pdev, chip);
	psy_cfg.drv_data = chip;
	psy_cfg.of_node = dev->of_node;

	SYNC_CLEAR_FLAG(chip->init_complete, &chip->lock);
	chip->battery = devm_power_supply_register(dev, &max77818_psy_desc,
						   &psy_cfg);

	if (IS_ERR(chip->battery)) {
		ret = PTR_ERR(chip->battery);
		dev_err(dev, "failed to register supply: %d \n", ret);
		return ret;
	}

	max77818_init_otg_supply(chip);

	ret = max77818_init_charger_detection(chip);
	if (ret) {
		dev_err(dev, "Failed to init charger detection\n");
		goto unreg_supply;
	}

	ret = max77818_init_fg_interrupt_handling(max77818, chip);
	if (ret) {
		dev_err(dev, "Failed to init FG interrupt handling\n");
		goto unreg_chg_det;
	}

	ret = max77818_init_chg_interrupt_handling(max77818, chip);
	if (ret) {
		dev_err(dev, "Failed to init charger interrupt handling\n");
		goto unreg_fg_irq;
	}

	/* A completion object is initiated in order for init to be run after
	 * completed initiation of this driver is kept waiting until all is done
	 * here.
	 *
	 * Also a worker is scheduled to run which will wait for this until
	 * performing final charger sync. */
	init_completion(&chip->init_completion);
	INIT_WORK(&chip->initial_charger_sync_work,
		  max77818_do_initial_charger_sync_worker);
	schedule_work(&chip->initial_charger_sync_work);

	/* Read the POR bit set when the device boots from a total power loss.
	 * If this bit is set, the device is given its initial config read from
	 * DT.
	 *
	 * If the POR is not set, but the module parameter 'config_update' is
	 * set to 'complete' when booting (by image update scripts typically),
	 * a complete re-config of the FG device is performed in order to apply
	 * battery model parameters in addition to custom params read
	 * from updated DT.
	 *
	 * If the POR is not set, but the module parameter 'config_update' is
	 * set to 'partial' when booting (by image update scripts typically),
	 * a partial re-config of the custom params is performed in order to apply
	 * only custom parameters read from updated DT. */
	regmap_read(chip->regmap, MAX17042_STATUS, &val);
	if ((val & STATUS_POR_BIT) || max77818_do_complete_update(chip)) {
		INIT_WORK(&chip->init_work, max77818_init_worker);
		schedule_work(&chip->init_work);
	} else if (max77818_do_partial_update(chip)) {
		max77818_write_all_custom_params(chip);
		max77818_do_init_completion(chip);
	} else if (max77818_do_param_verification(chip)) {
		max77818_write_mismatched_custom_params(chip);
		max77818_do_init_completion(chip);
	}
	else {
		dev_dbg(chip->dev, "No config change\n");
		max77818_do_init_completion(chip);
	}

	return 0;

unreg_fg_irq:
	free_irq(chip->fg_irq, NULL);
unreg_chg_det:
	usb_unregister_notifier(chip->usb_phy[0],
				&chip->charger_detection_nb[0]);

	usb_unregister_notifier(chip->usb_phy[1],
				&chip->charger_detection_nb[1]);
unreg_supply:
	power_supply_unregister(chip->battery);

	return ret;
}

static struct platform_driver max77818_fg_driver = {
	.driver = {
		.name = MAX77818_FUELGAUGE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = max77818_probe,
};
module_platform_driver(max77818_fg_driver);

MODULE_DESCRIPTION("MAX77818 Fuel Gauge");
MODULE_LICENSE("GPL");
