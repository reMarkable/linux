/*
 * Fuel gauge driver for Maxim 77818
 *  Note that Maxim 77818 is mfd and this is its subdevice.
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
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
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/platform_device.h>

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

struct max77818_chip {
	struct device *dev;
	int irq;
	struct regmap *regmap;
	struct power_supply *battery;
	struct max17042_platform_data *pdata;
	struct work_struct work;
	int init_complete;
};

static enum power_supply_property max77818_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

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

static int max77818_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct max77818_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	int ret;
	u32 data;
	u64 data64;

	if (!chip->init_complete)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = max77818_get_status(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(map, MAX17042_STATUS, &data);
		if (ret < 0)
			return ret;

		if (data & MAX17042_STATUS_BattAbsent)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
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
	case POWER_SUPPLY_PROP_TEMP_MIN:
		val->intval = chip->pdata->temp_min;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		val->intval = chip->pdata->temp_max;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max77818_get_battery_health(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
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

static int max77818_write_verify_reg(struct regmap *map, u8 reg, u32 value)
{
	int retries = 8;
	u32 read_value;
	int ret;

	do {
		ret = regmap_write(map, reg, value);
		regmap_read(map, reg, &read_value);
		if (read_value != value) {
			ret = -EIO;
			retries--;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}

static inline void max10742_unlock_model(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_MLOCKReg1, MODEL_UNLOCK1);
	regmap_write(map, MAX17042_MLOCKReg2, MODEL_UNLOCK2);
}

static inline void max10742_lock_model(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_MLOCKReg1, MODEL_LOCK1);
	regmap_write(map, MAX17042_MLOCKReg2, MODEL_LOCK2);
}

static inline void max77818_write_model_data(struct max77818_chip *chip,
					     u16 *data, int count)
{
	struct regmap *map = chip->regmap;
	int i;

	for (i = 0; i < count; i++)
		regmap_write(map, MAX17042_MODELChrTbl + i, data[i]);
}

static inline void max77818_read_model_data(struct max77818_chip *chip,
					    u16 *data, int count)
{
	struct regmap *map = chip->regmap;
	u32 tmp;
	int i;

	for (i = 0; i < count; i++) {
		regmap_read(map, MAX17042_MODELChrTbl + i, &tmp);
		data[i] = (u16)tmp;
	}
}

static inline int max77818_model_data_compare(struct max77818_chip *chip,
					u16 *data1, u16 *data2, int size)
{
	int i;

	if (memcmp(data1, data2, size)) {
		dev_err(chip->dev, "%s compare failed\n", __func__);
		for (i = 0; i < size; i++)
			dev_info(chip->dev, "0x%x, 0x%x", data1[i], data2[i]);
		dev_info(chip->dev, "\n");
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

	max10742_unlock_model(chip);
	max77818_write_model_data(chip, data, count);
	max77818_read_model_data(chip, rdata, count);

	ret = max77818_model_data_compare(chip, data, rdata, count);
	if (ret)
		dev_err(chip->dev, "model data compare failed: %d\n", ret);

	max10742_lock_model(chip);
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

	max77818_read_model_data(chip, data, count);

	for (i = 0; i < count; i++) {
		if (data[i]) {
			ret = -EINVAL;
			break;
		}
	}

	kfree(data);
	return ret;
}

static int max77818_model_loading(struct max77818_chip *chip)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(400);
	struct regmap *map = chip->regmap;
	u32 config2;

	/* Setting LdMdl bit */
	regmap_read(map, MAX77818_Config2, &config2);
	regmap_write(map, MAX77818_Config2, config2 | CONFIG2_LDMDL);

	/* Poll LdMdl bit to be 0 under 400ms */
	do {
		regmap_read(map, MAX77818_Config2, &config2);
		if ((config2 & CONFIG2_LDMDL) == 0)
			break;
		if (time_after(jiffies, timeout))
			break;
	} while (1);

	regmap_read(map, MAX77818_Config2, &config2);

	return (config2 & CONFIG2_LDMDL) ? -ETIMEDOUT : 0;
}

static inline u16 max77818_read_param(struct max77818_chip *chip, const char *param)
{
	struct device_node *np = chip->dev->of_node;
	u16 value = 0;

	if (of_property_read_u16(np, param, &value) != 0)
		dev_warn(chip->dev, "failed to read parameter: %s\n", param);

	return value;
}

static void max77818_write_custom_params(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	u32 value;

	regmap_write(map, MAX17042_RepCap, 0);
	regmap_write(map, MAX17042_RelaxCFG,
		     max77818_read_param(chip, "maxim,relax-cfg"));

	/* Unlock extra config registers for write access */
	regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);

	regmap_read(map, MAX17042_VFSOC, &value);
	max77818_write_verify_reg(map, MAX17042_VFSOC0, value);

	regmap_write(map, MAX17042_LearnCFG,
		     max77818_read_param(chip, "maxim,learn-cfg"));
	regmap_write(map, MAX17042_CONFIG,
		     max77818_read_param(chip, "maxim,config"));
	regmap_write(map, MAX77818_Config2,
		     max77818_read_param(chip, "maxim,config2"));
	regmap_write(map, MAX17047_FullSOCThr,
		     max77818_read_param(chip, "maxim,full-soc-threshold"));
	max77818_write_verify_reg(map, MAX17042_FullCAP0,
				  max77818_read_param(chip, "maxim,fullcaprep"));
	regmap_write(map, MAX17042_DesignCap,
		     max77818_read_param(chip, "maxim,design-cap"));
	max77818_write_verify_reg(map, MAX17042_dPacc,
				  max77818_read_param(chip, "maxim,dpacc"));
	max77818_write_verify_reg(map, MAX17042_dQacc,
				  max77818_read_param(chip, "maxim,dqacc"));
	max77818_write_verify_reg(map, MAX17042_FullCAPNom,
				  max77818_read_param(chip, "maxim,fullcapnom"));
	regmap_write(map, MAX17042_MiscCFG,
		     max77818_read_param(chip, "maxim,misc-cfg"));
	regmap_write(map, MAX17047_V_empty,
		     max77818_read_param(chip, "maxim,v-empty"));
	max77818_write_verify_reg(map, MAX17047_QRTbl00,
				  max77818_read_param(chip, "maxim,qresidual00"));
	max77818_write_verify_reg(map, MAX17047_QRTbl10,
				  max77818_read_param(chip, "maxim,qresidual10"));
	max77818_write_verify_reg(map, MAX17047_QRTbl20,
				  max77818_read_param(chip, "maxim,qresidual20"));
	max77818_write_verify_reg(map, MAX17047_QRTbl30,
				  max77818_read_param(chip, "maxim,qresidual30"));
	max77818_write_verify_reg(map, MAX17042_RCOMP0,
				  max77818_read_param(chip, "maxim,rcomp0"));
	max77818_write_verify_reg(map, MAX17042_TempCo,
				  max77818_read_param(chip, "maxim,tempco"));
	regmap_write(map, MAX17042_ICHGTerm,
		     max77818_read_param(chip, "maxim,ichg-term"));
	regmap_write(map, MAX17042_FilterCFG,
		     max77818_read_param(chip, "maxim,filter-cfg"));
	regmap_write(map, MAX17042_LAvg_empty,
		     max77818_read_param(chip, "maxim,iavg-empty"));

	/* The order of the following ones should be respected */
	regmap_write(map, MAX17042_AtRate,
		     max77818_read_param(chip, "maxim,at-rate"));
	regmap_write(map, MAX77818_SmartChgCfg,
		     max77818_read_param(chip, "maxim,smart-chg-cfg"));
	regmap_write(map, MAX77818_ConvgCfg,
		     max77818_read_param(chip, "maxim,convgcfg"));

	/* Back to lock */
	regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_LOCK);
}

static int max77818_init_chip(struct max77818_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret;

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

	/* Write custom parameters from devcie tree */
	max77818_write_custom_params(chip);

	/* Initiate model loading for MAX77818 */
	ret = max77818_model_loading(chip);
	if (ret) {
		dev_err(chip->dev, "initiate model loading failed: %d\n", ret);
		return ret;
	}

	/* Wait 500 ms for SOC to be calculated from the new parameters */
	msleep(500);

	/* Init complete, Clear the POR bit */
	regmap_update_bits(map, MAX17042_STATUS, STATUS_POR_BIT, 0x0);

	return 0;
}

static void max77818_set_soc_threshold(struct max77818_chip *chip, u16 off)
{
	struct regmap *map = chip->regmap;
	u32 soc, soc_tr;

	/* program interrupt threshold such that we should
	 * get interrupt for every 'off' percent change in the soc
	 */
	regmap_read(map, MAX17042_RepSOC, &soc);
	soc >>= 8;
	soc_tr = (soc + off) << 8;
	soc_tr |= (soc - off);
	regmap_write(map, MAX17042_SALRT_Th, soc_tr);
}

static irqreturn_t max77818_thread_handler(int id, void *dev)
{
	struct max77818_chip *chip = dev;
	u32 val;

	regmap_read(chip->regmap, MAX17042_STATUS, &val);
	if ((val & STATUS_INTR_SOCMIN_BIT) || (val & STATUS_INTR_SOCMAX_BIT)) {
		dev_dbg(chip->dev, "SOC threshold INTR\n");
		max77818_set_soc_threshold(chip, 1);
	}

	power_supply_changed(chip->battery);

	return IRQ_HANDLED;
}

static void max77818_init_worker(struct work_struct *work)
{
	struct max77818_chip *chip = container_of(work, struct max77818_chip,
						  work);
	int ret;

	ret = max77818_init_chip(chip);
	if (ret) {
		dev_err(chip->dev, "failed to init chip: %d\n", ret);
		return;
	}

	chip->init_complete = 1;
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

	chip->dev = dev;
	chip->regmap = max77818->regmap_fg;

	chip->pdata = max77818_get_pdata(chip);
	if (!chip->pdata) {
		dev_err(dev, "no platform data provided\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, chip);
	psy_cfg.drv_data = chip;
	psy_cfg.of_node = dev->of_node;

	chip->battery = devm_power_supply_register(dev, &max77818_psy_desc,
						   &psy_cfg);
	if (IS_ERR(chip->battery)) {
		ret = PTR_ERR(chip->battery);
		dev_err(dev, "failed to regiser supply: %d \n", ret);
		return ret;
	}

	chip->irq = regmap_irq_get_virq(max77818->irqc_intsrc, MAX77818_FG_INT);
	if (chip->irq <= 0) {
		dev_err(dev, "failed to get virq: %d\n", chip->irq);
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(dev, chip->irq, NULL,
					max77818_thread_handler, 0,
					chip->battery->desc->name,
					chip);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		return ret;
	}

	/* Get interrupt on each %1 change of SOC threshold */
	regmap_update_bits(chip->regmap, MAX17042_CONFIG,
			   CONFIG_ALRT_BIT_ENBL, CONFIG_ALRT_BIT_ENBL);
	max77818_set_soc_threshold(chip, 1);

	regmap_read(chip->regmap, MAX17042_STATUS, &val);
	if (val & STATUS_POR_BIT) {
		INIT_WORK(&chip->work, max77818_init_worker);
		schedule_work(&chip->work);
	} else {
		chip->init_complete = 1;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77818_suspend(struct device *dev)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);

	/*
	 * disable the irq and enable irq_wake
	 * capability to the interrupt line.
	 */
	disable_irq(chip->irq);
	enable_irq_wake(chip->irq);

	return 0;
}

static int max77818_resume(struct device *dev)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);

	disable_irq_wake(chip->irq);
	enable_irq(chip->irq);
	/* re-program the SOC threshold to 1% change */
	max77818_set_soc_threshold(chip, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(max77818_pm_ops, max77818_suspend,
			 max77818_resume);

static struct platform_driver max77818_fg_driver = {
	.driver = {
		.name = MAX77818_FUELGAUGE_NAME,
		.owner = THIS_MODULE,
		.pm = &max77818_pm_ops,
	},
	.probe = max77818_probe,
};
module_platform_driver(max77818_fg_driver);

MODULE_DESCRIPTION("MAX77818 Fuel Gauge");
MODULE_LICENSE("GPL");
