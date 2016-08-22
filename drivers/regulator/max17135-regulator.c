/*
 * Copyright (C) 2010-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2016 reMarkable AS. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/max17135.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

/*
 * Regulator definitions
 *   *_MIN_uV  - minimum microvolt for regulator
 *   *_MAX_uV  - maximum microvolt for regulator
 *   *_STEP_uV - microvolts between regulator output levels
 *   *_MIN_VAL - minimum register field value for regulator
 *   *_MAX_VAL - maximum register field value for regulator
 */
#define MAX17135_HVINP_MIN_uV    5000000
#define MAX17135_HVINP_MAX_uV   20000000
#define MAX17135_HVINP_STEP_uV   1000000
#define MAX17135_HVINP_MIN_VAL         0
#define MAX17135_HVINP_MAX_VAL        12

#define MAX17135_HVINN_MIN_uV    5000000
#define MAX17135_HVINN_MAX_uV   20000000
#define MAX17135_HVINN_STEP_uV   1000000
#define MAX17135_HVINN_MIN_VAL         0
#define MAX17135_HVINN_MAX_VAL         1

#define MAX17135_GVDD_MIN_uV    5000000
#define MAX17135_GVDD_MAX_uV   20000000
#define MAX17135_GVDD_STEP_uV   1000000
#define MAX17135_GVDD_MIN_VAL         0
#define MAX17135_GVDD_MAX_VAL         1

#define MAX17135_GVEE_MIN_uV    5000000
#define MAX17135_GVEE_MAX_uV   20000000
#define MAX17135_GVEE_STEP_uV   1000000
#define MAX17135_GVEE_MIN_VAL         0
#define MAX17135_GVEE_MAX_VAL         1

#define MAX17135_VCOM_MIN_VAL         0
#define MAX17135_VCOM_MAX_VAL       255

#define MAX17135_VNEG_MIN_uV    5000000
#define MAX17135_VNEG_MAX_uV   20000000
#define MAX17135_VNEG_STEP_uV   1000000
#define MAX17135_VNEG_MIN_VAL         0
#define MAX17135_VNEG_MAX_VAL         1

#define MAX17135_VPOS_MIN_uV    5000000
#define MAX17135_VPOS_MAX_uV   20000000
#define MAX17135_VPOS_STEP_uV   1000000
#define MAX17135_VPOS_MIN_VAL         0
#define MAX17135_VPOS_MAX_VAL         1

#define MAX17135_EXT_TEMP_DEFAULT	25

struct max17135_vcom_programming_data {
	int vcom_min_uV;
	int vcom_max_uV;
	int vcom_step_uV;
};

struct max17135_data {
	int num_regulators;
	struct max17135 *max17135;
	struct regulator_dev **rdev;
};

struct max17135_vcom_programming_data vcom_data[2] = {
	{
		-4325000,
		-500000,
		15000,
	},
	{
		-3050000,
		-500000,
		10000,
	},
};

static int max17135_is_power_good(struct max17135 *max17135);

/*
 * Regulator operations
 */
static int max17135_hvinp_set_voltage(struct regulator_dev *reg,
					int minuV, int uV, unsigned *selector)
{
	unsigned int reg_val;
	unsigned int fld_val;

	if ((uV >= MAX17135_HVINP_MIN_uV) &&
	    (uV <= MAX17135_HVINP_MAX_uV))
		fld_val = (uV - MAX17135_HVINP_MIN_uV) /
			MAX17135_HVINP_STEP_uV;
	else
		return -EINVAL;

	max17135_reg_read(REG_MAX17135_HVINP, &reg_val);

	reg_val &= ~BITFMASK(HVINP);
	reg_val |= BITFVAL(HVINP, fld_val); /* shift to correct bit */

	return max17135_reg_write(REG_MAX17135_HVINP, reg_val);
}

static int max17135_hvinp_get_voltage(struct regulator_dev *reg)
{
	unsigned int reg_val;
	unsigned int fld_val;
	int volt;

	max17135_reg_read(REG_MAX17135_HVINP, &reg_val);

	fld_val = (reg_val & BITFMASK(HVINP)) >> HVINP_LSH;

	if ((fld_val >= MAX17135_HVINP_MIN_VAL) &&
		(fld_val <= MAX17135_HVINP_MAX_VAL)) {
		volt = (fld_val * MAX17135_HVINP_STEP_uV) +
			MAX17135_HVINP_MIN_uV;
	} else {
		printk(KERN_ERR "MAX17135: HVINP voltage is out of range\n");
		volt = 0;
	}
	return volt;
}

static int max17135_hvinp_enable(struct regulator_dev *reg)
{
	return 0;
}

static int max17135_hvinp_disable(struct regulator_dev *reg)
{
	return 0;
}

/* Convert uV to the VCOM register bitfield setting */
static inline int vcom_uV_to_rs(int uV, int pass_num)
{
	return (vcom_data[pass_num].vcom_max_uV - uV)
		/ vcom_data[pass_num].vcom_step_uV;
}

/* Convert the VCOM register bitfield setting to uV */
static inline int vcom_rs_to_uV(int rs, int pass_num)
{
	return vcom_data[pass_num].vcom_max_uV
		- (vcom_data[pass_num].vcom_step_uV * rs);
}

static int set_vcom_voltage(struct max17135 *max17135, int uV)
{
	unsigned int reg_val;

	if ((uV < vcom_data[max17135->pass_num-1].vcom_min_uV)
		|| (uV > vcom_data[max17135->pass_num-1].vcom_max_uV))
		return -EINVAL;

	max17135_reg_read(REG_MAX17135_DVR, &reg_val);
	reg_val &= ~BITFMASK(DVR);
	reg_val |= BITFVAL(DVR, vcom_uV_to_rs(uV,
					      max17135->pass_num-1));

	return max17135_reg_write(REG_MAX17135_DVR, reg_val);
}

/*
 * This function should only be called with positive voltage values because
 * negative ones are considered errors by the regulator core implementation.
 *
 * The given positive value if the absolute value of the desired negative one.
 */
static int max17135_vcom_set_voltage(struct regulator_dev *reg,
					int minuV, int uV, unsigned *selector)

{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	/* Transform uV for our negative land values */
	uV = -uV;

	return set_vcom_voltage(max17135, uV);
}

static int get_vcom_voltage(struct max17135 *max17135)
{
	unsigned int reg_val;
	int uV;

	max17135_reg_read(REG_MAX17135_DVR, &reg_val);
	uV = vcom_rs_to_uV(BITFEXT(reg_val, DVR), max17135->pass_num-1);

	return uV;
}

/*
 * This function should only return positive voltage values because negative
 * ones are considered errors by the regulator core implementation.
 */
static int max17135_vcom_get_voltage(struct regulator_dev *reg)
{
	return (-1) * get_vcom_voltage(rdev_get_drvdata(reg));
}

static int max17135_vcom_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	/* enable VCOM regulator output */
	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_vcom_ctrl, 1);
	else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= ~BITFMASK(VCOM_ENABLE);
		reg_val |= BITFVAL(VCOM_ENABLE, 1); /* shift to correct bit */
		max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
	}

	return 0;
}

static int max17135_vcom_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_vcom_ctrl, 0);
	else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= ~BITFMASK(VCOM_ENABLE);
		max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
	}

	return 0;
}

static int max17135_vcom_is_enabled(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	/* read VCOM regulator enable setting */
	if (max17135->pass_num == 1) {
		int gpio = gpio_get_value(max17135->gpio_pmic_vcom_ctrl);
		if (gpio == 0)
			return 0;
		else
			return 1;
	} else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= BITFMASK(VCOM_ENABLE);
		if (reg_val != 0)
			return 1;
		else
			return 0;
	}
}

static int max17135_is_power_good(struct max17135 *max17135)
{
    unsigned int reg_val;

    if (max17135->pass_num == 1) {
	    return gpio_get_value(max17135->gpio_pmic_pwrgood);
    } else {
	    max17135_reg_read(REG_MAX17135_FAULT, &reg_val);

	    /* Check the POK bit */
	    reg_val = (reg_val & BITFMASK(FAULT_POK)) >> FAULT_POK_LSH;
	    return reg_val;
    }
}

static int max17135_wait_power_good(struct max17135 *max17135)
{
	int i;

	for (i = 0; i < max17135->max_wait * 3; i++) {
		if (max17135_is_power_good(max17135))
			return 0;

		msleep(1);
	}

	return -ETIMEDOUT;
}

static int max17135_enable(struct max17135 *max17135)
{
	int ret;

	/* The Pass 1 parts cannot turn on the PMIC via I2C. */
	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_wakeup, 1);
	else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= ~BITFMASK(ENABLE);
		reg_val |= BITFVAL(ENABLE, 1);
		ret = max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
		if (ret) {
			printk(KERN_ERR "Failed to write to ENABLE register\n");
			return ret;
		}
	}

	ret = max17135_wait_power_good(max17135);
	if (ret == -ETIMEDOUT) {
		printk("MAX17135 timeout while powering up DISPLAY (EPDC_PWRWAKEUP)\n");
	} else if (ret) {
		printk("MAX17135 error while powering up DISPLAY (EPDC_PWRWAKEUP): %d\n", ret);
	}

	return ret;
}

static int max17135_display_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	return max17135_enable(max17135);
}

static int max17135_disable(struct max17135 *max17135)
{
	int ret;

	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_wakeup, 0);
	else {
		unsigned int reg_val;

		ret = max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		if (ret) {
			printk(KERN_ERR "Failed to read DISPLAY via register for disabling\n");
			return ret;
		}

		reg_val &= ~BITFMASK(ENABLE);
		ret = max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
		if (ret) {
			printk(KERN_ERR "Failed to disable DISPLAY via register\n");
			return ret;
		}
	}

	msleep(max17135->max_wait);

	return 0;
}

static int max17135_display_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	return max17135_disable(max17135);
}

static int max17135_display_is_enabled(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	int gpio = gpio_get_value(max17135->gpio_pmic_wakeup);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int max17135_v3p3_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	gpio_set_value(max17135->gpio_pmic_v3p3, 1);
	return 0;
}

static int max17135_v3p3_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	gpio_set_value(max17135->gpio_pmic_v3p3, 0);
	return 0;
}

static int max17135_v3p3_is_enabled(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	int gpio = gpio_get_value(max17135->gpio_pmic_v3p3);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int max17135_tmst_get_temperature(struct regulator_dev *reg)
{
    struct max17135 *max17135 = rdev_get_drvdata(reg);
	unsigned int reg_val;
	int retry, temp;

	for (retry = 0; retry < 50; retry++) {
		/* max 500ms after VIN> VIN_UVLO and VDD>VDD_UVLO */
		if (max17135_reg_read(REG_MAX17135_EXT_TEMP, &reg_val) ==
				0) {
			reg_val >>= 8;
			if (reg_val&0x80) {
				reg_val = ((~reg_val)&0xFF)+1;
				temp = (0 - (int)reg_val);
			} else {
				temp = (int)reg_val;
			}
			dev_dbg(max17135->dev, "EXT temperature = %d after waiting %d ms\n",
				temp, retry*10);
			return temp;
		}
		msleep(10);
	}
	dev_dbg(max17135->dev, "Unable to read temperature, use default=%d\n",
		MAX17135_EXT_TEMP_DEFAULT);

	return MAX17135_EXT_TEMP_DEFAULT;
}

/*
 * Regulator operations
 */

static struct regulator_ops max17135_display_ops = {
	.enable = max17135_display_enable,
	.disable = max17135_display_disable,
	.is_enabled = max17135_display_is_enabled,
};

static struct regulator_ops max17135_gvdd_ops = {
};

static struct regulator_ops max17135_gvee_ops = {
};

static struct regulator_ops max17135_hvinn_ops = {
};

static struct regulator_ops max17135_hvinp_ops = {
	.enable = max17135_hvinp_enable,
	.disable = max17135_hvinp_disable,
	.get_voltage = max17135_hvinp_get_voltage,
	.set_voltage = max17135_hvinp_set_voltage,
};

static struct regulator_ops max17135_vcom_ops = {
	.enable = max17135_vcom_enable,
	.disable = max17135_vcom_disable,
	.get_voltage = max17135_vcom_get_voltage,
	.set_voltage = max17135_vcom_set_voltage,
	.is_enabled = max17135_vcom_is_enabled,
};

static struct regulator_ops max17135_vneg_ops = {
};

static struct regulator_ops max17135_vpos_ops = {
};

static struct regulator_ops max17135_v3p3_ops = {
	.enable = max17135_v3p3_enable,
	.disable = max17135_v3p3_disable,
	.is_enabled = max17135_v3p3_is_enabled,
};

static struct regulator_ops max17135_tmst_ops = {
       .get_voltage = max17135_tmst_get_temperature,
};


/*
 * Regulator descriptors
 */
static struct regulator_desc max17135_reg[MAX17135_NUM_REGULATORS] = {
{
	.name = "DISPLAY",
	.id = MAX17135_DISPLAY,
	.ops = &max17135_display_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "GVDD",
	.id = MAX17135_GVDD,
	.ops = &max17135_gvdd_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "GVEE",
	.id = MAX17135_GVEE,
	.ops = &max17135_gvee_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "HVINN",
	.id = MAX17135_HVINN,
	.ops = &max17135_hvinn_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "HVINP",
	.id = MAX17135_HVINP,
	.ops = &max17135_hvinp_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VCOM",
	.id = MAX17135_VCOM,
	.ops = &max17135_vcom_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VNEG",
	.id = MAX17135_VNEG,
	.ops = &max17135_vneg_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VPOS",
	.id = MAX17135_VPOS,
	.ops = &max17135_vpos_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "V3P3",
	.id = MAX17135_V3P3,
	.ops = &max17135_v3p3_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "TMST",
	.id = MAX17135_TMST,
	.ops = &max17135_tmst_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};

static void max17135_setup_timings(struct max17135 *max17135)
{
	unsigned int reg_val;

	int timing1, timing2, timing3, timing4,
		timing5, timing6, timing7, timing8;

	max17135_reg_read(REG_MAX17135_TIMING1, &timing1);
	max17135_reg_read(REG_MAX17135_TIMING2, &timing2);
	max17135_reg_read(REG_MAX17135_TIMING3, &timing3);
	max17135_reg_read(REG_MAX17135_TIMING4, &timing4);
	max17135_reg_read(REG_MAX17135_TIMING5, &timing5);
	max17135_reg_read(REG_MAX17135_TIMING6, &timing6);
	max17135_reg_read(REG_MAX17135_TIMING7, &timing7);
	max17135_reg_read(REG_MAX17135_TIMING8, &timing8);

	if ((timing1 != max17135->gvee_pwrup) ||
		(timing2 != max17135->vneg_pwrup) ||
		(timing3 != max17135->vpos_pwrup) ||
		(timing4 != max17135->gvdd_pwrup) ||
		(timing5 != max17135->gvdd_pwrdn) ||
		(timing6 != max17135->vpos_pwrdn) ||
		(timing7 != max17135->vneg_pwrdn) ||
		(timing8 != max17135->gvee_pwrdn)) {
		max17135_reg_write(REG_MAX17135_TIMING1, max17135->gvee_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING2, max17135->vneg_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING3, max17135->vpos_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING4, max17135->gvdd_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING5, max17135->gvdd_pwrdn);
		max17135_reg_write(REG_MAX17135_TIMING6, max17135->vpos_pwrdn);
		max17135_reg_write(REG_MAX17135_TIMING7, max17135->vneg_pwrdn);
		max17135_reg_write(REG_MAX17135_TIMING8, max17135->gvee_pwrdn);

		reg_val = BITFVAL(CTRL_TIMING, true); /* shift to correct bit */
		max17135_reg_write(REG_MAX17135_PRGM_CTRL, reg_val);
	}
}

#define CHECK_PROPERTY_ERROR_KFREE(prop) \
do { \
	int ret = of_property_read_u32(max17135->dev->of_node, \
					#prop, &max17135->prop); \
	if (ret < 0) { \
		return ret;	\
	}	\
} while (0);

#ifdef CONFIG_OF
static int max17135_pmic_dt_parse_pdata(struct platform_device *pdev,
					struct max17135_platform_data *pdata)
{
	struct max17135 *max17135 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct max17135_regulator_data *rdata;
	int i, ret;

	pmic_np = of_node_get(max17135->dev->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	pdata->num_regulators = of_get_child_count(regulators_np);
	dev_dbg(&pdev->dev, "num_regulators %d\n", pdata->num_regulators);

	rdata = devm_kzalloc(&pdev->dev, sizeof(*rdata) *
				pdata->num_regulators, GFP_KERNEL);
	if (!rdata) {
		of_node_put(regulators_np);
		dev_err(&pdev->dev, "could not allocate memory for"
			"regulator data\n");
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(max17135_reg); i++)
			if (!of_node_cmp(reg_np->name, max17135_reg[i].name))
				break;

		if (i == ARRAY_SIZE(max17135_reg)) {
			dev_warn(&pdev->dev, "don't know how to configure"
				"regulator %s\n", reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(&pdev->dev,
							     reg_np,
							     &max17135_reg[i]);
		rdata->reg_node = reg_np;
		rdata++;
	}
	of_node_put(regulators_np);

	CHECK_PROPERTY_ERROR_KFREE(vneg_pwrup);
	CHECK_PROPERTY_ERROR_KFREE(gvee_pwrup);
	CHECK_PROPERTY_ERROR_KFREE(vpos_pwrup);
	CHECK_PROPERTY_ERROR_KFREE(gvdd_pwrup);
	CHECK_PROPERTY_ERROR_KFREE(gvdd_pwrdn);
	CHECK_PROPERTY_ERROR_KFREE(vpos_pwrdn);
	CHECK_PROPERTY_ERROR_KFREE(gvee_pwrdn);
	CHECK_PROPERTY_ERROR_KFREE(vneg_pwrdn);

	if (of_property_read_u32(max17135->dev->of_node, "pass_num", &max17135->pass_num))
		max17135->pass_num = 2;

	dev_dbg(&pdev->dev, "vneg_pwrup %d, vneg_pwrdn %d, vpos_pwrup %d,"
		"vpos_pwrdn %d, gvdd_pwrup %d, gvdd_pwrdn %d, gvee_pwrup %d,"
		"gvee_pwrdn %d\n", max17135->vneg_pwrup, max17135->vneg_pwrdn,
		max17135->vpos_pwrup, max17135->vpos_pwrdn,
		max17135->gvdd_pwrup, max17135->gvdd_pwrdn,
		max17135->gvee_pwrup, max17135->gvee_pwrdn);

	max17135->max_wait = max17135->vpos_pwrup + max17135->vneg_pwrup +
		max17135->gvdd_pwrup + max17135->gvee_pwrup;

	max17135->gpio_pmic_wakeup = of_get_named_gpio(pmic_np,
					"gpio_pmic_wakeup", 0);
	if (!gpio_is_valid(max17135->gpio_pmic_wakeup)) {
		dev_err(&pdev->dev, "no epdc pmic wakeup pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, max17135->gpio_pmic_wakeup,
				GPIOF_OUT_INIT_LOW, "epdc-pmic-wake");
	if (ret < 0)
		goto err;

	max17135->gpio_pmic_vcom_ctrl = of_get_named_gpio(pmic_np,
					"gpio_pmic_vcom_ctrl", 0);
	if (!gpio_is_valid(max17135->gpio_pmic_vcom_ctrl)) {
		dev_err(&pdev->dev, "no epdc pmic vcom_ctrl pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, max17135->gpio_pmic_vcom_ctrl,
				GPIOF_OUT_INIT_LOW, "epdc-vcom");
	if (ret < 0)
		goto err;

	max17135->gpio_pmic_v3p3 = of_get_named_gpio(pmic_np,
					"gpio_pmic_v3p3", 0);
	if (!gpio_is_valid(max17135->gpio_pmic_v3p3)) {
		dev_err(&pdev->dev, "no epdc pmic v3p3 pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, max17135->gpio_pmic_v3p3,
				GPIOF_OUT_INIT_LOW, "epdc-v3p3");
	if (ret < 0)
		goto err;

	max17135->gpio_pmic_intr = of_get_named_gpio(pmic_np,
					"gpio_pmic_intr", 0);
	if (!gpio_is_valid(max17135->gpio_pmic_intr)) {
		dev_err(&pdev->dev, "no epdc pmic intr pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, max17135->gpio_pmic_intr,
				GPIOF_IN, "epdc-pmic-int");
	if (ret < 0)
		goto err;

	max17135->gpio_pmic_pwrgood = of_get_named_gpio(pmic_np,
					"gpio_pmic_pwrgood", 0);
	if (!gpio_is_valid(max17135->gpio_pmic_pwrgood)) {
		dev_err(&pdev->dev, "no epdc pmic pwrgood pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, max17135->gpio_pmic_pwrgood,
				GPIOF_IN, "epdc-pwrstat");
	if (ret < 0)
		goto err;

err:
	return 0;

}
#else
static int max17135_pmic_dt_parse_pdata(struct platform_device *pdev,
					struct max17135 *max17135)
{
	return 0;
}
#endif	/* !CONFIG_OF */

static ssize_t max17135_vcom_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct max17135 *max17135 = dev_get_drvdata(dev->parent);
	int voltage;

	voltage = get_vcom_voltage(max17135);

	return sprintf(buf, "%d\n", voltage);
}

static ssize_t max17135_vcom_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct max17135 *max17135 = dev_get_drvdata(dev->parent);
	int vcom_uV, ret;
	unsigned int reg_val;
	bool powered_by_us = false;

	if (sscanf(buf, " %d", &vcom_uV) <= 0) {
		dev_err(dev, "Invalid vcom value given: %s\n", buf);
		return -EINVAL;
	}

	/*
	 * Only program VCOM if it is not set to the desired value.
	 * Programming VCOM excessively degrades ability to keep
	 * DVR register value persistent.
	 */
	if (vcom_uV == get_vcom_voltage(max17135)) {
		return size;
	}

	ret = set_vcom_voltage(max17135, vcom_uV);
	if (ret < 0) {
		dev_err(dev, "Failed to set VCOM to value given(%s): %d\n", buf, ret);
		return -EINVAL;
	}

	/* Write to non-volatile memory */

	/* First check if we need to power up the PMIC */
	if (!max17135_is_power_good(max17135)) {
		powered_by_us = true;
		max17135_enable(max17135);

		ret = max17135_wait_power_good(max17135);
		if (ret < 0) {
			printk(KERN_ERR "Unable to wake PMIC to store VCOM: %d\n", ret);
			return -EINVAL;
		}
	}

	reg_val = BITFVAL(CTRL_DVR, true); /* shift to correct bit */
	ret = max17135_reg_write(REG_MAX17135_PRGM_CTRL, reg_val);

	if (ret < 0) {
		printk(KERN_ERR "MAX17135 failed to store VCOM to non-volatile memory: %d\n", ret);
	}

	/* Power it down again if we powered it up */
	if (powered_by_us) {
		max17135_disable(max17135);
	}

	return size;
}

static DEVICE_ATTR(vcom, 0600, max17135_vcom_show, max17135_vcom_store);


/*
 * Regulator init/probing/exit functions
 */
static int max17135_regulator_probe(struct platform_device *pdev)
{
	struct max17135 *max17135 = dev_get_drvdata(pdev->dev.parent);
	struct max17135_platform_data *pdata = max17135->pdata;
	struct max17135_data *priv;
	struct regulator_dev **rdev;
	struct regulator_config config = { };
	int size, i, ret = 0;

	if (max17135->dev->of_node) {
		ret = max17135_pmic_dt_parse_pdata(pdev, pdata);
		if (ret)
			return ret;
	}
	priv = devm_kzalloc(&pdev->dev, sizeof(struct max17135_data),
			       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	priv->rdev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!priv->rdev)
		return -ENOMEM;

	rdev = priv->rdev;
	priv->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, priv);

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;

		config.dev = max17135->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = max17135;
		config.of_node = pdata->regulators[i].reg_node;

		rdev[i] = regulator_register(&max17135_reg[id], &config);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n",
					id);
			rdev[i] = NULL;
			goto err;
		}
	}

	/*
	 * Set up PMIC timing values.
	 * Should only be done one time!  Timing values may only be
	 * changed a limited number of times according to spec.
	 */
	max17135_setup_timings(max17135);

	/* Create sysfs file for writing VCOM value */
	if (device_create_file(&pdev->dev, &dev_attr_vcom) < 0) {
		dev_err(&pdev->dev, "Unable to create sysfs file for vcom\n");
	}

	return 0;
err:
	while (--i >= 0)
		regulator_unregister(rdev[i]);
	return ret;
}

static int max17135_regulator_remove(struct platform_device *pdev)
{
	struct max17135_data *priv = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = priv->rdev;
	int i;

	for (i = 0; i < priv->num_regulators; i++)
		regulator_unregister(rdev[i]);

	device_remove_file(&pdev->dev, &dev_attr_vcom);

	return 0;
}

static const struct platform_device_id max17135_pmic_id[] = {
	{ "max17135-pmic", 0},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, max17135_pmic_id);

static struct platform_driver max17135_regulator_driver = {
	.probe = max17135_regulator_probe,
	.remove = max17135_regulator_remove,
	.id_table = max17135_pmic_id,
	.driver = {
		.name = "max17135-pmic",
	},
};

static int __init max17135_regulator_init(void)
{
	return platform_driver_register(&max17135_regulator_driver);
}
subsys_initcall_sync(max17135_regulator_init);

static void __exit max17135_regulator_exit(void)
{
	platform_driver_unregister(&max17135_regulator_driver);
}
module_exit(max17135_regulator_exit);

/* Module information */
MODULE_DESCRIPTION("MAX17135 regulator driver");
MODULE_LICENSE("GPL");
