/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver is based on max77823-regulator.c
 */

#include <linux/version.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regmap.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/module.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>

#define REG_SAFEOUTCTRL		0xC6
#define BIT_SAFEOUT1		GENMASK(1, 0)
#define BIT_SAFEOUT2		GENMASK(3, 2)
#define BIT_ACTDISSAFEO1	BIT(4)
#define BIT_ACTDISSAFEO2	BIT(5)
#define BIT_ENSAFEOUT1		BIT(6)
#define BIT_ENSAFEOUT2 		BIT(7)

/* MAX77818 regulator IDs */
enum max77818_regulators {
	MAX77818_SAFEOUT1 = 0,
	MAX77818_SAFEOUT2,
};

const static unsigned int max77818_safeout_volt_table[] =
{
	4850000, 4900000, 4950000, 3300000,
};

static struct regulator_ops max77818_safeout_ops = {
	.list_voltage		= regulator_list_voltage_table,
	.map_voltage		= regulator_map_voltage_ascend,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

#define REGULATOR_DESC_SFO(num, vsel_m, enable_m) { \
	.name		= "SAFEOUT"#num, \
	.of_match	= of_match_ptr("SAFEOUT"#num), \
	.regulators_node = of_match_ptr("regulators"), \
	.id		= MAX77818_SAFEOUT##num, \
	.ops		= &max77818_safeout_ops, \
	.type		= REGULATOR_VOLTAGE, \
	.owner		= THIS_MODULE, \
	.n_voltages	= ARRAY_SIZE(max77818_safeout_volt_table), \
	.volt_table	= max77818_safeout_volt_table, \
	.vsel_reg	= REG_SAFEOUTCTRL, \
	.vsel_mask	= (vsel_m), \
	.enable_reg	= REG_SAFEOUTCTRL, \
	.enable_mask	= (enable_m), \
}

static const struct regulator_desc max77818_safeout_descs[] = {
	REGULATOR_DESC_SFO(1, BIT_SAFEOUT1, BIT_ENSAFEOUT1),
	REGULATOR_DESC_SFO(2, BIT_SAFEOUT2, BIT_ENSAFEOUT2),
};

static int max77818_regulator_probe(struct platform_device *pdev)
{
	struct max77818_dev *max77818 = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = {};
	struct device *dev = &pdev->dev;
	int i, ret;

	config.dev = max77818->dev;
	config.driver_data = max77818;
	config.regmap = max77818->regmap_pmic;

	for (i = 0; i < ARRAY_SIZE(max77818_safeout_descs); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &max77818_safeout_descs[i],
					       &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(dev, "regulator init failed for %s: %d\n",
				max77818_safeout_descs[i].name, ret);
			return ret;
		}
	}

	return 0;
}

static struct platform_driver max77818_regulator_driver = {
	.driver = {
		   .name = MAX77818_REGULATOR_NAME,
		   .owner = THIS_MODULE,
	},
	.probe = max77818_regulator_probe,
};
module_platform_driver(max77818_regulator_driver);

MODULE_AUTHOR("TaiEup Kim<clark.kim@maximintegrated.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAXIM 77818 Regulator Driver");
