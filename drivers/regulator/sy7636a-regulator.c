/*
 * Functions to access SY3686A power management chip voltages
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/mfd/sy7636a.h>

static int get_vcom_voltage(struct regulator_dev *rdev)
{
	int ret;
	unsigned int val;

	ret = regmap_read(rdev->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, &val);
	if (ret)
		return ret;

	return (val & 0x1FF) * 10;
}

static const struct regulator_ops sy7636a_vcom_volt_ops = {
	.get_voltage = get_vcom_voltage,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

struct regulator_desc desc = {
	.name = "vcom",
	.id = 0,
	.ops = &sy7636a_vcom_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = SY7636A_REG_OPERATION_MODE_CRL,
	.enable_mask = SY7636A_OPERATION_MODE_CRL_ONOFF,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vcom"),
};

static int sy7636a_regulator_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	int ret;
	unsigned int val;

	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);

	ret = regmap_read(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, &val);
	if (ret) {
		dev_warn(&pdev->dev, "Unable to read vcom value, returned %d\n", ret);
		return ret;
	}

	sy7636a->vcom = val;

	return 0;
}

static int sy7636a_regulator_resume(struct platform_device *pdev)
{
	int ret;

	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);

	if (!sy7636a->vcom || sy7636a->vcom > 0x01FF) {
		dev_warn(&pdev->dev, "Vcom value invalid, and thus not restored\n");
		return -EINVAL;
	}

	ret = regmap_write(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L,
			sy7636a->vcom);
	if (ret) {
		dev_warn(&pdev->dev, "Unable to write vcom value, returned %d\n", ret);
		return ret;
	}

	return 0;
}

static int sy7636a_regulator_probe(struct platform_device *pdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	if (!sy7636a)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, sy7636a);

	config.dev = &pdev->dev;
	config.dev->of_node = sy7636a->dev->of_node;
	config.driver_data = sy7636a;
	config.regmap = sy7636a->regmap;

	rdev = devm_regulator_register(&pdev->dev, &desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(sy7636a->dev, "failed to register %s regulator\n",
			pdev->name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static const struct platform_device_id sy7636a_regulator_id_table[] = {
	{ "sy7636a-regulator", },
};
MODULE_DEVICE_TABLE(platform, sy7636a_regulator_id_table);

static struct platform_driver sy7636a_regulator_driver = {
	.driver = {
		.name = "sy7636a-regulator",
	},
	.probe = sy7636a_regulator_probe,
	.id_table = sy7636a_regulator_id_table,
	.suspend = sy7636a_regulator_suspend,
	.resume = sy7636a_regulator_resume,
};
module_platform_driver(sy7636a_regulator_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("SY7636A voltage regulator driver");
MODULE_LICENSE("GPL v2");
