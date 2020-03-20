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
#include <linux/gpio/consumer.h>

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

static int get_power_good(struct regulator_dev *rdev, bool *is_good)
{
	int ret;
	unsigned int val;

	ret = regmap_read(rdev->regmap, SY7636A_REG_FAULT_FLAG, &val);
	if (ret)
		return ret;

	*is_good = (val & SY7636A_FAULT_FLAG_PG);
	return ret;
}

static int enable_regulator_with_pwr_good_verification(struct regulator_dev *rdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(rdev->dev.parent);
	bool pwr_good = false;
	int wait_cnt = 0;
	int ret;
	unsigned long t0, t1;
	bool use_fault_reg = false;

	dev_dbg(&rdev->dev, "enabling regulator\n");

	t0 = jiffies;
	ret = regulator_enable_regmap(rdev);
	if (ret)
		return ret;

	/* WAIT FOR PWR-GOOD */
	while(!pwr_good && (wait_cnt < 500)) {
		if (!sy7636a->pgood_gpio | use_fault_reg) {
			/* dev_dbg(&rdev->dev, "Reading FAULT FLAG reg\n"); */
			ret = get_power_good(rdev, &pwr_good);
			if (ret)
				return ret;
		} else {
			/* dev_dbg(&rdev->dev, "Reading PGOOD GPIO\n"); */
			ret = gpiod_get_value_cansleep(sy7636a->pgood_gpio);
			if (ret < 0) {
				dev_err(&rdev->dev,
					"failed to read pgood gpio: %d, "
					"falling back to FALT FLAG register\n",
					ret);
				use_fault_reg = true;
			}
			pwr_good = (ret > 0);
		}

		if (!pwr_good) {
			usleep_range(1000, 1500);
			wait_cnt++;
		}
	}
	t1 = jiffies;

	if (!pwr_good) {
		dev_dbg(&rdev->dev, "pwr STILL NOT good after 500 ms\n");
		return -ETIME;
	}

	dev_dbg(&rdev->dev, "pwr GOOD (took %du ms, %d waits) !\n",
		jiffies_to_msecs(t1 - t0),
		wait_cnt);

	return 0;
}

static const struct regulator_ops sy7636a_vcom_volt_ops = {
	.get_voltage = get_vcom_voltage,
	.enable = enable_regulator_with_pwr_good_verification,
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

static int sy7636a_regulator_suspend(struct device *dev)
{
	int ret;
	unsigned int val;

	struct sy7636a *sy7636a = dev_get_drvdata(dev->parent);

	ret = regmap_read(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, &val);
	if (ret) {
		dev_warn(dev, "Unable to read vcom value, returned %d\n", ret);
		return ret;
	}

	sy7636a->vcom = val;

	return 0;
}

static int sy7636a_regulator_resume(struct device *dev)
{
	int ret;

	struct sy7636a *sy7636a = dev_get_drvdata(dev->parent);

	if (!sy7636a->vcom || sy7636a->vcom > 0x01FF) {
		dev_warn(dev, "Vcom value invalid, and thus not restored\n");
		return -EINVAL;
	}

	ret = regmap_write(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L,
			sy7636a->vcom);
	if (ret) {
		dev_warn(dev, "Unable to write vcom value, returned %d\n", ret);
		return ret;
	}

	return 0;
}

static int sy7636a_regulator_probe(struct platform_device *pdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct gpio_desc *gdp;

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

	/* Register gpio 181 (EPD PMIC PGOOD) */
	gdp = devm_gpiod_get(sy7636a->dev, "epd-pwr-good", GPIOD_IN);
	if (IS_ERR(gdp)) {
		if (PTR_ERR(gdp) != -ENOENT)
			dev_warn(sy7636a->dev,
				 "epd-pwr-good GPIO not given in DT, "
				 "falling back to reading FAULT FLAG reg\n");

		if (PTR_ERR(gdp) != -ENOSYS)
			dev_warn(sy7636a->dev,
				 "epd-pwr-good GPIO given is not valid, "
				 "falling back to reading FAULT FLAG reg\n");
	}
	else {
		sy7636a->pgood_gpio = gdp;
		dev_info(sy7636a->dev,
			"epd-pwr-good gpio registered (gpio %d)\n",
			desc_to_gpio(sy7636a->pgood_gpio));
	}

	return 0;
}

static const struct platform_device_id sy7636a_regulator_id_table[] = {
	{ "sy7636a-regulator", },
};
MODULE_DEVICE_TABLE(platform, sy7636a_regulator_id_table);

static const struct dev_pm_ops sy7636a_pm_ops = {
	.suspend = sy7636a_regulator_suspend,
	.resume = sy7636a_regulator_resume,
};

static struct platform_driver sy7636a_regulator_driver = {
	.driver = {
		.name = "sy7636a-regulator",
		.pm = &sy7636a_pm_ops,
	},
	.probe = sy7636a_regulator_probe,
	.id_table = sy7636a_regulator_id_table,
};
module_platform_driver(sy7636a_regulator_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("SY7636A voltage regulator driver");
MODULE_LICENSE("GPL v2");
