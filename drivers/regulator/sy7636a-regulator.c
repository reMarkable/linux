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
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>

#include <linux/mfd/sy7636a.h>

static int get_vcom_voltage_op(struct regulator_dev *rdev)
{
	int ret;

	ret = get_vcom_voltage_mv(rdev->regmap);
	if (ret < 0)
		return ret;

	return ret * 1000;
}

static int disable_regulator(struct regulator_dev *rdev)
{
	int ret;

	ret = regulator_disable_regmap(rdev);
	usleep_range(30000, 35000);

	return ret;
}

static int enable_regulator_pgood(struct regulator_dev *rdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(rdev->dev.parent);
	int pwr_good = 0;
	int ret;
	unsigned long t0, t1;
	const unsigned int wait_time = 500;
	unsigned int wait_cnt;

	t0 = jiffies;

	ret = regulator_enable_regmap(rdev);
	if (ret)
		return ret;

	for (wait_cnt = 0; wait_cnt < wait_time; wait_cnt++) {
		pwr_good = gpiod_get_value_cansleep(sy7636a->pgood_gpio);
		if (pwr_good < 0) {
			dev_err(&rdev->dev, "Failed to read pgood gpio: %d\n", pwr_good);
			return pwr_good;
		}
		else if (pwr_good)
			break;

		usleep_range(1000, 1500);
	}

	t1 = jiffies;

	if (!pwr_good) {
		dev_err(&rdev->dev, "Power good signal timeout after %u ms\n",
				jiffies_to_msecs(t1 - t0));
		return -ETIME;
	}

	dev_dbg(&rdev->dev, "Power good OK (took %u ms, %u waits)\n",
		jiffies_to_msecs(t1 - t0),
		wait_cnt);

	return ret;
}

static const struct regulator_ops sy7636a_vcom_volt_ops = {
	.get_voltage = get_vcom_voltage_op,
	.enable = enable_regulator_pgood,
	.disable = disable_regulator,
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

static int sy7636a_regulator_init(struct sy7636a *sy7636a)
{
	/* Control VCOM output with VCOM_EN pin */
	regmap_update_bits(sy7636a->regmap, SY7636A_REG_OPERATION_MODE_CRL,
			   SY7636A_OPERATION_MODE_CRL_VCOMCTL,
			   SY7636A_OPERATION_MODE_CRL_VCOMCTL);

	return regmap_write(sy7636a->regmap,
				SY7636A_REG_POWER_ON_DELAY_TIME,
				0x0);
}

static irqreturn_t sy7636a_pgood_irq_handler(int irq, void *_sy7636a)
{
	struct sy7636a *sy7636a = _sy7636a;
	int pwr_good;

	pwr_good = gpiod_get_value_cansleep(sy7636a->pgood_gpio);

	dev_dbg(sy7636a->dev, "Power good line changed to %d\n", pwr_good);

	return IRQ_HANDLED;
}

static int sy7636a_regulator_suspend(struct device *dev)
{
	int ret;
	struct sy7636a *sy7636a = dev_get_drvdata(dev->parent);

	ret = get_vcom_voltage_mv(sy7636a->regmap);
	if (ret > 0)
		sy7636a->vcom = (unsigned int)ret;

	return 0;
}

static int sy7636a_regulator_resume(struct device *dev)
{
	int ret;

	struct sy7636a *sy7636a = dev_get_drvdata(dev->parent);

	if (!sy7636a->vcom || sy7636a->vcom > 5000) {
		dev_warn(dev, "Vcom value invalid, and thus not restored\n");
	}
	else {
		ret = set_vcom_voltage_mv(sy7636a->regmap, sy7636a->vcom);
		if (ret)
			return ret;
	}

	return sy7636a_regulator_init(sy7636a);
}

static int sy7636a_regulator_probe(struct platform_device *pdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct gpio_desc *gdp;
	int ret;

	if (!sy7636a)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, sy7636a);

	gdp = devm_gpiod_get(sy7636a->dev, "epd-pwr-good", GPIOD_IN);
	if (IS_ERR(gdp)) {
		dev_err(sy7636a->dev, "Power good GPIO fault %ld\n", PTR_ERR(gdp));
		return PTR_ERR(gdp);
	}

	sy7636a->pgood_gpio = gdp;
	dev_info(sy7636a->dev,
		"Power good GPIO registered (gpio# %d)\n",
		desc_to_gpio(sy7636a->pgood_gpio));


	sy7636a->pgood_irq = gpiod_to_irq(gdp);
	ret = devm_request_threaded_irq(sy7636a->dev, sy7636a->pgood_irq,
					NULL, sy7636a_pgood_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"epd_pgood", sy7636a);
	if (ret) {
		dev_err(sy7636a->dev, "Unable to request power good irq %d\n",
			sy7636a->pgood_irq);
		/* non fatal */
	}

	ret = sy7636a_regulator_init(sy7636a);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to initialize regulator: %d\n", ret);
		return ret;
	}

	config.dev = &pdev->dev;
	config.dev->of_node = sy7636a->dev->of_node;
	config.driver_data = sy7636a;
	config.regmap = sy7636a->regmap;

	rdev = devm_regulator_register(&pdev->dev, &desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(sy7636a->dev, "Failed to register %s regulator\n",
			pdev->name);
		return PTR_ERR(rdev);
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
