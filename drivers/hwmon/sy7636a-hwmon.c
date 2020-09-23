/*
 * Functions to access SY3686A power management chip temperature
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#include <linux/mfd/sy7636a.h>

struct sy7636a_data {
	struct sy7636a *sy7636a;
	struct device *hwmon_dev;
};

static ssize_t show_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int reg_val;
	signed char temp;
	int ret;
	struct sy7636a_data *data = dev_get_drvdata(dev);

	ret = regmap_read(data->sy7636a->regmap,
			SY7636A_REG_TERMISTOR_READOUT, &reg_val);
	if (ret)
		return ret;

	temp = *((signed char*)&reg_val);

	return snprintf(buf, PAGE_SIZE, "%d\n", temp);
}

static SENSOR_DEVICE_ATTR(temp0, S_IRUGO, show_temp, NULL, 0);

static struct attribute *sy7636a_attrs[] = {
	&sensor_dev_attr_temp0.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(sy7636a);

static int sy7636a_sensor_probe(struct platform_device *pdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);
	struct sy7636a_data *data;
	int err;

	if (!sy7636a)
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(struct sy7636a_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}

	data->sy7636a = sy7636a;
	data->hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
			"sy7636a_temperature", data, sy7636a_groups);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		dev_err(&pdev->dev, "Unable to register hwmon device, returned %d", err);
		return err;
	}

	return 0;
}

static const struct platform_device_id sy7636a_sensor_id[] = {
	{ "sy7636a-temperature", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, sy7636a_sensor_id);

static struct platform_driver sy7636a_sensor_driver = {
	.probe = sy7636a_sensor_probe,
	.id_table = sy7636a_sensor_id,
	.driver = {
		.name = "sy7636a-temperature",
	},
};
module_platform_driver(sy7636a_sensor_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("Silergy SY7636A Sensor Driver");
MODULE_LICENSE("GPL v2");
