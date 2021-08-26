/*
 * MFD driver for SY7636A chip
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
 * Based on the lp87565 driver by Keerthy <j-keerthy@ti.com>
 */

#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/pinctrl/consumer.h>

#include <linux/mfd/sy7636a.h>

static const struct regmap_config sy7636a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct mfd_cell sy7636a_cells[] = {
	{ .name = "sy7636a-regulator", },
	{ .name = "sy7636a-temperature", },
	{ .name = "sy7636a-thermal", },
};

static const struct of_device_id of_sy7636a_match_table[] = {
	{ .compatible = "silergy,sy7636a", },
	{}
};
MODULE_DEVICE_TABLE(of, of_sy7636a_match_table);

static const char *states[] = {
	"no fault event",
	"UVP at VP rail",
	"UVP at VN rail",
	"UVP at VPOS rail",
	"UVP at VNEG rail",
	"UVP at VDDH rail",
	"UVP at VEE rail",
	"SCP at VP rail",
	"SCP at VN rail",
	"SCP at VPOS rail",
	"SCP at VNEG rail",
	"SCP at VDDH rail",
	"SCP at VEE rail",
	"SCP at V COM rail",
	"UVLO",
	"Thermal shutdown",
};

int get_vcom_voltage_mv(struct regmap *regmap)
{
	int ret;
	unsigned int val, val_h;

	ret = regmap_read(regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, &val);
	if (ret)
		return ret;

	ret = regmap_read(regmap, SY7636A_REG_VCOM_ADJUST_CTRL_H, &val_h);
	if (ret)
		return ret;

	val |= ((val_h & 0x80) << 1);

	return (val & 0x1FF) * 10;
}

int set_vcom_voltage_mv(struct regmap *regmap, unsigned int vcom)
{
	int ret;
	unsigned int val;

	if (vcom < 0 || vcom > 5000)
		return -EINVAL;

	val = (unsigned int)(vcom / 10) & 0x1ff;

	ret = regmap_write(regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, (val & 0xFF));
	if (ret)
		return ret;

	/* We touch only bit 7, the rest is reserved */
	return regmap_update_bits(regmap, SY7636A_REG_VCOM_ADJUST_CTRL_H, 0x80, (val >> 1)) ;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int ret;
	unsigned int val;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = regmap_read(sy7636a->regmap, SY7636A_REG_FAULT_FLAG, &val);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to read from device\n");
		return ret;
	}

	val = val >> 1;

	if (val >= ARRAY_SIZE(states)) {
		dev_err(sy7636a->dev, "Unexpected value read from device: %u\n", val);
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", states[val]);
}
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);

static ssize_t powergood_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int ret;
	unsigned int val;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = regmap_read(sy7636a->regmap, SY7636A_REG_FAULT_FLAG, &val);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to read from device\n");
		return ret;
	}

	val &= 0x01;

	return snprintf(buf, PAGE_SIZE, "%s\n", val ? "ON" : "OFF");
}
static DEVICE_ATTR(power_good, S_IRUGO, powergood_show, NULL);

static ssize_t vcom_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int ret;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = get_vcom_voltage_mv(sy7636a->regmap);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", -ret);
}

static ssize_t vcom_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int vcom;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 0, &vcom);
	if (ret)
		return ret;

	if (vcom > 0 || vcom < -5000)
		return -EINVAL;

	ret = set_vcom_voltage_mv(sy7636a->regmap, (unsigned int)(-vcom));
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR(vcom, S_IRUGO | S_IWUSR, vcom_show, vcom_store);

static struct attribute *sy7636a_sysfs_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_power_good.attr,
	&dev_attr_vcom.attr,
	NULL,
};

static const struct attribute_group sy7636a_sysfs_attr_group = {
	.attrs = sy7636a_sysfs_attrs,
};

static int sy7636a_probe(struct i2c_client *client,
			 const struct i2c_device_id *ids)
{
	struct sy7636a *sy7636a;
	int ret;

	sy7636a = devm_kzalloc(&client->dev, sizeof(struct sy7636a), GFP_KERNEL);
	if (sy7636a == NULL)
		return -ENOMEM;

	sy7636a->dev = &client->dev;

	sy7636a->regmap = devm_regmap_init_i2c(client, &sy7636a_regmap_config);
	if (IS_ERR(sy7636a->regmap)) {
		ret = PTR_ERR(sy7636a->regmap);
		dev_err(sy7636a->dev,
			"Failed to initialize register map: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, sy7636a);

	ret = sysfs_create_group(&client->dev.kobj, &sy7636a_sysfs_attr_group);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to create sysfs attributes\n");
		return ret;
	}

	ret = devm_mfd_add_devices(sy7636a->dev, PLATFORM_DEVID_AUTO,
					sy7636a_cells, ARRAY_SIZE(sy7636a_cells),
					NULL, 0, NULL);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to add mfd devices\n");
		sysfs_remove_group(&client->dev.kobj, &sy7636a_sysfs_attr_group);
		return ret;
	}

	return 0;
}

static const struct i2c_device_id sy7636a_id_table[] = {
	{ "sy7636a", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sy7636a_id_table);

#ifdef CONFIG_PM
static int sy7636a_suspend(struct device *dev)
{
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int sy7636a_resume(struct device *dev)
{
	pinctrl_pm_select_default_state(dev);

	return 0;
}

static const struct dev_pm_ops sy7636a_pm_ops = {
	.suspend = sy7636a_suspend,
	.resume = sy7636a_resume,
};
#endif

static struct i2c_driver sy7636a_driver = {
	.driver	= {
		.name	= "sy7636a",
		.of_match_table = of_sy7636a_match_table,
#ifdef CONFIG_PM
		.pm = &sy7636a_pm_ops,
#endif
	},
	.probe = sy7636a_probe,
	.id_table = sy7636a_id_table,
};
module_i2c_driver(sy7636a_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("Silergy SY7636A Multi-Function Device Driver");
MODULE_LICENSE("GPL v2");
