/*
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * Based on the lp87565 driver by Keerthy <j-keerthy@ti.com>
 */

#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <linux/mfd/sy7636a.h>

static const struct regmap_config sy7636a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct mfd_cell sy7636a_cells[] = {
	{ .name = "sy7636a-regulator", },
	{ .name = "sy7636a-temperature", },
};

static const struct of_device_id of_sy7636a_match_table[] = {
	{ .compatible = "silergy,sy7636a", },
	{}
};
MODULE_DEVICE_TABLE(of, of_sy7636a_match_table);

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

	return devm_mfd_add_devices(sy7636a->dev, PLATFORM_DEVID_AUTO,
				    sy7636a_cells, ARRAY_SIZE(sy7636a_cells),
				    NULL, 0, NULL);
}

static const struct i2c_device_id sy7636a_id_table[] = {
	{ "sy7636a", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sy7636a_id_table);

static struct i2c_driver sy7636a_driver = {
	.driver	= {
		.name	= "sy7636a",
		.of_match_table = of_sy7636a_match_table,
	},
	.probe = sy7636a_probe,
	.id_table = sy7636a_id_table,
};
module_i2c_driver(sy7636a_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("Silergy SY7636A Multi-Function Device Driver");
MODULE_LICENSE("GPL v2");
