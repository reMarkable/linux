/*
 * @file bd7181x.c  --  RoHM BD7181X/BD71817 mfd driver
 * 
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author: Tony Luo <luofc@embedinfo.com>
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/bd7181x.h>

/** @brief bd7181x irq resource */
static struct resource rtc_resources[] = {
	{
		.start  = BD7181X_IRQ_ALARM_12,
		.end    = BD7181X_IRQ_ALARM_12,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource power_resources[] = {
	// irq# 0
	{
		.start	= BD7181X_IRQ_DCIN_03,
		.end	= BD7181X_IRQ_DCIN_03,
		.flags	= IORESOURCE_IRQ,
	},
	// irq# 1
	{
		.start	= BD7181X_IRQ_BAT_MON_08,
		.end	= BD7181X_IRQ_BAT_MON_08,
		.flags	= IORESOURCE_IRQ,
	},
	// irq# 2
	{
		.start	= BD7181X_IRQ_TEMPERATURE_11,
		.end	= BD7181X_IRQ_TEMPERATURE_11,
		.flags	= IORESOURCE_IRQ,
	}
};

/** @brief bd7181x multi function cells */
static struct mfd_cell bd7181x_mfd_cells[] = {
	{
		.name = "bd7181x-pmic",
	},
	{
		.name = "bd7181x-power",
		.num_resources = ARRAY_SIZE(power_resources),
		.resources = &power_resources[0],
	},
	{
		.name = "bd7181x-gpo",
	},
	{
		.name = "bd7181x-rtc",
		.num_resources = ARRAY_SIZE(rtc_resources),
		.resources = &rtc_resources[0],
	},
};

/** @brief bd7181x irqs */
static const struct regmap_irq bd7181x_irqs[] = {
	[BD7181X_IRQ_BUCK_01] = {
		.mask = BD7181X_INT_EN_01_BUCKAST_MASK,
		.reg_offset = 1,
	},
	[BD7181X_IRQ_DCIN_02] = {
		.mask = BD7181X_INT_EN_02_DCINAST_MASK,
		.reg_offset = 2,
	},
	[BD7181X_IRQ_DCIN_03] = {
		.mask = BD7181X_INT_EN_03_DCINAST_MASK,
		.reg_offset = 3,
	},
	[BD7181X_IRQ_VSYS_04] = {
		.mask = BD7181X_INT_EN_04_VSYSAST_MASK,
		.reg_offset = 4,
	},
	[BD7181X_IRQ_CHARGE_05] = {
		.mask = BD7181X_INT_EN_05_CHGAST_MASK,
		.reg_offset = 5,
	},
	[BD7181X_IRQ_BAT_06] = {
		.mask = BD7181X_INT_EN_06_BATAST_MASK,
		.reg_offset = 6,
	},
	[BD7181X_IRQ_BAT_MON_07] = {
		.mask = BD7181X_INT_EN_07_BMONAST_MASK,
		.reg_offset = 7,
	},
	[BD7181X_IRQ_BAT_MON_08] = {
		.mask = BD7181X_INT_EN_08_BMONAST_MASK,
		.reg_offset = 8,
	},
	[BD7181X_IRQ_BAT_MON_09] = {
		.mask = BD7181X_INT_EN_09_BMONAST_MASK,
		.reg_offset = 9,
	},
	[BD7181X_IRQ_BAT_MON_10] = {
		.mask = BD7181X_INT_EN_10_BMONAST_MASK,
		.reg_offset = 10,
	},
	[BD7181X_IRQ_TEMPERATURE_11] = {
		.mask = BD7181X_INT_EN_11_TMPAST_MASK,
		.reg_offset = 11,
	},
	[BD7181X_IRQ_ALARM_12] = {
		.mask = BD7181X_INT_EN_12_ALMAST_MASK,
		.reg_offset = 12,
	},
};

/** @brief bd7181x irq chip definition */
static struct regmap_irq_chip bd7181x_irq_chip = {
	.name = "bd7181x",
	.irqs = bd7181x_irqs,
	.num_irqs = ARRAY_SIZE(bd7181x_irqs),
	.num_regs = 13,
	.irq_reg_stride = 1,
	.status_base = BD7181X_REG_INT_STAT,
	.mask_base = BD7181X_REG_INT_EN_01 - 1,
	.mask_invert = true,
	// .ack_base = BD7181X_REG_INT_STAT_00,
};

/** @brief bd7181x irq initialize 
 *  @param bd7181x bd7181x device to init
 *  @param bdinfo platform init data
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd7181x_irq_init(struct bd7181x *bd7181x, struct bd7181x_board* bdinfo) {
	int irq;
	int ret = 0;

	if (!bdinfo) {
		dev_warn(bd7181x->dev, "No interrupt support, no pdata\n");
		return -EINVAL;
	}
	
	irq = gpio_to_irq(bdinfo->gpio_intr);

	bd7181x->chip_irq = irq;
	printk("bd7181x->chip_irq=%d \n", bd7181x->chip_irq);
	ret = regmap_add_irq_chip(bd7181x->regmap, bd7181x->chip_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_FALLING, bdinfo->irq_base,
		&bd7181x_irq_chip, &bd7181x->irq_data);
	if (ret < 0) {
		dev_warn(bd7181x->dev, "Failed to add irq_chip %d\n", ret);
	}
	return ret;
}

/** @brief bd7181x irq initialize 
 *  @param bd7181x bd7181x device to init
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd7181x_irq_exit(struct bd7181x *bd7181x)
{
	if (bd7181x->chip_irq > 0)
		regmap_del_irq_chip(bd7181x->chip_irq, bd7181x->irq_data);
	return 0;
}

/** @brief check whether volatile register 
 *  @param dev kernel device pointer
 *  @param reg register index
 */
static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	// struct bd7181x *bd7181x = dev_get_drvdata(dev);

	/*
	 * Caching all regulator registers.
	 */
	return true;
}

/** @brief regmap configures */
static const struct regmap_config bd7181x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = is_volatile_reg,
	.max_register = BD7181X_MAX_REGISTER - 1,
	.cache_type = REGCACHE_RBTREE,
};

#ifdef CONFIG_OF
static struct of_device_id bd7181x_of_match[] = {
	{ .compatible = "rohm,bd71815", .data = (void *)0},
	{ .compatible = "rohm,bd71817", .data = (void *)1},
	{ },
};
MODULE_DEVICE_TABLE(of, bd7181x_of_match);


/** @brief parse device tree data of bd7181x
 *  @param client client object provided by system
 *  @param chip_id return chip id back to caller
 *  @return board initialize data
 */
static struct bd7181x_board *bd7181x_parse_dt(struct i2c_client *client,
						int *chip_id)
{
	struct device_node *np = client->dev.of_node;
	struct bd7181x_board *board_info;
	unsigned int prop;
	const struct of_device_id *match;
	int r = 0;

	match = of_match_device(bd7181x_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	*chip_id  = (int)match->data;

	board_info = devm_kzalloc(&client->dev, sizeof(*board_info),
			GFP_KERNEL);
	if (!board_info) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_info->gpio_intr = of_get_named_gpio(np, "gpio_intr", 0);
	if (!gpio_is_valid(board_info->gpio_intr)) {
		dev_err(&client->dev, "no pmic intr pin available\n");
		goto err_intr;
	}

	r = of_property_read_u32(np, "irq_base", &prop);
	if (!r) {
		board_info->irq_base = prop;
	} else {
		board_info->irq_base = -1;
	}

	return board_info;

err_intr:
	devm_kfree(&client->dev, board_info);
	return NULL;
}
#else
static inline
struct bd7181x_board *bd7181x_parse_dt(struct i2c_client *client,
					 int *chip_id)
{
	return NULL;
}
#endif

/** @brief probe bd7181x device
 *  @param i2c client object provided by system
 *  @param id chip id
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd7181x_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct bd7181x *bd7181x;
	struct bd7181x_board *pmic_plat_data;
	struct bd7181x_board *of_pmic_plat_data = NULL;
	int chip_id = id->driver_data;
	int ret = 0;

	pmic_plat_data = dev_get_platdata(&i2c->dev);

	if (!pmic_plat_data && i2c->dev.of_node) {
		pmic_plat_data = bd7181x_parse_dt(i2c, &chip_id);
		of_pmic_plat_data = pmic_plat_data;
	}

	if (!pmic_plat_data)
		return -EINVAL;

	bd7181x = kzalloc(sizeof(struct bd7181x), GFP_KERNEL);
	if (bd7181x == NULL)
		return -ENOMEM;

	bd7181x->of_plat_data = of_pmic_plat_data;
	i2c_set_clientdata(i2c, bd7181x);
	bd7181x->dev = &i2c->dev;
	bd7181x->i2c_client = i2c;
	bd7181x->id = chip_id;
	mutex_init(&bd7181x->io_mutex);

	bd7181x->regmap = devm_regmap_init_i2c(i2c, &bd7181x_regmap_config);
	if (IS_ERR(bd7181x->regmap)) {
		ret = PTR_ERR(bd7181x->regmap);
		dev_err(&i2c->dev, "regmap initialization failed: %d\n", ret);
		return ret;
	}

	ret = bd7181x_reg_read(bd7181x, BD7181X_REG_DEVICE);
	if(ret < 0) {
		dev_err(bd7181x->dev, "%s(): Read BD7181X_REG_DEVICE failed!\n", __func__);
		goto err;
	}
	dev_info(bd7181x->dev, "BD7181x: Device ID=0x%X\n", ret);

	bd7181x_irq_init(bd7181x, of_pmic_plat_data);

	ret = mfd_add_devices(bd7181x->dev, -1,
			      bd7181x_mfd_cells, ARRAY_SIZE(bd7181x_mfd_cells),
			      NULL, 0,
			      regmap_irq_get_domain(bd7181x->irq_data));
	if (ret < 0)
		goto err;

	return ret;

err:
	mfd_remove_devices(bd7181x->dev);
	kfree(bd7181x);
	return ret;
}

/** @brief remove bd7181x device
 *  @param i2c client object provided by system
 *  @return 0
 */
static int bd7181x_i2c_remove(struct i2c_client *i2c)
{
	struct bd7181x *bd7181x = i2c_get_clientdata(i2c);

	bd7181x_irq_exit(bd7181x);
	mfd_remove_devices(bd7181x->dev);
	kfree(bd7181x);

	return 0;
}

static const struct i2c_device_id bd7181x_i2c_id[] = {
	{ "bd71815", 0 },
	{ "bd71817", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd7181x_i2c_id);


static struct i2c_driver bd7181x_i2c_driver = {
	.driver = {
		.name = "bd7181x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bd7181x_of_match),
	},
	.probe = bd7181x_i2c_probe,
	.remove = bd7181x_i2c_remove,
	.id_table = bd7181x_i2c_id,
};

static int __init bd7181x_i2c_init(void)
{
	return i2c_add_driver(&bd7181x_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(bd7181x_i2c_init);

static void __exit bd7181x_i2c_exit(void)
{
	i2c_del_driver(&bd7181x_i2c_driver);
}
module_exit(bd7181x_i2c_exit);

MODULE_AUTHOR("Tony Luo <luofc@embest-tech.com>");
MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("BD71815/BD71817 chip multi-function driver");
MODULE_LICENSE("GPL");
