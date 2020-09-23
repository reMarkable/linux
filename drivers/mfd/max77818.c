/*
 * Maxim MAX77818 MFD Core
 *
 * Copyright (C) 2014 Maxim Integrated Product
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Steinar Bakkemo <steinar.bakkemo@remarkable.com>
 * Author: Shawn Guo  <shawn.guo@linaro.org>
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
 * This driver is based on max77843.c
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/power/max17042_battery.h>
#include <linux/power/max77818_battery_utils.h>

#define I2C_ADDR_PMIC		(0xcc >> 1) /* PMIC (CLOGIC/SAFELDOs) */
#define I2C_ADDR_CHARGER	(0xd2 >> 1) /* Charger */
#define I2C_ADDR_FUEL_GAUGE	(0x6c >> 1) /* Fuel Gauge */

#define REG_PMICID		0x20
#define REG_PMICREV        	0x21

static int max77818_chg_handle_pre_irq(void *irq_drv_data)
{
	struct max77818_dev *max77818_dev = (struct max77818_dev*) irq_drv_data;
	bool restore_state = 0;
	int ret = 0;

	if (!max77818_dev) {
		printk("%s: No driver data, unable to disable FGCC\n", __func__);
		return -EINVAL;
	}

	ret = MAX77818_START_NON_FGCC_OP(
			max77818_dev,
			restore_state,
			"Disabling FGCC before handling charger interrupt");
	if (ret)
		dev_err(max77818_dev->dev,
			"Failed to disable FGCC\n");

	return ret;
}

static int max77818_chg_handle_post_irq(void *irq_drv_data)
{
	struct max77818_dev *max77818_dev = (struct max77818_dev*) irq_drv_data;
	bool restore_state = 1;
	int ret = 0;

	if (!max77818_dev) {
		printk("%s: No driver data, unable to disable FGCC\n", __func__);
		return -EINVAL;
	}

	ret = MAX77818_FINISH_NON_FGCC_OP(
			max77818_dev,
			restore_state,
			"Enabling FGCC after handling charger interrupt");
	if (ret)
		dev_err(max77818_dev->dev,
			"Failed to enable FGCC\n");

	return ret;
}

static const struct regmap_config max77818_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config max77818_regmap_config_fg = {
	.reg_bits = 8,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

/* Declare Interrupt */
static const struct regmap_irq max77818_intsrc_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHGR_INT,	},
	{ .reg_offset = 0, .mask = BIT_FG_INT,		},
	{ .reg_offset = 0, .mask = BIT_SYS_INT, 	},
};

static const struct regmap_irq_chip max77818_intsrc_irq_chip = {
	.name = "max77818 intsrc",
	.status_base = REG_INTSRC,
	.mask_base = REG_INTSRCMASK,
	.num_regs = 1,
	.irqs = max77818_intsrc_irqs,
	.num_irqs = ARRAY_SIZE(max77818_intsrc_irqs),
};

static const struct regmap_irq max77818_sys_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_SYSUVLO_INT,	},
	{ .reg_offset = 0, .mask = BIT_SYSOVLO_INT,	},
	{ .reg_offset = 0, .mask = BIT_TSHDN_INT,	},
	{ .reg_offset = 0, .mask = BIT_TM_INT,		},
};

static const struct regmap_irq_chip max77818_sys_irq_chip = {
	.name = "max77818 system",
	.status_base = REG_SYSINTSRC,
	.mask_base = REG_SYSINTMASK,
	.num_regs = 1,
	.irqs = max77818_sys_irqs,
	.num_irqs = ARRAY_SIZE(max77818_sys_irqs),
};

static const struct regmap_irq max77818_chg_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHG_BYP_I, 	},
	{ .reg_offset = 0, .mask = BIT_CHG_BATP_I, 	},
	{ .reg_offset = 0, .mask = BIT_CHG_BAT_I, 	},
	{ .reg_offset = 0, .mask = BIT_CHG_CHG_I, 	},
	{ .reg_offset = 0, .mask = BIT_CHG_WCIN_I, 	},
	{ .reg_offset = 0, .mask = BIT_CHG_CHGIN_I,	},
	{ .reg_offset = 0, .mask = BIT_CHG_AICL_I, 	},
};

static struct regmap_irq_chip max77818_chg_irq_chip = {
	.name = "max77818 chg",
	.status_base = REG_CHARGER_INT,
	.mask_base = REG_CHARGER_INT_MASK,
	.num_regs = 1,
	.irqs = max77818_chg_irqs,
	.num_irqs = ARRAY_SIZE(max77818_chg_irqs),
	.handle_pre_irq = max77818_chg_handle_pre_irq,
	.handle_post_irq = max77818_chg_handle_post_irq,
};

static struct mfd_cell max77818_devices[] = {
	{
		.name = MAX77818_REGULATOR_NAME,
		.of_compatible = "maxim,"MAX77818_REGULATOR_NAME,
	}, {
		.name = MAX77818_CHARGER_NAME,
		.of_compatible = "maxim,"MAX77818_CHARGER_NAME,
	}, {
		.name = MAX77818_FUELGAUGE_NAME,
		.of_compatible = "maxim,"MAX77818_FUELGAUGE_NAME,
	},
};

static int max77818_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct max77818_dev *me;
	u32 chip_id, chip_rev;
	int ret;

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (!me)
		return -ENOMEM;

	i2c_set_clientdata(client, me);

	mutex_init(&me->lock);
	me->dev = &client->dev;
	me->irq = client->irq;
	me->pmic = client;

	me->regmap_pmic = devm_regmap_init_i2c(client, &max77818_regmap_config);
	if (IS_ERR(me->regmap_pmic)) {
		ret = PTR_ERR(me->regmap_pmic);
		dev_err(me->dev, "failed to initialize PMIC regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(me->regmap_pmic, REG_PMICID, &chip_id);
	if (ret < 0) {
		dev_err(me->dev, "failed to read chip id: %d\n", ret);
		return ret;
	} else {
		regmap_read(me->regmap_pmic, REG_PMICREV, &chip_rev);
		dev_info(me->dev, "device ID: 0x%x, REV: 0x%x\n",
			 chip_id, chip_rev);
	}

	me->chg = i2c_new_dummy(client->adapter, I2C_ADDR_CHARGER);
	if (!me->chg) {
		dev_err(me->dev, "failed to allocate I2C device for CHG\n");
		return ret;
	}
	i2c_set_clientdata(me->chg, me);

	me->fg = i2c_new_dummy(client->adapter, I2C_ADDR_FUEL_GAUGE);
	if (!me->fg) {
		dev_err(me->dev, "failed to allocate I2C device for FG\n");
		goto unreg_chg;
	}
	i2c_set_clientdata(me->fg, me);

	me->regmap_chg = devm_regmap_init_i2c(me->chg, &max77818_regmap_config);
	if (IS_ERR_OR_NULL(me->regmap_chg)) {
		ret = PTR_ERR(me->regmap_chg);
		dev_warn(me->dev, "failed to initialize CHG regmap: %d\n", ret);
		goto unreg_fg;
	}

	me->regmap_fg= devm_regmap_init_i2c(me->fg, &max77818_regmap_config_fg);
	if (IS_ERR_OR_NULL(me->regmap_fg)) {
		ret = PTR_ERR(me->regmap_fg);
		dev_err(me->dev, "failed to initialize FG regmap: %d\n", ret);
		goto unreg_fg;
	}

	/* Disable all interrupt source */
	regmap_write(me->regmap_pmic, REG_INTSRCMASK, 0xff);

	/* Register overall interrupt source (sys, fg, chg) */
	ret = regmap_add_irq_chip(me->regmap_pmic, me->irq,
				  IRQF_ONESHOT | IRQF_SHARED, 0,
				  &max77818_intsrc_irq_chip, &me->irqc_intsrc);
	if (ret) {
		dev_err(me->dev, "failed to add intsrc irq chip: %d\n", ret);
		goto unreg_fg;
	}

	/* Register system chip irq */
	ret = regmap_add_irq_chip(me->regmap_pmic, me->irq,
				  IRQF_ONESHOT | IRQF_SHARED, 0,
				  &max77818_sys_irq_chip, &me->irqc_sys);
	if (ret) {
		dev_err(me->dev, "failed to add system irq chip: %d\n", ret);
		goto del_irqc_intsrc;
	}

	/* Register charger chip irq */
	max77818_chg_irq_chip.irq_drv_data = me;
	ret = MAX77818_DO_NON_FGCC_OP(
			me,
			regmap_add_irq_chip(me->regmap_chg,
					    me->irq,
					    IRQF_ONESHOT | IRQF_SHARED,
					    0,
					    &max77818_chg_irq_chip,
					    &me->irqc_chg),
			"adding charger chip irq\n");

	if (ret) {
		dev_warn(me->dev, "failed to add chg irq chip: %d\n", ret);
		goto del_irqc_sys;
	}

	pm_runtime_set_active(me->dev);

	ret = mfd_add_devices(me->dev, -1, max77818_devices,
			      ARRAY_SIZE(max77818_devices), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(me->dev, "failed to add mfd devices: %d\n", ret);
		goto del_irqc_chg;
	}

	device_init_wakeup(me->dev, true);

	return 0;

del_irqc_chg:
	regmap_del_irq_chip(me->irq, me->irqc_chg);
del_irqc_sys:
	regmap_del_irq_chip(me->irq, me->irqc_sys);
del_irqc_intsrc:
	regmap_del_irq_chip(me->irq, me->irqc_intsrc);
unreg_fg:
	i2c_unregister_device(me->fg);
unreg_chg:
	i2c_unregister_device(me->chg);

	return ret;
}

static int max77818_i2c_remove(struct i2c_client *client)
{
	struct max77818_dev *me = i2c_get_clientdata(client);

	mfd_remove_devices(me->dev);

	regmap_del_irq_chip(me->irq, me->irqc_chg);
	regmap_del_irq_chip(me->irq, me->irqc_sys);
	regmap_del_irq_chip(me->irq, me->irqc_intsrc);

	i2c_unregister_device(me->fg);

	if (me->chg) {
		i2c_unregister_device(me->chg);
	}

	return 0;
}

static int max77818_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max77818_dev *me = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(me->irq);
		disable_irq(me->irq);
	}

	return 0;
}

static int max77818_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max77818_dev *me = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(me->irq);
		enable_irq(me->irq);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(max77818_pm, max77818_suspend, max77818_resume);

static struct of_device_id max77818_of_id[] = {
	{ .compatible = "maxim,max77818" },
	{ },
};
MODULE_DEVICE_TABLE(of, max77818_of_id);

static const struct i2c_device_id max77818_i2c_id[] = {
	{ "max77818" },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max77818_i2c_id);

static struct i2c_driver max77818_i2c_driver = {
	.driver = {
		   .name = "max77818",
		   .pm = &max77818_pm,
		   .of_match_table = max77818_of_id,
	},
	.probe = max77818_i2c_probe,
	.remove = max77818_i2c_remove,
	.id_table = max77818_i2c_id,
};
module_i2c_driver(max77818_i2c_driver);

MODULE_DESCRIPTION("MAX77818 MFD Driver");
MODULE_LICENSE("GPL v2");
