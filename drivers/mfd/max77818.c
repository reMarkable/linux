/*
 * Maxim MAX77818 MFD Core
 *
 * Copyright (C) 2014 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77818/max77818.h>

#define DRIVER_DESC    "MAX77818 MFD Driver"
#define DRIVER_NAME    MAX77818_NAME
#define DRIVER_VERSION "1.0"
#define DRIVER_AUTHOR  "TaiEup Kim <clark.kim@maximintegrated.com>"

#define I2C_ADDR_PMIC		(0xCC >> 1)	/* PMIC (CLOGIC/SAFELDOs) */
#define I2C_ADDR_CHARGER	(0xD2 >> 1) /* Charger */
#define I2C_ADDR_FUEL_GAUGE (0x6C >> 1) /* Fuel Gauge */


#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)


static const struct regmap_config max77818_regmap_config = {
    .reg_bits   = 8,
    .val_bits   = 8,
    .cache_type = REGCACHE_NONE,
};

static const struct regmap_config max77818_regmap_config_fuelgauge = {
    .reg_bits   = 8,
    .val_bits   = 16,
    .cache_type = REGCACHE_NONE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};


/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max77818_read (struct regmap *regmap, u8 addr, u8 *val)
{
    unsigned int buf = 0;
    int rc = regmap_read(regmap, (unsigned int)addr, &buf);

    if (likely(!IS_ERR_VALUE(rc))) {
        *val = (u8)buf;
    }
    return rc;
}
EXPORT_SYMBOL(max77818_read);

int max77818_write (struct regmap *regmap, u8 addr, u8 val)
{
    unsigned int buf = (unsigned int)val;
    return regmap_write(regmap, (unsigned int)addr, buf);
}
EXPORT_SYMBOL(max77818_write);

int max77818_fg_read (struct regmap *regmap, u8 addr, u16 *val)
{
    unsigned int buf = 0;
    int rc = regmap_read(regmap, (unsigned int)addr, &buf);

    if (likely(!IS_ERR_VALUE(rc))) {
        *val = buf;
    }
    return rc;
}
EXPORT_SYMBOL(max77818_fg_read);

int max77818_fg_write (struct regmap *regmap, u8 addr, u16 val)
{
    unsigned int buf = (unsigned int)val;
    return regmap_write(regmap, (unsigned int)addr, buf);
}
EXPORT_SYMBOL(max77818_fg_write);

int max77818_bulk_read (struct regmap *regmap, u8 addr, u8 *dst, u16 len)
{
    return regmap_bulk_read(regmap, (unsigned int)addr, dst, (size_t)len);
}
EXPORT_SYMBOL(max77818_bulk_read);

int max77818_bulk_write (struct regmap *regmap, u8 addr, const u8 *src, u16 len)
{
    return regmap_bulk_write(regmap, (unsigned int)addr, src, (size_t)len);
}
EXPORT_SYMBOL(max77818_bulk_write);

/*******************************************************************************
 *  device
 ******************************************************************************/
static int max77818_add_devices (struct max77818_dev *me,
    struct mfd_cell *cells, int n_devs)
{
    struct device *dev = me->dev;
    int rc;

	printk("[---- SBA ----] max77818_add_devices Enter:\n");

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
    rc = mfd_add_devices(dev, -1, cells, n_devs, NULL, 0);
#else /* LINUX_VERSION_CODE ... */
    rc = mfd_add_devices(dev, -1, cells, n_devs, NULL, 0, NULL);
#endif /* LINUX_VERSION_CODE ... */

    return rc;
}

/*******************************************************************************
 *** MAX77818 PMIC
 ******************************************************************************/

/* Register map */
/* PMIC */
#define REG_PMICID			0x20
#define REG_PMICREV        	0x21


/* Declare Interrupt */
static const struct regmap_irq max77818_intsrc_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHGR_INT,	},	// CHGR_INT
	{ .reg_offset = 0, .mask = BIT_FG_INT,		},	// FG_INT	
	{ .reg_offset = 0, .mask = BIT_SYS_INT, 	},	// SYS_INT
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
	{ .reg_offset = 0, .mask = BIT_SYSUVLO_INT, },	// SYSUVLO_INT
	{ .reg_offset = 0, .mask = BIT_SYSOVLO_INT, },	// SYSOVLO_INT
	{ .reg_offset = 0, .mask = BIT_TSHDN_INT, 	},	// TSHDN_INT
	{ .reg_offset = 0, .mask = BIT_TM_INT,		},	// TM_INT	
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
	{ .reg_offset = 0, .mask = BIT_CHG_BYP_I, 	},	// BYP_I
	{ .reg_offset = 0, .mask = BIT_CHG_BATP_I, 	},	// BATP_I
	{ .reg_offset = 0, .mask = BIT_CHG_BAT_I, 	},	// BAT_I
	{ .reg_offset = 0, .mask = BIT_CHG_CHG_I, 	},	// CHG_I
	{ .reg_offset = 0, .mask = BIT_CHG_WCIN_I, 	},	// WCIN_I
	{ .reg_offset = 0, .mask = BIT_CHG_CHGIN_I,	},	// CHGIN_I
	{ .reg_offset = 0, .mask = BIT_CHG_AICL_I, 	},	// AICL_I
};

static const struct regmap_irq_chip max77818_chg_irq_chip = {
	.name = "max77818 chg",
	.status_base = REG_CHARGER_INT,
	.mask_base = REG_CHARGER_INT_MASK,
	.num_regs = 1,
	.irqs = max77818_chg_irqs,
	.num_irqs = ARRAY_SIZE(max77818_chg_irqs),
};

int max77818_map_irq(struct max77818_dev *max77818, int irq)
{
	return regmap_irq_get_virq(max77818->irqc_intsrc, irq);
}
EXPORT_SYMBOL_GPL(max77818_map_irq);

int max77818_map_sys_irq(struct max77818_dev *max77818, int irq)
{
	return regmap_irq_get_virq(max77818->irqc_sys, irq);
}
EXPORT_SYMBOL_GPL(max77818_map_sys_irq);

int max77818_map_chg_irq(struct max77818_dev *max77818, int irq)
{
	return regmap_irq_get_virq(max77818->irqc_chg, irq);
}
EXPORT_SYMBOL_GPL(max77818_map_chg_irq);


static int max77818_pmic_irq_int(struct max77818_dev *me)
{
    struct device *dev = me->dev;
    struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;

	printk("[---- SBA ----] max77818_pmic_irq_int Enter:\n");

	/* disable all interrupt source */
	max77818_write(me->regmap_pmic, REG_INTSRCMASK, 0xFF);

	irq_set_status_flags(me->irq, IRQ_NOAUTOEN);

	/* interrupt source */
	rc = regmap_add_irq_chip(me->regmap_pmic, me->irq,
				  IRQF_ONESHOT | IRQF_SHARED, -1,
				  &max77818_intsrc_irq_chip,
				  &me->irqc_intsrc);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add insrc irq chip: %d\n", rc);
		goto out;
	}

	/* system interrupt */
	rc = regmap_add_irq_chip(me->regmap_pmic, me->irq,
				  IRQF_ONESHOT | IRQF_SHARED, -1,
				  &max77818_sys_irq_chip,
				  &me->irqc_sys);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add system irq chip: %d\n", rc);
		goto err_irqc_sys;
	}

	/* charger interrupt */
	rc = regmap_add_irq_chip(me->regmap_chg, me->irq,
				  IRQF_ONESHOT | IRQF_SHARED, -1,
				  &max77818_chg_irq_chip,
				  &me->irqc_chg);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add chg irq chip: %d\n", rc);
		goto err_irqc_chg;
	}

	pr_err("<%s> IRQ initialize done\n", client->name);
	return 0;

err_irqc_chg:
	regmap_del_irq_chip(me->irq, me->irqc_sys);
err_irqc_sys:
	regmap_del_irq_chip(me->irq, me->irqc_intsrc);
out:
	return rc;

}

static void *max77818_pmic_get_platdata (struct max77818_dev *pmic)
{
#ifdef CONFIG_OF
    struct device *dev = pmic->dev;
    struct device_node *np = dev->of_node;
    struct i2c_client *client = to_i2c_client(dev);
    struct max77818_pmic_platform_data *pdata;
    int rc;

	printk("[---- SBA ----] max77818_pmic_get_platdata Enter (CONFIG_OF):\n");

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (unlikely(!pdata)) {
       	pr_err("<%s> out of memory (%uB requested)\n", client->name,
            (unsigned int) sizeof(*pdata));
        pdata = ERR_PTR(-ENOMEM);
        goto out;
    }

    pmic->irq_gpio = of_get_named_gpio(np, "max77818,int-gpio", 0);

    if (pmic->irq_gpio < 0) {
        pdata->irq = irq_of_parse_and_map(np, 0);
    } else {
        unsigned gpio = (unsigned)pmic->irq_gpio;

        rc = gpio_request(gpio, DRIVER_NAME"-irq");
        if (unlikely(IS_ERR_VALUE(rc))) {
            pr_err("<%s> failed to request gpio %u [%d]\n", client->name, gpio,
                rc);
            pmic->irq_gpio = -1;
            pdata = ERR_PTR(rc);
            goto out;
        }

        gpio_direction_input(gpio);
        pr_info("<%s> INTGPIO %u assigned\n", client->name, gpio);

        /* override pdata irq */
        pdata->irq = gpio_to_irq(gpio);
    }

    pr_info("<%s> property:INTGPIO %d\n", client->name, pmic->irq_gpio);
    pr_info("<%s> property:IRQ     %d\n", client->name, pdata->irq);

out:
    return pdata;
#else /* CONFIG_OF */
	printk("[---- SBA ----] max77818_pmic_get_platdata Enter (CONFIG_OF NOT DEFINED):\n");

    return dev_get_platdata(pmic->dev) ?
        dev_get_platdata(pmic->dev) : ERR_PTR(-EINVAL);
#endif /* CONFIG_OF */
}

static struct mfd_cell max77818_devices[] = {
    { .name = MAX77818_REGULATOR_NAME,		},
	{ .name = MAX77818_CHARGER_NAME,		},
	{ .name = MAX77818_FUELGAUGE_NAME,		},
};

static int max77818_pmic_setup (struct max77818_dev *me)
{
    struct device *dev = me->dev;
    struct i2c_client *client = to_i2c_client(dev);
    struct max77818_pmic_platform_data *pdata;
    int rc = 0;
    u8 chip_id, chip_rev, val;

	printk("[---- SBA ----] max77818_pmic_setup Enter:\n");

    me->pdata = max77818_pmic_get_platdata(me);
    if (unlikely(IS_ERR(me->pdata))) {
        rc = PTR_ERR(me->pdata);
        me->pdata = NULL;
        pr_err("<%s> platform data is missing [%d]\n", client->name, rc);
        goto out;
    }

	// IRQ init //
	pdata = me->pdata;
	me->irq = pdata->irq;
	rc = max77818_pmic_irq_int(me);
	if (rc != 0) {
		dev_err(&client->dev, "failed to initialize irq: %d\n", rc);
		goto err_irq_init;
	}

	enable_irq(me->irq);

    rc = max77818_add_devices(me, max77818_devices,
            ARRAY_SIZE(max77818_devices));
    if (unlikely(IS_ERR_VALUE(rc))) {
        pr_err("<%s> failed to add sub-devices [%d]\n", client->name, rc);
        goto err_add_devices;
    }

    /* set device able to wake up system */
    device_init_wakeup(dev, true);
    if (likely(me->irq > 0)) {
        enable_irq_wake((unsigned int)me->irq);
    }

    pr_info("<%s> driver core "DRIVER_VERSION" installed\n", client->name);

    chip_id = 0;
    chip_rev = 0;

    max77818_read(me->regmap_pmic, REG_PMICID,  &chip_id );
    max77818_read(me->regmap_pmic, REG_PMICREV, &chip_rev);

    pr_info("<%s> CHIP ID %Xh REV %Xh\n", client->name, chip_id, chip_rev);

	/* clear IRQ */
	max77818_read(me->regmap_pmic, REG_INTSRC, &val);
	pr_info("<%s> intsrc %Xh\n", client->name, val);

	max77818_read(me->regmap_pmic, REG_INTSRCMASK, &val);
	pr_info("<%s> intsrc_mask %Xh\n", client->name, val);

	return 0;

err_add_devices:
	regmap_del_irq_chip(me->irq, me->irqc_intsrc);
err_irq_init:
out:
	return rc;
}

/*******************************************************************************
 *** MAX77818 MFD Core
 ******************************************************************************/

static __always_inline void max77818_destroy (struct max77818_dev *me)
{
    struct device *dev = me->dev;

	printk("[---- SBA ----] max77818_destroy Enter:\n");

	mfd_remove_devices(me->dev);

    if (likely(me->irq > 0)) {
        regmap_del_irq_chip(me->irq, me->irqc_intsrc);
    }

    if (likely(me->irq_gpio >= 0)) {
        gpio_free((unsigned)me->irq_gpio);
    }

    if (likely(me->regmap_pmic)) {
        regmap_exit(me->regmap_pmic);
    }

	if (likely(me->regmap_chg)) {
		regmap_exit(me->regmap_chg);
	}

	if (likely(me->regmap_fuel)) {
		regmap_exit(me->regmap_fuel);
	}

#ifdef CONFIG_OF
    if (likely(me->pdata)) {
        devm_kfree(dev, me->pdata);
    }
#endif /* CONFIG_OF */

    mutex_destroy(&me->lock);
    devm_kfree(dev, me);
}

static int max77818_i2c_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct max77818_dev *me;
    int rc;

    printk("[---- SBA ----] max77818_i2c_probe Enter:\n");

    pr_info("<%s> attached\n", client->name);

    me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
    if (unlikely(!me)) {
        pr_err("<%s> out of memory (%uB requested)\n", client->name,
            (unsigned int) sizeof(*me));
        return -ENOMEM;
    }

    i2c_set_clientdata(client, me);

    mutex_init(&me->lock);
    me->dev      = &client->dev;
    me->irq      = -1;
    me->irq_gpio = -1;

	me->pmic = client;
	me->regmap_pmic = devm_regmap_init_i2c(client, &max77818_regmap_config);
    if (unlikely(IS_ERR(me->regmap_pmic))) {
        rc = PTR_ERR(me->regmap_pmic);
        me->regmap_pmic = NULL;
        pr_err("<%s> failed to initialize i2c regmap pmic [%d]\n", client->name,
            rc);
        goto abort;
    }

	me->chg = i2c_new_dummy(client->adapter, I2C_ADDR_CHARGER);
	if (!me->chg) {
		rc = -ENOMEM;
		goto abort;
	}	
	i2c_set_clientdata(me->chg, me);
	me->regmap_chg = devm_regmap_init_i2c(me->chg, &max77818_regmap_config);
    if (unlikely(IS_ERR(me->regmap_chg))) {
        rc = PTR_ERR(me->regmap_chg);
        me->regmap_chg = NULL;
        pr_err("<%s> failed to initialize i2c regmap chg [%d]\n", client->name,
            rc);
        goto abort;
    }

	me->fuel= i2c_new_dummy(client->adapter, I2C_ADDR_FUEL_GAUGE);
	if (!me->fuel) {
		rc = -ENOMEM;
		goto abort;
	}	
	i2c_set_clientdata(me->fuel, me);
	me->regmap_fuel= devm_regmap_init_i2c(me->fuel, &max77818_regmap_config_fuelgauge);
    if (unlikely(IS_ERR(me->regmap_fuel))) {
        rc = PTR_ERR(me->regmap_fuel);
        me->regmap_fuel = NULL;
        pr_err("<%s> failed to initialize i2c regmap fuelgauge [%d]\n", client->name,
            rc);
        goto abort;
    }

	rc = max77818_pmic_setup(me);
	if (rc != 0) {
		pr_err("<%s> failed to set up interrupt and add sub-device [%d]\n", client->name,
            rc);
        goto abort;
    }

    return 0;

abort:
    i2c_set_clientdata(client, NULL);
    max77818_destroy(me);
    return rc;
}

static int max77818_i2c_remove (struct i2c_client *client)
{
    struct max77818_dev *me = i2c_get_clientdata(client);

    printk("[---- SBA ----] max77818_i2c_remove Enter:\n");

    i2c_set_clientdata(client, NULL);
    max77818_destroy(me);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77818_suspend (struct device *dev)
{
    struct max77818_dev *me = dev_get_drvdata(dev);
    struct i2c_client *client = to_i2c_client(dev);

    __lock(me);

    pr_info("<%s> suspending\n", client->name);

    __unlock(me);
    return 0;
}

static int max77818_resume (struct device *dev)
{
    struct max77818_dev *me = dev_get_drvdata(dev);
    struct i2c_client *client = to_i2c_client(dev);

    __lock(me);

    pr_info("<%s> resuming\n", client->name);

    __unlock(me);
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77818_pm, max77818_suspend, max77818_resume);

#ifdef CONFIG_OF
static struct of_device_id max77818_of_id[] = {
    { .compatible = "maxim,"MAX77818_NAME      },
    { },
};
MODULE_DEVICE_TABLE(of, max77818_of_id);
#endif /* CONFIG_OF */

static const struct i2c_device_id max77818_i2c_id[] = {
    { MAX77818_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, max77818_i2c_id);

static struct i2c_driver max77818_i2c_driver = {
    .driver.name            = DRIVER_NAME,
    .driver.owner           = THIS_MODULE,
    .driver.pm              = &max77818_pm,
#ifdef CONFIG_OF
    .driver.of_match_table  = max77818_of_id,
#endif /* CONFIG_OF */
    .id_table               = max77818_i2c_id,
    .probe                  = max77818_i2c_probe,
    .remove                 = max77818_i2c_remove,
};

static __init int max77818_init (void)
{
	int rc = -ENODEV;

	printk("[---- SBA ----] max77818_init Enter:\n");

    rc = i2c_add_driver(&max77818_i2c_driver);
	if (rc != 0)
		pr_err("Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(max77818_init);

static __exit void max77818_exit (void)
{

	printk("[---- SBA ----] max77818_exit Enter:\n");

    i2c_del_driver(&max77818_i2c_driver);
}
module_exit(max77818_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
