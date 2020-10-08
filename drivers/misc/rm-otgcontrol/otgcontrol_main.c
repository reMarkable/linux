/*
 * Sample kobject implementation
 *
 * Copyright (C) 2004-2007 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2007 Novell Inc.
 *
 * Released under the GPL version 2 only.
 *
 */
/* Internal includes */
#include <linux/rm-otgcontrol.h>

#include "otgcontrol_sysfs.h"
#include "otgcontrol_fsm.h"

/* Required Linux includes */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/string.h>

/* Sysfs */
#include <linux/kobject.h>

/* Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

/* Linking to charger/vbus supply driver */
#include <linux/power_supply.h>

/* Faking of VID signal to otg driver */
#include <linux/extcon.h>

static int rm_otgcontrol_init(struct rm_otgcontrol_data *otgc_data)
{
    int ret = 0;

    printk("%s: Initiating sysfs nodes\n", __func__);
    ret = otgcontrol_init_sysfs_nodes(otgc_data);
    if (ret < 0)
        return ret;

    printk("%s: Initiating default ONEWIRE_AUTH state\n", __func__);
    ret = otgcontrol_init_fsm(otgc_data);

    return ret;
}

#ifdef CONFIG_OF
static int rm_otgcontrol_parse_dt(struct rm_otgcontrol_data *otgc_data)
{
    struct device *dev = otgc_data->dev;
    struct device_node *np = dev->of_node;
    struct rm_otgcontrol_platform_data *pdata = otgc_data->pdata;
    const char *vbus_supply_name;
    int ret = 0;

    printk("[---- SBA ----] %s: Enter\n", __func__);

    if (of_find_property(np, "vbus-supply-name", NULL)) {
        printk("[---- SBA ----] %s: Found vbus-supply-name property, trying to read it\n", __func__);
        ret = of_property_read_string(np, "vbus-supply-name", &vbus_supply_name);
        if (ret) {
            printk("[---- SBA ----] %s: Failed to read property vbus-supply-name (code %d)\n", __func__, ret);
            return ret;
        }

        printk("[---- SBA ----] %s: Read vbus-supply-name: %s, trying to get reference to it\n", __func__, vbus_supply_name);
        pdata->vbus_supply = power_supply_get_by_name(vbus_supply_name);
        if (IS_ERR(pdata->vbus_supply)) {
            dev_err(dev, "%s: Failed to get supply '%s'\n", __func__, vbus_supply_name);
            return PTR_ERR(pdata->vbus_supply);
        }

        if (!pdata->vbus_supply) {
            printk("[---- SBA ----] %s: vbus supply not ready, defering probe\n", __func__);
            return -EPROBE_DEFER;
        }

        printk("[---- SBA ----] %s: Got pointer to vbus-supply\n", __func__);
    }
    else {
        printk("[---- SBA ----] %s: Failed to get pointer to vbus-supply - verify that the charger driver is loaded !\n", __func__);
    }

    if (ret < 0)
        return ret;
    else
        otgc_data->pdata = pdata;
    return 0;
}
#endif

static int rm_otgcontrol_probe(struct platform_device *pdev)
{
    struct rm_otgcontrol_data *otgc_data;
    struct rm_otgcontrol_platform_data *pdata;

    int ret = 0;

    printk("[---- SBA ----] %s: Enter:\n", __func__);

    pr_info("%s: rM OTGCONTROL Driver Loading\n", __func__);

    printk("[---- SBA ----] %s: Allocating otgcontrol_data\n", __func__);
    otgc_data = devm_kzalloc(&pdev->dev, sizeof(struct rm_otgcontrol_data), GFP_KERNEL);
    if (!otgc_data) {
        printk("[---- SBA ----] %s: Failed to allocate otgc_data\n", __func__);
        return -ENOMEM;
    }

    printk("[---- SBA ----] %s: Allocating otgcontrol_data\n", __func__);
    pdata = devm_kzalloc(&pdev->dev, sizeof(struct rm_otgcontrol_platform_data), GFP_KERNEL);
    if (unlikely(!pdata)) {
        pr_err("[---- SBA ----] %s: Failed to allocate pdata\n", __func__);
        pdata = ERR_PTR(-ENOMEM);
        ret = -ENOMEM;
        goto error_1;
    }

    otgc_data->dev = &pdev->dev;
    otgc_data->pdata = pdata;

#if defined(CONFIG_OF)
    printk("[---- SBA ---] %s: Reading platform data from devicetree\n", __func__);
    ret = rm_otgcontrol_parse_dt(otgc_data);
    if (ret < 0) {
        pr_err("[---- SBA ----] %s: Failed to load platform data from devicetree\n", __func__);
        goto error_2;
    }
#else
    printk("[---- SBA ----] %s: Driver does not support non-dt configuration\n", __func__);
    ret = -ENOTSUP;
    goto error_2;
#endif

    printk("[---- SBA ----] %s: Setting otgc_data reference in pdev, and initiating\n", __func__);
    ret = rm_otgcontrol_init(otgc_data);
    if(ret < 0)
        goto error_2;

    platform_set_drvdata(pdev, otgc_data);
    return 0;

error_1:
    kfree(otgc_data);
    return ret;

error_2:
    kfree(pdata);
    kfree(otgc_data);
    return ret;
}

static int rm_otgcontrol_remove(struct platform_device *pdev)
{
    struct rm_otgcontrol_data *otgc_data = platform_get_drvdata(pdev);

    printk("[---- SBA ----]otgcontrol1 %s Enter:\n", __func__);

    printk("%s: Un-initializing sysfs nodes\n", __func__);
    otgcontrol_uninit_sysfs_nodes(otgc_data);

    printk("%s: Freeing otgc->pdata\n", __func__);
    kfree(otgc_data->pdata);

    printk("%s: Freeing otgc\n", __func__);
    kfree(otgc_data);

    return 0;
}

#if defined CONFIG_PM
static int rm_otgcontrol_suspend(struct device *dev)
{
    printk("[---- SBA ----] %s Enter:\n", __func__);

    return 0;
}

static int rm_otgcontrol_resume(struct device *dev)
{
    printk("[---- SBA ----] %s Enter:\n", __func__);

    return 0;
}
#else
#define rm_otgcontrol_suspend NULL
#define rm_otgcontrol_resume NULL
#endif

#ifdef CONFIG_OF
static struct of_device_id rm_otgcontrol_dt_ids[] = {
    { .compatible = "rm-otgcontrol" },
    { }
};
MODULE_DEVICE_TABLE(of, rm_otgcontrol_dt_ids);
#endif

static SIMPLE_DEV_PM_OPS(rm_otgcontrol_pm_ops,
                        rm_otgcontrol_suspend,
                        rm_otgcontrol_resume);

static struct platform_driver rm_otgcontrol_driver = {
        .driver = {
        .name = "rM OTG Control",
        .owner = THIS_MODULE,
#ifdef CONFIG_PM
        .pm = &rm_otgcontrol_pm_ops,
#endif
#ifdef CONFIG_OF
        .of_match_table = rm_otgcontrol_dt_ids,
#endif
    },
    .probe = rm_otgcontrol_probe,
    .remove = rm_otgcontrol_remove,
};

static int __init otgcontrol_init(void)
{
    int ret;

    printk("%s: Registering platform driver 'rm-otgcontrol'\n", __func__);
    ret = platform_driver_register(&rm_otgcontrol_driver);
    if (ret < 0)
        printk("%s: Failed to register platform driver 'rm-otgcontrol', code %d\n", __func__, ret);

    return ret;
}

static void __exit otgcontrol_exit(void)
{
    printk("%s: Unregistering platform driver 'rm-otgcontrol'\n", __func__);
    platform_driver_unregister(&rm_otgcontrol_driver);
}

module_init(otgcontrol_init);
module_exit(otgcontrol_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("reMarkable OTG control driver, to enable authentication of devices connecting through the USB OTG interface");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Steinar Bakkemo <steinar.bakkemo@remarkable.no");
