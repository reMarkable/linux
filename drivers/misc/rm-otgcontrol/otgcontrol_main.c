/*
 * reMarkable OTG Control
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Steinar Bakkemo <steinar.bakkemo@remarkable.com>
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

#include "otgcontrol.h"
#include "otgcontrol_sysfs.h"
#include "otgcontrol_fsm.h"
#include "otgcontrol_dr_mode.h"
#include "otgcontrol_onewire.h"

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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/kobject.h>
#include <linux/power_supply.h>
#include <linux/extcon.h>

static int rm_otgcontrol_init(struct rm_otgcontrol_data *otgc_data)
{
	int ret = 0;

	dev_dbg(otgc_data->dev,
		"%s: Initiating sysfs nodes\n",
		__func__);

	ret = otgcontrol_init_sysfs_nodes(otgc_data);
	if (ret < 0)
		return ret;

	dev_dbg(otgc_data->dev,
		"%s: Initiating extcon device to control USB OTG dr mode\n",
		__func__);

	ret = otgcontrol_init_extcon(otgc_data);
	if (ret < 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to initiate extron (%d)\n",
			__func__, ret);
		return ret;
	}

	dev_dbg(otgc_data->dev,
		"%s: Initiating onewire state and setting to default state "
		"(GPIO)\n",
		__func__);

	ret = otgcontrol_init_one_wire_mux_state(otgc_data);
	if (ret < 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to initiate onewire pincontrol "
			"configuration\n",
			__func__);
		return ret;
	}

	dev_dbg(otgc_data->dev,
		"%s: Initiating one-wire gpio irq\n",
		__func__);

	ret = otgcontrol_init_gpio_irq(otgc_data);
	if (ret < 0)
		return ret;

	dev_dbg(otgc_data->dev,
		"%s: Initiating fsm to start in AUTHORIZED MODE !!\n",
		__func__);

	ret = otgcontrol_init_fsm(otgc_data);
	return ret;
}

static int rm_otgcontrol_parse_dt(struct rm_otgcontrol_data *otgc_data)
{
	struct device *dev = otgc_data->dev;
	struct device_node *np = dev->of_node;
	struct rm_otgcontrol_platform_data *pdata = otgc_data->pdata;
	const char *vbus_supply_name;
	int ret = 0;

	dev_dbg(otgc_data->dev,
		"%s: Enter\n",
		__func__);

	if (of_find_property(np, "vbus-supply", NULL)) {
		dev_dbg(otgc_data->dev,
			"%s: Found vbus-supply property, "
			"trying to get get vbus powersupply by phandle\n",
			__func__);

		pdata->vbus_supply = power_supply_get_by_phandle(np,
								 "vbus-supply");
		if (IS_ERR_OR_NULL(pdata->vbus_supply)) {
			dev_dbg(otgc_data->dev,
				"%s: vbus supply not ready, defering probe\n",
				__func__);
			return -EPROBE_DEFER;
		}
	}
	else if (of_find_property(np, "vbus-supply-name", NULL)) {
		dev_dbg(otgc_data->dev,
			"%s: Found vbus-supply-name property, "
			"trying to read it\n",
			__func__);

		ret = of_property_read_string(np,
					      "vbus-supply-name",
					      &vbus_supply_name);
		if (ret) {
			dev_err(otgc_data->dev,
				"%s: Failed to read property vbus-supply-name "
				"(code %d)\n",
				__func__, ret);
			return -EINVAL;
		}

		dev_dbg(otgc_data->dev,
			"%s: Read vbus-supply-name: %s, "
			"trying to get reference to it\n",
			__func__, vbus_supply_name);

		pdata->vbus_supply = power_supply_get_by_name(vbus_supply_name);
		if (IS_ERR(pdata->vbus_supply)) {
			dev_dbg(otgc_data->dev,
				"%s: vbus supply not ready, defering probe\n",
				__func__);
			return -EPROBE_DEFER;
		}
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: Required vbus-supply-name property not given !\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(otgc_data->dev,
		"%s: Got pointer to vbus-supply\n",
		__func__);

	if (of_find_property(np, "one-wire-tty-name", NULL)) {
		dev_dbg(otgc_data->dev,
			"%s: Found one-wire-tty-name property, "
			"trying to read it\n",
			__func__);

		ret = of_property_read_string(np,
					      "one-wire-tty-name",
					      &otgc_data->pdata->one_wire_tty_name);
		if (ret) {
			dev_err(otgc_data->dev,
				"%s: Failed to read property one-wire-tty-name "
				"(code %d)\n",
				__func__, ret);
			return -EINVAL;
		}
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: required property one-wire-tty-name not given !\n",
			__func__);
		return -EINVAL;
	}

	if (of_find_property(np, "one-wire-gpios", NULL)) {
		dev_dbg(otgc_data->dev,
			"%s: Found one-wire-gpio property, trying to read it\n",
			__func__);

		otgc_data->pdata->one_wire_gpio = devm_gpiod_get(otgc_data->dev,
							  "one-wire",
							  GPIOD_IN);
		if (IS_ERR(otgc_data->pdata->one_wire_gpio)) {
			dev_err(otgc_data->dev,
				"%s: Failed to read property one-wire-gpio "
				"(code %ld)\n",
				__func__,
				PTR_ERR(otgc_data->pdata->one_wire_gpio));
			return PTR_ERR(otgc_data->pdata->one_wire_gpio);
		}
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: required property one-wire-gpio not given !\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

static int rm_otgcontrol_probe(struct platform_device *pdev)
{
	struct rm_otgcontrol_data *otgc_data;
	struct rm_otgcontrol_platform_data *pdata;

	int ret = 0;

	dev_dbg(&pdev->dev,
		"%s: rM OTGCONTROL Driver Loading\n",
		__func__);

	dev_dbg(&pdev->dev,
		"%s: Allocating otgcontrol_data\n",
		__func__);

	otgc_data = devm_kzalloc(&pdev->dev,
				 sizeof(struct rm_otgcontrol_data),
				 GFP_KERNEL);
	if (!otgc_data) {
		dev_err(&pdev->dev,
			"%s: Failed to allocate otgc_data\n",
			__func__);
		return -ENOMEM;
	}

	dev_dbg(&pdev->dev,
		"%s: Allocating otgcontrol_data\n",
		__func__);

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct rm_otgcontrol_platform_data),
			     GFP_KERNEL);
	if (unlikely(!pdata)) {
		dev_err(&pdev->dev,
			"%s: Failed to allocate pdata\n",
			__func__);
		pdata = ERR_PTR(-ENOMEM);
		ret = -ENOMEM;
		goto error_1;
	}

	otgc_data->dev = &pdev->dev;
	otgc_data->pdata = pdata;

	dev_dbg(&pdev->dev,
		"%s: Reading platform data from devicetree\n",
		__func__);

	ret = rm_otgcontrol_parse_dt(otgc_data);
	if (ret < 0) {
		if (ret == -EPROBE_DEFER) {
			dev_info(&pdev->dev,
				 "%s: Defering probe due to charger driver not being"
				 "loaded/available yet\n",
				 __func__);
		}
		else {
			dev_err(&pdev->dev,
				"%s: Failed to load platform data from devicetree, "
				"code %d\n",
				__func__,
				ret);
		}
		goto error_1;
	}

	dev_dbg(&pdev->dev,
		"%s: Setting otgc_data reference in pdev, and initiating\n",
		__func__);

	ret = rm_otgcontrol_init(otgc_data);
	if(ret < 0) {
		dev_err(&pdev->dev,
			"%s: Failed to init otgcontrol, "
			"code %d\n",
			__func__,
			ret);
		goto error_2;
	}

	platform_set_drvdata(pdev, otgc_data);

	dev_info(&pdev->dev,
		 "Loaded successfully !\n");

	return 0;

error_2:
	otgcontrol_uninit_sysfs_nodes(otgc_data);
	otgcontrol_uninit_gpio_irq(otgc_data);

error_1:
	/* No need to do explicit calls to devm_kfree */
	return ret;
}

static int rm_otgcontrol_remove(struct platform_device *pdev)
{
	struct rm_otgcontrol_data *otgc_data = platform_get_drvdata(pdev);

	dev_dbg(otgc_data->dev,
		"%s: Un-initializing sysfs nodes\n",
		__func__);
	otgcontrol_uninit_sysfs_nodes(otgc_data);

	dev_dbg(otgc_data->dev,
		"%s: Un-initialize gpio irq\n",
		__func__);
	otgcontrol_uninit_gpio_irq(otgc_data);

	return 0;
}

#if defined CONFIG_PM
static int rm_otgcontrol_suspend(struct device *dev)
{
	dev_dbg(dev, "%s Enter:\n", __func__);

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int rm_otgcontrol_resume(struct device *dev)
{
	dev_dbg(dev, "%s Enter:\n", __func__);

	pinctrl_pm_select_default_state(dev);

	return 0;
}
#else
#define rm_otgcontrol_suspend NULL
#define rm_otgcontrol_resume NULL
#endif

static struct of_device_id rm_otgcontrol_dt_ids[] = {
	{ .compatible = "rm-otgcontrol" },
	{ }
};
MODULE_DEVICE_TABLE(of, rm_otgcontrol_dt_ids);

static SIMPLE_DEV_PM_OPS(rm_otgcontrol_pm_ops,
			 rm_otgcontrol_suspend,
			 rm_otgcontrol_resume);

static struct platform_driver rm_otgcontrol_driver = {
	.driver = {
		.name = "rm_otg_control",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &rm_otgcontrol_pm_ops,
#endif
		.of_match_table = rm_otgcontrol_dt_ids,
	},
	.probe = rm_otgcontrol_probe,
	.remove = rm_otgcontrol_remove,
};

module_platform_driver(rm_otgcontrol_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("reMarkable OTG control driver, to enable authentication of "
		   "devices connecting through the USB OTG interface");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Steinar Bakkemo <steinar.bakkemo@remarkable.no");
