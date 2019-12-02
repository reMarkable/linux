/*
 * Driver for the IMX SNVS ON/OFF Power Key over sc api
 * Copyright (C) 2017 NXP. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware/imx/sci.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define DEBOUNCE_TIME	100
#define REPEAT_INTERVAL	60

#define SC_IRQ_BUTTON	(1U << 0U)     /* Button interrupt */
#define SC_IRQ_GROUP_WAKE	3U      /* Wakeup interrupts */

#define IMX_SC_MISC_FUNC_GET_BUTTON_STATUS	18U

struct pwrkey_drv_data {
	int keycode;
	bool keystate;  /* 1: pressed, 0: release */
	bool delay_check;
	struct imx_sc_ipc *ipcHandle;
	int wakeup;
	struct delayed_work check_work;
	struct input_dev *input;
};

struct imx_sc_msg_pwrkey {
	struct imx_sc_rpc_msg hdr;
	u32 state;
};

static struct pwrkey_drv_data *pdata;

static int imx_sc_pwrkey_notify(struct notifier_block *nb,
				      unsigned long event, void *group)
{
	/* ignore other irqs */
	if (!(pdata && pdata->ipcHandle && (event & SC_IRQ_BUTTON) &&
		(*(u8 *)group == SC_IRQ_GROUP_WAKE)))
		return 0;

	if (!pdata->delay_check) {
		pdata->delay_check = 1;
		schedule_delayed_work(&pdata->check_work,
					msecs_to_jiffies(REPEAT_INTERVAL));
	}

	return 0;
}

static void imx_sc_check_for_events(struct work_struct *work)
{
	struct input_dev *input = pdata->input;
	struct imx_sc_msg_pwrkey msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	bool state;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_MISC;
	hdr->func = IMX_SC_MISC_FUNC_GET_BUTTON_STATUS;
	hdr->size = 1;

	/*
	 * Current SCU firmware does NOT have return value for
	 * this API, that means it is always successful.
	 */
	imx_scu_call_rpc(pdata->ipcHandle, &msg, true);

	/* Only care the least 1 byte */
	state = (bool)(msg.state & 0xff);
	/*
	 * restore status back if press interrupt received but pin's status
	 * released, which caused by pressing so quickly.
	 */
	if (!state && !pdata->keystate)
		state = true;

	if (state ^ pdata->keystate) {
		pm_wakeup_event(input->dev.parent, 0);
		pdata->keystate = !!state;
		input_event(input, EV_KEY, pdata->keycode, !!state);
		input_sync(input);
		if (!state)
			pdata->delay_check = 0;
		pm_relax(pdata->input->dev.parent);
	}
	/* repeat check if pressed long */
	if (state)
		schedule_delayed_work(&pdata->check_work,
					msecs_to_jiffies(DEBOUNCE_TIME));
}

static struct notifier_block imx_sc_pwrkey_notifier = {
	.notifier_call = imx_sc_pwrkey_notify,
};

static int imx_sc_pwrkey_probe(struct platform_device *pdev)
{
	struct input_dev *input = NULL;
	struct device_node *np = pdev->dev.of_node;
	int error;
	int ret;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = imx_scu_get_handle(&pdata->ipcHandle);
	if (ret)
		return ret;

	if (of_property_read_u32(np, "linux,keycode", &pdata->keycode)) {
		pdata->keycode = KEY_POWER;
		dev_warn(&pdev->dev, "KEY_POWER without setting in dts\n");
	}

	INIT_DELAYED_WORK(&pdata->check_work, imx_sc_check_for_events);

	pdata->wakeup = of_property_read_bool(np, "wakeup-source");

	input = devm_input_allocate_device(&pdev->dev);

	if (!input) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		return -ENOMEM;
	}

	input->name = pdev->name;
	input->phys = "imx-sc-pwrkey/input0";
	input->id.bustype = BUS_HOST;

	input_set_capability(input, EV_KEY, pdata->keycode);

	error = input_register_device(input);
	if (error < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		return error;
	}

	pdata->input = input;
	platform_set_drvdata(pdev, pdata);

	device_init_wakeup(&pdev->dev, !!(pdata->wakeup));


	ret = imx_scu_irq_group_enable(SC_IRQ_GROUP_WAKE,
					SC_IRQ_BUTTON, true);
	if (ret) {
		dev_warn(&pdev->dev, "Enable irq failed.\n");
		return ret;
	}

	ret = imx_scu_irq_register_notifier(&imx_sc_pwrkey_notifier);
	if (ret) {
		imx_scu_irq_group_enable(SC_IRQ_GROUP_WAKE,
					SC_IRQ_BUTTON, false);
		dev_warn(&pdev->dev, "reqister scu notifier failed.\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id imx_sc_pwrkey_ids[] = {
	{ .compatible = "fsl,imx8-pwrkey" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_sc_pwrkey_ids);

static struct platform_driver imx_sc_pwrkey_driver = {
	.driver = {
		.name = "imx8-pwrkey",
		.of_match_table = imx_sc_pwrkey_ids,
	},
	.probe = imx_sc_pwrkey_probe,
};
module_platform_driver(imx_sc_pwrkey_driver);

MODULE_AUTHOR("Robin Gong <yibin.gong@nxp.com>");
MODULE_DESCRIPTION("i.MX8 power key driver based on scu");
MODULE_LICENSE("GPL");
