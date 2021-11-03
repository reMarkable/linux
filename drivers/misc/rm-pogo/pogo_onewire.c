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

#include "pogo_fsm.h"

#include <linux/export.h>
#include <linux/errno.h>
#include <linux/pinctrl/consumer.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>

const char *otg_onewire_pinctrl_name[] = {
	[POGO_ONEWIRE_STATE__GPIO] = "default",
	[POGO_ONEWIRE_STATE__UART_TX] = "one_wire_uart_tx",
	[POGO_ONEWIRE_STATE__UART_RX] = "one_wire_uart_rx",
	[POGO_ONEWIRE_STATE__GPIO_5K_5K] = "one_wire_gpio_5k_5k",
	[POGO_ONEWIRE_STATE__GPIO_5K_47K] = "one_wire_gpio_5k_47k",
	[POGO_ONEWIRE_STATE__GPIO_5K_100K] = "one_wire_gpio_5k_100k",
	[POGO_ONEWIRE_STATE__GPIO_47K_47K] = "one_wire_gpio_47k_47k",
	[POGO_ONEWIRE_STATE__GPIO_47K_100K] = "one_wire_gpio_47k_100k",
	[POGO_ONEWIRE_STATE__GPIO_100K_100K] = "one_wire_gpio_100k_100k",
	[POGO_ONEWIRE_STATE__GPIO_100K_PD_100K_PD] = "one_wire_gpio_100k_PD_100k_PD",
};

static irqreturn_t pogo_gpio_irq_handler(int irq, void *data);
static void pogo_gpio_irq_work(struct work_struct *work);

int pogo_init_one_wire_mux_state(struct rm_pogo_data *pdata)
{
	int ret, idx;

	dev_dbg(pdata->dev,
		"%s: Initiating one-wire pinctrl states\n",
		__func__);

	pdata->one_wire_pinctrl = devm_pinctrl_get(pdata->dev);
	if (IS_ERR(pdata->one_wire_pinctrl)) {
		dev_err(pdata->dev,
			"%s: Failed to get pinctrl\n",
			__func__);

		return PTR_ERR(pdata->one_wire_pinctrl);
	}

	pdata->pogo_gpio_pinctrl_index = POGO_ONEWIRE_STATE__GPIO;

	for (idx = 0; idx < POGO_ONEWIRE_STATE_NR; idx++) {
		pdata->one_wire_pinctrl_states[idx] =
			pinctrl_lookup_state(pdata->one_wire_pinctrl,
					     otg_onewire_pinctrl_name[idx]);

		if (IS_ERR(pdata->one_wire_pinctrl_states[idx])) {
			dev_err(pdata->dev,
				"%s: Failed to find pin state %d: %s\n",
				__func__, idx, otg_onewire_pinctrl_name[idx]);

			devm_pinctrl_put(pdata->one_wire_pinctrl);
			return PTR_ERR(pdata->one_wire_pinctrl_states[idx]);
		}
	}

	dev_dbg(pdata->dev,
		"%s: Setting default state (GPIO)\n",
		__func__);

	ret = pinctrl_select_state(
		pdata->one_wire_pinctrl,
		pdata->one_wire_pinctrl_states[POGO_ONEWIRE_STATE__GPIO]);
	if (ret < 0) {
		dev_err(pdata->dev,
			"%s: Failed to set default state (GPIO)\n",
			__func__);

		devm_pinctrl_put(pdata->one_wire_pinctrl);
	}

	pdata->onewire_pinctrlstate = POGO_ONEWIRE_STATE__GPIO;
	return 0;
}

int pogo_switch_one_wire_mux_state(struct rm_pogo_data *pdata,
					 int state)
{
	int ret;

	if (state < 0 || state >= ARRAY_SIZE(pdata->one_wire_pinctrl_states)) {
		dev_err(pdata->dev,
			"%s: unable to switch onewire state (unknown state %d)\n",
			__func__, state);
		return -EINVAL;
	}

	ret = pinctrl_select_state(pdata->one_wire_pinctrl,
				   pdata->one_wire_pinctrl_states[state]);

	/* dev_dbg(pdata->dev, "%s: switched onewire state -> %s\n",   */
	/*                 __func__, otg_onewire_pinctrl_name[state]); */

	if (ret < 0) {
		dev_err(pdata->dev,
			"%s: Failed to set pinctrl state %s\n",
			__func__, otg_onewire_pinctrl_name[state]);
		return ret;
	}

	pdata->onewire_pinctrlstate = state;
	return 0;
}

int pogo_get_current_gpio_state(struct rm_pogo_data *pdata)
{
	return gpiod_get_raw_value(pdata->one_wire_gpio);
}

const char *pogo_gpio_state_name(int state)
{
	switch(state)
	{
	case POGO_ONEWIRE_GPIO_STATE__POGO_CONNECTED:
		return "DEVICE CONNECTED";
		break;
	case POGO_ONEWIRE_GPIO_STATE__POGO_NOT_CONNECTED:
		return "DEVICE NOT CONNECTED";
		break;
	default:
		return "UNKNOWN DEVICE CONNECTION STATE EXPECTED 0 or 1)";
	}
}

int pogo_init_gpio_irq(struct rm_pogo_data *pdata)
{
	int ret;

	dev_dbg(pdata->dev,
		"%s: Initiating irq worker\n",
		__func__);

	INIT_WORK(&pdata->one_wire_gpio_irq_work_queue,
			  pogo_gpio_irq_work);

	dev_dbg(pdata->dev,
		"%s: Getting IRQ from given gpio (%d)\n",
		__func__,
		desc_to_gpio(pdata->one_wire_gpio));

	pdata->one_wire_gpio_irq = gpiod_to_irq(pdata->one_wire_gpio);
	if (pdata->one_wire_gpio_irq < 0) {
		dev_err(pdata->dev,
			"%s: Failed to get irq for given gpio (%d)\n",
			__func__,
			desc_to_gpio(pdata->one_wire_gpio));
		return pdata->one_wire_gpio_irq;
	}

	pdata->one_wire_gpio_state =
			pogo_get_current_gpio_state(pdata);

	dev_dbg(pdata->dev,
		"%s: Current GPIO state: %s\n",
		__func__,
		pogo_gpio_state_name(pdata->one_wire_gpio_state));

	dev_dbg(pdata->dev,
		"%s: Clearing is-handling flag\n",
		__func__);

	dev_dbg(pdata->dev,
		"%s: Requesting threaded irq\n",
		__func__);

	ret = devm_request_irq(
		pdata->dev,
		pdata->one_wire_gpio_irq,
		pogo_gpio_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_SHARED | IRQF_NO_SUSPEND,
		"one_wire_gpio_irq",
		pdata);
	if (ret < 0) {
		dev_err(pdata->dev,
			"%s: Failed to request handler for IRQ (%d) "
			"given for GPIO (%d)\n",
			__func__,
			pdata->one_wire_gpio_irq,
			desc_to_gpio(pdata->one_wire_gpio));
		return ret;
	}

	enable_irq_wake(pdata->one_wire_gpio_irq);

	/* FSM will enable it when in idle state entry */
	disable_irq(pdata->one_wire_gpio_irq);
	return 0;
}

void pogo_activate_gpio_irq(struct rm_pogo_data *pdata)
{
	dev_dbg(pdata->dev, "%s\n", __func__);
	pdata->irq_detected = false;
	enable_irq(pdata->one_wire_gpio_irq);
}

void pogo_uninit_gpio_irq(struct rm_pogo_data *pdata)
{
	dev_dbg(pdata->dev, "%s\n", __func__);
	pdata->irq_detected = false;
	disable_irq(pdata->one_wire_gpio_irq);
}

static irqreturn_t pogo_gpio_irq_handler(int irq, void *data)
{
	struct rm_pogo_data *pdata = (struct rm_pogo_data*)data;

	if (pdata->suspend && device_may_wakeup(pdata->dev))
		pm_wakeup_event(pdata->dev, 0);

	if (pdata->pogo_fsm_state != POGO_STATE__ONEWIRE_IDLE)
		return IRQ_HANDLED;

	pdata->vbus_short = false;
	queue_work(system_power_efficient_wq,
			   &pdata->one_wire_gpio_irq_work_queue);

	return IRQ_HANDLED;
}

static void pogo_gpio_irq_work(struct work_struct *work)
{
	struct rm_pogo_data *pdata =
			container_of(work,
				     struct rm_pogo_data,
				     one_wire_gpio_irq_work_queue);

	dev_dbg(pdata->dev, "%s\n", __func__);
	mutex_lock(&pdata->lock);
	if (!pdata->ignore_next_irq) {
		pdata->irq_detected = true;
		wake_up_process(pdata->fsm_thread);
	} else {
		dev_dbg(pdata->dev, "%s: ignoring\n", __func__);
		pdata->ignore_next_irq = false;
	}
	mutex_unlock(&pdata->lock);
}
