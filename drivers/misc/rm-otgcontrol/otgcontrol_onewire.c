#include "otgcontrol_onewire.h"
#include "otgcontrol_fsm.h"

#include <linux/export.h>
#include <linux/errno.h>
#include <linux/pinctrl/consumer.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>

#define ONE_WIRE_GPIO_DEBOUNCE_MS	500	/* ms */

int otgcontrol_init_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	dev_dbg(otgc_data->dev,
		"%s: Initiating one-wire pinctrl states\n",
		__func__);

	otgc_data->one_wire_pinctrl = devm_pinctrl_get(otgc_data->dev);
	if (IS_ERR(otgc_data->one_wire_pinctrl)) {
		dev_err(otgc_data->dev,
			"%s: Failed to get pinctrl\n",
			__func__);

		return PTR_ERR(otgc_data->one_wire_pinctrl);
	}

	otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO] =
			pinctrl_lookup_state(otgc_data->one_wire_pinctrl,
					     "default");

	if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO])) {
		dev_err(otgc_data->dev,
			"%s: Failed to configure one-wire-gpio state\n",
			__func__);

		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
	}

	otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX] =
			pinctrl_lookup_state(otgc_data->one_wire_pinctrl,
					     "one_wire_uart_tx");

	if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX])) {
		dev_err(otgc_data->dev,
			"%s: Failed to configure one-wire-uart-tx state\n",
			__func__);

		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX]);
	}

	otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX] =
			pinctrl_lookup_state(otgc_data->one_wire_pinctrl,
					     "one_wire_uart_rx");

	if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX])) {
		dev_err(otgc_data->dev,
			"%s: Failed to configure one-wire-uart-rx\n",
			__func__);

		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX]);
	}

	dev_dbg(otgc_data->dev,
		"%s: Setting default state (GPIO)\n",
		__func__);

	ret = pinctrl_select_state(
		otgc_data->one_wire_pinctrl,
		otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
	if (ret < 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to set default state (GPIO)\n",
			__func__);

		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return ret;
	}
	return 0;
}

int otgcontrol_switch_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data,
					 int state)
{
	int ret;

	switch(state)
	{
	case OTG1_ONEWIRE_STATE__GPIO:
		dev_dbg(otgc_data->dev,
			"%s: Switching onewire state -> GPIO\n",
			__func__);

		ret = pinctrl_select_state(
			otgc_data->one_wire_pinctrl,
			otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to set pinctrl state\n",
				__func__);
			return ret;
		}
		break;

	case OTG1_ONEWIRE_STATE__UART_RX:
		dev_dbg(otgc_data->dev,
			"%s: Switching onewire state -> UART RX\n",
			__func__);

		ret = pinctrl_select_state(
			otgc_data->one_wire_pinctrl,
			otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX]);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to set pinctrl state\n",
				__func__);
			return ret;
		}
		break;

	case OTG1_ONEWIRE_STATE__UART_TX:
		dev_dbg(otgc_data->dev,
			"%s: switching onewire state -> UART TX\n",
			__func__);

		ret = pinctrl_select_state(
			otgc_data->one_wire_pinctrl,
			otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX]);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to set pinctrl state\n",
				__func__);
			return ret;
		}
		break;

	default:
		dev_err(otgc_data->dev,
			"%s: unable to switch onewire state (unknown state %d)\n",
			__func__, state);
		return -EINVAL;
	}

	return 0;
}

int otgcontrol_get_current_gpio_state(struct rm_otgcontrol_data *otgc_data)
{
	return gpiod_get_raw_value(otgc_data->pdata->one_wire_gpio);
}

const char *otgcontrol_gpio_state_name(int state)
{
	switch(state)
	{
	case OTG1_ONEWIRE_GPIO_STATE__DEVICE_CONNECTED:
		return "DEVICE CONNECTED";
		break;
	case OTG1_ONEWIRE_GPIO_STATE__DEVICE_NOT_CONNECTED:
		return "DEVICE NOT CONNECTED";
		break;
	default:
		return "UNKNOWN DEVICE CONNECTION STATE EXPECTED 0 or 1)";
	}
}

int otgcontrol_init_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	dev_dbg(otgc_data->dev,
		"%s: Setting local gpio debounce jiffies\n",
		__func__);

	otgc_data->one_wire_gpio_debounce_jiffies =
			msecs_to_jiffies(ONE_WIRE_GPIO_DEBOUNCE_MS);

	dev_dbg(otgc_data->dev,
		"%s: Initiating irq worker\n",
		__func__);

	INIT_DELAYED_WORK(&otgc_data->one_wire_gpio_irq_work_queue,
			  otgcontrol_gpio_irq_work);

	dev_dbg(otgc_data->dev,
		"%s: Getting IRQ from given gpio (%d)\n",
		   __func__,
		   desc_to_gpio(otgc_data->pdata->one_wire_gpio));

	otgc_data->pdata->one_wire_gpio_irq = gpiod_to_irq(otgc_data->pdata->one_wire_gpio);
	if (otgc_data->pdata->one_wire_gpio_irq < 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to get irq for given gpio (%d)\n",
			__func__,
			desc_to_gpio(otgc_data->pdata->one_wire_gpio));
		return otgc_data->pdata->one_wire_gpio_irq;
	}

	otgc_data->one_wire_gpio_state =
			otgcontrol_get_current_gpio_state(otgc_data);

	dev_dbg(otgc_data->dev,
		"%s: Current GPIO state: %s\n",
		__func__,
		otgcontrol_gpio_state_name(otgc_data->one_wire_gpio_state));

	dev_dbg(otgc_data->dev,
		"%s: Clearing is-handling flag\n",
		__func__);

	otgc_data->one_wire_gpio_irq_is_handling = false;

	dev_dbg(otgc_data->dev,
		"%s: Requesting threaded irq\n",
		__func__);

	ret = devm_request_threaded_irq(
		otgc_data->dev,
		otgc_data->pdata->one_wire_gpio_irq,
		NULL,
		otgcontrol_gpio_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
		"one_wire_gpio_irq",
		otgc_data);
	if (ret < 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to request handler for IRQ (%d) "
			"given for GPIO (%d)\n",
			__func__,
			otgc_data->pdata->one_wire_gpio_irq,
			desc_to_gpio(otgc_data->pdata->one_wire_gpio));
		return ret;
	}
	mutex_lock(&otgc_data->lock);
	otgc_data->one_wire_gpio_irq_is_active = true;
	mutex_unlock(&otgc_data->lock);

	return 0;
}

void otgcontrol_uninit_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
	cancel_delayed_work_sync(&otgc_data->one_wire_gpio_irq_work_queue);
}

void otgcontrol_activate_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
	if (!otgc_data->one_wire_gpio_irq_is_active) {
		enable_irq(otgc_data->pdata->one_wire_gpio_irq);
		otgc_data->one_wire_gpio_irq_is_active = true;
	}
}

void otgcontrol_deactivate_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
	if (otgc_data->one_wire_gpio_irq_is_active) {
		disable_irq(otgc_data->pdata->one_wire_gpio_irq);
		otgc_data->one_wire_gpio_irq_is_active = false;
	}
}

static irqreturn_t otgcontrol_gpio_irq_handler(int irq, void *data)
{
	struct rm_otgcontrol_data *otgc_data = (struct rm_otgcontrol_data*)data;

	if (SYNC_GET_FLAG(otgc_data->one_wire_gpio_irq_is_handling,
			  &otgc_data->lock)) {
		dev_dbg(otgc_data->dev,
			"%s: Is already handling irq, ignoring this\n",
			__func__);
		return IRQ_HANDLED;
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: Queueing IRQ handling for execution in %d ms\n",
			__func__,
			ONE_WIRE_GPIO_DEBOUNCE_MS);

		SYNC_SET_FLAG(otgc_data->one_wire_gpio_irq_is_handling,
			      &otgc_data->lock);

		queue_delayed_work(system_power_efficient_wq,
				   &otgc_data->one_wire_gpio_irq_work_queue,
				   otgc_data->one_wire_gpio_debounce_jiffies);
	}

	return IRQ_HANDLED;
}

static void otgcontrol_gpio_irq_work(struct work_struct *work)
{
	int cur_gpio_state;

	struct delayed_work *delayed_work =
			container_of(work,
				     struct delayed_work,
				     work);

	struct rm_otgcontrol_data *otgc_data =
			container_of(delayed_work,
				     struct rm_otgcontrol_data,
				     one_wire_gpio_irq_work_queue);

	dev_dbg(otgc_data->dev,
		"%s: Checking current gpio state\n",
		__func__);

	cur_gpio_state = otgcontrol_get_current_gpio_state(otgc_data);
	if (cur_gpio_state != otgc_data->one_wire_gpio_state) {
		dev_dbg(otgc_data->dev,
			"%s: GPIO state changed -> %s\n",
			__func__,
			otgcontrol_gpio_state_name(cur_gpio_state));

		otgc_data->one_wire_gpio_state = cur_gpio_state;

		if (otgc_data->one_wire_gpio_state ==
				OTG1_ONEWIRE_GPIO_STATE__DEVICE_CONNECTED) {
			SYNC_SET_FLAG(otgc_data->otg1_device_connected,
				      &otgc_data->lock);

			otgcontrol_handleInput(otgc_data,
					       OTG1_EVENT__DEVICE_CONNECTED,
					       NULL);
		}
		else {
			SYNC_CLEAR_FLAG(otgc_data->otg1_device_connected,
					&otgc_data->lock);
			otgcontrol_handleInput(otgc_data,
					       OTG1_EVENT__DEVICE_DISCONNECTED,
					       NULL);
		}
	}

	SYNC_CLEAR_FLAG(otgc_data->one_wire_gpio_irq_is_handling,
			&otgc_data->lock);
}

int otgcontrol_onewire_write_tty(struct rm_otgcontrol_data *otgc_data,
				 char *device_name,
				 char *text_to_send)
{
	struct file *f;
	char buf[128];
	mm_segment_t fs;
	int i;

	for(i = 0;i < 128;i++)
		buf[i] = 0;

	dev_dbg(otgc_data->dev, "%s: Trying to open %s\n",
		__func__,
		device_name);

	f = filp_open(device_name, O_RDWR, 0);
	if(f == NULL) {
		dev_err(otgc_data->dev,
			"%s: filp_open error!!.\n",
			__func__);
		return -1;
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: Getting current segment descriptor\n",
			__func__);

		fs = get_fs();

		dev_dbg(otgc_data->dev,
			"%s: Setting segment descriptor\n",
			__func__);
		set_fs(KERNEL_DS);

		dev_dbg(otgc_data->dev,
			"%s: Writing '%s' to file\n",
			__func__,
			text_to_send);

		kernel_write(f,
			     text_to_send,
			     strlen(text_to_send),
			     &f->f_pos);

		dev_dbg(otgc_data->dev,
			"%s: Restoring segment descriptor\n",
			__func__);

		set_fs(fs);

		dev_dbg(otgc_data->dev,
			"%s: Closing file\n",
			__func__);

		filp_close(f,NULL);
		return 0;
	}
}

int otgcontrol_onewire_read_until_cr(struct rm_otgcontrol_data *otgc_data, char *device_name, char *buf, int maxlen)
{
	struct file *f;
	mm_segment_t fs;
	char newchar;
	int pos, state;

	f = filp_open(device_name, O_RDONLY, 0);
	if(f == NULL) {
		dev_err(otgc_data->dev,
			"%s: filp_open error!!.\n",
			__func__);
		return -1;
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: Getting current segment descriptor\n",
			__func__);

		fs = get_fs();

		dev_dbg(otgc_data->dev,
			"%s: Setting segment descriptor\n",
			__func__);
		set_fs(KERNEL_DS);

		pos = 0;
		state = 0;
		dev_dbg(otgc_data->dev,
			"%s: Starting read loop\n",
			__func__);
		do {
			kernel_read(f,
				    &newchar,
				    1,
				    &f->f_pos);

			dev_dbg(otgc_data->dev,
				"%s: <-%c (0x%02x)\n",
				__func__,
				newchar, newchar);

			switch(state)
			{
			case 0:
				if (newchar == ':') {
					dev_dbg(otgc_data->dev,
						"%s: SOF\n",
						__func__);

					state = 1;
				}
				break;
			default:
				if (newchar != '#') {
					buf[pos++] = newchar;
				}
			}
		}while((newchar != '#') && (pos < maxlen));

		dev_dbg(otgc_data->dev,
			"%s: Done\n",
			__func__);

		dev_dbg(otgc_data->dev,
			"%s: Restoring segment descriptor\n",
			__func__);

		set_fs(fs);

		dev_dbg(otgc_data->dev,
			"%s: Closing file\n",
			__func__);

		filp_close(f, NULL);

		dev_dbg(otgc_data->dev,
			"%s: Returning %d bytes read\n",
			__func__,
			pos);

		return pos;
	}
}
