#include "otgcontrol_onewire.h"
#include "otgcontrol_fsm.h"

#include <linux/export.h>
#include <linux/printk.h>
#include <linux/errno.h>
#include <linux/pinctrl/consumer.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define ONE_WIRE_GPIO_DEBOUNCE_MS							200	/* ms */

int otgcontrol_init_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	printk("%s: Initiating one-wire pinctrl states\n", __func__);
	otgc_data->one_wire_pinctrl = devm_pinctrl_get(otgc_data->dev);
	if (IS_ERR(otgc_data->one_wire_pinctrl)) {
		printk("%s: Failed to get pinctrl\n", __func__);
		return PTR_ERR(otgc_data->one_wire_pinctrl);
	}

	otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO] = pinctrl_lookup_state(otgc_data->one_wire_pinctrl, "default");
	if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO])) {
		printk("%s: Failed to configure one-wire-gpio state\n", __func__);
		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
	}

	otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX] = pinctrl_lookup_state(otgc_data->one_wire_pinctrl, "one_wire_uart_tx");
	if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX])) {
		printk("%s: Failed to configure one-wire-uart-tx state\n", __func__);
		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX]);
	}

	otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX] = pinctrl_lookup_state(otgc_data->one_wire_pinctrl, "one_wire_uart_rx");
	if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX])) {
		printk("%s: Failed to configure one-wire-uart-rx\n", __func__);
		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX]);
	}

	printk("%s: Setting default state (GPIO)\n", __func__);
	ret = pinctrl_select_state(otgc_data->one_wire_pinctrl, otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
	if (ret < 0) {
		printk("%s: Failed to set default state (GPIO)\n", __func__);
		devm_pinctrl_put(otgc_data->one_wire_pinctrl);
		return ret;
	}
	return 0;
}

void otgcontrol_uninit_onw_wire_mux_state(struct rm_otgcontrol_data *otgc_data)
{
	devm_pinctrl_put(otgc_data->one_wire_pinctrl);
}

int otgcontrol_switch_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data, int state)
{
	int ret;

	switch(state)
	{
	case OTG1_ONEWIRE_STATE__GPIO:
		printk("%s: Switching onewire state -> GPIO\n", __func__);
		ret = pinctrl_select_state(otgc_data->one_wire_pinctrl, otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
		if (ret < 0) {
			printk("%s: Failed to set pinctrl state\n", __func__);
			return ret;
		}
		break;

	case OTG1_ONEWIRE_STATE__UART_RX:
		printk("%s: Switching onewire state -> UART RX\n", __func__);
		ret = pinctrl_select_state(otgc_data->one_wire_pinctrl, otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX]);
		if (ret < 0) {
			printk("%s: Failed to set pinctrl state\n", __func__);
			return ret;
		}
		break;

	case OTG1_ONEWIRE_STATE__UART_TX:
		printk("%s: switching onewire state -> UART TX\n", __func__);
		ret = pinctrl_select_state(otgc_data->one_wire_pinctrl, otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX]);
		if (ret < 0) {
			printk("%s: Failed to set pinctrl state\n", __func__);
			return ret;
		}
		break;

	default:
		printk("%s: unable to switch onewire state (unknown state %d)\n", __func__, state);
		return -EINVAL;
	}

	return 0;
}

int otgcontrol_get_current_gpio_state(struct rm_otgcontrol_data *otgc_data)
{
	printk("%s: Enter\n", __func__);
	return gpiod_get_raw_value(otgc_data->one_wire_gpio);
}

int otgcontrol_init_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	printk("%s: Enter\n", __func__);

	printk("%s: Setting local gpio debounce jiffies\n", __func__);
	otgc_data->one_wire_gpio_debounce_jiffies = msecs_to_jiffies(ONE_WIRE_GPIO_DEBOUNCE_MS);

	printk("%s: Initiating irq worker\n", __func__);
	INIT_DELAYED_WORK(&otgc_data->one_wire_gpio_irq_work_queue, otgcontrol_gpio_irq_work);

	printk("%s: Getting IRQ from given gpio (%d)\n",
		   __func__,
		   desc_to_gpio(otgc_data->one_wire_gpio));
	otgc_data->one_wire_gpio_irq = gpiod_to_irq(otgc_data->one_wire_gpio);
	if (otgc_data->one_wire_gpio_irq < 0) {
		dev_err(otgc_data->dev, "%s: Failed to get irq for given gpio (%d)\n",
				__func__,
				desc_to_gpio(otgc_data->one_wire_gpio));
		return otgc_data->one_wire_gpio_irq;
	}

	printk("%s: Requesting threaded irq\n", __func__);
	ret = devm_request_threaded_irq(otgc_data->dev, otgc_data->one_wire_gpio_irq, NULL,
					otgcontrol_gpio_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"one_wire_gpio_irq", otgc_data);
	if (ret < 0) {
		dev_err(otgc_data->dev, "%s: Failed to request handler for IRQ (%d) given for GPIO (%d)\n",
				__func__,
				otgc_data->one_wire_gpio_irq,
				desc_to_gpio(otgc_data->one_wire_gpio));
		return ret;
	}
	otgc_data->one_wire_gpio_irq_is_active = true;

	return 0;
}

void otgcontrol_activate_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
	if (!otgc_data->one_wire_gpio_irq_is_active) {
		enable_irq(otgc_data->one_wire_gpio_irq);
		otgc_data->one_wire_gpio_irq_is_active = true;
	}
}

void otgcontrol_deactivate_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
	if (otgc_data->one_wire_gpio_irq_is_active) {
		disable_irq(otgc_data->one_wire_gpio_irq);
		otgc_data->one_wire_gpio_irq_is_active = false;
	}
}

static irqreturn_t otgcontrol_gpio_irq_handler(int irq, void *data)
{
	struct rm_otgcontrol_data *otgc_data = (struct rm_otgcontrol_data*)data;

	printk("%s: Enter\n", __func__);
	/* otgcontrol_deactivate_gpio_irq(otgc_data); */
	queue_delayed_work(system_power_efficient_wq, &otgc_data->one_wire_gpio_irq_work_queue, otgc_data->one_wire_gpio_debounce_jiffies);

	return IRQ_HANDLED;
}

static void otgcontrol_gpio_irq_work(struct work_struct *work)
{
	int cur_gpio_state;

	struct delayed_work *delayed_work = container_of(work, struct delayed_work, work);
	struct rm_otgcontrol_data *otgc_data = container_of(delayed_work, struct rm_otgcontrol_data, one_wire_gpio_irq_work_queue);

	printk("%s: Enter\n", __func__);

	cur_gpio_state = otgcontrol_get_current_gpio_state(otgc_data);
	if (cur_gpio_state != otgc_data->one_wire_state) {
		printk("%s: GPIO state changed -> %s\n", __func__, cur_gpio_state ? "HIGH" : "LOW");
		otgc_data->one_wire_state = cur_gpio_state;

		if (otgc_data->one_wire_state == 0)
			otgcontrol_handleInput(otgc_data, OTG1_EVENT__DEVICE_CONNECTED, NULL);
		else
			otgcontrol_handleInput(otgc_data, OTG1_EVENT__DEVICE_DISCONNECTED, NULL);
	}

	/* otgcontrol_activate_gpio_irq(otgc_data); */
}

int otgcontrol_onewire_write_tty(char *device_name, char *text_to_send)
{
	// Create variables
	struct file *f;
	char buf[128];
	mm_segment_t fs;
	int i;

	// Init the buffer with 0
	for(i = 0;i < 128;i++)
		buf[i] = 0;

	printk("%s: Trying to open %s\n", __func__, device_name);
	f = filp_open(device_name, O_RDWR, 0);
	if(f == NULL) {
		printk("%s: filp_open error!!.\n", __func__);
		return -1;
	}
	else {
		// Get current segment descriptor
		printk("%s: Getting current segment descriptor\n", __func__);
		fs = get_fs();

		// Set segment descriptor associated to kernel space
		printk("%s: Setting segment descriptor\n", __func__);
		set_fs(get_ds());

		//Write to the file
		printk("%s: Writing '%s' to file\n", __func__, text_to_send);
		kernel_write(f, text_to_send, strlen(text_to_send), &f->f_pos);
//        f->f_op->write(f, text_to_send, strlen(text_to_send), &f->f_pos);

		// Restore segment descriptor
		printk("%s: Restoring segment descriptor\n", __func__);
		set_fs(fs);

		printk("%s: Closing file\n", __func__);
		filp_close(f,NULL);
		return 0;
	}
}

int otgcontrol_onewire_read_until_cr(char *device_name, char *buf, int maxlen)
{
	// Create variables
	struct file *f;
	mm_segment_t fs;
	char newchar;
	int pos;

	f = filp_open(device_name, O_RDONLY, 0);
	if(f == NULL) {
		printk("%s: filp_open error!!.\n", __func__);
		return -1;
	}
	else {

		// Get current segment descriptor
		printk("%s: Getting current segment descriptor\n", __func__);
		fs = get_fs();

		// Set segment descriptor associated to kernel space
		printk("%s: Setting segment descriptor\n", __func__);
		set_fs(get_ds());

		pos = 0;
		int state = 0;
		printk("%s: Starting read loop\n", __func__);
		do {
			// Read the file
//            f->f_op->read(f, &newchar, 1, &f->f_pos);
			kernel_read(f, &newchar, 1, &f->f_pos);
			printk("%s: <-%c (0x%02x)\n", __func__, newchar, newchar);
			switch(state)
			{
			case 0:
				// Wait :
				if (newchar == ':') {
					printk("%s: SOF\n", __func__);
					state = 1;
				}
				break;
			default:
				// Reading chars
				if (newchar != '#') {
					buf[pos++] = newchar;
				}
			}
		}while((newchar != '#') && (pos < maxlen));
		printk("%s: Done\n", __func__);

		// Restore segment descriptor
		printk("%s: Restoring segment descriptor\n", __func__);
		set_fs(fs);

		printk("%s: Closing file\n", __func__);
		filp_close(f,NULL);

		printk("%s: Returning %d bytes read\n", __func__, pos);
		return pos;
	}
}
