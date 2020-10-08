#include "otgcontrol_onewire.h"

#include <linux/export.h>
#include <linux/printk.h>
#include <linux/errno.h>
#include <linux/pinctrl/consumer.h>

int otgcontrol_init_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data)
{
    int ret;

    printk("%s: Initiating one-wire pinctrl states\n", __func__);
    otgc_data->one_wire_pinctrl = devm_pinctrl_get(otgc_data->dev);

    otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO] = pinctrl_lookup_state(otgc_data->one_wire_pinctrl, "one_wire_gpio");
    if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO])) {
        devm_pinctrl_put(otgc_data->one_wire_pinctrl);
        return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
    }

    otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX] = pinctrl_lookup_state(otgc_data->one_wire_pinctrl, "one_wire_uart6_tx");
    if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX])) {
        devm_pinctrl_put(otgc_data->one_wire_pinctrl);
        return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_TX]);
    }

    otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX] = pinctrl_lookup_state(otgc_data->one_wire_pinctrl, "one_wire_uart6_rx");
    if (IS_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX])) {
        devm_pinctrl_put(otgc_data->one_wire_pinctrl);
        return PTR_ERR(otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__UART_RX]);
    }

    printk("%s: Setting default state (GPIO)\n", __func__);
    ret = pinctrl_select_state(otgc_data->one_wire_pinctrl, otgc_data->one_wire_pinctrl_states[OTG1_ONEWIRE_STATE__GPIO]);
    if (ret < 0) {
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
    printk("%s: Enter\n", __func__);

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
    return 0;
}

int otgcontrol_init_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);
    return 0;
}

void otgcontrol_activate_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);
}

int otgcontrol_deactivate_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);
    return 0;
}

int otgcontrol_gpio_irq(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);
    return 0;
}

void otgcontrol_gpio_irq_work(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);
}
