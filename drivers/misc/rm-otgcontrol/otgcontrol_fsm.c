#include <linux/rm-otgcontrol.h>

#include "otgcontrol_fsm.h"
#include "otgcontrol_onewire.h"
#include "otgcontrol_charging_ctrl.h"

#include <linux/printk.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/power_supply.h>

int otgcontrol_init_fsm(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);

    printk("%s: Initiating onewire state to GPIO\n", __func__);
    otgcontrol_init_one_wire_mux_state(otgc_data);

    printk("%s: Checking if device is connected\n", __func__);
    if(otgcontrol_get_current_gpio_state(otgc_data) == 0) {
        printk("%s: Device is connected, running onewire authentication process", __func__);
        otgcontrol_start_onewire_authentication(otgc_data);
    }
    else {
        printk("%s: Device is not connected\n", __func__);
        printk("%s: Setting OTG1 mode (CHARGE)", __func__);
        otgcontrol_change_otg_charge_mode(otgc_data, OTG1_CHARGERMODE_CHARGE);

        printk("%s: Activating onewire gpio interrupt\n", __func__);
        otgcontrol_activate_gpio_irq(otgc_data);

        printk("%s: Waiting for low on GPIO input when device is connected", __func__);
        otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_NOT_CONNECTED;
    }
    return 0;
}
EXPORT_SYMBOL(otgcontrol_init_fsm);

int otgcontrol_set_controlmode(struct rm_otgcontrol_data *otgc_data, int mode)
{
    printk("%s: Enter\n", __func__);

    switch(mode)
    {
    case OTG_MODE__MANUAL_CONTROL:
        printk("%s: setting MANUAL_CONTROL mode\n", __func__);
        break;

    case OTG_MODE__ONEWIRE_AUTH:
        printk("%s: setting ONEWIRE_AUTH mode\n", __func__);
        break;

    case OTG_MODE__USB_NO_AUTH:
        printk("%s: setting USB_NO_AUTH mode\n", __func__);
        break;

    default:
        printk("%s: unable to set control mode (unknown mode %d)\n", __func__, mode);
        return -EINVAL;
    }

    return 0;
}

int otgcontrol_handleInput(struct rm_otgcontrol_data *otgc_data, int signal, void* param)
{
    printk("%s: Enter\n", __func__);

    switch(signal)
    {
    case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
        printk("%s: CHALLENGE REPLY RECEIVED\n", __func__);
        break;

    case OTG1_EVENT__MODE_CHANGE_REQUESTED:
        printk("%s: MODE CHANGE REQUESTED\n", __func__);
        break;

    case OTG1_EVENT__ONEWIRE_GPIO_STATE_CHANGED:
        printk("%s: ONEWIRE GPIO STATE CHANGED\n", __func__);
        break;

    case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
        printk("%s: CHARGERMODE CHANGE REQUESTED\n", __func__);
        break;

    case OTG1_EVENT__TIMEOUT:
        printk("%s: TIMEOUT\n", __func__);
        break;

    case OTG1_EVENT__VBUS_CHANGED:
        printk("%s: VBUS CHANGED\n", __func__);
        break;

    default:
        printk("%s: Unknown signal/event (%d)\n", __func__, signal);
        return -EINVAL;
    }

    return 0;
}

static int otgcontrol_start_onewire_authentication(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);

    printk("%s: PLEASE IMPLEMENT THIS\n", __func__);
    return 0;
}
