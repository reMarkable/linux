#include <linux/rm-otgcontrol.h>

#include "otgcontrol_fsm.h"
#include "otgcontrol_onewire.h"
#include "otgcontrol_charging_ctrl.h"
#include "otgcontrol_dr_mode.h"

#include <linux/printk.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/power_supply.h>
#include <linux/fs.h>      // Needed by filp
#include <asm/uaccess.h>   // Needed by segment descriptors
#include <linux/delay.h>

int write_tty(char *device_name, char *text_to_send);
int read_until_cr(char *device_name, char *buf, int maxlen);

int otgcontrol_init_fsm(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);

    printk("%s: Initiating onewire state to GPIO\n", __func__);
    otgcontrol_init_one_wire_mux_state(otgc_data);

    printk("%s: Checking if device is connected\n", __func__);
    if(otgcontrol_get_current_gpio_state(otgc_data) == 0) {
        printk("%s: Device is connected, running onewire authentication process", __func__);
        otgc_data->one_wire_state = 0;
        otgcontrol_start_onewire_authentication(otgc_data);
    }
    else {
        printk("%s: Device is not connected\n", __func__);
        printk("%s: Setting OTG1 mode (CHARGE)", __func__);
        otgc_data->one_wire_state = 1;

        otgcontrol_change_otg_charge_mode(otgc_data, OTG1_CHARGERMODE_CHARGE);

        printk("%s: Activating onewire gpio interrupt\n", __func__);
        otgcontrol_activate_gpio_irq(otgc_data);

        printk("%s: Waiting for low on GPIO input when device is connected", __func__);
        otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_NOT_CONNECTED;
    }
    return 0;
}
//EXPORT_SYMBOL(otgcontrol_init_fsm);

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
    int i;

    printk("%s: Enter\n", __func__);

    switch(signal)
    {
    case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
        printk("%s: CHALLENGE REPLY RECEIVED\n", __func__);
        printk("%s: PLEASE IMPLEMENT THIS !\n", __func__);
        break;

    case OTG1_EVENT__MODE_CHANGE_REQUESTED:
        printk("%s: MODE CHANGE REQUESTED\n", __func__);
        printk("%s: PLEASE IMPLEMENT THIS !\n", __func__);
        break;

    case OTG1_EVENT__ONEWIRE_GPIO_STATE_CHANGED:
        printk("%s: ONEWIRE GPIO STATE CHANGED\n", __func__);

        if(!otgc_data->one_wire_state) {
            // Device connected
            otgcontrol_change_otg_charge_mode(otgc_data, OTG1_CHARGERMODE_OTG); /* OTG POWER ON */
            otgcontrol_set_controlmode(otgc_data, OTG1_DR_MODE__HOST);
            otgcontrol_deactivate_gpio_irq(otgc_data);

            for(i = 0;i < 5000;i++) udelay(1000);

            otgcontrol_switch_one_wire_mux_state(otgc_data, OTG1_ONEWIRE_STATE__UART_TX);
            char tty_device_name[50], buf[100];
            sprintf(tty_device_name, "/dev/%s", otgc_data->one_wire_tty_name);
            int ret = write_tty("/dev/ttymxc5", ":0001ff#");

            for(i = 0;i < 100;i++) udelay(1000);
            otgcontrol_switch_one_wire_mux_state(otgc_data, OTG1_ONEWIRE_STATE__UART_RX);

            if(ret == 0) {
                int count = read_until_cr("/dev/ttymxc5", buf, 100);
                buf[count] = 0;
                printk("%s: Read '%s'", __func__, buf);
            }
        }
        else {
            // Device disconnected
            otgcontrol_change_otg_charge_mode(otgc_data, OTG1_CHARGERMODE_CHARGE); /* OTG POWER OFF */
            otgcontrol_set_controlmode(otgc_data, OTG1_DR_MODE__DEVICE);
        }
        break;

    case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
        printk("%s: CHARGERMODE CHANGE REQUESTED\n", __func__);
        printk("%s: PLEASE IMPLEMENT THIS !\n", __func__);
        break;

    case OTG1_EVENT__TIMEOUT:
        printk("%s: TIMEOUT\n", __func__);
        printk("%s: PLEASE IMPLEMENT SUPPORT FOR THIS !\n", __func__);
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

    printk("%s: PLEASE IMPLEMENT THIS !\n", __func__);
    return 0;
}


int write_tty(char *device_name, char *text_to_send)
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
        f->f_op->write(f, text_to_send, strlen(text_to_send), &f->f_pos);

        // Restore segment descriptor
        printk("%s: Restoring segment descriptor\n", __func__);
        set_fs(fs);

        printk("%s: Closing file\n", __func__);
        filp_close(f,NULL);
        return 0;
    }
}

int read_until_cr(char *device_name, char *buf, int maxlen)
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
            f->f_op->read(f, &newchar, 1, &f->f_pos);
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
                    printk("%s: <-%c (0x%02x\n", __func__, newchar, newchar);
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
