#include "otgcontrol_dr_mode.h"

int otgcontrol_set_dr_mode(struct rm_otgcontrol_data *otgc_dta, int mode)
{
    switch(mode)
    {
    case OTG1_DR_MODE__DEVICE:
        printk("%s: Switching OTG1 DR mode -> DEVICE\n", __func__);
        /* Set ID pin emulation to LOW */
        break;

    case OTG1_DR_MODE__HOST:
        printk("%s: Switching OTG1 DR mode -> HOST\n", __func__);
        /* Set ID pin emulation to HIGH */
        break;

    default:
        printk("%s: unable to switch OTG1 DR mode (unknown mode %d)\n", __func__, mode);
        return -EINVAL;
    }
    return 0;
}

int otgcontrol_get_dr_mode(struct rm_otgcontrol_data *otgc_data)
{
    /* Just return the last stored value */
    return otgc_data->otg1_dr_mode;
}
