#ifndef __OTGCONTROL_CHARGING_CTRL_H__
#define __OTGCONTROL_CHARGING_CTRL_H__

#include <linux/rm-otgcontrol.h>

#define OTG1_CHARGERMODE_CHARGE                         0
#define OTG1_CHARGERMODE_OTG                            1

int otgcontrol_change_otg_charge_mode(struct rm_otgcontrol_data *otgc_data, int mode);

#endif /* __OTGCONTROL_CHARGING_CTRL_H__ */
