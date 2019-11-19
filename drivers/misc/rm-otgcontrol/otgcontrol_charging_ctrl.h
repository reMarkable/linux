#ifndef __OTGCONTROL_CHARGING_CTRL_H__
#define __OTGCONTROL_CHARGING_CTRL_H__

#include "otgcontrol.h"

#define OTG1_CHARGERMODE_CHARGE	0
#define OTG1_CHARGERMODE_OTG	1
#define OTG1_CHARGERMODE_OFF	2

int otgcontrol_get_otg_charger_modes(struct rm_otgcontrol_data *otgc_data,
				     char *prop_buf);

int otgcontrol_change_otg_charger_mode_int(struct rm_otgcontrol_data *otgc_data,
					   int mode);

int otgcontrol_change_otg_charger_mode_str(struct rm_otgcontrol_data *otgc_data,
					   const char *buf);

#endif /* __OTGCONTROL_CHARGING_CTRL_H__ */
