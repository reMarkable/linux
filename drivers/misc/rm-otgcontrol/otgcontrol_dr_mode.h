#ifndef __OTGCONTROL_DR_MODE_H__
#define __OTGCONTROL_DR_MODE_H__

#include <linux/rm-otgcontrol.h>

#define OTG1_DR_MODE__DEVICE	0
#define OTG1_DR_MODE__HOST	1

int otgcontrol_init_extcon(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_uninit_extcon(struct rm_otgcontrol_data *otgc_data);
int otgcontrol_set_dr_mode(struct rm_otgcontrol_data *otgc_dta, int mode);
int otgcontrol_get_dr_mode(struct rm_otgcontrol_data *otgc_data);

#endif // __OTGCONTROL_DR_MODE_H__
