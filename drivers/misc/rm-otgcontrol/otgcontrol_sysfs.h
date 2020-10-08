#ifndef __OTGCONTROL_SYSFS_H__
#define __OTGCONTROL_SYSFS_H__

#include <linux/rm-otgcontrol.h>

int otgcontrol_init_sysfs_nodes(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_uninit_sysfs_nodes(struct rm_otgcontrol_data *otgc_data);

#endif /* __OTGCONTROL_SYSFS_H__ */
