/*
 * reMarkable OTG Control
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Steinar Bakkemo <steinar.bakkemo@remarkable.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __OTGCONTROL_DR_MODE_H__
#define __OTGCONTROL_DR_MODE_H__

#include "otgcontrol.h"

#define OTG1_DR_MODE__DEVICE	0
#define OTG1_DR_MODE__HOST	1

int otgcontrol_init_extcon(struct rm_otgcontrol_data *otgc_data);
int otgcontrol_set_dr_mode(struct rm_otgcontrol_data *otgc_dta, int mode);
int otgcontrol_get_dr_mode(struct rm_otgcontrol_data *otgc_data);

#endif // __OTGCONTROL_DR_MODE_H__
