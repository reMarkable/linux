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
