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

#ifndef __POGO_CHARGING_CTRL_H__
#define __POGO_CHARGING_CTRL_H__

#include "pogo.h"


#define POGO_CHARGERMODE_CHARGE	POWER_SUPPLY_MODE_CHARGER
#define POGO_CHARGERMODE_OTG	POWER_SUPPLY_MODE_OTG_SUPPLY
#define POGO_CHARGERMODE_OFF	POWER_SUPPLY_MODE_ALL_OFF

int pogo_get_otg_charger_modes(struct rm_pogo_data *pdata,
				     char *prop_buf);

int pogo_change_otg_charger_mode_int(struct rm_pogo_data *pdata,
					   int mode);

int pogo_change_otg_charger_mode_str(struct rm_pogo_data *pdata,
					   const char *buf);

#endif /* __POGO_CHARGING_CTRL_H__ */
