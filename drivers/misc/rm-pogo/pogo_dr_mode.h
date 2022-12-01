/* SPDX-License-Identifier: GPL-2.0-only */
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

#ifndef __POGO_DR_MODE_H__
#define __POGO_DR_MODE_H__

#include "pogo.h"

#define POGO_OTG_DR_MODE__DEVICE	0
#define POGO_OTG_DR_MODE__HOST	1

int pogo_init_extcon(struct rm_pogo_data *pdata);
int pogo_set_dr_mode(struct rm_pogo_data *otgc_dta, int mode);
int pogo_get_dr_mode(struct rm_pogo_data *pdata);

#endif // __POGO_DR_MODE_H__
