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

#ifndef __POGO_SYSFS_H__
#define __POGO_SYSFS_H__

#include "pogo.h"

int pogo_init_sysfs_nodes(struct rm_pogo_data *pogo_data);
void pogo_uninit_sysfs_nodes(struct rm_pogo_data *pogo_data);

#endif /* __POGO_SYSFS_H__ */
