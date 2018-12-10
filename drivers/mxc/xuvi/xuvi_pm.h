/*
 * Copyright 2020 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *
 * @file xuvi_pm.h
 *
 */

#ifndef _XUVI_PM_H_
#define _XUVI_PM_H_

#include <linux/version.h>
#include <linux/pm_domain.h>
#include <linux/platform_device.h>

#include "ppm.h"

int xuvi_attach_pm_domains(struct ppm_dev *dev);
void xuvi_detach_pm_domains(struct ppm_dev *dev);

#endif
