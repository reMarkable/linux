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
 * @file xuvi_mu.h
 *
 */

#ifndef _XUVI_MU_H_
#define _XUVI_MU_H_

#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/mx8_mu.h>
#include <linux/workqueue.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/mailbox_client.h>
#include <linux/pm_domain.h>
#include <linux/kfifo.h>

#include "ppm.h"

int xuvi_mu_request(struct ppm_dev *dev);
void xuvi_mu_free(struct ppm_dev *dev);
void xuvi_mu_send_msg(struct ppm_dev *dev, uint32_t value0,
		      uint32_t value1, uint32_t value2, uint32_t value3);

#endif
