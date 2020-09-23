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

#ifndef __OTGCONTROL_ONE_WIRE_H__
#define __OTGCONTROL_ONE_WIRE_H__

#include "otgcontrol.h"

#include <linux/workqueue.h>
#include <linux/interrupt.h>

#define OTG1_ONEWIRE_STATE__GPIO			0
#define OTG1_ONEWIRE_STATE__UART_TX			1
#define OTG1_ONEWIRE_STATE__UART_RX			2

#define OTG1_ONEWIRE_GPIO_STATE__DEVICE_CONNECTED	0
#define OTG1_ONEWIRE_GPIO_STATE__DEVICE_NOT_CONNECTED	1

int otgcontrol_init_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data);

int otgcontrol_switch_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data,
					 int newState);
int otgcontrol_get_current_gpio_state(struct rm_otgcontrol_data *otgc_data);
const char *otgcontrol_gpio_state_name(int state);
int otgcontrol_init_gpio_irq(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_uninit_gpio_irq(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_activate_gpio_irq(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_deactivate_gpio_irq(struct rm_otgcontrol_data *otgc_data);
static irqreturn_t otgcontrol_gpio_irq_handler(int irq, void *data);
static void otgcontrol_gpio_irq_work(struct work_struct *work);
int otgcontrol_onewire_read_until_cr(struct rm_otgcontrol_data *otgc_data,
				     char *device_name,
				     char *buf,
				     int maxlen);
int otgcontrol_onewire_write_tty(struct rm_otgcontrol_data *otgc_data,
				 char *device_name,
				 char *text_to_send);

#endif /* __OTGCONTROL_ONE_WIRE_H__ */
