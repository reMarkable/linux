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

#ifndef __RM_OTGCONTROL_H_
#define __RM_OTGCONTROL_H_

#include <linux/kobject.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/mutex.h>
#include <linux/extcon.h>

#define SYNC_SET_FLAG(flag, lock) ( \
	{ \
		mutex_lock(lock); \
		flag = true; \
		mutex_unlock(lock); \
	} \
)

#define SYNC_CLEAR_FLAG(flag, lock) ( \
	{ \
		mutex_lock(lock); \
		flag = false; \
		mutex_unlock(lock); \
	} \
)

#define SYNC_GET_FLAG(flag, lock) ( \
	{ \
		bool state; \
		mutex_lock(lock); \
		state = flag; \
		mutex_unlock(lock); \
		state; \
	} \
)

struct rm_otgcontrol_platform_data {
	/* Reference to charger driver for OTG power control */
	struct power_supply			*vbus_supply;

	/* One-wire tty device and gpio config */
	const char				*one_wire_tty_name;
	struct gpio_desc			*one_wire_gpio;
	int					one_wire_gpio_irq;
};

struct rm_otgcontrol_data {
	struct device           		*dev;
	struct rm_otgcontrol_platform_data	*pdata;

	struct mutex				lock;

	struct extcon_dev			*extcon_dev;

	unsigned long				one_wire_gpio_debounce_jiffies;
	struct delayed_work			one_wire_gpio_irq_work_queue;
	bool					one_wire_gpio_irq_is_active;
	int					one_wire_gpio_state;
	bool					one_wire_gpio_irq_is_handling;

	int					otg_controlstate;
	int					mode_requested;

	struct pinctrl*				one_wire_pinctrl;
	struct pinctrl_state*			one_wire_pinctrl_states[3];

	struct kobject*				kobject;

	struct kobj_attribute			otg1_device_connected_attribute;
	bool					otg1_device_connected;

	struct kobj_attribute			otg1_dr_mode_attribute;
	int					otg1_dr_mode;

	struct kobj_attribute			otg1_chargermode_attribute;
	int					otg1_chargermode;

	struct kobj_attribute			otg1_controllermode_attribute;
	int					otg1_controllermode;

	struct kobj_attribute			otg1_pinctrlstate_attribute;
	int					otg1_pinctrlstate;
};

#endif /* __RM_OTGCONTROL_H */
