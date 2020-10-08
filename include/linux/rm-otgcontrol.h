#ifndef __RM_OTGCONTROL_H_
#define __RM_OTGCONTROL_H_

#include <linux/kobject.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/extcon.h>

struct rm_otgcontrol_platform_data {
	struct power_supply		*vbus_supply;
};

struct rm_otgcontrol_data {
	struct device						*dev;
	struct rm_otgcontrol_platform_data 	*pdata;

	struct extcon_dev					*extcon_dev;

	const char							*one_wire_tty_name;
	struct gpio_desc					*one_wire_gpio;
	int									one_wire_gpio_irq;
	unsigned long						one_wire_gpio_debounce_jiffies;
	struct delayed_work					one_wire_gpio_irq_work_queue;
	bool								one_wire_gpio_irq_is_active;

	int									otg_controlstate;
	int									mode_requested;

	int									one_wire_state;
	struct pinctrl*						one_wire_pinctrl;
	struct pinctrl_state*				one_wire_pinctrl_states[3];

	struct kobject*						kobject;
	struct kobj_attribute				otg1_device_connected_attribute;
	bool								otg1_device_connected;

	struct kobj_attribute				otg1_dr_mode_attribute;
	int									otg1_dr_mode;

	struct kobj_attribute				otg1_chargermode_attribute;
	int									otg1_chargermode;

	struct kobj_attribute				otg1_controllermode_attribute;
	int									otg1_controllermode;

	struct kobj_attribute				otg1_pinctrlstate_attribute;
	int									otg1_pinctrlstate;
};

#endif /* __RM_OTGCONTROL_H */
