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

	const char							*one_wire_tty_name;					/* Name of the tty device connected to the one-wire interface pin */
	struct gpio_desc					*one_wire_gpio;						/* GPIO descriptor for GPIO connected to the one-wire-interface pin */
	int									one_wire_gpio_irq;					/* IRQ num for the GPIO num connected to the one-wire-interface pin */
	unsigned long						one_wire_gpio_debounce_jiffies;		/* Amount of jiffies corresponding to the defined debounce interval for the GPIO irq */
	struct delayed_work					one_wire_gpio_irq_work_queue;		/* Work queue used to do IRQ handling async/somewhat delayed/debounced */
	bool								one_wire_gpio_irq_is_active;		/* Local flag to keep track of irq enabled status, to prevent warnings */
	int									one_wire_gpio_state;				/* The last read state of the one-wire GPIO input */
	bool								one_wire_gpio_irq_is_handling;		/* Currently handling interrupt flag, to filter away irqs arriving while debouncing/handling */

	int									otg_controlstate;
	int									mode_requested;

	struct pinctrl*						one_wire_pinctrl;					/* Pinctrl object controlling the pin mux config for the one-wire pin */
	struct pinctrl_state*				one_wire_pinctrl_states[3];			/* The three pin mux definitions (pinmux states) to choose from */

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
