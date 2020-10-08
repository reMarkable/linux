#include "otgcontrol_sysfs.h"
#include "otgcontrol_fsm.h"
#include "otgcontrol_dr_mode.h"
#include "otgcontrol_charging_ctrl.h"
#include "otgcontrol_onewire.h"

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/export.h>
#include <linux/power_supply.h>

#define to_otgcontrol_data(kobj_attr_ptr, kobj_attr_member) \
	container_of(kobj_attr_ptr, struct rm_otgcontrol_data, kobj_attr_member);

#define SYSFS_PARENT_NODE NULL
#define SYSFS_NODE_NAME "otgcontrol"

#define STATUS_GROUP_ATTRIBUTE_COUNT	1
#define CONTROL_GROUP_ATTRIBUTE_COUNT	4

static struct attribute *control_attrs[CONTROL_GROUP_ATTRIBUTE_COUNT + 1] = {
	/* CONTROL_GROUP_ATTRIBUTE_COUNT number of initializing NULLs */
	NULL, NULL, NULL, NULL,

	/* NULL terminating the list */
	NULL
};

struct attribute_group control_attr_group = {
	.attrs = control_attrs,
	.name = "control"
};

struct attribute *status_attrs[STATUS_GROUP_ATTRIBUTE_COUNT + 1] = {
	/* STATUS_GROUP_ATTRIBUTE_COUNT number of NULLS */
	NULL,

	/* NULL terminating the list */
	NULL
};

struct attribute_group status_attr_group = {
	.attrs = status_attrs,
	.name = "status"
};

static ssize_t attribute_show(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      char *buf)
{
	int var;
	struct rm_otgcontrol_data *otgc_data;

	if (strcmp(attr->attr.name, "otg1_device_connected") == 0) {
		otgc_data = to_otgcontrol_data(attr,
					       otg1_device_connected_attribute);

		dev_dbg(otgc_data->dev,
			"%s: Returning cur otg1_device_connected value (%d)\n",
			__func__,
			otgc_data->otg1_device_connected);

		var = otgc_data->otg1_device_connected;
	}
	else if (strcmp(attr->attr.name, "otg1_dr_mode") == 0) {
		otgc_data = to_otgcontrol_data(attr,
					       otg1_dr_mode_attribute);

		dev_dbg(otgc_data->dev,
			"%s: Returning cur otg1_id_state value (%d)\n",
			__func__,
			otgc_data->otg1_dr_mode);

		var = otgc_data->otg1_dr_mode;
	}
	else if (strcmp(attr->attr.name, "otg1_chargermode") == 0) {
		otgc_data = to_otgcontrol_data(attr,
					       otg1_chargermode_attribute);

		dev_dbg(otgc_data->dev,
			"%s: Returning cur otg1_chargermode value (%d)\n",
			__func__,
			otgc_data->otg1_chargermode);

		var = otgc_data->otg1_chargermode;
	}
	else if (strcmp(attr->attr.name, "otg1_controllermode") == 0) {
		otgc_data = to_otgcontrol_data(attr,
					       otg1_controllermode_attribute);

		dev_dbg(otgc_data->dev,
			"%s: Returning cur otg1_controllermode value (%d)\n",
			__func__,
			otgc_data->otg1_controllermode);

		var = otgc_data->otg1_controllermode;
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: Invalid attribute name (%s)\n",
			__func__,
			attr->attr.name);
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", var);
}

static ssize_t attribute_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct rm_otgcontrol_data *otgc_data;
	int var, ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (strcmp(attr->attr.name, "otg1_dr_mode") == 0) {
		otgc_data = to_otgcontrol_data(attr,
					       otg1_dr_mode_attribute);

		dev_dbg(otgc_data->dev,
			"%s: Setting new otg1 dr mode (%d)\n",
			__func__,
			var);

		ret = otgcontrol_set_dr_mode(otgc_data,
					     var);
	}
	else if (strcmp(attr->attr.name, "otg1_chargermode") == 0) {
		otgc_data = to_otgcontrol_data(attr, otg1_chargermode_attribute);

		dev_dbg(otgc_data->dev,
			"%s: Setting new otg1 chargermode (%d)\n",
			__func__,
			var);

		ret = otgcontrol_change_otg_charge_mode(otgc_data,
							var);
	}
	else if (strcmp(attr->attr.name, "otg1_controllermode") == 0) {
		otgc_data = to_otgcontrol_data(attr,
					       otg1_controllermode_attribute);

		dev_dbg(otgc_data->dev,
			"%s: Setting new otg1 controllermode (%d)\n",
			__func__,
			var);

		ret = otgcontrol_handleInput(otgc_data,
					     OTG1_EVENT__MODE_CHANGE_REQUESTED,
					     (void*)&var);
	}
	else if (strcmp(attr->attr.name, "otg1_pinctrlstate") == 0) {
		otgc_data = to_otgcontrol_data(attr,
					       otg1_pinctrlstate_attribute);
		dev_dbg(otgc_data->dev,
			"%s: Setting new pinctrlstate (%d)\n",
			__func__,
			var);

		ret = otgcontrol_switch_one_wire_mux_state(otgc_data,
							   var);
	}
	else {
		return -EINVAL;
	}

	return count;
}

void otgcontrol_create_kobj_property(struct kobj_attribute *attr,
				     const char *name,
				     int permission,
				     ssize_t (*show)(struct kobject *kobj,
						     struct kobj_attribute *attr,
						     char *buf),
				     ssize_t (*store)(struct kobject *kobj,
						      struct kobj_attribute *attr,
						      const char *buf,
						      size_t count))
{
	attr->attr.name = name;
	attr->attr.mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO | S_IWUSR);
	attr->show = show;
	attr->store = store;
}

int otgcontrol_init_sysfs_nodes(struct rm_otgcontrol_data *otgc_data)
{
	struct kobject *otgcontrol_kobj;
	int retval;

	dev_dbg(otgc_data->dev,
		"%s: Creating control properties (R/W)\n",
		__func__);

	otgcontrol_create_kobj_property(&otgc_data->otg1_dr_mode_attribute,
					"otg1_dr_mode",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	otgcontrol_create_kobj_property(&otgc_data->otg1_chargermode_attribute,
					"otg1_chargermode",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	otgcontrol_create_kobj_property(&otgc_data->otg1_controllermode_attribute,
					"otg1_controllermode",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	otgcontrol_create_kobj_property(&otgc_data->otg1_pinctrlstate_attribute,
					"otg1_pinctrlstate",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	control_attrs[0] = &otgc_data->otg1_dr_mode_attribute.attr;
	control_attrs[1] = &otgc_data->otg1_chargermode_attribute.attr;
	control_attrs[2] = &otgc_data->otg1_controllermode_attribute.attr;
	control_attrs[3] = &otgc_data->otg1_pinctrlstate_attribute.attr;
	control_attrs[4] = NULL; /* NULL termination of the list */

	dev_dbg(otgc_data->dev,
		"%s: Creating status properties (R)\n",
		__func__);

	otgcontrol_create_kobj_property(&otgc_data->otg1_device_connected_attribute,
					"otg1_device_connected",
					S_IRUGO,
					attribute_show,
					attribute_store);

	status_attrs[0] = &otgc_data->otg1_device_connected_attribute.attr;
	status_attrs[1] = NULL;	/* NULL termination of the list */

	otgcontrol_kobj = kobject_create_and_add(SYSFS_NODE_NAME,
						 SYSFS_PARENT_NODE);
	if (!otgcontrol_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(otgcontrol_kobj,
				    &control_attr_group);

	retval = sysfs_create_group(otgcontrol_kobj,
				   &status_attr_group);
	if (retval) {
		otgc_data->kobject = otgcontrol_kobj;
	}

	kobject_put(otgc_data->kobject);
	return retval;
}

void otgcontrol_uninit_sysfs_nodes(struct rm_otgcontrol_data *otgc_data)
{
	dev_dbg(otgc_data->dev,
		"%s: Decrementing kobject refcount\n",
		__func__);

	if((otgc_data->kobject != NULL) && !IS_ERR(otgc_data->kobject)) {
		sysfs_remove_group(otgc_data->kobject, &control_attr_group);
		sysfs_remove_group(otgc_data->kobject, &status_attr_group);
		kobject_put(otgc_data->kobject);
		otgc_data->kobject = NULL;
	}
}
