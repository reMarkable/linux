#include "otgcontrol_sysfs.h"
#include "otgcontrol_fsm.h"
#include "otgcontrol_dr_mode.h"
#include "otgcontrol_charging_ctrl.h"

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/export.h>
#include <linux/power_supply.h>

#define to_otgcontrol_data(kobj_attr_ptr, kobj_attr_member) container_of(kobj_attr_ptr, struct rm_otgcontrol_data, kobj_attr_member);

#define SYSFS_PARENT_NODE NULL
#define SYSFS_NODE_NAME "otgcontrol"

static ssize_t attribute_show(struct kobject *kobj, struct kobj_attribute *attr,
              char *buf)
{
    int var;
    struct rm_otgcontrol_data *otgc_data;

    printk("%s: Enter\n", __func__);

    if (strcmp(attr->attr.name, "otg1_device_connected") == 0) {
        otgc_data = to_otgcontrol_data(attr, otg1_device_connected_attribute);
        printk("%s: Returning cur otg1_device_connected value (%d)\n", __func__, otgc_data->otg1_device_connected);
        var = otgc_data->otg1_device_connected;
    }
    else if (strcmp(attr->attr.name, "otg1_dr_mode") == 0) {
        otgc_data = to_otgcontrol_data(attr, otg1_dr_mode_attribute);
        printk("%s: Returning cur otg1_id_state value (%d)\n", __func__, otgc_data->otg1_dr_mode);
        var = otgc_data->otg1_dr_mode;
    }
    else if (strcmp(attr->attr.name, "otg1_chargermode") == 0) {
        otgc_data = to_otgcontrol_data(attr, otg1_chargermode_attribute);
        printk("%s: Returning cur otg1_chargermode value (%d)\n", __func__, otgc_data->otg1_chargermode);
        var = otgc_data->otg1_chargermode;
    }
    else if (strcmp(attr->attr.name, "otg1_controllermode") == 0) {
        otgc_data = to_otgcontrol_data(attr, otg1_controllermode_attribute);
        printk("%s: Returning cur otg1_controllermode value (%d)\n", __func__, otgc_data->otg1_controllermode);
        var = otgc_data->otg1_controllermode;
    }
    else {
        printk("%s: Invalid attribute name (%s)\n", __func__, attr->attr.name);
        return -EINVAL;
    }

    return sprintf(buf, "%d\n", var);
}

static ssize_t attribute_store(struct kobject *kobj, struct kobj_attribute *attr,
               const char *buf, size_t count)
{
    struct rm_otgcontrol_data *otgc_data;
    int var, ret;

    printk("%s: Enter\n", __func__);

    ret = kstrtoint(buf, 10, &var);
    if (ret < 0)
        return ret;

    if (strcmp(attr->attr.name, "otg1_dr_mode") == 0) {
        otgc_data = to_otgcontrol_data(attr, otg1_dr_mode_attribute);
        printk("%s: Setting new otg1 dr mode (%d)\n", __func__, var);
        ret = otgcontrol_set_dr_mode(otgc_data, var);
    }
    else if (strcmp(attr->attr.name, "otg1_chargermode") == 0) {
        otgc_data = to_otgcontrol_data(attr, otg1_chargermode_attribute);
        printk("%s: Setting new otg1 chargermode (%d)\n", __func__, var);
        ret = otgcontrol_change_otg_charge_mode(otgc_data, var);
    }
    else if (strcmp(attr->attr.name, "otg1_controllermode") == 0) {
        otgc_data = to_otgcontrol_data(attr, otg1_controllermode_attribute);
        printk("%s: Setting new otg1 controllermode (%d)\n", __func__, var);
        ret = otgcontrol_handleInput(otgc_data, OTG1_EVENT__MODE_CHANGE_REQUESTED, (void*)&var);
    }
    else {
        return -EINVAL;
    }

    return count;
}

void otgcontrol_create_kobj_property(struct kobj_attribute *attr,
                                     const char *name,
                                     int permission,
                                     ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr, char *buf),
                                     ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count))
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

    printk("%s: Enter\n", __func__);

    printk("%s: Creating control properties (R/W)\n", __func__);
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

    struct attribute *control_attrs[] = {
        &otgc_data->otg1_dr_mode_attribute.attr,
        &otgc_data->otg1_chargermode_attribute.attr,
        &otgc_data->otg1_controllermode_attribute.attr,
        NULL,	/* need to NULL terminate the list of attributes */
    };

    printk("%s: Creating status properties (R)\n", __func__);
    otgcontrol_create_kobj_property(&otgc_data->otg1_device_connected_attribute,
                                    "otg1_device_connected",
                                    S_IRUGO,
                                    attribute_show,
                                    attribute_store);

    struct attribute *status_attrs[] = {
        &otgc_data->otg1_device_connected_attribute.attr,
        NULL,	/* need to NULL terminate the list of attributes */
    };

    struct attribute_group control_attr_group = {
        .attrs = control_attrs,
        .name = "control"
    };

    struct attribute_group status_attr_group = {
        .attrs = status_attrs,
        .name = "status"
    };

    otgcontrol_kobj = kobject_create_and_add(SYSFS_NODE_NAME, SYSFS_PARENT_NODE);
    if (!otgcontrol_kobj)
        return -ENOMEM;

    /* Create the files associated with this kobject */
    retval = sysfs_create_group(otgcontrol_kobj, &control_attr_group);
    retval = sysfs_create_group(otgcontrol_kobj, &status_attr_group);
    if (retval) {
        otgc_data->kobject = otgcontrol_kobj;
    }

    kobject_put(otgc_data->kobject);
    return retval;
}
EXPORT_SYMBOL(otgcontrol_init_sysfs_nodes);

void otgcontrol_uninit_sysfs_nodes(struct rm_otgcontrol_data *otgc_data)
{
    printk("%s: Enter\n", __func__);
    printk("%s: Decrementing kobject refcount\n", __func__);
    if(!IS_ERR(otgc_data->kobject))
        kobject_put(otgc_data->kobject);
}
EXPORT_SYMBOL(otgcontrol_uninit_sysfs_nodes);
