#include "otgcontrol_charging_ctrl.h"

#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/power_supply.h>

int otgcontrol_change_otg_charge_mode(struct rm_otgcontrol_data *otgc_data, int mode)
{
	int ret;
	union power_supply_propval property_val;

	printk("%s: Enter\n", __func__);

	switch(mode)
	{
	case OTG1_CHARGERMODE_CHARGE:
		printk("%s: Setting OTG1 chargermode (CHARGE)\n", __func__);
		property_val.intval = POWER_SUPPLY_MODE_CHARGER;
		ret = power_supply_set_property(otgc_data->pdata->vbus_supply,
										POWER_SUPPLY_PROP_CHARGER_MODE,
										&property_val);
		if (ret < 0) {
			printk("%s: Failed to set charger mode\n", __func__);
			return ret;
		}

		property_val.intval = 1;
		ret = power_supply_set_property(otgc_data->pdata->vbus_supply,
										POWER_SUPPLY_PROP_ONLINE,
										&property_val);
		if (ret < 0) {
			printk("%s: Failed to enable charging after changing charger mode\n", __func__);
			return ret;
		}
		break;

	case OTG1_CHARGERMODE_OTG:
		printk("%s: Setting OTG1 chargermode (OTG)\n", __func__);
		property_val.intval = POWER_SUPPLY_MODE_OTG_SUPPLY;
		ret = power_supply_set_property(otgc_data->pdata->vbus_supply,
										POWER_SUPPLY_PROP_CHARGER_MODE,
										&property_val);
		if (ret < 0) {
			printk("%s: Failed to set charger mode\n", __func__);
			return ret;
		}

		property_val.intval = 1;
		ret = power_supply_set_property(otgc_data->pdata->vbus_supply,
										POWER_SUPPLY_PROP_ONLINE,
										&property_val);
		if (ret < 0) {
			printk("%s: Failed to enable charging after changing charger mode\n", __func__);
			return ret;
		}
		break;

	default:
		printk("%s: Unable to set OTG1 chargermode (invalid mode %d)", __func__, mode);
		return -EINVAL;
	}

	otgc_data->otg1_chargermode = mode;
	return 0;
}
