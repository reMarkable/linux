#include <linux/power/max77818_battery_utils.h>
#include <linux/power/max17042_battery.h>

#include <linux/device.h>
#include <linux/regmap.h>

/* Config register bits */
#define CONFIG_FGCC_BIT		(1 << 11)

/* Parameter to be given from command line in order to tune the delay introduced after
 * clearing the FGCC bit before forwarding requests to the charger driver */
static int post_fgcc_change_delay_us = 100000;
//module_param(post_fgcc_change_delay_us, int, 0644);
//MODULE_PARM_DESC(post_fgcc_change_delay_us,
//		 "Debug parameter used to tune the post FGCC change delay introduced "
//		  "to let the charger/fuelgauge take back charging control before doing "
//		  "any other configuration changes on either");

/* DO NOT CALL DIRECTLY !!
 *
 * ONLY TO _BE CALLED FROM MAX77818_DO_NON_FGCC_OP macro */
int max77818_set_fgcc_mode(struct max77818_dev *max77818_dev,
				  bool enabled,
				  bool *cur_mode)
{
	unsigned int read_data;
	int ret;

	if (cur_mode) {
		ret = regmap_read(max77818_dev->regmap_fg,
				  MAX17042_CONFIG, &read_data);
		if (ret) {
			dev_err(max77818_dev->dev,
				"Failed to read CONFIG register\n");
			return ret;
		}
		*cur_mode = (read_data & CONFIG_FGCC_BIT);
	}

	dev_dbg(max77818_dev->dev, "Turning %s FGCC\n", enabled ? "on" : "off");
	ret = regmap_update_bits(max77818_dev->regmap_fg,
				 MAX17042_CONFIG,
				 CONFIG_FGCC_BIT,
				 enabled ? CONFIG_FGCC_BIT : 0x0000);

	if (ret) {
		dev_err(max77818_dev->dev,
			"Failed to %s FGCC bit in CONFIG register\n",
			enabled ? "set" : "clear");
		return ret;
	}

	dev_dbg(max77818_dev->dev,
		"Waiting %d us after FGCC mode change..\n",
		post_fgcc_change_delay_us);
	usleep_range(post_fgcc_change_delay_us, post_fgcc_change_delay_us + 100000);

	return 0;
}
