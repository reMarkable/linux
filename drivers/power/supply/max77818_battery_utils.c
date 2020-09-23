/*
 * Maxim MAX77818 Battery Utils
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Steinar Bakkemo <steinar.bakkemo@remarkable.com>
 * Author: Shawn Guo  <shawn.guo@linaro.org>
 * Author: Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/power/max77818_battery_utils.h>
#include <linux/power/max17042_battery.h>

#include <linux/device.h>
#include <linux/regmap.h>

/* Config register bits */
#define CONFIG_FGCC_BIT		(1 << 11)

/* Parameter to be given from command line in order to tune the delay introduced after
 * clearing the FGCC bit before forwarding requests to the charger driver */
static int post_fgcc_change_delay_us = 100000;

/* DO NOT CALL DIRECTLY !!
 *
 * ONLY TO _BE CALLED FROM MAX77818_DO_NON_FGCC_OP macro */
int max77818_utils_set_fgcc_mode(struct max77818_dev *max77818_dev,
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
