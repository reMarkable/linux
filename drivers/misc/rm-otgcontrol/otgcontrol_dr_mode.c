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

#include "otgcontrol_dr_mode.h"

#include <linux/extcon-provider.h>
#include <linux/kernel.h>
#include <linux/slab.h>

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

int otgcontrol_init_extcon(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	dev_dbg(otgc_data->dev,
		"%s: Allocating extcon device\n",
		__func__);

	otgc_data->extcon_dev = devm_extcon_dev_allocate(otgc_data->dev,
							 usb_extcon_cable);
	if (IS_ERR(otgc_data->extcon_dev)) {
		dev_err(otgc_data->dev,
			"%s: failed to allocate extcon device\n",
			__func__);

		return -ENOMEM;
	}

	dev_dbg(otgc_data->dev,
		"%s: Registering extcon device\n",
		__func__);

	ret = devm_extcon_dev_register(otgc_data->dev, otgc_data->extcon_dev);
	if (ret < 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to register extcon device\n",
			__func__);

		dev_dbg(otgc_data->dev,
			"%s: De-allocating extcon device\n",
			__func__);

		kfree(otgc_data->extcon_dev);
		otgc_data->extcon_dev = NULL;
		return ret;
	}

	return 0;
}

int otgcontrol_set_dr_mode(struct rm_otgcontrol_data *otgc_data, int mode)
{
	int ret;

	switch(mode)
	{
	case OTG1_DR_MODE__DEVICE:
		dev_dbg(otgc_data->dev,
			"%s: Switching OTG1 DR mode -> DEVICE\n",
			__func__);

		ret = extcon_set_state_sync(otgc_data->extcon_dev,
					    EXTCON_USB_HOST,
					    false);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to set OTG1 DR mode\n",
				__func__);

			return ret;
		}
		break;

	case OTG1_DR_MODE__HOST:
		dev_dbg(otgc_data->dev,
			"%s: Switching OTG1 DR mode -> HOST\n",
			__func__);

		otgc_data->otg1_dr_mode = mode;
		ret = extcon_set_state_sync(otgc_data->extcon_dev,
					    EXTCON_USB_HOST,
					    true);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to set OTG1 DR mode\n",
				__func__);

			return ret;
		}
		break;

	default:
		dev_err(otgc_data->dev,
			"%s: unable to switch OTG1 DR mode (unknown mode %d)\n",
			__func__,
			mode);

		return -EINVAL;
	}

	otgc_data->otg1_dr_mode = mode;
	return ret;
}

int otgcontrol_get_dr_mode(struct rm_otgcontrol_data *otgc_data)
{
	/* Just return the last stored value */
	return otgc_data->otg1_dr_mode;
}
