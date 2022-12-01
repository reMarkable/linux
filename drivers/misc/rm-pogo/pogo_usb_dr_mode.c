// SPDX-License-Identifier: GPL-2.0-only
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

#include "pogo_dr_mode.h"

#include <linux/extcon-provider.h>
#include <linux/kernel.h>
#include <linux/slab.h>

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

int pogo_init_extcon(struct rm_pogo_data *pdata)
{
	int ret;

	dev_dbg(pdata->dev,
		"%s: Allocating extcon device\n",
		__func__);

	pdata->extcon_dev = devm_extcon_dev_allocate(pdata->dev,
							 usb_extcon_cable);
	if (IS_ERR(pdata->extcon_dev)) {
		dev_err(pdata->dev,
			"%s: failed to allocate extcon device\n",
			__func__);

		return -ENOMEM;
	}

	dev_dbg(pdata->dev,
		"%s: Registering extcon device\n",
		__func__);

	ret = devm_extcon_dev_register(pdata->dev, pdata->extcon_dev);
	if (ret < 0) {
		dev_err(pdata->dev,
			"%s: Failed to register extcon device\n",
			__func__);

		dev_dbg(pdata->dev,
			"%s: De-allocating extcon device\n",
			__func__);

		kfree(pdata->extcon_dev);
		pdata->extcon_dev = NULL;
		return ret;
	}

	return 0;
}

int pogo_set_dr_mode(struct rm_pogo_data *pdata, int mode)
{
	int ret;

	switch(mode)
	{
	case POGO_OTG_DR_MODE__DEVICE:
		dev_dbg(pdata->dev,
			"%s: Switching POGO DR mode -> DEVICE\n",
			__func__);

		ret = extcon_set_state_sync(pdata->extcon_dev,
					    EXTCON_USB_HOST,
					    false);
		if (ret < 0) {
			dev_err(pdata->dev,
				"%s: Failed to set POGO DR mode\n",
				__func__);

			return ret;
		}
		break;

	case POGO_OTG_DR_MODE__HOST:
		dev_dbg(pdata->dev,
			"%s: Switching POGO DR mode -> HOST\n",
			__func__);

		pdata->pogo_dr_mode = mode;
		ret = extcon_set_state_sync(pdata->extcon_dev,
					    EXTCON_USB_HOST,
					    true);
		if (ret < 0) {
			dev_err(pdata->dev,
				"%s: Failed to set POGO DR mode\n",
				__func__);

			return ret;
		}
		break;

	default:
		dev_err(pdata->dev,
			"%s: unable to switch POGO DR mode (unknown mode %d)\n",
			__func__,
			mode);

		return -EINVAL;
	}

	pdata->pogo_dr_mode = mode;
	return ret;
}

int pogo_get_dr_mode(struct rm_pogo_data *pdata)
{
	/* Just return the last stored value */
	return pdata->pogo_dr_mode;
}
