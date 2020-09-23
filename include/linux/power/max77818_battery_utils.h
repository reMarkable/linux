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

#ifndef __MAX17818_BATTERY_UTILS_H_
#define __MAX17818_BATTERY_UTILS_H_

#include <linux/mfd/max77818/max77818.h>

/* Exported function required for modules external to the max77818_battery
 * module to be able to use the MAX77818_DO_NON_FGCC_OP macrov*/
int max77818_utils_set_fgcc_mode(struct max77818_dev *max77818_dev,
				 bool enabled,
				 bool *cur_mode);

/* Magic to enable optional macro param */
#define VARGS_(_10, _9, _8, _7, _6, _5, _4, _3, _2, _1, N, ...) N
#define VARGS(...) VARGS_(__VA_ARGS__, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

#define CONCAT_(a, b) a##b
#define CONCAT(a, b) CONCAT_(a, b)

/* Common macro to be user from any context having access to the common
 * max77818 struct defined in the max77818 MDF driver */
#define MAX77818_START_NON_FGCC_OP_3(max77818_dev, fgcc_restore_state, op_description) ( \
{ \
	int ret = 0; \
	bool restore_state = 0; \
\
	if (!max77818_dev) { \
		printk("%s: max77818_dev is NULL in MAX77818_DO_NON_FGCC_OP\n", __func__); \
		ret = -EINVAL; \
	} \
	else { \
		dev_dbg(max77818_dev->dev, op_description); \
\
		dev_dbg(max77818_dev->dev, "Applying lock\n"); \
		mutex_lock(&max77818_dev->lock); \
\
		dev_dbg(max77818_dev->dev, "Clearing FGCC mode\n"); \
		ret = max77818_utils_set_fgcc_mode(max77818_dev, \
						   false, \
						   &restore_state); \
		if (ret) { \
			dev_err(max77818_dev->dev, \
				"Failed to clear FGCC bit in CONFIG register\n"); \
		} \
		else { \
			fgcc_restore_state = restore_state; \
		} \
\
		/* UNLOCKING IS DONE IN MAX77818_FINISH_NON_FGCC_OP */ \
	} \
	ret; \
})
#define MAX77818_START_NON_FGCC_OP_2(max77818_dev, fgcc_restore_state) MAX77818_START_NON_FGCC_OP_3(max77818_dev, fgcc_restore_state, "")
#define MAX77818_START_NON_FGCC_OP(...) ( CONCAT(MAX77818_START_NON_FGCC_OP_, VARGS(__VA_ARGS__))(__VA_ARGS__) )

/* Common macro to be user from any context having access to the common
 * max77818 struct defined in the max77818 MDF driver */
#define MAX77818_FINISH_NON_FGCC_OP_3(max77818_dev, fgcc_restore_state, op_description) ( \
{ \
	int ret = 0; \
\
	if (!max77818_dev) { \
		printk("%s: max77818_dev is NULL in MAX77818_DO_NON_FGCC_OP\n", __func__); \
		ret = -EINVAL; \
	} \
	else { \
		dev_dbg(max77818_dev->dev, op_description); \
\
		if (fgcc_restore_state) { \
			dev_dbg(max77818_dev->dev, "Restoring FGCC mode\n"); \
\
			ret = max77818_utils_set_fgcc_mode(max77818_dev, \
							  true, \
							  NULL); \
			if (ret) { \
				dev_err(max77818_dev->dev, \
					"Failed to set FGCC bit in CONFIG register\n"); \
			} \
		} \
		else { \
			dev_dbg(max77818_dev->dev, \
				"Leaving FGCC bit as it were (OFF)\n"); \
		} \
		dev_dbg(max77818_dev->dev, "Releasing lock\n"); \
		mutex_unlock(&max77818_dev->lock); \
	} \
	ret; \
})
#define MAX77818_FINISH_NON_FGCC_OP_2(max77818_dev, fgcc_restore_state) MAX77818_FINISH_NON_FGCC_OP_3(max77818_dev, fgcc_restore_state, "")
#define MAX77818_FINISH_NON_FGCC_OP(...) ( CONCAT(MAX77818_FINISH_NON_FGCC_OP_, VARGS(__VA_ARGS__))(__VA_ARGS__) )

/* Common macro to be used from any context having access to the common
 * max77818 struct defined in the max77818 MFD driver */
#define MAX77818_DO_NON_FGCC_OP_3(max77818_dev, op, op_description) ( \
{ \
	int ret = 0; \
	bool restore_state = 0; \
\
	if (!max77818_dev) { \
		printk("%s: max77818_dev is NULL in MAX77818_DO_NON_FGCC_OP\n", __func__); \
		ret = -EINVAL; \
	} \
	else { \
		dev_dbg(max77818_dev->dev, "Applying lock\n"); \
		mutex_lock(&max77818_dev->lock); \
\
		dev_dbg(max77818_dev->dev, "Clearing FGCC mode\n"); \
\
		ret = max77818_utils_set_fgcc_mode(max77818_dev, \
						   false, \
						   &restore_state); \
		if (ret) { \
			dev_err(max77818_dev->dev, \
				"Failed to clear FGCC bit in CONFIG register\n"); \
		} \
		else { \
			dev_dbg(max77818_dev->dev, op_description); \
			ret = op; \
\
			if (ret) { \
				dev_err(max77818_dev->dev, \
					"Failed to read charger mode from charger driver\n"); \
			} \
			else { \
				if (restore_state) { \
					dev_dbg(max77818_dev->dev, "Restoring FGCC mode\n"); \
\
					ret = max77818_utils_set_fgcc_mode( \
						max77818_dev, true, NULL); \
					if (ret) { \
						dev_err(max77818_dev->dev, \
							"Failed to set FGCC bit in CONFIG register\n"); \
					} \
				} \
				else { \
					dev_dbg(max77818_dev->dev, \
						"Leaving FGCC bit as it were (OFF)\n"); \
				} \
			} \
		} \
		dev_dbg(max77818_dev->dev, "Releasing lock\n"); \
		mutex_unlock(&max77818_dev->lock); \
	} \
	ret; \
})
#define MAX77818_DO_NON_FGCC_OP_2(max77818_dev, op) MAX77818_DO_NON_FGCC_OP_3(max77818_dev, op, "")
#define MAX77818_DO_NON_FGCC_OP(...) ( CONCAT(MAX77818_DO_NON_FGCC_OP_, VARGS(__VA_ARGS__))(__VA_ARGS__) )

#endif /* __MAX17818_BATTERY_UTILS_H_ */
