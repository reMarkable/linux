/*
 * linux/power/max77818-regulator.h
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#ifndef __LINUX_MAX77818_REGULATOR_H
#define __LINUX_MAX77818_REGULATOR_H

struct max77818_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};


/* MAX77843 regulator IDs */
enum max77818_regulators {
	MAX77818_SAFEOUT1 = 0,
	MAX77818_SAFEOUT2,
	MAX77818_REG_MAX,
};

struct max77818_regulator_platform_data
{
	struct max77818_regulator_data *regulators;
	int num_regulators;	
};

extern struct max77818_regulator_data max77818_regulators[];
#endif
