/*
 * Functions to access SY3686A power management chip.
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
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

#ifndef __LINUX_MFD_SY7636A_H
#define __LINUX_MFD_SY7636A_H

#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>

#define SY7636A_REG_OPERATION_MODE_CRL 0x00
#define SY7636A_OPERATION_MODE_CRL_VCOMCTL (1 << 6)
#define SY7636A_OPERATION_MODE_CRL_ONOFF (1 << 7)
#define SY7636A_REG_VCOM_ADJUST_CTRL_L 0x01
#define SY7636A_REG_VCOM_ADJUST_CTRL_H 0x02
#define SY7636A_REG_VCOM_ADJUST_CTRL_MASK 0x01ff
#define SY7636A_REG_VLDO_VOLTAGE_ADJULST_CTRL 0x03
#define SY7636A_REG_POWER_ON_DELAY_TIME 0x06
#define SY7636A_REG_FAULT_FLAG 0x07
#define SY7636A_FAULT_FLAG_PG (1 << 0)
#define SY7636A_REG_TERMISTOR_READOUT 0x08

#define SY7636A_REG_MAX 0x08

struct sy7636a {
	struct device *dev;
	struct regmap *regmap;
	unsigned int vcom;
	struct gpio_desc *pgood_gpio;
	int pgood_irq;
};

int get_vcom_voltage_mv(struct regmap *regmap);
int set_vcom_voltage_mv(struct regmap *regmap, unsigned int vcom);

#endif /* __LINUX_MFD_SY7636A_H */
