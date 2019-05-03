/*
 * gpio-bd7181x.c
 * @file Access to GPOs on ROHM BD7181XMWV chip
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#define DEBUG
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/mfd/bd7181x.h>

/** @brief bd7181x gpio chip core data */
static struct gpio_chip bd7181xgpo_chip;

/** @brief get gpo output value
 * @param chip pointer to core data
 * @param offset gpo number, start from 0
 * @retval 0 success
 * @retval negative error number
 */
static int bd7181xgpo_get(struct gpio_chip *chip, unsigned offset)
{
	struct bd7181x *bd7181x = dev_get_drvdata(chip->parent);
	int ret = 0;

	ret = bd7181x_reg_read(bd7181x, BD7181X_REG_GPO);
	if (ret < 0)
		return ret;

	return (ret >> offset) & 1;
}

/** @brief set gpo direction as output
 * @param chip pointer to core data
 * @param offset gpo number, start from 0
 * @param value output value when set direction out
 * @retval 0 success
 */
static int bd7181xgpo_direction_out(struct gpio_chip *chip, unsigned offset,
				    int value)
{
	/* This only drives GPOs, and can't change direction */
	return 0;
}

/** @brief set gpo output value
 * @param chip pointer to core data
 * @param offset gpo number, start from 0
 * @param value output value, not zero as high level, 0 as low level
 * @retval 0 success
 * @retval negative error number
 */
static void bd7181xgpo_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct bd7181x *bd7181x = dev_get_drvdata(chip->parent);
	int ret;
	u8 gpoctl;

	ret = bd7181x_reg_read(bd7181x, BD7181X_REG_GPO);
	if (ret < 0)
		return;

	if (value)
		gpoctl = ret | (1 << offset);
	else
		gpoctl = ret & ~(1 << offset);

	bd7181x_reg_write(bd7181x, BD7181X_REG_GPO, gpoctl);
}

/** @brief bd7181x gpio chip core data */
static struct gpio_chip bd7181xgpo_chip = {
	.label			= "bd7181x",		///< gpio chip name
	.owner			= THIS_MODULE,
	.get			= bd7181xgpo_get,
	.direction_output	= bd7181xgpo_direction_out,
	.set			= bd7181xgpo_set,
	.can_sleep		= 1,
};

/*----------------------------------------------------------------------*/
#ifdef CONFIG_OF
/** @brief retrive gpo platform data from device tree
 * @param pdev platfrom device pointer
 * @return pointer to platform data
 * @retval NULL error
 */
static struct bd7181x_gpo_plat_data *of_gpio_bd7181x(
	struct platform_device *pdev)
{
	struct bd7181x_gpo_plat_data *platform_data;
	struct device_node *np, *gpio_np;

	platform_data = devm_kzalloc(&pdev->dev, sizeof(*platform_data), GFP_KERNEL);
	if (!platform_data) {
		return NULL;
	}

	np = of_node_get(pdev->dev.parent->of_node);
	gpio_np = of_find_node_by_name(np, "gpo");
	if (!gpio_np) {
		dev_err(&pdev->dev, "gpio node not found\n");
		return NULL;
	}

	pdev->dev.of_node = gpio_np;
	
	if (of_property_read_u32(gpio_np, "rohm,mode", &platform_data->mode)) {
		platform_data->mode = -1;
	}
	
	return platform_data;
}
#endif

/** @brief probe bd7181x gpo device
 * @param pdev platfrom device pointer
 * @retval 0 success
 * @retval negative error number
 */
static int gpo_bd7181x_probe(struct platform_device *pdev)
{
	struct bd7181x_gpo_plat_data *pdata = pdev->dev.platform_data;
	struct device *mfd_dev = pdev->dev.parent;
	struct bd7181x *bd7181x = dev_get_drvdata(mfd_dev);
	int ret;

#ifdef CONFIG_OF
	pdata = of_gpio_bd7181x(pdev);
#endif
	if (pdata && pdata->gpio_base > 0)
		bd7181xgpo_chip.base = pdata->gpio_base;
	else
		bd7181xgpo_chip.base = -1;

	bd7181xgpo_chip.ngpio = 2;	/* bd71815/bd71817 have 2 GPO */

	bd7181xgpo_chip.parent = &pdev->dev;

	ret = gpiochip_add(&bd7181xgpo_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		bd7181xgpo_chip.ngpio = 0;
		return ret;
	}

	if (pdata && pdata->mode != -1UL) {
		bd7181x_update_bits(bd7181x, BD7181X_REG_GPO, 0x70, pdata->mode);
	}

	return ret;
}

/** @brief remove bd7181x gpo device
 * @param pdev platfrom device pointer
 * @retval 0 success
 * @retval negative error number
 */
static int gpo_bd7181x_remove(struct platform_device *pdev)
{
	gpiochip_remove(&bd7181xgpo_chip);
	return 0;
}

/* Note:  this hardware lives inside an I2C-based multi-function device. */
MODULE_ALIAS("platform:bd7181x-gpo");

/** @brief bd7181x gpo driver core data */
static struct platform_driver gpo_bd7181x_driver = {
	.driver = {
		.name	= "bd7181x-gpo",
		.owner	= THIS_MODULE,
	},
	.probe		= gpo_bd7181x_probe,
	.remove		= gpo_bd7181x_remove,
};

module_platform_driver(gpo_bd7181x_driver);

MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("GPO interface for BD71815/BD71817");
MODULE_LICENSE("GPL");
