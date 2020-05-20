/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2020 NXP */

#ifndef _CLK_IMX_ACM_UTILS_H
#define _CLK_IMX_ACM_UTILS_H

#include <linux/device.h>

struct clk_imx_acm_pm_domains {
	struct device **pd_dev;
	struct device_link **pd_dev_link;
	int    num_domains;
};

int clk_imx_acm_attach_pm_domains(struct device *dev,
				  struct clk_imx_acm_pm_domains *dev_pm);
int clk_imx_acm_detach_pm_domains(struct device *dev,
				  struct clk_imx_acm_pm_domains *dev_pm);
#endif /* _CLK_IMX_ACM_UTILS_H */
