// SPDX-License-Identifier: GPL-2.0+
// Copyright 2020 NXP

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include "clk-imx-acm-utils.h"

/**
 * clk_imx_acm_attach_pm_domains
 */
int clk_imx_acm_attach_pm_domains(struct device *dev,
				  struct clk_imx_acm_pm_domains *dev_pm)
{
	int ret;
	int i;

	dev_pm->num_domains = of_count_phandle_with_args(dev->of_node, "power-domains",
							 "#power-domain-cells");
	if (dev_pm->num_domains <= 1)
		return 0;

	dev_pm->pd_dev = devm_kmalloc_array(dev, dev_pm->num_domains,
					    sizeof(*dev_pm->pd_dev),
					    GFP_KERNEL);
	if (!dev_pm->pd_dev)
		return -ENOMEM;

	dev_pm->pd_dev_link = devm_kmalloc_array(dev,
						 dev_pm->num_domains,
						 sizeof(*dev_pm->pd_dev_link),
						 GFP_KERNEL);
	if (!dev_pm->pd_dev_link)
		return -ENOMEM;

	for (i = 0; i < dev_pm->num_domains; i++) {
		dev_pm->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(dev_pm->pd_dev[i]))
			return PTR_ERR(dev_pm->pd_dev[i]);

		dev_pm->pd_dev_link[i] = device_link_add(dev,
							 dev_pm->pd_dev[i],
							 DL_FLAG_STATELESS |
							 DL_FLAG_PM_RUNTIME |
							 DL_FLAG_RPM_ACTIVE);
		if (IS_ERR(dev_pm->pd_dev_link[i])) {
			dev_pm_domain_detach(dev_pm->pd_dev[i], false);
			ret = PTR_ERR(dev_pm->pd_dev_link[i]);
			goto detach_pm;
		}
	}
	return 0;

detach_pm:
	while (--i >= 0) {
		device_link_del(dev_pm->pd_dev_link[i]);
		dev_pm_domain_detach(dev_pm->pd_dev[i], false);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(clk_imx_acm_attach_pm_domains);

/**
 * fsl_dev_detach_pm_domains
 */
int clk_imx_acm_detach_pm_domains(struct device *dev,
				  struct clk_imx_acm_pm_domains *dev_pm)
{
	int i;

	if (dev_pm->num_domains <= 1)
		return 0;

	for (i = 0; i < dev_pm->num_domains; i++) {
		device_link_del(dev_pm->pd_dev_link[i]);
		dev_pm_domain_detach(dev_pm->pd_dev[i], false);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(clk_imx_acm_detach_pm_domains);
MODULE_LICENSE("GPL v2");
