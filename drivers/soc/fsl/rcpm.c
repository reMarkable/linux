// SPDX-License-Identifier: GPL-2.0
//
// rcpm.c - Freescale QorIQ RCPM driver
//
// Copyright 2019 NXP
//
// Author: Ran Wang <ran.wang_1@nxp.com>

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#define RCPM_WAKEUP_CELL_MAX_SIZE	7

struct rcpm {
	unsigned int	wakeup_cells;
	void __iomem	*ippdexpcr_base;
	bool		little_endian;
};

/**
 * rcpm_pm_prepare - performs device-level tasks associated with power
 * management, such as programming related to the wakeup source control.
 * @dev: Device to handle.
 *
 */
static int rcpm_pm_prepare(struct device *dev)
{
	int i, ret, idx;
	void __iomem *base;
	struct wakeup_source	*ws;
	struct rcpm		*rcpm;
	struct device_node	*np = dev->of_node;
	u32 value[RCPM_WAKEUP_CELL_MAX_SIZE + 1];
	u32 setting[RCPM_WAKEUP_CELL_MAX_SIZE] = {0};
	struct regmap *scfg_addr_regmap = NULL;
	u32 reg_offset[RCPM_WAKEUP_CELL_MAX_SIZE + 1];
	u32 reg_value = 0;

	rcpm = dev_get_drvdata(dev);
	if (!rcpm)
		return -EINVAL;

	base = rcpm->ippdexpcr_base;
	idx = wakeup_sources_read_lock();

	/* Begin with first registered wakeup source */
	for_each_wakeup_source(ws) {

		/* skip object which is not attached to device */
		if (!ws->dev || !ws->dev->parent)
			continue;

		ret = device_property_read_u32_array(ws->dev->parent,
				"fsl,rcpm-wakeup", value,
				rcpm->wakeup_cells + 1);

		/*  Wakeup source should refer to current rcpm device */
		if (ret || (np->phandle != value[0]))
			continue;

		/* Property "#fsl,rcpm-wakeup-cells" of rcpm node defines the
		 * number of IPPDEXPCR register cells, and "fsl,rcpm-wakeup"
		 * of wakeup source IP contains an integer array: <phandle to
		 * RCPM node, IPPDEXPCR0 setting, IPPDEXPCR1 setting,
		 * IPPDEXPCR2 setting, etc>.
		 *
		 * So we will go thought them to collect setting data.
		 */
		for (i = 0; i < rcpm->wakeup_cells; i++)
			setting[i] |= value[i + 1];
	}

	wakeup_sources_read_unlock(idx);

	/* Program all IPPDEXPCRn once */
	for (i = 0; i < rcpm->wakeup_cells; i++) {
		u32 tmp = setting[i];
		void __iomem *address = base + i * 4;

		if (!tmp)
			continue;

		/* We can only OR related bits */
		if (rcpm->little_endian) {
			tmp |= ioread32(address);
			iowrite32(tmp, address);
		} else {
			tmp |= ioread32be(address);
			iowrite32be(tmp, address);
		}
		/*
		 * Workaround of errata A-008646 on SoC LS1021A:
		 * There is a bug of register ippdexpcr1.
		 * Reading configuration register RCPM_IPPDEXPCR1
		 * always return zero. So save ippdexpcr1's value
		 * to register SCFG_SPARECR8.And the value of
		 * ippdexpcr1 will be read from SCFG_SPARECR8.
		 */
		if (device_property_present(dev, "fsl,ippdexpcr1-alt-addr")) {
			if (dev_of_node(dev)) {
				scfg_addr_regmap = syscon_regmap_lookup_by_phandle(np,
										   "fsl,ippdexpcr1-alt-addr");
			} else if (is_acpi_node(dev->fwnode)) {
				dev_err(dev, "not support acpi for rcpm\n");
				continue;
			}

			if (scfg_addr_regmap && (i == 1)) {
				if (device_property_read_u32_array(dev,
				    "fsl,ippdexpcr1-alt-addr",
				    reg_offset,
				    1 + sizeof(u64)/sizeof(u32))) {
					scfg_addr_regmap = NULL;
					continue;
				}
				/* Read value from register SCFG_SPARECR8 */
				regmap_read(scfg_addr_regmap,
					    ((((u64)reg_offset[1] << (sizeof(u32) * 8) |
					    reg_offset[2])) & 0xffffffff),
					    &reg_value);
				/* Write value to register SCFG_SPARECR8 */
				regmap_write(scfg_addr_regmap,
					     ((((u64)reg_offset[1] << (sizeof(u32) * 8) |
					     reg_offset[2])) & 0xffffffff),
					     tmp | reg_value);
			}
		}
	}

	return 0;
}

static const struct dev_pm_ops rcpm_pm_ops = {
	.prepare =  rcpm_pm_prepare,
};

static int rcpm_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	struct resource *r;
	struct rcpm	*rcpm;
	int ret;

	rcpm = devm_kzalloc(dev, sizeof(*rcpm), GFP_KERNEL);
	if (!rcpm)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENODEV;

	rcpm->ippdexpcr_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(rcpm->ippdexpcr_base)) {
		ret =  PTR_ERR(rcpm->ippdexpcr_base);
		return ret;
	}

	rcpm->little_endian = device_property_read_bool(
			&pdev->dev, "little-endian");

	ret = device_property_read_u32(&pdev->dev,
			"#fsl,rcpm-wakeup-cells", &rcpm->wakeup_cells);
	if (ret)
		return ret;

	dev_set_drvdata(&pdev->dev, rcpm);

	return 0;
}

static const struct of_device_id rcpm_of_match[] = {
	{ .compatible = "fsl,qoriq-rcpm-2.1+", },
	{}
};
MODULE_DEVICE_TABLE(of, rcpm_of_match);

static struct platform_driver rcpm_driver = {
	.driver = {
		.name = "rcpm",
		.of_match_table = rcpm_of_match,
		.pm	= &rcpm_pm_ops,
	},
	.probe = rcpm_probe,
};

module_platform_driver(rcpm_driver);
