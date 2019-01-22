// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe controller EP driver for Freescale Layerscape SoCs
 *
 * Copyright 2019 NXP
 *
 * Author: Xiaowei Bao <xiaowei.bao@nxp.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>

#include "pcie-mobiveil.h"

#define PCIE_LX2_BAR_NUM	4

#define to_ls_pcie_g4_ep(x)	dev_get_drvdata((x)->dev)

struct ls_pcie_g4_ep {
	struct mobiveil_pcie		*mv_pci;
};

static const struct of_device_id ls_pcie_g4_ep_of_match[] = {
	{ .compatible = "fsl,lx2160a-pcie-ep",},
	{ },
};

static const struct pci_epc_features ls_pcie_g4_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = true,
	.reserved_bar = (1 << BAR_4) | (1 << BAR_5),
};

static const struct pci_epc_features*
ls_pcie_g4_ep_get_features(struct mobiveil_pcie_ep *ep)
{
	return &ls_pcie_g4_epc_features;
}

static void ls_pcie_g4_ep_init(struct mobiveil_pcie_ep *ep)
{
	struct mobiveil_pcie *mv_pci = to_mobiveil_pcie_from_ep(ep);
	int win_idx;
	u8 bar;
	u32 val;

	/*
	 * Errata: unsupported request error on inbound posted write
	 * transaction, PCIe controller reports advisory error instead
	 * of uncorrectable error message to RC.
	 * workaround: set the bit20(unsupported_request_Error_severity) with
	 * value 1 in uncorrectable_Error_Severity_Register, make the
	 * unsupported request error generate the fatal error.
	 */
	val =  csr_readl(mv_pci, CFG_UNCORRECTABLE_ERROR_SEVERITY);
	val |= 1 << UNSUPPORTED_REQUEST_ERROR_SHIFT;
	csr_writel(mv_pci, val, CFG_UNCORRECTABLE_ERROR_SEVERITY);

	ep->bar_num = PCIE_LX2_BAR_NUM;

	for (bar = BAR_0; bar < ep->epc->max_functions * ep->bar_num; bar++)
		mobiveil_pcie_ep_reset_bar(mv_pci, bar);

	for (win_idx = 0; win_idx < ep->apio_wins; win_idx++)
		mobiveil_pcie_disable_ob_win(mv_pci, win_idx);
}

static int ls_pcie_g4_ep_raise_irq(struct mobiveil_pcie_ep *ep, u8 func_no,
				   enum pci_epc_irq_type type,
				   u16 interrupt_num)
{
	struct mobiveil_pcie *mv_pci = to_mobiveil_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return mobiveil_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return mobiveil_pcie_ep_raise_msi_irq(ep, func_no,
						      interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return mobiveil_pcie_ep_raise_msix_irq(ep, func_no,
						       interrupt_num);
	default:
		dev_err(&mv_pci->pdev->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static const struct mobiveil_pcie_ep_ops pcie_ep_ops = {
	.ep_init = ls_pcie_g4_ep_init,
	.raise_irq = ls_pcie_g4_ep_raise_irq,
	.get_features = ls_pcie_g4_ep_get_features,
};

static int __init ls_pcie_gen4_add_pcie_ep(struct ls_pcie_g4_ep *ls_ep,
					   struct platform_device *pdev)
{
	struct mobiveil_pcie *mv_pci = ls_ep->mv_pci;
	struct device *dev = &pdev->dev;
	struct mobiveil_pcie_ep *ep;
	struct resource *res;
	int ret;

	ep = &mv_pci->ep;
	ep->ops = &pcie_ep_ops;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	ret = mobiveil_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize layerscape endpoint\n");
		return ret;
	}

	return 0;
}

static int __init ls_pcie_g4_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mobiveil_pcie *mv_pci;
	struct ls_pcie_g4_ep *ls_ep;
	struct resource *res;
	int ret;

	ls_ep = devm_kzalloc(dev, sizeof(*ls_ep), GFP_KERNEL);
	if (!ls_ep)
		return -ENOMEM;

	mv_pci = devm_kzalloc(dev, sizeof(*mv_pci), GFP_KERNEL);
	if (!mv_pci)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	mv_pci->csr_axi_slave_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(mv_pci->csr_axi_slave_base))
		return PTR_ERR(mv_pci->csr_axi_slave_base);

	mv_pci->pdev = pdev;
	ls_ep->mv_pci = mv_pci;

	platform_set_drvdata(pdev, ls_ep);

	ret = ls_pcie_gen4_add_pcie_ep(ls_ep, pdev);

	return ret;
}

static struct platform_driver ls_pcie_g4_ep_driver = {
	.driver = {
		.name = "layerscape-pcie-gen4-ep",
		.of_match_table = ls_pcie_g4_ep_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver_probe(ls_pcie_g4_ep_driver, ls_pcie_g4_ep_probe);
