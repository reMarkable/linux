// SPDX-License-Identifier: GPL-2.0
/*
 * Mobiveil PCIe Endpoint controller driver
 *
 * Copyright 2019 NXP
 * Author: Xiaowei Bao <xiaowei.bao@nxp.com>
 */

#include <linux/of.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/platform_device.h>
#include "pcie-mobiveil.h"

static void mobiveil_pcie_ep_func_select(struct mobiveil_pcie *pcie, u8 func_no)
{
	u32 func_num;

	/*
	 * select to access the config space of func_no by setting func_no
	 * to FUNC_SEL_SHIFT bit of PAB_CTRL register.
	 */
	func_num = csr_readl(pcie, PAB_CTRL);
	func_num &= ~(FUNC_SEL_MASK << FUNC_SEL_SHIFT);
	func_num |= (func_no & FUNC_SEL_MASK) << FUNC_SEL_SHIFT;
	csr_writel(pcie, func_num, PAB_CTRL);
}

static void mobiveil_pcie_ep_func_deselect(struct mobiveil_pcie *pcie)
{
	u32 func_num;

	/*
	 * clear the FUNC_SEL_SHIFT bits when access other registers except
	 * config space register.
	 */
	func_num = csr_readl(pcie, PAB_CTRL);
	func_num &= ~(FUNC_SEL_MASK << FUNC_SEL_SHIFT);
	csr_writel(pcie, func_num, PAB_CTRL);
}

static void __mobiveil_pcie_ep_reset_bar(struct mobiveil_pcie *pcie, u8 bar)
{
	csr_writel(pcie, bar, GPEX_BAR_SELECT);
	csr_writel(pcie, 0, GPEX_BAR_SIZE_LDW);
	csr_writel(pcie, 0, GPEX_BAR_SIZE_UDW);
}

void mobiveil_pcie_ep_reset_bar(struct mobiveil_pcie *pcie, u8 bar)
{
	__mobiveil_pcie_ep_reset_bar(pcie, bar);
}

static u8 __mobiveil_pcie_ep_find_next_cap(struct mobiveil_pcie *pcie,
					   u8 func_no, u8 cap_ptr, u8 cap)
{
	u8 cap_id, next_cap_ptr;
	u16 reg;

	if (!cap_ptr)
		return 0;

	mobiveil_pcie_ep_func_select(pcie, func_no);

	reg = csr_readw(pcie, cap_ptr);
	cap_id = (reg & 0x00ff);

	mobiveil_pcie_ep_func_deselect(pcie);

	if (cap_id > PCI_CAP_ID_MAX)
		return 0;

	if (cap_id == cap)
		return cap_ptr;

	next_cap_ptr = (reg & 0xff00) >> 8;
	return __mobiveil_pcie_ep_find_next_cap(pcie, func_no,
						next_cap_ptr, cap);
}

static u8 mobiveil_pcie_ep_find_capability(struct mobiveil_pcie_ep *ep,
					   u8 func_no, u8 cap)
{
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	u8 next_cap_ptr;
	u16 reg;

	mobiveil_pcie_ep_func_select(pcie, func_no);

	reg = csr_readw(pcie, PCI_CAPABILITY_LIST);
	next_cap_ptr = (reg & 0x00ff);

	mobiveil_pcie_ep_func_deselect(pcie);

	return __mobiveil_pcie_ep_find_next_cap(pcie, func_no,
						next_cap_ptr, cap);
}

static int mobiveil_pcie_ep_write_header(struct pci_epc *epc, u8 func_no,
					 struct pci_epf_header *hdr)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);

	mobiveil_pcie_ep_func_select(pcie, func_no);

	csr_writew(pcie, hdr->vendorid, PCI_VENDOR_ID);
	csr_writew(pcie, hdr->deviceid, PCI_DEVICE_ID);
	csr_writeb(pcie, hdr->revid, PCI_REVISION_ID);
	csr_writeb(pcie, hdr->progif_code, PCI_CLASS_PROG);
	csr_writew(pcie, hdr->subclass_code | hdr->baseclass_code << 8,
		   PCI_CLASS_DEVICE);
	csr_writeb(pcie, hdr->cache_line_size, PCI_CACHE_LINE_SIZE);
	csr_writew(pcie, hdr->subsys_vendor_id, PCI_SUBSYSTEM_VENDOR_ID);
	csr_writew(pcie, hdr->subsys_id, PCI_SUBSYSTEM_ID);
	csr_writeb(pcie, hdr->interrupt_pin, PCI_INTERRUPT_PIN);

	mobiveil_pcie_ep_func_deselect(pcie);

	return 0;
}

static void mobiveil_pcie_ep_inbound_win(struct mobiveil_pcie_ep *ep,
					 u8 func_no, enum pci_barno bar,
					 dma_addr_t cpu_addr)
{
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);

	program_ib_windows_ep(pcie, func_no, bar, cpu_addr);
}

static int mobiveil_pcie_ep_outbound_win(struct mobiveil_pcie_ep *ep,
					 phys_addr_t phys_addr,
					 u64 pci_addr, u8 func_no,
					 size_t size)
{
	u32 free_win;
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);

	free_win = find_first_zero_bit(ep->apio_wins_map, ep->apio_wins);
	if (free_win >= ep->apio_wins) {
		dev_err(&pcie->pdev->dev, "No free outbound window\n");
		return -EINVAL;
	}

	program_ob_windows_ep(pcie, func_no, free_win, phys_addr,
			      pci_addr, MEM_WINDOW_TYPE, size);

	set_bit(free_win, ep->apio_wins_map);
	ep->apio_addr[free_win] = phys_addr;

	return 0;
}

static void mobiveil_pcie_ep_clear_bar(struct pci_epc *epc, u8 func_no,
				       struct pci_epf_bar *epf_bar)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	enum pci_barno bar = epf_bar->barno;

	if (bar < ep->bar_num) {
		__mobiveil_pcie_ep_reset_bar(pcie, func_no * ep->bar_num + bar);

		mobiveil_pcie_disable_ib_win_ep(pcie, func_no, bar);
	}
}

static int mobiveil_pcie_ep_set_bar(struct pci_epc *epc, u8 func_no,
				    struct pci_epf_bar *epf_bar)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	enum pci_barno bar = epf_bar->barno;
	size_t size = epf_bar->size;

	if (bar < ep->bar_num) {
		mobiveil_pcie_ep_inbound_win(ep, func_no, bar,
					     epf_bar->phys_addr);

		csr_writel(pcie, func_no * ep->bar_num + bar,
			   GPEX_BAR_SELECT);
		csr_writel(pcie, lower_32_bits(~(size - 1)),
			   GPEX_BAR_SIZE_LDW);
		csr_writel(pcie, upper_32_bits(~(size - 1)),
			   GPEX_BAR_SIZE_UDW);
	}

	return 0;
}

static int mobiveil_pcie_find_index(struct mobiveil_pcie_ep *ep,
				    phys_addr_t addr,
				    u32 *atu_index)
{
	u32 index;

	for (index = 0; index < ep->apio_wins; index++) {
		if (ep->apio_addr[index] != addr)
			continue;
		*atu_index = index;
		return 0;
	}

	return -EINVAL;
}

static void mobiveil_pcie_ep_unmap_addr(struct pci_epc *epc, u8 func_no,
					phys_addr_t addr)
{
	int ret;
	u32 atu_index;
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);

	ret = mobiveil_pcie_find_index(ep, addr, &atu_index);
	if (ret < 0)
		return;

	mobiveil_pcie_disable_ob_win(pcie, atu_index);
	clear_bit(atu_index, ep->apio_wins_map);
}

static int mobiveil_pcie_ep_map_addr(struct pci_epc *epc, u8 func_no,
				     phys_addr_t addr,
				     u64 pci_addr, size_t size)
{
	int ret;
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);

	ret = mobiveil_pcie_ep_outbound_win(ep, addr, pci_addr, func_no, size);
	if (ret) {
		dev_err(&pcie->pdev->dev, "Failed to enable address\n");
		return ret;
	}

	return 0;
}

static int mobiveil_pcie_ep_get_msi(struct pci_epc *epc, u8 func_no)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	u32 val, reg;
	u8 msi_cap;

	msi_cap = mobiveil_pcie_ep_find_capability(ep, func_no,
						   PCI_CAP_ID_MSI);
	if (!msi_cap)
		return -EINVAL;

	mobiveil_pcie_ep_func_select(pcie, func_no);

	reg = msi_cap + PCI_MSI_FLAGS;
	val = csr_readw(pcie, reg);

	mobiveil_pcie_ep_func_deselect(pcie);

	if (!(val & PCI_MSI_FLAGS_ENABLE))
		return -EINVAL;

	val = (val & PCI_MSI_FLAGS_QSIZE) >> 4;

	return val;
}

static int mobiveil_pcie_ep_set_msi(struct pci_epc *epc,
				    u8 func_no, u8 interrupts)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	u32 val, reg;
	u8 msi_cap;

	msi_cap = mobiveil_pcie_ep_find_capability(ep, func_no,
						   PCI_CAP_ID_MSI);
	if (!msi_cap)
		return -EINVAL;

	mobiveil_pcie_ep_func_select(pcie, func_no);

	reg = msi_cap + PCI_MSI_FLAGS;
	val = csr_readw(pcie, reg);
	val &= ~PCI_MSI_FLAGS_QMASK;
	val |= (interrupts << 1) & PCI_MSI_FLAGS_QMASK;
	csr_writew(pcie, val, reg);

	mobiveil_pcie_ep_func_deselect(pcie);

	return 0;
}

static int mobiveil_pcie_ep_get_msix(struct pci_epc *epc, u8 func_no)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	u32 val, reg;
	u8 msix_cap;

	msix_cap = mobiveil_pcie_ep_find_capability(ep, func_no,
						    PCI_CAP_ID_MSIX);
	if (!msix_cap)
		return -EINVAL;

	mobiveil_pcie_ep_func_select(pcie, func_no);

	reg = msix_cap + PCI_MSIX_FLAGS;
	val = csr_readw(pcie, reg);

	mobiveil_pcie_ep_func_deselect(pcie);

	if (!(val & PCI_MSIX_FLAGS_ENABLE))
		return -EINVAL;

	val &= PCI_MSIX_FLAGS_QSIZE;

	return val;
}

static int mobiveil_pcie_ep_set_msix(struct pci_epc *epc, u8 func_no,
				     u16 interrupts)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	u32 val, reg;
	u8 msix_cap;

	msix_cap = mobiveil_pcie_ep_find_capability(ep, func_no,
						    PCI_CAP_ID_MSIX);
	if (!msix_cap)
		return -EINVAL;

	mobiveil_pcie_ep_func_select(pcie, func_no);

	reg = msix_cap + PCI_MSIX_FLAGS;
	val = csr_readw(pcie, reg);
	val &= ~PCI_MSIX_FLAGS_QSIZE;
	val |= interrupts;
	csr_writew(pcie, val, reg);

	mobiveil_pcie_ep_func_deselect(pcie);

	return 0;
}

static int mobiveil_pcie_ep_raise_irq(struct pci_epc *epc, u8 func_no,
				      enum pci_epc_irq_type type,
				      u16 interrupt_num)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);

	if (!ep->ops->raise_irq)
		return -EINVAL;

	return ep->ops->raise_irq(ep, func_no, type, interrupt_num);
}

static const struct pci_epc_features*
mobiveil_pcie_ep_get_features(struct pci_epc *epc, u8 func_no)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);

	if (!ep->ops->get_features)
		return NULL;

	return ep->ops->get_features(ep);
}

static const struct pci_epc_ops epc_ops = {
	.write_header		= mobiveil_pcie_ep_write_header,
	.set_bar		= mobiveil_pcie_ep_set_bar,
	.clear_bar		= mobiveil_pcie_ep_clear_bar,
	.map_addr		= mobiveil_pcie_ep_map_addr,
	.unmap_addr		= mobiveil_pcie_ep_unmap_addr,
	.set_msi		= mobiveil_pcie_ep_set_msi,
	.get_msi		= mobiveil_pcie_ep_get_msi,
	.set_msix		= mobiveil_pcie_ep_set_msix,
	.get_msix		= mobiveil_pcie_ep_get_msix,
	.raise_irq		= mobiveil_pcie_ep_raise_irq,
	.get_features		= mobiveil_pcie_ep_get_features,
};

int mobiveil_pcie_ep_raise_legacy_irq(struct mobiveil_pcie_ep *ep, u8 func_no)
{
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);

	dev_err(&pcie->pdev->dev, "EP cannot trigger legacy IRQs\n");

	return -EINVAL;
}

int mobiveil_pcie_ep_raise_msi_irq(struct mobiveil_pcie_ep *ep, u8 func_no,
				   u8 interrupt_num)
{
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	struct pci_epc *epc = ep->epc;
	u16 msg_ctrl, msg_data;
	u32 msg_addr_lower, msg_addr_upper, reg;
	u64 msg_addr;
	bool has_upper;
	int ret;
	u8 msi_cap;

	msi_cap = mobiveil_pcie_ep_find_capability(ep, func_no,
						   PCI_CAP_ID_MSI);
	if (!msi_cap)
		return -EINVAL;

	mobiveil_pcie_ep_func_select(pcie, func_no);

	reg = msi_cap + PCI_MSI_FLAGS;
	msg_ctrl = csr_readw(pcie, reg);
	has_upper = !!(msg_ctrl & PCI_MSI_FLAGS_64BIT);
	reg = msi_cap + PCI_MSI_ADDRESS_LO;
	msg_addr_lower = csr_readl(pcie, reg);
	if (has_upper) {
		reg = msi_cap + PCI_MSI_ADDRESS_HI;
		msg_addr_upper = csr_readl(pcie, reg);
		reg = msi_cap + PCI_MSI_DATA_64;
		msg_data = csr_readw(pcie, reg);
	} else {
		msg_addr_upper = 0;
		reg = msi_cap + PCI_MSI_DATA_32;
		msg_data = csr_readw(pcie, reg);
	}
	msg_addr = ((u64) msg_addr_upper) << 32 | msg_addr_lower;

	mobiveil_pcie_ep_func_deselect(pcie);

	ret = mobiveil_pcie_ep_map_addr(epc, func_no, ep->msi_mem_phys,
					msg_addr, epc->mem->page_size);
	if (ret)
		return ret;

	writel(msg_data | (interrupt_num - 1), ep->msi_mem);

	mobiveil_pcie_ep_unmap_addr(epc, func_no, ep->msi_mem_phys);

	return 0;
}

int mobiveil_pcie_ep_raise_msix_irq(struct mobiveil_pcie_ep *ep, u8 func_no,
				    u16 interrupt_num)
{
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	struct pci_epc *epc = ep->epc;
	u32 msg_addr_upper, msg_addr_lower;
	u32 msg_data;
	u64 msg_addr;
	u8 msix_cap;
	int ret;

	msix_cap = mobiveil_pcie_ep_find_capability(ep, func_no,
						   PCI_CAP_ID_MSIX);
	if (!msix_cap)
		return -EINVAL;

	mobiveil_pcie_ep_func_deselect(pcie);

	msg_addr_lower = csr_readl(pcie, PAB_MSIX_TABLE_PBA_ACCESS +
				   PCI_MSIX_ENTRY_LOWER_ADDR +
				   (interrupt_num - 1) * PCI_MSIX_ENTRY_SIZE);
	msg_addr_upper = csr_readl(pcie, PAB_MSIX_TABLE_PBA_ACCESS +
				   PCI_MSIX_ENTRY_UPPER_ADDR +
				   (interrupt_num - 1) * PCI_MSIX_ENTRY_SIZE);
	msg_addr = ((u64) msg_addr_upper) << 32 | msg_addr_lower;
	msg_data = csr_readl(pcie, PAB_MSIX_TABLE_PBA_ACCESS +
			     PCI_MSIX_ENTRY_DATA +
			     (interrupt_num - 1) * PCI_MSIX_ENTRY_SIZE);

	ret = mobiveil_pcie_ep_map_addr(epc, func_no, ep->msi_mem_phys,
					msg_addr, epc->mem->page_size);
	if (ret)
		return ret;

	writel(msg_data, ep->msi_mem);

	mobiveil_pcie_ep_unmap_addr(epc, func_no, ep->msi_mem_phys);

	return 0;
}

void mobiveil_pcie_ep_exit(struct mobiveil_pcie_ep *ep)
{
	struct pci_epc *epc = ep->epc;

	pci_epc_mem_free_addr(epc, ep->msi_mem_phys, ep->msi_mem,
			      epc->mem->page_size);

	pci_epc_mem_exit(epc);
}

int mobiveil_pcie_ep_init(struct mobiveil_pcie_ep *ep)
{
	int ret;
	void *addr;
	struct pci_epc *epc;
	struct mobiveil_pcie *pcie = to_mobiveil_pcie_from_ep(ep);
	struct device *dev = &pcie->pdev->dev;
	struct device_node *np = dev->of_node;

	if (!pcie->csr_axi_slave_base) {
		dev_err(dev, "csr_base is not populated\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "apio-wins", &ep->apio_wins);
	if (ret < 0) {
		dev_err(dev, "Unable to read apio-wins property\n");
		return ret;
	}

	if (ep->apio_wins > MAX_IATU_OUT) {
		dev_err(dev, "Invalid apio-wins\n");
		return -EINVAL;
	}
	ep->apio_wins_map = devm_kcalloc(dev,
					 BITS_TO_LONGS(ep->apio_wins),
					 sizeof(long),
					 GFP_KERNEL);
	if (!ep->apio_wins_map)
		return -ENOMEM;

	addr = devm_kcalloc(dev, ep->apio_wins, sizeof(phys_addr_t),
			    GFP_KERNEL);
	if (!addr)
		return -ENOMEM;

	ep->apio_addr = addr;

	mobiveil_pcie_enable_bridge_pio(pcie);
	mobiveil_pcie_enable_engine_apio(pcie);
	mobiveil_pcie_enable_engine_ppio(pcie);
	mobiveil_pcie_enable_msi_ep(pcie);

	epc = devm_pci_epc_create(dev, &epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "Failed to create epc device\n");
		return PTR_ERR(epc);
	}

	ep->epc = epc;
	epc_set_drvdata(epc, ep);

	ret = of_property_read_u8(np, "max-functions", &epc->max_functions);
	if (ret < 0)
		epc->max_functions = 1;

	if (ep->ops->ep_init)
		ep->ops->ep_init(ep);

	ret = __pci_epc_mem_init(epc, ep->phys_base, ep->addr_size,
				 ep->page_size);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize address space\n");
		return ret;
	}

	ep->msi_mem = pci_epc_mem_alloc_addr(epc, &ep->msi_mem_phys,
					     epc->mem->page_size);
	if (!ep->msi_mem) {
		dev_err(dev, "Failed to reserve memory for MSI/MSI-X\n");
		return -ENOMEM;
	}

	return 0;
}
