/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe host controller driver for Mobiveil PCIe Host controller
 *
 * Copyright (c) 2018 Mobiveil Inc.
 * Copyright 2019 NXP
 *
 * Author: Subrahmanya Lingappa <l.subrahmanya@mobiveil.co.in>
 * Refactor: Zhiqiang Hou <Zhiqiang.Hou@nxp.com>
 */

#ifndef _PCIE_MOBIVEIL_H
#define _PCIE_MOBIVEIL_H

#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

#include "../../pci.h"

#define MAX_IATU_OUT			256
/* register offsets and bit positions */

/*
 * translation tables are grouped into windows, each window registers are
 * grouped into blocks of 4 or 16 registers each
 */
#define PAB_REG_BLOCK_SIZE		16
#define PAB_EXT_REG_BLOCK_SIZE		4

#define PAB_REG_ADDR(offset, win)	\
	(offset + (win * PAB_REG_BLOCK_SIZE))
#define PAB_EXT_REG_ADDR(offset, win)	\
	(offset + (win * PAB_EXT_REG_BLOCK_SIZE))

#define LTSSM_STATUS			0x0404
#define  LTSSM_STATUS_L0_MASK		0x3f
#define  LTSSM_STATUS_L0		0x2d

#define PAB_CTRL			0x0808
#define  AMBA_PIO_ENABLE_SHIFT		0
#define  PEX_PIO_ENABLE_SHIFT		1
#define  PAGE_SEL_SHIFT			13
#define  PAGE_SEL_MASK			0x3f
#define  PAGE_LO_MASK			0x3ff
#define  PAGE_SEL_OFFSET_SHIFT		10
#define  FUNC_SEL_SHIFT			19
#define  FUNC_SEL_MASK			0x1ff
#define  MSI_SW_CTRL_EN			BIT(29)

#define PAB_ACTIVITY_STAT		0x81c

#define PAB_AXI_PIO_CTRL		0x0840
#define  APIO_EN_MASK			0xf

#define PAB_PEX_PIO_CTRL		0x08c0
#define  PIO_ENABLE_SHIFT		0

#define PAB_INTP_AMBA_MISC_ENB		0x0b0c
#define  PAB_INTP_PAMR			BIT(0)
#define PAB_INTP_AMBA_MISC_STAT		0x0b1c
#define  PAB_INTP_RESET			BIT(1)
#define  PAB_INTP_MSI			BIT(3)
#define  PAB_INTP_INTA			BIT(5)
#define  PAB_INTP_INTB			BIT(6)
#define  PAB_INTP_INTC			BIT(7)
#define  PAB_INTP_INTD			BIT(8)
#define  PAB_INTP_PCIE_UE		BIT(9)
#define  PAB_INTP_IE_PMREDI		BIT(29)
#define  PAB_INTP_IE_EC			BIT(30)
#define  PAB_INTP_MSI_MASK		PAB_INTP_MSI
#define  PAB_INTP_INTX_MASK		(PAB_INTP_INTA | PAB_INTP_INTB |\
					PAB_INTP_INTC | PAB_INTP_INTD)

#define PAB_AXI_AMAP_CTRL(win)		PAB_REG_ADDR(0x0ba0, win)
#define  WIN_ENABLE_SHIFT		0
#define  WIN_TYPE_SHIFT			1
#define  WIN_TYPE_MASK			0x3
#define  WIN_SIZE_MASK			0xfffffc00

#define PAB_AXI_AMAP_PCI_HDR_PARAM(win)	PAB_EXT_REG_ADDR(0x5ba0, win)

#define PAB_EXT_AXI_AMAP_SIZE(win)	PAB_EXT_REG_ADDR(0xbaf0, win)

#define PAB_EXT_AXI_AMAP_AXI_WIN(win)	PAB_EXT_REG_ADDR(0x80a0, win)
#define PAB_AXI_AMAP_AXI_WIN(win)	PAB_REG_ADDR(0x0ba4, win)
#define  AXI_WINDOW_ALIGN_MASK		3

#define PAB_AXI_AMAP_PEX_WIN_L(win)	PAB_REG_ADDR(0x0ba8, win)
#define  PAB_BUS_SHIFT			24
#define  PAB_DEVICE_SHIFT		19
#define  PAB_FUNCTION_SHIFT		16

#define PAB_AXI_AMAP_PEX_WIN_H(win)	PAB_REG_ADDR(0x0bac, win)
#define PAB_INTP_AXI_PIO_CLASS		0x474

#define GPEX_ACK_REPLAY_TO		0x438
#define  ACK_LAT_TO_VAL_MASK		0x1fff
#define  ACK_LAT_TO_VAL_SHIFT		0

#define PAB_PEX_AMAP_CTRL(win)		PAB_REG_ADDR(0x4ba0, win)
#define  AMAP_CTRL_EN_SHIFT		0
#define  AMAP_CTRL_TYPE_SHIFT		1
#define  AMAP_CTRL_TYPE_MASK		3

#define PAB_EXT_PEX_AMAP_SIZEN(win)	PAB_EXT_REG_ADDR(0xbef0, win)
#define PAB_EXT_PEX_AMAP_AXI_WIN(win)	PAB_EXT_REG_ADDR(0xb4a0, win)
#define PAB_PEX_AMAP_AXI_WIN(win)	PAB_REG_ADDR(0x4ba4, win)
#define PAB_PEX_AMAP_PEX_WIN_L(win)	PAB_REG_ADDR(0x4ba8, win)
#define PAB_PEX_AMAP_PEX_WIN_H(win)	PAB_REG_ADDR(0x4bac, win)

/* PPIO WINs EP mode */
#define PAB_PEX_BAR_AMAP(func, bar)	(0x1ba0 + 0x20 * func + 4 * bar)
#define PAB_EXT_PEX_BAR_AMAP(func, bar)	(0x84a0 + 0x20 * func + 4 * bar)
#define PEX_BAR_AMAP_EN			BIT(0)

#define PAB_MSIX_TABLE_PBA_ACCESS	0xD000

#define GPEX_BAR_ENABLE			0x4D4
#define GPEX_BAR_SIZE_LDW		0x4D8
#define GPEX_BAR_SIZE_UDW		0x4DC
#define GPEX_BAR_SELECT			0x4E0

#define CFG_UNCORRECTABLE_ERROR_SEVERITY	0x10c
#define UNSUPPORTED_REQUEST_ERROR_SHIFT		20
#define CFG_UNCORRECTABLE_ERROR_MASK		0x108

/* starting offset of INTX bits in status register */
#define PAB_INTX_START			5

/* supported number of MSI interrupts */
#define PCI_NUM_MSI			16

/* MSI registers */
#define MSI_BASE_LO_OFFSET		0x04
#define MSI_BASE_HI_OFFSET		0x08
#define MSI_SIZE_OFFSET			0x0c
#define MSI_ENABLE_OFFSET		0x14
#define MSI_STATUS_OFFSET		0x18
#define MSI_DATA_OFFSET			0x20
#define MSI_ADDR_L_OFFSET		0x24
#define MSI_ADDR_H_OFFSET		0x28

/* outbound and inbound window definitions */
#define WIN_NUM_0			0
#define WIN_NUM_1			1
#define CFG_WINDOW_TYPE			0
#define IO_WINDOW_TYPE			1
#define MEM_WINDOW_TYPE			2
#define IB_WIN_SIZE			((u64)256 * 1024 * 1024 * 1024)
#define MAX_PIO_WINDOWS			8

/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIES		10
#define LINK_WAIT_MIN			90000
#define LINK_WAIT_MAX			100000

#define PAGED_ADDR_BNDRY		0xc00
#define OFFSET_TO_PAGE_ADDR(off)	\
	((off & PAGE_LO_MASK) | PAGED_ADDR_BNDRY)
#define OFFSET_TO_PAGE_IDX(off)		\
	((off >> PAGE_SEL_OFFSET_SHIFT) & PAGE_SEL_MASK)

struct mobiveil_pcie;
struct mobiveil_pcie_ep;

struct mobiveil_msi {			/* MSI information */
	struct mutex lock;		/* protect bitmap variable */
	struct irq_domain *msi_domain;
	struct irq_domain *dev_domain;
	phys_addr_t msi_pages_phys;
	int num_of_vectors;
	DECLARE_BITMAP(msi_irq_in_use, PCI_NUM_MSI);
};

struct mobiveil_rp_ops {
	int (*interrupt_init)(struct mobiveil_pcie *pcie);
	int (*read_other_conf)(struct pci_bus *bus, unsigned int devfn,
			       int where, int size, u32 *val);
};

struct root_port {
	u8 root_bus_nr;
	void __iomem *config_axi_slave_base;	/* endpoint config base */
	struct resource *ob_io_res;
	struct mobiveil_rp_ops *ops;
	int irq;
	raw_spinlock_t intx_mask_lock;
	struct irq_domain *intx_domain;
	struct mobiveil_msi msi;
};

struct mobiveil_pab_ops {
	int (*link_up)(struct mobiveil_pcie *pcie);
	int (*host_init)(struct mobiveil_pcie *pcie);
};

struct mobiveil_pcie_ep_ops {
	void (*ep_init)(struct mobiveil_pcie_ep *ep);
	int (*raise_irq)(struct mobiveil_pcie_ep *ep, u8 func_no,
			 enum pci_epc_irq_type type, u16 interrupt_num);
	const struct pci_epc_features* (*get_features)
				       (struct mobiveil_pcie_ep *ep);
};

struct mobiveil_pcie_ep {
	struct pci_epc *epc;
	const struct mobiveil_pcie_ep_ops *ops;
	phys_addr_t phys_base;
	size_t addr_size;
	size_t page_size;
	phys_addr_t *apio_addr;
	unsigned long *apio_wins_map;
	u32 apio_wins;
	void __iomem *msi_mem;
	phys_addr_t msi_mem_phys;
	u8 bar_num;
};

struct mobiveil_pcie {
	struct platform_device *pdev;
	struct list_head *resources;
	void __iomem *csr_axi_slave_base;	/* PAB registers base */
	phys_addr_t pcie_reg_base;	/* Physical PCIe Controller Base */
	void __iomem *apb_csr_base;	/* MSI register base */
	u32 apio_wins;
	u32 ppio_wins;
	u32 ob_wins_configured;		/* configured outbound windows */
	u32 ib_wins_configured;		/* configured inbound windows */
	const struct mobiveil_pab_ops *ops;
	struct root_port rp;
	struct pci_host_bridge *bridge;
	struct mobiveil_pcie_ep ep;
};

#define to_mobiveil_pcie_from_ep(endpoint)   \
			    container_of((endpoint), struct mobiveil_pcie, ep)

int mobiveil_pcie_host_probe(struct mobiveil_pcie *pcie);
int mobiveil_host_init(struct mobiveil_pcie *pcie, bool reinit);
bool mobiveil_pcie_link_up(struct mobiveil_pcie *pcie);
int mobiveil_bringup_link(struct mobiveil_pcie *pcie);
void program_ob_windows(struct mobiveil_pcie *pcie, int win_num, u64 cpu_addr,
			u64 pci_addr, u32 type, u64 size);
void program_ib_windows(struct mobiveil_pcie *pcie, int win_num, u64 cpu_addr,
			u64 pci_addr, u32 type, u64 size);
u32 csr_read(struct mobiveil_pcie *pcie, u32 off, size_t size);
void csr_write(struct mobiveil_pcie *pcie, u32 val, u32 off, size_t size);

static inline u32 csr_readl(struct mobiveil_pcie *pcie, u32 off)
{
	return csr_read(pcie, off, 0x4);
}

static inline u32 csr_readw(struct mobiveil_pcie *pcie, u32 off)
{
	return csr_read(pcie, off, 0x2);
}

static inline u32 csr_readb(struct mobiveil_pcie *pcie, u32 off)
{
	return csr_read(pcie, off, 0x1);
}

static inline void csr_writel(struct mobiveil_pcie *pcie, u32 val, u32 off)
{
	csr_write(pcie, val, off, 0x4);
}

static inline void csr_writew(struct mobiveil_pcie *pcie, u32 val, u32 off)
{
	csr_write(pcie, val, off, 0x2);
}

static inline void csr_writeb(struct mobiveil_pcie *pcie, u32 val, u32 off)
{
	csr_write(pcie, val, off, 0x1);
}

void program_ib_windows_ep(struct mobiveil_pcie *pcie, u8 func_no,
			   int bar, u64 phys);
void program_ob_windows_ep(struct mobiveil_pcie *pcie, u8 func_num, int win_num,
			   u64 cpu_addr, u64 pci_addr, u32 type, u64 size);
void mobiveil_pcie_disable_ib_win_ep(struct mobiveil_pcie *pci,
				     u8 func_no, u8 bar);
void mobiveil_pcie_disable_ob_win(struct mobiveil_pcie *pcie, int win_num);
int mobiveil_pcie_ep_init(struct mobiveil_pcie_ep *ep);
int mobiveil_pcie_ep_raise_legacy_irq(struct mobiveil_pcie_ep *ep, u8 func_no);
int mobiveil_pcie_ep_raise_msi_irq(struct mobiveil_pcie_ep *ep, u8 func_no,
				   u8 interrupt_num);
int mobiveil_pcie_ep_raise_msix_irq(struct mobiveil_pcie_ep *ep, u8 func_no,
				    u16 interrupt_num);
void mobiveil_pcie_ep_reset_bar(struct mobiveil_pcie *pci, u8 bar);
u8 mobiveil_pcie_ep_get_bar_num(struct mobiveil_pcie_ep *ep, u8 func_no);
void mobiveil_pcie_enable_bridge_pio(struct mobiveil_pcie *pci);
void mobiveil_pcie_enable_engine_apio(struct mobiveil_pcie *pci);
void mobiveil_pcie_enable_engine_ppio(struct mobiveil_pcie *pci);
void mobiveil_pcie_enable_msi_ep(struct mobiveil_pcie *pci);
#endif /* _PCIE_MOBIVEIL_H */
