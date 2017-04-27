// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * drivers/irqchip/irq-qeic.c
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.  All rights reserved.
 *
 * Author: Li Yang <leoli@freescale.com>
 * Based on code from Shlomi Gridish <gridish@freescale.com>
 *
 * QUICC ENGINE Interrupt Controller
 */

#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/errno.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <asm/io.h>

#define NR_QE_IC_INTS		64

/* QE IC registers offset */
#define QEIC_CICR		0x00
#define QEIC_CIVEC		0x04
#define QEIC_CRIPNR		0x08
#define QEIC_CIPNR		0x0c
#define QEIC_CIPXCC		0x10
#define QEIC_CIPYCC		0x14
#define QEIC_CIPWCC		0x18
#define QEIC_CIPZCC		0x1c
#define QEIC_CIMR		0x20
#define QEIC_CRIMR		0x24
#define QEIC_CICNR		0x28
#define QEIC_CIPRTA		0x30
#define QEIC_CIPRTB		0x34
#define QEIC_CRICR		0x3c
#define QEIC_CHIVEC		0x60

/* Interrupt priority registers */
#define CIPCC_SHIFT_PRI0	29
#define CIPCC_SHIFT_PRI1	26
#define CIPCC_SHIFT_PRI2	23
#define CIPCC_SHIFT_PRI3	20
#define CIPCC_SHIFT_PRI4	13
#define CIPCC_SHIFT_PRI5	10
#define CIPCC_SHIFT_PRI6	7
#define CIPCC_SHIFT_PRI7	4

/* CICR priority modes */
#define CICR_GWCC		0x00040000
#define CICR_GXCC		0x00020000
#define CICR_GYCC		0x00010000
#define CICR_GZCC		0x00080000
#define CICR_GRTA		0x00200000
#define CICR_GRTB		0x00400000
#define CICR_HPIT_SHIFT		8
#define CICR_HPIT_MASK		0x00000300
#define CICR_HP_SHIFT		24
#define CICR_HP_MASK		0x3f000000

/* CICNR */
#define CICNR_WCC1T_SHIFT	20
#define CICNR_ZCC1T_SHIFT	28
#define CICNR_YCC1T_SHIFT	12
#define CICNR_XCC1T_SHIFT	4

/* CRICR */
#define CRICR_RTA1T_SHIFT	20
#define CRICR_RTB1T_SHIFT	28

/* Signal indicator */
#define SIGNAL_MASK		3
#define SIGNAL_HIGH		2
#define SIGNAL_LOW		0

#define NUM_OF_QE_IC_GROUPS	6

/* Flags when we init the QE IC */
#define QE_IC_SPREADMODE_GRP_W			0x00000001
#define QE_IC_SPREADMODE_GRP_X			0x00000002
#define QE_IC_SPREADMODE_GRP_Y			0x00000004
#define QE_IC_SPREADMODE_GRP_Z			0x00000008
#define QE_IC_SPREADMODE_GRP_RISCA		0x00000010
#define QE_IC_SPREADMODE_GRP_RISCB		0x00000020

#define QE_IC_LOW_SIGNAL			0x00000100
#define QE_IC_HIGH_SIGNAL			0x00000200

#define QE_IC_GRP_W_PRI0_DEST_SIGNAL_HIGH	0x00001000
#define QE_IC_GRP_W_PRI1_DEST_SIGNAL_HIGH	0x00002000
#define QE_IC_GRP_X_PRI0_DEST_SIGNAL_HIGH	0x00004000
#define QE_IC_GRP_X_PRI1_DEST_SIGNAL_HIGH	0x00008000
#define QE_IC_GRP_Y_PRI0_DEST_SIGNAL_HIGH	0x00010000
#define QE_IC_GRP_Y_PRI1_DEST_SIGNAL_HIGH	0x00020000
#define QE_IC_GRP_Z_PRI0_DEST_SIGNAL_HIGH	0x00040000
#define QE_IC_GRP_Z_PRI1_DEST_SIGNAL_HIGH	0x00080000
#define QE_IC_GRP_RISCA_PRI0_DEST_SIGNAL_HIGH	0x00100000
#define QE_IC_GRP_RISCA_PRI1_DEST_SIGNAL_HIGH	0x00200000
#define QE_IC_GRP_RISCB_PRI0_DEST_SIGNAL_HIGH	0x00400000
#define QE_IC_GRP_RISCB_PRI1_DEST_SIGNAL_HIGH	0x00800000
#define QE_IC_GRP_W_DEST_SIGNAL_SHIFT		(12)

/* QE interrupt sources groups */
enum qe_ic_grp_id {
	QE_IC_GRP_W = 0,	/* QE interrupt controller group W */
	QE_IC_GRP_X,		/* QE interrupt controller group X */
	QE_IC_GRP_Y,		/* QE interrupt controller group Y */
	QE_IC_GRP_Z,		/* QE interrupt controller group Z */
	QE_IC_GRP_RISCA,	/* QE interrupt controller RISC group A */
	QE_IC_GRP_RISCB		/* QE interrupt controller RISC group B */
};

struct qe_ic {
	/* Control registers offset */
	u32 __iomem *regs;

	/* The remapper for this QEIC */
	struct irq_domain *irqhost;

	/* The "linux" controller struct */
	struct irq_chip hc_irq;

	/* VIRQ numbers of QE high/low irqs */
	unsigned int virq_high;
	unsigned int virq_low;
};

/*
 * QE interrupt controller internal structure
 */
struct qe_ic_info {
	/* location of this source at the QIMR register. */
	u32	mask;

	/* Mask register offset */
	u32	mask_reg;

	/*
	 * for grouped interrupts sources - the interrupt
	 * code as appears at the group priority register
	 */
	u8	pri_code;

	/* Group priority register offset */
	u32	pri_reg;
};

static DEFINE_RAW_SPINLOCK(qe_ic_lock);

static struct qe_ic_info qe_ic_info[] = {
	[1] = {
	       .mask = 0x00008000,
	       .mask_reg = QEIC_CIMR,
	       .pri_code = 0,
	       .pri_reg = QEIC_CIPWCC,
	       },
	[2] = {
	       .mask = 0x00004000,
	       .mask_reg = QEIC_CIMR,
	       .pri_code = 1,
	       .pri_reg = QEIC_CIPWCC,
	       },
	[3] = {
	       .mask = 0x00002000,
	       .mask_reg = QEIC_CIMR,
	       .pri_code = 2,
	       .pri_reg = QEIC_CIPWCC,
	       },
	[10] = {
		.mask = 0x00000040,
		.mask_reg = QEIC_CIMR,
		.pri_code = 1,
		.pri_reg = QEIC_CIPZCC,
		},
	[11] = {
		.mask = 0x00000020,
		.mask_reg = QEIC_CIMR,
		.pri_code = 2,
		.pri_reg = QEIC_CIPZCC,
		},
	[12] = {
		.mask = 0x00000010,
		.mask_reg = QEIC_CIMR,
		.pri_code = 3,
		.pri_reg = QEIC_CIPZCC,
		},
	[13] = {
		.mask = 0x00000008,
		.mask_reg = QEIC_CIMR,
		.pri_code = 4,
		.pri_reg = QEIC_CIPZCC,
		},
	[14] = {
		.mask = 0x00000004,
		.mask_reg = QEIC_CIMR,
		.pri_code = 5,
		.pri_reg = QEIC_CIPZCC,
		},
	[15] = {
		.mask = 0x00000002,
		.mask_reg = QEIC_CIMR,
		.pri_code = 6,
		.pri_reg = QEIC_CIPZCC,
		},
	[20] = {
		.mask = 0x10000000,
		.mask_reg = QEIC_CRIMR,
		.pri_code = 3,
		.pri_reg = QEIC_CIPRTA,
		},
	[25] = {
		.mask = 0x00800000,
		.mask_reg = QEIC_CRIMR,
		.pri_code = 0,
		.pri_reg = QEIC_CIPRTB,
		},
	[26] = {
		.mask = 0x00400000,
		.mask_reg = QEIC_CRIMR,
		.pri_code = 1,
		.pri_reg = QEIC_CIPRTB,
		},
	[27] = {
		.mask = 0x00200000,
		.mask_reg = QEIC_CRIMR,
		.pri_code = 2,
		.pri_reg = QEIC_CIPRTB,
		},
	[28] = {
		.mask = 0x00100000,
		.mask_reg = QEIC_CRIMR,
		.pri_code = 3,
		.pri_reg = QEIC_CIPRTB,
		},
	[32] = {
		.mask = 0x80000000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 0,
		.pri_reg = QEIC_CIPXCC,
		},
	[33] = {
		.mask = 0x40000000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 1,
		.pri_reg = QEIC_CIPXCC,
		},
	[34] = {
		.mask = 0x20000000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 2,
		.pri_reg = QEIC_CIPXCC,
		},
	[35] = {
		.mask = 0x10000000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 3,
		.pri_reg = QEIC_CIPXCC,
		},
	[36] = {
		.mask = 0x08000000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 4,
		.pri_reg = QEIC_CIPXCC,
		},
	[40] = {
		.mask = 0x00800000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 0,
		.pri_reg = QEIC_CIPYCC,
		},
	[41] = {
		.mask = 0x00400000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 1,
		.pri_reg = QEIC_CIPYCC,
		},
	[42] = {
		.mask = 0x00200000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 2,
		.pri_reg = QEIC_CIPYCC,
		},
	[43] = {
		.mask = 0x00100000,
		.mask_reg = QEIC_CIMR,
		.pri_code = 3,
		.pri_reg = QEIC_CIPYCC,
		},
};

static inline u32 qe_ic_read(__be32  __iomem *base, unsigned int reg)
{
	return ioread32be(base + (reg >> 2));
}

static inline void qe_ic_write(__be32  __iomem *base, unsigned int reg,
			       u32 value)
{
	iowrite32be(value, base + (reg >> 2));
}

static inline struct qe_ic *qe_ic_from_irq(unsigned int virq)
{
	return irq_get_chip_data(virq);
}

static inline struct qe_ic *qe_ic_from_irq_data(struct irq_data *d)
{
	return irq_data_get_irq_chip_data(d);
}

static void qe_ic_unmask_irq(struct irq_data *d)
{
	struct qe_ic *qe_ic = qe_ic_from_irq_data(d);
	unsigned int src = irqd_to_hwirq(d);
	unsigned long flags;
	u32 temp;

	raw_spin_lock_irqsave(&qe_ic_lock, flags);

	temp = qe_ic_read(qe_ic->regs, qe_ic_info[src].mask_reg);
	qe_ic_write(qe_ic->regs, qe_ic_info[src].mask_reg,
		    temp | qe_ic_info[src].mask);

	raw_spin_unlock_irqrestore(&qe_ic_lock, flags);
}

static void qe_ic_mask_irq(struct irq_data *d)
{
	struct qe_ic *qe_ic = qe_ic_from_irq_data(d);
	unsigned int src = irqd_to_hwirq(d);
	unsigned long flags;
	u32 temp;

	raw_spin_lock_irqsave(&qe_ic_lock, flags);

	temp = qe_ic_read(qe_ic->regs, qe_ic_info[src].mask_reg);
	qe_ic_write(qe_ic->regs, qe_ic_info[src].mask_reg,
		    temp & ~qe_ic_info[src].mask);

	/* Flush the above write before enabling interrupts; otherwise,
	 * spurious interrupts will sometimes happen.  To be 100% sure
	 * that the write has reached the device before interrupts are
	 * enabled, the mask register would have to be read back; however,
	 * this is not required for correctness, only to avoid wasting
	 * time on a large number of spurious interrupts.  In testing,
	 * a sync reduced the observed spurious interrupts to zero.
	 */
	mb();

	raw_spin_unlock_irqrestore(&qe_ic_lock, flags);
}

static struct irq_chip qe_ic_irq_chip = {
	.name = "QEIC",
	.irq_unmask = qe_ic_unmask_irq,
	.irq_mask = qe_ic_mask_irq,
	.irq_mask_ack = qe_ic_mask_irq,
};

static int qe_ic_host_match(struct irq_domain *h, struct device_node *node,
			    enum irq_domain_bus_token bus_token)
{
	/* Exact match, unless qe_ic node is NULL */
	struct device_node *of_node = irq_domain_get_of_node(h);
	return of_node == NULL || of_node == node;
}

static int qe_ic_host_map(struct irq_domain *h, unsigned int virq,
			  irq_hw_number_t hw)
{
	struct qe_ic *qe_ic = h->host_data;
	struct irq_chip *chip;

	if (hw >= ARRAY_SIZE(qe_ic_info)) {
		pr_err("%s: Invalid hw irq number for QEIC\n", __func__);
		return -EINVAL;
	}

	if (qe_ic_info[hw].mask == 0) {
		printk(KERN_ERR "Can't map reserved IRQ\n");
		return -EINVAL;
	}
	/* Default chip */
	chip = &qe_ic->hc_irq;

	irq_set_chip_data(virq, qe_ic);
	irq_set_status_flags(virq, IRQ_LEVEL);

	irq_set_chip_and_handler(virq, chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops qe_ic_host_ops = {
	.match = qe_ic_host_match,
	.map = qe_ic_host_map,
	.xlate = irq_domain_xlate_onetwocell,
};

/* Return an interrupt vector or 0 if no interrupt is pending. */
static unsigned int qe_ic_get_low_irq(struct qe_ic *qe_ic)
{
	int irq;

	BUG_ON(qe_ic == NULL);

	/* get the interrupt source vector. */
	irq = qe_ic_read(qe_ic->regs, QEIC_CIVEC) >> 26;

	if (irq == 0)
		return 0;

	return irq_linear_revmap(qe_ic->irqhost, irq);
}

/* Return an interrupt vector or 0 if no interrupt is pending. */
static unsigned int qe_ic_get_high_irq(struct qe_ic *qe_ic)
{
	int irq;

	BUG_ON(qe_ic == NULL);

	/* get the interrupt source vector. */
	irq = qe_ic_read(qe_ic->regs, QEIC_CHIVEC) >> 26;

	if (irq == 0)
		return 0;

	return irq_linear_revmap(qe_ic->irqhost, irq);
}

static inline void qe_ic_cascade_low_ipic(struct irq_desc *desc)
{
	struct qe_ic *qe_ic = irq_desc_get_handler_data(desc);
	unsigned int cascade_irq = qe_ic_get_low_irq(qe_ic);

	if (cascade_irq != 0)
		generic_handle_irq(cascade_irq);
}

static inline void qe_ic_cascade_high_ipic(struct irq_desc *desc)
{
	struct qe_ic *qe_ic = irq_desc_get_handler_data(desc);
	unsigned int cascade_irq = qe_ic_get_high_irq(qe_ic);

	if (cascade_irq != 0)
		generic_handle_irq(cascade_irq);
}

static inline void qe_ic_cascade_low_mpic(struct irq_desc *desc)
{
	struct qe_ic *qe_ic = irq_desc_get_handler_data(desc);
	unsigned int cascade_irq = qe_ic_get_low_irq(qe_ic);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	if (cascade_irq != 0)
		generic_handle_irq(cascade_irq);

	chip->irq_eoi(&desc->irq_data);
}

static inline void qe_ic_cascade_high_mpic(struct irq_desc *desc)
{
	struct qe_ic *qe_ic = irq_desc_get_handler_data(desc);
	unsigned int cascade_irq = qe_ic_get_high_irq(qe_ic);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	if (cascade_irq != 0)
		generic_handle_irq(cascade_irq);

	chip->irq_eoi(&desc->irq_data);
}

static inline void qe_ic_cascade_muxed_mpic(struct irq_desc *desc)
{
	struct qe_ic *qe_ic = irq_desc_get_handler_data(desc);
	unsigned int cascade_irq;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	cascade_irq = qe_ic_get_high_irq(qe_ic);
	if (cascade_irq == 0)
		cascade_irq = qe_ic_get_low_irq(qe_ic);

	if (cascade_irq != 0)
		generic_handle_irq(cascade_irq);

	chip->irq_eoi(&desc->irq_data);
}

static int __init qe_ic_init(struct device_node *node, unsigned int flags)
{
	struct qe_ic *qe_ic;
	struct resource res;
	u32 temp = 0, high_active = 0;
	int ret = 0;

	if (!node)
		return -ENODEV;

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		ret = -ENODEV;
		goto err_put_node;
	}

	qe_ic = kzalloc(sizeof(*qe_ic), GFP_KERNEL);
	if (qe_ic == NULL) {
		ret = -ENOMEM;
		goto err_put_node;
	}

	qe_ic->irqhost = irq_domain_add_linear(node, NR_QE_IC_INTS,
					       &qe_ic_host_ops, qe_ic);
	if (qe_ic->irqhost == NULL) {
		ret = -ENOMEM;
		goto err_free_qe_ic;
	}

	qe_ic->regs = ioremap(res.start, resource_size(&res));

	qe_ic->hc_irq = qe_ic_irq_chip;

	qe_ic->virq_high = irq_of_parse_and_map(node, 0);
	qe_ic->virq_low = irq_of_parse_and_map(node, 1);

	if (qe_ic->virq_low == 0) {
		pr_err("Failed to map QE_IC low IRQ\n");
		ret = -ENOMEM;
		goto err_domain_remove;
	}

	/* default priority scheme is grouped. If spread mode is    */
	/* required, configure cicr accordingly.                    */
	if (flags & QE_IC_SPREADMODE_GRP_W)
		temp |= CICR_GWCC;
	if (flags & QE_IC_SPREADMODE_GRP_X)
		temp |= CICR_GXCC;
	if (flags & QE_IC_SPREADMODE_GRP_Y)
		temp |= CICR_GYCC;
	if (flags & QE_IC_SPREADMODE_GRP_Z)
		temp |= CICR_GZCC;
	if (flags & QE_IC_SPREADMODE_GRP_RISCA)
		temp |= CICR_GRTA;
	if (flags & QE_IC_SPREADMODE_GRP_RISCB)
		temp |= CICR_GRTB;

	/* choose destination signal for highest priority interrupt */
	if (flags & QE_IC_HIGH_SIGNAL) {
		temp |= (SIGNAL_HIGH << CICR_HPIT_SHIFT);
		high_active = 1;
	}

	qe_ic_write(qe_ic->regs, QEIC_CICR, temp);

	irq_set_handler_data(qe_ic->virq_low, qe_ic);
	irq_set_chained_handler(qe_ic->virq_low, qe_ic_cascade_low_mpic);

	if (qe_ic->virq_high != 0 &&
			qe_ic->virq_high != qe_ic->virq_low) {
		irq_set_handler_data(qe_ic->virq_high, qe_ic);
		irq_set_chained_handler(qe_ic->virq_high,
					qe_ic_cascade_high_mpic);
	}
	of_node_put(node);
	return 0;

err_domain_remove:
	irq_domain_remove(qe_ic->irqhost);
err_free_qe_ic:
	kfree(qe_ic);
err_put_node:
	of_node_put(node);
	return ret;
}

static int __init init_qe_ic(struct device_node *node,
			     struct device_node *parent)
{
	int ret;

	ret = qe_ic_init(node, 0);
	if (ret)
		return ret;

	return 0;
}

IRQCHIP_DECLARE(qeic, "fsl,qe-ic", init_qe_ic);
