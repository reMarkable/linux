/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2019 NXP */

#include <linux/phy.h>

/* PCS registers */
#define ENETC_PCS_LINK_TIMER1		0x12
#define ENETC_PCS_LINK_TIMER1_VAL	0x06a0
#define ENETC_PCS_LINK_TIMER2		0x13
#define ENETC_PCS_LINK_TIMER2_VAL	0x0003
#define ENETC_PCS_IF_MODE		0x14
#define ENETC_PCS_IF_MODE_SGMII_AN	0x0003

struct enetc_mdio_priv {
	struct enetc_hw *hw;
	int mdio_base;
};

int enetc_mdio_write(struct mii_bus *bus, int phy_id, int regnum, u16 value);
int enetc_mdio_read(struct mii_bus *bus, int phy_id, int regnum);
struct enetc_hw *enetc_hw_alloc(struct device *dev, void __iomem *port_regs);
