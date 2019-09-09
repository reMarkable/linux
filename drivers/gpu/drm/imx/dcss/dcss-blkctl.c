// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/of.h>

#include "dcss-dev.h"

#define DCSS_BLKCTL_RESET_CTRL		0x00
#define   B_CLK_RESETN			BIT(0)
#define   APB_CLK_RESETN		BIT(1)
#define   P_CLK_RESETN			BIT(2)
#define   RTR_CLK_RESETN		BIT(3)
#define   HDMI_RESETN			BIT(4)
#define DCSS_BLKCTL_CONTROL0		0x10
#define   HDMI_MIPI_CLK_SEL		BIT(0)
#define   DISPMIX_REFCLK_SEL_POS	4
#define   DISPMIX_REFCLK_SEL_MASK	GENMASK(5, 4)
#define   DISPMIX_PIXCLK_SEL		BIT(8)
#define   HDMI_SRC_SECURE_EN		BIT(16)

struct dcss_blkctl {
	struct device *dev;
	void __iomem *base_reg;

	bool hdmi_output;
};

void dcss_blkctl_cfg(struct dcss_blkctl *blkctl)
{
	if (blkctl->hdmi_output)
		dcss_writel(0, blkctl->base_reg + DCSS_BLKCTL_CONTROL0);
	else
		dcss_writel(DISPMIX_PIXCLK_SEL,
			    blkctl->base_reg + DCSS_BLKCTL_CONTROL0);

	/* deassert all clock domains resets */
	dcss_set(B_CLK_RESETN | APB_CLK_RESETN | P_CLK_RESETN |
		 RTR_CLK_RESETN | HDMI_RESETN,
		 blkctl->base_reg + DCSS_BLKCTL_RESET_CTRL);
}

int dcss_blkctl_init(struct dcss_dev *dcss, unsigned long blkctl_base)
{
	struct device_node *node = dcss->dev->of_node;
	int len;
	const char *disp_dev;
	struct dcss_blkctl *blkctl;

	blkctl = devm_kzalloc(dcss->dev, sizeof(*blkctl), GFP_KERNEL);
	if (!blkctl)
		return -ENOMEM;

	blkctl->base_reg = devm_ioremap(dcss->dev, blkctl_base, SZ_4K);
	if (!blkctl->base_reg) {
		dev_err(dcss->dev, "unable to remap BLK CTRL base\n");
		return -ENOMEM;
	}

	dcss->blkctl = blkctl;
	blkctl->dev = dcss->dev;

	disp_dev = of_get_property(node, "disp-dev", &len);
	if (!disp_dev || !strncmp(disp_dev, "hdmi_disp", 9))
		blkctl->hdmi_output = true;

	dcss_blkctl_cfg(blkctl);

	return 0;
}

void dcss_blkctl_exit(struct dcss_blkctl *blkctl)
{
	/* assert clock domains resets */
	dcss_clr(B_CLK_RESETN | P_CLK_RESETN | HDMI_RESETN | RTR_CLK_RESETN,
		 blkctl->base_reg + DCSS_BLKCTL_RESET_CTRL);

	devm_kfree(blkctl->dev, blkctl);
}
