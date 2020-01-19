// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020 NXP
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>

struct samsung_hdmi_phy {
	struct device *dev;
	void __iomem *regs;

	struct phy *phy;
	struct clk *apbclk;
	struct clk *refclk;

	/* clk provider */
	struct clk_hw hw;
	struct clk *phyclk;
	u32 pixclk_rate;
};

struct phy_config {
	u32	clk_rate;
	u8 regs[48];
};

static const struct phy_config samsung_phy_cfg_table[] = {
	/* TODO more clock rate need added later */
	{	27000000, {
			0x00, 0xD1, 0x5A, 0xF0, 0x00, 0x00, 0x80, 0x00,
			0x4F, 0x30, 0x33, 0x65, 0x00, 0xE4, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x8F, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		74250000, {
			0x00, 0xD1, 0x5C, 0x52, 0x90, 0x0D, 0x84, 0x41,
			0x4F, 0x30, 0x33, 0x65, 0x10, 0xA6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x85, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, {
		148500000, {
			0x00, 0xd1, 0x3e, 0x15, 0xc0, 0x90, 0x88, 0x42, /* 00~1F   from verfication code */
			0x4f, 0x30, 0x33, 0x65, 0x20, 0xa6, 0x24, 0x80, /* 20~3F */
			0x6c, 0xf2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3a, /* 40~5F */
			0x74, 0x8f, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, /* 60~7F */
			0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 80~9F */
			0x00, 0xe0, 0x83, 0x0f, 0x3e, 0xf8, 0x00, 0x00, /* A0~BF */
		},
	}, {
		297000000, {
			0x00, 0xD1, 0x7B, 0x15, 0x84, 0x03, 0x90, 0x45,
			0x4F, 0x30, 0x33, 0x65, 0x30, 0xA6, 0x24, 0x80,
			0x6C, 0xF2, 0x67, 0x00, 0x10, 0x81, 0x30, 0x3A,
			0x74, 0x8F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
			0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xE0, 0x83, 0x0F, 0x3E, 0xF8, 0x00, 0x00,
		},
	}, { /* sentinel */ },
};

static inline struct samsung_hdmi_phy *to_samsung_hdmi_phy(struct clk_hw *hw)
{
	return container_of(hw, struct samsung_hdmi_phy, hw);
}

static int samsung_hdmi_phy_clk_prepare(struct clk_hw *hw)
{
	return 0;
}

static void samsung_hdmi_phy_clk_unprepare(struct clk_hw *hw)
{
}

static
unsigned long samsung_hdmi_phy_clk_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	return 0;
}

static long samsung_hdmi_phy_clk_round_rate(struct clk_hw *hw,
						unsigned long rate,
						unsigned long *parent_rate)
{
	const struct phy_config *phy_cfg = samsung_phy_cfg_table;

	for (; phy_cfg->clk_rate != 0; phy_cfg++)
		if (phy_cfg->clk_rate == rate)
			break;

	if (phy_cfg->clk_rate == 0)
		return -EINVAL;

	return phy_cfg->clk_rate;
}

static int samsung_hdmi_phy_clk_set_rate(struct clk_hw *hw,
					     unsigned long rate,
					     unsigned long parent_rate)
{
	struct samsung_hdmi_phy *samsung = to_samsung_hdmi_phy(hw);
	const struct phy_config *phy_cfg = samsung_phy_cfg_table;
	int i;

	dev_dbg(samsung->dev, "%s\n", __func__);

	for (; phy_cfg->clk_rate != 0; phy_cfg++)
		if (rate == phy_cfg->clk_rate)
			break;

	if (phy_cfg->clk_rate == 0)
		return -EINVAL;

	/* HDMI PHY init */
	writeb(2, samsung->regs + 0x84);

	for (i = 0; i < 48; i++)
		writeb(phy_cfg->regs[i], samsung->regs + i * 4);

	writeb(0x82, samsung->regs + 0x84);

	/* Wait for PHY PLL lock */
	/* while(reg32_read(0x32fc0230) != 0x000000ea); */
	msleep(20);
	samsung->pixclk_rate = rate;

	return 0;
}

static const struct clk_ops phy_clk_ops = {
	.prepare = samsung_hdmi_phy_clk_prepare,
	.unprepare = samsung_hdmi_phy_clk_unprepare,
	.recalc_rate = samsung_hdmi_phy_clk_recalc_rate,
	.round_rate = samsung_hdmi_phy_clk_round_rate,
	.set_rate = samsung_hdmi_phy_clk_set_rate,
};

static int samsung_hdmi_phy_clk_register(struct samsung_hdmi_phy *samsung)
{
	struct device *dev = samsung->dev;
	struct device_node *np = dev->of_node;
	struct clk_init_data init;
	const char *parent_name;
	int ret;

	parent_name = __clk_get_name(samsung->refclk);

	init.parent_names = &parent_name;
	init.num_parents = 0;
	init.flags = 0;
	init.name = "phy_pclk";
	init.ops = &phy_clk_ops;

	/* optional override of the clock name */
	of_property_read_string(np, "clock-output-names", &init.name);

	samsung->hw.init = &init;

	samsung->phyclk = devm_clk_register(dev, &samsung->hw);
	if (IS_ERR(samsung->phyclk)) {
		ret = PTR_ERR(samsung->phyclk);
		dev_err(dev, "failed to register clock: %d\n", ret);
		return ret;
	}

	ret = of_clk_add_provider(np, of_clk_src_simple_get, samsung->phyclk);
	if (ret) {
		dev_err(dev, "failed to register clock provider: %d\n", ret);
		return ret;
	}

	ret = device_reset(dev);
	if (ret == -EPROBE_DEFER)
		return ret;

	return 0;
}

static int samsung_hdmi_phy_power_on(struct phy *phy)
{
	return 0;
}

static int samsung_hdmi_phy_power_off(struct phy *phy)
{
	return 0;
}

static const struct phy_ops samsung_hdmi_phy_ops = {
	.owner = THIS_MODULE,
	.power_on = samsung_hdmi_phy_power_on,
	.power_off = samsung_hdmi_phy_power_off,
};

static int samsung_hdmi_phy_probe(struct platform_device *pdev)
{
	struct samsung_hdmi_phy *samsung;
	struct phy_provider *phy_provider;
	struct resource *res;
	void __iomem *regs;
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	samsung = devm_kzalloc(&pdev->dev, sizeof(*samsung), GFP_KERNEL);
	if (!samsung)
		return -ENOMEM;

	samsung->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(samsung->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	samsung->regs = regs;

	samsung->apbclk = devm_clk_get(samsung->dev, "apb");
	if (IS_ERR(samsung->apbclk)) {
		dev_err(samsung->dev, "failed to get phy apb clk\n");
		return -EPROBE_DEFER;
	}

	samsung->refclk = devm_clk_get(samsung->dev, "ref");
	if (IS_ERR(samsung->refclk)) {
		dev_err(samsung->dev, "failed to get phy refclk\n");
		return -EPROBE_DEFER;
	}

	ret = clk_prepare_enable(samsung->apbclk);
	if (ret) {
		dev_err(samsung->dev, "failed to enable apbclk\n");
		return ret;
	}

	samsung->phy = devm_phy_create(samsung->dev, NULL, &samsung_hdmi_phy_ops);
	if (IS_ERR(samsung->phy)) {
		dev_err(samsung->dev, "failed to create HDMI PHY\n");
		goto phy_failed;
	}

	phy_set_drvdata(samsung->phy, samsung);
	phy_set_bus_width(samsung->phy, 8);

	ret = samsung_hdmi_phy_clk_register(samsung);
	if (ret) {
		dev_err(&pdev->dev, "%s register clk failed\n", __func__);
		goto phy_failed;
	}

	dev_dbg(&pdev->dev, "%s exit\n", __func__);
	phy_provider = devm_of_phy_provider_register(samsung->dev,
						     of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(&pdev->dev, "failed to register PHY provider\n");
		ret =  PTR_ERR(phy_provider);
		goto phy_failed;
	}
	return 0;

phy_failed:
	clk_disable_unprepare(samsung->apbclk);
	return ret;
}

static int samsung_hdmi_phy_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static const struct of_device_id samsung_hdmi_phy_of_match[] = {
	{
		.compatible = "fsl,samsung-hdmi-phy",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, samsung_hdmi_phy_of_match);

static struct platform_driver samsung_hdmi_phy_driver = {
	.probe  = samsung_hdmi_phy_probe,
	.remove = samsung_hdmi_phy_remove,
	.driver = {
		.name = "samsung-hdmi-phy",
		.of_match_table = samsung_hdmi_phy_of_match,
	},
};
module_platform_driver(samsung_hdmi_phy_driver);

MODULE_AUTHOR("Sandor Yu <Sandor.yu@nxp.com>");
MODULE_DESCRIPTION("SAMSUNG HDMI 2.0 Transmitter PHY Driver");
MODULE_LICENSE("GPL v2");
