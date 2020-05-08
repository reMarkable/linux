// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define PHY_PLL_LOCK_WAIT_MAX_RETRIES	2000

struct imx8_pcie_phy {
	struct phy *phy;
	struct clk *clk;
	void __iomem *base;
};

static int imx8_pcie_phy_init(struct phy *phy)
{
	u32 value, retries = 0;
	struct imx8_pcie_phy *imx8_phy = phy_get_drvdata(phy);

	/*
	 * SW workaround for ERR050442 of the iMX865 PCIe.
	 * Description:
	 * PCIE PHY can't support GEN3. GEN 1/2 are supported. Some
	 * buffer structures within the digital implementation in the
	 * PHY required for GEN3 operation are incorrect. This logic can
	 * be bypassed (optionally) in GEN 1/2 operation. This same
	 * implementation bug prevents SW form reading some status bits
	 * in all PCIe modes.
	 *
	 * Workaround:
	 * SW driver doesn't need to read the PHY status bits for proper
	 * PCIe operation. The buffer structure can be bypassed to
	 * completely support GEN1/2 operation. With some SW driver
	 * workarounds to read status from the PCIe PCS instead of the
	 * PHY, proper operation can be achieved for GEN1/2. Proper
	 * operation for GEN3 cannot be achieved with the SW workaround
	 * since the buffer structure cannot be bypassed in GEN3 mode.
	 */
	/* wait for pipe0_clk locked by checking status from PCS. */
	for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES;
	     retries++) {
		value = readl(imx8_phy->base + 0x8188);
		if (value == BIT(1))
			break;
		udelay(10);
	}

	/* setup_deskew_fifo_bypass to workaround ERR050442 */
	writel(0x32, imx8_phy->base + 0xd8);
	writel(0x1, imx8_phy->base + 0x480);
	writel(0x0, imx8_phy->base + 0x488);
	writel(0x4, imx8_phy->base + 0x73c);
	writel(0x4, imx8_phy->base + 0x6ec);

	/* Configure TX drive level  */
	writel(0x2d, imx8_phy->base + 0x404);

	return 0;
}

static int imx8_pcie_phy_power_on(struct phy *phy)
{
	struct imx8_pcie_phy *imx8_phy = phy_get_drvdata(phy);

	return clk_prepare_enable(imx8_phy->clk);
}

static int imx8_pcie_phy_power_off(struct phy *phy)
{
	struct imx8_pcie_phy *imx8_phy = phy_get_drvdata(phy);

	clk_disable_unprepare(imx8_phy->clk);

	return 0;
}

static struct phy_ops imx8_pcie_phy_ops = {
	.init		= imx8_pcie_phy_init,
	.power_on	= imx8_pcie_phy_power_on,
	.power_off	= imx8_pcie_phy_power_off,
	.owner		= THIS_MODULE,
};

static int imx8_pcie_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct imx8_pcie_phy *imx8_phy;
	struct resource *res;

	imx8_phy = devm_kzalloc(dev, sizeof(*imx8_phy), GFP_KERNEL);
	if (!imx8_phy)
		return -ENOMEM;

	imx8_phy->clk = devm_clk_get(dev, "phy");
	if (IS_ERR(imx8_phy->clk)) {
		dev_err(dev, "failed to get imx pcie phy clock\n");
		return PTR_ERR(imx8_phy->clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	imx8_phy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(imx8_phy->base))
		return PTR_ERR(imx8_phy->base);

	imx8_phy->phy = devm_phy_create(dev, NULL, &imx8_pcie_phy_ops);
	if (IS_ERR(imx8_phy->phy))
		return PTR_ERR(imx8_phy->phy);

	phy_set_drvdata(imx8_phy->phy, imx8_phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id imx8_pcie_phy_of_match[] = {
	{.compatible = "fsl,imx8mp-pcie-phy",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx8_pcie_phy_of_match);

static struct platform_driver imx8_pcie_phy_driver = {
	.probe	= imx8_pcie_phy_probe,
	.driver = {
		.name	= "imx8-pcie-phy",
		.of_match_table	= imx8_pcie_phy_of_match,
	}
};
module_platform_driver(imx8_pcie_phy_driver);

MODULE_DESCRIPTION("FSL IMX8 PCIE PHY driver");
MODULE_LICENSE("GPL");
