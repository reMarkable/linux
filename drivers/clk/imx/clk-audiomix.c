// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <dt-bindings/clock/imx8mp-clock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>

#include "clk.h"

static int shared_count_pdm;
static struct clk *clks[IMX8MP_CLK_AUDIOMIX_END];
static struct clk_onecell_data clk_data;
static uint32_t audiomix_clk_saved_regs[14];
static struct clk *clk_audio_root;

/* descending order */
static const struct imx_pll14xx_rate_table imx_audiomix_sai_pll_tbl[] = {
	PLL_1443X_RATE(245760000U, 328, 4, 3, 0xae15),
	PLL_1443X_RATE(225792000U, 226, 3, 3, 0xcac1),
	PLL_1443X_RATE(122880000U, 328, 4, 4, 0xae15),
	PLL_1443X_RATE(112896000U, 226,	3, 4, 0xcac1),
	PLL_1443X_RATE(61440000U, 328, 4, 5, 0xae15),
	PLL_1443X_RATE(56448000U, 226, 3, 5, 0xcac1),
	PLL_1443X_RATE(49152000U, 393, 3, 6, 0x374c),
	PLL_1443X_RATE(45158400U, 241, 2, 6, 0xd845),
	PLL_1443X_RATE(40960000U, 109, 1, 6, 0x3a07),
};

static const struct imx_pll14xx_clk imx_audiomix_sai_pll = {
	.type = PLL_1443X,
	.rate_table = imx_audiomix_sai_pll_tbl,
	.rate_count = ARRAY_SIZE(imx_audiomix_sai_pll_tbl),
};

static const char *imx_sai_mclk2_sels[] = {"sai1", "sai2", "sai3", "dummy",
					"sai5", "sai6", "sai7", "dummy",
					"dummy", "dummy", "dummy",
					"dummy", "dummy", "dummy", "dummy"};
static const char *imx_sai1_mclk1_sels[] = {"sai1", "dummy", };
static const char *imx_sai2_mclk1_sels[] = {"sai2", "dummy", };
static const char *imx_sai3_mclk1_sels[] = {"sai3", "dummy", };
static const char *imx_sai5_mclk1_sels[] = {"sai5", "dummy", };
static const char *imx_sai6_mclk1_sels[] = {"sai6", "dummy", };
static const char *imx_sai7_mclk1_sels[] = {"sai7", "dummy", };
static const char *imx_pdm_sels[] = {"pdm", "sai_pll_div2", "dummy", "dummy" };
static const char *imx_sai_pll_ref_sels[] = {"osc_24m", "dummy", "dummy", "dummy", };
static const char *imx_sai_pll_bypass_sels[] = {"sai_pll", "sai_pll_ref_sel", };

static int imx_audiomix_clk_runtime_suspend(struct device *dev)
{
	void __iomem *base;

	base = dev_get_drvdata(dev->parent);

	audiomix_clk_saved_regs[0] = readl(base);
	audiomix_clk_saved_regs[1] = readl(base + 0x4);

	audiomix_clk_saved_regs[2] = readl(base + 0x300);
	audiomix_clk_saved_regs[3] = readl(base + 0x304);
	audiomix_clk_saved_regs[4] = readl(base + 0x308);
	audiomix_clk_saved_regs[5] = readl(base + 0x30C);
	audiomix_clk_saved_regs[6] = readl(base + 0x310);
	audiomix_clk_saved_regs[7] = readl(base + 0x314);
	audiomix_clk_saved_regs[8] = readl(base + 0x318);

	audiomix_clk_saved_regs[9] = readl(base + 0x400);
	audiomix_clk_saved_regs[10] = readl(base + 0x404);
	audiomix_clk_saved_regs[11] = readl(base + 0x408);
	audiomix_clk_saved_regs[12] = readl(base + 0x40C);
	audiomix_clk_saved_regs[13] = readl(base + 0x410);

	clk_disable_unprepare(clk_audio_root);

	return 0;
}

static int imx_audiomix_clk_runtime_resume(struct device *dev)
{
	void __iomem *base;

	base = dev_get_drvdata(dev->parent);

	clk_prepare_enable(clk_audio_root);

	/*
	 * Ignore bit26, which are clock gate for sdma clock root.
	 * We need to keep it on as reset state for hardware issue
	 * that sdma3' event logic depends on sdma2's clock gate.
	 * keep it enabled can workaround the issue.
	 */
	writel(audiomix_clk_saved_regs[0] | 0x4000000, base);
	writel(audiomix_clk_saved_regs[1], base + 0x4);

	writel(audiomix_clk_saved_regs[2], base + 0x300);
	writel(audiomix_clk_saved_regs[3], base + 0x304);
	writel(audiomix_clk_saved_regs[4], base + 0x308);
	writel(audiomix_clk_saved_regs[5], base + 0x30C);
	writel(audiomix_clk_saved_regs[6], base + 0x310);
	writel(audiomix_clk_saved_regs[7], base + 0x314);
	writel(audiomix_clk_saved_regs[8], base + 0x318);

	writel(audiomix_clk_saved_regs[9], base + 0x400);
	writel(audiomix_clk_saved_regs[10], base + 0x404);
	writel(audiomix_clk_saved_regs[11], base + 0x408);
	writel(audiomix_clk_saved_regs[12], base + 0x40C);
	writel(audiomix_clk_saved_regs[13], base + 0x410);

	return 0;
}

static int imx_audiomix_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *base;
	int ret;

	clk_audio_root = of_clk_get_by_name(np, "audio_root");
	if (IS_ERR(clk_audio_root))
		return PTR_ERR(clk_audio_root);

	base = dev_get_drvdata(dev->parent);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	clks[IMX8MP_CLK_AUDIOMIX_SAI_PLL_REF_SEL] = imx_dev_clk_mux(dev, "sai_pll_ref_sel", base + 0x400, 0, 2, imx_sai_pll_ref_sels, ARRAY_SIZE(imx_sai_pll_ref_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI_PLL] = imx_dev_clk_pll14xx(dev, "sai_pll", "sai_pll_ref_sel", base + 0x400, &imx_audiomix_sai_pll);

	clks[IMX8MP_CLK_AUDIOMIX_SAI_PLL_BYPASS] = imx_dev_clk_mux_flags(dev, "sai_pll_bypass", base + 0x400, 4, 1, imx_sai_pll_bypass_sels, ARRAY_SIZE(imx_sai_pll_bypass_sels), CLK_SET_RATE_PARENT);

	clks[IMX8MP_CLK_AUDIOMIX_SAI_PLL_OUT] = imx_dev_clk_gate(dev, "sai_pll_out", "sai_pll_bypass", base + 0x400, 13);

	clks[IMX8MP_CLK_AUDIOMIX_SAI1_MCLK1_SEL] = imx_dev_clk_mux_flags(dev, "sai1_mclk1_sel", base + 0x300, 0, 1, imx_sai1_mclk1_sels, ARRAY_SIZE(imx_sai1_mclk1_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MP_CLK_AUDIOMIX_SAI1_MCLK2_SEL] = imx_dev_clk_mux(dev, "sai1_mclk2_sel", base + 0x300, 1, 4, imx_sai_mclk2_sels, ARRAY_SIZE(imx_sai_mclk2_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI2_MCLK1_SEL] = imx_dev_clk_mux_flags(dev, "sai2_mclk1_sel", base + 0x304, 0, 1, imx_sai2_mclk1_sels, ARRAY_SIZE(imx_sai2_mclk1_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MP_CLK_AUDIOMIX_SAI2_MCLK2_SEL] = imx_dev_clk_mux(dev, "sai2_mclk2_sel", base + 0x304, 1, 4, imx_sai_mclk2_sels, ARRAY_SIZE(imx_sai_mclk2_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI3_MCLK1_SEL] = imx_dev_clk_mux_flags(dev, "sai3_mclk1_sel", base + 0x308, 0, 1, imx_sai3_mclk1_sels, ARRAY_SIZE(imx_sai3_mclk1_sels), CLK_SET_RATE_PARENT);
	clks[IMX8MP_CLK_AUDIOMIX_SAI3_MCLK2_SEL] = imx_dev_clk_mux(dev, "sai3_mclk2_sel", base + 0x308, 1, 4, imx_sai_mclk2_sels, ARRAY_SIZE(imx_sai_mclk2_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI5_MCLK1_SEL] = imx_dev_clk_mux(dev, "sai5_mclk1_sel", base + 0x30C, 0, 1, imx_sai5_mclk1_sels, ARRAY_SIZE(imx_sai5_mclk1_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI5_MCLK2_SEL] = imx_dev_clk_mux(dev, "sai5_mclk2_sel", base + 0x30C, 1, 4, imx_sai_mclk2_sels, ARRAY_SIZE(imx_sai_mclk2_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI6_MCLK1_SEL] = imx_dev_clk_mux(dev, "sai6_mclk1_sel", base + 0x310, 0, 1, imx_sai6_mclk1_sels, ARRAY_SIZE(imx_sai6_mclk1_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI6_MCLK2_SEL] = imx_dev_clk_mux(dev, "sai6_mclk2_sel", base + 0x310, 1, 4, imx_sai_mclk2_sels, ARRAY_SIZE(imx_sai_mclk2_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI7_MCLK1_SEL] = imx_dev_clk_mux(dev, "sai7_mclk1_sel", base + 0x314, 0, 1, imx_sai7_mclk1_sels, ARRAY_SIZE(imx_sai7_mclk1_sels));
	clks[IMX8MP_CLK_AUDIOMIX_SAI7_MCLK2_SEL] = imx_dev_clk_mux(dev, "sai7_mclk2_sel", base + 0x314, 1, 4, imx_sai_mclk2_sels, ARRAY_SIZE(imx_sai_mclk2_sels));
	clks[IMX8MP_CLK_AUDIOMIX_PDM_SEL]        = imx_dev_clk_mux_flags(dev, "pdm_sel", base + 0x318, 0, 2, imx_pdm_sels, ARRAY_SIZE(imx_pdm_sels), CLK_SET_RATE_PARENT);

	clks[IMX8MP_CLK_AUDIOMIX_SAI1_IPG]    = imx_dev_clk_gate(dev, "sai1_ipg_clk",   "ipg_audio_root", base, 0);
	clks[IMX8MP_CLK_AUDIOMIX_SAI1_MCLK1]  = imx_dev_clk_gate(dev, "sai1_mclk1_clk", "sai1_mclk1_sel", base, 1);
	clks[IMX8MP_CLK_AUDIOMIX_SAI1_MCLK2]  = imx_dev_clk_gate(dev, "sai1_mclk2_clk", "sai1_mclk2_sel", base, 2);
	clks[IMX8MP_CLK_AUDIOMIX_SAI1_MCLK3]  = imx_dev_clk_gate(dev, "sai1_mclk3_clk", "sai_pll_out", base, 3);
	clks[IMX8MP_CLK_AUDIOMIX_SAI2_IPG]    = imx_dev_clk_gate(dev, "sai2_ipg_clk",   "ipg_audio_root", base, 4);
	clks[IMX8MP_CLK_AUDIOMIX_SAI2_MCLK1]  = imx_dev_clk_gate(dev, "sai2_mclk1_clk", "sai2_mclk1_sel", base, 5);
	clks[IMX8MP_CLK_AUDIOMIX_SAI2_MCLK2]  = imx_dev_clk_gate(dev, "sai2_mclk2_clk", "sai2_mclk2_sel", base, 6);
	clks[IMX8MP_CLK_AUDIOMIX_SAI2_MCLK3]  = imx_dev_clk_gate(dev, "sai2_mclk3_clk", "sai_pll_out", base, 7);
	clks[IMX8MP_CLK_AUDIOMIX_SAI3_IPG]    = imx_dev_clk_gate(dev, "sai3_ipg_clk",   "ipg_audio_root", base, 8);
	clks[IMX8MP_CLK_AUDIOMIX_SAI3_MCLK1]  = imx_dev_clk_gate(dev, "sai3_mclk1_clk", "sai3_mclk1_sel", base, 9);
	clks[IMX8MP_CLK_AUDIOMIX_SAI3_MCLK2]  = imx_dev_clk_gate(dev, "sai3_mclk2_clk", "sai3_mclk2_sel", base, 10);
	clks[IMX8MP_CLK_AUDIOMIX_SAI3_MCLK3]  = imx_dev_clk_gate(dev, "sai3_mclk3_clk", "sai_pll_out", base, 11);
	clks[IMX8MP_CLK_AUDIOMIX_SAI5_IPG]    = imx_dev_clk_gate(dev, "sai5_ipg_clk",   "ipg_audio_root", base, 12);
	clks[IMX8MP_CLK_AUDIOMIX_SAI5_MCLK1]  = imx_dev_clk_gate(dev, "sai5_mclk1_clk", "sai5_mclk1_sel", base, 13);
	clks[IMX8MP_CLK_AUDIOMIX_SAI5_MCLK2]  = imx_dev_clk_gate(dev, "sai5_mclk2_clk", "sai5_mclk2_sel", base, 14);
	clks[IMX8MP_CLK_AUDIOMIX_SAI5_MCLK3]  = imx_dev_clk_gate(dev, "sai5_mclk3_clk", "sai_pll_out", base, 15);
	clks[IMX8MP_CLK_AUDIOMIX_SAI6_IPG]    = imx_dev_clk_gate(dev, "sai6_ipg_clk",   "ipg_audio_root", base, 16);
	clks[IMX8MP_CLK_AUDIOMIX_SAI6_MCLK1]  = imx_dev_clk_gate(dev, "sai6_mclk1_clk", "sai6_mclk1_sel", base, 17);
	clks[IMX8MP_CLK_AUDIOMIX_SAI6_MCLK2]  = imx_dev_clk_gate(dev, "sai6_mclk2_clk", "sai6_mclk2_sel", base, 18);
	clks[IMX8MP_CLK_AUDIOMIX_SAI6_MCLK3]  = imx_dev_clk_gate(dev, "sai6_mclk3_clk", "sai_pll_out", base, 19);
	clks[IMX8MP_CLK_AUDIOMIX_SAI7_IPG]    = imx_dev_clk_gate(dev, "sai7_ipg_clk",   "ipg_audio_root", base, 20);
	clks[IMX8MP_CLK_AUDIOMIX_SAI7_MCLK1]  = imx_dev_clk_gate(dev, "sai7_mclk1_clk", "sai7_mclk1_sel", base, 21);
	clks[IMX8MP_CLK_AUDIOMIX_SAI7_MCLK2]  = imx_dev_clk_gate(dev, "sai7_mclk2_clk", "sai7_mclk2_sel", base, 22);
	clks[IMX8MP_CLK_AUDIOMIX_SAI7_MCLK3]  = imx_dev_clk_gate(dev, "sai7_mclk3_clk", "sai_pll_out", base, 23);
	clks[IMX8MP_CLK_AUDIOMIX_ASRC_IPG]    = imx_dev_clk_gate(dev, "asrc_ipg_clk",   "ipg_audio_root", base, 24);
	clks[IMX8MP_CLK_AUDIOMIX_PDM_IPG]     = imx_dev_clk_gate_shared(dev, "pdm_ipg_clk", "ipg_audio_root", base, 25, &shared_count_pdm);
	clks[IMX8MP_CLK_AUDIOMIX_PDM_ROOT]    = imx_dev_clk_gate_shared(dev, "pdm_root_clk", "pdm_sel", base, 25, &shared_count_pdm);

	clks[IMX8MP_CLK_AUDIOMIX_SDMA3_ROOT]  = imx_dev_clk_gate(dev, "sdma3_root_clk", "ipg_audio_root", base, 27);
	clks[IMX8MP_CLK_AUDIOMIX_SPBA2_ROOT]  = imx_dev_clk_gate(dev, "spba2_root_clk", "ipg_audio_root", base, 28);
	clks[IMX8MP_CLK_AUDIOMIX_DSP_ROOT]    = imx_dev_clk_gate(dev, "dsp_root_clk",   "ipg_audio_root", base, 29);
	clks[IMX8MP_CLK_AUDIOMIX_DSPDBG_ROOT] = imx_dev_clk_gate(dev, "dsp_dbg_clk",    "ipg_audio_root", base, 30);
	clks[IMX8MP_CLK_AUDIOMIX_EARC_IPG]    = imx_dev_clk_gate(dev, "earc_ipg_clk",   "ipg_audio_root", base, 31);

	clks[IMX8MP_CLK_AUDIOMIX_OCRAMA_IPG]  = imx_dev_clk_gate(dev, "ocram_a_ipg_clk", "ipg_audio_root", base + 4, 0);
	clks[IMX8MP_CLK_AUDIOMIX_AUD2HTX_IPG] = imx_dev_clk_gate(dev, "aud2htx_ipg_clk", "ipg_audio_root", base + 4, 1);
	clks[IMX8MP_CLK_AUDIOMIX_EDMA_ROOT]   = imx_dev_clk_gate(dev, "edma_root_clk",   "ipg_audio_root", base + 4, 2);
	clks[IMX8MP_CLK_AUDIOMIX_AUDPLL_ROOT] = imx_dev_clk_gate(dev, "aud_pll_clk",  "ipg_audio_root", base + 4, 3);
	clks[IMX8MP_CLK_AUDIOMIX_MU2_ROOT]    = imx_dev_clk_gate(dev, "mu2_root_clk", "ipg_audio_root", base + 4, 4);
	clks[IMX8MP_CLK_AUDIOMIX_MU3_ROOT]    = imx_dev_clk_gate(dev, "mu3_root_clk", "ipg_audio_root", base + 4, 5);
	clks[IMX8MP_CLK_AUDIOMIX_EARC_PHY]    = imx_dev_clk_gate(dev, "earc_phy_clk", "sai_pll_out", base + 4, 6);

	/* unbypass the pll */
	clk_set_parent(clks[IMX8MP_CLK_AUDIOMIX_SAI_PLL_BYPASS], clks[IMX8MP_CLK_AUDIOMIX_SAI_PLL]);

	imx_check_clocks(clks, ARRAY_SIZE(clks));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
	if (ret < 0) {
		pr_err("failed to register clks for i.MX8MP\n");
		return -EINVAL;
	}

	pm_runtime_put_sync(dev);

	return 0;
}

static const struct dev_pm_ops imx_audiomix_clk_pm_ops = {
	SET_RUNTIME_PM_OPS(imx_audiomix_clk_runtime_suspend,
			   imx_audiomix_clk_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct of_device_id imx_audiomix_clk_of_match[] = {
	{ .compatible = "fsl,imx8mp-audiomix-clk" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx_audiomix_clk_of_match);


static struct platform_driver imx_audiomix_clk_driver = {
	.probe = imx_audiomix_clk_probe,
	.driver = {
		.name = "imx-audiomix-clk",
		.of_match_table = of_match_ptr(imx_audiomix_clk_of_match),
		.pm = &imx_audiomix_clk_pm_ops,
	},
};
module_platform_driver(imx_audiomix_clk_driver);
MODULE_LICENSE("GPL v2");
