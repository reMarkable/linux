// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 */

#include <linux/of_address.h>
#include <linux/clk.h>
#include <dt-bindings/clock/s32v234-clock.h>

#include "clk.h"

static void __iomem *mc_cgm0_base;
static void __iomem *mc_me_base;
static void __iomem *src_base;

DEFINE_SPINLOCK(s32v234_lock);

/* sources for multiplexer clocks, this is used multiple times */
PNAME(osc_sels) = {"firc", "fxosc", };

PNAME(lin_sels) = {"firc", "fxosc", "dummy",
		   "periphpll_phi0_div3", "dummy", "dummy",
		   "dummy", "dummy", "sys6",};

PNAME(sdhc_sels) = {"firc", "fxosc", "dummy",
		    "dummy",};

static struct clk *clk[S32V234_CLK_END];
static struct clk_onecell_data clk_data;

static void __init s32v234_clocks_init(struct device_node *mc_cgm0_node)
{
	struct device_node *np;

	clk[S32V234_CLK_DUMMY] = s32_clk_fixed("dummy", 0);
	clk[S32V234_CLK_FXOSC] = s32_obtain_fixed_clock("fxosc", 0);
	clk[S32V234_CLK_FIRC] = s32_obtain_fixed_clock("firc", 0);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_me");
	mc_me_base = of_iomap(np, 0);
	if (WARN_ON(!mc_me_base))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-src");
	src_base = of_iomap(np, 0);
	if (WARN_ON(!src_base))
		return;

	np = mc_cgm0_node;
	mc_cgm0_base = of_iomap(np, 0);
	if (WARN_ON(!mc_cgm0_base))
		return;

	enable_cpumodes_onperipheralconfig(mc_me_base, MC_ME_RUN_PCn_DRUN |
					    MC_ME_RUN_PCn_RUN0 |
					    MC_ME_RUN_PCn_RUN1 |
					    MC_ME_RUN_PCn_RUN2 |
					    MC_ME_RUN_PCn_RUN3,
					    0);

	/* turn on XOSC and FIRC */
	enable_clocks_sources(MC_ME_MODE_MC_MVRON, MC_ME_MODE_MC_XOSCON |
			      MC_ME_MODE_MC_FIRCON,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* transition the core to RUN0 mode */
	entry_to_target_mode(mc_me_base, MC_ME_MCTL_RUN0);

	clk[S32V234_CLK_PERIPHPLL_SRC_SEL] = s32_clk_mux("periphpll_sel",
		SRC_GPR1, SRC_GPR1_PERIPHPLL_SRC_SEL_OFFSET,
		SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels), &s32v234_lock);

	/* PERIPH_PLL */
	clk[S32V234_CLK_PERIPHPLL_VCO] = s32v234_clk_plldig(S32_PLLDIG_PERIPH,
		"periphpll_vco", "periphpll_sel",
		PERIPHPLL_PLLDIG(mc_cgm0_base),
		PERIPHPLL_PLLDIG_PLLDV_MFD, PERIPHPLL_PLLDIG_PLLDV_MFN,
		PERIPHPLL_PLLDIG_PLLDV_RFDPHI0,
		PERIPHPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_PERIPHPLL_PHI0] =
		s32v234_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi0", "periphpll_vco",
		PERIPHPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_PERIPHPLL_PHI1] =
		s32v234_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi1", "periphpll_vco",
		PERIPHPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV3] = s32_clk_fixed_factor(
		"periphpll_phi0_div3", "periphpll_phi0", 1, 3);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV5] = s32_clk_fixed_factor(
		"periphpll_phi0_div5", "periphpll_phi0", 1, 5);

	/* Lin Clock */
	clk[S32V234_CLK_LIN_SEL] = s32_clk_mux("lin_sel",
		CGM_ACn_SC(mc_cgm0_base, 3),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		lin_sels, ARRAY_SIZE(lin_sels), &s32v234_lock);

	clk[S32V234_CLK_LIN] = s32_clk_divider("lin", "lin_sel",
		CGM_ACn_DCm(mc_cgm0_base, 3, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE, &s32v234_lock);

	clk[S32V234_CLK_LIN_IPG] = s32_clk_fixed_factor("lin_ipg",
		"lin", 1, 2);

	/* enable PERIPHPLL */
	enable_clocks_sources(0, MC_ME_MODE_MC_PERIPHPLL,
			      MC_ME_RUNn_MC(mc_me_base, 0));

	/* SDHC Clock */
	clk[S32V234_CLK_SDHC_SEL] = s32_clk_mux("sdhc_sel",
		CGM_ACn_SC(mc_cgm0_base, 15),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		sdhc_sels, ARRAY_SIZE(sdhc_sels), &s32v234_lock);

	clk[S32V234_CLK_SDHC] = s32_clk_divider("sdhc", "sdhc_sel",
		CGM_ACn_DCm(mc_cgm0_base, 15, 0),
		MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE, &s32v234_lock);

	/* set the system clock */
	enable_sysclock(MC_ME_MODE_MC_SYSCLK(0x2),
			MC_ME_RUNn_MC(mc_me_base, 0));

	/* transition the core to RUN0 mode */
	entry_to_target_mode(mc_me_base, MC_ME_MCTL_RUN0);

	/* Add the clocks to provider list */
	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(S32V234, "fsl,s32v234-mc_cgm0", s32v234_clocks_init);
