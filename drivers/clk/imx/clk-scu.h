/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018 NXP
 *   Dong Aisheng <aisheng.dong@nxp.com>
 */

#ifndef __IMX_CLK_SCU_H
#define __IMX_CLK_SCU_H

#include <linux/firmware/imx/sci.h>
#include <linux/of.h>

extern u32 clock_cells;
extern struct list_head imx_scu_clks[];

int imx_clk_scu_init(struct device_node *np);
struct clk_hw *imx_scu_of_clk_src_get(struct of_phandle_args *clkspec,
				      void *data);
struct clk_hw *imx_clk_scu_alloc_dev(const char *name,
				     const char * const *parents,
				     int num_parents, u32 rsrc_id, u8 clk_type);

struct clk_hw *__imx_clk_scu(struct device *dev, const char *name,
			     const char * const *parents, int num_parents,
			     u32 rsrc_id, u8 clk_type);

static inline struct clk_hw *imx_clk_scu(const char *name, u32 rsrc_id,
					 u8 clk_type)
{
	if (clock_cells == 2)
		return imx_clk_scu_alloc_dev(name, NULL, 0, rsrc_id, clk_type);
	else
		return __imx_clk_scu(NULL, name, NULL, 0, rsrc_id, clk_type);
}

static inline struct clk_hw *imx_clk_scu2(const char *name, const char * const *parents,
					  int num_parents, u32 rsrc_id, u8 clk_type)
{
	if (clock_cells == 2)
		return imx_clk_scu_alloc_dev(name, parents, num_parents, rsrc_id, clk_type);
	else
		return __imx_clk_scu(NULL, name, parents, num_parents, rsrc_id, clk_type);
}

struct clk_hw *imx_clk_scu3(const char *name, const char *parent_name,
			    u32 rsrc_id, u8 gpr_id, bool invert_flag);

struct clk_hw *imx_clk_lpcg_scu(const char *name, const char *parent_name,
				unsigned long flags, void __iomem *reg,
				u8 bit_idx, bool hw_gate);

struct clk_hw *imx_clk_divider_gpr_scu(const char *name, const char *parent_name,
				       u32 rsrc_id, u8 gpr_id);

struct clk_hw *imx_clk_mux_gpr_scu(const char *name, const char **parents,
				   int num_parents, u32 rsrc_id, u8 gpr_id);

#endif
