// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "clk.h"

/**
 * struct clk_gate_shared - i.MX specific gate clock having the gate flag
 * shared with other gate clocks
 */
struct clk_gate_shared {
	struct clk_gate gate;
	spinlock_t	*lock;
	unsigned int	*share_count;
};

static int clk_gate_shared_enable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	struct clk_gate_shared *shgate = container_of(gate,
					struct clk_gate_shared, gate);
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(shgate->lock, flags);

	if (shgate->share_count && (*shgate->share_count)++ > 0)
		goto out;

	ret = clk_gate_ops.enable(hw);
out:
	spin_unlock_irqrestore(shgate->lock, flags);

	return ret;
}

static void clk_gate_shared_disable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	struct clk_gate_shared *shgate = container_of(gate,
					struct clk_gate_shared, gate);
	unsigned long flags = 0;

	spin_lock_irqsave(shgate->lock, flags);

	if (shgate->share_count) {
		if (WARN_ON(*shgate->share_count == 0))
			goto out;
		else if (--(*shgate->share_count) > 0)
			goto out;
	}

	clk_gate_ops.disable(hw);
out:
	spin_unlock_irqrestore(shgate->lock, flags);
}

static int clk_gate_shared_is_enabled(struct clk_hw *hw)
{
	return clk_gate_ops.is_enabled(hw);
}

static const struct clk_ops clk_gate_shared_ops = {
	.enable = clk_gate_shared_enable,
	.disable = clk_gate_shared_disable,
	.is_enabled = clk_gate_shared_is_enabled,
};

struct clk *imx_dev_clk_gate_shared(struct device *dev, const char *name,
				const char *parent, void __iomem *reg,
				u8 shift, unsigned int *share_count)
{
	struct clk_gate_shared *shgate;
	struct clk_gate *gate;
	struct clk *clk;
	struct clk_init_data init;

	shgate = kzalloc(sizeof(*shgate), GFP_KERNEL);
	if (!shgate)
		return ERR_PTR(-ENOMEM);
	gate = &shgate->gate;

	init.name = name;
	init.ops = &clk_gate_shared_ops;
	init.flags = CLK_OPS_PARENT_ENABLE | CLK_SET_RATE_PARENT;
	init.parent_names = parent ? &parent : NULL;
	init.num_parents = parent ? 1 : 0;

	gate->reg = reg;
	gate->bit_idx = shift;
	gate->lock = NULL;
	gate->hw.init = &init;
	shgate->lock = &imx_ccm_lock;
	shgate->share_count = share_count;

	clk = clk_register(dev, &gate->hw);
	if (IS_ERR(clk))
		kfree(shgate);

	return clk;
}
EXPORT_SYMBOL_GPL(imx_dev_clk_gate_shared);
MODULE_LICENSE("GPL v2");
