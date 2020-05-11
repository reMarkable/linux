// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "fsl_dsp_audiomix.h"

struct imx_audiomix_dsp_data {
	void __iomem *base;
};

void imx_audiomix_dsp_start(struct imx_audiomix_dsp_data *data)
{
	u32 val;

	val = readl(data->base + AudioDSP_REG2);
	val &= ~(1 << 5);
	writel(val, data->base + AudioDSP_REG2);
}
EXPORT_SYMBOL(imx_audiomix_dsp_start);

void imx_audiomix_dsp_pid_set(struct imx_audiomix_dsp_data *data, u32 val)
{
	writel(val, data->base + AudioDSP_REG3);
}
EXPORT_SYMBOL(imx_audiomix_dsp_pid_set);

bool imx_audiomix_dsp_reset(struct imx_audiomix_dsp_data *data)
{
	u32 val;

	val = readl(data->base + AudioDSP_REG3);
	if (val == 0)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(imx_audiomix_dsp_reset);

static int imx_audiomix_dsp_probe(struct platform_device *pdev)
{
	struct imx_audiomix_dsp_data *drvdata;
	struct device *dev = &pdev->dev;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata->base = dev_get_drvdata(dev->parent);

	platform_set_drvdata(pdev, drvdata);

	return 0;
}

static const struct of_device_id imx_audiomix_dsp_dt_ids[] = {
       { .compatible = "fsl,audiomix-dsp", },
       { /* sentinel */ },
};

static struct platform_driver imx_audiomix_dsp_driver = {
	.probe  = imx_audiomix_dsp_probe,
	.driver = {
		.name	= "audiomix-dsp",
		.of_match_table = imx_audiomix_dsp_dt_ids,
	},
};
module_platform_driver(imx_audiomix_dsp_driver);
