// SPDX-License-Identifier: GPL-2.0
//
// Freescale Generic DAI driver for DSP
//
// Copyright 2019 NXP
// Author: Daniel Baluta <daniel.baluta@nxp.com>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

enum fsl_dai_type {
	FSL_DAI_TYPE_NONE,
	FSL_DAI_TYPE_SAI,
	FSL_DAI_TYPE_ESAI,
};

#define FSL_DAI_ESAI_CLK_NUM	4
static const char *esai_clks[FSL_DAI_ESAI_CLK_NUM] = {
	"core",
	"extal",
	"fsys",
	"spba",
};

#define FSL_DAI_SAI_CLK_NUM	5
static const char *sai_clks[FSL_DAI_SAI_CLK_NUM] = {
	"bus",
	"mclk0",
	"mclk1",
	"mclk2",
	"mclk3",
};

struct fsl_dai {
	struct platform_device *pdev;

	/* DAI clocks */
	struct clk **clks;
	const char **clk_names;
	int num_clks;

	/* Power Domain handling */
	int num_domains;
	struct device **pd_dev;
	struct device_link **link;

	/* DAIS */
	struct snd_soc_dai_driver *dai_drv;
	int num_drv;
};

static struct snd_soc_dai_driver fsl_esai_dai = {
	.name = "esai0",
};

static struct snd_soc_dai_driver fsl_sai_dai = {
	.name = "sai1",
};

static const struct snd_soc_component_driver fsl_dai_component = {
	.name = "fsl-dai",
};

static int fsl_dai_init_clocks(struct fsl_dai *dai_priv)
{
	struct device *dev = &dai_priv->pdev->dev;
	int i;

	dai_priv->clks = devm_kcalloc(dev, dai_priv->num_clks,
				      sizeof(*dai_priv->clks), GFP_KERNEL);
	if (!dai_priv->clks)
		return -ENOMEM;

	for (i = 0; i < dai_priv->num_clks; i++) {
		dai_priv->clks[i] = devm_clk_get(dev, dai_priv->clk_names[i]);
		if (IS_ERR(dai_priv->clks[i])) {
			dev_dbg(dev, "Failed to get clk %s\n",
				dai_priv->clk_names[i]);
			dai_priv->clks[i] = NULL;
		}
	}

	return 0;
}

int fsl_get_dai_type(struct fsl_dai *dai_priv)
{
	struct device_node *np = dai_priv->pdev->dev.of_node;

	if (of_device_is_compatible(np, "fsl,esai-dai"))
		return FSL_DAI_TYPE_ESAI;

	if (of_device_is_compatible(np, "fsl,sai-dai"))
		return FSL_DAI_TYPE_SAI;

	return FSL_DAI_TYPE_NONE;
}

static int fsl_dai_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_dai *priv;
	int dai_type;
	int ret;
	int i;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	dev_set_drvdata(&pdev->dev, priv);

	dai_type = fsl_get_dai_type(priv);
	switch (dai_type) {
	case FSL_DAI_TYPE_ESAI:
		priv->clk_names = esai_clks;
		priv->num_clks = FSL_DAI_ESAI_CLK_NUM;
		priv->dai_drv = &fsl_esai_dai;
		priv->num_drv = 1;
		break;
	case FSL_DAI_TYPE_SAI:
		priv->clk_names = sai_clks;
		priv->num_clks = FSL_DAI_SAI_CLK_NUM;
		priv->dai_drv = &fsl_sai_dai;
		priv->num_drv = 1;
		break;
	default:
		dev_err(&pdev->dev, "Invalid DAI type %d\n", dai_type);
		return -EINVAL;
	}

	ret = fsl_dai_init_clocks(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error at init clocks\n");
		return ret;
	}

	priv->num_domains = of_count_phandle_with_args(np, "power-domains",
						       "#power-domain-cells");
	if (priv->num_domains < 0) {
		dev_err(&pdev->dev, "no power-domains property in %pOF\n", np);
		return priv->num_domains;
	}

	priv->pd_dev = devm_kmalloc_array(&pdev->dev, priv->num_domains,
					  sizeof(*priv->pd_dev), GFP_KERNEL);
	if (!priv->pd_dev)
		return -ENOMEM;

	priv->link = devm_kmalloc_array(&pdev->dev, priv->num_domains,
					sizeof(*priv->link), GFP_KERNEL);
	if (!priv->link)
		return -ENOMEM;

	for (i = 0; i < priv->num_domains; i++) {
		priv->pd_dev[i] = dev_pm_domain_attach_by_id(&pdev->dev, i);
		if (IS_ERR(priv->pd_dev[i])) {
			ret = PTR_ERR(priv->pd_dev[i]);
			goto unroll_pm;
		}

		priv->link[i] = device_link_add(&pdev->dev, priv->pd_dev[i],
						DL_FLAG_STATELESS |
						DL_FLAG_PM_RUNTIME |
						DL_FLAG_RPM_ACTIVE);
		if (!priv->link[i]) {
			ret = -EINVAL;
			dev_pm_domain_detach(priv->pd_dev[i], false);
			goto unroll_pm;
		}
	}

	pm_runtime_enable(&pdev->dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_dai_component,
					      priv->dai_drv, priv->num_drv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register DAI ret = %d\n", ret);
		return ret;
	}
	return 0;

unroll_pm:
	while (--i >= 0) {
		device_link_del(priv->link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}
	return ret;
}

static int fsl_dai_remove(struct platform_device *pdev)
{
	struct fsl_dai *priv = platform_get_drvdata(pdev);
	int i;

	pm_runtime_disable(&priv->pdev->dev);

	for (i = 0; i < priv->num_domains; i++) {
		device_link_del(priv->link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}

	return 0;
}

static const struct of_device_id fsl_dai_dt_ids[] = {
	{ .compatible = "fsl,esai-dai", },
	{ .compatible = "fsl,sai-dai", },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_dai_dt_ids);

#ifdef CONFIG_PM
static int fsl_dai_runtime_resume(struct device *dev)
{
	struct fsl_dai *priv = dev_get_drvdata(dev);
	int i, ret;

	for (i = 0; i < priv->num_clks; i++) {
		ret = clk_prepare_enable(priv->clks[i]);
		if (ret < 0) {
			dev_err(dev, "Failed to enable clk %s\n",
				priv->clk_names[i]);
			goto out;
		}
	}
	return 0;
out:
	while (--i >= 0)
		clk_disable_unprepare(priv->clks[i]);

	return ret;
}

static int fsl_dai_runtime_suspend(struct device *dev)
{
	struct fsl_dai *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < priv->num_clks; i++)
		clk_disable_unprepare(priv->clks[i]);

	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops fsl_dai_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_dai_runtime_suspend,
			   fsl_dai_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver fsl_dai_driver = {
	.probe = fsl_dai_probe,
	.remove = fsl_dai_remove,
	.driver = {
		.name = "fsl-dai",
		.pm = &fsl_dai_pm_ops,
		.of_match_table = fsl_dai_dt_ids,
	},
};

module_platform_driver(fsl_dai_driver);

MODULE_ALIAS("platform:fsl-dai");

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@nxp.com>");
MODULE_DESCRIPTION("FSL Generic DAI driver for DSP");
MODULE_LICENSE("GPL v2");
