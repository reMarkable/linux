// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <drm/drm_of.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

struct dcss_drv {
	struct dcss_dev *dcss;
	struct dcss_kms_dev *kms;

	bool is_componentized;
};

struct dcss_dev *dcss_drv_dev_to_dcss(struct device *dev)
{
	struct dcss_drv *mdrv = dev_get_drvdata(dev);

	return mdrv ? mdrv->dcss : NULL;
}

struct drm_device *dcss_drv_dev_to_drm(struct device *dev)
{
	struct dcss_drv *mdrv = dev_get_drvdata(dev);

	return mdrv ? &mdrv->kms->base : NULL;
}

static int dcss_drv_init(struct device *dev, bool componentized)
{
	struct dcss_drv *mdrv;
	int err = 0;

	mdrv = devm_kzalloc(dev, sizeof(*mdrv), GFP_KERNEL);
	if (!mdrv)
		return -ENOMEM;

	mdrv->is_componentized = componentized;

	mdrv->dcss = dcss_dev_create(dev, componentized);
	if (IS_ERR(mdrv->dcss)) {
		err = PTR_ERR(mdrv->dcss);
		goto err;
	}

	dev_set_drvdata(dev, mdrv);

	mdrv->kms = dcss_kms_attach(mdrv->dcss, componentized);
	if (IS_ERR(mdrv->kms)) {
		err = PTR_ERR(mdrv->kms);
		goto dcss_shutoff;
	}

	return 0;

dcss_shutoff:
	dcss_dev_destroy(mdrv->dcss);

	dev_set_drvdata(dev, NULL);

err:
	devm_kfree(dev, mdrv);
	return err;
}

static void dcss_drv_deinit(struct device *dev, bool componentized)
{
	struct dcss_drv *mdrv = dev_get_drvdata(dev);

	if (!mdrv)
		return;

	dcss_kms_detach(mdrv->kms, componentized);
	dcss_dev_destroy(mdrv->dcss);

	dev_set_drvdata(dev, NULL);
}

static int dcss_drv_bind(struct device *dev)
{
	return dcss_drv_init(dev, true);
}

static void dcss_drv_unbind(struct device *dev)
{
	return dcss_drv_deinit(dev, true);
}

static const struct component_master_ops dcss_master_ops = {
	.bind	= dcss_drv_bind,
	.unbind	= dcss_drv_unbind,
};

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static int dcss_drv_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct device_node *remote;

	if (!dev->of_node)
		return -ENODEV;

	remote = of_graph_get_remote_node(dev->of_node, 0, 0);
	if (!remote)
		return -ENODEV;

	if (of_device_is_compatible(remote, "fsl,imx8mq-nwl-dsi")) {
		of_node_put(remote);
		return dcss_drv_init(dev, false);
	}

	drm_of_component_match_add(dev, &match, compare_of, remote);
	of_node_put(remote);

	return component_master_add_with_match(dev, &dcss_master_ops, match);
}

static int dcss_drv_platform_remove(struct platform_device *pdev)
{
	struct dcss_drv *mdrv = dev_get_drvdata(&pdev->dev);

	if (mdrv->is_componentized)
		component_master_del(&pdev->dev, &dcss_master_ops);
	else
		dcss_drv_deinit(&pdev->dev, false);

	return 0;
}

static struct dcss_type_data dcss_types[] = {
	[DCSS_IMX8MQ] = {
		.name = "DCSS_IMX8MQ",
		.blkctl_ofs = 0x2F000,
		.ctxld_ofs = 0x23000,
		.dtg_ofs = 0x20000,
		.rdsrc_ofs = 0x22000,
		.wrscl_ofs = 0x21000,
		.scaler_ofs = 0x1C000,
		.ss_ofs = 0x1B000,
		.dpr_ofs = 0x18000,
		.dec400d_ofs = 0x15000,
		.hdr10_ofs = 0x00000,
		.dtrc_ofs = 0x16000,
	},
};

static const struct of_device_id dcss_of_match[] = {
	{ .compatible = "nxp,imx8mq-dcss", .data = &dcss_types[DCSS_IMX8MQ], },
	{},
};

MODULE_DEVICE_TABLE(of, dcss_of_match);

static const struct dev_pm_ops dcss_dev_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(dcss_dev_suspend, dcss_dev_resume)
	SET_RUNTIME_PM_OPS(dcss_dev_runtime_suspend,
			   dcss_dev_runtime_resume, NULL)
};

static struct platform_driver dcss_platform_driver = {
	.probe	= dcss_drv_platform_probe,
	.remove	= dcss_drv_platform_remove,
	.driver	= {
		.name = "imx-dcss",
		.of_match_table	= dcss_of_match,
		.pm = &dcss_dev_pm,
	},
};

module_platform_driver(dcss_platform_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@nxp.com>");
MODULE_DESCRIPTION("DCSS driver for i.MX8MQ");
MODULE_LICENSE("GPL v2");
