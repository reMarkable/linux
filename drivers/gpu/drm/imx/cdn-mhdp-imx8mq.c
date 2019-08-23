/*
 * Copyright (C) 2019 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#include <drm/bridge/cdns-mhdp-imx.h>
#include "cdn-mhdp-phy.h"
#include "imx-drm.h"

struct imx_hdmi {
	struct device *dev;
	struct drm_encoder encoder;
};

static void cdns_hdmi_imx_encoder_disable(struct drm_encoder *encoder)
{
}

static void cdns_hdmi_imx_encoder_enable(struct drm_encoder *encoder)
{
}

static int cdns_hdmi_imx_atomic_check(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	return 0;
}

static const struct drm_encoder_helper_funcs cdns_hdmi_imx_encoder_helper_funcs = {
	.enable     = cdns_hdmi_imx_encoder_enable,
	.disable    = cdns_hdmi_imx_encoder_disable,
	.atomic_check = cdns_hdmi_imx_atomic_check,
};

static const struct drm_encoder_funcs cdns_hdmi_imx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static struct cdn_plat_data imx8mq_hdmi_drv_data = {
	.bind	= cdns_hdmi_bind,
	.unbind	= cdns_hdmi_unbind,
	.phy_init = cdns_hdmi_phy_set_imx8mq,
};

static struct cdn_plat_data imx8mq_dp_drv_data = {
	.bind	= cdns_dp_bind,
	.unbind	= cdns_dp_unbind,
	.phy_init = cdns_dp_phy_init_imx8mq,
};

static const struct of_device_id cdns_hdmi_imx_dt_ids[] = {
	{ .compatible = "cdn,imx8mq-hdmi",
	  .data = &imx8mq_hdmi_drv_data
	},
	{ .compatible = "cdn,imx8mq-dp",
	  .data = &imx8mq_dp_drv_data
	},
	{},
};
MODULE_DEVICE_TABLE(of, cdns_hdmi_imx_dt_ids);

static int cdns_hdmi_imx_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct cdn_plat_data *plat_data;
	const struct of_device_id *match;
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct imx_hdmi *hdmi;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	match = of_match_node(cdns_hdmi_imx_dt_ids, pdev->dev.of_node);
	plat_data = match->data;
	hdmi->dev = &pdev->dev;
	encoder = &hdmi->encoder;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_helper_add(encoder, &cdns_hdmi_imx_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &cdns_hdmi_imx_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	ret = plat_data->bind(pdev, encoder, plat_data);

	/*
	 * If cdns_hdmi_bind() fails we'll never call cdns_hdmi_unbind(),
	 * which would have called the encoder cleanup.  Do it manually.
	 */
	if (ret)
		drm_encoder_cleanup(encoder);

	return ret;
}

static void cdns_hdmi_imx_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct imx_mhdp_device *hdp = dev_get_drvdata(dev);

	hdp->plat_data->unbind(dev);
}

static const struct component_ops cdns_hdmi_imx_ops = {
	.bind	= cdns_hdmi_imx_bind,
	.unbind	= cdns_hdmi_imx_unbind,
};

static int cdns_hdmi_imx_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &cdns_hdmi_imx_ops);
}

static int cdns_hdmi_imx_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cdns_hdmi_imx_ops);

	return 0;
}

static struct platform_driver cdns_hdmi_imx_platform_driver = {
	.probe  = cdns_hdmi_imx_probe,
	.remove = cdns_hdmi_imx_remove,
	.driver = {
		.name = "cdn-hdp-imx8mq",
		.of_match_table = cdns_hdmi_imx_dt_ids,
	},
};

module_platform_driver(cdns_hdmi_imx_platform_driver);

MODULE_AUTHOR("Sandor YU <sandor.yu@nxp.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdnhdmi-imx");
