// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8 NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/sys_soc.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "nwl-drv.h"
#include "nwl-dsi.h"

#define DRV_NAME "nwl-dsi"

/* Possible platform specific clocks */
#define NWL_DSI_CLK_CORE	"core"

static const struct regmap_config nwl_dsi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = NWL_DSI_IRQ_MASK2,
	.name = DRV_NAME,
};

struct nwl_dsi_platform_data {
	int (*poweron)(struct nwl_dsi *dsi);
	int (*poweroff)(struct nwl_dsi *dsi);
	int (*select_input)(struct nwl_dsi *dsi);
	int (*deselect_input)(struct nwl_dsi *dsi);
	struct nwl_dsi_plat_clk_config clk_config[NWL_DSI_MAX_PLATFORM_CLOCKS];
};

static inline struct nwl_dsi *bridge_to_dsi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct nwl_dsi, bridge);
}

static int nwl_dsi_set_platform_clocks(struct nwl_dsi *dsi, bool enable)
{
	struct device *dev = dsi->dev;
	const char *id;
	struct clk *clk;
	size_t i;
	unsigned long rate;
	int ret, result = 0;

	DRM_DEV_DEBUG_DRIVER(dev, "%s platform clocks\n",
			     enable ? "enabling" : "disabling");
	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;
		id = dsi->clk_config[i].id;
		clk = dsi->clk_config[i].clk;

		if (enable) {
			ret = clk_prepare_enable(clk);
			if (ret < 0) {
				DRM_DEV_ERROR(dev,
					      "Failed to enable %s clk: %d\n",
					      id, ret);
				result = result ?: ret;
			}
			rate = clk_get_rate(clk);
			DRM_DEV_DEBUG_DRIVER(dev, "Enabled %s clk @%lu Hz\n",
					     id, rate);
		} else {
			clk_disable_unprepare(clk);
			DRM_DEV_DEBUG_DRIVER(dev, "Disabled %s clk\n", id);
		}
	}

	return result;
}

static int nwl_dsi_plat_enable(struct nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;
	int ret;

	if (dsi->pdata->select_input)
		dsi->pdata->select_input(dsi);

	ret = nwl_dsi_set_platform_clocks(dsi, true);
	if (ret < 0)
		return ret;

	ret = dsi->pdata->poweron(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to power on DSI: %d\n", ret);
	return ret;
}

static void nwl_dsi_plat_disable(struct nwl_dsi *dsi)
{
	dsi->pdata->poweroff(dsi);
	nwl_dsi_set_platform_clocks(dsi, false);
	if (dsi->pdata->deselect_input)
		dsi->pdata->deselect_input(dsi);
}

static void nwl_dsi_bridge_disable(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);

	nwl_dsi_disable(dsi);
	nwl_dsi_plat_disable(dsi);
	pm_runtime_put(dsi->dev);
}

static int nwl_dsi_get_dphy_params(struct nwl_dsi *dsi,
				   const struct drm_display_mode *mode,
				   union phy_configure_opts *phy_opts)
{
	unsigned long rate;
	int ret;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	/*
	 * So far the DPHY spec minimal timings work for both mixel
	 * dphy and nwl dsi host
	 */
	ret = phy_mipi_dphy_get_default_config(
		mode->crtc_clock * 1000,
		mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes,
		&phy_opts->mipi_dphy);
	if (ret < 0)
		return ret;

	rate = clk_get_rate(dsi->tx_esc_clk);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "LP clk is @%lu Hz\n", rate);
	phy_opts->mipi_dphy.lp_clk_rate = rate;

	return 0;
}

static bool nwl_dsi_bridge_mode_fixup(struct drm_bridge *bridge,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	/* At least LCDIF + NWL needs active high sync */
	adjusted_mode->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	adjusted_mode->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

	return true;
}

static enum drm_mode_status
nwl_dsi_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_mode *mode)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	if (mode->clock * bpp > 15000000)
		return MODE_CLOCK_HIGH;

	if (mode->clock * bpp < 80000)
		return MODE_CLOCK_LOW;

	return MODE_OK;
}

static void
nwl_dsi_bridge_mode_set(struct drm_bridge *bridge,
			const struct drm_display_mode *mode,
			const struct drm_display_mode *adjusted_mode)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	struct device *dev = dsi->dev;
	union phy_configure_opts new_cfg;
	unsigned long phy_ref_rate;
	int ret;

	ret = nwl_dsi_get_dphy_params(dsi, adjusted_mode, &new_cfg);
	if (ret < 0)
		return;

	/*
	 * If hs clock is unchanged, we're all good - all parameters are
	 * derived from it atm.
	 */
	if (new_cfg.mipi_dphy.hs_clk_rate == dsi->phy_cfg.mipi_dphy.hs_clk_rate)
		return;

	phy_ref_rate = clk_get_rate(dsi->phy_ref_clk);
	DRM_DEV_DEBUG_DRIVER(dev, "PHY at ref rate: %lu\n", phy_ref_rate);
	/* Save the new desired phy config */
	memcpy(&dsi->phy_cfg, &new_cfg, sizeof(new_cfg));

	memcpy(&dsi->mode, adjusted_mode, sizeof(dsi->mode));
	drm_mode_debug_printmodeline(adjusted_mode);
}

static void nwl_dsi_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);

	pm_runtime_get_sync(dsi->dev);
	nwl_dsi_plat_enable(dsi);
	nwl_dsi_enable(dsi);
}

static int nwl_dsi_bridge_attach(struct drm_bridge *bridge)
{
	struct nwl_dsi *dsi = bridge->driver_private;

	return drm_bridge_attach(bridge->encoder, dsi->panel_bridge, bridge);
}

static const struct drm_bridge_funcs nwl_dsi_bridge_funcs = {
	.pre_enable = nwl_dsi_bridge_pre_enable,
	.disable    = nwl_dsi_bridge_disable,
	.mode_fixup = nwl_dsi_bridge_mode_fixup,
	.mode_set   = nwl_dsi_bridge_mode_set,
	.mode_valid = nwl_dsi_bridge_mode_valid,
	.attach	    = nwl_dsi_bridge_attach,
};

static int nwl_dsi_parse_dt(struct nwl_dsi *dsi)
{
	struct platform_device *pdev = to_platform_device(dsi->dev);
	struct clk *clk;
	const char *clk_id;
	void __iomem *base;
	int i, ret;

	dsi->phy = devm_phy_get(dsi->dev, "dphy");
	if (IS_ERR(dsi->phy)) {
		ret = PTR_ERR(dsi->phy);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dsi->dev, "Could not get PHY: %d\n", ret);
		return ret;
	}

	/* Platform dependent clocks */
	memcpy(dsi->clk_config, dsi->pdata->clk_config,
	       sizeof(dsi->pdata->clk_config));

	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;

		clk_id = dsi->clk_config[i].id;
		clk = devm_clk_get(dsi->dev, clk_id);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			DRM_DEV_ERROR(dsi->dev, "Failed to get %s clock: %d\n",
				      clk_id, ret);
			return ret;
		}
		DRM_DEV_DEBUG_DRIVER(dsi->dev, "Setup clk %s (rate: %lu)\n",
				     clk_id, clk_get_rate(clk));
		dsi->clk_config[i].clk = clk;
	}

	/* DSI clocks */
	clk = devm_clk_get(dsi->dev, "phy_ref");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get phy_ref clock: %d\n",
			      ret);
		return ret;
	}
	dsi->phy_ref_clk = clk;

	clk = devm_clk_get(dsi->dev, "rx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get rx_esc clock: %d\n",
			      ret);
		return ret;
	}
	dsi->rx_esc_clk = clk;

	clk = devm_clk_get(dsi->dev, "tx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		DRM_DEV_ERROR(dsi->dev, "Failed to get tx_esc clock: %d\n",
			      ret);
		return ret;
	}
	dsi->tx_esc_clk = clk;

	dsi->mux = devm_mux_control_get(dsi->dev, NULL);
	if (IS_ERR(dsi->mux)) {
		ret = PTR_ERR(dsi->mux);
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dsi->dev, "Failed to get mux: %d\n", ret);
		return ret;
	}

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dsi->regmap =
		devm_regmap_init_mmio(dsi->dev, base, &nwl_dsi_regmap_config);
	if (IS_ERR(dsi->regmap)) {
		ret = PTR_ERR(dsi->regmap);
		DRM_DEV_ERROR(dsi->dev, "Failed to create NWL DSI regmap: %d\n",
			      ret);
		return ret;
	}

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get device IRQ: %d\n",
			      dsi->irq);
		return dsi->irq;
	}

	dsi->rstc = devm_reset_control_array_get(dsi->dev, false, true);
	if (IS_ERR(dsi->rstc)) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get resets: %ld\n",
			      PTR_ERR(dsi->rstc));
		return PTR_ERR(dsi->rstc);
	}

	return 0;
}

static int imx8mq_dsi_select_input(struct nwl_dsi *dsi)
{
	struct device_node *remote;
	u32 use_dcss = 1;
	int ret;

	remote = of_graph_get_remote_node(dsi->dev->of_node, 0, 0);
	if (strcmp(remote->name, "lcdif") == 0)
		use_dcss = 0;

	DRM_DEV_INFO(dsi->dev, "Using %s as input source\n",
		     (use_dcss) ? "DCSS" : "LCDIF");

	ret = mux_control_try_select(dsi->mux, use_dcss);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to select input: %d\n", ret);

	of_node_put(remote);
	return ret;
}


static int imx8mq_dsi_deselect_input(struct nwl_dsi *dsi)
{
	int ret;

	ret = mux_control_deselect(dsi->mux);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to deselect input: %d\n", ret);

	return ret;
}


static int imx8mq_dsi_poweron(struct nwl_dsi *dsi)
{
	int ret = 0;

	/* otherwise the display stays blank */
	usleep_range(200, 300);

	if (dsi->rstc)
		ret = reset_control_deassert(dsi->rstc);

	return ret;
}

static int imx8mq_dsi_poweroff(struct nwl_dsi *dsi)
{
	int ret = 0;

	if (dsi->quirks & SRC_RESET_QUIRK)
		return 0;

	if (dsi->rstc)
		ret = reset_control_assert(dsi->rstc);
	return ret;
}

static const struct drm_bridge_timings nwl_dsi_timings = {
	.input_bus_flags = DRM_BUS_FLAG_DE_LOW,
};

static const struct nwl_dsi_platform_data imx8mq_dev = {
	.poweron = &imx8mq_dsi_poweron,
	.poweroff = &imx8mq_dsi_poweroff,
	.select_input = &imx8mq_dsi_select_input,
	.deselect_input = &imx8mq_dsi_deselect_input,
	.clk_config = {
		{ .id = NWL_DSI_CLK_CORE, .present = true },
	},
};

static const struct of_device_id nwl_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx8mq-nwl-dsi", .data = &imx8mq_dev, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nwl_dsi_dt_ids);

static const struct soc_device_attribute nwl_dsi_quirks_match[] = {
	{ .soc_id = "i.MX8MQ", .revision = "2.0",
	  .data = (void *)(E11418_HS_MODE_QUIRK | SRC_RESET_QUIRK) },
	{ /* sentinel. */ },
};

static int nwl_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id = of_match_device(nwl_dsi_dt_ids, dev);
	const struct nwl_dsi_platform_data *pdata = of_id->data;
	const struct soc_device_attribute *attr;
	struct nwl_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;
	dsi->pdata = pdata;

	ret = nwl_dsi_parse_dt(dsi);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, dsi->irq, nwl_dsi_irq_handler, 0,
			       dev_name(dev), dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to request IRQ %d: %d\n", dsi->irq,
			      ret);
		return ret;
	}

	dsi->dsi_host.ops = &nwl_dsi_host_ops;
	dsi->dsi_host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->dsi_host);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register MIPI host: %d\n", ret);
		return ret;
	}

	attr = soc_device_match(nwl_dsi_quirks_match);
	if (attr)
		dsi->quirks = (uintptr_t)attr->data;

	dsi->bridge.driver_private = dsi;
	dsi->bridge.funcs = &nwl_dsi_bridge_funcs;
	dsi->bridge.of_node = dev->of_node;
	dsi->bridge.timings = &nwl_dsi_timings;

	drm_bridge_add(&dsi->bridge);

	dev_set_drvdata(dev, dsi);
	pm_runtime_enable(dev);
	return 0;
}

static int nwl_dsi_remove(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsi->dsi_host);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static struct platform_driver nwl_dsi_driver = {
	.probe		= nwl_dsi_probe,
	.remove		= nwl_dsi_remove,
	.driver		= {
		.of_match_table = nwl_dsi_dt_ids,
		.name	= DRV_NAME,
	},
};

module_platform_driver(nwl_dsi_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_AUTHOR("Purism SPC");
MODULE_DESCRIPTION("Northwest Logic MIPI-DSI driver");
MODULE_LICENSE("GPL"); /* GPLv2 or later */
