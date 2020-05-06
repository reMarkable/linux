// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX drm driver - LVDS display bridge
 *
 * Copyright (C) 2012 Sascha Hauer, Pengutronix
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>

#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>

#include "imx-drm.h"

#define DRIVER_NAME "imx-ldb"

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_BGREF_RMODE_INT		(1 << 15)

struct imx_ldb;

struct imx_ldb_channel {
	struct ldb_channel base;
	struct imx_ldb *imx_ldb;

	struct drm_connector connector;
	struct drm_encoder encoder;

	struct i2c_adapter *ddc;
	void *edid;
	int edid_len;
	struct drm_display_mode mode;
	int mode_valid;
	u32 bus_flags;
};

static inline struct imx_ldb_channel *con_to_imx_ldb_ch(struct drm_connector *c)
{
	return container_of(c, struct imx_ldb_channel, connector);
}

static inline struct imx_ldb_channel *enc_to_imx_ldb_ch(struct drm_encoder *e)
{
	return container_of(e, struct imx_ldb_channel, encoder);
}

struct bus_mux {
	int reg;
	int shift;
	int mask;
};

struct imx_ldb {
	struct ldb base;
	struct imx_ldb_channel channel[2];
	struct clk *clk[2]; /* our own clock */
	struct clk *clk_sel[4]; /* parent of display clock */
	struct clk *clk_parent[4]; /* original parent of clk_sel */
	struct clk *clk_pll[2]; /* upstream clock we can adjust */
	const struct bus_mux *lvds_mux;
};

static int imx_ldb_connector_get_modes(struct drm_connector *connector)
{
	struct imx_ldb_channel *imx_ldb_ch = con_to_imx_ldb_ch(connector);
	int num_modes = 0;

	if (!imx_ldb_ch->edid && imx_ldb_ch->ddc)
		imx_ldb_ch->edid = drm_get_edid(connector, imx_ldb_ch->ddc);

	if (imx_ldb_ch->edid) {
		drm_connector_update_edid_property(connector,
							imx_ldb_ch->edid);
		num_modes = drm_add_edid_modes(connector, imx_ldb_ch->edid);
	}

	if (imx_ldb_ch->mode_valid) {
		struct drm_display_mode *mode;

		mode = drm_mode_create(connector->dev);
		if (!mode)
			return -EINVAL;
		drm_mode_copy(mode, &imx_ldb_ch->mode);
		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
		num_modes++;
	}

	return num_modes;
}

static struct drm_encoder *imx_ldb_connector_best_encoder(
		struct drm_connector *connector)
{
	struct imx_ldb_channel *imx_ldb_ch = con_to_imx_ldb_ch(connector);

	return &imx_ldb_ch->encoder;
}

static void imx_ldb_set_clock(struct imx_ldb *imx_ldb, int mux, int chno,
		unsigned long serial_clk, unsigned long di_clk)
{
	struct ldb *ldb = &imx_ldb->base;
	int ret;

	dev_dbg(ldb->dev, "%s: now: %ld want: %ld\n", __func__,
			clk_get_rate(imx_ldb->clk_pll[chno]), serial_clk);
	clk_set_rate(imx_ldb->clk_pll[chno], serial_clk);

	dev_dbg(ldb->dev, "%s after: %ld\n", __func__,
			clk_get_rate(imx_ldb->clk_pll[chno]));

	dev_dbg(ldb->dev, "%s: now: %ld want: %ld\n", __func__,
			clk_get_rate(imx_ldb->clk[chno]),
			(long int)di_clk);
	clk_set_rate(imx_ldb->clk[chno], di_clk);

	dev_dbg(ldb->dev, "%s after: %ld\n", __func__,
			clk_get_rate(imx_ldb->clk[chno]));

	/* set display clock mux to LDB input clock */
	ret = clk_set_parent(imx_ldb->clk_sel[mux], imx_ldb->clk[chno]);
	if (ret)
		dev_err(ldb->dev,
			"unable to set di%d parent clock to ldb_di%d\n", mux,
			chno);
}

static void imx_ldb_encoder_enable(struct drm_encoder *encoder)
{
	struct imx_ldb_channel *imx_ldb_ch = enc_to_imx_ldb_ch(encoder);
	struct imx_ldb *imx_ldb = imx_ldb_ch->imx_ldb;
	struct ldb_channel *ldb_ch = &imx_ldb_ch->base;
	struct ldb *ldb = &imx_ldb->base;
	int mux = drm_of_encoder_active_port_id(ldb_ch->child, encoder);

	if (ldb->dual) {
		clk_set_parent(imx_ldb->clk_sel[mux], imx_ldb->clk[0]);
		clk_set_parent(imx_ldb->clk_sel[mux], imx_ldb->clk[1]);

		clk_prepare_enable(imx_ldb->clk[0]);
		clk_prepare_enable(imx_ldb->clk[1]);
	} else {
		clk_set_parent(imx_ldb->clk_sel[mux],
			       imx_ldb->clk[ldb_ch->chno]);
	}

	if (imx_ldb_ch == &imx_ldb->channel[0] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
		if (mux == 0 || imx_ldb->lvds_mux)
			ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI0;
		else if (mux == 1)
			ldb->ldb_ctrl |= LDB_CH0_MODE_EN_TO_DI1;
	}
	if (imx_ldb_ch == &imx_ldb->channel[1] || ldb->dual) {
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;
		if (mux == 1 || imx_ldb->lvds_mux)
			ldb->ldb_ctrl |= LDB_CH1_MODE_EN_TO_DI1;
		else if (mux == 0)
			ldb->ldb_ctrl |= LDB_CH1_MODE_EN_TO_DI0;
	}

	if (imx_ldb->lvds_mux) {
		const struct bus_mux *lvds_mux = NULL;

		if (imx_ldb_ch == &imx_ldb->channel[0])
			lvds_mux = &imx_ldb->lvds_mux[0];
		else if (imx_ldb_ch == &imx_ldb->channel[1])
			lvds_mux = &imx_ldb->lvds_mux[1];

		regmap_update_bits(ldb->regmap, lvds_mux->reg, lvds_mux->mask,
				   mux << lvds_mux->shift);
	}
}

static void
imx_ldb_encoder_atomic_mode_set(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *connector_state)
{
	struct imx_ldb_channel *imx_ldb_ch = enc_to_imx_ldb_ch(encoder);
	struct imx_ldb *imx_ldb = imx_ldb_ch->imx_ldb;
	struct ldb_channel *ldb_ch = &imx_ldb_ch->base;
	struct ldb *ldb = &imx_ldb->base;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	unsigned long serial_clk;
	unsigned long di_clk = mode->clock * 1000;
	int mux = drm_of_encoder_active_port_id(ldb_ch->child, encoder);

	if (mode->clock > 170000) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 170 MHz pixel clock\n", __func__);
	}
	if (mode->clock > 85000 && !ldb->dual) {
		dev_warn(ldb->dev,
			 "%s: mode exceeds 85 MHz pixel clock\n", __func__);
	}

	if (ldb->dual) {
		serial_clk = 3500UL * mode->clock;
		imx_ldb_set_clock(imx_ldb, mux, 0, serial_clk, di_clk);
		imx_ldb_set_clock(imx_ldb, mux, 1, serial_clk, di_clk);
	} else {
		serial_clk = 7000UL * mode->clock;
		imx_ldb_set_clock(imx_ldb, mux, ldb_ch->chno, serial_clk,
				  di_clk);
	}

	if (!ldb_ch->bus_format) {
		struct drm_connector *connector = connector_state->connector;
		struct drm_display_info *di = &connector->display_info;

		if (di->num_bus_formats)
			ldb_ch->bus_format = di->bus_formats[0];
	}
}

static void imx_ldb_encoder_disable(struct drm_encoder *encoder)
{
	struct imx_ldb_channel *imx_ldb_ch = enc_to_imx_ldb_ch(encoder);
	struct imx_ldb *imx_ldb = imx_ldb_ch->imx_ldb;
	struct ldb *ldb = &imx_ldb->base;
	int mux, ret;

	if (ldb->dual) {
		clk_disable_unprepare(imx_ldb->clk[0]);
		clk_disable_unprepare(imx_ldb->clk[1]);
	}

	if (imx_ldb->lvds_mux) {
		const struct bus_mux *lvds_mux = NULL;

		if (imx_ldb_ch == &imx_ldb->channel[0])
			lvds_mux = &imx_ldb->lvds_mux[0];
		else if (imx_ldb_ch == &imx_ldb->channel[1])
			lvds_mux = &imx_ldb->lvds_mux[1];

		regmap_read(ldb->regmap, lvds_mux->reg, &mux);
		mux &= lvds_mux->mask;
		mux >>= lvds_mux->shift;
	} else {
		mux = (imx_ldb_ch == &imx_ldb->channel[0]) ? 0 : 1;
	}

	/* set display clock mux back to original input clock */
	ret = clk_set_parent(imx_ldb->clk_sel[mux], imx_ldb->clk_parent[mux]);
	if (ret)
		dev_err(ldb->dev,
			"unable to set di%d parent clock to original parent\n",
			mux);
}

static int imx_ldb_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx_ldb_channel *imx_ldb_ch = enc_to_imx_ldb_ch(encoder);
	struct ldb_channel *ldb_ch = &imx_ldb_ch->base;
	struct drm_display_info *di = &conn_state->connector->display_info;
	u32 bus_format = ldb_ch->bus_format;

	/* Bus format description in DT overrides connector display info. */
	if (!bus_format && di->num_bus_formats) {
		bus_format = di->bus_formats[0];
		imx_crtc_state->bus_flags = di->bus_flags;
	} else {
		bus_format = ldb_ch->bus_format;
		imx_crtc_state->bus_flags = imx_ldb_ch->bus_flags;
	}
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB666_1X18;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	default:
		return -EINVAL;
	}

	imx_crtc_state->di_hsync_pin = 2;
	imx_crtc_state->di_vsync_pin = 3;

	return 0;
}

static const struct drm_connector_funcs imx_ldb_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = imx_drm_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs imx_ldb_connector_helper_funcs = {
	.get_modes = imx_ldb_connector_get_modes,
	.best_encoder = imx_ldb_connector_best_encoder,
};

static const struct drm_encoder_funcs imx_ldb_encoder_funcs = {
	.destroy = imx_drm_encoder_destroy,
};

static const struct drm_encoder_helper_funcs imx_ldb_encoder_helper_funcs = {
	.atomic_mode_set = imx_ldb_encoder_atomic_mode_set,
	.enable = imx_ldb_encoder_enable,
	.disable = imx_ldb_encoder_disable,
	.atomic_check = imx_ldb_encoder_atomic_check,
};

static int imx_ldb_get_clk(struct imx_ldb *imx_ldb, int chno)
{
	struct ldb *ldb = &imx_ldb->base;
	struct device *dev = ldb->dev;
	char clkname[16];

	snprintf(clkname, sizeof(clkname), "di%d", chno);
	imx_ldb->clk[chno] = devm_clk_get(dev, clkname);
	if (IS_ERR(imx_ldb->clk[chno]))
		return PTR_ERR(imx_ldb->clk[chno]);

	snprintf(clkname, sizeof(clkname), "di%d_pll", chno);
	imx_ldb->clk_pll[chno] = devm_clk_get(dev, clkname);

	return PTR_ERR_OR_ZERO(imx_ldb->clk_pll[chno]);
}

static struct bus_mux imx6q_lvds_mux[2] = {
	{
		.reg = IOMUXC_GPR3,
		.shift = 6,
		.mask = IMX6Q_GPR3_LVDS0_MUX_CTL_MASK,
	}, {
		.reg = IOMUXC_GPR3,
		.shift = 8,
		.mask = IMX6Q_GPR3_LVDS1_MUX_CTL_MASK,
	}
};

/*
 * For a device declaring compatible = "fsl,imx6q-ldb",  "fsl,imx53-ldb",
 * of_match_device will walk through this  list and take the first entry
 * matching any of its compatible values. Therefore, the more generic
 * entries (in this case fsl,imx53-ldb) need to be ordered last.
 */
static const struct of_device_id imx_ldb_dt_ids[] = {
	{ .compatible = "fsl,imx6q-ldb", .data = imx6q_lvds_mux, },
	{ .compatible = "fsl,imx53-ldb", .data = NULL, },
	{ }
};
MODULE_DEVICE_TABLE(of, imx_ldb_dt_ids);

static int imx_ldb_panel_ddc(struct device *dev,
		struct imx_ldb_channel *imx_ldb_ch, struct device_node *child)
{
	struct ldb_channel *ldb_ch = &imx_ldb_ch->base;
	struct device_node *ddc_node;
	const u8 *edidp;
	int ret;

	ddc_node = of_parse_phandle(child, "ddc-i2c-bus", 0);
	if (ddc_node) {
		imx_ldb_ch->ddc = of_find_i2c_adapter_by_node(ddc_node);
		of_node_put(ddc_node);
		if (!imx_ldb_ch->ddc) {
			dev_warn(dev, "failed to get ddc i2c adapter\n");
			return -EPROBE_DEFER;
		}
	}

	if (!imx_ldb_ch->ddc) {
		/* if no DDC available, fallback to hardcoded EDID */
		dev_dbg(dev, "no ddc available\n");

		edidp = of_get_property(child, "edid",
					&imx_ldb_ch->edid_len);
		if (edidp) {
			imx_ldb_ch->edid = kmemdup(edidp,
						imx_ldb_ch->edid_len,
						GFP_KERNEL);
		} else if (!ldb_ch->panel) {
			/* fallback to display-timings node */
			ret = of_get_drm_display_mode(child,
						      &imx_ldb_ch->mode,
						      &imx_ldb_ch->bus_flags,
						      OF_USE_NATIVE_MODE);
			if (!ret)
				imx_ldb_ch->mode_valid = 1;
		}
	}
	return 0;
}

static int imx_ldb_register(struct drm_device *drm,
	struct imx_ldb_channel *imx_ldb_ch)
{
	struct imx_ldb *imx_ldb = imx_ldb_ch->imx_ldb;
	struct ldb *ldb = &imx_ldb->base;
	struct ldb_channel *ldb_ch = &imx_ldb_ch->base;
	struct drm_encoder *encoder = &imx_ldb_ch->encoder;
	int ret;

	ret = imx_drm_encoder_parse_of(drm, encoder, ldb_ch->child);
	if (ret)
		return ret;

	ret = imx_ldb_get_clk(imx_ldb, ldb_ch->chno);
	if (ret)
		return ret;

	if (ldb->dual) {
		ret = imx_ldb_get_clk(imx_ldb, 1);
		if (ret)
			return ret;
	}

	if (!ldb_ch->next_bridge) {
		/* panel ddc only if there is no bridge */
		ret = imx_ldb_panel_ddc(ldb->dev, imx_ldb_ch, ldb_ch->child);
		if (ret)
			return ret;

		/*
		 * We want to add the connector whenever there is no bridge
		 * that brings its own, not only when there is a panel. For
		 * historical reasons, the ldb driver can also work without
		 * a panel.
		 */
		drm_connector_helper_add(&imx_ldb_ch->connector,
				&imx_ldb_connector_helper_funcs);
		drm_connector_init(drm, &imx_ldb_ch->connector,
				&imx_ldb_connector_funcs,
				DRM_MODE_CONNECTOR_LVDS);
		drm_connector_attach_encoder(&imx_ldb_ch->connector, encoder);
	}

	return 0;
}

static int imx_ldb_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	const struct of_device_id *of_id =
			of_match_device(imx_ldb_dt_ids, dev);
	struct imx_ldb *imx_ldb = dev_get_drvdata(dev);
	struct ldb *ldb;
	struct ldb_channel *ldb_ch;
	struct drm_encoder *encoder[LDB_CH_NUM];
	int ret;
	int i;

	imx_ldb->lvds_mux = of_id ? of_id->data : NULL;

	ldb = &imx_ldb->base;
	ldb->dev = dev;
	ldb->ctrl_reg = IOMUXC_GPR2;
	/*
	 * The output port is port@4 with an external 4-port mux or
	 * port@2 with the internal 2-port mux or port@1 without mux.
	 */
	ldb->output_port = imx_ldb->lvds_mux ? 4 : 2;

	for (i = 0; i < LDB_CH_NUM; i++) {
		imx_ldb->channel[i].imx_ldb = imx_ldb;
		ldb->channel[i] = &imx_ldb->channel[i].base;
	}

	/*
	 * There are three different possible clock mux configurations:
	 * i.MX53:  ipu1_di0_sel, ipu1_di1_sel
	 * i.MX6q:  ipu1_di0_sel, ipu1_di1_sel, ipu2_di0_sel,
	 *          ipu2_di1_sel
	 * i.MX6dl: ipu1_di0_sel, ipu1_di1_sel, lcdif_sel
	 * Map them all to di0_sel...di3_sel.
	 */
	for (i = 0; i < 4; i++) {
		char clkname[16];

		sprintf(clkname, "di%d_sel", i);
		imx_ldb->clk_sel[i] = devm_clk_get(dev, clkname);
		if (IS_ERR(imx_ldb->clk_sel[i])) {
			ret = PTR_ERR(imx_ldb->clk_sel[i]);
			imx_ldb->clk_sel[i] = NULL;
			break;
		}

		imx_ldb->clk_parent[i] =
				clk_get_parent(imx_ldb->clk_sel[i]);
	}
	if (i == 0)
		return ret;

	for (i = 0; i < LDB_CH_NUM; i++) {
		encoder[i] = &imx_ldb->channel[i].encoder;

		drm_encoder_helper_add(encoder[i],
				      &imx_ldb_encoder_helper_funcs);
		drm_encoder_init(drm, encoder[i], &imx_ldb_encoder_funcs,
				 DRM_MODE_ENCODER_LVDS, NULL);
	}

	ret = ldb_bind(ldb, encoder);
	if (ret)
		return ret;

	for (i = 0; i < LDB_CH_NUM; i++) {
		ldb_ch = &imx_ldb->channel[i].base;
		if (!ldb_ch->is_valid) {
			drm_encoder_cleanup(encoder[i]);
			continue;
		}

		ret = imx_ldb_register(drm, &imx_ldb->channel[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static void imx_ldb_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct imx_ldb *imx_ldb = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < LDB_CH_NUM; i++) {
		struct imx_ldb_channel *imx_ldb_ch = &imx_ldb->channel[i];

		kfree(imx_ldb_ch->edid);
		i2c_put_adapter(imx_ldb_ch->ddc);
	}
}

static const struct component_ops imx_ldb_ops = {
	.bind	= imx_ldb_bind,
	.unbind	= imx_ldb_unbind,
};

static int imx_ldb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx_ldb *imx_ldb;

	imx_ldb = devm_kzalloc(dev, sizeof(*imx_ldb), GFP_KERNEL);
	if (!imx_ldb)
		return -ENOMEM;

	dev_set_drvdata(dev, imx_ldb);

	return component_add(dev, &imx_ldb_ops);
}

static int imx_ldb_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx_ldb_ops);
	return 0;
}

static struct platform_driver imx_ldb_driver = {
	.probe		= imx_ldb_probe,
	.remove		= imx_ldb_remove,
	.driver		= {
		.of_match_table = imx_ldb_dt_ids,
		.name	= DRIVER_NAME,
	},
};

module_platform_driver(imx_ldb_driver);

MODULE_DESCRIPTION("i.MX LVDS driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
