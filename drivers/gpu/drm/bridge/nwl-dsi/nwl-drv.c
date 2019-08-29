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

/* Maximum Video PLL frequency */
#define MAX_PLL_FREQ 1200000000

#define MBPS(x) ((x) * 1000000)
#define MIN_PHY_RATE MBPS(24)
#define MAX_PHY_RATE MBPS(30)

/* Possible valid PHY reference clock rates*/
static u32 phyref_rates[] = {
	27000000,
	25000000,
	24000000,
};

enum nwl_dsi_ext_regs {
       NWL_DSI_IMX_REG_GPR = BIT(1),
};

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

static unsigned long nwl_dsi_get_bit_clock(struct nwl_dsi *dsi,
		unsigned long pixclock, u32 lanes)
{
	int bpp;

	if (lanes < 1 || lanes > 4)
		return 0;

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	return (pixclock * bpp) / lanes;
}

/*
 * Utility function to calculate least commom multiple, using an improved
 * version of the Euclidean algorithm for greatest common factor.
 */
static unsigned long nwl_dsi_get_lcm(unsigned long a, unsigned long b)
{
	u32 gcf = 0; /* greatest common factor */
	unsigned long tmp_a = a;
	unsigned long tmp_b = b;

	if (!a || !b)
		return 0;

	while (tmp_a % tmp_b) {
		gcf = tmp_a % tmp_b;
		tmp_a = tmp_b;
		tmp_b = gcf;
	}

	if (!gcf)
		return a;

	return ((unsigned long long)a * b) / gcf;
}

/*
 * This function tries to adjust the crtc_clock for a DSI device in such a way
 * that the video pll will be able to satisfy both Display Controller pixel
 * clock (feeding out DPI interface) and our input phy_ref clock.
 * Also, the DC pixel clock must be lower than the actual clock in order to
 * have enough blanking space to send DSI commands, if the device is a panel.
 */
static void nwl_dsi_setup_pll_config(struct mode_config *config, u32 lvl)
{
	unsigned long pll_rate;
	int div;
	size_t i, num_rates = ARRAY_SIZE(config->phy_rates);

	config->crtc_clock = 0;

	for (i = 0; i < num_rates; i++) {
		int crtc_clock;

		if (!config->phy_rates[i])
			break;
		/*
		 * First, we need to check if phy_ref can actually be obtained
		 * from pixel clock. To do this, we check their lowest common
		 * multiple, which has to be in PLL range.
		 */
		pll_rate = nwl_dsi_get_lcm(config->clock, config->phy_rates[i]);
		if (pll_rate > MAX_PLL_FREQ) {
			/* Drop pll_rate to a realistic value */
			while (pll_rate > MAX_PLL_FREQ)
				pll_rate >>= 1;
			/* Make sure pll_rate can provide phy_ref rate */
			div = DIV_ROUND_UP(pll_rate, config->phy_rates[i]);
			pll_rate = config->phy_rates[i] * div;
		} else {
			/*
			 * Increase the pll rate to highest possible rate for
			 * better accuracy.
			 */
			while (pll_rate <= MAX_PLL_FREQ)
				pll_rate <<= 1;
			pll_rate >>= 1;
		}

		/*
		 * Next, we need to tweak the pll_rate to a value that can also
		 * satisfy the crtc_clock.
		 */
		div = DIV_ROUND_CLOSEST(pll_rate, config->clock);
		if (lvl)
			pll_rate -= config->phy_rates[i] * lvl;
		crtc_clock = pll_rate / div;
		config->pll_rates[i] = pll_rate;

		/*
		 * Pick a crtc_clock which is closest to pixel clock.
		 * Also, make sure that the pixel clock is a multiply of
		 * 50Hz.
		 */
		if (!(crtc_clock % 50) &&
		    abs(config->clock - crtc_clock) <
		    abs(config->clock - config->crtc_clock)) {
			config->crtc_clock = crtc_clock;
			config->phy_rate_idx = i;
		}
	}
}


/*
 * This function will try the required phy speed for current mode
 * If the phy speed can be achieved, the phy will save the speed
 * configuration
 */
static struct mode_config *nwl_dsi_mode_probe(struct nwl_dsi *dsi,
			    const struct drm_display_mode *mode)
{
	struct device *dev = dsi->dev;
	struct mode_config *config;
	union phy_configure_opts phy_opts;
	unsigned long clock = mode->clock * 1000;
	unsigned long bit_clk = 0;
	unsigned long phy_rates[3] = {0};
	int match_rates = 0;
	u32 lanes = dsi->lanes;
	size_t i = 0, num_rates = ARRAY_SIZE(phyref_rates);

	list_for_each_entry(config, &dsi->valid_modes, list)
		if (config->clock == clock)
			return config;

	phy_mipi_dphy_get_default_config(clock,
			mipi_dsi_pixel_format_to_bpp(dsi->format),
			lanes, &phy_opts.mipi_dphy);
	phy_opts.mipi_dphy.lp_clk_rate = clk_get_rate(dsi->tx_esc_clk);

	while (i < num_rates) {
		int ret;

		bit_clk = nwl_dsi_get_bit_clock(dsi, clock, lanes);

		clk_set_rate(dsi->pll_clk, phyref_rates[i] * 32);
		clk_set_rate(dsi->phy_ref_clk, phyref_rates[i]);
		ret = phy_validate(dsi->phy, PHY_MODE_MIPI_DPHY, 0, &phy_opts);

		/* Pick the non-failing rate, and search for more */
		if (!ret) {
			phy_rates[match_rates++] = phyref_rates[i++];
			continue;
		}

		if (match_rates)
			break;

		/* Reached the end of phyref_rates, try another lane config */
		if ((i++ == num_rates - 1) && (--lanes > 2)) {
			i = 0;
			continue;
		}
	}

	/*
	 * Try swinging between min and max pll rates and see what rate (in terms
	 * of kHz) we can custom use to get the required bit-clock.
	 */
	if (!match_rates) {
		int min_div, max_div;
		int bit_clk_khz;

		lanes = dsi->lanes;
		bit_clk = nwl_dsi_get_bit_clock(dsi, clock, lanes);

		min_div = DIV_ROUND_UP(bit_clk, MAX_PHY_RATE);
		max_div = DIV_ROUND_DOWN_ULL(bit_clk, MIN_PHY_RATE);
		bit_clk_khz = bit_clk / 1000;

		for (i = max_div; i > min_div; i--) {
			if (!(bit_clk_khz % i)) {
				phy_rates[0] = bit_clk / i;
				match_rates = 1;
				break;
			}
		}
	}

	if (!match_rates) {
		DRM_DEV_DEBUG_DRIVER(dev,
			"Cannot setup PHY for mode: %ux%u @%d kHz\n",
			mode->hdisplay,
			mode->vdisplay,
			mode->clock);

		return NULL;
	}

	config = devm_kzalloc(dsi->dev, sizeof(struct mode_config), GFP_KERNEL);
	config->clock = clock;
	config->lanes = lanes;
	config->bitclock = bit_clk;
	memcpy(&config->phy_rates, &phy_rates, sizeof(phy_rates));
	list_add(&config->list, &dsi->valid_modes);

	return config;
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
		mode->clock * 1000,
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
				      struct drm_display_mode *adjusted)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	struct mode_config *config;
	unsigned long pll_rate;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Fixup mode:\n");
	drm_mode_debug_printmodeline(adjusted);

	config = nwl_dsi_mode_probe(dsi, adjusted);
	if (!config)
		return false;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "lanes=%u, data_rate=%lu\n",
			     config->lanes, config->bitclock);
	if (config->lanes < 2 || config->lanes > 4)
		return false;

	/* Max data rate for this controller is 1.5Gbps */
	if (config->bitclock > 1500000000)
		return false;

	pll_rate = config->pll_rates[config->phy_rate_idx];
	if (dsi->pll_clk && pll_rate) {
		clk_set_rate(dsi->pll_clk, pll_rate);
		DRM_DEV_DEBUG_DRIVER(dsi->dev,
				     "Video pll rate: %lu (actual: %lu)",
				     pll_rate, clk_get_rate(dsi->pll_clk));
	}
	/* Update the crtc_clock to be used by display controller */
	if (config->crtc_clock)
		adjusted->crtc_clock = config->crtc_clock / 1000;


	/* At least LCDIF + NWL needs active high sync */
	adjusted->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	adjusted->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

	return true;
}

static enum drm_mode_status
nwl_dsi_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_mode *mode)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	struct mode_config *config;
	unsigned long pll_rate;
	int bit_rate;

	bit_rate = nwl_dsi_get_bit_clock(dsi, mode->clock * 1000, dsi->lanes);

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Validating mode:");
	drm_mode_debug_printmodeline(mode);

	if (bit_rate > MBPS(1500))
		return MODE_CLOCK_HIGH;

	if (bit_rate < MBPS(80))
		return MODE_CLOCK_LOW;

	config = nwl_dsi_mode_probe(dsi, mode);
	if (!config)
		return MODE_NOCLOCK;

	pll_rate = config->pll_rates[config->phy_rate_idx];
	if (dsi->pll_clk && !pll_rate)
		nwl_dsi_setup_pll_config(config, dsi->clk_drop_lvl);

	return MODE_OK;
}

static void
nwl_dsi_bridge_mode_set(struct drm_bridge *bridge,
			const struct drm_display_mode *mode,
			const struct drm_display_mode *adjusted)
{
	struct nwl_dsi *dsi = bridge_to_dsi(bridge);
	struct device *dev = dsi->dev;
	union phy_configure_opts new_cfg;
	unsigned long phy_ref_rate;
	struct mode_config *config;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "Setting mode:\n");
	drm_mode_debug_printmodeline(adjusted);

	config = nwl_dsi_mode_probe(dsi, adjusted);
	/* New mode? This should NOT happen */
	if (!config) {
		DRM_DEV_ERROR(dsi->dev, "Unsupported mode provided:\n");
		drm_mode_debug_printmodeline(adjusted);
		return;
	}

	phy_ref_rate = config->phy_rates[config->phy_rate_idx];
	clk_set_rate(dsi->phy_ref_clk, phy_ref_rate);
	ret = nwl_dsi_get_dphy_params(dsi, adjusted, &new_cfg);
	if (ret < 0)
		return;

	/*
	 * If hs clock is unchanged, we're all good - all parameters are
	 * derived from it atm.
	 */
	if (new_cfg.mipi_dphy.hs_clk_rate == dsi->phy_cfg.mipi_dphy.hs_clk_rate)
		return;

	DRM_DEV_DEBUG_DRIVER(dev,
			     "PHY at ref rate: %lu (actual: %lu)\n",
			     phy_ref_rate, clk_get_rate(dsi->phy_ref_clk));
	/* Save the new desired phy config */
	memcpy(&dsi->phy_cfg, &new_cfg, sizeof(new_cfg));

	memcpy(&dsi->mode, adjusted, sizeof(dsi->mode));
	drm_mode_debug_printmodeline(adjusted);
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
	struct device_node *np = dsi->dev->of_node;
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

	/* The video_pll clock is optional */
	clk = devm_clk_get(dsi->dev, "video_pll");
	if (!IS_ERR(clk))
		dsi->pll_clk = clk;

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

	of_property_read_u32(np, "fsl,clock-drop-level", &dsi->clk_drop_lvl);

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

	INIT_LIST_HEAD(&dsi->valid_modes);

	dev_set_drvdata(dev, dsi);
	pm_runtime_enable(dev);
	return 0;
}

static int nwl_dsi_remove(struct platform_device *pdev)
{
	struct nwl_dsi *dsi = platform_get_drvdata(pdev);
	struct mode_config *config;
	struct list_head *pos, *tmp;

	list_for_each_safe(pos, tmp, &dsi->valid_modes) {
		config = list_entry(pos, struct mode_config, list);
		list_del(pos);
		devm_kfree(dsi->dev, config);
	}

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
