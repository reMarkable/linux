/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#ifndef __NWL_DRV_H__
#define __NWL_DRV_H__

#include <linux/mux/consumer.h>
#include <linux/phy/phy.h>

#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>

struct nwl_dsi_platform_data;

/* i.MX8 NWL quirks */
/* i.MX8MQ errata E11418 */
#define E11418_HS_MODE_QUIRK	    BIT(0)
/* Skip DSI bits in SRC on disable to avoid blank display on enable */
#define SRC_RESET_QUIRK		    BIT(1)

/* * DPI color coding */
#define NWL_DSI_DPI_16_BIT_565_PACKED	0
#define NWL_DSI_DPI_16_BIT_565_ALIGNED	1
#define NWL_DSI_DPI_16_BIT_565_SHIFTED	2
#define NWL_DSI_DPI_18_BIT_PACKED	3
#define NWL_DSI_DPI_18_BIT_ALIGNED	4
#define NWL_DSI_DPI_24_BIT		5


#define NWL_DSI_MAX_PLATFORM_CLOCKS 2
struct nwl_dsi_plat_clk_config {
	const char *id;
	struct clk *clk;
	bool present;
};

struct mode_config {
	int				clock;
	int				crtc_clock;
	unsigned int			lanes;
	unsigned long			bitclock;
	unsigned long			phy_rates[3];
	unsigned long			pll_rates[3];
	int				phy_rate_idx;
	struct list_head		list;
};

struct nwl_dsi {
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge *panel_bridge;
	struct device *dev;
	struct phy *phy;
	union phy_configure_opts phy_cfg;
	unsigned int quirks;
	unsigned int instance;

	struct regmap *regmap;
	struct regmap *csr;
	int irq;
	struct reset_control *rst_byte;
	struct reset_control *rst_esc;
	struct reset_control *rst_dpi;
	struct reset_control *rst_pclk;
	struct mux_control *mux;

	/* DSI clocks */
	struct clk *phy_ref_clk;
	struct clk *rx_esc_clk;
	struct clk *tx_esc_clk;
	struct clk *pll_clk;
	struct clk *lcdif_clk;
	/* Platform dependent clocks */
	struct nwl_dsi_plat_clk_config clk_config[NWL_DSI_MAX_PLATFORM_CLOCKS];

	struct list_head valid_modes;
	/* dsi lanes */
	u32 lanes;
	u32 clk_drop_lvl;
	enum mipi_dsi_pixel_format format;
	struct drm_display_mode mode;
	unsigned long dsi_mode_flags;

	struct nwl_dsi_transfer *xfer;

	const struct nwl_dsi_platform_data *pdata;

	bool use_dcss;
};

#endif /* __NWL_DRV_H__ */
