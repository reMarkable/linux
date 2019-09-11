/*
 * Cadence High-Definition Multimedia Interface (HDMI) driver
 *
 * Copyright (C) 2019 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef CDNS_MHDP_IMX_H_
#define CDNS_MHDP_IMX_H_

#include <drm/bridge/cdns-mhdp-common.h>
#include <drm/drm_encoder_slave.h>


#define HDP_DUAL_MODE_MIN_PCLK_RATE	300000	/* KHz */
#define HDP_SINGLE_MODE_MAX_WIDTH	1920

static inline bool video_is_dual_mode(const struct drm_display_mode *mode)
{
	return (mode->clock > HDP_DUAL_MODE_MIN_PCLK_RATE ||
		mode->hdisplay > HDP_SINGLE_MODE_MAX_WIDTH) ? true : false;
}

struct imx_mhdp_device;

struct imx_hdp_clks {
	struct clk *av_pll;
	struct clk *dig_pll;
	struct clk *clk_ipg;
	struct clk *clk_core;
	struct clk *clk_pxl;
	struct clk *clk_pxl_mux;
	struct clk *clk_pxl_link;

	struct clk *lpcg_hdp;
	struct clk *lpcg_msi;
	struct clk *lpcg_pxl;
	struct clk *lpcg_vif;
	struct clk *lpcg_lis;
	struct clk *lpcg_apb;
	struct clk *lpcg_apb_csr;
	struct clk *lpcg_apb_ctrl;

	struct clk *lpcg_i2s;
	struct clk *clk_i2s_bypass;
};

struct imx_mhdp_device {
	struct cdns_mhdp_device mhdp;
	struct drm_encoder encoder;

	struct mutex audio_mutex;
	spinlock_t audio_lock;
	bool connected;
	bool active;
	bool suspended;
	struct imx_hdp_clks clks;

	int bus_type;

	u32 dual_mode;

	struct device		*pd_mhdp_dev;
	struct device		*pd_pll0_dev;
	struct device		*pd_pll1_dev;
	struct device_link	*pd_mhdp_link;
	struct device_link	*pd_pll0_link;
	struct device_link	*pd_pll1_link;

//	u32 phy_init;
};
void cdns_mhdp_plat_init_imx8qm(struct cdns_mhdp_device *mhdp);
void cdns_mhdp_plat_deinit_imx8qm(struct cdns_mhdp_device *mhdp);
void cdns_mhdp_pclk_rate_imx8qm(struct cdns_mhdp_device *mhdp);
int cdns_mhdp_firmware_init_imx8qm(struct cdns_mhdp_device *mhdp);
#endif /* CDNS_MHDP_IMX_H_ */
