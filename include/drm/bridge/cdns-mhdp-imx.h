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

#define IRQ_IN    0
#define IRQ_OUT   1
#define IRQ_NUM   2

#define hdp_plat_call(hdp, operation)			\
	(!(hdp) ? -ENODEV : (((hdp)->plat_data && (hdp)->plat_data->operation) ?	\
	 (hdp)->plat_data->operation(hdp) : ENOIOCTLCMD))

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

struct cdn_plat_data {
	/* Vendor PHY support */
	int (*phy_init)(struct imx_mhdp_device *hdmi);
	int (*bind)(struct platform_device *pdev,
			struct drm_encoder *encoder,
		 	const struct cdn_plat_data *plat_data);
	void (*unbind)(struct device *dev);
	int (*fw_init)(struct imx_mhdp_device *hdp);
	void (*pclock_change)(struct imx_mhdp_device *hdp);
	char is_dp;
};

struct imx_mhdp_device {
	struct cdns_mhdp_device mhdp;

	struct mutex lock;
	struct mutex audio_mutex;
	spinlock_t audio_lock;
	bool connected;
	bool active;
	bool suspended;
	struct imx_hdp_clks clks;

	const struct cdn_plat_data *plat_data;

	int irq[IRQ_NUM];
	struct delayed_work hotplug_work;
	//void __iomem *regmap_csr;
	struct regmap *regmap_csr;
	u32 csr_pxl_mux_reg;
	u32 csr_ctrl0_reg;
	u32 csr_ctrl0_sec;

	struct audio_info audio_info;
	bool sink_has_audio;
	u32 dual_mode;

	struct device		*pd_mhdp_dev;
	struct device		*pd_pll0_dev;
	struct device		*pd_pll1_dev;
	struct device_link	*pd_mhdp_link;
	struct device_link	*pd_pll0_link;
	struct device_link	*pd_pll1_link;

	u32 phy_init;
};

int cdns_hdmi_probe(struct platform_device *pdev,
		  const struct cdn_plat_data *plat_data);
void cdns_hdmi_remove(struct platform_device *pdev);
void cdns_hdmi_unbind(struct device *dev);
int cdns_hdmi_bind(struct platform_device *pdev, struct drm_encoder *encoder,
		 const struct cdn_plat_data *plat_data);
void cdns_hdmi_set_sample_rate(struct imx_mhdp_device *hdmi, unsigned int rate);
void cdns_hdmi_audio_enable(struct imx_mhdp_device *hdmi);
void cdns_hdmi_audio_disable(struct imx_mhdp_device *hdmi);
int cdns_dp_probe(struct platform_device *pdev,
		  const struct cdn_plat_data *plat_data);
void cdns_dp_remove(struct platform_device *pdev);
void cdns_dp_unbind(struct device *dev);
int cdns_dp_bind(struct platform_device *pdev, struct drm_encoder *encoder,
		 const struct cdn_plat_data *plat_data);

#endif /* CDNS_MHDP_IMX_H_ */
