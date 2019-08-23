/*
 * Copyright (C) 2019 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <linux/firmware/imx/sci.h>
#include <linux/regmap.h>
#include <linux/pm_domain.h>

#include <drm/bridge/cdns-mhdp-imx.h>
#include "cdn-mhdp-phy.h"
#include "imx-drm.h"

#define CSR_PIXEL_LINK_MUX_CTL		0x00
#define PL_MUX_CTL_VCP_OFFSET		5
#define PL_MUX_CTL_HCP_OFFSET		4

#define PLL_800MHZ (800000000)

struct imx_hdmi {
	struct device *dev;
	struct drm_encoder encoder;
};

static void imx8qm_pixel_link_mux(struct imx_mhdp_device *hdp)
{
	struct drm_display_mode *mode = &hdp->mhdp.mode;
	u32 val;

	val = 0x4;	/* RGB */
	if (hdp->dual_mode)
		val |= 0x2;	/* pixel link 0 and 1 are active */
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		val |= 1 << PL_MUX_CTL_VCP_OFFSET;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		val |= 1 << PL_MUX_CTL_HCP_OFFSET;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		val |= 0x2;

	regmap_write(hdp->regmap_csr, hdp->csr_pxl_mux_reg, val);
}

static void imx8qm_pixel_link_valid(u32 dual_mode)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST1_VLD, 1);
	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST2_VLD, 1);
}

static void imx8qm_pixel_link_invalid(u32 dual_mode)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST1_VLD, 0);
	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST2_VLD, 0);
}

static void imx8qm_pixel_link_sync_enable(u32 dual_mode)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL, 3);
	else
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL0, 1);
}

static void imx8qm_pixel_link_sync_disable(u32 dual_mode)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL, 0);
	else
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL0, 0);
}

static void imx8qm_phy_reset(u8 reset)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	/* set the pixel link mode and pixel type */
	imx_sc_misc_set_control(handle, IMX_SC_R_HDMI, IMX_SC_C_PHY_RESET, reset);
}

static void imx8qm_clk_mux(u8 is_dp)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	if (is_dp)
		/* Enable the 24MHz for HDP PHY */
		imx_sc_misc_set_control(handle, IMX_SC_R_HDMI, IMX_SC_C_MODE, 1);
	else
		imx_sc_misc_set_control(handle, IMX_SC_R_HDMI, IMX_SC_C_MODE, 0);
}

int imx8qm_clocks_init(struct imx_mhdp_device *hdp)
{
	struct device *dev = hdp->mhdp.dev;
	struct imx_hdp_clks *clks = &hdp->clks;

	clks->dig_pll = devm_clk_get(dev, "dig_pll");
	if (IS_ERR(clks->dig_pll)) {
		dev_warn(dev, "failed to get dig pll clk\n");
		return PTR_ERR(clks->dig_pll);
	}

	clks->av_pll = devm_clk_get(dev, "av_pll");
	if (IS_ERR(clks->av_pll)) {
		dev_warn(dev, "failed to get av pll clk\n");
		return PTR_ERR(clks->av_pll);
	}

	clks->clk_ipg = devm_clk_get(dev, "clk_ipg");
	if (IS_ERR(clks->clk_ipg)) {
		dev_warn(dev, "failed to get dp ipg clk\n");
		return PTR_ERR(clks->clk_ipg);
	}

	clks->clk_core = devm_clk_get(dev, "clk_core");
	if (IS_ERR(clks->clk_core)) {
		dev_warn(dev, "failed to get hdp core clk\n");
		return PTR_ERR(clks->clk_core);
	}

	clks->clk_pxl = devm_clk_get(dev, "clk_pxl");
	if (IS_ERR(clks->clk_pxl)) {
		dev_warn(dev, "failed to get pxl clk\n");
		return PTR_ERR(clks->clk_pxl);
	}

	clks->clk_pxl_mux = devm_clk_get(dev, "clk_pxl_mux");
	if (IS_ERR(clks->clk_pxl_mux)) {
		dev_warn(dev, "failed to get pxl mux clk\n");
		return PTR_ERR(clks->clk_pxl_mux);
	}

	clks->clk_pxl_link = devm_clk_get(dev, "clk_pxl_link");
	if (IS_ERR(clks->clk_pxl_mux)) {
		dev_warn(dev, "failed to get pxl link clk\n");
		return PTR_ERR(clks->clk_pxl_link);
	}

	clks->lpcg_hdp = devm_clk_get(dev, "lpcg_hdp");
	if (IS_ERR(clks->lpcg_hdp)) {
		dev_warn(dev, "failed to get lpcg hdp clk\n");
		return PTR_ERR(clks->lpcg_hdp);
	}

	clks->lpcg_msi = devm_clk_get(dev, "lpcg_msi");
	if (IS_ERR(clks->lpcg_msi)) {
		dev_warn(dev, "failed to get lpcg msi clk\n");
		return PTR_ERR(clks->lpcg_msi);
	}

	clks->lpcg_pxl = devm_clk_get(dev, "lpcg_pxl");
	if (IS_ERR(clks->lpcg_pxl)) {
		dev_warn(dev, "failed to get lpcg pxl clk\n");
		return PTR_ERR(clks->lpcg_pxl);
	}

	clks->lpcg_vif = devm_clk_get(dev, "lpcg_vif");
	if (IS_ERR(clks->lpcg_vif)) {
		dev_warn(dev, "failed to get lpcg vif clk\n");
		return PTR_ERR(clks->lpcg_vif);
	}

	clks->lpcg_lis = devm_clk_get(dev, "lpcg_lis");
	if (IS_ERR(clks->lpcg_lis)) {
		dev_warn(dev, "failed to get lpcg lis clk\n");
		return PTR_ERR(clks->lpcg_lis);
	}

	clks->lpcg_apb = devm_clk_get(dev, "lpcg_apb");
	if (IS_ERR(clks->lpcg_apb)) {
		dev_warn(dev, "failed to get lpcg apb clk\n");
		return PTR_ERR(clks->lpcg_apb);
	}

	clks->lpcg_apb_csr = devm_clk_get(dev, "lpcg_apb_csr");
	if (IS_ERR(clks->lpcg_apb_csr)) {
		dev_warn(dev, "failed to get apb csr clk\n");
		return PTR_ERR(clks->lpcg_apb_csr);
	}

	clks->lpcg_apb_ctrl = devm_clk_get(dev, "lpcg_apb_ctrl");
	if (IS_ERR(clks->lpcg_apb_ctrl)) {
		dev_warn(dev, "failed to get lpcg apb ctrl clk\n");
		return PTR_ERR(clks->lpcg_apb_ctrl);
	}

	clks->clk_i2s_bypass = devm_clk_get(dev, "clk_i2s_bypass");
	if (IS_ERR(clks->clk_i2s_bypass)) {
		dev_err(dev, "failed to get i2s bypass clk\n");
		return PTR_ERR(clks->clk_i2s_bypass);
	}

	clks->lpcg_i2s = devm_clk_get(dev, "lpcg_i2s");
	if (IS_ERR(clks->lpcg_i2s)) {
		dev_err(dev, "failed to get lpcg i2s clk\n");
		return PTR_ERR(clks->lpcg_i2s);
	}
	return true;
}

static int imx8qm_pixel_clk_enable(struct imx_mhdp_device *hdp)
{
	struct imx_hdp_clks *clks = &hdp->clks;
	struct device *dev = hdp->mhdp.dev;
	int ret;

	ret = clk_prepare_enable(clks->av_pll);
	if (ret < 0) {
		dev_err(dev, "%s, pre av pll  error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk pxl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_pxl_mux);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk pxl mux error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_pxl_link);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk pxl link error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_vif);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk vif error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, pre lpcg pxl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_hdp);
	if (ret < 0) {
		dev_err(dev, "%s, pre lpcg hdp error\n", __func__);
		return ret;
	}
	return ret;

}

static void imx8qm_pixel_clk_disable(struct imx_mhdp_device *hdp)
{
	struct imx_hdp_clks *clks = &hdp->clks;

	clk_disable_unprepare(clks->lpcg_pxl);
	clk_disable_unprepare(clks->lpcg_hdp);
	clk_disable_unprepare(clks->lpcg_vif);
	clk_disable_unprepare(clks->clk_pxl);
	clk_disable_unprepare(clks->clk_pxl_link);
	clk_disable_unprepare(clks->clk_pxl_mux);
	clk_disable_unprepare(clks->av_pll);
}

static void imx8qm_pixel_clk_set_rate(struct imx_mhdp_device *hdp, u32 pclock)
{
	struct imx_hdp_clks *clks = &hdp->clks;

	/* pixel clock for HDMI */
	clk_set_rate(clks->av_pll, pclock);

	if (hdp->dual_mode == true) {
		clk_set_rate(clks->clk_pxl, pclock/2);
		clk_set_rate(clks->clk_pxl_link, pclock/2);
	} else {
		clk_set_rate(clks->clk_pxl_link, pclock);
		clk_set_rate(clks->clk_pxl, pclock);
	}
	clk_set_rate(clks->clk_pxl_mux, pclock);
}

static void imx8qm_pixel_clk_rate_change(struct imx_mhdp_device *hdp)
{
	/* set pixel clock before video mode setup */
	imx8qm_pixel_clk_disable(hdp);

	imx8qm_pixel_clk_set_rate(hdp, hdp->mhdp.mode.clock * 1000);

	imx8qm_pixel_clk_enable(hdp);

	/* Config pixel link mux */
	imx8qm_pixel_link_mux(hdp);
}

static int imx8qm_ipg_clk_enable(struct imx_mhdp_device *hdp)
{
	int ret;
	struct imx_hdp_clks *clks = &hdp->clks;
	struct device *dev = hdp->mhdp.dev;

	ret = clk_prepare_enable(clks->dig_pll);
	if (ret < 0) {
		dev_err(dev, "%s, pre dig pll error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_ipg);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk_ipg error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_core);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk core error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->lpcg_apb);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_lis);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk lis error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_msi);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk msierror\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_apb_csr);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb csr error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_apb_ctrl);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb ctrl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_i2s);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk i2s error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_i2s_bypass);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk i2s bypass error\n", __func__);
		return ret;
	}
	return ret;
}

static void imx8qm_ipg_clk_disable(struct imx_mhdp_device *hdp)
{
	struct imx_hdp_clks *clks = &hdp->clks;

	clk_disable_unprepare(clks->clk_i2s_bypass);
	clk_disable_unprepare(clks->lpcg_i2s);
	clk_disable_unprepare(clks->lpcg_apb_ctrl);
	clk_disable_unprepare(clks->lpcg_apb_csr);
	clk_disable_unprepare(clks->lpcg_msi);
	clk_disable_unprepare(clks->lpcg_lis);
	clk_disable_unprepare(clks->lpcg_apb);
	clk_disable_unprepare(clks->clk_core);
	clk_disable_unprepare(clks->clk_ipg);
	clk_disable_unprepare(clks->dig_pll);
}

static void imx8qm_ipg_clk_set_rate(struct imx_mhdp_device *hdp)
{
	struct imx_hdp_clks *clks = &hdp->clks;

	/* ipg/core clock */
	clk_set_rate(clks->dig_pll,  PLL_800MHZ);
	clk_set_rate(clks->clk_core, PLL_800MHZ/4);
	clk_set_rate(clks->clk_ipg,  PLL_800MHZ/8);
}

static void imx8qm_detach_pm_domains(struct imx_mhdp_device *hdp)
{
	if (hdp->pd_pll1_link && !IS_ERR(hdp->pd_pll1_link))
		device_link_del(hdp->pd_pll1_link);
	if (hdp->pd_pll1_dev && !IS_ERR(hdp->pd_pll1_dev))
		dev_pm_domain_detach(hdp->pd_pll1_dev, true);

	if (hdp->pd_pll0_link && !IS_ERR(hdp->pd_pll0_link))
		device_link_del(hdp->pd_pll0_link);
	if (hdp->pd_pll0_dev && !IS_ERR(hdp->pd_pll0_dev))
		dev_pm_domain_detach(hdp->pd_pll0_dev, true);

	if (hdp->pd_mhdp_link && !IS_ERR(hdp->pd_mhdp_link))
		device_link_del(hdp->pd_mhdp_link);
	if (hdp->pd_mhdp_dev && !IS_ERR(hdp->pd_mhdp_dev))
		dev_pm_domain_detach(hdp->pd_mhdp_dev, true);

	hdp->pd_mhdp_dev = NULL;
	hdp->pd_mhdp_link = NULL;
	hdp->pd_pll0_dev = NULL;
	hdp->pd_pll0_link = NULL;
	hdp->pd_pll1_dev = NULL;
	hdp->pd_pll1_link = NULL;
}

static int imx8qm_attach_pm_domains(struct imx_mhdp_device *hdp)
{
	struct device *dev = hdp->mhdp.dev;
	u32 flags = DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE;
	int ret = 0;

	hdp->pd_mhdp_dev = dev_pm_domain_attach_by_name(dev, "hdmi");
	if (IS_ERR(hdp->pd_mhdp_dev)) {
		ret = PTR_ERR(hdp->pd_mhdp_dev);
		dev_err(dev, "Failed to attach dc pd dev: %d\n", ret);
		goto fail;
	}
	hdp->pd_mhdp_link = device_link_add(dev, hdp->pd_mhdp_dev, flags);
	if (IS_ERR(hdp->pd_mhdp_link)) {
		ret = PTR_ERR(hdp->pd_mhdp_link);
		dev_err(dev, "Failed to add device link to dc pd dev: %d\n",
			ret);
		goto fail;
	}

	hdp->pd_pll0_dev = dev_pm_domain_attach_by_name(dev, "pll0");
	if (IS_ERR(hdp->pd_pll0_dev)) {
		ret = PTR_ERR(hdp->pd_pll0_dev);
		dev_err(dev, "Failed to attach pll0 pd dev: %d\n", ret);
		goto fail;
	}
	hdp->pd_pll0_link = device_link_add(dev, hdp->pd_pll0_dev, flags);
	if (IS_ERR(hdp->pd_pll0_link)) {
		ret = PTR_ERR(hdp->pd_pll0_link);
		dev_err(dev, "Failed to add device link to pll0 pd dev: %d\n",
			ret);
		goto fail;
	}

	hdp->pd_pll1_dev = dev_pm_domain_attach_by_name(dev, "pll1");
	if (IS_ERR(hdp->pd_pll1_dev)) {
		ret = PTR_ERR(hdp->pd_pll1_dev);
		dev_err(dev, "Failed to attach pll0 pd dev: %d\n", ret);
		goto fail;
	}
	hdp->pd_pll1_link = device_link_add(dev, hdp->pd_pll1_dev, flags);
	if (IS_ERR(hdp->pd_pll1_link)) {
		ret = PTR_ERR(hdp->pd_pll1_link);
		dev_err(dev, "Failed to add device link to pll1 pd dev: %d\n",
			ret);
		goto fail;
	}
fail:
	imx8qm_detach_pm_domains(hdp);
	return ret;
}

static int imx8qm_firmware_init(struct imx_mhdp_device *hdp)
{
	u32 rate;
	int ret;

	/* Power on PM Domains */
	imx8qm_attach_pm_domains(hdp);

	/* clock init and  rate set */
	imx8qm_clocks_init(hdp);

	imx8qm_ipg_clk_set_rate(hdp);

	/* Init pixel clock with 148.5MHz before FW init */
	imx8qm_pixel_clk_set_rate(hdp, 148500000);

	imx8qm_ipg_clk_enable(hdp);

	imx8qm_clk_mux(hdp->plat_data->is_dp);

	imx8qm_pixel_clk_enable(hdp);

	imx8qm_phy_reset(1);

	hdp->csr_pxl_mux_reg = 0;
	hdp->csr_ctrl0_reg = 0x8;
	hdp->csr_ctrl0_sec = 0xc;
	/* iMX8QM HDP register, Remap HPD memory address to low 4K */
	regmap_write(hdp->regmap_csr, hdp->csr_ctrl0_reg, 0);

	/* configure HDMI/DP core clock */
	rate = clk_get_rate(hdp->clks.clk_core);
	cdns_mhdp_set_fw_clk(&hdp->mhdp, rate);

	/* un-reset ucpu */
	writel(0,  (APB_CTRL << 2) + hdp->mhdp.regs);
	DRM_INFO("Started firmware!\n");

	ret = cdns_mhdp_check_alive(&hdp->mhdp);
	if (ret == false) {
		DRM_ERROR("NO HDMI FW running\n");
		return -ENXIO;
	}

	/* turn on IP activity */
	cdns_mhdp_set_firmware_active(&hdp->mhdp, 1);

	DRM_INFO("HDP FW Version - ver %d verlib %d\n",
			__raw_readb(VER_L + hdp->mhdp.regs) + (__raw_readb(VER_H + hdp->mhdp.regs) << 8),
	__raw_readb(VER_LIB_L_ADDR + hdp->mhdp.regs) + (__raw_readb(VER_LIB_H_ADDR + hdp->mhdp.regs) << 8));

	return 0;
}

static void cdns_hdmi_imx_encoder_disable(struct drm_encoder *encoder)
{
	struct imx_mhdp_device *hdp = encoder->bridge->driver_private;

	imx8qm_pixel_link_sync_disable(hdp->dual_mode);
	imx8qm_pixel_link_invalid(hdp->dual_mode);
}

static void cdns_hdmi_imx_encoder_enable(struct drm_encoder *encoder)
{
	struct imx_mhdp_device *hdp = encoder->bridge->driver_private;

	imx8qm_pixel_link_valid(hdp->dual_mode);
	imx8qm_pixel_link_sync_enable(hdp->dual_mode);
}

static int cdns_hdmi_imx_encoder_atomic_check(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);

	imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB101010_1X30;
	return 0;
}

static const struct drm_encoder_helper_funcs cdns_hdmi_imx_encoder_helper_funcs = {
	.enable     = cdns_hdmi_imx_encoder_enable,
	.disable    = cdns_hdmi_imx_encoder_disable,
	.atomic_check = cdns_hdmi_imx_encoder_atomic_check,
};

static const struct drm_encoder_funcs cdns_hdmi_imx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

#if 0
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
#endif

static struct cdn_plat_data imx8qm_hdmi_drv_data = {
	.bind	= cdns_hdmi_bind,
	.unbind	= cdns_hdmi_unbind,
	.phy_init = cdns_hdmi_phy_set_imx8qm,
	.fw_init = imx8qm_firmware_init,
	.pclock_change = imx8qm_pixel_clk_rate_change,
};

static struct cdn_plat_data imx8qm_dp_drv_data = {
	.bind	= cdns_dp_bind,
	.unbind	= cdns_dp_unbind,
	.phy_init = cdns_dp_phy_init_imx8qm,
	.fw_init = imx8qm_firmware_init,
	.pclock_change = imx8qm_pixel_clk_rate_change,
	.is_dp = true,
};

static const struct of_device_id cdns_hdmi_imx_dt_ids[] = {
#if 0
	{ .compatible = "cdn,imx8mq-hdmi",
	  .data = &imx8mq_hdmi_drv_data
	},
	{ .compatible = "cdn,imx8mq-dp",
	  .data = &imx8mq_dp_drv_data
	},
#endif
	{ .compatible = "cdn,imx8qm-hdmi",
	  .data = &imx8qm_hdmi_drv_data
	},
	{ .compatible = "cdn,imx8qm-dp",
	  .data = &imx8qm_dp_drv_data
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

static const struct component_ops cdns_hdmi_imx8qm_ops = {
	.bind	= cdns_hdmi_imx_bind,
	.unbind	= cdns_hdmi_imx_unbind,
};

static int cdns_hdmi_imx_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &cdns_hdmi_imx8qm_ops);
}

static int cdns_hdmi_imx_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cdns_hdmi_imx8qm_ops);

	return 0;
}

static struct platform_driver cdns_hdmi_imx_platform_driver = {
	.probe  = cdns_hdmi_imx_probe,
	.remove = cdns_hdmi_imx_remove,
	.driver = {
		.name = "cdn-hdp-imx8qm",
		.of_match_table = cdns_hdmi_imx_dt_ids,
	},
};

module_platform_driver(cdns_hdmi_imx_platform_driver);

MODULE_AUTHOR("Sandor YU <sandor.yu@nxp.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdnhdmi-imx");
