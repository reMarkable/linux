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

#include <drm/bridge/cdns-mhdp-imx.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drmP.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hdmi.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/of_device.h>

static void hdmi_writel(struct cdns_mhdp_device *mhdp, u32 val, u32 offset)
{
	struct imx_mhdp_device *hdmi = container_of(mhdp, struct imx_mhdp_device, mhdp);

	/* TODO */
	if (offset >= 0x1000 && hdmi->regmap_csr) {
		/* Remap address to low 4K memory */
		regmap_write(hdmi->regmap_csr, hdmi->csr_ctrl0_reg, offset >> 12);
		writel(val, (offset & 0xfff) + mhdp->regs);
		/* Restore address mapping */
		regmap_write(hdmi->regmap_csr, hdmi->csr_ctrl0_reg, 0);

	} else
		writel(val, mhdp->regs + offset);
}

static int hdmi_sink_config(struct cdns_mhdp_device *mhdp)
{
	struct drm_scdc *scdc = &mhdp->connector.base.display_info.hdmi.scdc;
	u8 buff;
	int ret;

	if (mhdp->hdmi.char_rate > 340000) {
		/*
		 * TMDS Character Rate above 340MHz should working in HDMI2.0
		 * Enable scrambling and TMDS_Bit_Clock_Ratio
		 */
		buff = 3;
		mhdp->hdmi.hdmi_type = MODE_HDMI_2_0;
	} else  if (scdc->scrambling.low_rates) {
		/*
		 * Enable scrambling and HDMI2.0 when scrambling capability of sink
		 * be indicated in the HF-VSDB LTE_340Mcsc_scramble bit
		 */
		buff = 1;
		mhdp->hdmi.hdmi_type = MODE_HDMI_2_0;
	} else {
		/* Default work in HDMI1.4 */
		buff = 0;
		mhdp->hdmi.hdmi_type = MODE_HDMI_1_4;
	 }

	/* TMDS config */
	ret = cdns_hdmi_scdc_write(mhdp, 0x20, buff);
	return ret;
}

static int hdmi_lanes_config(struct cdns_mhdp_device *mhdp)
{
	int ret;

	/* TODO */
	/* Set the lane swapping */
//	if (cpu_is_imx8qm())
		ret = cdns_mhdp_reg_write(mhdp, LANES_CONFIG,
						    F_SOURCE_PHY_LANE0_SWAP(3) |
						    F_SOURCE_PHY_LANE1_SWAP(0) |
						    F_SOURCE_PHY_LANE2_SWAP(1) |
						    F_SOURCE_PHY_LANE3_SWAP(2) |
						    F_SOURCE_PHY_COMB_BYPASS(0) |
							F_SOURCE_PHY_20_10(1));
#if 0
	else
		ret = cdns_mhdp_reg_write(mhdp, LANES_CONFIG,
						    F_SOURCE_PHY_LANE0_SWAP(0) |
						    F_SOURCE_PHY_LANE1_SWAP(1) |
						    F_SOURCE_PHY_LANE2_SWAP(2) |
						    F_SOURCE_PHY_LANE3_SWAP(3) |
						    F_SOURCE_PHY_COMB_BYPASS(0) |
							F_SOURCE_PHY_20_10(1));
#endif
	return ret;
}

static void hdmi_info_frame_set(struct cdns_mhdp_device *mhdp,
					u8 entry_id, u8 packet_len, u8 *packet, u8 packet_type)
{
	u32 *packet32, len32;
	u32 val, i;

	/* invalidate entry */
	val = F_ACTIVE_IDLE_TYPE(1) | F_PKT_ALLOC_ADDRESS(entry_id);
	hdmi_writel(mhdp, val, SOURCE_PIF_PKT_ALLOC_REG);
	hdmi_writel(mhdp, F_PKT_ALLOC_WR_EN(1), SOURCE_PIF_PKT_ALLOC_WR_EN);

	/* flush fifo 1 */
	hdmi_writel(mhdp, F_FIFO1_FLUSH(1), SOURCE_PIF_FIFO1_FLUSH);

	/* write packet into memory */
	packet32 = (u32 *)packet;
	len32 = packet_len / 4;
	for (i = 0; i < len32; i++)
		hdmi_writel(mhdp, F_DATA_WR(packet32[i]), SOURCE_PIF_DATA_WR);

	/* write entry id */
	hdmi_writel(mhdp, F_WR_ADDR(entry_id), SOURCE_PIF_WR_ADDR);

	/* write request */
	hdmi_writel(mhdp, F_HOST_WR(1), SOURCE_PIF_WR_REQ);

	/* update entry */
	val =  F_ACTIVE_IDLE_TYPE(1) | F_TYPE_VALID(1) |
			F_PACKET_TYPE(packet_type) | F_PKT_ALLOC_ADDRESS(entry_id);
	hdmi_writel(mhdp, val, SOURCE_PIF_PKT_ALLOC_REG);

	hdmi_writel(mhdp, F_PKT_ALLOC_WR_EN(1), SOURCE_PIF_PKT_ALLOC_WR_EN);
}

#define RGB_ALLOWED_COLORIMETRY (BIT(HDMI_EXTENDED_COLORIMETRY_BT2020) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_OPRGB))
#define YCC_ALLOWED_COLORIMETRY (BIT(HDMI_EXTENDED_COLORIMETRY_BT2020) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_BT2020_CONST_LUM) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_OPYCC_601) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_S_YCC_601) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_709) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_601))
static int hdmi_avi_info_set(struct cdns_mhdp_device *mhdp,
				struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
//	struct drm_display_info *di = &mhdp->connector.base.display_info;
//	enum hdmi_extended_colorimetry ext_col;
//	u32 sink_col, allowed_col;
	int format = mhdp->video_info.color_fmt;
	u8 buf[32];
	int ret;

	/* Initialise info frame from DRM mode */
	drm_hdmi_avi_infoframe_from_display_mode(&frame, &mhdp->connector.base, mode);

#if 0 //TODO to DCSS
	/* Set up colorimetry */
	allowed_col = format == PXL_RGB ? RGB_ALLOWED_COLORIMETRY :
						  YCC_ALLOWED_COLORIMETRY;

	sink_col = di->hdmi.colorimetry & allowed_col;

	if (sink_col & BIT(HDMI_EXTENDED_COLORIMETRY_BT2020))
		ext_col = HDMI_EXTENDED_COLORIMETRY_BT2020;
	else if (sink_col & BIT(HDMI_EXTENDED_COLORIMETRY_BT2020_CONST_LUM))
		ext_col = HDMI_EXTENDED_COLORIMETRY_BT2020_CONST_LUM;
	else if (sink_col & BIT(HDMI_EXTENDED_COLORIMETRY_OPRGB))
		ext_col = HDMI_EXTENDED_COLORIMETRY_OPRGB;
	else if (sink_col & BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_709))
		ext_col = HDMI_EXTENDED_COLORIMETRY_XV_YCC_709;
	else if (sink_col & BIT(HDMI_EXTENDED_COLORIMETRY_OPYCC_601))
		ext_col = HDMI_EXTENDED_COLORIMETRY_OPYCC_601;
	else if (sink_col & BIT(HDMI_EXTENDED_COLORIMETRY_S_YCC_601))
		ext_col = HDMI_EXTENDED_COLORIMETRY_S_YCC_601;
	else if (sink_col & BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_601))
		ext_col = HDMI_EXTENDED_COLORIMETRY_XV_YCC_601;
	else
		ext_col = 0;

	frame.colorimetry = sink_col ? HDMI_COLORIMETRY_EXTENDED :
					  HDMI_COLORIMETRY_NONE;
	frame.extended_colorimetry = ext_col;
#endif

	switch (format) {
	case YCBCR_4_4_4:
		frame.colorspace = HDMI_COLORSPACE_YUV444;
		break;
	case YCBCR_4_2_2:
		frame.colorspace = HDMI_COLORSPACE_YUV422;
		break;
	case YCBCR_4_2_0:
		frame.colorspace = HDMI_COLORSPACE_YUV420;
		break;
	default:
		frame.colorspace = HDMI_COLORSPACE_RGB;
		break;
	}

	ret = hdmi_avi_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_ERROR("failed to pack AVI infoframe: %d\n", ret);
		return -1;
	}

	buf[0] = 0;
	hdmi_info_frame_set(mhdp, 0, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_AVI);
	return 0;
}

static int hdmi_vendor_info_set(struct cdns_mhdp_device *mhdp,
				struct drm_display_mode *mode)
{
	struct hdmi_vendor_infoframe frame;
	u8 buf[32];
	int ret;

	/* Initialise vendor frame from DRM mode */
	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame, &mhdp->connector.base, mode);
	if (ret < 0) {
		DRM_WARN("Unable to init vendor infoframe: %d\n", ret);
		return -1;
	}

	ret = hdmi_vendor_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_WARN("Unable to pack vendor infoframe: %d\n", ret);
		return -1;
	}

	buf[0] = 0;
	hdmi_info_frame_set(mhdp, 3, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_VENDOR);
	return 0;
}

void cdns_hdmi_mode_set(struct cdns_mhdp_device *mhdp)
{
	struct drm_display_mode *mode = &mhdp->mode;
	int ret;

	ret = hdmi_sink_config(mhdp);
	if (ret < 0) {
		DRM_ERROR("%s failed\n", __func__);
		return;
	}

	ret = cdns_hdmi_ctrl_init(mhdp, mhdp->hdmi.hdmi_type, mhdp->hdmi.char_rate);
	if (ret < 0) {
		DRM_ERROR("%s, ret = %d\n", __func__, ret);
		return;
	}

	/* Config GCP */
	if (mhdp->video_info.color_depth == 8)
		cdns_hdmi_disable_gcp(mhdp);
	else
		cdns_hdmi_enable_gcp(mhdp);

	ret = hdmi_avi_info_set(mhdp, mode);
	if (ret < 0) {
		DRM_ERROR("%s ret = %d\n", __func__, ret);
		return;
	}

	/* vendor info frame is enable only  when HDMI1.4 4K mode */
	ret = hdmi_vendor_info_set(mhdp, mode);
	if (ret < 0)
		DRM_WARN("Unable to configure Vendor infoframe\n");

	ret = cdns_hdmi_mode_config(mhdp, mode, &mhdp->video_info);
	if (ret < 0) {
		DRM_ERROR("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);
		return;
	}

	/* wait HDMI PHY pixel clock stable */
	msleep(50);
}

static enum drm_connector_status
cdns_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct imx_mhdp_device *hdmi =
				container_of(connector, struct imx_mhdp_device, mhdp.connector.base);

	u8 hpd = 0xf;

	hpd = cdns_mhdp_read_hpd(&hdmi->mhdp);

	if (hpd == 1)
		/* Cable Connected */
		return connector_status_connected;
	else if (hpd == 0)
		/* Cable Disconnedted */
		return connector_status_disconnected;
	else {
		/* Cable status unknown */
		DRM_INFO("Unknow cable status, hdp=%u\n", hpd);
		return connector_status_unknown;
	}
}

static int cdns_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct imx_mhdp_device *hdmi = container_of(connector, struct imx_mhdp_device,
					     mhdp.connector.base);
	int num_modes = 0;
	struct edid *edid;

	edid = drm_do_get_edid(&hdmi->mhdp.connector.base,
				   cdns_hdmi_get_edid_block, &hdmi->mhdp);
	if (edid) {
		dev_info(hdmi->mhdp.dev, "%x,%x,%x,%x,%x,%x,%x,%x\n",
			 edid->header[0], edid->header[1],
			 edid->header[2], edid->header[3],
			 edid->header[4], edid->header[5],
			 edid->header[6], edid->header[7]);
		drm_connector_update_edid_property(connector, edid);
		num_modes = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	if (num_modes == 0)
		DRM_ERROR("Invalid edid\n");
	return num_modes;
}

static const struct drm_connector_funcs cdns_hdmi_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = cdns_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs cdns_hdmi_connector_helper_funcs = {
	.get_modes = cdns_hdmi_connector_get_modes,
};

static int cdns_hdmi_bridge_attach(struct drm_bridge *bridge)
{
	struct imx_mhdp_device *hdmi = bridge->driver_private;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_connector *connector = &hdmi->mhdp.connector.base;

	connector->interlace_allowed = 1;
	connector->polled = DRM_CONNECTOR_POLL_HPD;

	drm_connector_helper_add(connector, &cdns_hdmi_connector_helper_funcs);

	drm_connector_init(bridge->dev, connector, &cdns_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);

	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static enum drm_mode_status
cdns_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_mode *mode)
{
	enum drm_mode_status mode_status = MODE_OK;

	/* We don't support double-clocked and Interlaced modes */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK ||
			mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_BAD;

	/* MAX support pixel clock rate 148.5MHz */
	if (mode->clock > 148500)
		return MODE_CLOCK_HIGH;

	/* 4096x2160 is not supported */
	if (mode->hdisplay > 3840 || mode->vdisplay > 2160)
		return MODE_BAD_HVALUE;

	return mode_status;
}

static void cdns_hdmi_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *orig_mode,
				    const struct drm_display_mode *mode)
{
	struct imx_mhdp_device *hdmi = bridge->driver_private;
	struct drm_display_info *display_info = &hdmi->mhdp.connector.base.display_info;
	struct video_info *video = &hdmi->mhdp.video_info;

	switch (display_info->bpc) {
	case 10:
		video->color_depth = 10;
		break;
	case 6:
		video->color_depth = 6;
		break;
	default:
		video->color_depth = 8;
		break;
	}

	video->color_fmt = PXL_RGB;
	video->v_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);
	video->h_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);

	mutex_lock(&hdmi->lock);

	DRM_INFO("Mode: %dx%dp%d\n", mode->hdisplay, mode->vdisplay, mode->clock); 

	memcpy(&hdmi->mhdp.mode, mode, sizeof(struct drm_display_mode));

	hdmi->dual_mode = video_is_dual_mode(mode);

	hdmi_lanes_config(&hdmi->mhdp);

	hdp_plat_call(hdmi, pclock_change);

	hdp_plat_call(hdmi, phy_init);

	cdns_hdmi_mode_set(&hdmi->mhdp);

	mutex_unlock(&hdmi->lock);
}

static const struct drm_bridge_funcs cdns_hdmi_bridge_funcs = {
	.attach = cdns_hdmi_bridge_attach,
	.mode_set = cdns_hdmi_bridge_mode_set,
	.mode_valid = cdns_hdmi_bridge_mode_valid,
};

static void hotplug_work_func(struct work_struct *work)
{
	struct imx_mhdp_device *hdmi = container_of(work,
					   struct imx_mhdp_device, hotplug_work.work);
	struct drm_connector *connector = &hdmi->mhdp.connector.base;

	drm_helper_hpd_irq_event(connector->dev);

	if (connector->status == connector_status_connected) {
		/* Cable Connected */
		DRM_INFO("HDMI Cable Plug In\n");
		enable_irq(hdmi->irq[IRQ_OUT]);
	} else if (connector->status == connector_status_disconnected) {
		/* Cable Disconnedted  */
		DRM_INFO("HDMI Cable Plug Out\n");
		enable_irq(hdmi->irq[IRQ_IN]);
	}
}

static irqreturn_t cdns_hdmi_irq_thread(int irq, void *data)
{
	struct imx_mhdp_device *hdmi = data;

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &hdmi->hotplug_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static struct imx_mhdp_device *
__cdns_hdmi_probe(struct platform_device *pdev,
			const struct cdn_plat_data *plat_data)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct platform_device_info pdevinfo;
	struct imx_mhdp_device *hdmi;
	struct resource *iores = NULL;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return ERR_PTR(-ENOMEM);

	hdmi->plat_data = plat_data;
	hdmi->mhdp.dev = dev;

	mutex_init(&hdmi->lock);
	mutex_init(&hdmi->audio_mutex);
	spin_lock_init(&hdmi->audio_lock);

	INIT_DELAYED_WORK(&hdmi->hotplug_work, hotplug_work_func);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi->mhdp.regs = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(hdmi->mhdp.regs)) {
		ret = PTR_ERR(hdmi->mhdp.regs);
		goto err_out;
	}

	/* csr register base */
	hdmi->regmap_csr = syscon_regmap_lookup_by_phandle(np, "csr");
	if (IS_ERR(hdmi->regmap_csr)) {
		dev_info(dev, "No csr regmap\n");
	}

	hdmi->irq[IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (hdmi->irq[IRQ_IN] < 0) {
		dev_info(&pdev->dev, "No plug_in irq number\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	hdmi->irq[IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (hdmi->irq[IRQ_OUT] < 0) {
		dev_info(&pdev->dev, "No plug_out irq number\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	/* Initialize dual_mode to false */
	hdmi->dual_mode = false;

	/* Initialize FW */
	hdp_plat_call(hdmi, fw_init);

	/* HDMI FW alive check */
	ret = cdns_mhdp_check_alive(&hdmi->mhdp);
	if (ret == false) {
		DRM_ERROR("NO HDMI FW running\n");
		return ERR_PTR(-ENXIO);
	}

	/* Enable Hotplug Detect thread */
	irq_set_status_flags(hdmi->irq[IRQ_IN], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, hdmi->irq[IRQ_IN],
					NULL, cdns_hdmi_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					hdmi);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n",
						hdmi->irq[IRQ_IN]);
		goto err_out;
	}
	
	irq_set_status_flags(hdmi->irq[IRQ_OUT], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, hdmi->irq[IRQ_OUT],
					NULL, cdns_hdmi_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					hdmi);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n",
						hdmi->irq[IRQ_OUT]);
		goto err_out;
	}

	if (cdns_mhdp_read_hpd(&hdmi->mhdp))
		enable_irq(hdmi->irq[IRQ_OUT]);
	else
		enable_irq(hdmi->irq[IRQ_IN]);

	hdmi->mhdp.bridge.base.driver_private = hdmi;
	hdmi->mhdp.bridge.base.funcs = &cdns_hdmi_bridge_funcs;
#ifdef CONFIG_OF
	hdmi->mhdp.bridge.base.of_node = pdev->dev.of_node;
#endif

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.parent = dev;
	pdevinfo.id = PLATFORM_DEVID_AUTO;

	platform_set_drvdata(pdev, hdmi);

	return hdmi;

err_out:

	return ERR_PTR(ret);
}

static void __cdns_hdmi_remove(struct imx_mhdp_device *hdmi)
{
}

/* -----------------------------------------------------------------------------
 * Probe/remove API, used from platforms based on the DRM bridge API.
 */
int cdns_hdmi_probe(struct platform_device *pdev,
		  const struct cdn_plat_data *plat_data)
{
	struct imx_mhdp_device *hdmi;

	hdmi = __cdns_hdmi_probe(pdev, plat_data);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);

	drm_bridge_add(&hdmi->mhdp.bridge.base);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_hdmi_probe);

void cdns_hdmi_remove(struct platform_device *pdev)
{
	struct imx_mhdp_device *hdmi = platform_get_drvdata(pdev);

	drm_bridge_remove(&hdmi->mhdp.bridge.base);

	__cdns_hdmi_remove(hdmi);
}
EXPORT_SYMBOL_GPL(cdns_hdmi_remove);

/* -----------------------------------------------------------------------------
 * Bind/unbind API, used from platforms based on the component framework.
 */
int cdns_hdmi_bind(struct platform_device *pdev, struct drm_encoder *encoder,
		 const struct cdn_plat_data *plat_data)
{
	struct imx_mhdp_device *hdmi;
	int ret;

	hdmi = __cdns_hdmi_probe(pdev, plat_data);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);

	ret = drm_bridge_attach(encoder, &hdmi->mhdp.bridge.base, NULL);
	if (ret) {
		cdns_hdmi_remove(pdev);
		DRM_ERROR("Failed to initialize bridge with drm\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_hdmi_bind);

void cdns_hdmi_unbind(struct device *dev)
{
	struct imx_mhdp_device *hdmi = dev_get_drvdata(dev);

	__cdns_hdmi_remove(hdmi);
}
EXPORT_SYMBOL_GPL(cdns_hdmi_unbind);

MODULE_AUTHOR("Sandor Yu <sandor.yu@nxp.com>");
MODULE_DESCRIPTION("Cadence HDMI transmitter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdn-hdmi");
