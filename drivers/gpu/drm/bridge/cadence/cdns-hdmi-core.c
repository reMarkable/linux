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
#include <drm/bridge/cdns-mhdp-common.h>
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
#include <linux/of_device.h>

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

static void hdmi_lanes_config(struct cdns_mhdp_device *mhdp)
{
	/* Line swaping */
	cdns_mhdp_reg_write(mhdp, LANES_CONFIG, 0x00400000 | mhdp->lane_mapping);
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
#if 0
	struct drm_display_info *di = &mhdp->connector.base.display_info;
	enum hdmi_extended_colorimetry ext_col;
	u32 sink_col, allowed_col;
#endif
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
	cdns_mhdp_infoframe_set(mhdp, 0, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_AVI);
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
	cdns_mhdp_infoframe_set(mhdp, 3, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_VENDOR);
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
	struct cdns_mhdp_device *mhdp =
				container_of(connector, struct cdns_mhdp_device, connector.base);

	u8 hpd = 0xf;

	hpd = cdns_mhdp_read_hpd(mhdp);

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
	struct cdns_mhdp_device *mhdp =
				container_of(connector, struct cdns_mhdp_device, connector.base);
	int num_modes = 0;
	struct edid *edid;

	edid = drm_do_get_edid(&mhdp->connector.base,
				   cdns_hdmi_get_edid_block, mhdp);
	if (edid) {
		dev_info(mhdp->dev, "%x,%x,%x,%x,%x,%x,%x,%x\n",
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
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_connector *connector = &mhdp->connector.base;

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
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_display_info *display_info = &mhdp->connector.base.display_info;
	struct video_info *video = &mhdp->video_info;

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

	mutex_lock(&mhdp->lock);

	DRM_INFO("Mode: %dx%dp%d\n", mode->hdisplay, mode->vdisplay, mode->clock); 

	memcpy(&mhdp->mode, mode, sizeof(struct drm_display_mode));

	hdmi_lanes_config(mhdp);

	cdns_mhdp_plat_call(mhdp, pclk_rate);

	cdns_mhdp_plat_call(mhdp, phy_set);

	cdns_hdmi_mode_set(mhdp);

	mutex_unlock(&mhdp->lock);
}

static const struct drm_bridge_funcs cdns_hdmi_bridge_funcs = {
	.attach = cdns_hdmi_bridge_attach,
	.mode_set = cdns_hdmi_bridge_mode_set,
	.mode_valid = cdns_hdmi_bridge_mode_valid,
};

static void hotplug_work_func(struct work_struct *work)
{
	struct cdns_mhdp_device *mhdp = container_of(work,
					   struct cdns_mhdp_device, hotplug_work.work);
	struct drm_connector *connector = &mhdp->connector.base;

	drm_helper_hpd_irq_event(connector->dev);

	if (connector->status == connector_status_connected) {
		/* Cable Connected */
		DRM_INFO("HDMI Cable Plug In\n");
		enable_irq(mhdp->irq[IRQ_OUT]);
	} else if (connector->status == connector_status_disconnected) {
		/* Cable Disconnedted  */
		DRM_INFO("HDMI Cable Plug Out\n");
		enable_irq(mhdp->irq[IRQ_IN]);
	}
}

static irqreturn_t cdns_hdmi_irq_thread(int irq, void *data)
{
	struct cdns_mhdp_device *mhdp = data;

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &mhdp->hotplug_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static void cdns_hdmi_parse_dt(struct cdns_mhdp_device *mhdp)
{
	struct device_node *of_node = mhdp->dev->of_node;
	int ret;

	ret = of_property_read_u32(of_node, "lane-mapping", &mhdp->lane_mapping);
	if (ret) {
		mhdp->lane_mapping = 0xc6;
		dev_warn(mhdp->dev, "Failed to get lane_mapping - using default 0xc6\n");
	}
	dev_info(mhdp->dev, "lane-mapping 0x%02x\n", mhdp->lane_mapping);
}

static int __cdns_hdmi_probe(struct platform_device *pdev,
		  struct cdns_mhdp_device *mhdp)
{
	struct device *dev = &pdev->dev;
	struct platform_device_info pdevinfo;
	struct resource *iores = NULL;
	int ret;

	mutex_init(&mhdp->lock);

	INIT_DELAYED_WORK(&mhdp->hotplug_work, hotplug_work_func);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mhdp->regs_base = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(mhdp->regs_base)) {
		dev_err(dev, "No regs_base memory\n");
		return -ENOMEM;
	}

	/* sec register base */
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mhdp->regs_sec = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(mhdp->regs_sec)) {
		dev_err(dev, "No regs_sec memory\n");
		return -ENOMEM;
	}

	mhdp->irq[IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (mhdp->irq[IRQ_IN] < 0) {
		dev_info(dev, "No plug_in irq number\n");
		return -EPROBE_DEFER;
	}

	mhdp->irq[IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (mhdp->irq[IRQ_OUT] < 0) {
		dev_info(dev, "No plug_out irq number\n");
		return -EPROBE_DEFER;
	}

	cdns_mhdp_plat_call(mhdp, power_on);

	/* Initialize FW */
	cdns_mhdp_plat_call(mhdp, firmware_init);

	/* HDMI FW alive check */
	ret = cdns_mhdp_check_alive(mhdp);
	if (ret == false) {
		dev_err(dev, "NO HDMI FW running\n");
		return -ENXIO;
	}

	/* Enable Hotplug Detect thread */
	irq_set_status_flags(mhdp->irq[IRQ_IN], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, mhdp->irq[IRQ_IN],
					NULL, cdns_hdmi_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					mhdp);
	if (ret < 0) {
		dev_err(dev, "can't claim irq %d\n",
						mhdp->irq[IRQ_IN]);
		return -EINVAL;
	}
	
	irq_set_status_flags(mhdp->irq[IRQ_OUT], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, mhdp->irq[IRQ_OUT],
					NULL, cdns_hdmi_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					mhdp);
	if (ret < 0) {
		dev_err(dev, "can't claim irq %d\n",
						mhdp->irq[IRQ_OUT]);
		return -EINVAL;
	}

	cdns_hdmi_parse_dt(mhdp);

	if (cdns_mhdp_read_hpd(mhdp))
		enable_irq(mhdp->irq[IRQ_OUT]);
	else
		enable_irq(mhdp->irq[IRQ_IN]);

	mhdp->bridge.base.driver_private = mhdp;
	mhdp->bridge.base.funcs = &cdns_hdmi_bridge_funcs;
#ifdef CONFIG_OF
	mhdp->bridge.base.of_node = dev->of_node;
#endif

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.parent = dev;
	pdevinfo.id = PLATFORM_DEVID_AUTO;

	dev_set_drvdata(dev, mhdp);

	/* register audio driver */
	cdns_mhdp_register_audio_driver(dev);

	/* register cec driver */
#ifdef CONFIG_DRM_CDNS_HDMI_CEC
	cdns_mhdp_register_cec_driver(dev);
#endif

	return 0;
}

static void __cdns_hdmi_remove(struct cdns_mhdp_device *mhdp)
{
	/* unregister cec driver */
#ifdef CONFIG_DRM_CDNS_HDMI_CEC
	cdns_mhdp_unregister_cec_driver(mhdp->dev);
#endif
	cdns_mhdp_unregister_audio_driver(mhdp->dev);
}

/* -----------------------------------------------------------------------------
 * Probe/remove API, used from platforms based on the DRM bridge API.
 */
int cdns_hdmi_probe(struct platform_device *pdev,
		struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret  = __cdns_hdmi_probe(pdev, mhdp);
	if (ret < 0)
		return ret;

	drm_bridge_add(&mhdp->bridge.base);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_hdmi_probe);

void cdns_hdmi_remove(struct platform_device *pdev)
{
	struct cdns_mhdp_device *mhdp = platform_get_drvdata(pdev);

	drm_bridge_remove(&mhdp->bridge.base);

	__cdns_hdmi_remove(mhdp);
}
EXPORT_SYMBOL_GPL(cdns_hdmi_remove);

/* -----------------------------------------------------------------------------
 * Bind/unbind API, used from platforms based on the component framework.
 */
int cdns_hdmi_bind(struct platform_device *pdev, struct drm_encoder *encoder,
			struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret = __cdns_hdmi_probe(pdev, mhdp);
	if (ret)
		return ret;

	ret = drm_bridge_attach(encoder, &mhdp->bridge.base, NULL);
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
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	__cdns_hdmi_remove(mhdp);
}
EXPORT_SYMBOL_GPL(cdns_hdmi_unbind);

MODULE_AUTHOR("Sandor Yu <sandor.yu@nxp.com>");
MODULE_DESCRIPTION("Cadence HDMI transmitter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdn-hdmi");
