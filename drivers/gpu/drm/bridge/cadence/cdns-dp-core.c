/*
 * Cadence Display Port Interface (DP) driver
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
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>

#define aux_to_hdp(x) container_of(x, struct imx_mhdp_device, aux)

/*
 * This function only implements native DPDC reads and writes
 */
static ssize_t dp_aux_transfer(struct drm_dp_aux *aux,
		struct drm_dp_aux_msg *msg)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(aux->dev);
	bool native = msg->request & (DP_AUX_NATIVE_WRITE & DP_AUX_NATIVE_READ);
	int ret;

	/* Ignore address only message */
	if ((msg->size == 0) || (msg->buffer == NULL)) {
		msg->reply = native ?
			DP_AUX_NATIVE_REPLY_ACK : DP_AUX_I2C_REPLY_ACK;
		return msg->size;
	}

	if (!native) {
		dev_err(mhdp->dev, "%s: only native messages supported\n", __func__);
		return -EINVAL;
	}

	/* msg sanity check */
	if (msg->size > DP_AUX_MAX_PAYLOAD_BYTES) {
		dev_err(mhdp->dev, "%s: invalid msg: size(%zu), request(%x)\n",
						__func__, msg->size, (unsigned int)msg->request);
		return -EINVAL;
	}

	if (msg->request == DP_AUX_NATIVE_WRITE) {
		const u8 *buf = msg->buffer;
		int i;
		for (i = 0; i < msg->size; ++i) {
			ret = cdns_mhdp_dpcd_write(mhdp,
						   msg->address + i, buf[i]);
			if (!ret)
				continue;

			DRM_DEV_ERROR(mhdp->dev, "Failed to write DPCD\n");

			return ret;
		}
	}

	if (msg->request == DP_AUX_NATIVE_READ) {
		ret = cdns_mhdp_dpcd_read(mhdp, msg->address, msg->buffer, msg->size);
		if (ret < 0)
			return -EIO;
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		return msg->size;
	}
	return 0;
}

static int dp_aux_init(struct cdns_mhdp_device *mhdp,
		  struct device *dev)
{
	int ret;

	mhdp->dp.aux.name = "imx_dp_aux";
	mhdp->dp.aux.dev = dev;
	mhdp->dp.aux.transfer = dp_aux_transfer;

	ret = drm_dp_aux_register(&mhdp->dp.aux);

	return ret;
}

static int dp_aux_destroy(struct cdns_mhdp_device *mhdp)
{
	drm_dp_aux_unregister(&mhdp->dp.aux);
	return 0;
}

static void dp_pixel_clk_reset(struct cdns_mhdp_device *mhdp)
{
	u32 val;

	/* reset pixel clk */
	val = cdns_mhdp_reg_read(mhdp, SOURCE_HDTX_CAR);
	cdns_mhdp_reg_write(mhdp, SOURCE_HDTX_CAR, val & 0xFD);
	cdns_mhdp_reg_write(mhdp, SOURCE_HDTX_CAR, val);
}

static void cdns_dp_mode_set(struct imx_mhdp_device *dp,
			const struct drm_display_mode *mode)
{
	struct drm_dp_link link;
	struct cdns_mhdp_device *mhdp = &dp->mhdp;
	u32 lane_mapping = mhdp->dp.lane_mapping;
	int ret;
	char linkid[6];

	memcpy(&mhdp->mode, mode, sizeof(struct drm_display_mode));

	dp->dual_mode = video_is_dual_mode(mode);

	dp_pixel_clk_reset(mhdp);

	hdp_plat_call(dp, pclock_change);

	hdp_plat_call(dp, phy_init);

	ret = drm_dp_downstream_id(&mhdp->dp.aux, linkid);
	if (ret < 0) {
		DRM_INFO("Failed to Get DP link ID: %d\n", ret);
		return;
	}
	DRM_INFO("DP link id: %s, 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		 linkid, linkid[0], linkid[1], linkid[2], linkid[3], linkid[4],
		 linkid[5]);

	/* Check dp link */
	ret = drm_dp_link_probe(&mhdp->dp.aux, &link);
	if (ret < 0) {
		DRM_INFO("Failed to probe DP link: %d\n", ret);
		return;
	}
	DRM_INFO("DP revision: 0x%x\n", link.revision);
	DRM_INFO("DP rate: %d Mbps\n", link.rate);
	DRM_INFO("DP number of lanes: %d\n", link.num_lanes);
	DRM_INFO("DP capabilities: 0x%lx\n", link.capabilities);

	drm_dp_link_power_up(&mhdp->dp.aux, &mhdp->dp.link);
	if (ret < 0) {
		DRM_INFO("Failed to power DP link: %d\n", ret);
		return;
	}

	/* always use the number of lanes from the display*/
	mhdp->dp.link.num_lanes = link.num_lanes;

	/* Use the lower link rate */
	if (mhdp->dp.link_rate != 0) {
		mhdp->dp.link.rate = min(mhdp->dp.link_rate, (u32)link.rate);
		DRM_DEBUG("DP actual link rate:  0x%x\n", link.rate);
	}

	/* initialize phy if lanes or link rate differnt */
	if (mhdp->dp.link.num_lanes != mhdp->dp.num_lanes ||
			mhdp->dp.link.rate != mhdp->dp.link_rate)
		hdp_plat_call(dp, phy_init);

	/* Video off */
	ret = cdns_mhdp_set_video_status(mhdp, CONTROL_VIDEO_IDLE);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to valid video %d\n", ret);
		return;
	}

	/* Line swaping */
	cdns_mhdp_reg_write(mhdp, LANES_CONFIG, 0x00400000 | lane_mapping);

	/* Set DP host capability */
	ret = cdns_mhdp_set_host_cap(mhdp, mhdp->dp.link.num_lanes, false);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to set host cap %d\n", ret);
		return;
	}

	ret = cdns_mhdp_config_video(mhdp);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to config video %d\n", ret);
		return;
	}

	/* Link trainning */
	ret = cdns_mhdp_train_link(mhdp);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed link train %d\n", ret);
		return;
	}

	ret = cdns_mhdp_set_video_status(mhdp, CONTROL_VIDEO_VALID);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to valid video %d\n", ret);
		return;
	}

	return;
}

/* -----------------------------------------------------------------------------
 * dp TX Setup
 */
static enum drm_connector_status
cdns_dp_connector_detect(struct drm_connector *connector, bool force)
{
	struct imx_mhdp_device *dp = container_of(connector,
					struct imx_mhdp_device, mhdp.connector.base);
	u8 hpd = 0xf;

	hpd = cdns_mhdp_read_hpd(&dp->mhdp);
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

static int cdns_dp_connector_get_modes(struct drm_connector *connector)
{
	struct imx_mhdp_device *dp = container_of(connector,
						struct imx_mhdp_device, mhdp.connector.base);
	int num_modes = 0;
	struct edid *edid;

	edid = drm_do_get_edid(&dp->mhdp.connector.base,
				   cdns_mhdp_get_edid_block, &dp->mhdp);
	if (edid) {
		dev_info(dp->mhdp.dev, "%x,%x,%x,%x,%x,%x,%x,%x\n",
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

static const struct drm_connector_funcs cdns_dp_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = cdns_dp_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs cdns_dp_connector_helper_funcs = {
	.get_modes = cdns_dp_connector_get_modes,
};

static int cdns_dp_bridge_attach(struct drm_bridge *bridge)
{
	struct imx_mhdp_device *dp = bridge->driver_private;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_connector *connector = &dp->mhdp.connector.base;

	connector->interlace_allowed = 1;
	connector->polled = DRM_CONNECTOR_POLL_HPD;

	drm_connector_helper_add(connector, &cdns_dp_connector_helper_funcs);

	drm_connector_init(bridge->dev, connector, &cdns_dp_connector_funcs,
			   DRM_MODE_CONNECTOR_DisplayPort);

	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static enum drm_mode_status
cdns_dp_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_mode *mode)
{
	enum drm_mode_status mode_status = MODE_OK;

	/* We don't support double-clocked modes */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK ||
			mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_BAD;

	/* MAX support pixel clock rate 594MHz */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	/* 4096x2160 is not supported now */
	if (mode->hdisplay > 3840)
		return MODE_BAD_HVALUE;

	if (mode->vdisplay > 2160)
		return MODE_BAD_VVALUE;

	return mode_status;
}

static void cdns_dp_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *orig_mode,
				    const struct drm_display_mode *mode)
{
	struct imx_mhdp_device *dp = bridge->driver_private;
	struct drm_display_info *display_info = &dp->mhdp.connector.base.display_info;
	struct video_info *video = &dp->mhdp.video_info;

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

	DRM_INFO("Mode: %dx%dp%d\n", mode->hdisplay, mode->vdisplay, mode->clock); 

	mutex_lock(&dp->lock);

	cdns_dp_mode_set(dp, mode);

	mutex_unlock(&dp->lock);
}

static void cdn_hdp_bridge_enable(struct drm_bridge *bridge)
{
}

static void cdn_hdp_bridge_disable(struct drm_bridge *bridge)
{	
	struct imx_mhdp_device *dp = bridge->driver_private;
	struct cdns_mhdp_device *mhdp = &dp->mhdp;

	cdns_mhdp_set_video_status(mhdp, CONTROL_VIDEO_IDLE);
	drm_dp_link_power_down(&mhdp->dp.aux, &mhdp->dp.link);
}

static const struct drm_bridge_funcs cdns_dp_bridge_funcs = {
	.attach = cdns_dp_bridge_attach,
	.enable = cdn_hdp_bridge_enable,
	.disable = cdn_hdp_bridge_disable,
	.mode_set = cdns_dp_bridge_mode_set,
	.mode_valid = cdns_dp_bridge_mode_valid,
};

static void hotplug_work_func(struct work_struct *work)
{
	struct imx_mhdp_device *dp = container_of(work,
					   struct imx_mhdp_device, hotplug_work.work);
	struct drm_connector *connector = &dp->mhdp.connector.base;

	drm_helper_hpd_irq_event(connector->dev);

	if (connector->status == connector_status_connected) {
		DRM_INFO("HDMI/DP Cable Plug In\n");
		enable_irq(dp->irq[IRQ_OUT]);
	} else if (connector->status == connector_status_disconnected) {
		/* Cable Disconnedted  */
		DRM_INFO("HDMI/DP Cable Plug Out\n");
		enable_irq(dp->irq[IRQ_IN]);
	}
}

static irqreturn_t cdns_dp_irq_thread(int irq, void *data)
{
	struct imx_mhdp_device *dp = data;

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &dp->hotplug_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static void cdns_dp_parse_dt(struct cdns_mhdp_device *mhdp)
{
	struct device_node *of_node = mhdp->dev->of_node;
	int ret;

	ret = of_property_read_u32(of_node, "lane-mapping",
						&mhdp->dp.lane_mapping);
	if (ret) {
		mhdp->dp.lane_mapping = 0xc6;
		dev_warn(mhdp->dev, "Failed to get lane_mapping - using default 0xc6\n");
	}
	dev_info(mhdp->dev, "lane-mapping 0x%02x\n", mhdp->dp.lane_mapping);

	ret = of_property_read_u32(of_node, "link-rate", &mhdp->dp.link_rate);
	if (ret) {
		mhdp->dp.link_rate = 162000 ;
		dev_warn(mhdp->dev, "Failed to get link-rate, use default 1620MHz\n");
	}
	dev_info(mhdp->dev, "link-rate %d\n", mhdp->dp.link_rate);
	
	ret = of_property_read_u32(of_node, "num-lanes", &mhdp->dp.num_lanes);
	if (ret) {
		mhdp->dp.num_lanes = 4;
		dev_warn(mhdp->dev, "Failed to get num_lanes - using default\n");
	}
	dev_info(mhdp->dev, "dp_num_lanes 0x%02x\n", mhdp->dp.num_lanes);

	mhdp->dp.link.num_lanes = mhdp->dp.num_lanes;
	mhdp->dp.link.rate= mhdp->dp.link_rate;
}

static struct imx_mhdp_device *
__cdns_dp_probe(struct platform_device *pdev,
		const struct cdn_plat_data *plat_data)
{
	struct device *dev = &pdev->dev;
	struct imx_mhdp_device *dp;
	struct resource *iores = NULL;
	int ret;

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return ERR_PTR(-ENOMEM);

	dp->plat_data = plat_data;
	dp->mhdp.dev = dev;

	mutex_init(&dp->lock);
	mutex_init(&dp->audio_mutex);
	spin_lock_init(&dp->audio_lock);

	INIT_DELAYED_WORK(&dp->hotplug_work, hotplug_work_func);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dp->mhdp.regs = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(dp->mhdp.regs)) {
		ret = PTR_ERR(dp->mhdp.regs);
		goto err_out;
	}

#if 0
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	dp->regs_ss = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(dp->regs_ss)) {
		ret = PTR_ERR(dp->regs_ss);
		goto err_out;
	}
#endif

	dp->irq[IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (dp->irq[IRQ_IN] < 0)
		dev_info(&pdev->dev, "No plug_in irq number\n");

	dp->irq[IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (dp->irq[IRQ_OUT] < 0)
		dev_info(&pdev->dev, "No plug_out irq number\n");

	cdns_dp_parse_dt(&dp->mhdp);

	dp->dual_mode = false;
	hdp_plat_call(dp, fw_init);

	/* DP FW alive check */
	ret = cdns_mhdp_check_alive(&dp->mhdp);
	if (ret == false) {
		DRM_ERROR("NO dp FW running\n");
		return ERR_PTR(-ENXIO);
	}

	/* DP PHY init before AUX init */
	hdp_plat_call(dp, phy_init);

	/* Enable Hotplug Detect IRQ thread */
	irq_set_status_flags(dp->irq[IRQ_IN], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, dp->irq[IRQ_IN],
					NULL, cdns_dp_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					dp);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n",
						dp->irq[IRQ_IN]);
		goto err_out;
	}
	
	irq_set_status_flags(dp->irq[IRQ_OUT], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, dp->irq[IRQ_OUT],
					NULL, cdns_dp_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					dp);
	if (ret) {
		dev_err(&pdev->dev, "can't claim irq %d\n",
						dp->irq[IRQ_OUT]);
		goto err_out;
	}
	if (cdns_mhdp_read_hpd(&dp->mhdp))
		enable_irq(dp->irq[IRQ_OUT]);
	else
		enable_irq(dp->irq[IRQ_IN]);

	dp->mhdp.bridge.base.driver_private = dp;
	dp->mhdp.bridge.base.funcs = &cdns_dp_bridge_funcs;
#ifdef CONFIG_OF
	dp->mhdp.bridge.base.of_node = pdev->dev.of_node;
#endif

	platform_set_drvdata(pdev, dp);
	
	dp_aux_init(&dp->mhdp, dev);

	return dp;

err_out:
	return ERR_PTR(ret);
}

static void __cdns_dp_remove(struct imx_mhdp_device *dp)
{
	dp_aux_destroy(&dp->mhdp);
}

/* -----------------------------------------------------------------------------
 * Probe/remove API, used from platforms based on the DRM bridge API.
 */
int cdns_dp_probe(struct platform_device *pdev,
		  const struct cdn_plat_data *plat_data)
{
	struct imx_mhdp_device *dp;

	dp = __cdns_dp_probe(pdev, plat_data);
	if (IS_ERR(dp))
		return PTR_ERR(dp);

	drm_bridge_add(&dp->mhdp.bridge.base);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_dp_probe);

void cdns_dp_remove(struct platform_device *pdev)
{
	struct imx_mhdp_device *dp = platform_get_drvdata(pdev);

	drm_bridge_remove(&dp->mhdp.bridge.base);

	__cdns_dp_remove(dp);
}
EXPORT_SYMBOL_GPL(cdns_dp_remove);

/* -----------------------------------------------------------------------------
 * Bind/unbind API, used from platforms based on the component framework.
 */
int cdns_dp_bind(struct platform_device *pdev, struct drm_encoder *encoder,
		 const struct cdn_plat_data *plat_data)
{
	struct imx_mhdp_device *dp;
	int ret;

	dp = __cdns_dp_probe(pdev, plat_data);
	if (IS_ERR(dp))
		return PTR_ERR(dp);

	ret = drm_bridge_attach(encoder, &dp->mhdp.bridge.base, NULL);
	if (ret) {
		cdns_dp_remove(pdev);
		DRM_ERROR("Failed to initialize bridge with drm\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_dp_bind);

void cdns_dp_unbind(struct device *dev)
{
	struct imx_mhdp_device *dp = dev_get_drvdata(dev);

	__cdns_dp_remove(dp);
}
EXPORT_SYMBOL_GPL(cdns_dp_unbind);

MODULE_AUTHOR("Sandor Yu <sandor.yu@nxp.com>");
MODULE_DESCRIPTION("Cadence Display Port transmitter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdn-dp");
