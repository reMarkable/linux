/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * i.MX23/i.MX28/i.MX6SX MXSFB LCD controller driver.
 */

#ifndef __MXSFB_DRV_H__
#define __MXSFB_DRV_H__

struct mxsfb_devdata {
	unsigned int	 transfer_count;
	unsigned int	 cur_buf;
	unsigned int	 next_buf;
	unsigned int	 debug0;
	unsigned int	 hs_wdth_mask;
	unsigned int	 hs_wdth_shift;
	unsigned int	 ipversion;
	unsigned int	 num_formats;
};

struct mxsfb_drm_private {
	const struct mxsfb_devdata	*devdata;

	void __iomem			*base;	/* registers */
	struct clk			*clk;
	struct clk			*clk_axi;
	struct clk			*clk_disp_axi;

	struct drm_simple_display_pipe	pipe;
	struct drm_connector		panel_connector;
	struct drm_connector		*connector;
	struct drm_panel		*panel;
	struct drm_bridge		*bridge;

	u32				max_bw;
};

int mxsfb_setup_crtc(struct drm_device *dev);
int mxsfb_create_output(struct drm_device *dev);

void mxsfb_crtc_enable(struct mxsfb_drm_private *mxsfb);
void mxsfb_crtc_disable(struct mxsfb_drm_private *mxsfb);
void mxsfb_plane_atomic_update(struct mxsfb_drm_private *mxsfb,
			       struct drm_plane_state *old_state);

#endif /* __MXSFB_DRV_H__ */
