/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 NXP.
 */

#ifndef _DCSS_KMS_H_
#define _DCSS_KMS_H_

#include <drm/drm_encoder.h>

struct dcss_plane {
	struct drm_plane base;

	uint64_t dtrc_table_ofs_val;
	struct drm_property *dtrc_table_ofs_prop;

	int ch_num;

	enum drm_plane_type type;
	bool use_dtrc;
};

struct dcss_crtc {
	struct drm_crtc		base;
	struct drm_crtc_state	*state;

	struct dcss_plane	*plane[3];

	int			irq;
	bool			irq_enabled;

	struct completion en_completion;
	struct completion dis_completion;

	bool output_is_yuv;
	enum dcss_hdr10_nonlinearity opipe_nl;
	enum dcss_hdr10_gamut opipe_g;
	enum dcss_hdr10_pixel_range opipe_pr;
};

struct commit {
	wait_queue_head_t wait;
	bool pending;
};

struct dcss_kms_dev {
	struct drm_device base;
	struct dcss_crtc crtc;
	struct drm_encoder encoder;
	struct workqueue_struct *commit_wq;
	struct commit commit;
};

struct dcss_kms_dev *dcss_kms_attach(struct dcss_dev *dcss, bool componentized);
void dcss_kms_detach(struct dcss_kms_dev *kms, bool componentized);
int dcss_crtc_init(struct dcss_crtc *crtc, struct drm_device *drm);
void dcss_crtc_deinit(struct dcss_crtc *crtc, struct drm_device *drm);
struct dcss_plane *dcss_plane_init(struct drm_device *drm,
				   unsigned int possible_crtcs,
				   enum drm_plane_type type,
				   unsigned int zpos);
void dcss_crtc_attach_color_mgmt_properties(struct dcss_crtc *crtc);

#endif
