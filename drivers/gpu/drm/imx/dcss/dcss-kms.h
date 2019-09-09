/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef _DCSS_KMS_H_
#define _DCSS_KMS_H_

struct dcss_plane {
	struct drm_plane base;

	int ch_num;
};

struct dcss_crtc {
	struct drm_crtc		base;
	struct drm_crtc_state	*state;

	struct dcss_plane	*plane[3];

	int			irq;
	bool			irq_enabled;

	struct completion en_completion;
	struct completion dis_completion;
};

struct commit {
	wait_queue_head_t wait;
	bool pending;
};

struct dcss_kms_dev {
	struct drm_device base;
	struct dcss_crtc crtc;
	struct workqueue_struct *commit_wq;
	struct commit commit;
};

struct dcss_kms_dev *dcss_kms_attach(struct dcss_dev *dcss);
void dcss_kms_detach(struct dcss_kms_dev *kms);
int dcss_crtc_init(struct dcss_crtc *crtc, struct drm_device *drm);
struct dcss_plane *dcss_plane_init(struct drm_device *drm,
				   unsigned int possible_crtcs,
				   enum drm_plane_type type,
				   unsigned int zpos);

#endif
