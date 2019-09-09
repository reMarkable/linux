// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drmP.h>
#include <linux/component.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

DEFINE_DRM_GEM_CMA_FOPS(dcss_cma_fops);

struct dcss_drm_commit {
	struct work_struct work;
	struct drm_device *drm;
	struct drm_atomic_state *state;
};

static void dcss_drm_atomic_commit_tail(struct dcss_drm_commit *commit)
{
	struct drm_atomic_state *state = commit->state;
	struct drm_device *drm = commit->drm;
	struct dcss_kms_dev *kms = container_of(drm, struct dcss_kms_dev, base);

	drm_atomic_helper_wait_for_fences(drm, state, false);

	drm_atomic_helper_wait_for_dependencies(state);

	drm_atomic_helper_commit_modeset_disables(drm, state);

	drm_atomic_helper_commit_modeset_enables(drm, state);

	drm_atomic_helper_commit_planes(drm, state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(drm, state);

	drm_atomic_helper_cleanup_planes(drm, state);

	drm_atomic_helper_commit_cleanup_done(state);

	drm_atomic_state_put(state);

	spin_lock(&kms->commit.wait.lock);
	kms->commit.pending = false;
	wake_up_all_locked(&kms->commit.wait);
	spin_unlock(&kms->commit.wait.lock);

	kfree(commit);
}

static void dcss_commit_work(struct work_struct *work)
{
	struct dcss_drm_commit *commit = container_of(work,
						      struct dcss_drm_commit,
						      work);

	dcss_drm_atomic_commit_tail(commit);
}

static int dcss_drm_atomic_commit(struct drm_device *drm,
				  struct drm_atomic_state *state,
				  bool nonblock)
{
	int ret;
	struct dcss_kms_dev *kms = container_of(drm, struct dcss_kms_dev, base);
	struct dcss_drm_commit *commit;

	if (state->async_update) {
		ret = drm_atomic_helper_prepare_planes(drm, state);
		if (ret)
			return ret;

		drm_atomic_helper_async_commit(drm, state);
		drm_atomic_helper_cleanup_planes(drm, state);

		return 0;
	}

	commit = kzalloc(sizeof(*commit), GFP_KERNEL);
	if (!commit)
		return -ENOMEM;

	commit->drm = drm;
	commit->state = state;

	ret = drm_atomic_helper_setup_commit(state, nonblock);
	if (ret)
		goto err_free;

	INIT_WORK(&commit->work, dcss_commit_work);

	ret = drm_atomic_helper_prepare_planes(drm, state);
	if (ret)
		goto err_free;

	if (!nonblock) {
		ret = drm_atomic_helper_wait_for_fences(drm, state, true);
		if (ret)
			goto err;
	}

	spin_lock(&kms->commit.wait.lock);
	ret = wait_event_interruptible_locked(kms->commit.wait,
					      !kms->commit.pending);
	if (ret == 0)
		kms->commit.pending = true;
	spin_unlock(&kms->commit.wait.lock);

	if (ret)
		goto err;

	ret = drm_atomic_helper_swap_state(state, true);
	if (ret)
		goto err;

	drm_atomic_state_get(state);
	if (nonblock)
		queue_work(kms->commit_wq, &commit->work);
	else
		dcss_drm_atomic_commit_tail(commit);

	return 0;

err:
	drm_atomic_helper_cleanup_planes(drm, state);

err_free:
	kfree(commit);
	return ret;
}

const struct drm_mode_config_funcs dcss_drm_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.output_poll_changed = drm_fb_helper_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = dcss_drm_atomic_commit,
};

static struct drm_driver dcss_kms_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,

	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,
	.fops			= &dcss_cma_fops,
	.name			= "imx-dcss",
	.desc			= "i.MX8MQ Display Subsystem",
	.date			= "20190917",
	.major			= 1,
	.minor			= 0,
	.patchlevel		= 0,
};

static const struct drm_mode_config_helper_funcs dcss_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

static void dcss_kms_mode_config_init(struct dcss_kms_dev *kms)
{
	struct drm_mode_config *config = &kms->base.mode_config;

	drm_mode_config_init(&kms->base);

	config->min_width = 1;
	config->min_height = 1;
	config->max_width = 4096;
	config->max_height = 4096;
	config->allow_fb_modifiers = true;
	config->normalize_zpos = true;

	config->funcs = &dcss_drm_mode_config_funcs;
	config->helper_private = &dcss_mode_config_helpers;
}

static const struct drm_encoder_funcs dcss_kms_simple_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int dcss_kms_setup_encoder(struct dcss_kms_dev *kms)
{
	struct drm_device *ddev = &kms->base;
	struct drm_encoder *encoder = &kms->encoder;
	struct drm_crtc *crtc = (struct drm_crtc *)&kms->crtc;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	int ret;

	ret = drm_of_find_panel_or_bridge(ddev->dev->of_node, 0, 0,
					  &panel, &bridge);
	if (ret)
		return ret;

	if (!bridge) {
		dev_err(ddev->dev, "No bridge found %d.\n", ret);
		return -ENODEV;
	}

	encoder->possible_crtcs = drm_crtc_mask(crtc);

	ret = drm_encoder_init(&kms->base, encoder,
			       &dcss_kms_simple_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret) {
		dev_err(ddev->dev, "Failed initializing encoder %d.\n", ret);
		return ret;
	}

	return drm_bridge_attach(encoder, bridge, NULL);
}

struct dcss_kms_dev *dcss_kms_attach(struct dcss_dev *dcss, bool componentized)
{
	struct dcss_kms_dev *kms = kzalloc(sizeof(*kms), GFP_KERNEL);
	struct drm_device *drm;
	struct dcss_crtc *crtc;
	int ret;

	if (!kms)
		return ERR_PTR(-ENOMEM);

	drm = &kms->base;
	crtc = &kms->crtc;
	ret = drm_dev_init(drm, &dcss_kms_driver, dcss->dev);
	if (ret)
		goto free_kms;

	drm->dev_private = dcss;

	dcss_kms_mode_config_init(kms);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto cleanup_mode_config;

	drm->irq_enabled = true;

	ret = dcss_crtc_init(crtc, drm);
	if (ret)
		goto cleanup_mode_config;

	kms->commit_wq = alloc_ordered_workqueue("dcss_nonblock_commit_wq", 0);
	if (!kms->commit_wq) {
		ret = -ENOMEM;
		goto cleanup_crtc;
	}

	init_waitqueue_head(&kms->commit.wait);

	if (componentized)
		ret = component_bind_all(dcss->dev, kms);
	else
		ret = dcss_kms_setup_encoder(kms);

	if (ret)
		goto cleanup_wq;

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto cleanup_wq;

	drm_fbdev_generic_setup(drm, 32);

	return kms;

cleanup_wq:
	drm_kms_helper_poll_fini(drm);
	destroy_workqueue(kms->commit_wq);

cleanup_crtc:
	dcss_crtc_deinit(crtc, drm);

cleanup_mode_config:
	drm_mode_config_cleanup(drm);

free_kms:
	kfree(kms);
	return ERR_PTR(ret);
}

void dcss_kms_detach(struct dcss_kms_dev *kms, bool componentized)
{
	struct drm_device *drm = &kms->base;
	struct dcss_dev *dcss = drm->dev_private;

	drm_dev_unregister(drm);
	drm_kms_helper_poll_fini(drm);
	drm_atomic_helper_shutdown(drm);
	drm_crtc_vblank_off(&kms->crtc.base);
	drm->irq_enabled = false;
	drm_mode_config_cleanup(drm);
	destroy_workqueue(kms->commit_wq);
	dcss_crtc_deinit(&kms->crtc, drm);
	if (componentized)
		component_unbind_all(dcss->dev, drm);
	drm->dev_private = NULL;
	drm_dev_put(drm);
}
