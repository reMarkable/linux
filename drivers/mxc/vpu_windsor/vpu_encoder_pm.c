/*
 * Copyright 2019 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *
 * @file vpu_encoder_pm.c
 *
 */

#include "vpu_encoder_pm.h"

int vpu_enc_attach_pm_domains(struct vpu_dev *dev)
{
	int ret = 0;

	dev->pd_vpu = dev_pm_domain_attach_by_name(&dev->plat_dev->dev, "vpu");
	if (IS_ERR(dev->pd_vpu)) {
		vpu_err("error: unable to get vpu power domain\n");
		ret = PTR_ERR(dev->pd_vpu);
		goto err;
	}
	dev->pd_vpu_link = device_link_add(&dev->plat_dev->dev, dev->pd_vpu,
		DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_vpu_link)) {
		vpu_err("error: unable to link vpu power domain\n");
		ret = PTR_ERR(dev->pd_vpu_link);
		goto err;
	}

	dev->pd_enc1 = dev_pm_domain_attach_by_name(&dev->plat_dev->dev,
						    "vpuenc1");
	if (IS_ERR(dev->pd_enc1)) {
		vpu_err("error: unable to get vpu enc1 power domain\n");
		ret = PTR_ERR(dev->pd_enc1);
		goto err;
	}
	dev->pd_enc1_link = device_link_add(&dev->plat_dev->dev, dev->pd_enc1,
		DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_enc1_link)) {
		vpu_err("error: unable to link vpu enc1 power domain\n");
		ret = PTR_ERR(dev->pd_enc1_link);
		goto err;
	}

	dev->pd_mu1 = dev_pm_domain_attach_by_name(&dev->plat_dev->dev,
						   "vpumu1");
	if (IS_ERR(dev->pd_mu1)) {
		vpu_err("error: unable to get vpu mu1 power domain\n");
		ret = PTR_ERR(dev->pd_mu1);
		goto err;
	}
	dev->pd_mu1_link = device_link_add(&dev->plat_dev->dev, dev->pd_mu1,
		DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_mu1_link)) {
		vpu_err("error: unable to link vpu mu1 power domain\n");
		ret = PTR_ERR(dev->pd_mu1_link);
		goto err;
	}

	if (dev->plat_type == IMX8QM) {
		dev->pd_enc2 = dev_pm_domain_attach_by_name(&dev->plat_dev->dev,
							    "vpuenc2");
		if (IS_ERR(dev->pd_enc2)) {
			vpu_err("error: unable to get vpu enc2 power domain\n");
			ret = PTR_ERR(dev->pd_enc2);
			goto err;
		}
		dev->pd_enc2_link = device_link_add(&dev->plat_dev->dev, dev->pd_enc2,
			DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (IS_ERR(dev->pd_enc2_link)) {
			vpu_err("error: unable to link vpu enc2 power domain\n");
			ret = PTR_ERR(dev->pd_enc2_link);
			goto err;
		}

		dev->pd_mu2 = dev_pm_domain_attach_by_name(&dev->plat_dev->dev,
							   "vpumu2");
		if (IS_ERR(dev->pd_mu2)) {
			vpu_err("error: unable to get vpu mu2 power domain\n");
			ret = PTR_ERR(dev->pd_mu2);
			goto err;
		}

		dev->pd_mu2_link = device_link_add(&dev->plat_dev->dev, dev->pd_mu2,
			DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (IS_ERR(dev->pd_mu2_link)) {
			vpu_err("error: unable to link vpu mu2 power domain\n");
			ret = PTR_ERR(dev->pd_mu2_link);
			goto err;
		}
	}

	return ret;
err:
	vpu_enc_detach_pm_domains(dev);
	return ret;
}

void vpu_enc_detach_pm_domains(struct vpu_dev *dev)
{
	if (dev->pd_vpu_link && !IS_ERR(dev->pd_vpu_link))
		device_link_del(dev->pd_vpu_link);
	if (dev->pd_vpu && !IS_ERR(dev->pd_vpu))
		dev_pm_domain_detach(dev->pd_vpu, true);

	if (dev->pd_enc1_link && !IS_ERR(dev->pd_enc1_link))
		device_link_del(dev->pd_enc1_link);
	if (dev->pd_enc1 && !IS_ERR(dev->pd_enc1))
		dev_pm_domain_detach(dev->pd_enc1, true);

	if (dev->pd_mu1_link && !IS_ERR(dev->pd_mu1_link))
		device_link_del(dev->pd_mu1_link);
	if (dev->pd_mu1 && !IS_ERR(dev->pd_mu1))
		dev_pm_domain_detach(dev->pd_mu1, true);

	if (dev->plat_type == IMX8QM) {
		if (dev->pd_enc2_link && !IS_ERR(dev->pd_enc2_link))
			device_link_del(dev->pd_enc2_link);
		if (dev->pd_enc2 && !IS_ERR(dev->pd_enc2))
			dev_pm_domain_detach(dev->pd_enc2, true);
		if (dev->pd_mu2_link && !IS_ERR(dev->pd_mu2_link))
			device_link_del(dev->pd_mu2_link);
		if (dev->pd_mu2 && !IS_ERR(dev->pd_mu2))
			dev_pm_domain_detach(dev->pd_mu2, true);
	}

	dev->pd_vpu = NULL;
	dev->pd_vpu_link = NULL;
	dev->pd_enc1 = NULL;
	dev->pd_enc1_link = NULL;
	dev->pd_mu1 = NULL;
	dev->pd_mu1_link = NULL;
	if (dev->plat_type == IMX8QM) {
		dev->pd_enc2 = NULL;
		dev->pd_enc2_link = NULL;
		dev->pd_mu2 = NULL;
		dev->pd_mu2_link = NULL;
	}
}
