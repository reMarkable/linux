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
 * @file vpu_pm.c
 *
 */

#include "vpu_pm.h"

int vpu_attach_pm_domains(struct vpu_dev *dev)
{
	int ret = 0;

	dev->pd_vpu = dev_pm_domain_attach_by_name(&dev->plat_dev->dev, "vpu");
	if (IS_ERR(dev->pd_vpu)) {
		vpu_err("error: %s unable to get vpu power domain\n", __func__);
		ret = PTR_ERR(dev->pd_vpu);
		goto err;
	}
	dev->pd_vpu_link = device_link_add(&dev->plat_dev->dev, dev->pd_vpu,
		DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_vpu_link)) {
		vpu_err("error: %s unable to link vpu power domain\n", __func__);
		ret = PTR_ERR(dev->pd_vpu_link);
		goto err;
	}

	dev->pd_dec = dev_pm_domain_attach_by_name(&dev->plat_dev->dev, "vpudec");
	if (IS_ERR(dev->pd_dec)) {
		vpu_err("error: %s unable to get vpu dec power domain\n", __func__);
		ret = PTR_ERR(dev->pd_dec);
		goto err;
	}
	dev->pd_dec_link = device_link_add(&dev->plat_dev->dev, dev->pd_dec,
		DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_dec_link)) {
		vpu_err("error: %s unable to link vpu dec power domain\n", __func__);
		ret = PTR_ERR(dev->pd_dec_link);
		goto err;
	}

	dev->pd_mu = dev_pm_domain_attach_by_name(&dev->plat_dev->dev, "vpumu0");
	if (IS_ERR(dev->pd_mu)) {
		vpu_err("error: %s unable to get vpu mu0 power domain\n", __func__);
		ret = PTR_ERR(dev->pd_mu);
		goto err;
	}
	dev->pd_mu_link = device_link_add(&dev->plat_dev->dev, dev->pd_mu,
		DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_mu_link)) {
		vpu_err("error: %s unable to link vpu mu0 power domain\n", __func__);
		ret = PTR_ERR(dev->pd_mu_link);
		goto err;
	}

	return ret;
err:
	vpu_detach_pm_domains(dev);
	return ret;
}

void vpu_detach_pm_domains(struct vpu_dev *dev)
{
	if (dev->pd_vpu_link && !IS_ERR(dev->pd_vpu_link))
		device_link_del(dev->pd_vpu_link);
	if (dev->pd_vpu && !IS_ERR(dev->pd_vpu))
		dev_pm_domain_detach(dev->pd_vpu, true);

	if (dev->pd_dec_link && !IS_ERR(dev->pd_dec_link))
		device_link_del(dev->pd_dec_link);
	if (dev->pd_dec && !IS_ERR(dev->pd_dec))
		dev_pm_domain_detach(dev->pd_dec, true);

	if (dev->pd_mu_link && !IS_ERR(dev->pd_mu_link))
		device_link_del(dev->pd_mu_link);
	if (dev->pd_mu && !IS_ERR(dev->pd_mu))
		dev_pm_domain_detach(dev->pd_mu, true);

	dev->pd_vpu = NULL;
	dev->pd_vpu_link = NULL;
	dev->pd_dec = NULL;
	dev->pd_dec_link = NULL;
	dev->pd_mu = NULL;
	dev->pd_mu_link = NULL;
}
