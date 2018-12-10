/*
 * Copyright 2020 NXP
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
 * @file xuvi_pm.c
 *
 */

#include "xuvi_pm.h"
#define vpu_err printk
int xuvi_attach_pm_domains(struct ppm_dev *dev)
{
	int ret = 0;

	dev->pd_vpu =
	    dev_pm_domain_attach_by_name(&dev->plat_dev->dev, "vpu");
	if (IS_ERR(dev->pd_vpu)) {
		vpu_err("error: %s unable to get vpu power domain\n",
			__func__);
		ret = PTR_ERR(dev->pd_vpu);
		goto err;
	}
	dev->pd_vpu_link =
	    device_link_add(&dev->plat_dev->dev, dev->pd_vpu,
			    DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME |
			    DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_vpu_link)) {
		vpu_err("error: %s unable to link vpu power domain\n",
			__func__);
		ret = PTR_ERR(dev->pd_vpu_link);
		goto err;
	}

	dev->pd_ts =
	    dev_pm_domain_attach_by_name(&dev->plat_dev->dev, "vputs");
	if (IS_ERR(dev->pd_ts)) {
		vpu_err("error: %s unable to get vpu ts power domain\n",
			__func__);
		ret = PTR_ERR(dev->pd_ts);
		goto err;
	}
	dev->pd_ts_link = device_link_add(&dev->plat_dev->dev, dev->pd_ts,
					  DL_FLAG_STATELESS |
					  DL_FLAG_PM_RUNTIME |
					  DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_ts_link)) {
		vpu_err("error: %s unable to link vpu ts power domain\n",
			__func__);
		ret = PTR_ERR(dev->pd_ts_link);
		goto err;
	}

	dev->pd_mu =
	    dev_pm_domain_attach_by_name(&dev->plat_dev->dev, "vpumu3");
	if (IS_ERR(dev->pd_mu)) {
		vpu_err("error: %s unable to get vpu mu0 power domain\n",
			__func__);
		ret = PTR_ERR(dev->pd_mu);
		goto err;
	}
	dev->pd_mu_link = device_link_add(&dev->plat_dev->dev, dev->pd_mu,
					  DL_FLAG_STATELESS |
					  DL_FLAG_PM_RUNTIME |
					  DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(dev->pd_mu_link)) {
		vpu_err("error: %s unable to link vpu mu0 power domain\n",
			__func__);
		ret = PTR_ERR(dev->pd_mu_link);
		goto err;
	}

	return ret;
err:
	xuvi_detach_pm_domains(dev);
	return ret;
}

void xuvi_detach_pm_domains(struct ppm_dev *dev)
{
	if (dev->pd_vpu_link && !IS_ERR(dev->pd_vpu_link))
		device_link_del(dev->pd_vpu_link);
	if (dev->pd_vpu && !IS_ERR(dev->pd_vpu))
		dev_pm_domain_detach(dev->pd_vpu, true);

	if (dev->pd_ts_link && !IS_ERR(dev->pd_ts_link))
		device_link_del(dev->pd_ts_link);
	if (dev->pd_ts && !IS_ERR(dev->pd_ts))
		dev_pm_domain_detach(dev->pd_ts, true);

	if (dev->pd_mu_link && !IS_ERR(dev->pd_mu_link))
		device_link_del(dev->pd_mu_link);
	if (dev->pd_mu && !IS_ERR(dev->pd_mu))
		dev_pm_domain_detach(dev->pd_mu, true);

	dev->pd_vpu = NULL;
	dev->pd_vpu_link = NULL;
	dev->pd_ts = NULL;
	dev->pd_ts_link = NULL;
	dev->pd_mu = NULL;
	dev->pd_mu_link = NULL;
}
