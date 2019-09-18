// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2013-2016 Freescale Semiconductor Inc.
 * Copyright 2016-2017,2019 NXP
 */

#include <linux/device.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vfio.h>
#include <linux/fsl/mc.h>

#include "vfio_fsl_mc_private.h"


static int vfio_fsl_mc_open(void *device_data)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	return 0;
}

static void vfio_fsl_mc_release(void *device_data)
{
	module_put(THIS_MODULE);
}

static long vfio_fsl_mc_ioctl(void *device_data, unsigned int cmd,
			      unsigned long arg)
{
	unsigned long minsz;
	struct vfio_fsl_mc_device *vdev = device_data;
	struct fsl_mc_device *mc_dev = vdev->mc_dev;

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
	{
		struct vfio_device_info info;

		minsz = offsetofend(struct vfio_device_info, num_irqs);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		info.flags = VFIO_DEVICE_FLAGS_FSL_MC;
		info.num_regions = mc_dev->obj_desc.region_count;
		info.num_irqs = mc_dev->obj_desc.irq_count;

		return copy_to_user((void __user *)arg, &info, minsz) ?
			-EFAULT : 0;

	}
	case VFIO_DEVICE_GET_REGION_INFO:
	{
		return -EINVAL;
	}
	case VFIO_DEVICE_GET_IRQ_INFO:
	{
		return -EINVAL;
	}
	case VFIO_DEVICE_SET_IRQS:
	{
		return -EINVAL;
	}
	case VFIO_DEVICE_RESET:
	{
		return -EINVAL;
	}
	default:
		return -EINVAL;
	}
}

static ssize_t vfio_fsl_mc_read(void *device_data, char __user *buf,
				size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static ssize_t vfio_fsl_mc_write(void *device_data, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static int vfio_fsl_mc_mmap(void *device_data, struct vm_area_struct *vma)
{
	return -EINVAL;
}
static int vfio_fsl_mc_init_device(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	struct fsl_mc_bus *mc_bus;
	size_t region_size;
	int ret = 0;
	unsigned int irq_count;

	/* Non-dprc devices share mc_io from parent */
	if (!is_fsl_mc_bus_dprc(mc_dev)) {
		struct fsl_mc_device *mc_cont = to_fsl_mc_device(mc_dev->dev.parent);

		mc_dev->mc_io = mc_cont->mc_io;
		return 0;
	}

	/* Use dprc mc-portal for interaction with MC f/w. */
	region_size = resource_size(mc_dev->regions);
	ret = fsl_create_mc_io(&mc_dev->dev,
			 mc_dev->regions[0].start,
			 region_size,
			 NULL,
			 FSL_MC_IO_ATOMIC_CONTEXT_PORTAL,
			 &mc_dev->mc_io);
	if (ret < 0) {
		dev_err(&mc_dev->dev, "failed to create mc-io (error = %d)\n", ret);
		return ret;
	}
	ret = dprc_open(mc_dev->mc_io, 0, mc_dev->obj_desc.id,
			  &mc_dev->mc_handle);
	if (ret < 0) {
		dev_err(&mc_dev->dev, "dprc_open() failed: %d\n", ret);
		goto clean_mc_io;
	}

	mc_bus = to_fsl_mc_bus(mc_dev);
	/* initialize resource pools */
	fsl_mc_init_all_resource_pools(mc_dev);

	mutex_init(&mc_bus->scan_mutex);

	mutex_lock(&mc_bus->scan_mutex);
	ret = dprc_scan_objects(mc_dev, mc_dev->driver_override, false,
		&irq_count);
	mutex_unlock(&mc_bus->scan_mutex);

	if (ret < 0) {
		dev_err(&mc_dev->dev, "dprc_scan_objects() failed: %d\n", ret);
		goto clean_resource_pool;
	}

	if (irq_count > FSL_MC_IRQ_POOL_MAX_TOTAL_IRQS) {
		dev_warn(&mc_dev->dev,
			 "IRQs needed (%u) exceed IRQs preallocated (%u)\n",
			 irq_count, FSL_MC_IRQ_POOL_MAX_TOTAL_IRQS);
	}

return 0;

clean_resource_pool:
	fsl_mc_cleanup_all_resource_pools(mc_dev);
	dprc_close(mc_dev->mc_io, 0, mc_dev->mc_handle);

clean_mc_io:
	fsl_destroy_mc_io(mc_dev->mc_io);
	mc_dev->mc_io = NULL;

	return ret;
}
static const struct vfio_device_ops vfio_fsl_mc_ops = {
	.name		= "vfio-fsl-mc",
	.open		= vfio_fsl_mc_open,
	.release	= vfio_fsl_mc_release,
	.ioctl		= vfio_fsl_mc_ioctl,
	.read		= vfio_fsl_mc_read,
	.write		= vfio_fsl_mc_write,
	.mmap		= vfio_fsl_mc_mmap,
};

static int vfio_fsl_mc_probe(struct fsl_mc_device *mc_dev)
{
	struct iommu_group *group;
	struct vfio_fsl_mc_device *vdev;
	struct device *dev = &mc_dev->dev;
	int ret;

	group = vfio_iommu_group_get(dev);
	if (!group) {
		dev_err(dev, "%s: VFIO: No IOMMU group\n", __func__);
		return -EINVAL;
	}

	vdev = devm_kzalloc(dev, sizeof(*vdev), GFP_KERNEL);
	if (!vdev) {
		vfio_iommu_group_put(group, dev);
		return -ENOMEM;
	}

	vdev->mc_dev = mc_dev;

	ret = vfio_add_group_dev(dev, &vfio_fsl_mc_ops, vdev);
	if (ret) {
		dev_err(dev, "%s: Failed to add to vfio group\n", __func__);
		vfio_iommu_group_put(group, dev);
		return ret;
	}

	ret = vfio_fsl_mc_init_device(vdev);
	if (ret) {
		vfio_iommu_group_put(group, dev);
		return ret;
	}

	return ret;
}
static int vfio_fsl_mc_device_remove(struct device *dev, void *data)
{
	struct fsl_mc_device *mc_dev;

	WARN_ON(dev == NULL);
	mc_dev = to_fsl_mc_device(dev);
	if (WARN_ON(mc_dev == NULL))
		return -ENODEV;
	fsl_mc_device_remove(mc_dev);

	return 0;
}

static void vfio_fsl_mc_cleanup_dprc(struct fsl_mc_device *mc_dev)
{
	device_for_each_child(&mc_dev->dev, NULL, vfio_fsl_mc_device_remove);
	fsl_mc_cleanup_all_resource_pools(mc_dev);
	dprc_close(mc_dev->mc_io, 0, mc_dev->mc_handle);
	fsl_destroy_mc_io(mc_dev->mc_io);
}

static int vfio_fsl_mc_remove(struct fsl_mc_device *mc_dev)
{
	struct vfio_fsl_mc_device *vdev;
	struct device *dev = &mc_dev->dev;

	vdev = vfio_del_group_dev(dev);
	if (!vdev)
		return -EINVAL;

	if (is_fsl_mc_bus_dprc(mc_dev))
		vfio_fsl_mc_cleanup_dprc(vdev->mc_dev);

	mc_dev->mc_io = NULL;

	vfio_iommu_group_put(mc_dev->dev.iommu_group, dev);
	devm_kfree(dev, vdev);

	return 0;
}

/*
 * vfio-fsl_mc is a meta-driver, so use driver_override interface to
 * bind a fsl_mc container with this driver and match_id_table is NULL.
 */
static struct fsl_mc_driver vfio_fsl_mc_driver = {
	.probe		= vfio_fsl_mc_probe,
	.remove		= vfio_fsl_mc_remove,
	.match_id_table = NULL,
	.driver	= {
		.name	= "vfio-fsl-mc",
		.owner	= THIS_MODULE,
	},
};

static int __init vfio_fsl_mc_driver_init(void)
{
	return fsl_mc_driver_register(&vfio_fsl_mc_driver);
}

static void __exit vfio_fsl_mc_driver_exit(void)
{
	fsl_mc_driver_unregister(&vfio_fsl_mc_driver);
}

module_init(vfio_fsl_mc_driver_init);
module_exit(vfio_fsl_mc_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("VFIO for FSL-MC devices - User Level meta-driver");
