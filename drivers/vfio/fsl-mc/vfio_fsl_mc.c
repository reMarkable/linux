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
#include <linux/delay.h>

#include "vfio_fsl_mc_private.h"

static DEFINE_MUTEX(reflck_lock);

static void vfio_fsl_mc_reflck_get(struct vfio_fsl_mc_reflck *reflck)
{
	kref_get(&reflck->kref);
}

static void vfio_fsl_mc_reflck_release(struct kref *kref)
{
	struct vfio_fsl_mc_reflck *reflck = container_of(kref,
						      struct vfio_fsl_mc_reflck,
						      kref);

	kfree(reflck);
	mutex_unlock(&reflck_lock);
}

static void vfio_fsl_mc_reflck_put(struct vfio_fsl_mc_reflck *reflck)
{
	kref_put_mutex(&reflck->kref, vfio_fsl_mc_reflck_release, &reflck_lock);
}

static struct vfio_fsl_mc_reflck *vfio_fsl_mc_reflck_alloc(void)
{
	struct vfio_fsl_mc_reflck *reflck;

	reflck = kzalloc(sizeof(*reflck), GFP_KERNEL);
	if (!reflck)
		return ERR_PTR(-ENOMEM);

	kref_init(&reflck->kref);
	mutex_init(&reflck->lock);

	return reflck;
}

static int vfio_fsl_mc_reflck_attach(struct vfio_fsl_mc_device *vdev)
{
	int ret = 0;

	mutex_lock(&reflck_lock);
	if (is_fsl_mc_bus_dprc(vdev->mc_dev)) {
		vdev->reflck = vfio_fsl_mc_reflck_alloc();
	} else {
		struct device *mc_cont_dev = vdev->mc_dev->dev.parent;
		struct vfio_device *device;
		struct vfio_fsl_mc_device *cont_vdev;

		device = vfio_device_get_from_dev(mc_cont_dev);
		if (!device) {
			ret = -ENODEV;
			goto unlock;
		}

		cont_vdev = vfio_device_data(device);
		if (!cont_vdev->reflck) {
			vfio_device_put(device);
			ret = -ENODEV;
			goto unlock;
		}
		vfio_fsl_mc_reflck_get(cont_vdev->reflck);
		vdev->reflck = cont_vdev->reflck;
		vfio_device_put(device);
	}

unlock:
	mutex_unlock(&reflck_lock);
	return ret;
}

static int vfio_fsl_mc_regions_init(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	int count = mc_dev->obj_desc.region_count;
	int i;

	vdev->regions = kcalloc(count, sizeof(struct vfio_fsl_mc_region),
				GFP_KERNEL);
	if (!vdev->regions)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		struct resource *res = &mc_dev->regions[i];

		vdev->regions[i].addr = res->start;
		vdev->regions[i].size = PAGE_ALIGN((resource_size(res)));
		vdev->regions[i].flags = VFIO_REGION_INFO_FLAG_MMAP;
		vdev->regions[i].flags |= VFIO_REGION_INFO_FLAG_READ;
		if (!(mc_dev->regions[i].flags & IORESOURCE_READONLY))
			vdev->regions[i].flags |= VFIO_REGION_INFO_FLAG_WRITE;
		vdev->regions[i].type = mc_dev->regions[i].flags & IORESOURCE_BITS;
	}

	vdev->num_regions = mc_dev->obj_desc.region_count;
	return 0;
}
static void vfio_fsl_mc_regions_cleanup(struct vfio_fsl_mc_device *vdev)
{
	int i;

	for (i = 0; i < vdev->num_regions; i++)
		iounmap(vdev->regions[i].ioaddr);

	vdev->num_regions = 0;
	kfree(vdev->regions);
}

static int vfio_fsl_mc_open(void *device_data)
{
	struct vfio_fsl_mc_device *vdev = device_data;
	int ret;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	mutex_lock(&vdev->reflck->lock);
	if (!vdev->refcnt) {
		ret = vfio_fsl_mc_regions_init(vdev);
		if (ret)
			goto err_reg_init;
	}
	vdev->refcnt++;

	mutex_unlock(&vdev->reflck->lock);
	return 0;

err_reg_init:
	mutex_unlock(&vdev->reflck->lock);
	module_put(THIS_MODULE);
	return ret;
}

static void vfio_fsl_mc_release(void *device_data)
{
	struct vfio_fsl_mc_device *vdev = device_data;
	int ret;

	mutex_lock(&vdev->reflck->lock);

	if (!(--vdev->refcnt)) {
		struct fsl_mc_device *mc_dev = vdev->mc_dev;
		struct device *cont_dev = fsl_mc_cont_dev(&mc_dev->dev);
		struct fsl_mc_device *mc_cont = to_fsl_mc_device(cont_dev);
		struct fsl_mc_bus *mc_bus;

		mc_bus = to_fsl_mc_bus(mc_cont);

		vfio_fsl_mc_regions_cleanup(vdev);

		/* reset the device before cleaning up the interrupts */
		ret = dprc_reset_container(mc_dev->mc_io, 0,
		      mc_dev->mc_handle,
			  mc_dev->obj_desc.id);

		vfio_fsl_mc_irqs_cleanup(vdev);

		fsl_mc_cleanup_irq_pool(mc_bus);
	}

	mutex_unlock(&vdev->reflck->lock);

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
		struct vfio_region_info info;

		minsz = offsetofend(struct vfio_region_info, offset);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		if (info.index >= vdev->num_regions)
			return -EINVAL;

		/* map offset to the physical address  */
		info.offset = VFIO_FSL_MC_INDEX_TO_OFFSET(info.index);
		info.size = vdev->regions[info.index].size;
		info.flags = vdev->regions[info.index].flags;

		return copy_to_user((void __user *)arg, &info, minsz);

	}
	case VFIO_DEVICE_GET_IRQ_INFO:
	{
		struct vfio_irq_info info;

		minsz = offsetofend(struct vfio_irq_info, count);
		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		if (info.index >= mc_dev->obj_desc.irq_count)
			return -EINVAL;

		info.flags = VFIO_IRQ_INFO_EVENTFD;
		info.count = 1;

		return copy_to_user((void __user *)arg, &info, minsz);
	}
	case VFIO_DEVICE_SET_IRQS:
	{
		struct vfio_irq_set hdr;
		u8 *data = NULL;
		int ret = 0;

		minsz = offsetofend(struct vfio_irq_set, count);

		if (copy_from_user(&hdr, (void __user *)arg, minsz))
			return -EFAULT;

		if (hdr.argsz < minsz)
			return -EINVAL;

		if (hdr.index >= mc_dev->obj_desc.irq_count)
			return -EINVAL;

		if (hdr.start != 0 || hdr.count > 1)
			return -EINVAL;

		if (hdr.count == 0 &&
		    (!(hdr.flags & VFIO_IRQ_SET_DATA_NONE) ||
		    !(hdr.flags & VFIO_IRQ_SET_ACTION_TRIGGER)))
			return -EINVAL;

		if (hdr.flags & ~(VFIO_IRQ_SET_DATA_TYPE_MASK |
				  VFIO_IRQ_SET_ACTION_TYPE_MASK))
			return -EINVAL;

		if (!(hdr.flags & VFIO_IRQ_SET_DATA_NONE)) {
			size_t size;

			if (hdr.flags & VFIO_IRQ_SET_DATA_BOOL)
				size = sizeof(uint8_t);
			else if (hdr.flags & VFIO_IRQ_SET_DATA_EVENTFD)
				size = sizeof(int32_t);
			else
				return -EINVAL;

			if (hdr.argsz - minsz < hdr.count * size)
				return -EINVAL;

			data = memdup_user((void __user *)(arg + minsz),
					   hdr.count * size);
			if (IS_ERR(data))
				return PTR_ERR(data);
		}

		ret = vfio_fsl_mc_set_irqs_ioctl(vdev, hdr.flags,
						 hdr.index, hdr.start,
						 hdr.count, data);
		return ret;
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
	struct vfio_fsl_mc_device *vdev = device_data;
	unsigned int index = VFIO_FSL_MC_OFFSET_TO_INDEX(*ppos);
	loff_t off = *ppos & VFIO_FSL_MC_OFFSET_MASK;
	struct vfio_fsl_mc_region *region;
	uint64_t data[8];
	int i;

	/* Read ioctl supported only for DPRC and DPMCP device */
	if (strcmp(vdev->mc_dev->obj_desc.type, "dprc") &&
	    strcmp(vdev->mc_dev->obj_desc.type, "dpmcp"))
		return -EINVAL;

	if (index >= vdev->num_regions)
		return -EINVAL;

	region = &vdev->regions[index];

	if (!(region->flags & VFIO_REGION_INFO_FLAG_READ))
		return -EINVAL;


	if (!region->ioaddr) {
		region->ioaddr = ioremap_nocache(region->addr, region->size);
		if (!region->ioaddr)
			return -ENOMEM;
	}

	if (count != 64 || off != 0)
		return -EINVAL;

	for (i = 7; i >= 0; i--)
		data[i] = readq(region->ioaddr + i * sizeof(uint64_t));

	if (copy_to_user(buf, data, 64))
		return -EFAULT;

	return count;
}

#define MC_CMD_COMPLETION_TIMEOUT_MS    5000
#define MC_CMD_COMPLETION_POLLING_MAX_SLEEP_USECS    500

static int vfio_fsl_mc_send_command(void __iomem *ioaddr, uint64_t *cmd_data)
{
	int i;
	enum mc_cmd_status status;
	unsigned long timeout_usecs = MC_CMD_COMPLETION_TIMEOUT_MS * 1000;

	/* Write at command parameter into portal */
	for (i = 7; i >= 1; i--)
		writeq_relaxed(cmd_data[i], ioaddr + i * sizeof(uint64_t));

	/* Write command header in the end */
	writeq(cmd_data[0], ioaddr);

	/* Wait for response before returning to user-space
	 * This can be optimized in future to even prepare response
	 * before returning to user-space and avoid read ioctl.
	 */
	for (;;) {
		u64 header;
		struct mc_cmd_header *resp_hdr;

		header = cpu_to_le64(readq_relaxed(ioaddr));

		resp_hdr = (struct mc_cmd_header *)&header;
		status = (enum mc_cmd_status)resp_hdr->status;
		if (status != MC_CMD_STATUS_READY)
			break;

		udelay(MC_CMD_COMPLETION_POLLING_MAX_SLEEP_USECS);
		timeout_usecs -= MC_CMD_COMPLETION_POLLING_MAX_SLEEP_USECS;
		if (timeout_usecs == 0)
			return -ETIMEDOUT;
	}

	return 0;
}


static ssize_t vfio_fsl_mc_write(void *device_data, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct vfio_fsl_mc_device *vdev = device_data;
	unsigned int index = VFIO_FSL_MC_OFFSET_TO_INDEX(*ppos);
	loff_t off = *ppos & VFIO_FSL_MC_OFFSET_MASK;
	struct vfio_fsl_mc_region *region;
	uint64_t data[8];
	int ret;


	/* Write ioctl supported only for DPRC and DPMCP device */
	if (strcmp(vdev->mc_dev->obj_desc.type, "dprc") &&
	    strcmp(vdev->mc_dev->obj_desc.type, "dpmcp"))
		return -EINVAL;

	if (index >= vdev->num_regions)
		return -EINVAL;

	region = &vdev->regions[index];

	if (!(region->flags & VFIO_REGION_INFO_FLAG_WRITE))
		return -EINVAL;

	if (!region->ioaddr) {
		region->ioaddr = ioremap_nocache(region->addr, region->size);
		if (!region->ioaddr)
			return -ENOMEM;
	}

	if (count != 64 || off != 0)
		return -EINVAL;

	if (copy_from_user(&data, buf, 64))
		return -EFAULT;

	ret = vfio_fsl_mc_send_command(region->ioaddr, data);
	if (ret)
		return ret;

	return count;

}

static int vfio_fsl_mc_mmap_mmio(struct vfio_fsl_mc_region region,
				 struct vm_area_struct *vma)
{
	u64 size = vma->vm_end - vma->vm_start;
	u64 pgoff, base;

	pgoff = vma->vm_pgoff &
		((1U << (VFIO_FSL_MC_OFFSET_SHIFT - PAGE_SHIFT)) - 1);
	base = pgoff << PAGE_SHIFT;

	if (region.size < PAGE_SIZE || base + size > region.size)
		return -EINVAL;

	if (region.type & DPRC_REGION_CACHEABLE) {
		if (!(region.type & DPRC_REGION_SHAREABLE))
			vma->vm_page_prot = pgprot_cached_ns(vma->vm_page_prot);
	} else
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	vma->vm_pgoff = (region.addr >> PAGE_SHIFT) + pgoff;

	return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       size, vma->vm_page_prot);
}

static int vfio_fsl_mc_mmap(void *device_data, struct vm_area_struct *vma)
{
	struct vfio_fsl_mc_device *vdev = device_data;
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	unsigned long size, addr;
	int index;

	index = vma->vm_pgoff >> (VFIO_FSL_MC_OFFSET_SHIFT - PAGE_SHIFT);

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;
	if (vma->vm_start & ~PAGE_MASK)
		return -EINVAL;
	if (vma->vm_end & ~PAGE_MASK)
		return -EINVAL;
	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;
	if (index >= vdev->num_regions)
		return -EINVAL;

	if (!(vdev->regions[index].flags & VFIO_REGION_INFO_FLAG_MMAP))
		return -EINVAL;

	if (!(vdev->regions[index].flags & VFIO_REGION_INFO_FLAG_READ)
			&& (vma->vm_flags & VM_READ))
		return -EINVAL;

	if (!(vdev->regions[index].flags & VFIO_REGION_INFO_FLAG_WRITE)
			&& (vma->vm_flags & VM_WRITE))
		return -EINVAL;

	addr = vdev->regions[index].addr;
	size = vdev->regions[index].size;

	vma->vm_private_data = mc_dev;

	return vfio_fsl_mc_mmap_mmio(vdev->regions[index], vma);

	return -EFAULT;
}

static int vfio_fsl_mc_init_device(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	struct fsl_mc_bus *mc_bus;
	size_t region_size;
	int ret = 0;
	unsigned int irq_count;

	/* innherit the msi domain from parent */
	dev_set_msi_domain(&mc_dev->dev, dev_get_msi_domain(mc_dev->dev.parent));

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

	ret = vfio_fsl_mc_reflck_attach(vdev);
	if (ret) {
		vfio_iommu_group_put(group, dev);
		return ret;
	}

	ret = vfio_fsl_mc_init_device(vdev);
	if (ret) {
		vfio_fsl_mc_reflck_put(vdev->reflck);
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

	vfio_fsl_mc_reflck_put(vdev->reflck);

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
