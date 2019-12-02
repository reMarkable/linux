// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2019-2020 NXP
 */

/*
 * This driver allows to send messages to the SECO using a shared mailbox. The
 * messages must follow the protocol defined.
 */

/*
 * Architecture of the driver:
 *
 *                                     Non-Secure           +   Secure
 *                                                          |
 *                                                          |
 *                   +---------+      +-------------+       |
 *                   |seco_mu.c+<---->+imx-mailbox.c|       |
 *                   |         |      |  mailbox.c  +<-->+------+    +------+
 *                   +---+-----+      +-------------+    | MU X +<-->+ SECO |
 *                       |                               +------+    +------+
 *                       +----------------+                 |
 *                       |                |                 |
 *                       v                v                 |
 *                   logical           logical              |
 *                   receiver          waiter               |
 *                      +                 +                 |
 *                      |                 |                 |
 *                      |                 |                 |
 *                      |            +----+------+          |
 *                      |            |           |          |
 *                      |            |           |          |
 *               device_ctx     device_ctx     device_ctx   |
 *                                                          |
 *                 User 0        User 1       User Y        |
 *                 +------+      +------+     +------+      |
 *                 |misc.c|      |misc.c|     |misc.c|      |
 * kernel space    +------+      +------+     +------+      |
 *                                                          |
 *  +------------------------------------------------------ |
 *                     |             |           |          |
 * userspace     /dev/seco_muXch0    |           |          |
 *                          /dev/seco_muXch1     |          |
 *                                        /dev/seco_muXchY  |
 *                                                          |
 *
 * When a user sends a command to the seco, it registers its device_ctx as
 * waiter of a response from SECO
 *
 * A user can be registered as receiver of command by the SECO.
 *
 * When a message is received, the driver select the device_ctx receiving the
 * message depending on the tag in the message. It selects the device_ctx
 * accordingly.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/firmware/imx/sci.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/firmware/imx/seco_mu_ioctl.h>
#include <linux/mailbox_client.h>

#define MAX_RECV_SIZE 8
#define MAX_RECV_SIZE_BYTES (MAX_RECV_SIZE * sizeof(u32))
#define MAX_MESSAGE_SIZE 20
#define MAX_MESSAGE_SIZE_BYTES (MAX_MESSAGE_SIZE * sizeof(u32))
#define MESSAGE_SIZE(hdr) (((struct she_mu_hdr *)(&(hdr)))->size)
#define MESSAGE_TAG(hdr) (((struct she_mu_hdr *)(&(hdr)))->tag)

#define MESSAGING_TAG_COMMAND           (0x17u)
#define MESSAGING_TAG_RESPONSE          (0xe1u)

/* number of MU_TR registers */
#define MU_TR_COUNT		(4u)
/*  number of MU_RR registers */
#define MU_RR_COUNT		(4u)

#define SECURE_RAM_BASE_ADDRESS	(0x31800000ULL)
#define SECURE_RAM_BASE_ADDRESS_SCU	(0x20800000u)
#define SECURE_RAM_SIZE	(0x10000ULL)

#define SECO_MU_INTERRUPT_INDEX	(0u)
#define SECO_DEFAULT_MU_INDEX	(1u)
#define SECO_DEFAULT_TZ		(0u)
#define DEFAULT_DID		(0u)

#define MAX_DATA_SIZE_PER_USER  (16 * 1024)

/* Header of the messages exchange with the SECO */
struct she_mu_hdr {
	u8 ver;
	u8 size;
	u8 command;
	u8 tag;
}  __packed;

/* Status of a char device */
enum mu_device_status_t {
	MU_FREE,
	MU_OPENED
};

struct seco_shared_mem {
	dma_addr_t dma_addr;
	u32 size;
	u32 pos;
	u8 *ptr;
};

struct seco_out_buffer_desc {
	u8 *out_ptr;
	u8 *out_usr_ptr;
	u32 out_size;
	struct list_head link;
};

/* Private struct for each char device instance. */
struct seco_mu_device_ctx {
	struct device *dev;
	struct seco_mu_priv *mu_priv;
	struct miscdevice miscdev;

	enum mu_device_status_t status;
	wait_queue_head_t wq;
	struct semaphore fops_lock;

	u32 pending_hdr;
	struct list_head pending_out;

	struct seco_shared_mem secure_mem;
	struct seco_shared_mem non_secure_mem;

	u32 temp_cmd[MAX_MESSAGE_SIZE];
	u32 temp_resp[MAX_RECV_SIZE];
	u32 temp_resp_size;
};

/* Private struct for seco MU driver. */
struct seco_mu_priv {
	struct seco_mu_device_ctx *cmd_receiver_dev;
	struct seco_mu_device_ctx *waiting_rsp_dev;
	/*
	 * prevent parallel access to the MU registers
	 * e.g. a user trying to send a command while the other one is
	 * sending a response.
	 */
	struct mutex mu_lock;
	/*
	 * prevent a command to be sent on the MU while another one is still
	 * processing. (response to a command is allowed)
	 */
	struct mutex mu_cmd_lock;
	struct device *dev;
	u32 seco_mu_id;

	struct mbox_client cl;
	struct com_chan {
		struct mbox_client cl;
		struct mbox_chan *chan;
		u8 idx;
	} com_chans[MU_TR_COUNT + MU_RR_COUNT];
	struct mbox_chan *tx_started;

	struct imx_sc_ipc *ipc_scu;
	u8 seco_part_owner;

	void *debug_ptr;

	struct seco_mu_device_ctx *prot_ctx;
	u32 prot_buf[MAX_RECV_SIZE];
	u32 prot_word_received; /* bit arrray of the word received */
	u32 prot_idx;
	u32 prot_size;
};

/* macro to log operation of a misc device */
#define miscdev_dbg(p_miscdev, fmt, va_args...)                                \
	({                                                                     \
		struct miscdevice *_p_miscdev = p_miscdev;                     \
		dev_dbg((_p_miscdev)->parent, "%s: " fmt, (_p_miscdev)->name,  \
		##va_args);                                                    \
	})

#define miscdev_info(p_miscdev, fmt, va_args...)                               \
	({                                                                     \
		struct miscdevice *_p_miscdev = p_miscdev;                     \
		dev_info((_p_miscdev)->parent, "%s: " fmt, (_p_miscdev)->name, \
		##va_args);                                                    \
	})

#define miscdev_err(p_miscdev, fmt, va_args...)                                \
	({                                                                     \
		struct miscdevice *_p_miscdev = p_miscdev;                     \
		dev_err((_p_miscdev)->parent, "%s: " fmt, (_p_miscdev)->name,  \
		##va_args);                                                    \
	})

/* macro to log operation of a device context */
#define devctx_dbg(p_devctx, fmt, va_args...) \
	miscdev_dbg(&((p_devctx)->miscdev), fmt, ##va_args)
#define devctx_info(p_devctx, fmt, va_args...) \
	miscdev_info(&((p_devctx)->miscdev), fmt, ##va_args)
#define devctx_err(p_devctx, fmt, va_args...) \
	miscdev_err((&(p_devctx)->miscdev), fmt, ##va_args)

#define IMX_SC_RM_PERM_FULL         7U	/* Full access */

/* Give access to SECU to the memory we want to share */
static int seco_mu_setup_seco_memory_access(struct seco_mu_device_ctx *dev_ctx,
					    u64 addr, u32 len)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	int ret;
	u8 mr;

	ret = imx_sc_rm_find_memreg(priv->ipc_scu, &mr, addr, addr + len);
	if (ret) {
		devctx_err(dev_ctx, "Fail find memreg\n");
		goto exit;
	}

	ret = imx_sc_rm_set_memreg_permissions(priv->ipc_scu, mr,
					       priv->seco_part_owner,
					       IMX_SC_RM_PERM_FULL);
	if (ret) {
		devctx_err(dev_ctx, "Fail set permission for resource\n");
		goto exit;
	}

exit:
	return ret;
}

/*
 * File operations for user-space
 */
/* Open a char device. */
static int seco_mu_fops_open(struct inode *nd, struct file *fp)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	int err;

	/* Avoid race if opened at the same time */
	if (down_trylock(&dev_ctx->fops_lock))
		return -EBUSY;

	/* Authorize only 1 instance. */
	if (dev_ctx->status != MU_FREE) {
		err = -EBUSY;
		goto exit;
	}

	/*
	 * Allocate some memory for data exchanges with SECO.
	 * This will be used for data not requiring secure memory.
	 */
	dev_ctx->non_secure_mem.ptr = dmam_alloc_coherent(dev_ctx->dev,
					MAX_DATA_SIZE_PER_USER,
					&dev_ctx->non_secure_mem.dma_addr,
					GFP_KERNEL);
	if (!dev_ctx->non_secure_mem.ptr) {
		err = -ENOMEM;
		devctx_err(dev_ctx, "Failed to map shared memory with SECO\n");
		goto exit;
	}

	err = seco_mu_setup_seco_memory_access(dev_ctx,
					       dev_ctx->non_secure_mem.dma_addr,
					       MAX_DATA_SIZE_PER_USER);
	if (err) {
		err = -EPERM;
		devctx_err(dev_ctx,
			   "Failed to share access to shared memory\n");
		goto free_coherent;
	}

	dev_ctx->non_secure_mem.size = MAX_DATA_SIZE_PER_USER;
	dev_ctx->non_secure_mem.pos = 0;
	dev_ctx->status = MU_OPENED;

	dev_ctx->pending_hdr = 0;

	goto exit;

free_coherent:
	dmam_free_coherent(dev_ctx->mu_priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

exit:
	up(&dev_ctx->fops_lock);
	return err;
}

/* Close a char device. */
static int seco_mu_fops_close(struct inode *nd, struct file *fp)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv = dev_ctx->mu_priv;
	struct seco_out_buffer_desc *out_buf_desc;

	/* Avoid race if closed at the same time */
	if (down_trylock(&dev_ctx->fops_lock))
		return -EBUSY;

	/* The device context has not been opened */
	if (dev_ctx->status != MU_OPENED)
		goto exit;

	/* check if this device was registered as command receiver. */
	if (mu_priv->cmd_receiver_dev == dev_ctx)
		mu_priv->cmd_receiver_dev = NULL;

	/* check if this device was registered as waiting response. */
	if (mu_priv->waiting_rsp_dev == dev_ctx) {
		mu_priv->waiting_rsp_dev = NULL;
		mutex_unlock(&mu_priv->mu_cmd_lock);
	}

	/* Unmap secure memory shared buffer. */
	if (dev_ctx->secure_mem.ptr)
		devm_iounmap(dev_ctx->dev, dev_ctx->secure_mem.ptr);

	dev_ctx->secure_mem.ptr = NULL;
	dev_ctx->secure_mem.dma_addr = 0;
	dev_ctx->secure_mem.size = 0;
	dev_ctx->secure_mem.pos = 0;

	/* Free non-secure shared buffer. */
	dmam_free_coherent(dev_ctx->mu_priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

	dev_ctx->non_secure_mem.ptr = NULL;
	dev_ctx->non_secure_mem.dma_addr = 0;
	dev_ctx->non_secure_mem.size = 0;
	dev_ctx->non_secure_mem.pos = 0;

	while (!list_empty(&dev_ctx->pending_out)) {
		out_buf_desc = list_first_entry_or_null(&dev_ctx->pending_out,
						struct seco_out_buffer_desc,
						link);
		__list_del_entry(&out_buf_desc->link);
		devm_kfree(dev_ctx->dev, out_buf_desc);
	}

	dev_ctx->status = MU_FREE;

exit:
	up(&dev_ctx->fops_lock);
	return 0;
}

/* Write a message to the MU. */
static ssize_t seco_mu_fops_write(struct file *fp, const char __user *buf,
				  size_t size, loff_t *ppos)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv = dev_ctx->mu_priv;
	u32 *data, data_idx = 0, nb_words = 0, header;
	struct mbox_chan *chan;
	int err;

	devctx_dbg(dev_ctx, "write from buf (%p)%ld, ppos=%lld\n", buf, size,
		   ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	if (size < sizeof(struct she_mu_hdr)) {
		devctx_err(dev_ctx, "User buffer too small(%ld < %lu)\n", size,
			   sizeof(struct she_mu_hdr));
		err = -ENOSPC;
		goto exit;
	}

	if (size > MAX_MESSAGE_SIZE_BYTES) {
		devctx_err(dev_ctx, "User buffer too big(%ld > %lu)\n", size,
			   MAX_MESSAGE_SIZE_BYTES);
		err = -ENOSPC;
		goto exit;
	}

	/* Copy data to buffer */
	err = (int)copy_from_user(dev_ctx->temp_cmd, buf, size);
	if (err) {
		err = -EFAULT;
		devctx_err(dev_ctx, "Fail copy message from user\n");
		goto exit;
	}

	header = dev_ctx->temp_cmd[0];

	/* Check the message is valid according to tags */
	if (MESSAGE_TAG(header) == MESSAGING_TAG_COMMAND) {
		/*
		 * unlocked in seco_mu_receive_work_handler when the
		 * response to this command is received.
		 */
		mutex_lock(&mu_priv->mu_cmd_lock);
		mu_priv->waiting_rsp_dev = dev_ctx;
	} else if (MESSAGE_TAG(header) == MESSAGING_TAG_RESPONSE) {
		/* Check the device context can send the command */
		if (dev_ctx != mu_priv->cmd_receiver_dev) {
			devctx_err(dev_ctx,
				   "This channel is not configured to send response to SECO\n");
			err = -EPERM;
			goto exit;
		}
	} else {
		devctx_err(dev_ctx, "The message does not have a valid TAG\n");
		err = -EINVAL;
		goto exit;
	}

	/*
	 * Check that the size passed as argument matches the size
	 * carried in the message.
	 */
	nb_words = MESSAGE_SIZE(header);
	if (nb_words * sizeof(u32) != size) {
		devctx_err(dev_ctx, "User buffer too small\n");
		goto exit;
	}

	mutex_lock(&mu_priv->mu_lock);

	/* Send the first word along with the signaling */
	data = &dev_ctx->temp_cmd[data_idx];
	chan = mu_priv->com_chans[0].chan;
	devctx_dbg(dev_ctx, "sending[%d] %.8x\n", data_idx, *data);
	err = mbox_send_message(chan, data);
	if (err < 0) {
		devctx_err(dev_ctx, "Failed to send header\n");
		goto unlock;
	}

	devctx_dbg(dev_ctx, "%s\n", "signaling");
	err = mbox_send_message(mu_priv->tx_started, data);
	if (err < 0) {
		devctx_err(dev_ctx, "Failed to send signal\n");
		goto unlock;
	}

	data_idx = 1;

	/* Loop over the data of the message to send */
	while (data_idx < nb_words) {
		data = &dev_ctx->temp_cmd[data_idx];
		chan = mu_priv->com_chans[data_idx % MU_TR_COUNT].chan;

		devctx_dbg(dev_ctx, "sending[%d] %.8x\n", data_idx, *data);
		err = mbox_send_message(chan, data);
		if (err < 0) {
			devctx_err(dev_ctx, "Failed to send data %d\n",
				   data_idx);
			goto unlock;
		}
		data_idx++;
	}

	err = data_idx * (u32)sizeof(u32);

unlock:
	mutex_unlock(&mu_priv->mu_lock);

exit:
	up(&dev_ctx->fops_lock);
	return err;
}

/*
 * Read a message from the MU.
 * Blocking until a message is available.
 */
static ssize_t seco_mu_fops_read(struct file *fp, char __user *buf,
				 size_t size, loff_t *ppos)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	u32 data_size = 0, size_to_copy = 0;
	struct seco_out_buffer_desc *b_desc;
	int err;

	devctx_dbg(dev_ctx, "read to buf %p(%ld), ppos=%lld\n", buf, size,
		   ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	/* Wait until the complete message is received on the MU. */
	err = wait_event_interruptible(dev_ctx->wq, dev_ctx->pending_hdr != 0);
	if (err) {
		devctx_err(dev_ctx, "Interrupted by signal\n");
		goto exit;
	}

	devctx_dbg(dev_ctx, "%s %s\n", __func__,
		   "message received, start transmit to user");

	/* Check that the size passed as argument is larger than
	 * the one carried in the message.
	 */
	data_size = dev_ctx->temp_resp_size * sizeof(u32);
	size_to_copy = data_size;
	if (size_to_copy > size) {
		devctx_dbg(dev_ctx, "User buffer too small (%ld < %d)\n",
			   size, size_to_copy);
		size_to_copy = size;
	}

	/* We may need to copy the output data to user before
	 * delivering the completion message.
	 */
	while (!list_empty(&dev_ctx->pending_out)) {
		b_desc = list_first_entry_or_null(&dev_ctx->pending_out,
						  struct seco_out_buffer_desc,
						  link);
		if (b_desc->out_usr_ptr && b_desc->out_ptr) {
			devctx_dbg(dev_ctx, "Copy output data to user\n");
			err = (int)copy_to_user(b_desc->out_usr_ptr,
						b_desc->out_ptr,
						b_desc->out_size);
			if (err) {
				devctx_err(dev_ctx,
					   "Failed to copy output data to user\n");
				err = -EFAULT;
				goto exit;
			}
		}
		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}

	/* Copy data from the buffer */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4,
			     dev_ctx->temp_resp, size_to_copy, false);
	err = (int)copy_to_user(buf, dev_ctx->temp_resp, size_to_copy);
	if (err) {
		devctx_err(dev_ctx, "Failed to copy to user\n");
		err = -EFAULT;
		goto exit;
	}

	err = size_to_copy;

	/* free memory allocated on the shared buffers. */
	dev_ctx->secure_mem.pos = 0;
	dev_ctx->non_secure_mem.pos = 0;

	dev_ctx->pending_hdr = 0;

exit:
	up(&dev_ctx->fops_lock);
	return err;
}

/* Configure the shared memory according to user config */
static int
seco_mu_ioctl_shared_mem_cfg_handler(struct seco_mu_device_ctx *dev_ctx,
				     unsigned long arg)
{
	struct seco_mu_ioctl_shared_mem_cfg cfg;
	int err = -EINVAL;
	u64 high_boundary;

	/* Check if not already configured. */
	if (dev_ctx->secure_mem.dma_addr != 0u) {
		devctx_err(dev_ctx, "Shared memory not configured\n");
		goto exit;
	}

	err = (int)copy_from_user(&cfg, (u8 *)arg,
		sizeof(cfg));
	if (err) {
		devctx_err(dev_ctx, "Fail copy shared memory config to user\n");
		err = -EFAULT;
		goto exit;
	}

	devctx_dbg(dev_ctx, "cfg offset: %u(%d)\n", cfg.base_offset, cfg.size);

	high_boundary = cfg.base_offset;
	if (high_boundary > SECURE_RAM_SIZE) {
		devctx_err(dev_ctx, "base offset is over secure memory\n");
		err = -ENOMEM;
		goto exit;
	}

	high_boundary += cfg.size;
	if (high_boundary > SECURE_RAM_SIZE) {
		devctx_err(dev_ctx, "total memory is over secure memory\n");
		err = -ENOMEM;
		goto exit;
	}

	dev_ctx->secure_mem.dma_addr = (dma_addr_t)cfg.base_offset;
	dev_ctx->secure_mem.size = cfg.size;
	dev_ctx->secure_mem.pos = 0;
	dev_ctx->secure_mem.ptr = devm_ioremap_nocache(dev_ctx->dev,
					(phys_addr_t)(SECURE_RAM_BASE_ADDRESS +
					(u64)dev_ctx->secure_mem.dma_addr),
					dev_ctx->secure_mem.size);
	if (!dev_ctx->secure_mem.ptr) {
		devctx_err(dev_ctx, "Failed to map secure memory\n");
		err = -ENOMEM;
		goto exit;
	}

exit:
	return err;
}

/*
 * Copy a buffer of daa to/from the user and return the address to use in
 * messages
 */
static int seco_mu_ioctl_setup_iobuf_handler(struct seco_mu_device_ctx *dev_ctx,
					     unsigned long arg)
{
	struct seco_out_buffer_desc *out_buf_desc;
	struct seco_mu_ioctl_setup_iobuf io;
	struct seco_shared_mem *shared_mem;
	int err = -EINVAL;
	u32 pos;

	err = (int)copy_from_user(&io,
		(u8 *)arg,
		sizeof(io));
	if (err) {
		devctx_err(dev_ctx, "Failed copy iobuf config from user\n");
		err = -EFAULT;
		goto exit;
	}

	devctx_dbg(dev_ctx, "io [buf: %p(%d) flag: %x]\n",
		   io.user_buf, io.length, io.flags);

	if (io.length == 0 || !io.user_buf) {
		/*
		 * Accept NULL pointers since some buffers are optional
		 * in SECO commands. In this case we should return 0 as
		 * pointer to be embedded into the message.
		 * Skip all data copy part of code below.
		 */
		io.seco_addr = 0;
		goto copy;
	}

	/* Select the shared memory to be used for this buffer. */
	if (io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) {
		/* App requires to use secure memory for this buffer.*/
		shared_mem = &dev_ctx->secure_mem;
	} else {
		/* No specific requirement for this buffer. */
		shared_mem = &dev_ctx->non_secure_mem;
	}

	/* Check there is enough space in the shared memory. */
	if (io.length >= shared_mem->size - shared_mem->pos) {
		devctx_err(dev_ctx, "Not enough space in shared memory\n");
		err = -ENOMEM;
		goto exit;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;
	shared_mem->pos += round_up(io.length, 8u);
	io.seco_addr = (u64)shared_mem->dma_addr + pos;

	if ((io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) &&
	    !(io.flags & SECO_MU_IO_FLAGS_USE_SHORT_ADDR))
		/*Add base address to get full address.*/
		io.seco_addr += SECURE_RAM_BASE_ADDRESS_SCU;

	if (io.flags & SECO_MU_IO_FLAGS_IS_INPUT) {
		/*
		 * buffer is input:
		 * copy data from user space to this allocated buffer.
		 */
		err = (int)copy_from_user(shared_mem->ptr + pos, io.user_buf,
					  io.length);
		if (err) {
			devctx_err(dev_ctx,
				   "Failed copy data to shared memory\n");
			err = -EFAULT;
			goto exit;
		}
	} else {
		/*
		 * buffer is output:
		 * add an entry in the "pending buffers" list so data
		 * can be copied to user space when receiving SECO
		 * response.
		 */
		out_buf_desc = devm_kmalloc(dev_ctx->dev, sizeof(*out_buf_desc),
					    GFP_KERNEL);
		if (!out_buf_desc) {
			err = -ENOMEM;
			devctx_err(dev_ctx,
				   "Failed allocating mem for pending buffer\n"
				   );
			goto exit;
		}

		out_buf_desc->out_ptr = shared_mem->ptr + pos;
		out_buf_desc->out_usr_ptr = io.user_buf;
		out_buf_desc->out_size = io.length;
		list_add_tail(&out_buf_desc->link, &dev_ctx->pending_out);
	}

copy:
	/* Provide the seco address to user space only if success. */
	err = (int)copy_to_user((u8 *)arg, &io,
		sizeof(io));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy iobuff setup to user\n");
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

/* Retrieve info about the MU */
static int seco_mu_ioctl_get_mu_info_handler(struct seco_mu_device_ctx *dev_ctx,
					     unsigned long arg)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	struct seco_mu_ioctl_get_mu_info info;
	int err = -EINVAL;

	info.seco_mu_idx = (u8)priv->seco_mu_id;
	info.interrupt_idx = SECO_MU_INTERRUPT_INDEX;
	info.tz = SECO_DEFAULT_TZ;

	err = imx_sc_rm_get_did(priv->ipc_scu, &info.did);
	if (err) {
		devctx_err(dev_ctx, "Get did failed\n");
		goto exit;
	}

	devctx_dbg(dev_ctx,
		   "info [mu_idx: %d, irq_idx: %d, tz: 0x%x, did: 0x%x]\n",
		   info.seco_mu_idx, info.interrupt_idx, info.tz, info.did);

	err = (int)copy_to_user((u8 *)arg, &info,
		sizeof(info));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy mu info to user\n");
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

static int seco_mu_ioctl_signed_msg_handler(struct seco_mu_device_ctx *dev_ctx,
					    unsigned long arg)
{
	struct seco_shared_mem *shared_mem = &dev_ctx->non_secure_mem;
	struct seco_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	struct seco_mu_ioctl_signed_message msg;
	int err = -EINVAL;
	u64 addr;
	u32 pos;

	err = (int)copy_from_user(&msg,
		(u8 *)arg,
		sizeof(msg));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy from user: %d\n", err);
		err = -EFAULT;
		goto exit;
	}

	/* Check there is enough space in the shared memory. */
	if (msg.msg_size >= shared_mem->size - shared_mem->pos) {
		devctx_err(dev_ctx, "Not enough mem: %d left, %d required\n",
			   shared_mem->size - shared_mem->pos, msg.msg_size);
		err = -ENOMEM;
		goto exit;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;

	/* get physical address from the pos */
	addr = (u64)shared_mem->dma_addr + pos;

	/* copy signed message from user space to this allocated buffer */
	err = (int)copy_from_user(shared_mem->ptr + pos, msg.message,
				  msg.msg_size);
	if (err) {
		devctx_err(dev_ctx, "Failed to signed message from user: %d\n",
			   err);
		err = -EFAULT;
		goto exit;
	}

	/* Send the message to SECO through SCU */
	msg.error_code = imx_sc_seco_sab_msg(priv->ipc_scu, addr);

	err = (int)copy_to_user((u8 *)arg, &msg,
		sizeof(msg));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy to user: %d\n", err);
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

/* IOCTL entry point of a char device */
static long seco_mu_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv = dev_ctx->mu_priv;
	int err = -EINVAL;

	/* Prevent race during change of device context */
	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	switch (cmd) {
	case SECO_MU_IOCTL_ENABLE_CMD_RCV:
		if (!mu_priv->cmd_receiver_dev) {
			devctx_dbg(dev_ctx, "setting as receiver\n");
			mu_priv->cmd_receiver_dev = dev_ctx;
			err = 0;
		};
		break;
	case SECO_MU_IOCTL_SHARED_BUF_CFG:
		err = seco_mu_ioctl_shared_mem_cfg_handler(dev_ctx, arg);
		break;
	case SECO_MU_IOCTL_SETUP_IOBUF:
		err = seco_mu_ioctl_setup_iobuf_handler(dev_ctx, arg);
		break;
	case SECO_MU_IOCTL_GET_MU_INFO:
		err = seco_mu_ioctl_get_mu_info_handler(dev_ctx, arg);
		break;
	case SECO_MU_IOCTL_SIGNED_MESSAGE:
		err = seco_mu_ioctl_signed_msg_handler(dev_ctx, arg);
		break;
	default:
		err = -EINVAL;
		devctx_dbg(dev_ctx, "IOCTL %.8x not supported\n", cmd);
	}

	up(&dev_ctx->fops_lock);
	return (long)err;
}

/*
 * Callback called by mailbox FW when data are received
 */
static void seco_mu_rx_callback(struct mbox_client *c, void *msg)
{
	struct device *dev = c->dev;
	struct com_chan *com = container_of(c, struct com_chan, cl);
	struct seco_mu_priv *priv = dev_get_drvdata(dev);
	struct seco_mu_device_ctx *dev_ctx = priv->prot_ctx;
	u32 idx = com->idx;
	u32 val_msg;

	dev_dbg(dev, "Message received on mailbox\n");

	/* The function can be called with NULL msg */
	if (!msg) {
		dev_warn(dev, "Message is invalid\n");
		return;
	}

	val_msg = *(u32 *)msg;

	/* Store the message at its postion*/
	idx += (priv->prot_word_received & (1 << idx)) ? 4 : 0;
	priv->prot_buf[idx] = val_msg;
	priv->prot_word_received |= (1 << idx);
	priv->prot_idx++;
	dev_dbg(dev, "Received: %.8x from %d, idx: %d, prot_idx: %d\n", val_msg,
		com->idx, idx, priv->prot_idx);

	/* We check if we have received the header */
	if (idx == 0) {
		int msg_size;

		dev_dbg(dev, "Selecting device\n");

		/* Incoming command: wake up the receiver if any. */
		if (MESSAGE_TAG(val_msg) == MESSAGING_TAG_COMMAND) {
			dev_dbg(dev, "Selecting receiver\n");
			priv->prot_ctx = priv->cmd_receiver_dev;
		} else if (MESSAGE_TAG(val_msg) == MESSAGING_TAG_RESPONSE) {
			dev_dbg(dev, "Selecting waiter\n");
			/* This is a response. */
			priv->prot_ctx = priv->waiting_rsp_dev;
		} else {
			dev_err(dev,
				"Failed to select a device for message: %.8x\n",
				val_msg);
			return;
		}

		if (!priv->prot_ctx) {
			dev_err(dev, "The device context could not be set\n");
			return;
		}
		dev_ctx = priv->prot_ctx;

		/* Init reception */
		msg_size = MESSAGE_SIZE(val_msg);
		if (msg_size > MAX_MESSAGE_SIZE) {
			devctx_err(dev_ctx, "Message is too big (%d > %d)",
				   msg_size, MAX_RECV_SIZE);
			return;
		}
		priv->prot_size = msg_size;
	}

	/* Check end of reception */
	if (priv->prot_idx == priv->prot_size) {
		devctx_dbg(dev_ctx, "Transfert finished\n");

		/* Cleanup */
		priv->prot_ctx = NULL;
		memcpy(dev_ctx->temp_resp, priv->prot_buf, MAX_RECV_SIZE_BYTES);
		dev_ctx->temp_resp_size = priv->prot_size;

		priv->prot_idx = 0;
		priv->prot_size = 0;
		priv->prot_word_received = 0;

		/* Allow user to read and/or write */
		dev_ctx->pending_hdr = dev_ctx->temp_resp[0];
		wake_up_interruptible(&dev_ctx->wq);

		if (MESSAGE_TAG(dev_ctx->temp_resp[0]) ==
				MESSAGING_TAG_RESPONSE) {
			/*
			 * The response to the previous command is received.
			 * Allow following command to be sent on the MU.
			 */
			mutex_unlock(&priv->mu_cmd_lock);
		}
	}
}

#define SECO_FW_VER_FEAT_MASK		(0x0000FFF0u)
#define SECO_FW_VER_FEAT_SHIFT		(0x04u)
#define SECO_FW_VER_FEAT_MIN_ALL_MU	(0x04u)

/*
 * Get SECO FW version and check if it supports receiving commands on all MUs
 * The version is retrieved through SCU since this is the only communication
 * channel to SECO always present.
 */
static int seco_mu_check_all_mu_supported(struct device *dev)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev);
	u32 seco_ver;
	int ret;

	ret = imx_sc_seco_build_info(priv->ipc_scu, &seco_ver, NULL);
	if (ret) {
		dev_err(dev, "failed to retrieve SECO build info\n");
		goto exit;
	}

dev_info(dev, "build info: %.8x", seco_ver);

	if (((seco_ver & SECO_FW_VER_FEAT_MASK) >> SECO_FW_VER_FEAT_SHIFT)
		< SECO_FW_VER_FEAT_MIN_ALL_MU) {
		dev_err(dev, "current SECO FW do not support MU with Linux\n");
		ret = -ENOTSUPP;
		goto exit;
	}

exit:
	return ret;
}

/* Char driver setup */
static const struct file_operations seco_mu_fops = {
	.open		= seco_mu_fops_open,
	.owner		= THIS_MODULE,
	.read		= seco_mu_fops_read,
	.release	= seco_mu_fops_close,
	.write		= seco_mu_fops_write,
	.unlocked_ioctl = seco_mu_ioctl,
};

/* interface for managed res to free a mailbox channel */
static void if_mbox_free_channel(void *mbox_chan)
{
	mbox_free_channel(mbox_chan);
}

/* interface for managed res to unregister a char device */
static void if_misc_deregister(void *miscdevice)
{
	misc_deregister(miscdevice);
}

/* Driver probe.*/
static int seco_mu_probe(struct platform_device *pdev)
{
	struct seco_mu_device_ctx *dev_ctx;
	struct device *dev = &pdev->dev;
	struct seco_mu_priv *priv;
	struct mbox_chan *chan;
	struct device_node *np;
	int max_nb_users = 0;
	char *chan_name;
	char *devname;
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		dev_err(dev, "Fail allocate mem for private data\n");
		goto exit;
	}
	priv->dev = dev;
	dev_set_drvdata(dev, priv);

	/*
	 * Get the address of MU to be used for communication with the SCU
	 */
	np = pdev->dev.of_node;
	if (!np) {
		dev_err(dev, "Cannot find MU User entry in device tree\n");
		ret = -ENOTSUPP;
		goto exit;
	}

	ret = imx_scu_get_handle(&priv->ipc_scu);
	if (ret) {
		dev_err(dev, "Fail to retrieve IPC handle\n");
		goto exit;
	}

	ret = imx_sc_rm_get_resource_owner(priv->ipc_scu, IMX_SC_R_SECO,
					   &priv->seco_part_owner);
	if (ret) {
		dev_err(dev, "Fail get owner of SECO resource\n");
		goto exit;
	}

	ret = seco_mu_check_all_mu_supported(dev);
	if (ret) {
		dev_err(dev, "Fail seco_mu_check_all_mu_supported\n");
		goto exit;
	}

	/* Initialize the mutex. */
	mutex_init(&priv->mu_cmd_lock);
	mutex_init(&priv->mu_lock);

	priv->cmd_receiver_dev = NULL;
	priv->waiting_rsp_dev = NULL;

	ret = of_property_read_u32(np, "fsl,seco_mu_id", &priv->seco_mu_id);
	if (ret) {
		dev_warn(dev, "%s: Not able to read mu_id", __func__);
		priv->seco_mu_id = SECO_DEFAULT_MU_INDEX;
	}

	ret = of_property_read_u32(np, "fsl,seco_max_users", &max_nb_users);
	if (ret) {
		dev_warn(dev, "%s: Not able to read mu_max_user", __func__);
		max_nb_users = 2; /* 2 users max by default. */
	}

	/* Mailbox client configuration */
	priv->cl.dev = dev;
	priv->cl.tx_tout = 3000;
	priv->cl.knows_txdone = true;
	priv->cl.rx_callback = seco_mu_rx_callback;

	/* Create comm chans */
	for (i = 0; i < MU_TR_COUNT + MU_RR_COUNT; i++) {
		struct mbox_client *cl = &priv->com_chans[i].cl;

		if (i < MU_TR_COUNT)
			chan_name = devm_kasprintf(dev, GFP_KERNEL, "tx%d", i);
		else
			chan_name = devm_kasprintf(dev, GFP_KERNEL, "rx%d",
						   i - 4);

		if (!chan_name) {
			ret = -ENOMEM;
			dev_err(dev, "Failed to build chan name %d\n", i);
			goto exit;
		}

		memcpy(cl, &priv->cl, sizeof(priv->cl));

		dev_dbg(dev, "request mbox chan %s\n", chan_name);
		chan = mbox_request_channel_byname(cl, chan_name);
		if (IS_ERR(chan)) {
			ret = PTR_ERR(chan);
			if (ret != -EPROBE_DEFER)
				dev_err(dev,
					"Failed to request chan %s ret %d\n",
					chan_name, ret);
			goto exit;
		}
		priv->com_chans[i].chan = chan;
		priv->com_chans[i].idx = i - 4;

		ret = devm_add_action(dev, if_mbox_free_channel, chan);
		if (ret)
			dev_warn(dev, "failed to add devm removal of mbox\n");

		/* chan_name is not used anymore by mailbox framework */
		devm_kfree(dev, chan_name);
	}

	/* Create signaling chan */
	dev_dbg(dev, "request signal chan %s\n", chan_name);

	priv->tx_started = mbox_request_channel_byname(&priv->cl,
						       "tx_started");
	if (IS_ERR(priv->tx_started)) {
		ret = PTR_ERR(priv->tx_started);
		if (ret != -EPROBE_DEFER)
			dev_err(dev,
				"Failed to request chan %s ret %d\n",
				chan_name, ret);
		goto exit;
	}

	ret = devm_add_action(dev, if_mbox_free_channel,
			      priv->tx_started);
	if (ret)
		dev_warn(dev,
			 "failed to add managed removal of mbox\n");

	/* Create users */
	for (i = 0; i < max_nb_users; i++) {
		dev_ctx = devm_kzalloc(dev, sizeof(*dev_ctx), GFP_KERNEL);
		if (!dev_ctx) {
			ret = -ENOMEM;
			dev_err(dev,
				"Fail to allocate memory for device context\n");
			goto exit;
		}

		dev_ctx->dev = dev;
		dev_ctx->status = MU_FREE;
		dev_ctx->mu_priv = priv;
		/* Default value invalid for an header. */
		init_waitqueue_head(&dev_ctx->wq);

		INIT_LIST_HEAD(&dev_ctx->pending_out);
		sema_init(&dev_ctx->fops_lock, 1);

		devname = devm_kasprintf(dev, GFP_KERNEL, "seco_mu%d_ch%d",
					 priv->seco_mu_id, i);
		if (!devname) {
			ret = -ENOMEM;
			dev_err(dev,
				"Fail to allocate memory for misc dev name\n");
			goto exit;
		}

		dev_ctx->miscdev.name = devname;
		dev_ctx->miscdev.minor	= MISC_DYNAMIC_MINOR;
		dev_ctx->miscdev.fops = &seco_mu_fops;
		dev_ctx->miscdev.parent = dev;
		ret = misc_register(&dev_ctx->miscdev);
		if (ret) {
			dev_err(dev, "failed to register misc device %d\n",
				ret);
			goto exit;
		}

		ret = devm_add_action(dev, if_misc_deregister,
				      &dev_ctx->miscdev);
		if (ret)
			dev_warn(dev,
				 "failed to add managed removal of miscdev\n");
	}

exit:
	return ret;
}

static const struct of_device_id seco_mu_match[] = {
	{
		.compatible = "fsl,imx-seco-mu",
	},
	{},
};
MODULE_DEVICE_TABLE(of, seco_mu_match);

static struct platform_driver seco_mu_driver = {
	.driver = {
		.name = "seco_mu",
		.of_match_table = seco_mu_match,
	},
	.probe       = seco_mu_probe,
};

module_platform_driver(seco_mu_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IMX Seco MU");
MODULE_AUTHOR("NXP");
