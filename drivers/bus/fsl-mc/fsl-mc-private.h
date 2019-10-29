/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Freescale Management Complex (MC) bus private declarations
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 */
#ifndef _FSL_MC_PRIVATE_H_
#define _FSL_MC_PRIVATE_H_

#include <linux/fsl/mc.h>
#include <linux/mutex.h>

/*
 * Data Path Management Complex (DPMNG) General API
 */

/* DPMNG command versioning */
#define DPMNG_CMD_BASE_VERSION		1
#define DPMNG_CMD_ID_OFFSET		4

#define DPMNG_CMD(id)	(((id) << DPMNG_CMD_ID_OFFSET) | DPMNG_CMD_BASE_VERSION)

/* DPMNG command IDs */
#define DPMNG_CMDID_GET_VERSION		DPMNG_CMD(0x831)

struct dpmng_rsp_get_version {
	__le32 revision;
	__le32 version_major;
	__le32 version_minor;
};

/*
 * Data Path Management Command Portal (DPMCP) API
 */

/* Minimal supported DPMCP Version */
#define DPMCP_MIN_VER_MAJOR		3
#define DPMCP_MIN_VER_MINOR		0

/* DPMCP command versioning */
#define DPMCP_CMD_BASE_VERSION		1
#define DPMCP_CMD_ID_OFFSET		4

#define DPMCP_CMD(id)	(((id) << DPMCP_CMD_ID_OFFSET) | DPMCP_CMD_BASE_VERSION)

/* DPMCP command IDs */
#define DPMCP_CMDID_CLOSE		DPMCP_CMD(0x800)
#define DPMCP_CMDID_OPEN		DPMCP_CMD(0x80b)
#define DPMCP_CMDID_RESET		DPMCP_CMD(0x005)

struct dpmcp_cmd_open {
	__le32 dpmcp_id;
};

/*
 * Initialization and runtime control APIs for DPMCP
 */
int dpmcp_open(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       int dpmcp_id,
	       u16 *token);

int dpmcp_close(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		u16 token);

int dpmcp_reset(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		u16 token);

/*
 * Data Path Buffer Pool (DPBP) API
 */

/* DPBP Version */
#define DPBP_VER_MAJOR				3
#define DPBP_VER_MINOR				2

/* Command versioning */
#define DPBP_CMD_BASE_VERSION			1
#define DPBP_CMD_ID_OFFSET			4

#define DPBP_CMD(id)	(((id) << DPBP_CMD_ID_OFFSET) | DPBP_CMD_BASE_VERSION)

/* Command IDs */
#define DPBP_CMDID_CLOSE		DPBP_CMD(0x800)
#define DPBP_CMDID_OPEN			DPBP_CMD(0x804)

#define DPBP_CMDID_ENABLE		DPBP_CMD(0x002)
#define DPBP_CMDID_DISABLE		DPBP_CMD(0x003)
#define DPBP_CMDID_GET_ATTR		DPBP_CMD(0x004)
#define DPBP_CMDID_RESET		DPBP_CMD(0x005)

struct dpbp_cmd_open {
	__le32 dpbp_id;
};

#define DPBP_ENABLE			0x1

struct dpbp_rsp_get_attributes {
	/* response word 0 */
	__le16 pad;
	__le16 bpid;
	__le32 id;
	/* response word 1 */
	__le16 version_major;
	__le16 version_minor;
};

/*
 * Data Path Concentrator (DPCON) API
 */

/* DPCON Version */
#define DPCON_VER_MAJOR				3
#define DPCON_VER_MINOR				2

/* Command versioning */
#define DPCON_CMD_BASE_VERSION			1
#define DPCON_CMD_ID_OFFSET			4

#define DPCON_CMD(id)	(((id) << DPCON_CMD_ID_OFFSET) | DPCON_CMD_BASE_VERSION)

/* Command IDs */
#define DPCON_CMDID_CLOSE			DPCON_CMD(0x800)
#define DPCON_CMDID_OPEN			DPCON_CMD(0x808)

#define DPCON_CMDID_ENABLE			DPCON_CMD(0x002)
#define DPCON_CMDID_DISABLE			DPCON_CMD(0x003)
#define DPCON_CMDID_GET_ATTR			DPCON_CMD(0x004)
#define DPCON_CMDID_RESET			DPCON_CMD(0x005)

#define DPCON_CMDID_SET_NOTIFICATION		DPCON_CMD(0x100)

struct dpcon_cmd_open {
	__le32 dpcon_id;
};

#define DPCON_ENABLE			1

struct dpcon_rsp_get_attr {
	/* response word 0 */
	__le32 id;
	__le16 qbman_ch_id;
	u8 num_priorities;
	u8 pad;
};

struct dpcon_cmd_set_notification {
	/* cmd word 0 */
	__le32 dpio_id;
	u8 priority;
	u8 pad[3];
	/* cmd word 1 */
	__le64 user_ctx;
};

int __must_check fsl_mc_device_add(struct fsl_mc_obj_desc *obj_desc,
				   struct fsl_mc_io *mc_io,
				   struct device *parent_dev,
				   const char *driver_override,
				   struct fsl_mc_device **new_mc_dev);

int __init dprc_driver_init(void);

void dprc_driver_exit(void);

int __init fsl_mc_allocator_driver_init(void);

void fsl_mc_allocator_driver_exit(void);

int __must_check fsl_mc_resource_allocate(struct fsl_mc_bus *mc_bus,
					  enum fsl_mc_pool_type pool_type,
					  struct fsl_mc_resource
							  **new_resource);

void fsl_mc_resource_free(struct fsl_mc_resource *resource);

int fsl_mc_msi_domain_alloc_irqs(struct device *dev,
				 unsigned int irq_count);

void fsl_mc_msi_domain_free_irqs(struct device *dev);

bool fsl_mc_is_root_dprc(struct device *dev);

#ifdef CONFIG_FSL_MC_UAPI_SUPPORT

int fsl_mc_uapi_create_device_file(struct fsl_mc_bus *mc_bus);

void fsl_mc_uapi_remove_device_file(struct fsl_mc_bus *mc_bus);

#else

static inline int fsl_mc_uapi_create_device_file(struct fsl_mc_bus *mc_bus)
{
	return 0;
}

static inline void fsl_mc_uapi_remove_device_file(struct fsl_mc_bus *mc_bus)
{
}

#endif

int disable_dprc_irq(struct fsl_mc_device *mc_dev);
int enable_dprc_irq(struct fsl_mc_device *mc_dev);
int get_dprc_irq_state(struct fsl_mc_device *mc_dev);

#endif /* _FSL_MC_PRIVATE_H_ */
