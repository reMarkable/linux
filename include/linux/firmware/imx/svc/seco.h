/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020 NXP
 *
 * Header file containing the public API for the System Controller (SC)
 * Security Controller (SECO) function.
 *
 * SECO_SVC (SVC) Security Controller Service
 *
 * Module for the Security Controller (SECO) service.
 */

#ifndef _SC_SECO_API_H
#define _SC_SECO_API_H

#include <linux/errno.h>
#include <linux/firmware/imx/sci.h>

/*
 * This type is used to indicate RPC RM function calls.
 */
enum imx_sc_seco_func {
	IMX_SC_SECO_FUNC_UNKNOWN = 0,
	IMX_SC_SECO_FUNC_BUILD_INFO = 16,
	IMX_SC_SECO_FUNC_SAB_MSG = 23,
};

#if IS_ENABLED(CONFIG_IMX_SCU)
int imx_sc_seco_build_info(struct imx_sc_ipc *ipc, uint32_t *version,
			   uint32_t *commit);
int imx_sc_seco_sab_msg(struct imx_sc_ipc *ipc, u64 smsg_addr);
#else /* IS_ENABLED(CONFIG_IMX_SCU) */
static inline
int imx_sc_seco_build_info(struct imx_sc_ipc *ipc, uint32_t *version,
			   uint32_t *commit)
{
	return -ENOTSUP;
}

static inline
int imx_sc_seco_sab_msg(struct imx_sc_ipc *ipc, u64 smsg_addr)
{
	return -ENOTSUP;
}
#endif /* IS_ENABLED(CONFIG_IMX_SCU) */

#endif /* _SC_SECO_API_H */
