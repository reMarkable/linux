// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 *
 * File containing client-side RPC functions for the SECO service. These
 * function are ported to clients that communicate to the SC.
 */

#include <linux/firmware/imx/sci.h>

struct imx_sc_msg_seco_get_build_id {
	struct imx_sc_rpc_msg hdr;
	u32 version;
	u32 commit;
};

int imx_sc_seco_build_info(struct imx_sc_ipc *ipc, uint32_t *version,
			   uint32_t *commit)
{
	struct imx_sc_msg_seco_get_build_id msg = {0};
	struct imx_sc_rpc_msg *hdr = &msg.hdr;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_SECO;
	hdr->func = IMX_SC_SECO_FUNC_BUILD_INFO;
	hdr->size = 1;

	imx_scu_call_rpc(ipc, &msg, true);

	if (version)
		*version = msg.version;
	if (commit)
		*commit = msg.commit;

	return 0;
}
EXPORT_SYMBOL(imx_sc_seco_build_info);

struct imx_sc_msg_seco_sab_msg {
	struct imx_sc_rpc_msg hdr;
	u32 smsg_addr_hi;
	u32 smsg_addr_lo;
};

int imx_sc_seco_sab_msg(struct imx_sc_ipc *ipc, u64 smsg_addr)
{
	struct imx_sc_msg_seco_sab_msg msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_SECO;
	hdr->func = IMX_SC_SECO_FUNC_SAB_MSG;
	hdr->size = 3;

	msg.smsg_addr_hi = smsg_addr >> 32;
	msg.smsg_addr_lo = smsg_addr;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	return ret;
}
EXPORT_SYMBOL(imx_sc_seco_sab_msg);
