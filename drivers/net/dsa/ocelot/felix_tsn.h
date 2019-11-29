/* SPDX-License-Identifier: (GPL-2.0 OR MIT)
 *
 * TSN_SWITCH driver
 *
 * Copyright 2018-2019 NXP
 */

#ifndef _MSCC_FELIX_SWITCH_TSN_H_
#define _MSCC_FELIX_SWITCH_TSN_H_
#include <net/tsn.h>

u32 felix_tsn_get_cap(struct net_device *ndev);
int felix_qbv_set(struct net_device *ndev,
		  struct tsn_qbv_conf *shaper_config);
int felix_qbv_get(struct net_device *ndev,
		  struct tsn_qbv_conf *shaper_config);
int felix_qbv_get_status(struct net_device *ndev,
			 struct tsn_qbv_status *qbvstatus);
int felix_cut_thru_set(struct net_device *ndev, u8 cut_thru);
int felix_cbs_set(struct net_device *ndev, u8 tc, u8 bw);
int felix_cbs_get(struct net_device *ndev, u8 tc);
int felix_qbu_set(struct net_device *ndev, u8 preemptible);
int felix_qbu_get(struct net_device *ndev, struct tsn_preempt_status *c);
int felix_cb_streamid_get(struct net_device *ndev, u32 index,
			  struct tsn_cb_streamid *streamid);
int felix_cb_streamid_set(struct net_device *ndev, u32 index,
			  bool enable, struct tsn_cb_streamid *streamid);
int felix_cb_streamid_counters_get(struct net_device *ndev, u32 index,
				   struct tsn_cb_streamid_counters *sc);
int felix_qci_sfi_get(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_sfi_conf *sfi);
int felix_qci_sfi_set(struct net_device *ndev, u32 index,
		      bool enable, struct tsn_qci_psfp_sfi_conf *sfi);
int felix_cb_streamid_counters_get(struct net_device *ndev, u32 index,
				   struct tsn_cb_streamid_counters *s_counters);
int felix_qci_sfi_counters_get(struct net_device *ndev, u32 index,
			       struct tsn_qci_psfp_sfi_counters *sfi_counters);
int felix_qci_max_cap_get(struct net_device *ndev,
			  struct tsn_qci_psfp_stream_param *stream_para);
int felix_qci_sgi_set(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_sgi_conf *sgi_conf);
int felix_qci_sgi_get(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_sgi_conf *sgi_conf);
int felix_qci_sgi_status_get(struct net_device *ndev, u16 index,
			     struct tsn_psfp_sgi_status *sgi_status);
int felix_qci_fmi_set(struct net_device *ndev, u32 index,
		      bool enable, struct tsn_qci_psfp_fmi *fmi);
int felix_qci_fmi_get(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_fmi *fmi,
		      struct tsn_qci_psfp_fmi_counters *counters);
int felix_seq_gen_set(struct net_device *ndev, u32 index,
		      struct tsn_seq_gen_conf *sg_conf);
int felix_seq_rec_set(struct net_device *ndev, u32 index,
		      struct tsn_seq_rec_conf *sr_conf);
int felix_cb_get(struct net_device *ndev, u32 index,
		 struct tsn_cb_status  *c);
int felix_dscp_set(struct net_device *ndev, bool enable, const u8 dscp_ix,
		   struct tsn_qos_switch_dscp_conf *c);

void felix_tsn_init(struct net_device *ndev);
#endif
