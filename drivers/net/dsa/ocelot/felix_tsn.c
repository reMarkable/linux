// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/* Felix Switch TSN driver
 *
 * Copyright 2018-2019 NXP
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <soc/mscc/ocelot.h>
#include <net/tsn.h>
#include "felix.h"

static struct ocelot *felix_dev_to_ocelot(struct net_device *ndev)
{
	struct pci_dev *pdev;
	struct felix *felix;

	pdev = list_entry(ndev->dev.parent, struct pci_dev, dev);
	felix = pci_get_drvdata(pdev);
	if (!felix)
		return NULL;

	return &felix->ocelot;
}

static int felix_dev_to_port(struct net_device *ndev, struct ocelot *ocelot)
{
	struct felix *felix = ocelot_to_felix(ocelot);
	struct dsa_switch *ds = felix->ds;
	struct dsa_port *dp;
	int i;

	for (i = 0; i < ds->num_ports; i++) {
		dp = &ds->ports[i];
		if (dp->dn == ndev->dev.of_node)
			return dp->index;
	}

	return -ENODEV;
}

u32 felix_tsn_get_cap(struct net_device *ndev)
{
	u32 cap = 0;

	cap = (TSN_CAP_QBV | TSN_CAP_QCI | TSN_CAP_QBU | TSN_CAP_CBS |
	       TSN_CAP_CB | TSN_CAP_TBS | TSN_CAP_CTH);

	return cap;
}

int felix_qbv_set(struct net_device *ndev,
		  struct tsn_qbv_conf *shaper_config)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qbv_set(ocelot, port, shaper_config);
}

int felix_qbv_get(struct net_device *ndev,
		  struct tsn_qbv_conf *shaper_config)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qbv_get(ocelot, port, shaper_config);
}

int felix_qbv_get_status(struct net_device *ndev,
			 struct tsn_qbv_status *qbvstatus)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qbv_get_status(ocelot, port, qbvstatus);
}

int felix_qbu_set(struct net_device *ndev, u8 preemptible)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qbu_set(ocelot, port, preemptible);
}

int felix_qbu_get(struct net_device *ndev, struct tsn_preempt_status *c)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qbu_get(ocelot, port, c);
}

int felix_cb_streamid_set(struct net_device *ndev, u32 index, bool enable,
			  struct tsn_cb_streamid *streamid)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_cb_streamid_set(ocelot, port, index, enable, streamid);
}

int felix_cb_streamid_get(struct net_device *ndev, u32 index,
			  struct tsn_cb_streamid *streamid)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_cb_streamid_get(ocelot, port, index, streamid);
}

int felix_cb_streamid_counters_get(struct net_device *ndev, u32 index,
				   struct tsn_cb_streamid_counters *sc)
{
	return 0;
}

int felix_qci_sfi_set(struct net_device *ndev, u32 index, bool enable,
		      struct tsn_qci_psfp_sfi_conf *sfi)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_sfi_set(ocelot, port, index, enable, sfi);
}

int felix_qci_sfi_get(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_sfi_conf *sfi)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_sfi_get(ocelot, port, index, sfi);
}

int felix_qci_sfi_counters_get(struct net_device *ndev, u32 index,
			       struct tsn_qci_psfp_sfi_counters *sfi_cnt)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_sfi_counters_get(ocelot, port, index, sfi_cnt);
}

int felix_qci_max_cap_get(struct net_device *ndev,
			  struct tsn_qci_psfp_stream_param *stream_para)
{
	struct ocelot *ocelot;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;

	return ocelot_qci_max_cap_get(ocelot, stream_para);
}

int felix_qci_sgi_set(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_sgi_conf *sgi_conf)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_sgi_set(ocelot, port, index, sgi_conf);
}

int felix_qci_sgi_get(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_sgi_conf *sgi_conf)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_sgi_get(ocelot, port, index, sgi_conf);
}

int felix_qci_sgi_status_get(struct net_device *ndev, u32 index,
			     struct tsn_psfp_sgi_status *sgi_status)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_sgi_status_get(ocelot, port, index, sgi_status);
}

int felix_qci_fmi_set(struct net_device *ndev, u32 index,
		      bool enable, struct tsn_qci_psfp_fmi *fmi)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_fmi_set(ocelot, port, index, enable, fmi);
}

int felix_qci_fmi_get(struct net_device *ndev, u32 index,
		      struct tsn_qci_psfp_fmi *fmi,
		      struct tsn_qci_psfp_fmi_counters *counters)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_qci_fmi_get(ocelot, port, index, fmi, counters);
}

int felix_cbs_set(struct net_device *ndev, u8 tc, u8 bw)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_cbs_set(ocelot, port, tc, bw);
}

int felix_cbs_get(struct net_device *ndev, u8 tc)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_cbs_get(ocelot, port, tc);
}

int felix_cut_thru_set(struct net_device *ndev, u8 cut_thru)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_cut_thru_set(ocelot, port, cut_thru);
}

int felix_seq_gen_set(struct net_device *ndev, u32 index,
		      struct tsn_seq_gen_conf *sg_conf)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_seq_gen_set(ocelot, port, index, sg_conf);
}

int felix_seq_rec_set(struct net_device *ndev, u32 index,
		      struct tsn_seq_rec_conf *sr_conf)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_seq_rec_set(ocelot, port, index, sr_conf);
}

int felix_cb_get(struct net_device *ndev, u32 index,
		 struct tsn_cb_status *c)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_cb_get(ocelot, port, index, c);
}

int felix_dscp_set(struct net_device *ndev, bool enable, const u8 dscp_ix,
		   struct tsn_qos_switch_dscp_conf *c)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return -ENODEV;
	port = felix_dev_to_port(ndev, ocelot);
	if (port < 0)
		return -ENODEV;

	return ocelot_dscp_set(ocelot, port, enable, dscp_ix, c);
}

void felix_tsn_init(struct net_device *ndev)
{
	struct ocelot *ocelot;
	int port;

	ocelot = felix_dev_to_ocelot(ndev);
	if (!ocelot)
		return;
	port = felix_dev_to_port(ndev, ocelot);

	ocelot_pcp_map_enable(ocelot, port);
	ocelot_rtag_parse_enable(ocelot, port);
}
