/* Copyright 2015 Freescale Semiconductor Inc.
 * Copyright 2018-2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/msi.h>
#include <linux/rtnetlink.h>
#include <linux/if_vlan.h>

#include <uapi/linux/if_bridge.h>
#include <net/netlink.h>

#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>

#include <linux/fsl/mc.h>

#include "dpmac.h"
#include "dpmac-cmd.h"

struct dpaa2_mac_priv {
	struct net_device		*netdev;
	struct fsl_mc_device		*mc_dev;
	struct dpmac_attr		attr;
	struct dpmac_link_state		old_state;
	u16				dpmac_ver_major;
	u16				dpmac_ver_minor;
};

/* TODO: fix the 10G modes, mapping can't be right:
 *  XGMII is paralel
 *  XAUI is serial, using 8b/10b encoding
 *  XFI is also serial but using 64b/66b encoding
 * they can't all map to XGMII...
 *
 * This must be kept in sync with enum dpmac_eth_if.
 */
static phy_interface_t dpaa2_mac_iface_mode[] =  {
	PHY_INTERFACE_MODE_MII,		/* DPMAC_ETH_IF_MII */
	PHY_INTERFACE_MODE_RMII,	/* DPMAC_ETH_IF_RMII */
	PHY_INTERFACE_MODE_SMII,	/* DPMAC_ETH_IF_SMII */
	PHY_INTERFACE_MODE_GMII,	/* DPMAC_ETH_IF_GMII */
	PHY_INTERFACE_MODE_RGMII,	/* DPMAC_ETH_IF_RGMII */
	PHY_INTERFACE_MODE_SGMII,	/* DPMAC_ETH_IF_SGMII */
	PHY_INTERFACE_MODE_QSGMII,	/* DPMAC_ETH_IF_QSGMII */
	PHY_INTERFACE_MODE_XAUI,	/* DPMAC_ETH_IF_XAUI */
	PHY_INTERFACE_MODE_XGMII,	/* DPMAC_ETH_IF_XFI */
	PHY_INTERFACE_MODE_XGMII,        /* DPMAC_ETH_IF_CAUI */
	PHY_INTERFACE_MODE_1000BASEX,   /* DPMAC_ETH_IF_1000BASEX */
	PHY_INTERFACE_MODE_USXGMII,     /* DPMAC_ETH_IF_USXGMII */
};

static int cmp_dpmac_ver(struct dpaa2_mac_priv *priv,
			 u16 ver_major, u16 ver_minor)
{
	if (priv->dpmac_ver_major == ver_major)
		return priv->dpmac_ver_minor - ver_minor;
	return priv->dpmac_ver_major - ver_major;
}

#define DPMAC_LINK_AUTONEG_VER_MAJOR		4
#define DPMAC_LINK_AUTONEG_VER_MINOR		3

struct dpaa2_mac_link_mode_map {
	u64 dpmac_lm;
	u64 ethtool_lm;
};

static const struct dpaa2_mac_link_mode_map dpaa2_mac_lm_map[] = {
	{DPMAC_ADVERTISED_10BASET_FULL, ETHTOOL_LINK_MODE_10baseT_Full_BIT},
	{DPMAC_ADVERTISED_100BASET_FULL, ETHTOOL_LINK_MODE_100baseT_Full_BIT},
	{DPMAC_ADVERTISED_1000BASET_FULL, ETHTOOL_LINK_MODE_1000baseT_Full_BIT},
	{DPMAC_ADVERTISED_10000BASET_FULL, ETHTOOL_LINK_MODE_10000baseT_Full_BIT},
	{DPMAC_ADVERTISED_2500BASEX_FULL, ETHTOOL_LINK_MODE_2500baseT_Full_BIT},
	{DPMAC_ADVERTISED_AUTONEG, ETHTOOL_LINK_MODE_Autoneg_BIT},
};

static void link_mode_dpmac2phydev(u64 dpmac_lm, unsigned long *phydev_lm)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dpaa2_mac_lm_map); i++) {
		if (dpmac_lm & dpaa2_mac_lm_map[i].dpmac_lm)
			linkmode_set_bit(dpaa2_mac_lm_map[i].ethtool_lm, phydev_lm);
	}
}

static void link_mode_phydev2dpmac(unsigned long *phydev_lm, u64 *dpni_lm)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dpaa2_mac_lm_map); i++) {
		if (linkmode_test_bit(dpaa2_mac_lm_map[i].ethtool_lm, phydev_lm))
			*dpni_lm |= dpaa2_mac_lm_map[i].dpmac_lm;
	}
}

static void dpaa2_mac_link_changed(struct net_device *netdev)
{
	struct phy_device	*phydev;
	struct dpmac_link_state	state = { 0 };
	struct dpaa2_mac_priv	*priv = netdev_priv(netdev);
	int			err;

	/* the PHY just notified us of link state change */
	phydev = netdev->phydev;

	state.up = !!phydev->link;
	if (phydev->link) {
		state.rate = phydev->speed;

		if (!phydev->duplex)
			state.options |= DPMAC_LINK_OPT_HALF_DUPLEX;
		if (phydev->autoneg)
			state.options |= DPMAC_LINK_OPT_AUTONEG;

		if (phydev->pause && linkmode_test_bit(ETHTOOL_LINK_MODE_Pause_BIT, phydev->advertising))
			state.options |= DPMAC_LINK_OPT_PAUSE;
		if (phydev->pause && linkmode_test_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, phydev->advertising))
			state.options |= DPMAC_LINK_OPT_ASYM_PAUSE;

		netif_carrier_on(netdev);
	} else {
		netif_carrier_off(netdev);
	}

	/* Call the dpmac_set_link_state() only if there is a change in the
	 * link configuration
	 */
	if (priv->old_state.up == state.up &&
	    priv->old_state.rate == state.rate &&
	    priv->old_state.options == state.options)
		return;

	priv->old_state = state;
	phy_print_status(phydev);

	if (cmp_dpmac_ver(priv, DPMAC_LINK_AUTONEG_VER_MAJOR,
			  DPMAC_LINK_AUTONEG_VER_MINOR) < 0) {
		err = dpmac_set_link_state(priv->mc_dev->mc_io, 0,
					   priv->mc_dev->mc_handle, &state);
	} else {
		link_mode_phydev2dpmac(phydev->supported, &state.supported);
		link_mode_phydev2dpmac(phydev->advertising, &state.advertising);
		state.state_valid = 1;

		err = dpmac_set_link_state_v2(priv->mc_dev->mc_io, 0,
					      priv->mc_dev->mc_handle, &state);
	}
	if (unlikely(err))
		dev_err(&priv->mc_dev->dev, "dpmac_set_link_state: %d\n", err);
}

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
static int dpaa2_mac_open(struct net_device *netdev)
{
	/* start PHY state machine */
	phy_start(netdev->phydev);

	return 0;
}
#endif

static int dpaa2_mac_stop(struct net_device *netdev)
{
	if (!netdev->phydev)
		goto done;

	/* stop PHY state machine */
	phy_stop(netdev->phydev);

	/* signal link down to firmware */
	netdev->phydev->link = 0;
	dpaa2_mac_link_changed(netdev);

done:
	return 0;
}

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
static netdev_tx_t dpaa2_mac_drop_frame(struct sk_buff *skb,
					struct net_device *dev)
{
	/* we don't support I/O for now, drop the frame */
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void dpaa2_mac_get_drvinfo(struct net_device *net_dev,
				  struct ethtool_drvinfo *drvinfo)
{
	struct dpaa2_mac_priv *priv = netdev_priv(net_dev);

	strlcpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));
	snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
		 "%u.%u", priv->dpmac_ver_major, priv->dpmac_ver_minor);
	strlcpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static int dpaa2_mac_get_link_ksettings(struct net_device *netdev,
					struct ethtool_link_ksettings *ks)
{
	phy_ethtool_ksettings_get(netdev->phydev, ks);

	return 0;
}

static int dpaa2_mac_set_link_ksettings(struct net_device *netdev,
					const struct ethtool_link_ksettings *ks)
{
	return phy_ethtool_ksettings_set(netdev->phydev, ks);
}

static void dpaa2_mac_get_stats(struct net_device *netdev,
				struct rtnl_link_stats64 *storage)
{
	struct dpaa2_mac_priv	*priv = netdev_priv(netdev);
	u64			tmp;
	int			err;

	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_EGR_MCAST_FRAME,
				&storage->tx_packets);
	if (err)
		goto error;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_EGR_BCAST_FRAME, &tmp);
	if (err)
		goto error;
	storage->tx_packets += tmp;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_EGR_UCAST_FRAME, &tmp);
	if (err)
		goto error;
	storage->tx_packets += tmp;

	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_EGR_UNDERSIZED, &storage->tx_dropped);
	if (err)
		goto error;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_EGR_BYTE, &storage->tx_bytes);
	if (err)
		goto error;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_EGR_ERR_FRAME, &storage->tx_errors);
	if (err)
		goto error;

	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_ING_ALL_FRAME, &storage->rx_packets);
	if (err)
		goto error;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_ING_MCAST_FRAME, &storage->multicast);
	if (err)
		goto error;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_ING_FRAME_DISCARD,
				&storage->rx_dropped);
	if (err)
		goto error;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_ING_ALIGN_ERR, &storage->rx_errors);
	if (err)
		goto error;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_ING_OVERSIZED, &tmp);
	if (err)
		goto error;
	storage->rx_errors += tmp;
	err = dpmac_get_counter(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle,
				DPMAC_CNT_ING_BYTE, &storage->rx_bytes);
	if (err)
		goto error;

	return;
error:
	netdev_err(netdev, "dpmac_get_counter err %d\n", err);
}

static struct {
	enum dpmac_counter id;
	char name[ETH_GSTRING_LEN];
} dpaa2_mac_counters[] =  {
	{DPMAC_CNT_ING_ALL_FRAME,		"rx all frames"},
	{DPMAC_CNT_ING_GOOD_FRAME,		"rx frames ok"},
	{DPMAC_CNT_ING_ERR_FRAME,		"rx frame errors"},
	{DPMAC_CNT_ING_FRAME_DISCARD,		"rx frame discards"},
	{DPMAC_CNT_ING_UCAST_FRAME,		"rx u-cast"},
	{DPMAC_CNT_ING_BCAST_FRAME,		"rx b-cast"},
	{DPMAC_CNT_ING_MCAST_FRAME,		"rx m-cast"},
	{DPMAC_CNT_ING_FRAME_64,		"rx 64 bytes"},
	{DPMAC_CNT_ING_FRAME_127,		"rx 65-127 bytes"},
	{DPMAC_CNT_ING_FRAME_255,		"rx 128-255 bytes"},
	{DPMAC_CNT_ING_FRAME_511,		"rx 256-511 bytes"},
	{DPMAC_CNT_ING_FRAME_1023,		"rx 512-1023 bytes"},
	{DPMAC_CNT_ING_FRAME_1518,		"rx 1024-1518 bytes"},
	{DPMAC_CNT_ING_FRAME_1519_MAX,		"rx 1519-max bytes"},
	{DPMAC_CNT_ING_FRAG,			"rx frags"},
	{DPMAC_CNT_ING_JABBER,			"rx jabber"},
	{DPMAC_CNT_ING_ALIGN_ERR,		"rx align errors"},
	{DPMAC_CNT_ING_OVERSIZED,		"rx oversized"},
	{DPMAC_CNT_ING_VALID_PAUSE_FRAME,	"rx pause"},
	{DPMAC_CNT_ING_BYTE,			"rx bytes"},
	{DPMAC_CNT_ENG_GOOD_FRAME,		"tx frames ok"},
	{DPMAC_CNT_EGR_UCAST_FRAME,		"tx u-cast"},
	{DPMAC_CNT_EGR_MCAST_FRAME,		"tx m-cast"},
	{DPMAC_CNT_EGR_BCAST_FRAME,		"tx b-cast"},
	{DPMAC_CNT_EGR_ERR_FRAME,		"tx frame errors"},
	{DPMAC_CNT_EGR_UNDERSIZED,		"tx undersized"},
	{DPMAC_CNT_EGR_VALID_PAUSE_FRAME,	"tx b-pause"},
	{DPMAC_CNT_EGR_BYTE,			"tx bytes"},

};

static void dpaa2_mac_get_strings(struct net_device *netdev,
				  u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(dpaa2_mac_counters); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       dpaa2_mac_counters[i].name,
			       ETH_GSTRING_LEN);
		break;
	}
}

static void dpaa2_mac_get_ethtool_stats(struct net_device *netdev,
					struct ethtool_stats *stats,
					u64 *data)
{
	struct dpaa2_mac_priv	*priv = netdev_priv(netdev);
	int			i;
	int			err;

	for (i = 0; i < ARRAY_SIZE(dpaa2_mac_counters); i++) {
		err = dpmac_get_counter(priv->mc_dev->mc_io,
					0,
					priv->mc_dev->mc_handle,
					dpaa2_mac_counters[i].id, &data[i]);
		if (err)
			netdev_err(netdev, "dpmac_get_counter[%s] err %d\n",
				   dpaa2_mac_counters[i].name, err);
	}
}

static int dpaa2_mac_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(dpaa2_mac_counters);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops dpaa2_mac_ndo_ops = {
	.ndo_open		= &dpaa2_mac_open,
	.ndo_stop		= &dpaa2_mac_stop,
	.ndo_start_xmit		= &dpaa2_mac_drop_frame,
	.ndo_get_stats64	= &dpaa2_mac_get_stats,
};

static const struct ethtool_ops dpaa2_mac_ethtool_ops = {
	.get_drvinfo		= &dpaa2_mac_get_drvinfo,
	.get_link_ksettings	= &dpaa2_mac_get_link_ksettings,
	.set_link_ksettings	= &dpaa2_mac_set_link_ksettings,
	.get_strings		= &dpaa2_mac_get_strings,
	.get_ethtool_stats	= &dpaa2_mac_get_ethtool_stats,
	.get_sset_count		= &dpaa2_mac_get_sset_count,
};
#endif /* CONFIG_FSL_DPAA2_MAC_NETDEVS */

static void configure_link(struct dpaa2_mac_priv *priv,
			   struct dpmac_link_cfg *cfg)
{
	struct phy_device *phydev = priv->netdev->phydev;

	if (unlikely(!phydev))
		return;

	phydev->speed = cfg->rate;
	phydev->duplex  = !!(cfg->options & DPMAC_LINK_OPT_HALF_DUPLEX);

	if (cfg->advertising != 0) {
		linkmode_zero(phydev->advertising);
		link_mode_dpmac2phydev(cfg->advertising, phydev->advertising);
	}

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Pause_BIT, phydev->supported)) {
		if (cfg->options & DPMAC_LINK_OPT_PAUSE)
			linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, phydev->advertising);
		else
			linkmode_clear_bit(ETHTOOL_LINK_MODE_Pause_BIT, phydev->advertising);
	}

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, phydev->supported)) {
		if (cfg->options & DPMAC_LINK_OPT_ASYM_PAUSE)
			linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, phydev->advertising);
		else
			linkmode_clear_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, phydev->advertising);
	}

	if (cfg->options & DPMAC_LINK_OPT_AUTONEG) {
		phydev->autoneg = AUTONEG_ENABLE;
		linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->advertising);
	} else {
		phydev->autoneg = AUTONEG_DISABLE;
		linkmode_clear_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->advertising);
	}

	phy_start_aneg(phydev);
}

static irqreturn_t dpaa2_mac_irq_handler(int irq_num, void *arg)
{
	struct device *dev = (struct device *)arg;
	struct fsl_mc_device *mc_dev = to_fsl_mc_device(dev);
	struct dpaa2_mac_priv *priv = dev_get_drvdata(dev);
	struct net_device *ndev = priv->netdev;
	struct dpmac_link_cfg link_cfg = { 0 };
	u32 status;
	int err;

	err = dpmac_get_irq_status(mc_dev->mc_io, 0, mc_dev->mc_handle,
				   DPMAC_IRQ_INDEX, &status);
	if (unlikely(err || !status))
		return IRQ_NONE;

	/* DPNI-initiated link configuration; 'ifconfig up' also calls this */
	if (status & DPMAC_IRQ_EVENT_LINK_CFG_REQ) {
		if (cmp_dpmac_ver(priv, DPMAC_LINK_AUTONEG_VER_MAJOR,
				  DPMAC_LINK_AUTONEG_VER_MINOR) < 0)
			err = dpmac_get_link_cfg(mc_dev->mc_io, 0,
						 mc_dev->mc_handle, &link_cfg);
		else
			err = dpmac_get_link_cfg_v2(mc_dev->mc_io, 0,
						    mc_dev->mc_handle,
						    &link_cfg);
		if (unlikely(err))
			goto out;

		configure_link(priv, &link_cfg);
	}

	if (status & DPMAC_IRQ_EVENT_LINK_DOWN_REQ)
		phy_stop(ndev->phydev);

	if (status & DPMAC_IRQ_EVENT_LINK_UP_REQ)
		phy_start(ndev->phydev);
out:
	dpmac_clear_irq_status(mc_dev->mc_io, 0, mc_dev->mc_handle,
			       DPMAC_IRQ_INDEX, status);

	return IRQ_HANDLED;
}

static int setup_irqs(struct fsl_mc_device *mc_dev)
{
	int err = 0;
	struct fsl_mc_device_irq *irq;

	err = fsl_mc_allocate_irqs(mc_dev);
	if (err) {
		dev_err(&mc_dev->dev, "fsl_mc_allocate_irqs err %d\n", err);
		return err;
	}

	irq = mc_dev->irqs[0];
	err = devm_request_threaded_irq(&mc_dev->dev, irq->msi_desc->irq,
					NULL, &dpaa2_mac_irq_handler,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					dev_name(&mc_dev->dev), &mc_dev->dev);
	if (err) {
		dev_err(&mc_dev->dev, "devm_request_threaded_irq err %d\n",
			err);
		goto free_irq;
	}

	err = dpmac_set_irq_mask(mc_dev->mc_io, 0, mc_dev->mc_handle,
				 DPMAC_IRQ_INDEX, DPMAC_IRQ_EVENT_LINK_CFG_REQ |
				 DPMAC_IRQ_EVENT_LINK_UP_REQ |
				 DPMAC_IRQ_EVENT_LINK_DOWN_REQ);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_set_irq_mask err %d\n", err);
		goto free_irq;
	}
	err = dpmac_set_irq_enable(mc_dev->mc_io, 0, mc_dev->mc_handle,
				   DPMAC_IRQ_INDEX, 1);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_set_irq_enable err %d\n", err);
		goto free_irq;
	}

	return 0;

free_irq:
	fsl_mc_free_irqs(mc_dev);

	return err;
}

static void teardown_irqs(struct fsl_mc_device *mc_dev)
{
	int err;

	err = dpmac_set_irq_enable(mc_dev->mc_io, 0, mc_dev->mc_handle,
				   DPMAC_IRQ_INDEX, 0);
	if (err)
		dev_err(&mc_dev->dev, "dpmac_set_irq_enable err %d\n", err);

	fsl_mc_free_irqs(mc_dev);
}

static struct device_node *find_dpmac_node(struct device *dev, u16 dpmac_id)
{
	struct device_node *dpmacs, *dpmac = NULL;
	struct device_node *mc_node = dev->of_node;
	u32 id;
	int err;

	dpmacs = of_find_node_by_name(mc_node, "dpmacs");
	if (!dpmacs) {
		dev_err(dev, "No dpmacs subnode in device-tree\n");
		return NULL;
	}

	while ((dpmac = of_get_next_child(dpmacs, dpmac))) {
		err = of_property_read_u32(dpmac, "reg", &id);
		if (err)
			continue;
		if (id == dpmac_id)
			return dpmac;
	}

	return NULL;
}

static int dpaa2_mac_probe(struct fsl_mc_device *mc_dev)
{
	struct device		*dev;
	struct dpaa2_mac_priv	*priv = NULL;
	struct device_node	*phy_node, *dpmac_node;
	struct net_device	*netdev;
	int			if_mode;
	int			err = 0;

	dev = &mc_dev->dev;

	/* prepare a net_dev structure to make the phy lib API happy */
	netdev = alloc_etherdev(sizeof(*priv));
	if (!netdev) {
		dev_err(dev, "alloc_etherdev error\n");
		err = -ENOMEM;
		goto err_exit;
	}
	priv = netdev_priv(netdev);
	priv->mc_dev = mc_dev;
	priv->netdev = netdev;

	SET_NETDEV_DEV(netdev, dev);

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	snprintf(netdev->name, IFNAMSIZ, "mac%d", mc_dev->obj_desc.id);
#endif

	dev_set_drvdata(dev, priv);

	/* We may need to issue MC commands while in atomic context */
	err = fsl_mc_portal_allocate(mc_dev, FSL_MC_IO_ATOMIC_CONTEXT_PORTAL,
				     &mc_dev->mc_io);
	if (err || !mc_dev->mc_io) {
		dev_dbg(dev, "fsl_mc_portal_allocate error: %d\n", err);
		err = -EPROBE_DEFER;
		goto err_free_netdev;
	}

	err = dpmac_open(mc_dev->mc_io, 0, mc_dev->obj_desc.id,
			 &mc_dev->mc_handle);
	if (err || !mc_dev->mc_handle) {
		dev_err(dev, "dpmac_open error: %d\n", err);
		err = -ENODEV;
		goto err_free_mcp;
	}

	err = dpmac_get_api_version(mc_dev->mc_io, 0, &priv->dpmac_ver_major,
				    &priv->dpmac_ver_minor);
	if (err) {
		dev_err(dev, "dpmac_get_api_version failed\n");
		goto err_version;
	}

	if (cmp_dpmac_ver(priv, DPMAC_VER_MAJOR, DPMAC_VER_MINOR) < 0) {
		dev_err(dev, "DPMAC version %u.%u lower than supported %u.%u\n",
			priv->dpmac_ver_major, priv->dpmac_ver_minor,
			DPMAC_VER_MAJOR, DPMAC_VER_MINOR);
		err = -ENOTSUPP;
		goto err_version;
	}

	err = dpmac_get_attributes(mc_dev->mc_io, 0,
				   mc_dev->mc_handle, &priv->attr);
	if (err) {
		dev_err(dev, "dpmac_get_attributes err %d\n", err);
		err = -EINVAL;
		goto err_close;
	}

	/* Look up the DPMAC node in the device-tree. */
	dpmac_node = find_dpmac_node(dev, priv->attr.id);
	if (!dpmac_node) {
		dev_err(dev, "No dpmac@%d subnode found.\n", priv->attr.id);
		err = -ENODEV;
		goto err_close;
	}

	err = setup_irqs(mc_dev);
	if (err) {
		err = -EFAULT;
		goto err_close;
	}

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	/* OPTIONAL, register netdev just to make it visible to the user */
	netdev->netdev_ops = &dpaa2_mac_ndo_ops;
	netdev->ethtool_ops = &dpaa2_mac_ethtool_ops;

	err = register_netdev(priv->netdev);
	if (err < 0) {
		dev_err(dev, "register_netdev error %d\n", err);
		err = -ENODEV;
		goto err_free_irq;
	}
#endif /* CONFIG_FSL_DPAA2_MAC_NETDEVS */

	/* get the interface mode from the dpmac of node or from the MC attributes */
	if_mode = of_get_phy_mode(dpmac_node);
	if (if_mode >= 0) {
		dev_dbg(dev, "\tusing if mode %s for eth_if %d\n",
			phy_modes(if_mode), priv->attr.eth_if);
		goto link_type;
	}

	if (priv->attr.eth_if < ARRAY_SIZE(dpaa2_mac_iface_mode)) {
		if_mode = dpaa2_mac_iface_mode[priv->attr.eth_if];
		dev_dbg(dev, "\tusing if mode %s for eth_if %d\n",
			phy_modes(if_mode), priv->attr.eth_if);
	} else {
		dev_err(dev, "Unexpected interface mode %d\n",
			priv->attr.eth_if);
		err = -EINVAL;
		goto err_no_if_mode;
	}

link_type:
	/* probe the PHY as fixed-link if the DPMAC attribute indicates so */
	if (priv->attr.link_type == DPMAC_LINK_TYPE_FIXED)
		goto probe_fixed_link;

	/* or if there's no phy-handle defined in the device tree */
	phy_node = of_parse_phandle(dpmac_node, "phy-handle", 0);
	if (!phy_node) {
		goto probe_fixed_link;
	}

	/* try to connect to the PHY */
	netdev->phydev = of_phy_connect(netdev, phy_node,
					&dpaa2_mac_link_changed, 0, if_mode);
	if (!netdev->phydev) {
		/* No need for dev_err(); the kernel's loud enough as it is. */
		dev_dbg(dev, "Can't of_phy_connect() now.\n");
		/* We might be waiting for the MDIO MUX to probe, so defer
		 * our own probing.
		 */
		err = -EPROBE_DEFER;
		goto err_defer;
	}
	dev_info(dev, "Connected to %s PHY.\n", phy_modes(if_mode));

probe_fixed_link:
	if (!netdev->phydev) {
		struct fixed_phy_status status = {
			.link = 1,
			/* fixed-phys don't support 10Gbps speed for now */
			.speed = 1000,
			.duplex = 1,
		};

		/* try to register a fixed link phy */
		netdev->phydev = fixed_phy_register(PHY_POLL, &status, NULL);
		if (!netdev->phydev || IS_ERR(netdev->phydev)) {
			dev_err(dev, "error trying to register fixed PHY\n");
			/* So we don't crash unregister_netdev() later on */
			netdev->phydev = NULL;
			err = -EFAULT;
			goto err_no_phy;
		}

		err = phy_connect_direct(netdev, netdev->phydev,
					 &dpaa2_mac_link_changed, if_mode);
		if (err) {
			dev_err(dev, "error trying to connect to PHY\n");
			goto err_no_phy;
		}

		dev_info(dev, "Registered fixed PHY.\n");
	}

	return 0;

err_no_if_mode:
err_defer:
err_no_phy:
#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	unregister_netdev(netdev);
err_free_irq:
#endif
	teardown_irqs(mc_dev);
err_version:
err_close:
	dpmac_close(mc_dev->mc_io, 0, mc_dev->mc_handle);
err_free_mcp:
	fsl_mc_portal_free(mc_dev->mc_io);
err_free_netdev:
	free_netdev(netdev);
err_exit:
	return err;
}

static int dpaa2_mac_remove(struct fsl_mc_device *mc_dev)
{
	struct device		*dev = &mc_dev->dev;
	struct dpaa2_mac_priv	*priv = dev_get_drvdata(dev);
	struct net_device	*netdev = priv->netdev;

	if (netdev->flags & IFF_UP)
		dpaa2_mac_stop(netdev);

	if (phy_is_pseudo_fixed_link(netdev->phydev))
		fixed_phy_unregister(netdev->phydev);
	else
		phy_disconnect(netdev->phydev);
	netdev->phydev = NULL;

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	unregister_netdev(priv->netdev);
#endif
	teardown_irqs(priv->mc_dev);
	dpmac_close(priv->mc_dev->mc_io, 0, priv->mc_dev->mc_handle);
	fsl_mc_portal_free(priv->mc_dev->mc_io);
	free_netdev(priv->netdev);

	dev_set_drvdata(dev, NULL);

	return 0;
}

static const struct fsl_mc_device_id dpaa2_mac_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpmac",
	},
	{ .vendor = 0x0 }
};
MODULE_DEVICE_TABLE(fslmc, dpaa2_mac_match_id_table);

static struct fsl_mc_driver dpaa2_mac_drv = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
	},
	.probe		= dpaa2_mac_probe,
	.remove		= dpaa2_mac_remove,
	.match_id_table = dpaa2_mac_match_id_table,
};

module_fsl_mc_driver(dpaa2_mac_drv);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DPAA2 PHY proxy interface driver");
