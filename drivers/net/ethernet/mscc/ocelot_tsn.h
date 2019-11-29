/* SPDX-License-Identifier: (GPL-2.0 OR MIT)
 *
 * TSN_SWITCH driver
 *
 * Copyright 2018-2019 NXP
 */

#ifndef _MSCC_OCELOT_SWITCH_TSN_H_
#define _MSCC_OCELOT_SWITCH_TSN_H_

#define TRUE 1
#define FALSE 0

struct mscc_switch_capa {
	u8 num_tas_gcl; /* Number of TAS Gate Control Lists */
	u32 tas_ct_min; /* Minimum supported TAS CycleTime in nS */
	u32 tas_ct_max; /* Maximum supported TAS CycleTime in nS */
	u32 tas_cte_max; /* Maximum supported TAS CycleTimeExtension in nS
			  */
	u32 tas_it_max;
	u32 tas_it_min;
	u8 num_hsch;
	u8 num_psfp_sfid;
	u8 num_frer_ssid;
	u8 num_psfp_sgid;
	u16 psfp_fmi_max;
	u16 psfp_fmi_min;
	u8 num_sgi_gcl;
	u32 sgi_ct_min;
	u32 sgi_ct_max;
	u32 sgi_cte_max;
	u16 qos_pol_max;
	u8 pol_cbs_max;
	u8 pol_pbs_max;
	u8 frer_seq_len_min;
	u8 frer_seq_len_max;
	u8 frer_his_len_min;
	u8 frer_his_len_max;
	u8 qos_dscp_max;
	u8 qos_cos_max;
	u8 qos_dp_max;
};

static inline void ocelot_port_rmwl(struct ocelot_port *port, u32 val,
				    u32 mask, u32 reg)
{
	u32 cur = ocelot_port_readl(port, reg);

	ocelot_port_writel(port, (cur & (~mask)) | val, reg);
}
#endif
