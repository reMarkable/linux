/** @file mlan_11ax.h
 *
 *  @brief This file defines the private and adapter data
 *  structures and declares global function prototypes used
 *  in MLAN module.
 *
 *
 *  Copyright 2014-2020 NXP
 *
 *  NXP CONFIDENTIAL
 *  The source code contained or described herein and all documents related to
 *  the source code (Materials) are owned by NXP, its
 *  suppliers and/or its licensors. Title to the Materials remains with NXP,
 *  its suppliers and/or its licensors. The Materials contain
 *  trade secrets and proprietary and confidential information of NXP, its
 *  suppliers and/or its licensors. The Materials are protected by worldwide copyright
 *  and trade secret laws and treaty provisions. No part of the Materials may be
 *  used, copied, reproduced, modified, published, uploaded, posted,
 *  transmitted, distributed, or disclosed in any way without NXP's prior
 *  express written permission.
 *
 *  No license under any patent, copyright, trade secret or other intellectual
 *  property right is granted to or conferred upon you by disclosure or delivery
 *  of the Materials, either expressly, by implication, inducement, estoppel or
 *  otherwise. Any license under such intellectual property rights must be
 *  express and approved by NXP in writing.
 *
 */

#ifndef _MLAN_11AX_H_
#define _MLAN_11AX_H_

/** device support 2.4G 40MHZ*/
#define AX_2G_40MHZ_SUPPORT     MBIT(1)
/** device support 2.4G 242 tone RUs */
#define AX_2G_20MHZ_SUPPORT     MBIT(5)

t_u8 wlan_check_11ax_twt_supported(mlan_private *pmpriv,
				   BSSDescriptor_t *pbss_desc);
t_u16 wlan_fill_he_cap_tlv(mlan_private *pmpriv, t_u8 band,
			   MrvlIEtypes_Extension_t * phe_cap, t_u8 flag);
void wlan_update_11ax_cap(mlan_adapter *pmadapter,
			  MrvlIEtypes_Extension_t * hw_he_cap);
int wlan_cmd_append_11ax_tlv(mlan_private *pmpriv, BSSDescriptor_t *pbss_desc,
			     t_u8 **ppbuffer);
t_u16 wlan_11ax_bandconfig_allowed(mlan_private *pmpriv, t_u16 bss_band);
mlan_status wlan_11ax_cfg_ioctl(pmlan_adapter pmadapter,
				pmlan_ioctl_req pioctl_req);
mlan_status wlan_11ax_ioctl_cmd(pmlan_adapter pmadapter,
				pmlan_ioctl_req pioctl_req);

mlan_status wlan_cmd_11ax_cfg(IN pmlan_private pmpriv,
			      IN HostCmd_DS_COMMAND *cmd,
			      IN t_u16 cmd_action, IN t_void *pdata_buf);
mlan_status wlan_ret_11ax_cfg(IN pmlan_private pmpriv,
			      IN HostCmd_DS_COMMAND *resp,
			      IN mlan_ioctl_req *pioctl_buf);
mlan_status wlan_cmd_11ax_cmd(IN pmlan_private pmpriv,
			      IN HostCmd_DS_COMMAND *cmd,
			      IN t_u16 cmd_action, IN t_void *pdata_buf);
mlan_status wlan_ret_11ax_cmd(IN pmlan_private pmpriv,
			      IN HostCmd_DS_COMMAND *resp,
			      IN mlan_ioctl_req *pioctl_buf);

#endif /* _MLAN_11AX_H_ */
