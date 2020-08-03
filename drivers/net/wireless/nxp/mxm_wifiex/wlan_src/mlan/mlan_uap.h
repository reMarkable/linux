/** @file mlan_uap.h
 *
 *  @brief This file contains related macros, enum, and struct
 *  of uap functionalities
 *
 *
 *  Copyright 2014-2020 NXP
 *
 *  This software file (the File) is distributed by NXP
 *  under the terms of the GNU General Public License Version 2, June 1991
 *  (the License).  You may use, redistribute and/or modify the File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 *
 */

/********************************************************
Change log:
    02/05/2009: initial version
********************************************************/

#ifndef _MLAN_UAP_H_
#define _MLAN_UAP_H_

mlan_status wlan_uap_get_channel(pmlan_private pmpriv);

mlan_status wlan_uap_set_channel(pmlan_private pmpriv,
				 Band_Config_t uap_band_cfg, t_u8 channel);

mlan_status wlan_uap_get_beacon_dtim(pmlan_private pmpriv);

mlan_status wlan_ops_uap_ioctl(t_void *adapter, pmlan_ioctl_req pioctl_req);

mlan_status wlan_ops_uap_prepare_cmd(t_void *priv, t_u16 cmd_no,
				     t_u16 cmd_action, t_u32 cmd_oid,
				     t_void *pioctl_buf, t_void *pdata_buf,
				     t_void *pcmd_buf);

mlan_status wlan_ops_uap_process_cmdresp(t_void *priv, t_u16 cmdresp_no,
					 t_void *pcmd_buf, t_void *pioctl);

mlan_status wlan_ops_uap_process_rx_packet(t_void *adapter, pmlan_buffer pmbuf);

mlan_status wlan_ops_uap_process_event(t_void *priv);

t_void *wlan_ops_uap_process_txpd(t_void *priv, pmlan_buffer pmbuf);

mlan_status wlan_ops_uap_init_cmd(t_void *priv, t_u8 first_bss);

#endif /* _MLAN_UAP_H_ */
