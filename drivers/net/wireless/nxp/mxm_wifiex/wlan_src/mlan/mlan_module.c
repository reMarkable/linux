/** @file mlan_module.c
 *
 *  @brief This file declares the exported symbols from MLAN.
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

/******************************************************
Change log:
    12/08/2008: initial version
******************************************************/

#ifdef LINUX
#include <linux/module.h>
#include "mlan_decl.h"
#include "mlan_ioctl.h"

EXPORT_SYMBOL(mlan_register);
EXPORT_SYMBOL(mlan_unregister);
EXPORT_SYMBOL(mlan_init_fw);
EXPORT_SYMBOL(mlan_set_init_param);
EXPORT_SYMBOL(mlan_dnld_fw);
EXPORT_SYMBOL(mlan_shutdown_fw);
#ifdef USB
EXPORT_SYMBOL(mlan_write_data_async_complete);
EXPORT_SYMBOL(mlan_recv);
#endif
EXPORT_SYMBOL(mlan_send_packet);
EXPORT_SYMBOL(mlan_ioctl);
EXPORT_SYMBOL(mlan_main_process);
EXPORT_SYMBOL(mlan_rx_process);
EXPORT_SYMBOL(mlan_select_wmm_queue);
#if defined(SDIO) || defined(PCIE)
EXPORT_SYMBOL(mlan_interrupt);
#if defined(SYSKT)
EXPORT_SYMBOL(mlan_hs_callback);
#endif /* SYSKT_MULTI || SYSKT */
#endif /* SDIO || PCIE */

EXPORT_SYMBOL(mlan_pm_wakeup_card);
EXPORT_SYMBOL(mlan_is_main_process_running);
#ifdef PCIE
EXPORT_SYMBOL(mlan_set_int_mode);
#endif

MODULE_DESCRIPTION("M-WLAN MLAN Driver");
MODULE_AUTHOR("NXP");
MODULE_VERSION(MLAN_RELEASE_VERSION);
MODULE_LICENSE("NXP Proprietary");
#endif /* LINUX */
