/** @file moal_sdio.h
 *
 * @brief This file contains definitions for SDIO interface.
 * driver.
 *
 *
 * Copyright 2014-2020 NXP
 *
 * This software file (the File) is distributed by NXP
 * under the terms of the GNU General Public License Version 2, June 1991
 * (the License).  You may use, redistribute and/or modify the File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 *
 */
/****************************************************
Change log:
****************************************************/

#ifndef _MOAL_SDIO_H
#define _MOAL_SDIO_H

#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

#include "moal_main.h"

#ifndef BLOCK_MODE
/** Block mode */
#define BLOCK_MODE 1
#endif

#ifndef BYTE_MODE
/** Byte Mode */
#define BYTE_MODE 0
#endif

#ifndef FIXED_ADDRESS
/** Fixed address mode */
#define FIXED_ADDRESS 0
#endif

#if defined(SD8977)
#define SD8977_V0 0x0
#define SD8977_V1 0x8
#define SD8977_V2 0x9
#define SD8977_V0_FW_NAME "nxp/sdsd8977_combo.bin"
#define SD8977_V1_FW_NAME "nxp/sdsd8977_combo_v1.bin"
#define SD8977_V2_FW_NAME "nxp/sdsd8977_combo_v2.bin"
#define SD8977_WLAN_V2_FW_NAME "nxp/sd8977_wlan_v2.bin"
#define SD8977_WLAN_V1_FW_NAME "nxp/sd8977_wlan_v1.bin"
#define SD8977_WLAN_V0_FW_NAME "nxp/sd8977_wlan.bin"
#endif /* SD8977_MULTI_FW */

#if defined(SD8887)
/** SD8887 chip revision ID */
#define SD8887_A0 0x0
#define SD8887_A2 0x2

#define SD8887_A0_FW_NAME "nxp/sd8887_uapsta.bin"
#define SD8887_A2_FW_NAME "nxp/sd8887_uapsta_a2.bin"
#define SD8887_WLAN_A2_FW_NAME "nxp/sd8887_wlan_a2.bin"
#define SD8887_WLAN_A0_FW_NAME "nxp/sd8887_wlan.bin"
#endif /* SD8887_MULTI_FW */

/** Default firmware name */
#ifdef SD8887
#define SD8887_DEFAULT_COMBO_FW_NAME "nxp/sd8887_uapsta_a2.bin"
#define SD8887_DEFAULT_WLAN_FW_NAME "nxp/sd8887_wlan_a2.bin"
#endif /* SD8887 */

#ifdef SD8977
#define SD8977_DEFAULT_COMBO_FW_NAME "nxp/sdsd8977_combo_v2.bin"
#define SD8977_DEFAULT_WLAN_FW_NAME "nxp/sd8977_wlan_v2.bin"
#endif /* SD8977 */

#ifdef SD8997
#define SD8997_DEFAULT_COMBO_FW_NAME "nxp/sdsd8997_combo_v4.bin"
#define SDUART8997_DEFAULT_COMBO_FW_NAME "nxp/sduart8997_combo_v4.bin"
#define SDSD8997_DEFAULT_COMBO_FW_NAME "nxp/sdsd8997_combo_v4.bin"
#define SD8997_DEFAULT_WLAN_FW_NAME "nxp/sd8997_wlan_v4.bin"
#endif /* SD8997 */

#ifdef SD8987
#define SD8987_DEFAULT_COMBO_FW_NAME "nxp/sdsd8987_combo.bin"
#define SDUART8987_DEFAULT_COMBO_FW_NAME "nxp/sduart8987_combo.bin"
#define SDSD8987_DEFAULT_COMBO_FW_NAME "nxp/sdsd8987_combo.bin"
#define SD8987_DEFAULT_WLAN_FW_NAME "nxp/sd8987_wlan.bin"
#endif /* SD8987 */

#ifdef SD8897
#define SD8897_DEFAULT_COMBO_FW_NAME "nxp/sdsd8897_uapsta.bin"
#define SD8897_DEFAULT_WLAN_FW_NAME "nxp/sd8897_wlan.bin"
#endif /* SD8897 */

#ifdef SD8978
#define SD8978_DEFAULT_COMBO_FW_NAME "nxp/sdsd8978_combo.bin"
#define SDUART8978_DEFAULT_COMBO_FW_NAME "nxp/sduart8978_combo.bin"
#define SDSD8978_DEFAULT_COMBO_FW_NAME "nxp/sdsd8978_combo.bin"
#define SD8978_DEFAULT_WLAN_FW_NAME "nxp/sd8978_wlan.bin"
#endif /* SD8978 */

#ifdef SD9098
#define SD9098_Z1Z2 0x00
#define SD9098_A0 0x01
#define SD9098_A1 0x02
#define SD9098_A2 0x03
#define SD9098_DEFAULT_COMBO_FW_NAME "nxp/sdsd9098_combo.bin"
#define SDUART9098_DEFAULT_COMBO_FW_NAME "nxp/sduart9098_combo.bin"
#define SDSD9098_DEFAULT_COMBO_FW_NAME "nxp/sdsd9098_combo.bin"
#define SD9098_DEFAULT_WLAN_FW_NAME "nxp/sd9098_wlan.bin"
#define SDUART9098_COMBO_V1_FW_NAME "nxp/sduart9098_combo_v1.bin"
#define SDSD9098_COMBO_V1_FW_NAME "nxp/sdsd9098_combo_v1.bin"
#define SD9098_WLAN_V1_FW_NAME "nxp/sd9098_wlan_v1.bin"
#endif /* SD9098 */

#ifdef SD9097
#define SD9097_B0 0x01
#define SD9097_B1 0x02
#define SD9097_DEFAULT_COMBO_FW_NAME "nxp/sdsd9097_combo_v1.bin"

#define SD9097_DEFAULT_WLAN_FW_NAME "nxp/sd9097_wlan_v1.bin"
#define SDUART9097_COMBO_V1_FW_NAME "nxp/sduart9097_combo_v1.bin"
#define SDSD9097_COMBO_V1_FW_NAME "nxp/sdsd9097_combo_v1.bin"
#define SD9097_WLAN_V1_FW_NAME "nxp/sd9097_wlan_v1.bin"
#endif /* SD9097 */

/********************************************************
		Global Functions
********************************************************/

/** Register to bus driver function */
mlan_status woal_sdiommc_bus_register(void);
/** Unregister from bus driver function */
void woal_sdiommc_bus_unregister(void);

int woal_sdio_set_bus_clock(moal_handle *handle, t_u8 option);

#ifdef SDIO_SUSPEND_RESUME
#ifdef MMC_PM_FUNC_SUSPENDED
/** Notify SDIO bus driver that WLAN is suspended */
void woal_wlan_is_suspended(moal_handle *handle);
#endif
/** SDIO Suspend */
int woal_sdio_suspend(struct device *dev);
/** SDIO Resume */
int woal_sdio_resume(struct device *dev);
#endif /* SDIO_SUSPEND_RESUME */

#ifdef SDIO_MMC
/** Structure: SDIO MMC card */
struct sdio_mmc_card {
	/** sdio_func structure pointer */
	struct sdio_func *func;
	/** moal_handle structure pointer */
	moal_handle *handle;
	/** saved host clock value */
	unsigned int host_clock;
};
#endif /* SDIO_MMC */

/** cmd52 read write */
int woal_sdio_read_write_cmd52(moal_handle *handle, int func, int reg, int val);
#endif /* _MOAL_SDIO_H */
