/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */
#ifndef __NWL_DSI_H__
#define __NWL_DSI_H__

#include <linux/irqreturn.h>

#include <drm/drm_mipi_dsi.h>

#include "nwl-drv.h"

/* DSI HOST registers */
#define NWL_DSI_CFG_NUM_LANES			0x0
#define NWL_DSI_CFG_NONCONTINUOUS_CLK		0x4
#define NWL_DSI_CFG_T_PRE			0x8
#define NWL_DSI_CFG_T_POST			0xc
#define NWL_DSI_CFG_TX_GAP			0x10
#define NWL_DSI_CFG_AUTOINSERT_EOTP		0x14
#define NWL_DSI_CFG_EXTRA_CMDS_AFTER_EOTP	0x18
#define NWL_DSI_CFG_HTX_TO_COUNT		0x1c
#define NWL_DSI_CFG_LRX_H_TO_COUNT		0x20
#define NWL_DSI_CFG_BTA_H_TO_COUNT		0x24
#define NWL_DSI_CFG_TWAKEUP			0x28
#define NWL_DSI_CFG_STATUS_OUT			0x2c
#define NWL_DSI_RX_ERROR_STATUS			0x30

/* DSI DPI registers */
#define NWL_DSI_PIXEL_PAYLOAD_SIZE		0x200
#define NWL_DSI_PIXEL_FIFO_SEND_LEVEL		0x204
#define NWL_DSI_INTERFACE_COLOR_CODING		0x208
#define NWL_DSI_PIXEL_FORMAT			0x20c
#define NWL_DSI_VSYNC_POLARITY			0x210
#define NWL_DSI_VSYNC_POLARITY_ACTIVE_LOW	0
#define NWL_DSI_VSYNC_POLARITY_ACTIVE_HIGH	BIT(1)

#define NWL_DSI_HSYNC_POLARITY			0x214
#define NWL_DSI_HSYNC_POLARITY_ACTIVE_LOW	0
#define NWL_DSI_HSYNC_POLARITY_ACTIVE_HIGH	BIT(1)

#define NWL_DSI_VIDEO_MODE			0x218
#define NWL_DSI_HFP				0x21c
#define NWL_DSI_HBP				0x220
#define NWL_DSI_HSA				0x224
#define NWL_DSI_ENABLE_MULT_PKTS		0x228
#define NWL_DSI_VBP				0x22c
#define NWL_DSI_VFP				0x230
#define NWL_DSI_BLLP_MODE			0x234
#define NWL_DSI_USE_NULL_PKT_BLLP		0x238
#define NWL_DSI_VACTIVE				0x23c
#define NWL_DSI_VC				0x240

/* DSI APB PKT control */
#define NWL_DSI_TX_PAYLOAD			0x280
#define NWL_DSI_PKT_CONTROL			0x284
#define NWL_DSI_SEND_PACKET			0x288
#define NWL_DSI_PKT_STATUS			0x28c
#define NWL_DSI_PKT_FIFO_WR_LEVEL		0x290
#define NWL_DSI_PKT_FIFO_RD_LEVEL		0x294
#define NWL_DSI_RX_PAYLOAD			0x298
#define NWL_DSI_RX_PKT_HEADER			0x29c

/* DSI IRQ handling */
#define NWL_DSI_IRQ_STATUS			0x2a0
#define NWL_DSI_SM_NOT_IDLE			BIT(0)
#define NWL_DSI_TX_PKT_DONE			BIT(1)
#define NWL_DSI_DPHY_DIRECTION			BIT(2)
#define NWL_DSI_TX_FIFO_OVFLW			BIT(3)
#define NWL_DSI_TX_FIFO_UDFLW			BIT(4)
#define NWL_DSI_RX_FIFO_OVFLW			BIT(5)
#define NWL_DSI_RX_FIFO_UDFLW			BIT(6)
#define NWL_DSI_RX_PKT_HDR_RCVD			BIT(7)
#define NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD	BIT(8)
#define NWL_DSI_BTA_TIMEOUT			BIT(29)
#define NWL_DSI_LP_RX_TIMEOUT			BIT(30)
#define NWL_DSI_HS_TX_TIMEOUT			BIT(31)

#define NWL_DSI_IRQ_STATUS2			0x2a4
#define NWL_DSI_SINGLE_BIT_ECC_ERR		BIT(0)
#define NWL_DSI_MULTI_BIT_ECC_ERR		BIT(1)
#define NWL_DSI_CRC_ERR				BIT(2)

#define NWL_DSI_IRQ_MASK			0x2a8
#define NWL_DSI_SM_NOT_IDLE_MASK		BIT(0)
#define NWL_DSI_TX_PKT_DONE_MASK		BIT(1)
#define NWL_DSI_DPHY_DIRECTION_MASK		BIT(2)
#define NWL_DSI_TX_FIFO_OVFLW_MASK		BIT(3)
#define NWL_DSI_TX_FIFO_UDFLW_MASK		BIT(4)
#define NWL_DSI_RX_FIFO_OVFLW_MASK		BIT(5)
#define NWL_DSI_RX_FIFO_UDFLW_MASK		BIT(6)
#define NWL_DSI_RX_PKT_HDR_RCVD_MASK		BIT(7)
#define NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD_MASK	BIT(8)
#define NWL_DSI_BTA_TIMEOUT_MASK		BIT(29)
#define NWL_DSI_LP_RX_TIMEOUT_MASK		BIT(30)
#define NWL_DSI_HS_TX_TIMEOUT_MASK		BIT(31)

#define NWL_DSI_IRQ_MASK2			0x2ac
#define NWL_DSI_SINGLE_BIT_ECC_ERR_MASK		BIT(0)
#define NWL_DSI_MULTI_BIT_ECC_ERR_MASK		BIT(1)
#define NWL_DSI_CRC_ERR_MASK			BIT(2)

extern const struct mipi_dsi_host_ops nwl_dsi_host_ops;

irqreturn_t nwl_dsi_irq_handler(int irq, void *data);
int nwl_dsi_enable(struct nwl_dsi *dsi);
int nwl_dsi_disable(struct nwl_dsi *dsi);

#endif /* __NWL_DSI_H__ */
