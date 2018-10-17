/* Copyright 2013-2016 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _FSL_DPMAC_CMD_H
#define _FSL_DPMAC_CMD_H

/* DPMAC Version */
#define DPMAC_VER_MAJOR				4
#define DPMAC_VER_MINOR				2
#define DPMAC_CMD_BASE_VERSION			1
#define DPMAC_CMD_2ND_VERSION			2
#define DPMAC_CMD_ID_OFFSET			4

#define DPMAC_CMD(id)	(((id) << DPMAC_CMD_ID_OFFSET) | DPMAC_CMD_BASE_VERSION)
#define DPMAC_CMD_V2(id) (((id) << DPMAC_CMD_ID_OFFSET) | DPMAC_CMD_2ND_VERSION)

/* Command IDs */
#define DPMAC_CMDID_CLOSE		DPMAC_CMD(0x800)
#define DPMAC_CMDID_OPEN		DPMAC_CMD(0x80c)
#define DPMAC_CMDID_CREATE		DPMAC_CMD(0x90c)
#define DPMAC_CMDID_DESTROY		DPMAC_CMD(0x98c)
#define DPMAC_CMDID_GET_API_VERSION	DPMAC_CMD(0xa0c)

#define DPMAC_CMDID_GET_ATTR		DPMAC_CMD(0x004)
#define DPMAC_CMDID_RESET		DPMAC_CMD(0x005)

#define DPMAC_CMDID_SET_IRQ_ENABLE	DPMAC_CMD(0x012)
#define DPMAC_CMDID_GET_IRQ_ENABLE	DPMAC_CMD(0x013)
#define DPMAC_CMDID_SET_IRQ_MASK	DPMAC_CMD(0x014)
#define DPMAC_CMDID_GET_IRQ_MASK	DPMAC_CMD(0x015)
#define DPMAC_CMDID_GET_IRQ_STATUS	DPMAC_CMD(0x016)
#define DPMAC_CMDID_CLEAR_IRQ_STATUS	DPMAC_CMD(0x017)

#define DPMAC_CMDID_GET_LINK_CFG	DPMAC_CMD(0x0c2)
#define DPMAC_CMDID_GET_LINK_CFG_V2	DPMAC_CMD_V2(0x0c2)
#define DPMAC_CMDID_SET_LINK_STATE	DPMAC_CMD(0x0c3)
#define DPMAC_CMDID_SET_LINK_STATE_V2	DPMAC_CMD_V2(0x0c3)
#define DPMAC_CMDID_GET_COUNTER		DPMAC_CMD(0x0c4)

#define DPMAC_CMDID_SET_PORT_MAC_ADDR	DPMAC_CMD(0x0c5)

/* Macros for accessing command fields smaller than 1byte */
#define DPMAC_MASK(field)        \
	GENMASK(DPMAC_##field##_SHIFT + DPMAC_##field##_SIZE - 1, \
		DPMAC_##field##_SHIFT)
#define dpmac_set_field(var, field, val) \
	((var) |= (((val) << DPMAC_##field##_SHIFT) & DPMAC_MASK(field)))
#define dpmac_get_field(var, field)      \
	(((var) & DPMAC_MASK(field)) >> DPMAC_##field##_SHIFT)

struct dpmac_cmd_open {
	u32 dpmac_id;
};

struct dpmac_cmd_create {
	u32 mac_id;
};

struct dpmac_cmd_destroy {
	u32 dpmac_id;
};

struct dpmac_cmd_set_irq_enable {
	u8 enable;
	u8 pad[3];
	u8 irq_index;
};

struct dpmac_cmd_get_irq_enable {
	u32 pad;
	u8 irq_index;
};

struct dpmac_rsp_get_irq_enable {
	u8 enabled;
};

struct dpmac_cmd_set_irq_mask {
	u32 mask;
	u8 irq_index;
};

struct dpmac_cmd_get_irq_mask {
	u32 pad;
	u8 irq_index;
};

struct dpmac_rsp_get_irq_mask {
	u32 mask;
};

struct dpmac_cmd_get_irq_status {
	u32 status;
	u8 irq_index;
};

struct dpmac_rsp_get_irq_status {
	u32 status;
};

struct dpmac_cmd_clear_irq_status {
	u32 status;
	u8 irq_index;
};

struct dpmac_rsp_get_attributes {
	u8 eth_if;
	u8 link_type;
	u16 id;
	u32 max_rate;
};

struct dpmac_rsp_get_link_cfg {
	u64 options;
	u32 rate;
};

struct dpmac_rsp_get_link_cfg_v2 {
	u64 options;
	u32 rate;
	u32 pad;
	u64 advertising;
};

#define DPMAC_STATE_SIZE	1
#define DPMAC_STATE_SHIFT	0
#define DPMAC_STATE_VALID_SIZE	1
#define DPMAC_STATE_VALID_SHIFT	1

struct dpmac_cmd_set_link_state {
	u64 options;
	u32 rate;
	u32 pad;
	/* only least significant bit is valid */
	u8 up;
};

struct dpmac_cmd_set_link_state_v2 {
	u64 options;
	u32 rate;
	u32 pad0;
	/* from lsb: up:1, state_valid:1 */
	u8 state;
	u8 pad1[7];
	u64 supported;
	u64 advertising;
};

struct dpmac_cmd_get_counter {
	u8 type;
};

struct dpmac_rsp_get_counter {
	u64 pad;
	u64 counter;
};

struct dpmac_rsp_get_api_version {
	u16 major;
	u16 minor;
};

struct dpmac_cmd_set_port_mac_addr {
	u8 pad[2];
	u8 addr[6];
};

#endif /* _FSL_DPMAC_CMD_H */
