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
#ifndef __FSL_DPMAC_H
#define __FSL_DPMAC_H

/* Data Path MAC API
 * Contains initialization APIs and runtime control APIs for DPMAC
 */

struct fsl_mc_io;

int dpmac_open(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       int dpmac_id,
	       u16 *token);

int dpmac_close(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		u16 token);

/**
 * enum dpmac_link_type -  DPMAC link type
 * @DPMAC_LINK_TYPE_NONE: No link
 * @DPMAC_LINK_TYPE_FIXED: Link is fixed type
 * @DPMAC_LINK_TYPE_PHY: Link by PHY ID
 * @DPMAC_LINK_TYPE_BACKPLANE: Backplane link type
 */
enum dpmac_link_type {
	DPMAC_LINK_TYPE_NONE,
	DPMAC_LINK_TYPE_FIXED,
	DPMAC_LINK_TYPE_PHY,
	DPMAC_LINK_TYPE_BACKPLANE
};

/**
 * enum dpmac_eth_if - DPMAC Ethrnet interface
 * @DPMAC_ETH_IF_MII: MII interface
 * @DPMAC_ETH_IF_RMII: RMII interface
 * @DPMAC_ETH_IF_SMII: SMII interface
 * @DPMAC_ETH_IF_GMII: GMII interface
 * @DPMAC_ETH_IF_RGMII: RGMII interface
 * @DPMAC_ETH_IF_SGMII: SGMII interface
 * @DPMAC_ETH_IF_QSGMII: QSGMII interface
 * @DPMAC_ETH_IF_XAUI: XAUI interface
 * @DPMAC_ETH_IF_XFI: XFI interface
 */
enum dpmac_eth_if {
	DPMAC_ETH_IF_MII,
	DPMAC_ETH_IF_RMII,
	DPMAC_ETH_IF_SMII,
	DPMAC_ETH_IF_GMII,
	DPMAC_ETH_IF_RGMII,
	DPMAC_ETH_IF_SGMII,
	DPMAC_ETH_IF_QSGMII,
	DPMAC_ETH_IF_XAUI,
	DPMAC_ETH_IF_XFI
};

/**
 * struct dpmac_cfg - Structure representing DPMAC configuration
 * @mac_id:	Represents the Hardware MAC ID; in case of multiple WRIOP,
 *		the MAC IDs are continuous.
 *		For example:  2 WRIOPs, 16 MACs in each:
 *				MAC IDs for the 1st WRIOP: 1-16,
 *				MAC IDs for the 2nd WRIOP: 17-32.
 */
struct dpmac_cfg {
	u16 mac_id;
};

int dpmac_create(struct fsl_mc_io *mc_io,
		 u16 dprc_token,
		 u32 cmd_flags,
		 const struct dpmac_cfg *cfg,
		 u32 *obj_id);

int dpmac_destroy(struct fsl_mc_io *mc_io,
		  u16 dprc_token,
		  u32 cmd_flags,
		  u32 object_id);

/**
 * DPMAC IRQ Index and Events
 */

/**
 * IRQ index
 */
#define DPMAC_IRQ_INDEX				0
/**
 * IRQ event - indicates a change in link state
 */
#define DPMAC_IRQ_EVENT_LINK_CFG_REQ		0x00000001
/**
 * IRQ event - Indicates that the link state changed
 */
#define DPMAC_IRQ_EVENT_LINK_CHANGED		0x00000002

int dpmac_set_irq_enable(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u8 irq_index,
			 u8 en);

int dpmac_get_irq_enable(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u8 irq_index,
			 u8 *en);

int dpmac_set_irq_mask(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u8 irq_index,
		       u32 mask);

int dpmac_get_irq_mask(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u8 irq_index,
		       u32 *mask);

int dpmac_get_irq_status(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u8 irq_index,
			 u32 *status);

int dpmac_clear_irq_status(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u8 irq_index,
			   u32 status);

/**
 * struct dpmac_attr - Structure representing DPMAC attributes
 * @id:		DPMAC object ID
 * @max_rate:	Maximum supported rate - in Mbps
 * @eth_if:	Ethernet interface
 * @link_type:	link type
 */
struct dpmac_attr {
	u16 id;
	u32 max_rate;
	enum dpmac_eth_if eth_if;
	enum dpmac_link_type link_type;
};

int dpmac_get_attributes(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 struct dpmac_attr *attr);

/**
 * DPMAC link configuration/state options
 */

/**
 * Enable auto-negotiation
 */
#define DPMAC_LINK_OPT_AUTONEG		0x0000000000000001ULL
/**
 * Enable half-duplex mode
 */
#define DPMAC_LINK_OPT_HALF_DUPLEX	0x0000000000000002ULL
/**
 * Enable pause frames
 */
#define DPMAC_LINK_OPT_PAUSE		0x0000000000000004ULL
/**
 * Enable a-symmetric pause frames
 */
#define DPMAC_LINK_OPT_ASYM_PAUSE	0x0000000000000008ULL

/**
 * Advertised link speeds
 */
#define DPMAC_ADVERTISED_10BASET_FULL		0x0000000000000001ULL
#define DPMAC_ADVERTISED_100BASET_FULL		0x0000000000000002ULL
#define DPMAC_ADVERTISED_1000BASET_FULL		0x0000000000000004ULL
#define DPMAC_ADVERTISED_10000BASET_FULL	0x0000000000000010ULL
#define DPMAC_ADVERTISED_2500BASEX_FULL		0x0000000000000020ULL

/**
 * Advertise auto-negotiation enable
 */
#define DPMAC_ADVERTISED_AUTONEG		0x0000000000000008ULL

/**
 * struct dpmac_link_cfg - Structure representing DPMAC link configuration
 * @rate: Link's rate - in Mbps
 * @options: Enable/Disable DPMAC link cfg features (bitmap)
 * @advertising: Speeds that are advertised for autoneg (bitmap)
 */
struct dpmac_link_cfg {
	u32 rate;
	u64 options;
	u64 advertising;
};

int dpmac_get_link_cfg(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       struct dpmac_link_cfg *cfg);

int dpmac_get_link_cfg_v2(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  struct dpmac_link_cfg *cfg);

/**
 * struct dpmac_link_state - DPMAC link configuration request
 * @rate: Rate in Mbps
 * @options: Enable/Disable DPMAC link cfg features (bitmap)
 * @up: Link state
 * @state_valid: Ignore/Update the state of the link
 * @supported: Speeds capability of the phy (bitmap)
 * @advertising: Speeds that are advertised for autoneg (bitmap)
 */
struct dpmac_link_state {
	u32 rate;
	u64 options;
	int up;
	int state_valid;
	u64 supported;
	u64 advertising;
};

int dpmac_set_link_state(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 struct dpmac_link_state *link_state);

int dpmac_set_link_state_v2(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    struct dpmac_link_state *link_state);

/**
 * enum dpmac_counter - DPMAC counter types
 * @DPMAC_CNT_ING_FRAME_64: counts 64-bytes frames, good or bad.
 * @DPMAC_CNT_ING_FRAME_127: counts 65- to 127-bytes frames, good or bad.
 * @DPMAC_CNT_ING_FRAME_255: counts 128- to 255-bytes frames, good or bad.
 * @DPMAC_CNT_ING_FRAME_511: counts 256- to 511-bytes frames, good or bad.
 * @DPMAC_CNT_ING_FRAME_1023: counts 512- to 1023-bytes frames, good or bad.
 * @DPMAC_CNT_ING_FRAME_1518: counts 1024- to 1518-bytes frames, good or bad.
 * @DPMAC_CNT_ING_FRAME_1519_MAX: counts 1519-bytes frames and larger
 *				  (up to max frame length specified),
 *				  good or bad.
 * @DPMAC_CNT_ING_FRAG: counts frames which are shorter than 64 bytes received
 *			with a wrong CRC
 * @DPMAC_CNT_ING_JABBER: counts frames longer than the maximum frame length
 *			  specified, with a bad frame check sequence.
 * @DPMAC_CNT_ING_FRAME_DISCARD: counts dropped frames due to internal errors.
 *				 Occurs when a receive FIFO overflows.
 *				 Includes also frames truncated as a result of
 *				 the receive FIFO overflow.
 * @DPMAC_CNT_ING_ALIGN_ERR: counts frames with an alignment error
 *			     (optional used for wrong SFD).
 * @DPMAC_CNT_EGR_UNDERSIZED: counts frames transmitted that was less than 64
 *			      bytes long with a good CRC.
 * @DPMAC_CNT_ING_OVERSIZED: counts frames longer than the maximum frame length
 *			     specified, with a good frame check sequence.
 * @DPMAC_CNT_ING_VALID_PAUSE_FRAME: counts valid pause frames (regular and PFC)
 * @DPMAC_CNT_EGR_VALID_PAUSE_FRAME: counts valid pause frames transmitted
 *				     (regular and PFC).
 * @DPMAC_CNT_ING_BYTE: counts bytes received except preamble for all valid
 *			frames and valid pause frames.
 * @DPMAC_CNT_ING_MCAST_FRAME: counts received multicast frames.
 * @DPMAC_CNT_ING_BCAST_FRAME: counts received broadcast frames.
 * @DPMAC_CNT_ING_ALL_FRAME: counts each good or bad frames received.
 * @DPMAC_CNT_ING_UCAST_FRAME: counts received unicast frames.
 * @DPMAC_CNT_ING_ERR_FRAME: counts frames received with an error
 *			     (except for undersized/fragment frame).
 * @DPMAC_CNT_EGR_BYTE: counts bytes transmitted except preamble for all valid
 *			frames and valid pause frames transmitted.
 * @DPMAC_CNT_EGR_MCAST_FRAME: counts transmitted multicast frames.
 * @DPMAC_CNT_EGR_BCAST_FRAME: counts transmitted broadcast frames.
 * @DPMAC_CNT_EGR_UCAST_FRAME: counts transmitted unicast frames.
 * @DPMAC_CNT_EGR_ERR_FRAME: counts frames transmitted with an error.
 * @DPMAC_CNT_ING_GOOD_FRAME: counts frames received without error, including
 *			      pause frames.
 * @DPMAC_CNT_ENG_GOOD_FRAME: counts frames transmitted without error, including
 *			      pause frames.
 */
enum dpmac_counter {
	DPMAC_CNT_ING_FRAME_64,
	DPMAC_CNT_ING_FRAME_127,
	DPMAC_CNT_ING_FRAME_255,
	DPMAC_CNT_ING_FRAME_511,
	DPMAC_CNT_ING_FRAME_1023,
	DPMAC_CNT_ING_FRAME_1518,
	DPMAC_CNT_ING_FRAME_1519_MAX,
	DPMAC_CNT_ING_FRAG,
	DPMAC_CNT_ING_JABBER,
	DPMAC_CNT_ING_FRAME_DISCARD,
	DPMAC_CNT_ING_ALIGN_ERR,
	DPMAC_CNT_EGR_UNDERSIZED,
	DPMAC_CNT_ING_OVERSIZED,
	DPMAC_CNT_ING_VALID_PAUSE_FRAME,
	DPMAC_CNT_EGR_VALID_PAUSE_FRAME,
	DPMAC_CNT_ING_BYTE,
	DPMAC_CNT_ING_MCAST_FRAME,
	DPMAC_CNT_ING_BCAST_FRAME,
	DPMAC_CNT_ING_ALL_FRAME,
	DPMAC_CNT_ING_UCAST_FRAME,
	DPMAC_CNT_ING_ERR_FRAME,
	DPMAC_CNT_EGR_BYTE,
	DPMAC_CNT_EGR_MCAST_FRAME,
	DPMAC_CNT_EGR_BCAST_FRAME,
	DPMAC_CNT_EGR_UCAST_FRAME,
	DPMAC_CNT_EGR_ERR_FRAME,
	DPMAC_CNT_ING_GOOD_FRAME,
	DPMAC_CNT_ENG_GOOD_FRAME
};

int dpmac_get_counter(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      enum dpmac_counter type,
		      u64 *counter);

/**
 * dpmac_set_port_mac_addr() - Set a MAC address associated with the physical
 *              port.  This is not used for filtering, MAC is always in
 *              promiscuous mode, it is passed to DPNIs through DPNI API for
 *              application used.
 * @mc_io:	Pointer to opaque I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @addr:	MAC address to set
 *
 * Return:	The requested counter; '0' otherwise.
 */
int dpmac_set_port_mac_addr(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    const u8 addr[6]);

int dpmac_get_api_version(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 *major_ver,
			  u16 *minor_ver);

#endif /* __FSL_DPMAC_H */
