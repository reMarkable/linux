/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#ifndef __RTL8812A_XMIT_H__
#define __RTL8812A_XMIT_H__



//
//defined for TX DESC Operation
//

#define MAX_TID (15)

//OFFSET 0
#define OFFSET_SZ	0
#define OFFSET_SHT	16
#define BMC			BIT(24)
#define LSG			BIT(26)
#define FSG			BIT(27)
#define OWN 		BIT(31)


//OFFSET 4
#define PKT_OFFSET_SZ		0
#define QSEL_SHT			8
#define RATE_ID_SHT			16
#define NAVUSEHDR			BIT(20)
#define SEC_TYPE_SHT 		22
#define PKT_OFFSET_SHT		26

//OFFSET 8
#define AGG_EN				BIT(12)
#define AGG_BK				BIT(16)
#define AMPDU_DENSITY_SHT	20
#define ANTSEL_A			BIT(24)
#define ANTSEL_B			BIT(25)
#define TX_ANT_CCK_SHT		26
#define TX_ANTL_SHT			28
#define TX_ANT_HT_SHT		30

//OFFSET 12
#define SEQ_SHT				16
#define EN_HWSEQ			BIT(31)

//OFFSET 16
#define QOS					BIT(6)
#define	HW_SSN				BIT(7)
#define USERATE				BIT(8)
#define DISDATAFB			BIT(10)
#define CTS_2_SELF			BIT(11)
#define	RTS_EN				BIT(12)
#define	HW_RTS_EN			BIT(13)
#define DATA_SHORT			BIT(24)
#define PWR_STATUS_SHT	15
#define DATA_SC_SHT		20
#define DATA_BW				BIT(25)

//OFFSET 20
#define	RTY_LMT_EN			BIT(17)

//OFFSET 20
#define SGI					BIT(6)
#define USB_TXAGG_NUM_SHT	24


#define USB_DUMMY_OFFSET		1
#define USB_DUMMY_LENGTH		(USB_DUMMY_OFFSET * PACKET_OFFSET_SZ)

void rtl8812au_xmit_tasklet(void *priv);

u8	BWMapping_8812(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib);

#endif //__RTL8812_XMIT_H__

#include "rtl8821a_xmit.h"

