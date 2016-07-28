/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
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
#ifndef _RTW_RECV_H_
#define _RTW_RECV_H_


	#ifdef CONFIG_SINGLE_RECV_BUF
		#define NR_RECVBUFF (1)
	#else
			#define NR_RECVBUFF (4)
	#endif //CONFIG_SINGLE_RECV_BUF

	#define NR_PREALLOC_RECV_SKB (8)

#define NR_RECVFRAME 256

#define RXFRAME_ALIGN	8
#define RXFRAME_ALIGN_SZ	(1<<RXFRAME_ALIGN)

#define DRVINFO_SZ	4 // unit is 8bytes

#define MAX_RXFRAME_CNT	512
#define MAX_RX_NUMBLKS		(32)
#define RECVFRAME_HDR_ALIGN 128


#define PHY_RSSI_SLID_WIN_MAX				100
#define PHY_LINKQUALITY_SLID_WIN_MAX		20


#define SNAP_SIZE sizeof(struct ieee80211_snap_hdr)

#define RX_MPDU_QUEUE				0
#define RX_CMD_QUEUE				1
#define RX_MAX_QUEUE				2

static uint8_t SNAP_ETH_TYPE_IPX[2] = {0x81, 0x37};

static uint8_t SNAP_ETH_TYPE_APPLETALK_AARP[2] = {0x80, 0xf3};
static uint8_t SNAP_ETH_TYPE_APPLETALK_DDP[2] = {0x80, 0x9b};
static uint8_t SNAP_ETH_TYPE_TDLS[2] = {0x89, 0x0d};
static uint8_t SNAP_HDR_APPLETALK_DDP[3] = {0x08, 0x00, 0x07}; // Datagram Delivery Protocol

static uint8_t oui_8021h[] = {0x00, 0x00, 0xf8};
static uint8_t oui_rfc1042[]= {0x00,0x00,0x00};

#define MAX_SUBFRAME_COUNT	64
static uint8_t rtw_rfc1042_header[] =
{ 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };
/* Bridge-Tunnel header (for EtherTypes ETH_P_AARP and ETH_P_IPX) */
static uint8_t rtw_bridge_tunnel_header[] =
{ 0xaa, 0xaa, 0x03, 0x00, 0x00, 0xf8 };

//for Rx reordering buffer control
struct recv_reorder_ctrl
{
	struct rtl_priv	*rtlpriv;
	uint8_t enable;
	u16 indicate_seq;//=wstart_b, init_value=0xffff
	u16 wend_b;
	uint8_t wsize_b;
	struct __queue pending_recvframe_queue;
	struct timer_list reordering_ctrl_timer;
};

struct	stainfo_rxcache	{
	u16 	tid_rxseq[16];
/*
	unsigned short 	tid0_rxseq;
	unsigned short 	tid1_rxseq;
	unsigned short 	tid2_rxseq;
	unsigned short 	tid3_rxseq;
	unsigned short 	tid4_rxseq;
	unsigned short 	tid5_rxseq;
	unsigned short 	tid6_rxseq;
	unsigned short 	tid7_rxseq;
	unsigned short 	tid8_rxseq;
	unsigned short 	tid9_rxseq;
	unsigned short 	tid10_rxseq;
	unsigned short 	tid11_rxseq;
	unsigned short 	tid12_rxseq;
	unsigned short 	tid13_rxseq;
	unsigned short 	tid14_rxseq;
	unsigned short 	tid15_rxseq;
*/
};


struct smooth_rssi_data {
	u32	elements[100];	//array to store values
	u32	index;			//index to current array to store
	u32	total_num;		//num of valid elements
	u32	total_val;		//sum of valid elements
};

struct signal_stat {
	uint8_t	update_req;		//used to indicate
	uint8_t	avg_val;		//avg of valid elements
	u32	total_num;		//num of valid elements
	u32	total_val;		//sum of valid elements
};

struct phy_info
{
	uint8_t		RxPWDBAll;

	uint8_t		SignalQuality;	 // in 0-100 index.
	s8		RxMIMOSignalQuality[4];	//per-path's EVM
	uint8_t		RxMIMOEVMdbm[4]; 		//per-path's EVM dbm

	uint8_t		RxMIMOSignalStrength[4];// in 0~100 index

	u16		Cfo_short[4]; 			// per-path's Cfo_short
	u16		Cfo_tail[4];			// per-path's Cfo_tail

	s8		RxPower; // in dBm Translate from PWdB
	s8		RecvSignalPower;// Real power in dBm for this packet, no beautification and aggregation. Keep this raw info to be used for the other procedures.
	uint8_t		BTRxRSSIPercentage;
	uint8_t		SignalStrength; // in 0-100 index.

	uint8_t		RxPwr[4];				//per-path's pwdb
	uint8_t		RxSNR[4];				//per-path's SNR
	uint8_t		BandWidth;
	uint8_t		btCoexPwrAdjust;
};


struct rx_pkt_attrib	{
	u16	pkt_len;
	uint8_t	physt;
	uint8_t	drvinfo_sz;
	uint8_t	shift_sz;
	uint8_t	hdrlen; //the WLAN Header Len
	uint8_t 	to_fr_ds;
	uint8_t 	amsdu;
	uint8_t	qos;
	uint8_t	priority;
	uint8_t	pw_save;
	uint8_t	mdata;
	u16	seq_num;
	uint8_t	frag_num;
	uint8_t	mfrag;
	uint8_t	order;
	uint8_t	privacy; //in frame_ctrl field
	uint8_t	bdecrypted;
	uint8_t	encrypt; //when 0 indicate no encrypt. when non-zero, indicate the encrypt algorith
	uint8_t	iv_len;
	uint8_t	icv_len;
	uint8_t	crc_err;
	uint8_t	icv_err;

	u16	eth_type;

	uint8_t 	dst[ETH_ALEN];
	uint8_t 	src[ETH_ALEN];
	uint8_t 	ta[ETH_ALEN];
	uint8_t 	ra[ETH_ALEN];
	uint8_t 	bssid[ETH_ALEN];

	uint8_t	ack_policy;

	uint8_t 	key_index;

	uint8_t	data_rate;
	uint8_t 	sgi;
	uint8_t 	pkt_rpt_type;
	u32	MacIDValidEntry[2];	// 64 bits present 64 entry.

/*
	uint8_t	signal_qual;
	s8	rx_mimo_signal_qual[2];
	uint8_t	signal_strength;
	u32	RxPWDBAll;
	int32_t	RecvSignalPower;
*/
	struct phy_info phy_info;
};


//These definition is used for Rx packet reordering.
#define SN_LESS(a, b)		(((a-b)&0x800)!=0)
#define SN_EQUAL(a, b)	(a == b)
//#define REORDER_WIN_SIZE	128
//#define REORDER_ENTRY_NUM	128
#define REORDER_WAIT_TIME	(50) // (ms)

#define RECVBUFF_ALIGN_SZ 8

#define RXDESC_SIZE	24
#define RXDESC_OFFSET RXDESC_SIZE

struct recv_stat
{
	unsigned int rxdw0;

	unsigned int rxdw1;

	unsigned int rxdw2;

	unsigned int rxdw3;

	unsigned int rxdw4;

	unsigned int rxdw5;

};

#define EOR BIT(30)


/*
accesser of recv_priv: rtw_recv_entry(dispatch / passive level); recv_thread(passive) ; returnpkt(dispatch)
; halt(passive) ;

using enter_critical section to protect
*/
struct recv_priv
{
	spinlock_t	lock;

	//struct __queue	blk_strms[MAX_RX_NUMBLKS];    // keeping the block ack frame until return ack
	struct __queue	free_recv_queue;
	struct __queue	recv_pending_queue;
	struct __queue	uc_swdec_pending_queue;


	uint8_t *pallocated_frame_buf;
	uint8_t *precv_frame_buf;

	uint free_recvframe_cnt;

	struct rtl_priv	*rtlpriv;

	u64	rx_bytes;
	u64	rx_pkts;
	u64	rx_drop;
	u64	last_rx_bytes;

	uint  rx_icv_err;
	uint  rx_largepacket_crcerr;
	uint  rx_smallpacket_crcerr;
	uint  rx_middlepacket_crcerr;

	//uint8_t *pallocated_urb_buf;
	struct semaphore allrxreturnevt;
	uint8_t	rx_pending_cnt;


	struct tasklet_struct irq_prepare_beacon_tasklet;
	struct tasklet_struct recv_tasklet;
	struct sk_buff_head free_recv_skb_queue;
	struct sk_buff_head rx_skb_queue;
#ifdef CONFIG_RX_INDICATE_QUEUE
	struct task rx_indicate_tasklet;
	struct ifqueue rx_indicate_queue;
#endif	// CONFIG_RX_INDICATE_QUEUE

	uint8_t *pallocated_recv_buf;
	uint8_t *precv_buf;    // 4 alignment
	struct __queue	free_recv_buf_queue;
	u32	free_recv_buf_queue_cnt;



	//For display the phy informatiom
	uint8_t is_signal_dbg;	// for debug
	uint8_t signal_strength_dbg;	// for debug
	s8 rssi;
	s8 rxpwdb;
	uint8_t signal_strength;
	uint8_t signal_qual;
	uint8_t noise;
	int RxSNRdB[2];
	s8 RxRssi[2];
	int FalseAlmCnt_all;

#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
	struct timer_list signal_stat_timer;
	u32 signal_stat_sampling_interval;
	//u32 signal_stat_converging_constant;
	struct signal_stat signal_qual_data;
	struct signal_stat signal_strength_data;
#else //CONFIG_NEW_SIGNAL_STAT_PROCESS
	struct smooth_rssi_data signal_qual_data;
	struct smooth_rssi_data signal_strength_data;
#endif //CONFIG_NEW_SIGNAL_STAT_PROCESS

};

#ifdef CONFIG_NEW_SIGNAL_STAT_PROCESS
#define rtw_set_signal_stat_timer(recvpriv) _set_timer(&(recvpriv)->signal_stat_timer, (recvpriv)->signal_stat_sampling_interval)
#endif //CONFIG_NEW_SIGNAL_STAT_PROCESS

struct sta_recv_priv {

	spinlock_t	lock;
	int	option;

	//struct __queue	blk_strms[MAX_RX_NUMBLKS];
	struct __queue defrag_q;	 //keeping the fragment frame until defrag

	struct	stainfo_rxcache rxcache;

	//uint	sta_rx_bytes;
	//uint	sta_rx_pkts;
	//uint	sta_rx_fail;

};


struct recv_buf
{
	struct list_head list;

	spinlock_t recvbuf_lock;

	u32	ref_cnt;

	struct rtl_priv *rtlpriv;
	u32	len;

	struct urb *purb;
	dma_addr_t dma_transfer_addr;	/* (in) dma addr for transfer_buffer */
	u32 alloc_sz;

	uint8_t  irp_pending;

	struct sk_buff *skb;
	uint8_t	reuse;
};


/*
	head  ----->

		data  ----->

			payload

		tail  ----->


	end   ----->

	len = (unsigned int )(tail - data);

*/
struct recv_frame {
	struct list_head	list;
	struct sk_buff *skb;
	struct rtl_priv  *rtlpriv;

	uint8_t fragcnt;

	int frame_tag;

	struct rx_pkt_attrib attrib;

	uint  len;
	uint8_t *rx_head;
	uint8_t *rx_data;
	uint8_t *rx_tail;
	uint8_t *rx_end;

	void *precvbuf;


	//
	struct sta_info *psta;

	//for A-MPDU Rx reordering buffer control
	struct recv_reorder_ctrl *preorder_ctrl;
};

typedef enum _RX_PACKET_TYPE{
	NORMAL_RX,//Normal rx packet
	TX_REPORT1,//CCX
	TX_REPORT2,//TX RPT
	HIS_REPORT,// USB HISR RPT
	C2H_PACKET
}RX_PACKET_TYPE, *PRX_PACKET_TYPE;

extern struct recv_frame *_rtw_alloc_recvframe (struct __queue *pfree_recv_queue);  //get a free recv_frame from pfree_recv_queue
extern struct recv_frame *rtw_alloc_recvframe (struct __queue *pfree_recv_queue);  //get a free recv_frame from pfree_recv_queue
extern void rtw_init_recvframe(struct recv_frame *precvframe ,struct recv_priv *precvpriv);
extern int	 rtw_free_recvframe(struct recv_frame *precvframe, struct __queue *pfree_recv_queue);

#define rtw_dequeue_recvframe(queue) rtw_alloc_recvframe(queue)
extern int _rtw_enqueue_recvframe(struct recv_frame *precvframe, struct __queue *queue);
extern int rtw_enqueue_recvframe(struct recv_frame *precvframe, struct __queue *queue);

extern void rtw_free_recvframe_queue(struct __queue *pframequeue,  struct __queue *pfree_recv_queue);
u32 rtw_free_uc_swdec_pending_queue(struct rtl_priv *rtlpriv);

int rtw_enqueue_recvbuf_to_head(struct recv_buf *precvbuf, struct __queue *queue);
int rtw_enqueue_recvbuf(struct recv_buf *precvbuf, struct __queue *queue);
struct recv_buf *rtw_dequeue_recvbuf (struct __queue *queue);

void rtw_reordering_ctrl_timeout_handler(void *pcontext);

__inline static uint8_t *get_rxmem(struct recv_frame *precvframe)
{
	//always return rx_head...
	if(precvframe==NULL)
		return NULL;

	return precvframe->rx_head;
}

__inline static uint8_t *get_rx_status(struct recv_frame *precvframe)
{

	return get_rxmem(precvframe);

}

__inline static uint8_t *get_recvframe_data(struct recv_frame *precvframe)
{

	//alwasy return rx_data
	if(precvframe==NULL)
		return NULL;

	return precvframe->rx_data;

}

__inline static uint8_t *recvframe_push(struct recv_frame *precvframe, int sz)
{
	// append data before rx_data

	/* add data to the start of recv_frame
 *
 *      This function extends the used data area of the recv_frame at the buffer
 *      start. rx_data must be still larger than rx_head, after pushing.
 */

	if(precvframe==NULL)
		return NULL;


	precvframe->rx_data -= sz ;
	if( precvframe->rx_data < precvframe->rx_head )
	{
		precvframe->rx_data += sz ;
		return NULL;
	}

	precvframe->len +=sz;

	return precvframe->rx_data;

}


__inline static uint8_t *recvframe_pull(struct recv_frame *precvframe, int sz)
{
	// rx_data += sz; move rx_data sz bytes  hereafter

	//used for extract sz bytes from rx_data, update rx_data and return the updated rx_data to the caller


	if(precvframe==NULL)
		return NULL;


	precvframe->rx_data += sz;

	if(precvframe->rx_data > precvframe->rx_tail)
	{
		precvframe->rx_data -= sz;
		return NULL;
	}

	precvframe->len -=sz;

	return precvframe->rx_data;

}

__inline static uint8_t *recvframe_put(struct recv_frame *precvframe, int sz)
{
	// rx_tai += sz; move rx_tail sz bytes  hereafter

	//used for append sz bytes from ptr to rx_tail, update rx_tail and return the updated rx_tail to the caller
	//after putting, rx_tail must be still larger than rx_end.
 	unsigned char * prev_rx_tail;

	if(precvframe==NULL)
		return NULL;

	prev_rx_tail = precvframe->rx_tail;

	precvframe->rx_tail += sz;

	if(precvframe->rx_tail > precvframe->rx_end)
	{
		precvframe->rx_tail -= sz;
		return NULL;
	}

	precvframe->len +=sz;

	return precvframe->rx_tail;

}



__inline static uint8_t *recvframe_pull_tail(struct recv_frame *precvframe, int sz)
{
	// rmv data from rx_tail (by yitsen)

	//used for extract sz bytes from rx_end, update rx_end and return the updated rx_end to the caller
	//after pulling, rx_end must be still larger than rx_data.

	if(precvframe==NULL)
		return NULL;

	precvframe->rx_tail -= sz;

	if(precvframe->rx_tail < precvframe->rx_data)
	{
		precvframe->rx_tail += sz;
		return NULL;
	}

	precvframe->len -=sz;

	return precvframe->rx_tail;

}

__inline static struct recv_frame *rxmem_to_recvframe(uint8_t *rxmem)
{
	//due to the design of 2048 bytes alignment of recv_frame, we can reference the struct recv_frame
	//from any given member of recv_frame.
	// rxmem indicates the any member/address in recv_frame

	return (struct recv_frame*)(((SIZE_PTR)rxmem >> RXFRAME_ALIGN) << RXFRAME_ALIGN);

}

__inline static struct recv_frame *pkt_to_recvframe(struct sk_buff *pkt)
{

	uint8_t * buf_star;
	struct recv_frame * precv_frame;
	precv_frame = rxmem_to_recvframe((unsigned char*)buf_star);

	return precv_frame;
}

__inline static uint8_t *pkt_to_recvmem(struct sk_buff *pkt)
{
	// return the rx_head

	struct recv_frame * precv_frame = pkt_to_recvframe(pkt);

	return 	precv_frame->rx_head;

}

__inline static uint8_t *pkt_to_recvdata(struct sk_buff *pkt)
{
	// return the rx_data

	struct recv_frame * precv_frame =pkt_to_recvframe(pkt);

	return 	precv_frame->rx_data;

}


__inline static int get_recvframe_len(struct recv_frame *precvframe)
{
	return precvframe->len;
}


__inline static int32_t translate_percentage_to_dbm(u32 SignalStrengthIndex)
{
	int32_t	SignalPower; // in dBm.

	// Translate to dBm (x=0.5y-95).
	SignalPower = (int32_t)((SignalStrengthIndex + 1) >> 1);
	SignalPower -= 95;

	return SignalPower;
}


struct sta_info;

extern void _rtw_init_sta_recv_priv(struct sta_recv_priv *psta_recvpriv);

extern void  mgt_dispatcher(struct rtl_priv *rtlpriv, struct recv_frame *precv_frame);

#endif

