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
#ifndef _RTW_XMIT_H_
#define _RTW_XMIT_H_

#define MAX_XMITBUF_SZ	(20480)	// 20k

#ifdef CONFIG_SINGLE_XMIT_BUF
#define NR_XMITBUFF	(1)
#else
#define NR_XMITBUFF	(4)
#endif //CONFIG_SINGLE_XMIT_BUF

#define XMITBUF_ALIGN_SZ 512

// xmit extension buff defination
#define MAX_XMIT_EXTBUF_SZ	(1536)
#define NR_XMIT_EXTBUFF	(32)

#define MAX_NUMBLKS		(1)

#define XMIT_VO_QUEUE (0)
#define XMIT_VI_QUEUE (1)
#define XMIT_BE_QUEUE (2)
#define XMIT_BK_QUEUE (3)

#define HW_QUEUE_ENTRY	8


#define WEP_IV(pattrib_iv, dot11txpn, keyidx)\
do{\
	pattrib_iv[0] = dot11txpn._byte_.TSC0;\
	pattrib_iv[1] = dot11txpn._byte_.TSC1;\
	pattrib_iv[2] = dot11txpn._byte_.TSC2;\
	pattrib_iv[3] = ((keyidx & 0x3)<<6);\
	dot11txpn.val = (dot11txpn.val == 0xffffff) ? 0: (dot11txpn.val+1);\
}while(0)


#define TKIP_IV(pattrib_iv, dot11txpn, keyidx)\
do{\
	pattrib_iv[0] = dot11txpn._byte_.TSC1;\
	pattrib_iv[1] = (dot11txpn._byte_.TSC1 | 0x20) & 0x7f;\
	pattrib_iv[2] = dot11txpn._byte_.TSC0;\
	pattrib_iv[3] = BIT(5) | ((keyidx & 0x3)<<6);\
	pattrib_iv[4] = dot11txpn._byte_.TSC2;\
	pattrib_iv[5] = dot11txpn._byte_.TSC3;\
	pattrib_iv[6] = dot11txpn._byte_.TSC4;\
	pattrib_iv[7] = dot11txpn._byte_.TSC5;\
	dot11txpn.val = dot11txpn.val == 0xffffffffffffULL ? 0: (dot11txpn.val+1);\
}while(0)

#define AES_IV(pattrib_iv, dot11txpn, keyidx)\
do{\
	pattrib_iv[0] = dot11txpn._byte_.TSC0;\
	pattrib_iv[1] = dot11txpn._byte_.TSC1;\
	pattrib_iv[2] = 0;\
	pattrib_iv[3] = BIT(5) | ((keyidx & 0x3)<<6);\
	pattrib_iv[4] = dot11txpn._byte_.TSC2;\
	pattrib_iv[5] = dot11txpn._byte_.TSC3;\
	pattrib_iv[6] = dot11txpn._byte_.TSC4;\
	pattrib_iv[7] = dot11txpn._byte_.TSC5;\
	dot11txpn.val = dot11txpn.val == 0xffffffffffffULL ? 0: (dot11txpn.val+1);\
}while(0)


#define HWXMIT_ENTRY	4

#define TXDESC_SIZE 40

#define PACKET_OFFSET_SZ (8)
#define TXDESC_OFFSET (TXDESC_SIZE + PACKET_OFFSET_SZ)


enum TXDESC_SC{
	SC_DONT_CARE = 0x00,
	SC_UPPER= 0x01,
	SC_LOWER=0x02,
	SC_DUPLICATE=0x03
};

#define TXDESC_40_BYTES

struct tx_desc
{
	unsigned int txdw0;
	unsigned int txdw1;
	unsigned int txdw2;
	unsigned int txdw3;
	unsigned int txdw4;
	unsigned int txdw5;
	unsigned int txdw6;
	unsigned int txdw7;

#if defined(TXDESC_40_BYTES) || defined(TXDESC_64_BYTES)
	unsigned int txdw8;
	unsigned int txdw9;
#endif // TXDESC_40_BYTES

#ifdef TXDESC_64_BYTES
	unsigned int txdw10;
	unsigned int txdw11;

	// 2008/05/15 MH Because PCIE HW memory R/W 4K limit. And now,  our descriptor
	// size is 40 bytes. If you use more than 102 descriptor( 103*40>4096), HW will execute
	// memoryR/W CRC error. And then all DMA fetch will fail. We must decrease descriptor
	// number or enlarge descriptor size as 64 bytes.
	unsigned int txdw12;
	unsigned int txdw13;
	unsigned int txdw14;
	unsigned int txdw15;
#endif
};


union txdesc {
	struct tx_desc txdesc;
	unsigned int value[TXDESC_SIZE>>2];
};


struct	hw_xmit	{
	//spinlock_t xmit_lock;
	//struct list_head	pending;
	struct __queue *sta_queue;
	//struct hw_txqueue *phwtxqueue;
	//int	txcmdcnt;
};

//reduce size
struct tx_pkt_attrib {
	uint8_t	type;
	uint8_t	subtype;
	uint8_t	bswenc;
	uint8_t	dhcp_pkt;
	u16	ether_type;
	u16	seqnum;
	u16	pkt_hdrlen;	//the original 802.3 pkt header len
	u16	hdrlen;		//the WLAN Header Len
	u32	pktlen;		//the original 802.3 pkt raw_data len (not include ether_hdr data)
	u32	last_txcmdsz;
	uint8_t	nr_frags;
	uint8_t	encrypt;	//when 0 indicate no encrypt. when non-zero, indicate the encrypt algorith
	uint8_t	iv_len;
	uint8_t	icv_len;
	uint8_t	iv[18];
	uint8_t	icv[16];
	uint8_t	tx_priority;
	uint8_t	ack_policy;
	uint8_t	mac_id;
	uint8_t	vcs_mode;	//virtual carrier sense method
	uint8_t 	dst[ETH_ALEN];
	uint8_t	src[ETH_ALEN];
	uint8_t	ta[ETH_ALEN];
	uint8_t 	ra[ETH_ALEN];
	uint8_t	key_idx;
	uint8_t	qos_en;
	uint8_t	ht_en;
	uint8_t	raid;//rate adpative id
	uint8_t	bwmode;
	uint8_t	ch_offset;//PRIME_CHNL_OFFSET
	uint8_t	sgi;//short GI
	uint8_t	ampdu_en;//tx ampdu enable
	uint8_t	mdata;//more data bit
	uint8_t	pctrl;//per packet txdesc control enable
	uint8_t	triggered;//for ap mode handling Power Saving sta
	uint8_t	tx_qsel;
	uint8_t	eosp;
	uint8_t	rate;
	uint8_t	intel_proxim;
	uint8_t 	retry_ctrl;
	uint8_t   mbssid;
	uint8_t	ldpc;
	uint8_t	stbc;
	struct sta_info * psta;
#ifdef CONFIG_TCP_CSUM_OFFLOAD_TX
	uint8_t	hw_tcp_csum;
#endif

	uint8_t rtsen;
	uint8_t cts2self;
	union Keytype	dot11tkiptxmickey;
	//union Keytype	dot11tkiprxmickey;
	union Keytype	dot118021x_UncstKey;


};


#define WLANHDR_OFFSET	64

#define NULL_FRAMETAG		(0x0)
#define DATA_FRAMETAG		0x01
#define L2_FRAMETAG		0x02
#define MGNT_FRAMETAG		0x03
#define AMSDU_FRAMETAG	0x04

#define EII_FRAMETAG		0x05
#define IEEE8023_FRAMETAG  0x06

#define MP_FRAMETAG		0x07

#define TXAGG_FRAMETAG 	0x08

enum {
	XMITBUF_DATA = 0,
	XMITBUF_MGNT = 1,
	XMITBUF_CMD = 2,
};

struct  submit_ctx{
	u32 submit_time; /* */
	u32 timeout_ms; /* <0: not synchronous, 0: wait forever, >0: up to ms waiting */
	int status; /* status for operation */

	struct completion done;
};

enum {
	RTW_SCTX_SUBMITTED = -1,
	RTW_SCTX_DONE_SUCCESS = 0,
	RTW_SCTX_DONE_UNKNOWN,
	RTW_SCTX_DONE_TIMEOUT,
	RTW_SCTX_DONE_BUF_ALLOC,
	RTW_SCTX_DONE_BUF_FREE,
	RTW_SCTX_DONE_WRITE_PORT_ERR,
	RTW_SCTX_DONE_TX_DESC_NA,
	RTW_SCTX_DONE_TX_DENY,
	RTW_SCTX_DONE_CCX_PKT_FAIL,
	RTW_SCTX_DONE_DRV_STOP,
	RTW_SCTX_DONE_DEV_REMOVE,
};


void rtw_sctx_init(struct submit_ctx *sctx, int timeout_ms);
int rtw_sctx_wait(struct submit_ctx *sctx);
void rtw_sctx_done_err(struct submit_ctx **sctx, int status);
void rtw_sctx_done(struct submit_ctx **sctx);

struct xmit_buf
{
	struct list_head	list;

	struct rtl_priv *rtlpriv;

	uint8_t *pallocated_buf;

	uint8_t *pbuf;

	void *priv_data;

	u16 buf_tag; // 0: Normal xmitbuf, 1: extension xmitbuf, 2:cmd xmitbuf
	u16 flags;
	u32 alloc_sz;

	u32  len;

	struct submit_ctx *sctx;


	//u32 sz[8];
	u32	ff_hwaddr;

	struct urb *pxmit_urb[8];
	dma_addr_t dma_transfer_addr;	/* (in) dma addr for transfer_buffer */

	uint8_t bpending[8];

	int last[8];




};


struct xmit_frame {
	struct list_head	list;

	struct tx_pkt_attrib tx_attrib;

	struct sk_buff *skb;

	int	frame_tag;

	struct rtl_priv *rtlpriv;

	uint8_t	*buf_addr;

	struct xmit_buf *pxmitbuf;


	uint8_t	agg_num;
	s8	pkt_offset;

	uint8_t *alloc_addr; /* the actual address this xmitframe allocated */
	uint8_t ext_tag; /* 0:data, 1:mgmt */

};

struct tx_servq {
	struct list_head	tx_pending;
	struct __queue	sta_pending;
};


struct sta_xmit_priv
{
	spinlock_t	lock;
	int	option;
	int	apsd_setting;	//When bit mask is on, the associated edca queue supports APSD.


	//struct tx_servq blk_q[MAX_NUMBLKS];
	struct tx_servq	be_q;			//priority == 0,3
	struct tx_servq	bk_q;			//priority == 1,2
	struct tx_servq	vi_q;			//priority == 4,5
	struct tx_servq	vo_q;			//priority == 6,7
	struct list_head 	legacy_dz;
	struct list_head  apsd;

	u16 txseq_tid[16];

	//uint	sta_tx_bytes;
	//u64	sta_tx_pkts;
	//uint	sta_tx_fail;


};


struct	hw_txqueue	{
	volatile int	head;
	volatile int	tail;
	volatile int 	free_sz;	//in units of 64 bytes
	volatile int      free_cmdsz;
	volatile int	 txsz[8];
	uint	ff_hwaddr;
	uint	cmd_hwaddr;
	int	ac_tag;
};

struct agg_pkt_info{
	u16 offset;
	u16 pkt_len;
};

struct	xmit_priv	{

	spinlock_t	lock;

	struct semaphore	xmit_sema;
	struct semaphore	terminate_xmitthread_sema;

	//struct __queue	blk_strms[MAX_NUMBLKS];
	struct __queue	be_pending;
	struct __queue	bk_pending;
	struct __queue	vi_pending;
	struct __queue	vo_pending;

	//struct __queue	legacy_dz_queue;
	//struct __queue	apsd_queue;

	uint8_t *pallocated_frame_buf;
	uint8_t *pxmit_frame_buf;
	uint free_xmitframe_cnt;
	struct __queue	free_xmit_queue;

	//uint mapping_addr;
	//uint pkt_sz;

	uint8_t *xframe_ext_alloc_addr;
	uint8_t *xframe_ext;
	uint free_xframe_ext_cnt;
	struct __queue free_xframe_ext_queue;

	//struct	hw_txqueue	be_txqueue;
	//struct	hw_txqueue	bk_txqueue;
	//struct	hw_txqueue	vi_txqueue;
	//struct	hw_txqueue	vo_txqueue;
	//struct	hw_txqueue	bmc_txqueue;

	uint	frag_len;

	struct rtl_priv	*rtlpriv;

	uint8_t   vcs_setting;
	uint8_t	vcs;
	uint8_t	vcs_type;
	//u16  rts_thresh;

	u64	tx_bytes;
	u64	tx_pkts;
	u64	tx_drop;
	u64	last_tx_bytes;
	u64	last_tx_pkts;

	struct hw_xmit hwxmits[HWXMIT_ENTRY];

	uint8_t	wmm_para_seq[4];//sequence for wmm ac parameter strength from large to small. it's value is 0->vo, 1->vi, 2->be, 3->bk.

	struct semaphore	tx_retevt;//all tx return event;
	uint8_t		txirp_cnt;//

	struct tasklet_struct xmit_tasklet;

	struct __queue free_xmitbuf_queue;
	struct __queue pending_xmitbuf_queue;
	uint8_t *pallocated_xmitbuf;
	uint8_t *pxmitbuf;
	uint free_xmitbuf_cnt;

	struct __queue free_xmit_extbuf_queue;
	uint8_t *pallocated_xmit_extbuf;
	uint8_t *pxmit_extbuf;
	uint free_xmit_extbuf_cnt;

	struct xmit_buf	pcmd_xmitbuf;

	u16	nqos_ssn;

	spinlock_t lock_sctx;
};

extern struct xmit_frame *rtw_alloc_cmdxmitframe(struct xmit_priv *pxmitpriv, u32 buffsize);
extern void	rtw_free_cmdxmitframe(struct xmit_priv *pxmitpriv, struct xmit_frame *pxmitframe);
extern struct xmit_buf *rtw_alloc_cmd_xmitbuf(struct xmit_priv *pxmitpriv, u32 buffsize);
extern int32_t	rtw_free_cmd_xmitbuf(struct xmit_priv *pxmitpriv);

extern struct xmit_buf *rtw_alloc_xmitbuf_ext(struct xmit_priv *pxmitpriv);
extern int32_t rtw_free_xmitbuf_ext(struct xmit_priv *pxmitpriv, struct xmit_buf *pxmitbuf);

extern struct xmit_buf *rtw_alloc_xmitbuf(struct xmit_priv *pxmitpriv);
extern int32_t rtw_free_xmitbuf(struct xmit_priv *pxmitpriv, struct xmit_buf *pxmitbuf);

void rtw_count_tx_stats(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe, int sz);
extern void rtw_update_protection(struct rtl_priv *rtlpriv, uint8_t *ie, uint ie_len);
extern int32_t rtw_make_wlanhdr(struct rtl_priv *rtlpriv, uint8_t *hdr, struct tx_pkt_attrib *pattrib);
extern int32_t rtw_put_snap(uint8_t *data, u16 h_proto);

extern struct xmit_frame *rtw_alloc_xmitframe(struct xmit_priv *pxmitpriv);
struct xmit_frame *rtw_alloc_xmitframe_ext(struct xmit_priv *pxmitpriv);
struct xmit_frame *rtw_alloc_xmitframe_once(struct xmit_priv *pxmitpriv);
extern int32_t rtw_free_xmitframe(struct xmit_priv *pxmitpriv, struct xmit_frame *pxmitframe);
extern void rtw_free_xmitframe_queue(struct xmit_priv *pxmitpriv, struct __queue *pframequeue);
extern int32_t rtw_xmitframe_enqueue(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe);
extern struct xmit_frame* rtw_dequeue_xframe(struct xmit_priv *pxmitpriv, struct hw_xmit *phwxmit_i);

extern int32_t rtw_xmit_classifier(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe);
extern u32 rtw_calculate_wlan_pkt_size_by_attribue(struct tx_pkt_attrib *pattrib);
#define rtw_wlan_pkt_size(f) rtw_calculate_wlan_pkt_size_by_attribue(&f->attrib)
extern int32_t rtw_xmitframe_coalesce(struct rtl_priv *rtlpriv, struct sk_buff *pkt, struct xmit_frame *pxmitframe);
int32_t _rtw_init_hw_txqueue(struct hw_txqueue* phw_txqueue, uint8_t ac_tag);
void _rtw_init_sta_xmit_priv(struct sta_xmit_priv *psta_xmitpriv);


int32_t rtw_txframes_pending(struct rtl_priv *rtlpriv);
bool rtw_txframes_sta_ac_pending(struct rtl_priv *rtlpriv, struct tx_pkt_attrib *pattrib);
void rtw_init_hwxmits(struct hw_xmit *phwxmit);


int32_t _rtw_init_xmit_priv(struct xmit_priv *pxmitpriv, struct rtl_priv *rtlpriv);
void _rtw_free_xmit_priv (struct xmit_priv *pxmitpriv);


void rtw_alloc_hwxmits(struct rtl_priv *rtlpriv);


int32_t rtw_xmit(struct rtl_priv *rtlpriv, struct sk_buff **pkt);

#if defined(CONFIG_AP_MODE)
int xmitframe_enqueue_for_sleeping_sta(struct rtl_priv *rtlpriv, struct xmit_frame *pxmitframe);
void stop_sta_xmit(struct rtl_priv *rtlpriv, struct sta_info *psta);
void wakeup_sta_to_xmit(struct rtl_priv *rtlpriv, struct sta_info *psta);
void xmit_delivery_enabled_frames(struct rtl_priv *rtlpriv, struct sta_info *psta);
#endif

uint8_t	qos_acm(uint8_t acm_mask, uint8_t priority);

//include after declaring struct xmit_buf, in order to avoid warning
#include <xmit_osdep.h>

#endif	//_RTL871X_XMIT_H_

