#ifndef __RTL_USB_H__
#define __RTL_USB_H__

#include <linux/usb.h>

enum rtl_txq {
	/* These definitions shall be consistent with value
	 * returned by skb_get_queue_mapping
	 *------------------------------------*/
	RTL_TXQ_BK,
	RTL_TXQ_BE,
	RTL_TXQ_VI,
	RTL_TXQ_VO,
	/*------------------------------------*/
	RTL_TXQ_BCN,
	RTL_TXQ_MGT,
	RTL_TXQ_HI,

	/* Must be last */
	__RTL_TXQ_NUM,
};

struct rtl_ep_map {
	u32 ep_mapping[__RTL_TXQ_NUM];
};

/*  Add suspend/resume later */
enum rtl_usb_state {
	USB_STATE_STOP	= 0,
	USB_STATE_START	= 1,
};

struct rtl_usb {
	struct usb_device *udev;
	struct usb_interface *intf;
	enum rtl_usb_state state;

	/* Bcn control register setting */
	u32 reg_bcn_ctrl_val;
	/* for 88/92cu card disable */
	u8	disableHWSM;
#if 0	/* ULLI : currently not defined */	
	/*QOS & EDCA */
	enum acm_method acm_method;
#endif	
	/* irq  . HIMR,HIMR_EX */
	u32 irq_mask[2];
	bool irq_enabled;

	u16 (*usb_mq_to_hwq)(__le16 fc, u16 mac80211_queue_index);

	/* Tx */
	u8 out_ep_nums ;
	u8 out_queue_sel;
	struct rtl_ep_map ep_map;

	u32 max_bulk_out_size;
	u32 tx_submitted_urbs;
#if 0	/* ULLI : currently not defined */	
	struct sk_buff_head tx_skb_queue[RTL_USB_MAX_EP_NUM];
	struct usb_anchor tx_pending[RTL_USB_MAX_EP_NUM];
	struct usb_anchor tx_submitted;

	struct sk_buff *(*usb_tx_aggregate_hdl)(struct ieee80211_hw *,
						struct sk_buff_head *);
	int (*usb_tx_post_hdl)(struct ieee80211_hw *,
			       struct urb *, struct sk_buff *);
	void (*usb_tx_cleanup)(struct ieee80211_hw *, struct sk_buff *);
#endif

	/* Rx */
	u8 in_ep_nums;
	u32 in_ep;		/* Bulk IN endpoint number */
	u32 rx_max_size;	/* Bulk IN max buffer size */
	u32 rx_urb_num;		/* How many Bulk INs are submitted to host. */
	struct usb_anchor	rx_submitted;
	struct usb_anchor	rx_cleanup_urbs;
	struct tasklet_struct   rx_work_tasklet;
	struct sk_buff_head	rx_queue;
#if 0	/* ULLI : currently not defined */	
	void (*usb_rx_segregate_hdl)(struct ieee80211_hw *, struct sk_buff *,
				     struct sk_buff_head *);
	void (*usb_rx_hdl)(struct ieee80211_hw *, struct sk_buff *);
#endif	
	/* ULLI : end of rtlwifi rtl_usb */
	
        struct rtl_priv *rtlpriv;

	//for local/global synchronization
	//
	spinlock_t	lock;
	int macid[NUM_STA];

	struct mutex hw_init_mutex;
	struct mutex h2c_fwcmd_mutex;
	struct mutex setch_mutex;
	struct mutex setbw_mutex;

	unsigned char	oper_channel; //saved channel info when call set_channel_bw

	//For 92D, DMDP have 2 interface.

	//In /Out Pipe information
	int	RtInPipe[2];

	uint8_t	irq_alloc;

/*-------- below is for SDIO INTERFACE --------*/

#ifdef INTF_DATA
	INTF_DATA intf_data;
#endif

/*-------- below is for USB INTERFACE --------*/


	uint8_t	usb_speed; // 1.1, 2.0 or 3.0
	uint8_t	nr_endpoint;
	uint8_t	RtNumInPipes;
	uint8_t	RtNumOutPipes;
	int	ep_num[6]; //endpoint number

	int	RegUsbSS;

#ifdef CONFIG_USB_VENDOR_REQ_MUTEX
	struct mutex usb_vendor_req_mutex;
#endif

#ifdef CONFIG_USB_VENDOR_REQ_BUFFER_PREALLOC
	uint8_t * usb_alloc_vendor_req_buf;
	uint8_t * usb_vendor_req_buf;
#endif


	atomic_t continual_urb_error;

/*-------- below is for PCIE INTERFACE --------*/

};

struct rtl_usb_priv {
	struct rtl_usb	dev;
	struct	rtl_led_ctl	ledpriv;
};

#define rtl_usbpriv(rtl)       (&(rtl->priv))

static inline struct device *dvobj_to_dev(struct rtl_usb *dvobj)
{
	/* todo: get interface type from dvobj and the return the dev accordingly */
	return &dvobj->intf->dev;
}

void usb_write_port_cancel(struct rtl_priv *rtlpriv);
void usb_read_port_cancel(struct rtl_priv *rtlprivl);

uint32_t usb_read_port(struct rtl_priv *rtlpriv, uint32_t cnt, uint8_t *rmem);

/* ULLI : _rtlw* prefix because of rtlwifi namespace */

u32  _rtlw_usb_transmit(struct rtl_priv *rtlpriv, u32 addr, u32 cnt, struct xmit_buf *pxmitbu);


/* ULLI : we use here rtw_usb_* because interface is not complete */
struct rtl_hal_cfg;
int rtw_usb_probe(struct usb_interface *pusb_intf, const struct usb_device_id *pdid,
		struct rtl_hal_cfg *rtl_hal_cfg);
void rtw_usb_disconnect(struct usb_interface *pusb_intf);
void usb_intf_stop(struct rtl_priv *rtlpriv);
void usb_intf_start(struct rtl_priv *rtlpriv);
#endif
