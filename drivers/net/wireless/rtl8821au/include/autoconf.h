//***** temporarily flag *******
//for FPGA VERIFICATION config

//***** temporarily flag *******
/*
 * Public  General Config
 */
#define DRV_NAME "rtl8821au"

//#define CONFIG_IOCTL_CFG80211 1

/*
 * Internal  General Config
 */
//#define CONFIG_PWRCTRL

#define CONFIG_RECV_REORDERING_CTRL	1

#define CONFIG_DFS	1
#define CONFIG_AP_MODE	1
#define CONFIG_SKB_COPY	1//for amsdu
#define CONFIG_NEW_SIGNAL_STAT_PROCESS
#define CONFIG_TX_MCAST2UNI	1	// Support IP multicast->unicast

/*
 * Interface  Related Config
 */

/*
 * CONFIG_USE_USB_BUFFER_ALLOC_XX uses Linux USB Buffer alloc API and is for Linux platform only now!
 */

#define CONFIG_USB_VENDOR_REQ_BUFFER_PREALLOC

#define CONFIG_USB_VENDOR_REQ_MUTEX
#define CONFIG_VENDOR_REQ_RETRY

#define RTL8812A_RX_PACKET_INCLUDE_CRC	0

#define ENABLE_USB_DROP_INCORRECT_OUT	0

/* ULLI symbol used in usb_halinit.c */
#define RTL8192CU_ADHOC_WORKAROUND_SETTING	1

#define ENABLE_NEW_RFE_TYPE	0

/*
 * Outsource  Related Config
 */

#define RATE_ADAPTIVE_SUPPORT 		0

#define	RTL8188E_EARLY_MODE_PKT_NUM_10	0

#define CONFIG_80211D

#define CONFIG_ATTEMPT_TO_FIX_AP_BEACON_ERROR

