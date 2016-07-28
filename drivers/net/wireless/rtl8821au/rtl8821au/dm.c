#ifdef CONFIG_RTLWIFI

#include <../drivers/net/wireless/realtek/rtlwifi/wifi.h>
#include <../drivers/net/wireless/realtek/rtlwifi/base.h>

#else

#include <drv_types.h>
#include "dm.h"
#include "phy.h"
#include "reg.h"
#include "fw.h"

#endif

static const u32 txscalling_tbl[TXSCALE_TABLE_SIZE] = {
	0x081, /* 0,  -12.0dB */
	0x088, /* 1,  -11.5dB */
	0x090, /* 2,  -11.0dB */
	0x099, /* 3,  -10.5dB */
	0x0A2, /* 4,  -10.0dB */
	0x0AC, /* 5,  -9.5dB */
	0x0B6, /* 6,  -9.0dB */
	0x0C0, /* 7,  -8.5dB */
	0x0CC, /* 8,  -8.0dB */
	0x0D8, /* 9,  -7.5dB */
	0x0E5, /* 10, -7.0dB */
	0x0F2, /* 11, -6.5dB */
	0x101, /* 12, -6.0dB */
	0x110, /* 13, -5.5dB */
	0x120, /* 14, -5.0dB */
	0x131, /* 15, -4.5dB */
	0x143, /* 16, -4.0dB */
	0x156, /* 17, -3.5dB */
	0x16A, /* 18, -3.0dB */
	0x180, /* 19, -2.5dB */
	0x197, /* 20, -2.0dB */
	0x1AF, /* 21, -1.5dB */
	0x1C8, /* 22, -1.0dB */
	0x1E3, /* 23, -0.5dB */
	0x200, /* 24, +0  dB */
	0x21E, /* 25, +0.5dB */
	0x23E, /* 26, +1.0dB */
	0x261, /* 27, +1.5dB */
	0x285, /* 28, +2.0dB */
	0x2AB, /* 29, +2.5dB */
	0x2D3, /* 30, +3.0dB */
	0x2FE, /* 31, +3.5dB */
	0x32B, /* 32, +4.0dB */
	0x35C, /* 33, +4.5dB */
	0x38E, /* 34, +5.0dB */
	0x3C4, /* 35, +5.5dB */
	0x3FE  /* 36, +6.0dB */
};

static void rtl8821au_dm_dig(struct rtl_priv *rtlpriuv);

static const u32 edca_setting_ul[HT_IOT_PEER_MAX] = {
	0x5e4322,	/*  0 UNKNOWN */
	0xa44f,		/*  1 REALTEK_90 */
	0x5e4322,	/*  2 REALTEK_92SE */
	0x5ea32b,	/*  3 BROADCOM */
	0x5ea422,	/*  4 RALINK */
	0x5ea322,	/*  5 ATHEROS */
	0x3ea430,	/*  6 CISCO */
	0x5ea42b,	/*  7 MERU */
	0x5ea44f,	/*  8 MARVELL */
	0x5e4322,	/*  9 92U_AP */
	0x5e4322,	/* 10 SELF_AP(DownLink/Tx) */
};


static const u32 edca_setting_dl[HT_IOT_PEER_MAX] = {
	0xa44f,		/*  0 UNKNOWN */
	0x5ea44f,	/*  1 REALTEK_90 */
	0x5e4322,	/*  2 REALTEK_92SE */
	0x5ea42b,	/*  3 BROADCOM */
	0xa44f,		/*  4 RALINK */
	0xa630,		/*  5 ATHEROS */
	0x5ea630,	/*  6 CISCO */
	0x5ea42b,	/*  7 MERU */
	0xa44f,		/*  8 MARVELL */
	0xa42b,		/*  9 92U_AP */
	0xa42b		/* 10 SELF_AP(UpLink/Rx) */
};

static const u32 edca_setting_gmode[HT_IOT_PEER_MAX] = {
	0x4322,		/*  0 UNKNOWN */
	0xa44f,		/*  1 REALTEK_90 */
	0x5e4322,	/*  2 REALTEK_92SE */
	0xa42b,		/*  3 BROADCOM */
	0x5e4322,	/*  4 RALINK */
	0x4322,		/*  5 ATHEROS */
	0xa42b,		/*  6 CISCO */
	0x5ea42b,	/*  7 MERU */
	0xa44f,		/*  8 MARVELL */
	0x5e4322,	/*  9 92U_AP */
	0x5ea42b	/* 10 SELF_AP */
};

const u8 cckswing_table_ch1ch13_new[CCK_TABLE_SIZE][8] = {
	{0x09, 0x08, 0x07, 0x06, 0x04, 0x03, 0x01, 0x01},	/*  0, -16.0dB */
	{0x09, 0x09, 0x08, 0x06, 0x05, 0x03, 0x01, 0x01},	/*  1, -15.5dB */
	{0x0a, 0x09, 0x08, 0x07, 0x05, 0x03, 0x02, 0x01},	/*  2, -15.0dB */
	{0x0a, 0x0a, 0x09, 0x07, 0x05, 0x03, 0x02, 0x01},	/*  3, -14.5dB */
	{0x0b, 0x0a, 0x09, 0x08, 0x06, 0x04, 0x02, 0x01},	/*  4, -14.0dB */
	{0x0b, 0x0b, 0x0a, 0x08, 0x06, 0x04, 0x02, 0x01},	/*  5, -13.5dB */
	{0x0c, 0x0c, 0x0a, 0x09, 0x06, 0x04, 0x02, 0x01},	/*  6, -13.0dB */
	{0x0d, 0x0c, 0x0b, 0x09, 0x07, 0x04, 0x02, 0x01},	/*  7, -12.5dB */
	{0x0d, 0x0d, 0x0c, 0x0a, 0x07, 0x05, 0x02, 0x01},	/*  8, -12.0dB */
	{0x0e, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x02, 0x01},	/*  9, -11.5dB */
	{0x0f, 0x0f, 0x0d, 0x0b, 0x08, 0x05, 0x03, 0x01},	/* 10, -11.0dB */
	{0x10, 0x10, 0x0e, 0x0b, 0x08, 0x05, 0x03, 0x01},	/* 11, -10.5dB */
	{0x11, 0x11, 0x0f, 0x0c, 0x09, 0x06, 0x03, 0x01},	/* 12, -10.0dB */
	{0x12, 0x12, 0x0f, 0x0c, 0x09, 0x06, 0x03, 0x01},	/* 13, -9.5dB */
	{0x13, 0x13, 0x10, 0x0d, 0x0a, 0x06, 0x03, 0x01},	/* 14, -9.0dB */
	{0x14, 0x14, 0x11, 0x0e, 0x0b, 0x07, 0x03, 0x02},	/* 15, -8.5dB */
	{0x16, 0x15, 0x12, 0x0f, 0x0b, 0x07, 0x04, 0x01},	/* 16, -8.0dB */
	{0x17, 0x16, 0x13, 0x10, 0x0c, 0x08, 0x04, 0x02},	/* 17, -7.5dB */
	{0x18, 0x17, 0x15, 0x11, 0x0c, 0x08, 0x04, 0x02},	/* 18, -7.0dB */
	{0x1a, 0x19, 0x16, 0x12, 0x0d, 0x09, 0x04, 0x02},	/* 19, -6.5dB */
	{0x1b, 0x1a, 0x17, 0x13, 0x0e, 0x09, 0x04, 0x02},	/* 20, -6.0dB */
	{0x1d, 0x1c, 0x18, 0x14, 0x0f, 0x0a, 0x05, 0x02},	/* 21, -5.5dB */
	{0x1f, 0x1e, 0x1a, 0x15, 0x10, 0x0a, 0x05, 0x02},	/* 22, -5.0dB */
	{0x20, 0x20, 0x1b, 0x16, 0x11, 0x08, 0x05, 0x02},	/* 23, -4.5dB */
	{0x22, 0x21, 0x1d, 0x18, 0x11, 0x0b, 0x06, 0x02},	/* 24, -4.0dB */
	{0x24, 0x23, 0x1f, 0x19, 0x13, 0x0c, 0x06, 0x03},	/* 25, -3.5dB */
	{0x26, 0x25, 0x21, 0x1b, 0x14, 0x0d, 0x06, 0x03},	/* 26, -3.0dB */
	{0x28, 0x28, 0x22, 0x1c, 0x15, 0x0d, 0x07, 0x03},	/* 27, -2.5dB */
	{0x2b, 0x2a, 0x25, 0x1e, 0x16, 0x0e, 0x07, 0x03},	/* 28, -2.0dB */
	{0x2d, 0x2d, 0x27, 0x1f, 0x18, 0x0f, 0x08, 0x03},	/* 29, -1.5dB */
	{0x30, 0x2f, 0x29, 0x21, 0x19, 0x10, 0x08, 0x03},	/* 30, -1.0dB */
	{0x33, 0x32, 0x2b, 0x23, 0x1a, 0x11, 0x08, 0x04},	/* 31, -0.5dB */
	{0x36, 0x35, 0x2e, 0x25, 0x1c, 0x12, 0x09, 0x04} 	/* 32, +0dB */
};

const u8 cckswing_table_ch14_new[CCK_TABLE_SIZE][8] = {
	{0x09, 0x08, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00},	/*  0, -16.0dB */
	{0x09, 0x09, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00},	/*  1, -15.5dB */
	{0x0a, 0x09, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00},	/*  2, -15.0dB */
	{0x0a, 0x0a, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00},	/*  3, -14.5dB */
	{0x0b, 0x0a, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00},	/*  4, -14.0dB */
	{0x0b, 0x0b, 0x0a, 0x06, 0x00, 0x00, 0x00, 0x00},	/*  5, -13.5dB */
	{0x0c, 0x0c, 0x0a, 0x06, 0x00, 0x00, 0x00, 0x00},	/*  6, -13.0dB */
	{0x0d, 0x0c, 0x0b, 0x06, 0x00, 0x00, 0x00, 0x00},	/*  7, -12.5dB */
	{0x0d, 0x0d, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x00},	/*  8, -12.0dB */
	{0x0e, 0x0e, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x00},	/*  9, -11.5dB */
	{0x0f, 0x0f, 0x0d, 0x08, 0x00, 0x00, 0x00, 0x00},	/* 10, -11.0dB */
	{0x10, 0x10, 0x0e, 0x08, 0x00, 0x00, 0x00, 0x00},	/* 11, -10.5dB */
	{0x11, 0x11, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x00},	/* 12, -10.0dB */
	{0x12, 0x12, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x00},	/* 13, -9.5dB */
	{0x13, 0x13, 0x10, 0x0a, 0x00, 0x00, 0x00, 0x00},	/* 14, -9.0dB */
	{0x14, 0x14, 0x11, 0x0a, 0x00, 0x00, 0x00, 0x00},	/* 15, -8.5dB */
	{0x16, 0x15, 0x12, 0x0b, 0x00, 0x00, 0x00, 0x00},	/* 16, -8.0dB */
	{0x17, 0x16, 0x13, 0x0b, 0x00, 0x00, 0x00, 0x00},	/* 17, -7.5dB */
	{0x18, 0x17, 0x15, 0x0c, 0x00, 0x00, 0x00, 0x00},	/* 18, -7.0dB */
	{0x1a, 0x19, 0x16, 0x0d, 0x00, 0x00, 0x00, 0x00},	/* 19, -6.5dB */
	{0x1b, 0x1a, 0x17, 0x0e, 0x00, 0x00, 0x00, 0x00},	/* 20, -6.0dB */
	{0x1d, 0x1c, 0x18, 0x0e, 0x00, 0x00, 0x00, 0x00},	/* 21, -5.5dB */
	{0x1f, 0x1e, 0x1a, 0x0f, 0x00, 0x00, 0x00, 0x00},	/* 22, -5.0dB */
	{0x20, 0x20, 0x1b, 0x10, 0x00, 0x00, 0x00, 0x00},	/* 23, -4.5dB */
	{0x22, 0x21, 0x1d, 0x11, 0x00, 0x00, 0x00, 0x00},	/* 24, -4.0dB */
	{0x24, 0x23, 0x1f, 0x12, 0x00, 0x00, 0x00, 0x00},	/* 25, -3.5dB */
	{0x26, 0x25, 0x21, 0x13, 0x00, 0x00, 0x00, 0x00},	/* 26, -3.0dB */
	{0x28, 0x28, 0x24, 0x14, 0x00, 0x00, 0x00, 0x00},	/* 27, -2.5dB */
	{0x2b, 0x2a, 0x25, 0x15, 0x00, 0x00, 0x00, 0x00},	/* 28, -2.0dB */
	{0x2d, 0x2d, 0x17, 0x17, 0x00, 0x00, 0x00, 0x00},	/* 29, -1.5dB */
	{0x30, 0x2f, 0x29, 0x18, 0x00, 0x00, 0x00, 0x00},	/* 30, -1.0dB */
	{0x33, 0x32, 0x2b, 0x19, 0x00, 0x00, 0x00, 0x00},	/* 31, -0.5dB */
	{0x36, 0x35, 0x2e, 0x1b, 0x00, 0x00, 0x00, 0x00} 	/* 32, +0dB */
};


const u8 cckswing_table_ch1ch13[CCK_TABLE_SIZE][8] = {
	{0x36, 0x35, 0x2e, 0x25, 0x1c, 0x12, 0x09, 0x04},	/* 0, +0dB */
	{0x33, 0x32, 0x2b, 0x23, 0x1a, 0x11, 0x08, 0x04},	/* 1, -0.5dB */
	{0x30, 0x2f, 0x29, 0x21, 0x19, 0x10, 0x08, 0x03},	/* 2, -1.0dB */
	{0x2d, 0x2d, 0x27, 0x1f, 0x18, 0x0f, 0x08, 0x03},	/* 3, -1.5dB */
	{0x2b, 0x2a, 0x25, 0x1e, 0x16, 0x0e, 0x07, 0x03},	/* 4, -2.0dB */
	{0x28, 0x28, 0x22, 0x1c, 0x15, 0x0d, 0x07, 0x03},	/* 5, -2.5dB */
	{0x26, 0x25, 0x21, 0x1b, 0x14, 0x0d, 0x06, 0x03},	/* 6, -3.0dB */
	{0x24, 0x23, 0x1f, 0x19, 0x13, 0x0c, 0x06, 0x03},	/* 7, -3.5dB */
	{0x22, 0x21, 0x1d, 0x18, 0x11, 0x0b, 0x06, 0x02},	/* 8, -4.0dB */
	{0x20, 0x20, 0x1b, 0x16, 0x11, 0x08, 0x05, 0x02},	/* 9, -4.5dB */
	{0x1f, 0x1e, 0x1a, 0x15, 0x10, 0x0a, 0x05, 0x02},	/* 10, -5.0dB */
	{0x1d, 0x1c, 0x18, 0x14, 0x0f, 0x0a, 0x05, 0x02},	/* 11, -5.5dB */
	{0x1b, 0x1a, 0x17, 0x13, 0x0e, 0x09, 0x04, 0x02},	/* 12, -6.0dB <== default */
	{0x1a, 0x19, 0x16, 0x12, 0x0d, 0x09, 0x04, 0x02},	/* 13, -6.5dB */
	{0x18, 0x17, 0x15, 0x11, 0x0c, 0x08, 0x04, 0x02},	/* 14, -7.0dB */
	{0x17, 0x16, 0x13, 0x10, 0x0c, 0x08, 0x04, 0x02},	/* 15, -7.5dB */
	{0x16, 0x15, 0x12, 0x0f, 0x0b, 0x07, 0x04, 0x01},	/* 16, -8.0dB */
	{0x14, 0x14, 0x11, 0x0e, 0x0b, 0x07, 0x03, 0x02},	/* 17, -8.5dB */
	{0x13, 0x13, 0x10, 0x0d, 0x0a, 0x06, 0x03, 0x01},	/* 18, -9.0dB */
	{0x12, 0x12, 0x0f, 0x0c, 0x09, 0x06, 0x03, 0x01},	/* 19, -9.5dB */
	{0x11, 0x11, 0x0f, 0x0c, 0x09, 0x06, 0x03, 0x01},	/* 20, -10.0dB */
	{0x10, 0x10, 0x0e, 0x0b, 0x08, 0x05, 0x03, 0x01},	/* 21, -10.5dB */
	{0x0f, 0x0f, 0x0d, 0x0b, 0x08, 0x05, 0x03, 0x01},	/* 22, -11.0dB */
	{0x0e, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x02, 0x01},	/* 23, -11.5dB */
	{0x0d, 0x0d, 0x0c, 0x0a, 0x07, 0x05, 0x02, 0x01},	/* 24, -12.0dB */
	{0x0d, 0x0c, 0x0b, 0x09, 0x07, 0x04, 0x02, 0x01},	/* 25, -12.5dB */
	{0x0c, 0x0c, 0x0a, 0x09, 0x06, 0x04, 0x02, 0x01},	/* 26, -13.0dB */
	{0x0b, 0x0b, 0x0a, 0x08, 0x06, 0x04, 0x02, 0x01},	/* 27, -13.5dB */
	{0x0b, 0x0a, 0x09, 0x08, 0x06, 0x04, 0x02, 0x01},	/* 28, -14.0dB */
	{0x0a, 0x0a, 0x09, 0x07, 0x05, 0x03, 0x02, 0x01},	/* 29, -14.5dB */
	{0x0a, 0x09, 0x08, 0x07, 0x05, 0x03, 0x02, 0x01},	/* 30, -15.0dB */
	{0x09, 0x09, 0x08, 0x06, 0x05, 0x03, 0x01, 0x01},	/* 31, -15.5dB */
	{0x09, 0x08, 0x07, 0x06, 0x04, 0x03, 0x01, 0x01}	/* 32, -16.0dB */
};


const u8 cckswing_table_ch14[CCK_TABLE_SIZE][8] = {
	{0x36, 0x35, 0x2e, 0x1b, 0x00, 0x00, 0x00, 0x00},	/* 0, +0dB */
	{0x33, 0x32, 0x2b, 0x19, 0x00, 0x00, 0x00, 0x00},	/* 1, -0.5dB */
	{0x30, 0x2f, 0x29, 0x18, 0x00, 0x00, 0x00, 0x00},	/* 2, -1.0dB */
	{0x2d, 0x2d, 0x17, 0x17, 0x00, 0x00, 0x00, 0x00},	/* 3, -1.5dB */
	{0x2b, 0x2a, 0x25, 0x15, 0x00, 0x00, 0x00, 0x00},	/* 4, -2.0dB */
	{0x28, 0x28, 0x24, 0x14, 0x00, 0x00, 0x00, 0x00},	/* 5, -2.5dB */
	{0x26, 0x25, 0x21, 0x13, 0x00, 0x00, 0x00, 0x00},	/* 6, -3.0dB */
	{0x24, 0x23, 0x1f, 0x12, 0x00, 0x00, 0x00, 0x00},	/* 7, -3.5dB */
	{0x22, 0x21, 0x1d, 0x11, 0x00, 0x00, 0x00, 0x00},	/* 8, -4.0dB */
	{0x20, 0x20, 0x1b, 0x10, 0x00, 0x00, 0x00, 0x00},	/* 9, -4.5dB */
	{0x1f, 0x1e, 0x1a, 0x0f, 0x00, 0x00, 0x00, 0x00},	/* 10, -5.0dB */
	{0x1d, 0x1c, 0x18, 0x0e, 0x00, 0x00, 0x00, 0x00},	/* 11, -5.5dB */
	{0x1b, 0x1a, 0x17, 0x0e, 0x00, 0x00, 0x00, 0x00},	/* 12, -6.0dB  <== default */
	{0x1a, 0x19, 0x16, 0x0d, 0x00, 0x00, 0x00, 0x00},	/* 13, -6.5dB */
	{0x18, 0x17, 0x15, 0x0c, 0x00, 0x00, 0x00, 0x00},	/* 14, -7.0dB */
	{0x17, 0x16, 0x13, 0x0b, 0x00, 0x00, 0x00, 0x00},	/* 15, -7.5dB */
	{0x16, 0x15, 0x12, 0x0b, 0x00, 0x00, 0x00, 0x00},	/* 16, -8.0dB */
	{0x14, 0x14, 0x11, 0x0a, 0x00, 0x00, 0x00, 0x00},	/* 17, -8.5dB */
	{0x13, 0x13, 0x10, 0x0a, 0x00, 0x00, 0x00, 0x00},	/* 18, -9.0dB */
	{0x12, 0x12, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x00},	/* 19, -9.5dB */
	{0x11, 0x11, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x00},	/* 20, -10.0dB */
	{0x10, 0x10, 0x0e, 0x08, 0x00, 0x00, 0x00, 0x00},	/* 21, -10.5dB */
	{0x0f, 0x0f, 0x0d, 0x08, 0x00, 0x00, 0x00, 0x00},	/* 22, -11.0dB */
	{0x0e, 0x0e, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x00},	/* 23, -11.5dB */
	{0x0d, 0x0d, 0x0c, 0x07, 0x00, 0x00, 0x00, 0x00},	/* 24, -12.0dB */
	{0x0d, 0x0c, 0x0b, 0x06, 0x00, 0x00, 0x00, 0x00},	/* 25, -12.5dB */
	{0x0c, 0x0c, 0x0a, 0x06, 0x00, 0x00, 0x00, 0x00},	/* 26, -13.0dB */
	{0x0b, 0x0b, 0x0a, 0x06, 0x00, 0x00, 0x00, 0x00},	/* 27, -13.5dB */
	{0x0b, 0x0a, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 28, -14.0dB */
	{0x0a, 0x0a, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 29, -14.5dB */
	{0x0a, 0x09, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 30, -15.0dB */
	{0x09, 0x09, 0x08, 0x05, 0x00, 0x00, 0x00, 0x00},	/* 31, -15.5dB */
	{0x09, 0x08, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00}	/* 32, -16.0dB */
};

/*
 * 2011/09/21 MH Add to describe different team necessary resource allocate??
 */

static void rtl8821au_cm_common_info_self_update(struct rtl_priv *rtlpriv)
{
	struct rtl_phy *rtlphy = &(rtlpriv->phy);
	u8 tmp;

	rtlphy->cck_high_power = (bool) rtl_get_bbreg(rtlpriv, ODM_REG_CCK_RPT_FORMAT_11AC, ODM_BIT_CCK_RPT_FORMAT_11AC);
	tmp = (u8) rtl_get_bbreg(rtlpriv, ODM_REG_BB_RX_PATH_11AC,
				 ODM_BIT_BB_RX_PATH_11AC);

	if (tmp & BIT(0))
		rtlpriv->dm.rfpath_rxenable[0] = true;
	if (tmp & BIT(1))
		rtlpriv->dm.rfpath_rxenable[1] = true;
}

/* Ulli : check function in rtlwifi/core.c for _rtl_dm_diginit() */

static void _rtl_dm_diginit(struct rtl_priv *rtlpriv)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);
	struct dig_t *dm_digtable = &(rtlpriv->dm_digtable);

	/* dm_digtable->Dig_Enable_Flag = true; */
	/* dm_digtable->Dig_Ext_Port_Stage = DIG_EXT_PORT_STAGE_MAX; */
	dm_digtable->cur_igvalue = (u8) rtl_get_bbreg(rtlpriv, ODM_REG_IGI_A_11AC, ODM_BIT_IGI_11AC);
	/* dm_digtable->PreIGValue = 0x0; */
	/* dm_digtable->CurSTAConnectState = dm_digtable->PreSTAConnectState = DIG_STA_DISCONNECT; */
	/* dm_digtable->CurMultiSTAConnectState = DIG_MultiSTA_DISCONNECT; */
	dm_digtable->rssi_lowthresh 	= DM_DIG_THRESH_LOW;
	dm_digtable->rssi_highthresh	= DM_DIG_THRESH_HIGH;
	dm_digtable->fa_lowthresh	= DM_FALSEALARM_THRESH_LOW;
	dm_digtable->fa_highthresh	= DM_FALSEALARM_THRESH_HIGH;

	if (rtlhal->board_type & (ODM_BOARD_EXT_PA|ODM_BOARD_EXT_LNA)) {
		dm_digtable->rx_gain_max = DM_DIG_MAX_NIC;
		dm_digtable->rx_gain_min = DM_DIG_MIN_NIC;
	} else {
		dm_digtable->rx_gain_max = DM_DIG_MAX_NIC;
		dm_digtable->rx_gain_min = DM_DIG_MIN_NIC;
	}
	dm_digtable->back_val = DM_DIG_BACKOFF_DEFAULT;
	dm_digtable->back_range_max = DM_DIG_BACKOFF_MAX;
	dm_digtable->back_range_min = DM_DIG_BACKOFF_MIN;
	dm_digtable->pre_cck_cca_thres = 0xFF;
	dm_digtable->cur_cck_cca_thres = 0x83;
	dm_digtable->forbidden_igi= DM_DIG_MIN_NIC;
	dm_digtable->large_fa_hit = 0;
	dm_digtable->recover_cnt = 0;
	dm_digtable->dig_min_0 = DM_DIG_MIN_NIC;
	dm_digtable->dig_min_1 = DM_DIG_MIN_NIC;
	dm_digtable->media_connect_0 = false;
	dm_digtable->media_connect_1 = false;

	/* To Initi BT30 IGI */
	dm_digtable->bt30_cur_igi = 0x32;

}

static void odm_AdaptivityInit(struct rtl_priv *rtlpriv)
{
	struct _rtw_hal	*pHalData = GET_HAL_DATA(rtlpriv);
	struct _rtw_dm *pDM_Odm = &pHalData->odmpriv;

	pDM_Odm->TH_H = 0xfa; 		/* -6dB */
	pDM_Odm->TH_L = 0xfd; 		/* -3dB */
	pDM_Odm->IGI_Base = 0x32;
	pDM_Odm->IGI_target = 0x1c;
	pDM_Odm->ForceEDCCA = 0;
	pDM_Odm->AdapEn_RSSI = 32;	/* 45; */
}

static void rtl8821au_dm_init_rate_adaptive_mask(struct rtl_priv *rtlpriv)
{
	struct rate_adaptive *p_ra = &(rtlpriv->ra);

	p_ra->ratr_state = DM_RATR_STA_INIT;
	p_ra->ldpc_thres = 35;
	p_ra->use_ldpc = false;
	p_ra->high_rssi_thresh_for_ra = 50;
	p_ra->low2high_rssi_thresh_for_ra40m = 20;
}

static void rtl8821au_dm_init_edca_turbo(struct rtl_priv *rtlpriv)
{
	rtlpriv->dm.current_turbo_edca = false;
	rtlpriv->dm.is_cur_rdlstate = false;
	rtlpriv->dm.is_any_nonbepkts = false;

	RT_TRACE(rtlpriv, COMP_TURBO, DBG_LOUD, "Orginial VO PARAM: 0x%x\n", rtl_read_dword(rtlpriv, ODM_EDCA_VO_PARAM));
	RT_TRACE(rtlpriv, COMP_TURBO, DBG_LOUD, "Orginial VI PARAM: 0x%x\n", rtl_read_dword(rtlpriv, ODM_EDCA_VI_PARAM));
	RT_TRACE(rtlpriv, COMP_TURBO, DBG_LOUD, "Orginial BE PARAM: 0x%x\n", rtl_read_dword(rtlpriv, ODM_EDCA_BE_PARAM));
	RT_TRACE(rtlpriv, COMP_TURBO, DBG_LOUD, "Orginial BK PARAM: 0x%x\n", rtl_read_dword(rtlpriv, ODM_EDCA_BK_PARAM));


}

static u8 getSwingIndex(struct rtl_priv *rtlpriv)
{
	u8 			i = 0;
	uint32_t 			bbSwing;

	bbSwing = phy_get_tx_swing_8821au(rtlpriv, rtlpriv->rtlhal.current_bandtype, RF90_PATH_A);

	for (i = 0; i < TXSCALE_TABLE_SIZE; ++i)
		if (bbSwing == txscalling_tbl[i])
			break;

	return i;
}


static void rtl8821au_dm_initialize_txpower_tracking_thermalmeter(struct rtl_priv *rtlpriv)
{
	u8 		p = 0;
	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);

	rtldm->txpower_tracking = true;
	rtldm->txpowercount = 0;
	rtldm->txpower_trackinginit = false;
	/* #if	(MP_DRIVER != 1) */		/* for mp driver, turn off txpwrtracking as default */
	/* #endif//#if	(MP_DRIVER != 1) */
	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "rtldm->txpower_track_control= %d\n", rtldm->txpower_track_control);

	rtldm->txpower_track_control = true;
	rtldm->thermalvalue = efuse->eeprom_thermalmeter;
	rtldm->thermalvalue_iqk = efuse->eeprom_thermalmeter;
	rtldm->thermalvalue_lck = efuse->eeprom_thermalmeter;

	/* The index of "0 dB" in SwingTable. */
	{
		u8 defaultSwingIndex = getSwingIndex(rtlpriv);


		rtldm->default_ofdm_index = (defaultSwingIndex == TXSCALE_TABLE_SIZE) ? 24 : defaultSwingIndex;
		rtldm->default_cck_index = 24;
	}

	rtldm->swing_idx_cck_base = rtldm->default_cck_index;
	rtldm->cck_index = rtldm->default_cck_index;

	for (p = RF90_PATH_A; p < MAX_RF_PATH; ++p) {
		rtldm->swing_idx_ofdm_base[p] = rtldm->default_ofdm_index;
		rtldm->ofdm_index[p] = rtldm->default_ofdm_index;
		rtldm->delta_power_index[p] = 0;
		rtldm->delta_power_index_last[p] = 0;
		rtldm->power_index_offset[p] = 0;
	}
}

void ODM_DMInit(struct rtl_priv *rtlpriv)
{
	/* 2012.05.03 Luke: For all IC series */
	rtl8821au_cm_common_info_self_update(rtlpriv);
	/* Ulli : check function in rtlwifi/core.c for _rtl_dm_diginit() */
	_rtl_dm_diginit(rtlpriv);
	odm_AdaptivityInit(rtlpriv);
	rtl8821au_dm_init_rate_adaptive_mask(rtlpriv);

	rtl8821au_dm_init_edca_turbo(rtlpriv);

	rtl8821au_dm_initialize_txpower_tracking_thermalmeter(rtlpriv);
}

/* From hal/OUTSRC/rtl8812a/HalPhyRf_8812A.c */

/*
 * ============================================================
 *  Tx Power Tracking
 * ============================================================
 */
/* From hal/OUTSRC/rtl8812a/HalPhyRf_8812A.c, caution function pointer */
static void rtl8812au_do_iqk(struct rtl_priv *rtlpriv, u8 DeltaThermalIndex,
	u8 	ThermalValue, u8 Threshold)
{
	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);

	rtldm->thermalvalue_iqk = ThermalValue;
	rtl8812au_phy_iq_calibrate(rtlpriv, false);
}

/*-----------------------------------------------------------------------------
 * Function:	odm_TxPwrTrackSetPwr88E()
 *
 * Overview:	88E change all channel tx power accordign to flag.
 *				OFDM & CCK are all different.
 *
 * Input:		NONE
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	04/23/2012	MHC		Create Version 0.
 *
 *---------------------------------------------------------------------------*/

static void rtl8812au_dm_pxpwr_track_set_pwr(struct rtl_priv *rtlpriv,
					     enum pwr_track_control_method Method,
					     u8 RFPath, u8 ChannelMappedIndex)
{
	uint32_t 	finalBbSwingIdx[2];

	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);

	u8 PwrTrackingLimit = 26; /* +1.0dB */
	u8 TxRate = 0xFF;
	s8 Final_OFDM_Swing_Index = 0;

	if (TxRate != 0xFF) {
		/* Ulli better with switch case, see in rtlwifi-lib */
		/* 2 CCK */
		if ((TxRate >= MGN_1M) && (TxRate <= MGN_11M))
			PwrTrackingLimit = 32;				/* +4dB */
		/* 2 OFDM */
		else if ((TxRate >= MGN_6M) && (TxRate <= MGN_48M))
			PwrTrackingLimit = 32;				/* +4dB */
		else if (TxRate == MGN_54M)
			PwrTrackingLimit = 30;				/* +3dB */

		/* ULLI 80211n */
		/* 2 HT */
		else if ((TxRate >= MGN_MCS0) && (TxRate <= MGN_MCS2))  /* QPSK/BPSK */
			PwrTrackingLimit = 34;				/* +5dB */
		else if ((TxRate >= MGN_MCS3) && (TxRate <= MGN_MCS4))  /* 16QAM */
			PwrTrackingLimit = 32;				/* +4dB */
		else if ((TxRate >= MGN_MCS5) && (TxRate <= MGN_MCS7))  /* 64QAM */
			PwrTrackingLimit = 30;				/* +3dB */

		else if ((TxRate >= MGN_MCS8) && (TxRate <= MGN_MCS10)) 	/* QPSK/BPSK */
			PwrTrackingLimit = 34; 				/* +5dB */
		else if ((TxRate >= MGN_MCS11) && (TxRate <= MGN_MCS12)) 	/* 16QAM */
			PwrTrackingLimit = 32; 				/* +4dB */
		else if ((TxRate >= MGN_MCS13) && (TxRate <= MGN_MCS15)) 	/* 64QAM */
			PwrTrackingLimit = 30; 				/* +3dB */

		/* Ulli 802.11ac */
		/* 2 VHT */
		else if ((TxRate >= MGN_VHT1SS_MCS0) && (TxRate <= MGN_VHT1SS_MCS2))    /* QPSK/BPSK */
			PwrTrackingLimit = 34;						/* +5dB */
		else if ((TxRate >= MGN_VHT1SS_MCS3) && (TxRate <= MGN_VHT1SS_MCS4))    /* 16QAM */
			PwrTrackingLimit = 32;						/* +4dB */
		else if ((TxRate >= MGN_VHT1SS_MCS5) && (TxRate <= MGN_VHT1SS_MCS6))    /* 64QAM */
			PwrTrackingLimit = 30;						/* +3dB */
		else if (TxRate == MGN_VHT1SS_MCS7)					/* 64QAM */
			PwrTrackingLimit = 28;						/* +2dB */
		else if (TxRate == MGN_VHT1SS_MCS8)					/* 256QAM */
			PwrTrackingLimit = 26;						/* +1dB */
		else if (TxRate == MGN_VHT1SS_MCS9)					/* 256QAM */
			PwrTrackingLimit = 24;						/* +0dB */

		else if ((TxRate >= MGN_VHT2SS_MCS0) && (TxRate <= MGN_VHT2SS_MCS2)) 	/* QPSK/BPSK */
			PwrTrackingLimit = 34; 						/* +5dB */
		else if ((TxRate >= MGN_VHT2SS_MCS3) && (TxRate <= MGN_VHT2SS_MCS4)) 	/* 16QAM */
			PwrTrackingLimit = 32; 						/* +4dB */
		else if ((TxRate >= MGN_VHT2SS_MCS5) && (TxRate <= MGN_VHT2SS_MCS6)) 	/* 64QAM */
			PwrTrackingLimit = 30; 						/* +3dB */
		else if (TxRate == MGN_VHT2SS_MCS7) 					/* 64QAM */
			PwrTrackingLimit = 28; 						/* +2dB */
		else if (TxRate == MGN_VHT2SS_MCS8) 					/* 256QAM */
			PwrTrackingLimit = 26; 						/* +1dB */
		else if (TxRate == MGN_VHT2SS_MCS9) 					/* 256QAM */
			PwrTrackingLimit = 24; 						/* +0dB */

		else
			PwrTrackingLimit = 24;
	}
	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "TxRate=0x%x, PwrTrackingLimit=%d\n", TxRate, PwrTrackingLimit);
	if (Method == BBSWING) {
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "===>rtl8812au_dm_pxpwr_track_set_pwr\n");
		if (RFPath == RF90_PATH_A) {
			finalBbSwingIdx[RF90_PATH_A] = (rtldm->ofdm_index[RF90_PATH_A] > PwrTrackingLimit) ? PwrTrackingLimit : rtldm->ofdm_index[RF90_PATH_A];
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "pDM_Odm->RFCalibrateInfo.OFDM_index[RF90_PATH_A]=%d, pDM_Odm->RealBbSwingIdx[RF90_PATH_A]=%d\n",
				rtldm->ofdm_index[RF90_PATH_A], finalBbSwingIdx[RF90_PATH_A]);
			rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[finalBbSwingIdx[RF90_PATH_A]]);
		} else {
			finalBbSwingIdx[RF90_PATH_B] = (rtldm->ofdm_index[RF90_PATH_B] > PwrTrackingLimit) ? PwrTrackingLimit : rtldm->ofdm_index[RF90_PATH_B];
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "pDM_Odm->RFCalibrateInfo.OFDM_index[RF90_PATH_B]=%d, pDM_Odm->RealBbSwingIdx[RF90_PATH_B]=%d\n",
				rtldm->ofdm_index[RF90_PATH_B], finalBbSwingIdx[RF90_PATH_B]);
			rtl_set_bbreg(rtlpriv, rB_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[finalBbSwingIdx[RF90_PATH_B]]);
		}
	} else if (Method == MIX_MODE) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "pDM_Odm->DefaultOfdmIndex=%d, pDM_Odm->Aboslute_OFDMSwingIdx[RFPath]=%d, RF_Path = %d\n",
				rtldm->default_ofdm_index, rtldm->absolute_ofdm_swing_idx[RFPath], RFPath);

			Final_OFDM_Swing_Index = rtldm->default_ofdm_index + rtldm->absolute_ofdm_swing_idx[RFPath];

			if (RFPath == RF90_PATH_A) {
				if (Final_OFDM_Swing_Index > PwrTrackingLimit) {    /* BBSwing higher then Limit */
					rtldm->remnant_cck_idx = Final_OFDM_Swing_Index - PwrTrackingLimit;            /*  CCK Follow the same compensate value as Path A */
					rtldm->remnant_ofdm_swing_idx[RFPath] = Final_OFDM_Swing_Index - PwrTrackingLimit;

					rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[PwrTrackingLimit]);

					rtldm->modify_txagc_flag_path_a = true;

					/* et TxAGC Page C{}; */
					/* rtlpriv->cfg->ops.SetTxPowerLevelHandler(rtlpriv, pHalData->CurrentChannel); */
					PHY_SetTxPowerLevel8812(rtlpriv, rtlpriv->phy.current_channel);
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A Over BBSwing Limit , PwrTrackingLimit = %d , Remnant TxAGC Value = %d \n", PwrTrackingLimit, rtldm->remnant_ofdm_swing_idx[RFPath]);
				} else if (Final_OFDM_Swing_Index < 0) {
					rtldm->remnant_cck_idx = Final_OFDM_Swing_Index;            /* CCK Follow the same compensate value as Path A */
					rtldm->remnant_ofdm_swing_idx[RFPath] = Final_OFDM_Swing_Index;

					rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[0]);

					rtldm->modify_txagc_flag_path_a = true;

					/* Set TxAGC Page C{}; */
					/* rtlpriv->cfg->ops.SetTxPowerLevelHandler(rtlpriv, pHalData->CurrentChannel);*/
					PHY_SetTxPowerLevel8812(rtlpriv, rtlpriv->phy.current_channel);
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A Lower then BBSwing lower bound  0 , Remnant TxAGC Value = %d \n", rtldm->remnant_ofdm_swing_idx[RFPath]);
				} else 	{
					rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[Final_OFDM_Swing_Index]);
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A Compensate with BBSwing , Final_OFDM_Swing_Index = %d \n", Final_OFDM_Swing_Index);

					if (rtldm->modify_txagc_flag_path_a) { /* If TxAGC has changed, reset TxAGC again */
						rtldm->remnant_cck_idx = 0;
						rtldm->remnant_ofdm_swing_idx[RFPath] = 0;

						/* Set TxAGC Page C{}; */
						/* rtlpriv->cfg->ops.SetTxPowerLevelHandler(rtlpriv, pHalData->CurrentChannel); */
						PHY_SetTxPowerLevel8812(rtlpriv, rtlpriv->phy.current_channel);

						rtldm->modify_txagc_flag_path_a = false;
						RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A pDM_Odm->Modify_TxAGC_Flag = false \n");
					}
				}
			}

			if (RFPath == RF90_PATH_B) {
				if (Final_OFDM_Swing_Index > PwrTrackingLimit) {			/* BBSwing higher then Limit */
					rtldm->remnant_ofdm_swing_idx[RFPath] = Final_OFDM_Swing_Index - PwrTrackingLimit;

					rtl_set_bbreg(rtlpriv, rB_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[PwrTrackingLimit]);

					rtldm->modify_txagc_flag_path_b = true;

					/* Set TxAGC Page E{}; */
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_B Over BBSwing Limit , PwrTrackingLimit = %d , Remnant TxAGC Value = %d \n", PwrTrackingLimit, rtldm->remnant_ofdm_swing_idx[RFPath]);
				} else if (Final_OFDM_Swing_Index < 0) {
					rtldm->remnant_ofdm_swing_idx[RFPath] = Final_OFDM_Swing_Index;

					rtl_set_bbreg(rtlpriv, rB_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[0]);

					rtldm->modify_txagc_flag_path_b = true;

					/* Set TxAGC Page E{}; */
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_B Lower then BBSwing lower bound  0 , Remnant TxAGC Value = %d \n", rtldm->remnant_ofdm_swing_idx[RFPath]);
				} else {
					rtl_set_bbreg(rtlpriv, rB_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[Final_OFDM_Swing_Index]);
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_B Compensate with BBSwing , Final_OFDM_Swing_Index = %d \n", Final_OFDM_Swing_Index);
					if (rtldm->modify_txagc_flag_path_b) {			/* If TxAGC has changed, reset TxAGC again */
						rtldm->remnant_cck_idx = 0;
						rtldm->remnant_ofdm_swing_idx[RFPath] = 0;

						/* Set TxAGC Page E{}; */

						rtldm->modify_txagc_flag_path_b = false;
						RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_B pDM_Odm->Modify_TxAGC_Flag = false \n");
					}
				}
			}

	} else {
		return;
	}
}

/* START Copied from hal/OUTSRC/rtl8812a/HalHWImg8812A_RF.c */
/******************************************************************************
*                           TxPowerTrack_USB.TXT
******************************************************************************/

static u8 rtl8818e_delta_swing_table_idx_24gb_p[] = {
	0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4,  4,  4,  4,  4,
	4,  5,  5,  7,  7,  8,  8,  8,  9,  9,  9,  9,  9};
static u8 rtl8818e_delta_swing_table_idx_24gb_n[] = {
	0, 0, 0, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5,  6,  6,  7,  7,
	7,  7,  8,  8,  9,  9, 10, 10, 10, 11, 11, 11, 11};

static u8 rtl8812ae_delta_swing_table_idx_5gb_n[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14},
	{0, 1, 1, 2, 3, 3, 4, 5, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 6,  7,  7,  8,  8,  9,  9, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8812ae_delta_swing_table_idx_5gb_p[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 3, 4, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 6,  7,  7,  8,  8,  9,  9, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8812ae_delta_swing_table_idx_5ga_n[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 15, 15, 15},
	{0, 1, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 15, 15, 15},
	{0, 1, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 15, 15, 15},
};

static u8 rtl8812ae_delta_swing_table_idx_5ga_p[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 6,  7,  7,  8,  8,  9, 10, 11,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 6,  7,  7,  8,  8,  9, 10, 11,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 6,  7,  7,  8,  8,  9, 10, 11,
	11, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8812au_delta_swing_table_idx_24gb_n[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  5,  5,  5,  6,  6,
	7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 11, 11, 11, 11
};

static u8 rtl8812au_delta_swing_table_idx_24gb_p[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};

static u8 rtl8812au_delta_swing_table_idx_24ga_n[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  8,  8,  9, 10, 10, 10, 10, 10, 10
};

static u8 rtl8812au_delta_swing_table_idx_24ga_p[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};

static u8 rtl8812au_delta_swing_table_idx_24gcckb_n[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  5,  5,  5,  6,  6,
	7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 11, 11, 11, 11
};

static u8 rtl8812au_delta_swing_table_idx_24gcckb_p[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};

static u8 rtl8812au_delta_swing_table_idx_24gccka_n[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  8,  8,  9, 10, 10, 10, 10, 10, 10
};

static u8 rtl8812au_delta_swing_table_idx_24gccka_p[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};

/******************************************************************************
*                           TxPowerTrack_USB_RFE3.TXT
******************************************************************************/

static u8 rtl8812ae_delta_swing_table_idx_5gb_n_rfe3[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 3, 3, 4, 5, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 12, 13, 13, 14, 15, 16, 16, 17, 17, 18, 18},
	{0, 1, 1, 2, 2, 3, 3, 4, 4,  5,  6,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 2, 3, 3, 4, 4,  5,  6,  6,  7,  7,  8,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8812ae_delta_swing_table_idx_5gb_p_rfe3[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 3, 3, 4, 5, 6,  7,  7,  8,  9,  9, 10, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 2, 3, 3, 4, 4,  5,  6,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 2, 3, 3, 4, 4,  5,  6,  6,  7,  7,  8,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8812ae_delta_swing_table_idx_5ga_n_rfe3[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 3, 3, 4, 5, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 12, 13, 14, 15, 16, 16, 17, 17, 18, 18},
	{0, 1, 1, 2, 3, 3, 4, 4, 5,  6,  6,  7,  7,  8,  9,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 16, 16, 17, 17},
	{0, 1, 1, 2, 3, 3, 4, 4, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 17, 18, 18},
};

static u8 rtl8812ae_delta_swing_table_idx_5ga_p_rfe3[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 6,  7,  7,  8,  9,  9, 10, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 2, 3, 4, 4, 5, 5, 6,  7,  7,  8,  9,  9, 10, 11,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 2, 3, 4, 4, 5, 5, 6,  7,  7,  8,  9,  9, 10, 11,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8812au_delta_swing_table_idx_24gb_n_rfe3[] = {
	0, 1, 1, 2, 2, 3, 3, 4, 4,  4,  5,  5,  6,  6,  6,  7,
	7,  7,  8,  8,  9,  9, 10, 11, 12, 12, 13, 14, 15, 15
};

static u8 rtl8812au_delta_swing_table_idx_24gb_p_rfe3[] = {
	0, 1, 1, 2, 2, 2, 2, 3, 3,  3,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  8,  9, 10, 10, 10, 10, 11, 11, 11, 11, 11
};

static u8 rtl8812au_delta_swing_table_idx_24ga_n_rfe3[] = {
	0, 1, 1, 2, 2, 3, 4, 5, 6,  6,  6,  7,  7,  8,  8,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 15, 15
};

static u8 rtl8812au_delta_swing_table_idx_24ga_p_rfe3[] = {
	0, 0, 1, 1, 1, 2, 2, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 11, 11, 11
};

static u8 rtl8812au_delta_swing_table_idx_24gcckb_n_rfe3[] = {
	0, 1, 1, 2, 2, 3, 3, 4, 4,  4,  5,  5,  6,  6,  6,  7,
	7,  7,  8,  8,  9,  9, 10, 11, 12, 12, 13, 14, 15, 15
};

static u8 rtl8812au_delta_swing_table_idx_24gcckb_p_rfe3[] = {
	0, 1, 1, 2, 2, 2, 2, 3, 3,  3,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  8,  9, 10, 10, 10, 10, 11, 11, 11, 11, 11
};

static u8 rtl8812au_delta_swing_table_idx_24gccka_n_rfe3[] = {
	0, 1, 1, 2, 2, 3, 4, 5, 6,  6,  6,  7,  7,  8,  8,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 15, 15
};

static u8 rtl8812au_delta_swing_table_idx_24gccka_p_rfe3[] = {
	0, 0, 1, 1, 1, 2, 2, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 11, 11, 11
};

/* END Copied from hal/OUTSRC/rtl8812a/HalHWImg8812A_RF.c */

void rtl8812au_get_delta_swing_table(struct rtl_priv *rtlpriv,
					    u8 **up_a, u8 **down_a,
					    u8 **up_b, u8 **down_b)
{
	struct rtl_hal	*rtlhal = rtl_hal(rtlpriv);
	struct rtl_phy *rtlphy = &rtlpriv->phy;
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);

	u8 rate = rtldm->tx_rate;
	u8 channel = rtlphy->current_channel;

	if (rtlhal->rfe_type == 3 && IS_NORMAL_CHIP(rtlhal->version)) {
		if (1 <= channel && channel <= 14) {
			if (IS_CCK_RATE(rate)) {
				*up_a   = rtl8812au_delta_swing_table_idx_24gccka_p_rfe3;
				*down_a = rtl8812au_delta_swing_table_idx_24gccka_n_rfe3;
				*up_b   = rtl8812au_delta_swing_table_idx_24gcckb_p_rfe3;
				*down_b = rtl8812au_delta_swing_table_idx_24gcckb_n_rfe3;
			} else {
				*up_a   = rtl8812au_delta_swing_table_idx_24ga_p_rfe3;
				*down_a = rtl8812au_delta_swing_table_idx_24ga_n_rfe3;
				*up_b   = rtl8812au_delta_swing_table_idx_24gb_p_rfe3;
				*down_b = rtl8812au_delta_swing_table_idx_24gb_n_rfe3;
			}
		} else if (36 <= channel && channel <= 64) {
			*up_a   = rtl8812ae_delta_swing_table_idx_5ga_p_rfe3[0];
			*down_a = rtl8812ae_delta_swing_table_idx_5ga_n_rfe3[0];
			*up_b   = rtl8812ae_delta_swing_table_idx_5gb_p_rfe3[0];
			*down_b = rtl8812ae_delta_swing_table_idx_5gb_n_rfe3[0];
		} else if (100 <= channel && channel <= 140) {
			*up_a   = rtl8812ae_delta_swing_table_idx_5ga_p_rfe3[1];
			*down_a = rtl8812ae_delta_swing_table_idx_5ga_n_rfe3[1];
			*up_b   = rtl8812ae_delta_swing_table_idx_5gb_p_rfe3[1];
			*down_b = rtl8812ae_delta_swing_table_idx_5gb_n_rfe3[1];
		} else if (149 <= channel && channel <= 173) {
			*up_a   = rtl8812ae_delta_swing_table_idx_5ga_p_rfe3[2];
			*down_a = rtl8812ae_delta_swing_table_idx_5ga_n_rfe3[2];
			*up_b   = rtl8812ae_delta_swing_table_idx_5gb_p_rfe3[2];
			*down_b = rtl8812ae_delta_swing_table_idx_5gb_n_rfe3[2];
		} else {
			*up_a   = (u8 *)rtl8818e_delta_swing_table_idx_24gb_p;
			*down_a = (u8 *)rtl8818e_delta_swing_table_idx_24gb_n;
			*up_b   = (u8 *)rtl8818e_delta_swing_table_idx_24gb_p;
			*down_b = (u8 *)rtl8818e_delta_swing_table_idx_24gb_n;
		}
	} else {
		if (1 <= channel && channel <= 14) {
			if (IS_CCK_RATE(rate)) {
				*up_a   = rtl8812au_delta_swing_table_idx_24gccka_p;
				*down_a = rtl8812au_delta_swing_table_idx_24gccka_n;
				*up_b   = rtl8812au_delta_swing_table_idx_24gcckb_p;
				*down_b = rtl8812au_delta_swing_table_idx_24gcckb_n;
			} else {
				*up_a   = rtl8812au_delta_swing_table_idx_24ga_p;
				*down_a = rtl8812au_delta_swing_table_idx_24ga_n;
				*up_b   = rtl8812au_delta_swing_table_idx_24gb_p;
				*down_b = rtl8812au_delta_swing_table_idx_24gb_n;
			}
		} else if (36 <= channel && channel <= 64) {
			*up_a   = rtl8812ae_delta_swing_table_idx_5ga_p[0];
			*down_a = rtl8812ae_delta_swing_table_idx_5ga_n[0];
			*up_b   = rtl8812ae_delta_swing_table_idx_5gb_p[0];
			*down_b = rtl8812ae_delta_swing_table_idx_5gb_n[0];
		} else if (100 <= channel && channel <= 140) {
			*up_a   = rtl8812ae_delta_swing_table_idx_5ga_p[1];
			*down_a = rtl8812ae_delta_swing_table_idx_5ga_n[1];
			*up_b   = rtl8812ae_delta_swing_table_idx_5gb_p[1];
			*down_b = rtl8812ae_delta_swing_table_idx_5gb_n[1];
		} else if (149 <= channel && channel <= 173) {
			*up_a   = rtl8812ae_delta_swing_table_idx_5ga_p[2];
			*down_a = rtl8812ae_delta_swing_table_idx_5ga_n[2];
			*up_b   = rtl8812ae_delta_swing_table_idx_5gb_p[2];
			*down_b = rtl8812ae_delta_swing_table_idx_5gb_n[2];
		} else {
			*up_a   = (u8 *)rtl8818e_delta_swing_table_idx_24gb_p;
			*down_a = (u8 *)rtl8818e_delta_swing_table_idx_24gb_n;
			*up_b   = (u8 *)rtl8818e_delta_swing_table_idx_24gb_p;
			*down_b = (u8 *)rtl8818e_delta_swing_table_idx_24gb_n;
		}
	}

	return;
}


/* From hal/OUTSRC/rtl8821a/HalPhyRf_8821A.c, caution function pointer */

static void rtl8821au_do_iqk(struct rtl_priv *rtlpriv, u8 DeltaThermalIndex,
	u8 ThermalValue, u8 Threshold)
{
	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);

	rtldm->thermalvalue_iqk = ThermalValue;
	rtl8821au_phy_iq_calibrate(rtlpriv, false);
}


static void rtl8821au_dm_txpwr_track_set_pwr(struct rtl_priv *rtlpriv,
					     enum pwr_track_control_method Method,
					     u8 RFPath, u8 ChannelMappedIndex)
{
	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);

	u8 PwrTrackingLimit = 26; /* +1.0dB */
	u8 TxRate = 0xFF;
	s8 Final_OFDM_Swing_Index = 0;
	uint32_t finalBbSwingIdx[1];

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "===>ODM_TxPwrTrackSetPwr8821A\n");
	if (TxRate != 0xFF) {
		/* 2 CCK */
		if ((TxRate >= MGN_1M) && (TxRate <= MGN_11M))
			PwrTrackingLimit = 32;				/* +4dB */
		/* 2 OFDM */
		else if ((TxRate >= MGN_6M) && (TxRate <= MGN_48M))
			PwrTrackingLimit = 32;				/* +4dB */
		else if (TxRate == MGN_54M)
			PwrTrackingLimit = 30;				/* +3dB */
		/* 2 HT */
		else if ((TxRate >= MGN_MCS0) && (TxRate <= MGN_MCS2))  /* QPSK/BPSK */
			PwrTrackingLimit = 34;				/* +5dB */
		else if ((TxRate >= MGN_MCS3) && (TxRate <= MGN_MCS4))  /* 16QAM */
			PwrTrackingLimit = 32;				/* +4dB */
		else if ((TxRate >= MGN_MCS5) && (TxRate <= MGN_MCS7))  /* 64QAM */
			PwrTrackingLimit = 30;				/* +3dB */
		/* 2 VHT */
		else if ((TxRate >= MGN_VHT1SS_MCS0) && (TxRate <= MGN_VHT1SS_MCS2))    /* QPSK/BPSK */
			PwrTrackingLimit = 34;						/* +5dB */
		else if ((TxRate >= MGN_VHT1SS_MCS3) && (TxRate <= MGN_VHT1SS_MCS4))    /* 16QAM */
			PwrTrackingLimit = 32;						/* +4dB */
		else if ((TxRate >= MGN_VHT1SS_MCS5) && (TxRate <= MGN_VHT1SS_MCS6))    /* 64QAM */
			PwrTrackingLimit = 30;						/* +3dB */
		else if (TxRate == MGN_VHT1SS_MCS7)					/* 64QAM */
			PwrTrackingLimit = 28;						/* +2dB */
		else if (TxRate == MGN_VHT1SS_MCS8)					/* 256QAM */
			PwrTrackingLimit = 26;						/* +1dB */
		else if (TxRate == MGN_VHT1SS_MCS9)					/* 256QAM */
			PwrTrackingLimit = 24;						/* +0dB */
		else
			PwrTrackingLimit = 24;
	}
	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "TxRate=0x%x, PwrTrackingLimit=%d\n", TxRate, PwrTrackingLimit);
	if (Method == BBSWING) {
		if (RFPath == RF90_PATH_A) {
			finalBbSwingIdx[RF90_PATH_A] = (rtldm->ofdm_index[RF90_PATH_A] > PwrTrackingLimit) ? PwrTrackingLimit : rtldm->ofdm_index[RF90_PATH_A];
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "pDM_Odm->RFCalibrateInfo.OFDM_index[RF90_PATH_A]=%d, pDM_Odm->RealBbSwingIdx[RF90_PATH_A]=%d\n",
				rtldm->ofdm_index[RF90_PATH_A], finalBbSwingIdx[RF90_PATH_A]);
			rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[finalBbSwingIdx[RF90_PATH_A]]);
		}
	} else if (Method == MIX_MODE) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "pDM_Odm->DefaultOfdmIndex=%d, pDM_Odm->Aboslute_OFDMSwingIdx[RFPath]=%d, RF_Path = %d\n",
				rtldm->default_ofdm_index, rtldm->absolute_ofdm_swing_idx[RFPath], RFPath);

			Final_OFDM_Swing_Index = rtldm->default_ofdm_index + rtldm->absolute_ofdm_swing_idx[RFPath];

			if (RFPath == RF90_PATH_A) {
				if (Final_OFDM_Swing_Index > PwrTrackingLimit) {
					/* BBSwing higher then Limit */
					rtldm->remnant_cck_idx = Final_OFDM_Swing_Index - PwrTrackingLimit;            /* CCK Follow the same compensate value as Path A */
					rtldm->remnant_ofdm_swing_idx[RFPath] = Final_OFDM_Swing_Index - PwrTrackingLimit;

					rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[PwrTrackingLimit]);

					rtldm->modify_txagc_flag_path_a = true;

					/* Set TxAGC Page C{}; */
					/* rtlpriv->cfg->ops.SetTxPowerLevelHandler(rtlpriv, pHalData->CurrentChannel); */
					PHY_SetTxPowerLevel8812(rtlpriv, rtlpriv->phy.current_channel);
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A Over BBSwing Limit , PwrTrackingLimit = %d , Remnant TxAGC Value = %d \n", PwrTrackingLimit, rtldm->remnant_ofdm_swing_idx[RFPath]);
				} else if (Final_OFDM_Swing_Index < 0) {
					rtldm->remnant_cck_idx = Final_OFDM_Swing_Index;            /* CCK Follow the same compensate value as Path A */
					rtldm->remnant_ofdm_swing_idx[RFPath] = Final_OFDM_Swing_Index;

					rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[0]);

					rtldm->modify_txagc_flag_path_a = true;

					/* Set TxAGC Page C{}; */
					/* rtlpriv->cfg->ops.SetTxPowerLevelHandler(rtlpriv, pHalData->CurrentChannel); */
					PHY_SetTxPowerLevel8812(rtlpriv, rtlpriv->phy.current_channel);
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A Lower then BBSwing lower bound  0 , Remnant TxAGC Value = %d \n", rtldm->remnant_ofdm_swing_idx[RFPath]);
				} else {
					rtl_set_bbreg(rtlpriv, rA_TxScale_Jaguar, 0xFFE00000, txscalling_tbl[Final_OFDM_Swing_Index]);
					RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A Compensate with BBSwing , Final_OFDM_Swing_Index = %d \n", Final_OFDM_Swing_Index);
					if (rtldm->modify_txagc_flag_path_a) {
						/* If TxAGC has changed, reset TxAGC again */
						rtldm->remnant_cck_idx = 0;
						rtldm->remnant_ofdm_swing_idx[RFPath] = 0;

						/* Set TxAGC Page C{}; */
						/* rtlpriv->cfg->ops.SetTxPowerLevelHandler(rtlpriv, pHalData->CurrentChannel); */
						PHY_SetTxPowerLevel8812(rtlpriv, rtlpriv->phy.current_channel);

						rtldm->modify_txagc_flag_path_a = false;
						RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Path_A pDM_Odm->Modify_TxAGC_Flag = false \n");
					}
				}
			}

	} else {
		return;
	}
}
/* START Copied from hal/OUTSRC/rtl8821a/HalHWImg8821A_RF.c */

/******************************************************************************
*                           TxPowerTrack_USB.TXT
******************************************************************************/

static u8 rtl8821au_delta_swing_table_idx_5gb_n[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 14},
	{0, 1, 1, 2, 3, 3, 4, 5, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 6,  7,  7,  8,  8,  9,  9, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8821au_delta_swing_table_idx_5gb_p[][DELTA_SWINGIDX_SIZE] = {
	{0, 1, 1, 2, 2, 3, 3, 4, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 5,  6,  7,  7,  8,  8,  9,  9,
	10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
	{0, 1, 1, 2, 3, 3, 4, 5, 6,  7,  7,  8,  8,  9,  9, 10,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11},
};

static u8 rtl8821au_delta_swing_table_idx_5ga_n[][DELTA_SWINGIDX_SIZE] = {
	{0, 0, 0, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,
	9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 15, 15},
	{0, 0, 0, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,
	9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 15, 15},
	{0, 0, 0, 1, 2, 2, 3, 4, 5, 6,  6,  7,  7,  8,  8,  9,
	9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 15, 15},
};

static u8 rtl8821au_delta_swing_table_idx_5ga_p[][DELTA_SWINGIDX_SIZE] = {
	{1, 2, 3, 4, 5, 6, 7, 8, 9,  10, 11, 12,  13,  14, 15, 16,
	16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16},
	{1, 2, 3, 4, 5, 6, 7, 8, 9,  10, 11, 12,  13,  14, 15, 16,
	16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16},
	{1, 2, 3, 4, 5, 6, 7, 8, 9,  10, 11, 12,  13,  14, 15, 16,
	16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16},
};

static u8 rtl8821au_delta_swing_table_idx_24gb_n[]    = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  5,  5,  5,  6,  6,
	7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 11, 11, 11, 11
};

static u8 rtl8821au_delta_swing_table_idx_24gb_p[]    = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};

static u8 rtl8821au_delta_swing_table_idx_24ga_n[]    = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  8,  8,  9, 10, 10, 10, 10, 10, 10
};

static u8 rtl8821au_delta_swing_table_idx_24ga_p[]    = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};

static u8 rtl8821au_delta_swing_table_idx_24gcckb_n[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  5,  5,  5,  6,  6,
	7,  7,  8,  8,  9,  9, 10, 10, 11, 11, 11, 11, 11, 11
};

static u8 rtl8821au_delta_swing_table_idx_24gcckb_p[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};

static u8 rtl8821au_delta_swing_table_idx_24gccka_n[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  8,  8,  9, 10, 10, 10, 10, 10, 10
};

static u8 rtl8821au_delta_swing_table_idx_24gccka_p[] = {
	0, 1, 1, 2, 2, 2, 3, 3, 3,  4,  4,  4,  5,  5,  5,  6,
	6,  6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
};


/* END Copied from hal/OUTSRC/rtl8821a/HalHWImg8821A_RF.c */

void rtl8821au_get_delta_swing_table(struct rtl_priv *rtlpriv,
	u8 **up_a, u8 **down_a,
	u8 **up_b, u8 **down_b)
{
	/* u16     rate = pMgntInfo->ForcedDataRate; */
	u16	rate = 0;
	u8         	channel   		 = rtlpriv->phy.current_channel;

	if (1 <= channel && channel <= 14) {
		if (IS_CCK_RATE(rate)) {
			*up_a   = rtl8821au_delta_swing_table_idx_24gccka_p;
			*down_a = rtl8821au_delta_swing_table_idx_24gccka_n;
			*up_b   = rtl8821au_delta_swing_table_idx_24gcckb_p;
			*down_b = rtl8821au_delta_swing_table_idx_24gcckb_n;
		} else {
			*up_a   = rtl8821au_delta_swing_table_idx_24ga_p;
			*down_a = rtl8821au_delta_swing_table_idx_24ga_n;
			*up_b   = rtl8821au_delta_swing_table_idx_24gb_p;
			*down_b = rtl8821au_delta_swing_table_idx_24gb_n;
		}
	} else if (36 <= channel && channel <= 64) {
		*up_a   = rtl8821au_delta_swing_table_idx_5ga_p[0];
		*down_a = rtl8821au_delta_swing_table_idx_5ga_n[0];
		*up_b   = rtl8821au_delta_swing_table_idx_5gb_p[0];
		*down_b = rtl8821au_delta_swing_table_idx_5gb_n[0];
	} else if (100 <= channel && channel <= 140) {
		*up_a   = rtl8821au_delta_swing_table_idx_5ga_p[1];
		*down_a = rtl8821au_delta_swing_table_idx_5ga_n[1];
		*up_b   = rtl8821au_delta_swing_table_idx_5gb_p[1];
		*down_b = rtl8821au_delta_swing_table_idx_5gb_n[1];
	} else if (149 <= channel && channel <= 173) {
		*up_a   = rtl8821au_delta_swing_table_idx_5ga_p[2];
		*down_a = rtl8821au_delta_swing_table_idx_5ga_n[2];
		*up_b   = rtl8821au_delta_swing_table_idx_5gb_p[2];
		*down_b = rtl8821au_delta_swing_table_idx_5gb_n[2];
	} else {
		*up_a   = (u8 *)rtl8818e_delta_swing_table_idx_24gb_p;
		*down_a = (u8 *)rtl8818e_delta_swing_table_idx_24gb_n;
		*up_b   = (u8 *)rtl8818e_delta_swing_table_idx_24gb_p;
		*down_b = (u8 *)rtl8818e_delta_swing_table_idx_24gb_n;
	}

	return;
}

static void rtl8812au_phy_lc_calibrate(struct rtl_priv *rtlpriv)
{
	uint32_t	/*RF_Amode=0, RF_Bmode=0,*/ LC_Cal = 0, tmp = 0;

	/* Check continuous TX and Packet TX */
	uint32_t	reg0x914 = rtl_read_dword(rtlpriv, rSingleTone_ContTx_Jaguar);;

	/* Backup RF reg18. */
	LC_Cal = rtl_get_rfreg(rtlpriv, RF90_PATH_A, RF_CHNLBW, bRFRegOffsetMask);

	if ((reg0x914 & 0x70000) != 0)	/* If contTx, disable all continuous TX. 0x914[18:16] */
		/*
		 *  <20121121, Kordan> A workaround: If we set 0x914[18:16] as zero, BB turns off ContTx
		 *  until another packet comes in. To avoid ContTx being turned off, we skip this step.
		 * ODM_Write4Byte(pDM_Odm, rSingleTone_ContTx_Jaguar, reg0x914 & (~0x70000));
		 */

		;
	else		/* If packet Tx-ing, pause Tx. */
		rtl_write_byte(rtlpriv, REG_TXPAUSE, 0xFF);


/*
	//3 1. Read original RF mode
	RF_Amode = rtl_get_rfreg(pDM_Odm->rtlpriv, RF90_PATH_A, RF_AC, bRFRegOffsetMask);
	if(is2T)
		RF_Bmode = rtl_get_rfreg(pDM_Odm->rtlpriv, RF90_PATH_B, RF_AC, bRFRegOffsetMask);


	//3 2. Set RF mode = standby mode
	rtl_set_rfreg(pDM_Odm->rtlpriv, RF90_PATH_A, RF_AC, bRFRegOffsetMask, (RF_Amode&0x8FFFF)|0x10000);
	if(is2T)
		rtl_set_rfreg(pDM_Odm->rtlpriv, RF90_PATH_B, RF_AC, bRFRegOffsetMask, (RF_Bmode&0x8FFFF)|0x10000);
*/

	/* Enter LCK mode */
	tmp = rtl_get_rfreg(rtlpriv, RF90_PATH_A, RF_LCK, bRFRegOffsetMask);
	rtl_set_rfreg(rtlpriv, RF90_PATH_A, RF_LCK, bRFRegOffsetMask, tmp | BIT(14));

	/* 3 3. Read RF reg18 */
	LC_Cal = rtl_get_rfreg(rtlpriv, RF90_PATH_A, RF_CHNLBW, bRFRegOffsetMask);

	/* 3 4. Set LC calibration begin BIT(15) */
	rtl_set_rfreg(rtlpriv, RF90_PATH_A, RF_CHNLBW, bRFRegOffsetMask, LC_Cal|0x08000);

	/* Leave LCK mode */
	tmp = rtl_get_rfreg(rtlpriv, RF90_PATH_A, RF_LCK, bRFRegOffsetMask);
	rtl_set_rfreg(rtlpriv, RF90_PATH_A, RF_LCK, bRFRegOffsetMask, tmp & ~BIT(14));

	mdelay(100);

	/* 3 Restore original situation */
	if ((reg0x914 & 70000) != 0) {	/* Deal with contisuous TX case, 0x914[18:16] */
		/*
		 * <20121121, Kordan> A workaround: If we set 0x914[18:16] as zero, BB turns off ContTx
		 * until another packet comes in. To avoid ContTx being turned off, we skip this step.
		 * ODM_Write4Byte(pDM_Odm, rSingleTone_ContTx_Jaguar, reg0x914);
		 */
		;
	} else {
		/* Deal with Packet TX case */
		rtl_write_byte(rtlpriv, REG_TXPAUSE, 0x00);
	}

	/* Recover channel number */
	rtl_set_rfreg(rtlpriv, RF90_PATH_A, RF_CHNLBW, bRFRegOffsetMask, LC_Cal);

	/*
	rtl_set_rfreg(pDM_Odm->rtlpriv, RF90_PATH_A, RF_AC, bRFRegOffsetMask, RF_Amode);
	if(is2T)
		rtl_set_rfreg(pDM_Odm->rtlpriv, RF90_PATH_B, RF_AC, bRFRegOffsetMask, RF_Bmode);
		*/
}

static void rtl8812au_dm_txpower_tracking_callback_thermalmeter(struct rtl_priv *rtlpriv)
{
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);
	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);

	u8 thermal_value = 0, delta, delta_lck, delta_iqk, p = 0, i = 0;
	u8 thermal_value_avg_count = 0;
	u32 thermal_value_avg = 0;

	u8 ofdm_min_index = 0;		/* OFDM BB Swing should be less than +if (.0dB, which is required by Arthur */
	u8 index_for_channel = 0;	/* GetRightChnlPlaceforIQK(pHalData->CurrentChannel) */

	/* 4 1. The following TWO tables decide the final index of OFDM/CCK swing table. */
	u8 *up_a, *down_a, *up_b, *down_b;

	/* 4 2. Initilization ( 7 steps in total ) */

	rtl8812au_get_delta_swing_table(rtlpriv, (u8 **)&up_a, (u8 **)&down_a,
					 (u8 **)&up_b, (u8 **)&down_b);

#if 0		/* ULLI : only writing, no use */
#endif
	rtldm->txpower_trackinginit = true;

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD,
		"===>ODM_TXPowerTrackingCallback_ThermalMeter, \n rtldm->BbSwingIdxCckBase: %d, rtldm->BbSwingIdxOfdmBase[A]: %d, rtldm->DefaultOfdmIndex: %d\n",
		rtldm->swing_idx_cck_base, rtldm->swing_idx_ofdm_base[RF90_PATH_A],
		rtldm->default_ofdm_index);

	thermal_value = (u8)rtl_get_rfreg(rtlpriv, RF90_PATH_A, RF_T_METER_8812A, 0xfc00);	/* 0x42: RF Reg[15:10] 88E */
	if (!rtldm->txpower_track_control
	 || efuse->eeprom_thermalmeter == 0
	 || efuse->eeprom_thermalmeter == 0xFF)
		return;

	/* 4 if (. Initialize ThermalValues of RFCalibrateInfo */

	/* 4 4. Calculate average thermal meter */

	rtldm->thermalvalue_avg[rtldm->thermalvalue_avg_index] = thermal_value;
	rtldm->thermalvalue_avg_index++;
	if (rtldm->thermalvalue_avg_index == AVG_THERMAL_NUM_8812A)   /* Average times =  c.AverageThermalNum */
		rtldm->thermalvalue_avg_index = 0;

	for (i = 0; i < AVG_THERMAL_NUM_8812A; i++) {
		if (rtldm->thermalvalue_avg[i]) {
			thermal_value_avg += rtldm->thermalvalue_avg[i];
			thermal_value_avg_count++;
		}
	}

	if (thermal_value_avg_count) {               /* Calculate Average ThermalValue after average enough times */
		thermal_value = (u8)(thermal_value_avg / thermal_value_avg_count);
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "AVG Thermal Meter = 0x%X, EFUSE Thermal Base = 0x%X\n", thermal_value, efuse->eeprom_thermalmeter);
	}

	/* 4 5. Calculate delta, delta_LCK, delta_IQK. */

	/* "delta" here is used to determine whether thermal value changes or not. */
	delta 	  = (thermal_value > rtldm->thermalvalue)?(thermal_value - rtldm->thermalvalue):(rtldm->thermalvalue - thermal_value);
	delta_lck = (thermal_value > rtldm->thermalvalue_lck)?(thermal_value - rtldm->thermalvalue_lck):(rtldm->thermalvalue_lck - thermal_value);
	delta_iqk = (thermal_value > rtldm->thermalvalue_iqk)?(thermal_value - rtldm->thermalvalue_iqk):(rtldm->thermalvalue_iqk - thermal_value);

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "(delta, delta_LCK, delta_IQK) = (%d, %d, %d)\n", delta, delta_lck, delta_iqk);

	/* 4 6. If necessary, do LCK. */

	if ((delta_lck >= IQK_THRESHOLD)) {	/* Delta temperature is equal to or larger than 20 centigrade. */
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "delta_LCK(%d) >= Threshold_IQK(%d)\n", delta_lck, IQK_THRESHOLD);
		rtldm->thermalvalue_lck = thermal_value;
		rtl8812au_phy_lc_calibrate(rtlpriv);
	}

	/* if ( 7. If necessary, move the index of swing table to adjust Tx power. */

	if (delta > 0 && rtldm->txpower_track_control) {
		/* "delta" here is used to record the absolute value of differrence. */
	    delta = thermal_value > efuse->eeprom_thermalmeter?(thermal_value - efuse->eeprom_thermalmeter):(efuse->eeprom_thermalmeter - thermal_value);
		if (delta >= TXSCALE_TABLE_SIZE)
			delta = TXSCALE_TABLE_SIZE - 1;

		/* 4 7.1 The Final Power Index = BaseIndex + PowerIndexOffset */

		if (thermal_value > efuse->eeprom_thermalmeter) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "deltaSwingTableIdx_TUP_A[%d] = %d\n", delta, up_a[delta]);
			rtldm->delta_power_index_last[RF90_PATH_A] = rtldm->delta_power_index[RF90_PATH_A];
			rtldm->delta_power_index[RF90_PATH_A] = up_a[delta];

			rtldm->absolute_ofdm_swing_idx[RF90_PATH_A] =  up_a[delta];        /* Record delta swing for mix mode power tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Temp is higher and rtldm->Aboslute_OFDMSwingIdx[RF90_PATH_A] = %d\n", rtldm->absolute_ofdm_swing_idx[RF90_PATH_A]);

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "deltaSwingTableIdx_TUP_B[%d] = %d\n", delta, up_b[delta]);
			rtldm->delta_power_index_last[RF90_PATH_B] = rtldm->delta_power_index[RF90_PATH_B];
			rtldm->delta_power_index[RF90_PATH_B] = up_b[delta];

			rtldm->absolute_ofdm_swing_idx[RF90_PATH_B] =  up_a[delta];       /* Record delta swing for mix mode power tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Temp is higher and rtldm->Aboslute_OFDMSwingIdx[RF90_PATH_B] = %d\n", rtldm->absolute_ofdm_swing_idx[RF90_PATH_B]);

		} else {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "deltaSwingTableIdx_TDOWN_A[%d] = %d\n", delta, down_a[delta]);

			rtldm->delta_power_index_last[RF90_PATH_A] = rtldm->delta_power_index[RF90_PATH_A];
			rtldm->delta_power_index[RF90_PATH_A] = -1 * down_a[delta];

			rtldm->absolute_ofdm_swing_idx[RF90_PATH_A] =  -1 * down_a[delta];        /* Record delta swing for mix mode power tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Temp is lower and rtldm->Aboslute_OFDMSwingIdx[RF90_PATH_A] = %d\n", rtldm->absolute_ofdm_swing_idx[RF90_PATH_A]);

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "deltaSwingTableIdx_TDOWN_B[%d] = %d\n", delta, down_b[delta]);

			rtldm->delta_power_index_last[RF90_PATH_B] = rtldm->delta_power_index[RF90_PATH_B];
			rtldm->delta_power_index[RF90_PATH_B] = -1 * down_b[delta];

			rtldm->absolute_ofdm_swing_idx[RF90_PATH_B] =  -1 * down_b[delta];       /* Record delta swing for mix mode power tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Temp is lower and rtldm->Aboslute_OFDMSwingIdx[RF90_PATH_B] = %d\n", rtldm->absolute_ofdm_swing_idx[RF90_PATH_B]);
		}

	    for (p = RF90_PATH_A; p < MAX_PATH_NUM_8812A; p++) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "\n\n================================ [Path-%c] Calculating PowerIndexOffset ================================\n", (p == RF90_PATH_A ? 'A' : 'B'));
			if (rtldm->delta_power_index[p] == rtldm->delta_power_index_last[p])         /* If Thermal value changes but lookup table value still the same */
				rtldm->power_index_offset[p] = 0;
			else
				rtldm->power_index_offset[p] = rtldm->delta_power_index[p] - rtldm->delta_power_index_last[p];      /* Power Index Diff between 2 times Power Tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "[Path-%c] PowerIndexOffset(%d) = DeltaPowerIndex(%d) - DeltaPowerIndexLast(%d)\n", (p == RF90_PATH_A ? 'A' : 'B'), rtldm->power_index_offset[p], rtldm->delta_power_index[p],
			rtldm->delta_power_index_last[p]);

			rtldm->ofdm_index[p] = rtldm->swing_idx_ofdm_base[p] + rtldm->power_index_offset[p];
			rtldm->cck_index = rtldm->swing_idx_cck_base + rtldm->power_index_offset[p];

			rtldm->swing_idx_cck = rtldm->cck_index;
			rtldm->swing_idx_ofdm[p] = rtldm->ofdm_index[p];

	       /* *************Print BB Swing Base and Index Offset************* */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "The 'CCK' final index(%d) = BaseIndex(%d) + PowerIndexOffset(%d)\n",  rtldm->swing_idx_cck, rtldm->swing_idx_cck_base, rtldm->power_index_offset[p]);
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "The 'OFDM' final index(%d) = BaseIndex[%c](%d) + PowerIndexOffset(%d)\n", rtldm->swing_idx_ofdm[p], (p == RF90_PATH_A ? 'A' : 'B'), rtldm->swing_idx_ofdm_base[p], rtldm->power_index_offset[p]);

		    /* 4 7.1 Handle boundary conditions of index. */

			if (rtldm->ofdm_index[p] > TXSCALE_TABLE_SIZE-1) {
				rtldm->ofdm_index[p] = TXSCALE_TABLE_SIZE-1;
			} else if (rtldm->ofdm_index[p] < ofdm_min_index) {
				rtldm->ofdm_index[p] = ofdm_min_index;
			}
		}
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "\n\n========================================================================================================\n");
		if (rtldm->cck_index > TXSCALE_TABLE_SIZE-1)
			rtldm->cck_index = TXSCALE_TABLE_SIZE-1;
	} else {
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "The thermal meter is unchanged or TxPowerTracking OFF(%d): ThermalValue: %d , rtldm->RFCalibrateInfo.ThermalValue: %d\n", rtldm->txpower_track_control, thermal_value, rtldm->thermalvalue);

		for (p = RF90_PATH_A; p < MAX_PATH_NUM_8812A; p++)
			rtldm->power_index_offset[p] = 0;
	}
	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "TxPowerTracking: [CCK] Swing Current Index: %d, Swing Base Index: %d\n", rtldm->cck_index, rtldm->swing_idx_cck_base);       /* Print Swing base & current */
	for (p = RF90_PATH_A; p < MAX_PATH_NUM_8812A; p++) {
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "TxPowerTracking: [OFDM] Swing Current Index: %d, Swing Base Index[%c]: %d\n", rtldm->ofdm_index[p], (p == RF90_PATH_A ? 'A' : 'B'), rtldm->swing_idx_ofdm_base[p]);
	}

	if ((rtldm->power_index_offset[RF90_PATH_A] != 0 ||  rtldm->power_index_offset[RF90_PATH_B] != 0)
	 && rtldm->txpower_track_control) {
		/* 4 7.2 Configure the Swing Table to adjust Tx Power. */
#if 0		/* ULLI : only writing, no use */
		rtldm->bTxPowerChanged = true; /* Always true after Tx Power is adjusted by power tracking. */
#endif
		/*
		 *  2012/04/2if ( MH According to Luke's suggestion, we can not write BB digital
		 *  to increase TX power. Otherwise, EVM will be bad.
		 *
		 *  2012/04/25 MH Add for tx power tracking to set tx power in tx agc for 88E.
		 */
		if (thermal_value > rtldm->thermalvalue) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature Increasing(A): delta_pi: %d , delta_t: %d, Now_t: %d, EFUSE_t: %d, Last_t: %d\n", rtldm->power_index_offset[RF90_PATH_A], delta, thermal_value, efuse->eeprom_thermalmeter, rtldm->thermalvalue);
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature Increasing(B): delta_pi: %d , delta_t: %d, Now_t: %d, EFUSE_t: %d, Last_t: %d\n", rtldm->power_index_offset[RF90_PATH_B], delta, thermal_value, efuse->eeprom_thermalmeter, rtldm->thermalvalue);

			} else if (thermal_value < rtldm->thermalvalue) { /* Low temperature */
				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature Decreasing(A): delta_pi: %d , delta_t: %d, Now_t: %d, EFUSE_t: %d, Last_t: %d\n", rtldm->power_index_offset[RF90_PATH_A], delta, thermal_value, efuse->eeprom_thermalmeter, rtldm->thermalvalue);
				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature Decreasing(B): delta_pi: %d , delta_t: %d, Now_t: %d, EFUSE_t: %d, Last_t: %d\n", rtldm->power_index_offset[RF90_PATH_B], delta, thermal_value, efuse->eeprom_thermalmeter, rtldm->thermalvalue);

			}
			if (thermal_value > efuse->eeprom_thermalmeter) {
				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature(%d) higher than PG value(%d)\n", thermal_value, efuse->eeprom_thermalmeter);

				for (p = RF90_PATH_A; p < MAX_PATH_NUM_8812A; p++)
					rtl8812au_dm_pxpwr_track_set_pwr(rtlpriv, BBSWING, p, index_for_channel);
			} else {
				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature(%d) lower than PG value(%d)\n", thermal_value, efuse->eeprom_thermalmeter);

				for (p = RF90_PATH_A; p < MAX_PATH_NUM_8812A; p++)
					rtl8812au_dm_pxpwr_track_set_pwr(rtlpriv, BBSWING, p, index_for_channel);
			}

			rtldm->swing_idx_cck_base = rtldm->swing_idx_cck;  	/* Record last time Power Tracking result as base. */
			for (p = RF90_PATH_A; p < MAX_PATH_NUM_8812A; p++)	/* ULLI huh ?? */
				rtldm->swing_idx_ofdm_base[p] = rtldm->swing_idx_ofdm[p];

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD,  "rtldm->RFCalibrateInfo.ThermalValue = %d ThermalValue= %d\n", rtldm->thermalvalue, thermal_value);

			rtldm->thermalvalue = thermal_value;     /* Record last Power Tracking Thermal Value */

	}
	if ((delta_iqk >= IQK_THRESHOLD))	/* Delta temperature is equal to or larger than 20 centigrade (When threshold is 8). */
		rtl8812au_do_iqk(rtlpriv, delta_iqk, thermal_value, 8);

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "<===ODM_TXPowerTrackingCallback_ThermalMeter\n");

	rtldm->txpowercount = 0;
}


static void rtl8821au_dm_txpower_tracking_callback_thermalmeter(struct rtl_priv *rtlpriv)
{
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);
	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);

	u8 thermal_value = 0, delta, delta_lck, delta_iqk, p = 0, i = 0;
	u8 thermal_value_avg_count = 0;
	u32 thermal_value_avg = 0;

	u8 ofdm_min_index = 0;  /* OFDM BB Swing should be less than +if (.0dB, which is required by Arthur */
	u8 index_for_channel = 0;	/* GetRightChnlPlaceforIQK(pHalData->CurrentChannel) */

	/* 4 1. The following TWO tables decide the final index of OFDM/CCK swing table. */
	u8 *up_a, *down_a, *up_b, *down_b;

	/* 4 2. Initilization ( 7 steps in total ) */

	rtl8821au_get_delta_swing_table(rtlpriv, (u8 **)&up_a, (u8 **)&down_a,
					 (u8 **)&up_b, (u8 **)&down_b);

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "===>ODM_TXPowerTrackingCallback_ThermalMeter, \n rtldm->BbSwingIdxCckBase: %d, rtldm->BbSwingIdxOfdmBase[A]: %d, rtldm->DefaultOfdmIndex: %d\n", rtldm->swing_idx_cck_base, rtldm->swing_idx_ofdm_base[RF90_PATH_A], rtldm->default_ofdm_index);

	thermal_value = (u8)rtl_get_rfreg(rtlpriv, RF90_PATH_A, RF_T_METER_8812A, 0xfc00);	/* 0x42: RF Reg[15:10] 88E */
	if (!rtldm->txpower_track_control
	 || efuse->eeprom_thermalmeter == 0
	 || efuse->eeprom_thermalmeter == 0xFF)
		return;

	/* 4 if (. Initialize ThermalValues of RFCalibrateInfo */

	/* 4 4. Calculate average thermal meter */

	rtldm->thermalvalue_avg[rtldm->thermalvalue_avg_index] = thermal_value;
	rtldm->thermalvalue_avg_index++;
	if (rtldm->thermalvalue_avg_index == AVG_THERMAL_NUM_8812A)   /* Average times =  c.AverageThermalNum */
		rtldm->thermalvalue_avg_index = 0;

	for (i = 0; i < AVG_THERMAL_NUM_8812A; i++) {
		if (rtldm->thermalvalue_avg[i]) {
			thermal_value_avg += rtldm->thermalvalue_avg[i];
			thermal_value_avg_count++;
		}
	}

	if (thermal_value_avg_count) {               /* Calculate Average ThermalValue after average enough times */
		thermal_value = (u8)(thermal_value_avg / thermal_value_avg_count);
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "AVG Thermal Meter = 0x%X, EFUSE Thermal Base = 0x%X\n", thermal_value, efuse->eeprom_thermalmeter);
	}

	/* 4 5. Calculate delta, delta_LCK, delta_IQK. */

	/* "delta" here is used to determine whether thermal value changes or not. */
	delta 	  = (thermal_value > rtldm->thermalvalue)?(thermal_value - rtldm->thermalvalue):(rtldm->thermalvalue - thermal_value);
	delta_lck = (thermal_value > rtldm->thermalvalue_lck)?(thermal_value - rtldm->thermalvalue_lck):(rtldm->thermalvalue_lck - thermal_value);
	delta_iqk = (thermal_value > rtldm->thermalvalue_iqk)?(thermal_value - rtldm->thermalvalue_iqk):(rtldm->thermalvalue_iqk - thermal_value);

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "(delta, delta_LCK, delta_IQK) = (%d, %d, %d)\n", delta, delta_lck, delta_iqk);

	/* 4 6. If necessary, do LCK. */

	if ((delta_lck >= IQK_THRESHOLD)) {	/* Delta temperature is equal to or larger than 20 centigrade. */
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "delta_LCK(%d) >= Threshold_IQK(%d)\n", delta_lck, IQK_THRESHOLD);
		rtldm->thermalvalue_lck = thermal_value;
		rtl8812au_phy_lc_calibrate(rtlpriv);
	}

	/* if ( 7. If necessary, move the index of swing table to adjust Tx power. */

	if (delta > 0 && rtldm->txpower_track_control) {
		/* "delta" here is used to record the absolute value of differrence. */
	    delta = thermal_value > efuse->eeprom_thermalmeter?(thermal_value - efuse->eeprom_thermalmeter):(efuse->eeprom_thermalmeter - thermal_value);
		if (delta >= TXSCALE_TABLE_SIZE)
			delta = TXSCALE_TABLE_SIZE - 1;

		/* 4 7.1 The Final Power Index = BaseIndex + PowerIndexOffset */

		if (thermal_value > efuse->eeprom_thermalmeter) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "deltaSwingTableIdx_TUP_A[%d] = %d\n", delta, up_a[delta]);
			rtldm->delta_power_index_last[RF90_PATH_A] = rtldm->delta_power_index[RF90_PATH_A];
			rtldm->delta_power_index[RF90_PATH_A] = up_a[delta];

			rtldm->absolute_ofdm_swing_idx[RF90_PATH_A] =  up_a[delta];        /* Record delta swing for mix mode power tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Temp is higher and rtldm->Aboslute_OFDMSwingIdx[RF90_PATH_A] = %d\n", rtldm->absolute_ofdm_swing_idx[RF90_PATH_A]);
		} else {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "deltaSwingTableIdx_TDOWN_A[%d] = %d\n", delta, down_a[delta]);

			rtldm->delta_power_index_last[RF90_PATH_A] = rtldm->delta_power_index[RF90_PATH_A];
			rtldm->delta_power_index[RF90_PATH_A] = -1 * down_a[delta];

			rtldm->absolute_ofdm_swing_idx[RF90_PATH_A] =  -1 * down_a[delta];        /* Record delta swing for mix mode power tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "******Temp is lower and rtldm->Aboslute_OFDMSwingIdx[RF90_PATH_A] = %d\n", rtldm->absolute_ofdm_swing_idx[RF90_PATH_A]);
		}

	    for (p = RF90_PATH_A; p < MAX_PATH_NUM_8821A; p++) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "\n\n================================ [Path-%c] Calculating PowerIndexOffset ================================\n", (p == RF90_PATH_A ? 'A' : 'B'));
			if (rtldm->delta_power_index[p] == rtldm->delta_power_index_last[p])         /* If Thermal value changes but lookup table value still the same */
				rtldm->power_index_offset[p] = 0;
			else
				rtldm->power_index_offset[p] = rtldm->delta_power_index[p] - rtldm->delta_power_index_last[p];      /* Power Index Diff between 2 times Power Tracking */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "[Path-%c] PowerIndexOffset(%d) = DeltaPowerIndex(%d) - DeltaPowerIndexLast(%d)\n", (p == RF90_PATH_A ? 'A' : 'B'), rtldm->power_index_offset[p], rtldm->delta_power_index[p], rtldm->delta_power_index_last[p]);

			rtldm->ofdm_index[p] = rtldm->swing_idx_ofdm_base[p] + rtldm->power_index_offset[p];
			rtldm->cck_index = rtldm->swing_idx_cck_base + rtldm->power_index_offset[p];

			rtldm->swing_idx_cck = rtldm->cck_index;
			rtldm->swing_idx_ofdm[p] = rtldm->ofdm_index[p];

	       /* *************Print BB Swing Base and Index Offset************* */

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "The 'CCK' final index(%d) = BaseIndex(%d) + PowerIndexOffset(%d)\n", rtldm->swing_idx_cck, rtldm->swing_idx_cck_base, rtldm->power_index_offset[p]);
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "The 'OFDM' final index(%d) = BaseIndex[%c](%d) + PowerIndexOffset(%d)\n", rtldm->swing_idx_ofdm[p], (p == RF90_PATH_A ? 'A' : 'B'), rtldm->swing_idx_ofdm_base[p], rtldm->power_index_offset[p]);

		    /* 4 7.1 Handle boundary conditions of index. */

			if (rtldm->ofdm_index[p] > TXSCALE_TABLE_SIZE-1) {
				rtldm->ofdm_index[p] = TXSCALE_TABLE_SIZE-1;
			} else if (rtldm->ofdm_index[p] < ofdm_min_index) {
				rtldm->ofdm_index[p] = ofdm_min_index;
			}
		}
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "\n\n========================================================================================================\n");
		if (rtldm->cck_index > TXSCALE_TABLE_SIZE-1)
			rtldm->cck_index = TXSCALE_TABLE_SIZE-1;
	} else {
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "The thermal meter is unchanged or TxPowerTracking OFF(%d): ThermalValue: %d , rtldm->RFCalibrateInfo.ThermalValue: %d\n", rtldm->txpower_track_control, thermal_value, rtldm->thermalvalue);

		for (p = RF90_PATH_A; p < MAX_PATH_NUM_8821A; p++)
			rtldm->power_index_offset[p] = 0;
	}
	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "TxPowerTracking: [CCK] Swing Current Index: %d, Swing Base Index: %d\n", rtldm->cck_index, rtldm->swing_idx_cck_base);       /* Print Swing base & current */
	for (p = RF90_PATH_A; p < MAX_PATH_NUM_8821A; p++) {
		RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "TxPowerTracking: [OFDM] Swing Current Index: %d, Swing Base Index[%c]: %d\n", rtldm->ofdm_index[p], (p == RF90_PATH_A ? 'A' : 'B'), rtldm->swing_idx_ofdm_base[p]);
	}

	if ((rtldm->power_index_offset[RF90_PATH_A] != 0 ||  rtldm->power_index_offset[RF90_PATH_B] != 0)
	 && rtldm->txpower_track_control) {
		/* 4 7.2 Configure the Swing Table to adjust Tx Power. */
#if 0		/* ULLI : only writing, no use */
		rtldm->bTxPowerChanged = true; /* Always true after Tx Power is adjusted by power tracking. */
#endif
		/*
		 *  2012/04/2if ( MH According to Luke's suggestion, we can not write BB digital
		 *  to increase TX power. Otherwise, EVM will be bad.
		 *
		 *  2012/04/25 MH Add for tx power tracking to set tx power in tx agc for 88E.
		 */
		if (thermal_value > rtldm->thermalvalue) {
			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature Increasing(A): delta_pi: %d , delta_t: %d, Now_t: %d, EFUSE_t: %d, Last_t: %d\n", rtldm->power_index_offset[RF90_PATH_A], delta, thermal_value, efuse->eeprom_thermalmeter, rtldm->thermalvalue);

			} else if (thermal_value < rtldm->thermalvalue) { /* Low temperature */
				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature Decreasing(A): delta_pi: %d , delta_t: %d, Now_t: %d, EFUSE_t: %d, Last_t: %d\n", rtldm->power_index_offset[RF90_PATH_A], delta, thermal_value, efuse->eeprom_thermalmeter, rtldm->thermalvalue);

			}
			if (thermal_value > efuse->eeprom_thermalmeter) {
				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature(%d) higher than PG value(%d)\n", thermal_value, efuse->eeprom_thermalmeter);

				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "**********Enter POWER Tracking MIX_MODE**********\n");
				for (p = RF90_PATH_A; p < MAX_PATH_NUM_8821A; p++)
					rtl8821au_dm_txpwr_track_set_pwr(rtlpriv, MIX_MODE, p, index_for_channel);
			} else {
				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "Temperature(%d) lower than PG value(%d)\n", thermal_value, efuse->eeprom_thermalmeter);

				RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "**********Enter POWER Tracking MIX_MODE**********\n");
				for (p = RF90_PATH_A; p < MAX_PATH_NUM_8821A; p++)
					rtl8821au_dm_txpwr_track_set_pwr(rtlpriv, MIX_MODE, p, index_for_channel);
			}

			rtldm->swing_idx_cck_base = rtldm->swing_idx_cck;  	/* Record last time Power Tracking result as base. */
			for (p = RF90_PATH_A; p < MAX_PATH_NUM_8821A; p++)
				rtldm->swing_idx_ofdm_base[p] = rtldm->swing_idx_ofdm[p];

			RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD,  "rtldm->RFCalibrateInfo.ThermalValue = %d ThermalValue= %d\n", rtldm->thermalvalue, thermal_value);

			rtldm->thermalvalue = thermal_value;     /* Record last Power Tracking Thermal Value */

	}
	if ((delta_iqk >= IQK_THRESHOLD))	/* Delta temperature is equal to or larger than 20 centigrade (When threshold is 8). */
		rtl8821au_do_iqk(rtlpriv, delta_iqk, thermal_value, 8);

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD, "<===ODM_TXPowerTrackingCallback_ThermalMeter\n");

	rtldm->txpowercount = 0;
}

static void rtl8821au_check_tx_power_tracking_thermalmeter(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);

	if ( rtldm->tm_trigger) {		/* at least delay 1 sec */
		/* pHalData->TxPowerCheckCnt++;	//cosa add for debug */
		rtl_set_rfreg(rtlpriv, RF90_PATH_A, RF_T_METER_NEW, (BIT(17) | BIT(16)), 0x03);

		/* DBG_871X("Trigger Thermal Meter!!\n"); */

		 rtldm->tm_trigger = 1;
		return;
	} else {
		/* DBG_871X("Schedule TxPowerTracking direct call!!\n"); */
		if (IS_HARDWARE_TYPE_8812AU(rtlhal))
			rtl8812au_dm_txpower_tracking_callback_thermalmeter(rtlpriv);
		if (IS_HARDWARE_TYPE_8821U(rtlhal))
			rtl8821au_dm_txpower_tracking_callback_thermalmeter(rtlpriv);
		 rtldm->tm_trigger = 0;
	}

}



/*
 * 3============================================================
 * 3 FASLE ALARM CHECK
 * 3============================================================
 */

static void rtl8821au_dm_false_alarm_counter_statistics(struct rtl_priv *rtlpriv)
{
	struct false_alarm_statistics *FalseAlmCnt = &(rtlpriv->falsealm_cnt);
	uint32_t CCKenable;
	/* read OFDM FA counter */
	FalseAlmCnt->cnt_ofdm_fail = rtl_get_bbreg(rtlpriv, ODM_REG_OFDM_FA_11AC, bMaskLWord);
	FalseAlmCnt->cnt_cck_fail = rtl_get_bbreg(rtlpriv, ODM_REG_CCK_FA_11AC, bMaskLWord);

	CCKenable =  rtl_get_bbreg(rtlpriv, ODM_REG_BB_RX_PATH_11AC, BIT(28));
	if (CCKenable)		/* if (*pDM_Odm->pBandType == ODM_BAND_2_4G) */
		FalseAlmCnt->cnt_all = FalseAlmCnt->cnt_ofdm_fail + FalseAlmCnt->cnt_cck_fail;
	else
		FalseAlmCnt->cnt_all = FalseAlmCnt->cnt_ofdm_fail;

	/* reset OFDM FA coutner */
	rtl_set_bbreg(rtlpriv, ODM_REG_OFDM_FA_RST_11AC, BIT(17), 1);
	rtl_set_bbreg(rtlpriv, ODM_REG_OFDM_FA_RST_11AC, BIT(17), 0);
	/* reset CCK FA counter */
	rtl_set_bbreg(rtlpriv, ODM_REG_CCK_FA_RST_11AC, BIT(15), 0);
	rtl_set_bbreg(rtlpriv, ODM_REG_CCK_FA_RST_11AC, BIT(15), 1);

	RT_TRACE(rtlpriv, COMP_DIG, DBG_TRACE, "Cnt_Cck_fail=%d\n", FalseAlmCnt->cnt_cck_fail);
	RT_TRACE(rtlpriv, COMP_DIG, DBG_TRACE, "Cnt_Ofdm_fail=%d\n", FalseAlmCnt->cnt_ofdm_fail);
	RT_TRACE(rtlpriv, COMP_DIG, DBG_TRACE, "Total false Alarm=%d\n", FalseAlmCnt->cnt_all);
}


/*
 * 3============================================================
 * 3 RSSI Monitor
 * 3============================================================
 */


static void FindMinimumRSSI(struct rtl_priv *rtlpriv)
{
	struct dig_t *rtl_dm_dig = &rtlpriv->dm_digtable;

	/* 1 1.Determine the minimum RSSI */

	if (rtlpriv->mac80211.link_state < MAC80211_LINKED && (rtlpriv->dm.entry_min_undec_sm_pwdb == 0)) {
		rtl_dm_dig->min_undec_pwdb_for_dm = 0;
		/* RT_TRACE(rtlrpiv,COMP_BB_POWERSAVING, DBG_LOUD, ("Not connected to any \n")); */
	} else {
		rtl_dm_dig->min_undec_pwdb_for_dm = rtlpriv->dm.entry_min_undec_sm_pwdb;
	}

	/* DBG_8192C("%s=>MinUndecoratedPWDBForDM(%d)\n",__FUNCTION__,pdmpriv->MinUndecoratedPWDBForDM); */
	/* RT_TRACE(rtlrpiv,COMP_DIG, DBG_LOUD, ("MinUndecoratedPWDBForDM =%d\n",pHalData->MinUndecoratedPWDBForDM)); */
}

static void rtl8821au_dm_check_rssi_monitor(struct rtl_priv *rtlpriv)
{
	struct dig_t *rtl_dm_dig = &rtlpriv->dm_digtable;
	int	i;
	int	tmpEntryMaxPWDB = 0, tmpEntryMinPWDB = 0xff;
	u8 	sta_cnt = 0;
	u8	UL_DL_STATE = 0;			/*  for 8812 use */
	uint32_t PWDB_rssi[NUM_STA] = { 0 };		/* [0~15]:MACID, [16~31]:PWDB_rssi */

	if (rtlpriv->mac80211.link_state < MAC80211_LINKED)
		return;

	if (1) {
		u64	curTxOkCnt = rtlpriv->xmitpriv.tx_bytes - rtlpriv->xmitpriv.last_tx_bytes;
		u64	curRxOkCnt = rtlpriv->recvpriv.rx_bytes - rtlpriv->recvpriv.last_rx_bytes;

		if (curRxOkCnt > (curTxOkCnt*6))
			UL_DL_STATE = 1;
		else
			UL_DL_STATE = 0;
	}


	/* if (check_fwstate(&rtlpriv->mlmepriv, WIFI_AP_STATE|WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE) == true) */
	{
#if 1
		/* ULLI : This will go away in rtlwifi-lib */
		struct sta_info *psta;
		struct rtl_dm *rtldm = rtl_dm(rtlpriv);
		struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);


		for (i = 0; i < ODM_ASSOCIATE_ENTRY_NUM; i++) {
			psta = rtldm->pODM_StaInfo[i];
			if (IS_STA_VALID(psta)) {
				if (psta->rssi_stat.UndecoratedSmoothedPWDB < tmpEntryMinPWDB)
					tmpEntryMinPWDB = psta->rssi_stat.UndecoratedSmoothedPWDB;

				if (psta->rssi_stat.UndecoratedSmoothedPWDB > tmpEntryMaxPWDB)
					tmpEntryMaxPWDB = psta->rssi_stat.UndecoratedSmoothedPWDB;

				if (psta->rssi_stat.UndecoratedSmoothedPWDB != (-1)) {
					if (1)
						PWDB_rssi[sta_cnt++] = (((u8)(psta->mac_id&0xFF)) | ((psta->rssi_stat.UndecoratedSmoothedPWDB&0x7F)<<16));
					else
						PWDB_rssi[sta_cnt++] = (psta->mac_id | (psta->rssi_stat.UndecoratedSmoothedPWDB<<16));

				}
			}
		}
#else
		_irqL irqL;
		struct list_head	*plist, *phead;
		struct sta_info *psta;
		struct sta_priv *pstapriv = &rtlpriv->stapriv;
		u8 bcast_addr[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

		spin_lock_bh(&pstapriv->sta_hash_lock, &irqL);

		for (i = 0; i < NUM_STA; i++) {
			phead = &(pstapriv->sta_hash[i]);
			plist = get_next(phead);

			while ((rtw_end_of_queue_search(phead, plist)) == false) {
				psta = container_of(plist, struct sta_info, hash_list);

				plist = get_next(plist);

				if (memcmp(psta->hwaddr, bcast_addr, ETH_ALEN) == 0
				 || memcmp(psta->hwaddr, myid(&rtlpriv->eeprompriv), ETH_ALEN) == 0)
					continue;

				if (psta->state & WIFI_ASOC_STATE) {
					if (psta->rssi_stat.UndecoratedSmoothedPWDB < tmpEntryMinPWDB)
						tmpEntryMinPWDB = psta->rssi_stat.UndecoratedSmoothedPWDB;

					if (psta->rssi_stat.UndecoratedSmoothedPWDB > tmpEntryMaxPWDB)
						tmpEntryMaxPWDB = psta->rssi_stat.UndecoratedSmoothedPWDB;

					if (psta->rssi_stat.UndecoratedSmoothedPWDB != (-1)) {
						/* printk("%s==> mac_id(%d),rssi(%d)\n",__FUNCTION__,psta->mac_id,psta->rssi_stat.UndecoratedSmoothedPWDB); */
						PWDB_rssi[sta_cnt++] = (psta->mac_id | (psta->rssi_stat.UndecoratedSmoothedPWDB<<16));
					}
				}

			}

		}

		spin_unlock_bh(&pstapriv->sta_hash_lock, &irqL);
#endif

		/* printk("%s==> sta_cnt(%d)\n",__FUNCTION__,sta_cnt); */

		for (i = 0; i < sta_cnt; i++) {
			if (PWDB_rssi[i] != (0)) {
				if (pHalData->fw_ractrl == true) {	/* Report every sta's RSSI to FW */
					PWDB_rssi[i] |= (UL_DL_STATE << 24);
					rtl8812_set_rssi_cmd(rtlpriv, (u8 *)(&PWDB_rssi[i]));
				} else {
				}
			}
		}
	}



	if (tmpEntryMaxPWDB != 0) {	/* If associated entry is found */
		rtlpriv->dm.entry_max_undec_sm_pwdb = tmpEntryMaxPWDB;
	} else {
		rtlpriv->dm.entry_max_undec_sm_pwdb = 0;
	}

	if (tmpEntryMinPWDB != 0xff) {	/* If associated entry is found */
		rtlpriv->dm.entry_min_undec_sm_pwdb = tmpEntryMinPWDB;
	} else {
		rtlpriv->dm.entry_min_undec_sm_pwdb = 0;
	}

	FindMinimumRSSI(rtlpriv);	/* get pdmpriv->MinUndecoratedPWDBForDM */

	rtl_dm_dig->rssi_val_min = rtl_dm_dig->min_undec_pwdb_for_dm;
}


/*
 * ============================================================
 * EDCA Turbo
 * ============================================================
 */

static void rtl8821au_dm_check_edca_turbo(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal =&(rtlpriv->rtlhal);
	
	u64		cur_tx_bytes = 0;
	u64		cur_rx_bytes = 0;
	uint32_t	EDCA_BE_UL = 0x5ea42b;	/* Parameter suggested by Scott  */	/* edca_setting_UL[pMgntInfo->IOTPeer]; */
	uint32_t	EDCA_BE_DL = 0x5ea42b;	/* Parameter suggested by Scott  */	/* edca_setting_DL[pMgntInfo->IOTPeer]; */
	uint32_t	IOTPeer = 0;
	
	
	
	struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct _rtw_dm *pDM_Odm = &(pHalData->odmpriv);
	uint32_t 	trafficIndex;
	uint32_t	edca_param;
	u8		bbtchange = false;
	struct xmit_priv		*pxmitpriv = &(rtlpriv->xmitpriv);
	struct recv_priv		*precvpriv = &(rtlpriv->recvpriv);
	struct mlme_ext_priv	*pmlmeext = &(rtlpriv->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	/*
	 * For AP/ADSL use prtl8192cd_priv
	 * For CE/NIC use _ADAPTER
	 */

	/*
	 *
	 * 2011/09/29 MH In HW integration first stage, we provide 4 different handle to operate
	 * at the same time. In the stage2/3, we need to prive universal interface and merge all
	 * HW dynamic mechanism.
	 */

	IOTPeer = pmlmeinfo->assoc_AP_vendor;

	if (IOTPeer >=  HT_IOT_PEER_MAX) {
		goto dm_CheckEdcaTurbo_EXIT;
	}

	/* Check if the status needs to be changed. */
	if ((bbtchange) || (!rtlpriv->dm.is_any_nonbepkts)) {
		cur_tx_bytes = pxmitpriv->tx_bytes - pxmitpriv->last_tx_bytes;
		cur_rx_bytes = precvpriv->rx_bytes - precvpriv->last_rx_bytes;

		/* traffic, TX or RX */
		if ((IOTPeer == HT_IOT_PEER_RALINK) || (IOTPeer == HT_IOT_PEER_ATHEROS)) {
			if (cur_tx_bytes > (cur_rx_bytes << 2)) {
				/* Uplink TP is present. */
				trafficIndex = UP_LINK;
			} else {
				/* Balance TP is present. */
				trafficIndex = DOWN_LINK;
			}
		} else {
			if (cur_rx_bytes > (cur_tx_bytes << 2)) {
				/* Downlink TP is present. */
				trafficIndex = DOWN_LINK;
			} else {
				/* Balance TP is present. */
				trafficIndex = UP_LINK;
			}
		}

		if ((pDM_Odm->DM_EDCA_Table.prv_traffic_idx != trafficIndex) || (!rtlpriv->dm.current_turbo_edca)) {
			/* merge from 92s_92c_merge temp brunch v2445    20120215 */
			if ((IOTPeer == HT_IOT_PEER_CISCO)) {
				EDCA_BE_DL = edca_setting_gmode[IOTPeer];
			} else if ((IOTPeer == HT_IOT_PEER_AIRGO)) {
					EDCA_BE_DL = 0xa630;
			} else if (IOTPeer == HT_IOT_PEER_MARVELL) {
				EDCA_BE_DL = edca_setting_dl[IOTPeer];
				EDCA_BE_UL = edca_setting_ul[IOTPeer];
			} else if (IOTPeer == HT_IOT_PEER_ATHEROS) {
				/* Set DL EDCA for Atheros peer to 0x3ea42b. Suggested by SD3 Wilson for ASUS TP issue. */
				EDCA_BE_DL = edca_setting_dl[IOTPeer];
			}

			if ((IS_HARDWARE_TYPE_8811AU(rtlhal))) {		/* add 8812AU/8812AE */
				EDCA_BE_UL = 0x5ea42b;
				EDCA_BE_DL = 0x5ea42b;
				RT_TRACE(rtlpriv, COMP_TURBO, DBG_LOUD, "8812A: EDCA_BE_UL=0x%x EDCA_BE_DL =0x%x", EDCA_BE_UL, EDCA_BE_DL);
			}

			if (trafficIndex == DOWN_LINK)
				edca_param = EDCA_BE_DL;
			else
				edca_param = EDCA_BE_UL;

			rtl_write_dword(rtlpriv, REG_EDCA_BE_PARAM, edca_param);

			pDM_Odm->DM_EDCA_Table.prv_traffic_idx = trafficIndex;
		}

		rtlpriv->dm.current_turbo_edca = true;
	} else {
		/*
		 * Turn Off EDCA turbo here.
		 * Restore original EDCA according to the declaration of AP.
		 */
		if (rtlpriv->dm.current_turbo_edca) {
			rtl_write_dword(rtlpriv, REG_EDCA_BE_PARAM, pHalData->AcParam_BE);
			rtlpriv->dm.current_turbo_edca = false;
		}
	}

dm_CheckEdcaTurbo_EXIT:
	/* Set variables for next time. */
	rtlpriv->dm.is_any_nonbepkts = false;
	pxmitpriv->last_tx_bytes = pxmitpriv->tx_bytes;
	precvpriv->last_rx_bytes = precvpriv->rx_bytes;
}

static void dm_CheckPbcGPIO(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	uint8_t	tmp1byte;
	uint8_t	bPbcPressed = false;

	if(!rtlpriv->registrypriv.hw_wps_pbc)
		return;

	if (IS_HARDWARE_TYPE_8812(rtlhal)) {
		tmp1byte = rtl_read_byte(rtlpriv, GPIO_IO_SEL);
		tmp1byte |= (HAL_8192C_HW_GPIO_WPS_BIT);
		rtl_write_byte(rtlpriv, GPIO_IO_SEL, tmp1byte);	/* enable GPIO[2] as output mode */

		tmp1byte &= ~(HAL_8192C_HW_GPIO_WPS_BIT);
		rtl_write_byte(rtlpriv,  GPIO_IN, tmp1byte);	/* reset the floating voltage level */

		tmp1byte = rtl_read_byte(rtlpriv, GPIO_IO_SEL);
		tmp1byte &= ~(HAL_8192C_HW_GPIO_WPS_BIT);
		rtl_write_byte(rtlpriv, GPIO_IO_SEL, tmp1byte);	/* enable GPIO[2] as input mode */

		tmp1byte =rtl_read_byte(rtlpriv, GPIO_IN);

		if (tmp1byte == 0xff)
			return ;

		if (tmp1byte&HAL_8192C_HW_GPIO_WPS_BIT) {
			bPbcPressed = true;
		}
	} else if (IS_HARDWARE_TYPE_8821(rtlhal)) {
		tmp1byte = rtl_read_byte(rtlpriv, GPIO_IO_SEL_8811A);
		tmp1byte |= (BIT(4));
		rtl_write_byte(rtlpriv, GPIO_IO_SEL_8811A, tmp1byte);	/* enable GPIO[2] as output mode */

		tmp1byte &= ~(BIT(4));
		rtl_write_byte(rtlpriv,  GPIO_IN_8811A, tmp1byte);		/* reset the floating voltage level */

		tmp1byte = rtl_read_byte(rtlpriv, GPIO_IO_SEL_8811A);
		tmp1byte &= ~(BIT(4));
		rtl_write_byte(rtlpriv, GPIO_IO_SEL_8811A, tmp1byte);	/* enable GPIO[2] as input mode */

		tmp1byte =rtl_read_byte(rtlpriv, GPIO_IN_8811A);

		if (tmp1byte == 0xff)
			return ;

		if (tmp1byte&BIT(4)) {
			bPbcPressed = true;
		}
	}
	if( true == bPbcPressed) {
		/*
		 * Here we only set bPbcPressed to true
		 * After trigger PBC, the variable will be set to false
		 */
		printk("rtl8821au:CheckPbcGPIO - PBC is pressed\n");
	}
}


static void rtl8821au_dm_common_info_self_update(struct rtl_priv *rtlpriv)
{
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);
	u8	EntryCnt = 0;
	u8	i;
	struct sta_info *pEntry;

	for (i = 0; i < ODM_ASSOCIATE_ENTRY_NUM; i++) {
		pEntry = rtldm->pODM_StaInfo[i];
		if (IS_STA_VALID(pEntry))
			EntryCnt++;
	}

	if (EntryCnt == 1)
		rtldm->bOneEntryOnly = true;
	else
		rtldm->bOneEntryOnly = false;
}

static void rtl8821au_dm_cck_packet_detection_thresh(struct rtl_priv *rtlpriv)
{
	struct dig_t *dm_digtable = &(rtlpriv->dm_digtable);
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);

	u8	CurCCK_CCAThres;
	struct false_alarm_statistics *FalseAlmCnt = &(rtlpriv->falsealm_cnt);

	if (rtlhal->external_lna_2g)
		return;

	if (rtlpriv->mac80211.link_state >= MAC80211_LINKED) {
		if (dm_digtable->rssi_val_min > 25)
			CurCCK_CCAThres = 0xcd;
		else if ((dm_digtable->rssi_val_min <= 25) && (dm_digtable->rssi_val_min > 10))
			CurCCK_CCAThres = 0x83;
		else {
			if (FalseAlmCnt->cnt_cck_fail > 1000)
				CurCCK_CCAThres = 0x83;
			else
				CurCCK_CCAThres = 0x40;
		}
	} else {
		if (FalseAlmCnt->cnt_cck_fail > 1000)
			CurCCK_CCAThres = 0x83;
		else
			CurCCK_CCAThres = 0x40;
	}

		ODM_Write_CCK_CCA_Thres(rtlpriv, CurCCK_CCAThres);
}

void rtl8821au_dm_watchdog(struct rtl_priv *rtlpriv)
{
	struct dig_t *dm_digtable = &(rtlpriv->dm_digtable);
	bool fw_current_inpsmode = false;
	bool fw_ps_awake = true;
	uint8_t hw_init_completed = false;

	struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct _rtw_dm *pDM_Odm = &(pHalData->odmpriv);

	if (hw_init_completed == false)
		goto skip_dm;

	fw_current_inpsmode = rtlpriv->pwrctrlpriv.fw_current_inpsmode;
	rtlpriv->cfg->ops->get_hw_reg(rtlpriv, HW_VAR_FWLPS_RF_ON, (uint8_t *)(&fw_ps_awake));

	/* ODM */
	if (rtlpriv->hw_init_completed == true) {
		if(rtw_linked_check(rtlpriv))
			rtlpriv->mac80211.link_state = MAC80211_LINKED;
		else
			rtlpriv->mac80211.link_state = MAC80211_NOLINK;

		/*
		 * 2011/09/20 MH This is the entry pointer for all team to execute HW out source DM.
		 * You can not add any dummy function here, be care, you can only use DM structure
		 * to perform any new ODM_DM.
		 */

		rtl8821au_dm_common_info_self_update(rtlpriv);
		rtl8821au_dm_false_alarm_counter_statistics(rtlpriv);
		RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): RSSI=0x%x\n", dm_digtable->rssi_val_min);

		rtl8821au_dm_check_rssi_monitor(rtlpriv);

		rtl8821au_dm_dig(rtlpriv);

		odm_Adaptivity(rtlpriv, dm_digtable->cur_igvalue);

		rtl8821au_dm_cck_packet_detection_thresh(rtlpriv);

		if (rtlpriv->pwrctrlpriv.bpower_saving == true)
			return;

		odm_RefreshRateAdaptiveMask(rtlpriv);
		rtl8821au_dm_check_edca_turbo(rtlpriv);

		rtl8821au_check_tx_power_tracking_thermalmeter(rtlpriv);

		rtlpriv->dm.dbginfo.num_qry_beacon_pkt = 0;
	}

skip_dm:

	/*
	 * Check GPIO to determine current RF on/off and Pbc status.
	 * Check Hardware Radio ON/OFF or not
	 */

	/* temp removed */
	/* ULLI : for WPS Button */
	dm_CheckPbcGPIO(rtlpriv);

	return;
}

static void rtl8821au_dm_dig(struct rtl_priv *rtlpriv)
{
	struct rtl_hal *rtlhal = rtl_hal(rtlpriv);
	struct dig_t *dm_digtable = &(rtlpriv->dm_digtable);
	struct rtl_mac *mac = rtl_mac(rtlpriv);
	struct false_alarm_statistics *FalseAlmCnt = &(rtlpriv->falsealm_cnt);
	u8 dig_dynamic_min, dig_maxofmin;
	bool						FirstConnect, FirstDisConnect;
	u8						dm_dig_max, dm_dig_min, offset;
	u8 current_igi = dm_digtable->cur_igvalue;

	struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct _rtw_dm *pDM_Odm = &(pHalData->odmpriv);

	RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG()==>\n");

	if (mac->act_scanning) {
		RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG() Return: In Scan Progress \n");
		return;
	}

	/* add by Neil Chen to avoid PSD is processing */
	dig_dynamic_min = dm_digtable->dig_min_0;
	FirstConnect = (rtlpriv->mac80211.link_state >= MAC80211_LINKED)  &&
		       (dm_digtable->media_connect_0 == false);
	FirstDisConnect = (rtlpriv->mac80211.link_state < MAC80211_LINKED) &&
			   (dm_digtable->media_connect_0 == true);


	/* 1 Boundary Decision */
	dm_dig_max = DM_DIG_MAX_NIC;

	if (IS_HARDWARE_TYPE_8821U(rtlhal))
		dm_dig_min = DM_DIG_MIN_NIC;
	else
		dm_dig_min = 0x1C;

	dig_maxofmin = DM_DIG_MAX_AP;

	if (rtlpriv->mac80211.link_state >= MAC80211_LINKED) {
		/* 2 Modify DIG upper bound */
		/* 2013.03.19 Luke: Modified upper bound for Netgear rental house test */
		if (IS_HARDWARE_TYPE_8821U(rtlhal))
			offset = 20;
		else
			offset = 10;

		if ((dm_digtable->rssi_val_min + offset) > dm_dig_max)
			dm_digtable->rx_gain_max = dm_dig_max;
		else if ((dm_digtable->rssi_val_min + offset) < dm_dig_min)
			dm_digtable->rx_gain_max = dm_dig_min;
		else
			dm_digtable->rx_gain_max = dm_digtable->rssi_val_min + offset;


		/* 2 Modify DIG lower bound */
		/*
		if ((pFalseAlmCnt->Cnt_all > 500)&&(DIG_Dynamic_MIN < 0x25))
			DIG_Dynamic_MIN++;
		else if (((pFalseAlmCnt->Cnt_all < 500)||(pDM_Odm->rssi_val_min < 8))&&(DIG_Dynamic_MIN > dm_dig_min))
			DIG_Dynamic_MIN--;
		*/
		if (pDM_Odm->bOneEntryOnly) {
			if (dm_digtable->rssi_val_min < dm_dig_min)
				dig_dynamic_min = dm_dig_min;
			else if (dm_digtable->rssi_val_min > dig_maxofmin)
				dig_dynamic_min = dig_maxofmin;
			else
				dig_dynamic_min = dm_digtable->rssi_val_min;
			RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG() : bOneEntryOnly=true,  DIG_Dynamic_MIN=0x%x\n", dig_dynamic_min);
			RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG() : pDM_Odm->rssi_val_min=%d\n", dm_digtable->rssi_val_min);
		} else {
			/* 1 Lower Bound for 88E AntDiv */
			dig_dynamic_min = dm_dig_min;
		}
	} else {
		dm_digtable->rx_gain_max = dm_dig_max;
		dig_dynamic_min = dm_dig_min;
		RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG() : No Link\n");
	}

	/* 1 Modify DIG lower bound, deal with abnorally large false alarm */
	if (FalseAlmCnt->cnt_all > 10000) {
		RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "dm_DIG(): Abnornally false alarm case. \n");

		if (dm_digtable->large_fa_hit != 3)
			dm_digtable->large_fa_hit++;
		if (dm_digtable->forbidden_igi < current_igi) {			/* if (dm_digtable->ForbiddenIGI < dm_digtable->CurIGValue) */
			dm_digtable->forbidden_igi = (u8)current_igi;	/* dm_digtable->ForbiddenIGI = dm_digtable->CurIGValue; */
			dm_digtable->large_fa_hit = 1;
		}

		if (dm_digtable->large_fa_hit >= 3) {
			if ((dm_digtable->forbidden_igi+1) > dm_digtable->rx_gain_max)
				dm_digtable->rx_gain_min = dm_digtable->rx_gain_max;
			else
				dm_digtable->rx_gain_min = (dm_digtable->forbidden_igi + 1);
			dm_digtable->recover_cnt = 3600; 	/* 3600=2hr */
		}

	} else {
		/* Recovery mechanism for IGI lower bound */
		if (dm_digtable->recover_cnt != 0)
			dm_digtable->recover_cnt--;
		else {
			if (dm_digtable->large_fa_hit < 3) {
				if ((dm_digtable->forbidden_igi-1) < dig_dynamic_min) {		/* DM_DIG_MIN)  */
					dm_digtable->forbidden_igi = dig_dynamic_min;		/* DM_DIG_MIN; */
					dm_digtable->rx_gain_min = dig_dynamic_min;	/* DM_DIG_MIN; */
					RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): Normal Case: At Lower Bound\n");
				} else {
					dm_digtable->forbidden_igi--;
					dm_digtable->rx_gain_min = (dm_digtable->forbidden_igi+ 1);
					RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): Normal Case: Approach Lower Bound\n");
				}
			} else {
				dm_digtable->large_fa_hit = 0;
			}
		}
	}
	RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): dm_digtable->LargeFAHit=%d\n", dm_digtable->large_fa_hit);

	if (rtlpriv->dm.dbginfo.num_qry_beacon_pkt < 10)
		dm_digtable->rx_gain_min = dm_dig_min;

	if (dm_digtable->rx_gain_min > dm_digtable->rx_gain_max)
		dm_digtable->rx_gain_min = dm_digtable->rx_gain_max;

	/* 1 Adjust initial gain by false alarm */
	if (rtlpriv->mac80211.link_state >= MAC80211_LINKED) {
		RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): DIG AfterLink\n");
		if (FirstConnect) {
			if (dm_digtable->rssi_val_min <= dig_maxofmin)
			    current_igi = dm_digtable->rssi_val_min;
			else
			    current_igi = dig_maxofmin;
			RT_TRACE(rtlpriv,	COMP_DIG, DBG_LOUD, "DIG: First Connect\n");
		} else 	{
			/* FA for Combo IC--NeilChen--2012--09--28 */
			if (FalseAlmCnt->cnt_all > DM_DIG_FA_TH2)
				current_igi = current_igi + 4;	/* dm_digtable->CurIGValue = dm_digtable->PreIGValue+2; */
			else if (FalseAlmCnt->cnt_all > DM_DIG_FA_TH1)
				current_igi = current_igi + 2;	/* dm_digtable->CurIGValue = dm_digtable->PreIGValue+1; */
			else if (FalseAlmCnt->cnt_all < DM_DIG_FA_TH0)
				current_igi = current_igi - 2;	/* dm_digtable->CurIGValue =dm_digtable->PreIGValue-1; */

			if ((rtlpriv->dm.dbginfo.num_qry_beacon_pkt < 10)
			 && (FalseAlmCnt->cnt_all < DM_DIG_FA_TH1))
				current_igi = dm_digtable->rx_gain_min;
		}
	} else {
		/* current_igi = dm_digtable->rx_gain_range_min; */	/* dm_digtable->CurIGValue = dm_digtable->rx_gain_range_min */
		RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): DIG BeforeLink\n");
		if (FirstDisConnect) {
			current_igi = dm_digtable->rx_gain_min;
			RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): First DisConnect \n");
		} else {
			/* 2012.03.30 LukeLee: enable DIG before link but with very high thresholds */
			if (FalseAlmCnt->cnt_all > 10000)
				current_igi = current_igi + 4;
			else if (FalseAlmCnt->cnt_all > 8000)
				current_igi = current_igi + 2;
			else if (FalseAlmCnt->cnt_all < 500)
				current_igi = current_igi - 2;
			RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): England DIG \n");
		}
	}
	RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): DIG End Adjust IGI\n");
	/* 1 Check initial gain by upper/lower bound */

	if (current_igi > dm_digtable->rx_gain_max)
		current_igi = dm_digtable->rx_gain_max;
	if (current_igi < dm_digtable->rx_gain_min)
		current_igi = dm_digtable->rx_gain_min;

	if (current_igi > (pDM_Odm->IGI_target + 4))
		current_igi = (u8)pDM_Odm->IGI_target + 4;

	RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): rx_gain_range_max=0x%x, rx_gain_range_min=0x%x\n",
		dm_digtable->rx_gain_max, dm_digtable->rx_gain_min);
	RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): TotalFA=%d\n", FalseAlmCnt->cnt_all);
	RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "odm_DIG(): CurIGValue=0x%x\n", current_igi);

	/* 2 High power RSSI threshold */

	rtl8821au_dm_write_dig(rtlpriv, current_igi);	/* ODM_Write_DIG(pDM_Odm, dm_digtable->CurIGValue); */
	dm_digtable->media_connect_0 =
		(rtlpriv->mac80211.link_state >= MAC80211_LINKED) ? 1 : 0;
		dm_digtable->dig_min_0 = dig_dynamic_min;
}

void rtl8821au_dm_clean_txpower_tracking_state(struct rtl_priv *rtlpriv)
{
	struct rtl_efuse *efuse = rtl_efuse(rtlpriv);
	struct rtl_dm	*rtldm = rtl_dm(rtlpriv);
	u8 p = 0;

	rtldm->swing_idx_cck_base = rtldm->default_cck_index;
	rtldm->swing_idx_cck = rtldm->default_cck_index;
	rtldm->cck_index;	/* ULLI BUG ?? */

	for (p = RF90_PATH_A; p < MAX_RF_PATH; ++p) {
		rtldm->swing_idx_ofdm_base[p] = rtldm->default_ofdm_index;
		rtldm->swing_idx_ofdm[p] = rtldm->default_ofdm_index;
		rtldm->ofdm_index[p] = rtldm->default_ofdm_index;

		rtldm->power_index_offset[p] = 0;
		rtldm->delta_power_index[p] = 0;
		rtldm->delta_power_index_last[p] = 0;
		rtldm->power_index_offset[p] = 0;	/* ULLI Huh */

		rtldm->absolute_ofdm_swing_idx[p] = 0;    /* Initial Mix mode power tracking */
		rtldm->remnant_ofdm_swing_idx[p] = 0;
	}

	rtldm->modify_txagc_flag_path_a = false;       /* Initial at Modify Tx Scaling Mode */
	rtldm->modify_txagc_flag_path_b = false;       /* Initial at Modify Tx Scaling Mode */
	rtldm->remnant_cck_idx = 0;
	rtldm->thermalvalue = efuse->eeprom_thermalmeter;
	rtldm->thermalvalue_iqk = efuse->eeprom_thermalmeter;
	rtldm->thermalvalue_lck = efuse->eeprom_thermalmeter;
}

static void rtl8821_dm_init_gpio_setting(struct rtl_priv *rtlpriv)
{
	uint8_t	tmp1byte;

	tmp1byte = rtl_read_byte(rtlpriv, REG_GPIO_MUXCFG);
	tmp1byte &= (GPIOSEL_GPIO | ~GPIOSEL_ENBT);

	rtl_write_byte(rtlpriv, REG_GPIO_MUXCFG, tmp1byte);

}

static void Update_ODM_ComInfo_8812(struct rtl_priv *rtlpriv)
{
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);

	struct mlme_priv	*pmlmepriv = &rtlpriv->mlmepriv;
	struct pwrctrl_priv *pwrctrlpriv = &rtlpriv->pwrctrlpriv;
	struct _rtw_hal *pHalData = GET_HAL_DATA(rtlpriv);
	struct _rtw_dm *	pDM_Odm = &(pHalData->odmpriv);

	int i;
	
	ODM_CmnInfoHook(pDM_Odm,ODM_CMNINFO_SCAN,&(pmlmepriv->bScanInProcess));

	for (i = 0; i < NUM_STA; i++) {
		rtldm->pODM_StaInfo[i] = NULL;
		/* pDM_Odm->pODM_StaInfo[i] = NULL; */
	}
}


void rtl8812_dm_init(struct rtl_priv *rtlpriv)
{
	rtl8821_dm_init_gpio_setting(rtlpriv);

	rtlpriv->dm.dm_type = DM_Type_ByDriver;
	rtlpriv->dm.dm_flag = 0;

	Update_ODM_ComInfo_8812(rtlpriv);
	ODM_DMInit(rtlpriv);

	rtlpriv->fix_rate = 0xFF;
}

void rtl8821au_dm_write_dig(struct rtl_priv *rtlpriv, u8 current_igi)
{
	struct dig_t *dm_digtable = &(rtlpriv->dm_digtable);

	if (dm_digtable->stop_dig)
		return;

	if (dm_digtable->cur_igvalue != current_igi) {	/*if (pDM_DigTable->PreIGValue != current_igi) */
		rtl_set_bbreg(rtlpriv, ODM_REG_IGI_A_11AC, ODM_BIT_IGI_11AC, current_igi);
		if (rtlpriv->phy.rf_type != ODM_1T1R)
			rtl_set_bbreg(rtlpriv, ODM_REG_IGI_B_11AC, ODM_BIT_IGI_11AC, current_igi);

		RT_TRACE(rtlpriv, COMP_DIG, DBG_LOUD, "current_igi(0x%02x). \n", current_igi);
		/* pDM_DigTable->PreIGValue = pDM_DigTable->CurIGValue; */
		dm_digtable->cur_igvalue = current_igi;
	}
}

/* ULLI : From RTL8821AE */

void rtl8821au_dm_update_init_rate(struct rtl_priv *rtlpriv, u8 rate)
{
	struct rtl_dm *rtldm = rtl_dm(rtlpriv);
	u8 p = 0;

	RT_TRACE(rtlpriv, COMP_POWER_TRACKING, DBG_LOUD,
		 "Get C2H Command! Rate=0x%x\n", rate);

	rtldm->tx_rate = rate;

#if 0	/* ULLI : Must check */
	if (rtlhal->hw_type == HARDWARE_TYPE_RTL8821AE) {
		rtl8821ae_dm_txpwr_track_set_pwr(hw, MIX_MODE, RF90_PATH_A, 0);
	} else {
		for (p = RF90_PATH_A; p < MAX_PATH_NUM_8812A; p++)
			rtl8812ae_dm_txpwr_track_set_pwr(hw, MIX_MODE, p, 0);
	}
#endif
}
