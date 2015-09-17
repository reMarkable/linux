/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2016 by Freescale Semiconductor
 * Copyright 2016-2017 NXP
 */

#ifndef __DT_BINDINGS_S32V234_PINCTRL_H__
#define __DT_BINDINGS_S32V234_PINCTRL_H__

/*
 * Use to set PAD control
 */
#define PAD_CTL_DCYLE_TRIM_OFS      (22)
#define PAD_CTL_DCYLE_TRIM_NONE     (0 << PAD_CTL_DCYLE_TRIM_OFS)
#define PAD_CTL_DCYLE_TRIM_LEFT     (1 << PAD_CTL_DCYLE_TRIM_OFS)
#define PAD_CTL_DCYLE_TRIM_RIGHT    (2 << PAD_CTL_DCYLE_TRIM_OFS)

#define PAD_CTL_OBE         (1 << 21)
#define PAD_CTL_ODE         (1 << 20)
#define PAD_CTL_IBE         (1 << 19)
#define PAD_CTL_HYS         (1 << 18)
#define PAD_CTL_INV         (1 << 17)
#define PAD_CTL_PKE         (1 << 16)

#define PAD_CTL_SRE_OFS             (14)
#define PAD_CTL_SRE_LOW_50HZ        (0 << PAD_CTL_SRE_OFS)
#define PAD_CTL_SRE_LOW_100MHZ      (1 << PAD_CTL_SRE_OFS)
/* The manual reports the same value for SRE = 01 and SRE = 10 */
#define PAD_CTL_SRE_HIGH_100MHZ     (2 << PAD_CTL_SRE_OFS)
#define PAD_CTL_SRE_HIGH_200MHZ     (3 << PAD_CTL_SRE_OFS)

#define PAD_CTL_PUE             (1 << 13)

#define PAD_CTL_PUS_OFS         (11)
#define PAD_CTL_PUS_100K_DOWN   (0 << PAD_CTL_PUS_OFS)
#define PAD_CTL_PUS_50K_UP      (1 << PAD_CTL_PUS_OFS)
#define PAD_CTL_PUS_100K_UP     (2 << PAD_CTL_PUS_OFS)
#define PAD_CTL_PUS_33K_UP      (3 << PAD_CTL_PUS_OFS)
#define PAD_CTL_PUS_MASK        (3 << PAD_CTL_PUS_OFS)

#define PAD_CTL_DSE_OFS         (8)
#define PAD_CTL_DSE_OUT_DISABLE (0 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_240         (1 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_120         (2 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_80          (3 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_60          (4 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_48          (5 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_40          (6 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_34          (7 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_MASK        (7 << PAD_CTL_DSE_OFS)

#define PAD_CTL_CRPOINT_TRIM    (3 << 6)

#define PAD_CTL_SMC             (1 << 5)

#define PAD_CTL_MUX_MODE_ALT0   (0)
#define PAD_CTL_MUX_MODE_ALT1   (1)
#define PAD_CTL_MUX_MODE_ALT2   (2)
#define PAD_CTL_MUX_MODE_ALT3   (3)
#define PAD_CTL_MUX_MODE_ALT4   (4)
#define PAD_CTL_MUX_MODE_ALT5   (5)
#define PAD_CTL_MUX_MODE_ALT6   (6)
#define PAD_CTL_MUX_MODE_ALT7   (7)
#define PAD_CTL_MUX_MODE_MASK   (0xF)

/* UART configuration */
#define PAD_CTL_UART_TX         (PAD_CTL_OBE | PAD_CTL_PUS_100K_UP |\
				PAD_CTL_DSE_60 | PAD_CTL_SRE_LOW_100MHZ |\
				PAD_CTL_MUX_MODE_ALT1)
#define PAD_CTL_UART_RX_MSCR    (PAD_CTL_PUE | PAD_CTL_IBE |\
				PAD_CTL_DCYLE_TRIM_RIGHT)
#define PAD_CTL_UART_RX_IMCR    (PAD_CTL_MUX_MODE_ALT2)

/* USDHC configuration  */
#define PAD_CTL_USDHC_BASE      (PAD_CTL_SRE_HIGH_200MHZ | PAD_CTL_OBE | \
				PAD_CTL_DSE_34 | PAD_CTL_PKE |  \
				PAD_CTL_IBE | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE)
#define PAD_CTL_USDHC_CMD       (PAD_CTL_USDHC_BASE | PAD_CTL_MUX_MODE_ALT1)
#define PAD_CTL_USDHC_CLK       (PAD_CTL_USDHC_BASE | PAD_CTL_MUX_MODE_ALT2)
#define PAD_CTL_USDHC_DAT0_3    (PAD_CTL_USDHC_BASE | PAD_CTL_MUX_MODE_ALT2)
#define PAD_CTL_USDHC_DAT4_7    (PAD_CTL_USDHC_BASE | PAD_CTL_MUX_MODE_ALT3)

#endif /* __DT_BINDINGS_S32V234_PINCTRL_H__ */
