/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2016 by Freescale Semiconductor
 * Copyright 2016-2017, 2019 NXP
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

/* ENET CFG1 = 0x203701 */
#define PAD_CTL_ENET_CFG1       (PAD_CTL_DSE_34 | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
                                PAD_CTL_OBE | PAD_CTL_MUX_MODE_ALT1)

/* ENET CFG2 = 0x20c701 */
#define PAD_CTL_ENET_CFG2       (PAD_CTL_DSE_34 | PAD_CTL_SRE_HIGH_200MHZ | \
                                PAD_CTL_OBE | PAD_CTL_MUX_MODE_ALT1)

/* ENET CFG3 = 0x28c701 */
#define PAD_CTL_ENET_CFG3       (PAD_CTL_DSE_34 | PAD_CTL_SRE_HIGH_200MHZ | \
                                PAD_CTL_OBE | PAD_CTL_IBE | PAD_CTL_MUX_MODE_ALT1)

/* ENET CFG7 = 0x8c700 */
#define PAD_CTL_ENET_CFG4       (PAD_CTL_DSE_34 | PAD_CTL_SRE_HIGH_200MHZ | PAD_CTL_IBE)

/* MSCR register numbers associated to port or function */
#define S32V234_MSCR_PA0        0
#define S32V234_MSCR_PA1        1
#define S32V234_MSCR_PA2        2
#define S32V234_MSCR_PA3        3
#define S32V234_MSCR_PA4        4
#define S32V234_MSCR_PA5        5
#define S32V234_MSCR_PA6        6
#define S32V234_MSCR_PA7        7
#define S32V234_MSCR_PA8        8
#define S32V234_MSCR_PA9        9
#define S32V234_MSCR_PA10       10
#define S32V234_MSCR_PA11       11
#define S32V234_MSCR_PA12       12
#define S32V234_MSCR_PA13       13
#define S32V234_MSCR_PA14       14
#define S32V234_MSCR_PA15       15
#define S32V234_MSCR_PB0        16
#define S32V234_MSCR_PB1        17
#define S32V234_MSCR_PB2        18
#define S32V234_MSCR_PB3        19
#define S32V234_MSCR_PB4        20
#define S32V234_MSCR_PB5        21
#define S32V234_MSCR_PB6        22
#define S32V234_MSCR_PB7        23
#define S32V234_MSCR_PB8        24
#define S32V234_MSCR_PB9        25
#define S32V234_MSCR_PB10       26
#define S32V234_MSCR_PB11       27
#define S32V234_MSCR_PB12       28
#define S32V234_MSCR_PB13       29
#define S32V234_MSCR_PB14       30
#define S32V234_MSCR_PB15       31
#define S32V234_MSCR_PC0        32
#define S32V234_MSCR_PC1        33
#define S32V234_MSCR_PC2        34
#define S32V234_MSCR_PC3        35
#define S32V234_MSCR_PC4        36
#define S32V234_MSCR_PC5        37
#define S32V234_MSCR_PC6        38
#define S32V234_MSCR_PC7        39
#define S32V234_MSCR_PC8        40
#define S32V234_MSCR_PC9        41
#define S32V234_MSCR_PC10       42
#define S32V234_MSCR_PC11       43
#define S32V234_MSCR_PC12       44
#define S32V234_MSCR_PC13       45
#define S32V234_MSCR_PC14       46
#define S32V234_MSCR_PC15       47
#define S32V234_MSCR_PD0        48
#define S32V234_MSCR_PD1        49
#define S32V234_MSCR_PD2        50
#define S32V234_MSCR_PD3        51
#define S32V234_MSCR_PD4        52
#define S32V234_MSCR_PD5        53
#define S32V234_MSCR_PD6        54
#define S32V234_MSCR_PD7        55
#define S32V234_MSCR_PD8        56
#define S32V234_MSCR_PD9        57
#define S32V234_MSCR_PD10       58
#define S32V234_MSCR_PD11       59
#define S32V234_MSCR_PD12       60
#define S32V234_MSCR_PD13       61
#define S32V234_MSCR_PD14       62
#define S32V234_MSCR_PD15       63
#define S32V234_MSCR_PE0        64
#define S32V234_MSCR_PE1        65
#define S32V234_MSCR_PE2        66
#define S32V234_MSCR_PE3        67
#define S32V234_MSCR_PE4        68
#define S32V234_MSCR_PE5        69
#define S32V234_MSCR_PE6        70
#define S32V234_MSCR_PE7        71
#define S32V234_MSCR_PE8        72
#define S32V234_MSCR_PE9        73
#define S32V234_MSCR_PE10       74
#define S32V234_MSCR_PE11       75
#define S32V234_MSCR_PE12       76
#define S32V234_MSCR_PE13       77
#define S32V234_MSCR_PE14       78
#define S32V234_MSCR_PE15       79
#define S32V234_MSCR_PF0        80
#define S32V234_MSCR_PF1        81
#define S32V234_MSCR_PF2        82
#define S32V234_MSCR_PF3        83
#define S32V234_MSCR_PF4        84
#define S32V234_MSCR_PF5        85
#define S32V234_MSCR_PF6        86
#define S32V234_MSCR_PF7        87
#define S32V234_MSCR_PF8        88
#define S32V234_MSCR_PF9        89
#define S32V234_MSCR_PF10       90
#define S32V234_MSCR_PF11       91
#define S32V234_MSCR_PF12       92
#define S32V234_MSCR_PF13       93
#define S32V234_MSCR_PF14       94
#define S32V234_MSCR_PF15       95
#define S32V234_MSCR_PG0        96
#define S32V234_MSCR_PG1        97
#define S32V234_MSCR_PG2        98
#define S32V234_MSCR_PG3        99
#define S32V234_MSCR_PG4        100
#define S32V234_MSCR_PG5        101
#define S32V234_MSCR_PG6        102
#define S32V234_MSCR_PG7        103
#define S32V234_MSCR_PG8        104
#define S32V234_MSCR_PG9        105
#define S32V234_MSCR_PG10       106
#define S32V234_MSCR_PG11       107
#define S32V234_MSCR_PG12       108
#define S32V234_MSCR_PG13       109
#define S32V234_MSCR_PG14       110
#define S32V234_MSCR_PG15       111
#define S32V234_MSCR_PH0        112
#define S32V234_MSCR_PH1        113
#define S32V234_MSCR_PH2        114
#define S32V234_MSCR_PH3        115
#define S32V234_MSCR_PH4        116
#define S32V234_MSCR_PH5        117
#define S32V234_MSCR_PH6        118
#define S32V234_MSCR_PH7        119
#define S32V234_MSCR_PH8        120
#define S32V234_MSCR_PH9        121
#define S32V234_MSCR_PH10       122
#define S32V234_MSCR_PH11       123
#define S32V234_MSCR_PH12       124
#define S32V234_MSCR_PH13       125
#define S32V234_MSCR_PH14       126
#define S32V234_MSCR_PH15       127
#define S32V234_MSCR_PJ0        128
#define S32V234_MSCR_PJ1        129
#define S32V234_MSCR_PJ2        130
#define S32V234_MSCR_PJ3        131
#define S32V234_MSCR_PJ4        132
#define S32V234_MSCR_PJ5        133
#define S32V234_MSCR_PJ6        134
#define S32V234_MSCR_PJ7        135
#define S32V234_MSCR_PJ8        136
#define S32V234_MSCR_PJ9        137
#define S32V234_MSCR_PJ10       138
#define S32V234_MSCR_PJ11       139
#define S32V234_MSCR_PJ12       140
#define S32V234_MSCR_PJ13       141
#define S32V234_MSCR_PJ14       142
#define S32V234_MSCR_PJ15       143
#define S32V234_MSCR_PK0        144
#define S32V234_MSCR_PK1        145
#define S32V234_MSCR_PK2        146
#define S32V234_MSCR_PK3        147
#define S32V234_MSCR_PK4        148
#define S32V234_MSCR_PK5        149
#define S32V234_MSCR_PK6        150
#define S32V234_MSCR_PK7        151
#define S32V234_MSCR_PK8        152
#define S32V234_MSCR_PK9        153
#define S32V234_MSCR_PK10       154
#define S32V234_MSCR_PK11       155
#define S32V234_MSCR_PK12       156
#define S32V234_MSCR_PK13       157
#define S32V234_MSCR_PK14       158
#define S32V234_MSCR_PK15       159
#define S32V234_MSCR_PL0        160
#define S32V234_MSCR_PL1        161
#define S32V234_MSCR_PL2        162
#define S32V234_MSCR_PL3        163
#define S32V234_MSCR_PL4        164
#define S32V234_MSCR_PL5        165
#define S32V234_MSCR_PL8        166

#define S32V234_IMCR_UART0_RXD         712
#define S32V234_IMCR_UART1_RXD         714
#define S32V234_IMCR_USDHC_WP          900
#define S32V234_IMCR_USDHC_CMD         901
#define S32V234_IMCR_USDHC_CLK         902
#define S32V234_IMCR_USDHC_DAT0        903
#define S32V234_IMCR_USDHC_DAT1        904
#define S32V234_IMCR_USDHC_DAT2        905
#define S32V234_IMCR_USDHC_DAT3        906
#define S32V234_IMCR_USDHC_DAT4        907
#define S32V234_IMCR_USDHC_DAT5        908
#define S32V234_IMCR_USDHC_DAT6        909
#define S32V234_IMCR_USDHC_DAT7        910
#define S32V234_IMCR_Ethernet_RX_ER    970
#define S32V234_IMCR_Ethernet_COL      971
#define S32V234_IMCR_Ethernet_CRS      972
#define S32V234_IMCR_Ethernet_RX_DV    973
#define S32V234_IMCR_Ethernet_RX_D0    974
#define S32V234_IMCR_Ethernet_RX_D1    975
#define S32V234_IMCR_Ethernet_RX_D2    976
#define S32V234_IMCR_Ethernet_RX_D3    977
#define S32V234_IMCR_Ethernet_TX_CLK   978
#define S32V234_IMCR_Ethernet_RX_CLK   979
#define S32V234_IMCR_Ethernet_MDIO     981
#define S32V234_IMCR_Ethernet_TIMER0   982
#define S32V234_IMCR_Ethernet_TIMER1   983
#define S32V234_IMCR_Ethernet_TIMER2   984

/* ENET configuration */
#define S32V234_PAD_PC13__MDC       S32V234_MSCR_PC13 PAD_CTL_ENET_CFG2

#define S32V234_PAD_PC14__MDIO_OUT  S32V234_MSCR_PC14 PAD_CTL_ENET_CFG3
#define S32v234_PAD_PC14__MDIO_IN   S32V234_IMCR_Ethernet_MDIO \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PC15__TXCLK_OUT S32V234_MSCR_PC15 PAD_CTL_ENET_CFG1
#define S32V234_PAD_PC15__TXCLK_IN  S32V234_IMCR_Ethernet_TX_CLK \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PD0__RXCLK_OUT  S32V234_MSCR_PD0 PAD_CTL_ENET_CFG4
#define S32V234_PAD_PD0__RXCLK_IN   S32V234_IMCR_Ethernet_RX_CLK \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PD1__RX_D0_OUT  S32V234_MSCR_PD1 PAD_CTL_ENET_CFG4
#define S32V234_PAD_PD1__RX_D0_IN   S32V234_IMCR_Ethernet_RX_D0 \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PD2__RX_D1_OUT  S32V234_MSCR_PD2 PAD_CTL_ENET_CFG4
#define S32V234_PAD_PD2__RX_D1_IN   S32V234_IMCR_Ethernet_RX_D1 \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PD3__RX_D2_OUT  S32V234_MSCR_PD3 PAD_CTL_ENET_CFG4
#define S32V234_PAD_PD3__RX_D2_IN   S32V234_IMCR_Ethernet_RX_D2 \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PD4__RX_D3_OUT  S32V234_MSCR_PD4 PAD_CTL_ENET_CFG4
#define S32V234_PAD_PD4__RX_D3_IN   S32V234_IMCR_Ethernet_RX_D3 \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PD4__RX_DV_OUT  S32V234_MSCR_PD5 PAD_CTL_ENET_CFG4
#define S32V234_PAD_PD4__RX_DV_IN   S32V234_IMCR_Ethernet_RX_DV \
				    PAD_CTL_MUX_MODE_ALT2

#define S32V234_PAD_PD7__TX_D0_OUT  S32V234_MSCR_PD7  PAD_CTL_ENET_CFG2
#define S32V234_PAD_PD8__TX_D1_OUT  S32V234_MSCR_PD8  PAD_CTL_ENET_CFG2
#define S32V234_PAD_PD9__TX_D2_OUT  S32V234_MSCR_PD9  PAD_CTL_ENET_CFG2
#define S32V234_PAD_PD10__TX_D3_OUT S32V234_MSCR_PD10 PAD_CTL_ENET_CFG2
#define S32V234_PAD_PD11__TX_EN_OUT S32V234_MSCR_PD11 PAD_CTL_ENET_CFG2


#endif /* __DT_BINDINGS_S32V234_PINCTRL_H__ */
