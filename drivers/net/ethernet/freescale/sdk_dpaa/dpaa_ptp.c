/*
 * DPAA Ethernet Driver -- PTP 1588 clock using the dTSEC
 *
 * Author: Yangbo Lu <yangbo.lu@freescale.com>
 *
 * Copyright 2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
*/

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/timex.h>
#include <linux/io.h>

#include <linux/ptp_clock_kernel.h>

#include "dpaa_eth.h"
#include "mac.h"

struct ptp_clock *clock;

static struct mac_device *mac_dev;
static u32 freqCompensation;

/* Bit definitions for the TMR_CTRL register */
#define ALM1P                 (1<<31) /* Alarm1 output polarity */
#define ALM2P                 (1<<30) /* Alarm2 output polarity */
#define FS                    (1<<28) /* FIPER start indication */
#define PP1L                  (1<<27) /* Fiper1 pulse loopback mode enabled. */
#define PP2L                  (1<<26) /* Fiper2 pulse loopback mode enabled. */
#define TCLK_PERIOD_SHIFT     (16) /* 1588 timer reference clock period. */
#define TCLK_PERIOD_MASK      (0x3ff)
#define RTPE                  (1<<15) /* Record Tx Timestamp to PAL Enable. */
#define FRD                   (1<<14) /* FIPER Realignment Disable */
#define ESFDP                 (1<<11) /* External Tx/Rx SFD Polarity. */
#define ESFDE                 (1<<10) /* External Tx/Rx SFD Enable. */
#define ETEP2                 (1<<9) /* External trigger 2 edge polarity */
#define ETEP1                 (1<<8) /* External trigger 1 edge polarity */
#define COPH                  (1<<7) /* Generated clock output phase. */
#define CIPH                  (1<<6) /* External oscillator input clock phase */
#define TMSR                  (1<<5) /* Timer soft reset. */
#define BYP                   (1<<3) /* Bypass drift compensated clock */
#define TE                    (1<<2) /* 1588 timer enable. */
#define CKSEL_SHIFT           (0)    /* 1588 Timer reference clock source */
#define CKSEL_MASK            (0x3)

/* Bit definitions for the TMR_TEVENT register */
#define ETS2                  (1<<25) /* External trigger 2 timestamp sampled */
#define ETS1                  (1<<24) /* External trigger 1 timestamp sampled */
#define ALM2                  (1<<17) /* Current time = alarm time register 2 */
#define ALM1                  (1<<16) /* Current time = alarm time register 1 */
#define PP1                   (1<<7)  /* periodic pulse generated on FIPER1 */
#define PP2                   (1<<6)  /* periodic pulse generated on FIPER2 */
#define PP3                   (1<<5)  /* periodic pulse generated on FIPER3 */

/* Bit definitions for the TMR_TEMASK register */
#define ETS2EN                (1<<25) /* External trigger 2 timestamp enable */
#define ETS1EN                (1<<24) /* External trigger 1 timestamp enable */
#define ALM2EN                (1<<17) /* Timer ALM2 event enable */
#define ALM1EN                (1<<16) /* Timer ALM1 event enable */
#define PP1EN                 (1<<7) /* Periodic pulse event 1 enable */
#define PP2EN                 (1<<6) /* Periodic pulse event 2 enable */

/* Bit definitions for the TMR_PEVENT register */
#define TXP2                  (1<<9) /* PTP transmitted timestamp im TXTS2 */
#define TXP1                  (1<<8) /* PTP transmitted timestamp in TXTS1 */
#define RXP                   (1<<0) /* PTP frame has been received */

/* Bit definitions for the TMR_PEMASK register */
#define TXP2EN                (1<<9) /* Transmit PTP packet event 2 enable */
#define TXP1EN                (1<<8) /* Transmit PTP packet event 1 enable */
#define RXPEN                 (1<<0) /* Receive PTP packet event enable */

/* Bit definitions for the TMR_STAT register */
#define STAT_VEC_SHIFT        (0) /* Timer general purpose status vector */
#define STAT_VEC_MASK         (0x3f)

/* Bit definitions for the TMR_PRSC register */
#define PRSC_OCK_SHIFT        (0) /* Output clock division/prescale factor. */
#define PRSC_OCK_MASK         (0xffff)


#define N_EXT_TS	2

static void set_alarm(void)
{
	u64 ns;

	if (mac_dev->fm_rtc_get_cnt)
		mac_dev->fm_rtc_get_cnt(mac_dev->fm_dev, &ns);
	ns += 1500000000ULL;
	ns = div_u64(ns, 1000000000UL) * 1000000000ULL;
	ns -= DPA_PTP_NOMINAL_FREQ_PERIOD_NS;
	if (mac_dev->fm_rtc_set_alarm)
		mac_dev->fm_rtc_set_alarm(mac_dev->fm_dev, 0, ns);
}

static void set_fipers(void)
{
	u64 fiper;

	if (mac_dev->fm_rtc_disable)
		mac_dev->fm_rtc_disable(mac_dev->fm_dev);

	set_alarm();
	fiper = 1000000000ULL - DPA_PTP_NOMINAL_FREQ_PERIOD_NS;
	if (mac_dev->fm_rtc_set_fiper)
		mac_dev->fm_rtc_set_fiper(mac_dev->fm_dev, 0, fiper);

	if (mac_dev->fm_rtc_enable)
		mac_dev->fm_rtc_enable(mac_dev->fm_dev);
}

/* PTP clock operations */

static int ptp_dpa_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, tmr_add;
	int neg_adj = 0;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	tmr_add = freqCompensation;
	adj = tmr_add;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);

	tmr_add = neg_adj ? tmr_add - diff : tmr_add + diff;

	if (mac_dev->fm_rtc_set_drift)
		mac_dev->fm_rtc_set_drift(mac_dev->fm_dev, tmr_add);

	return 0;
}

static int ptp_dpa_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	s64 now;

	if (mac_dev->fm_rtc_get_cnt)
		mac_dev->fm_rtc_get_cnt(mac_dev->fm_dev, &now);

	now += delta;

	if (mac_dev->fm_rtc_set_cnt)
		mac_dev->fm_rtc_set_cnt(mac_dev->fm_dev, now);
	set_fipers();

	return 0;
}

static int ptp_dpa_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	u64 ns;
	u32 remainder;

	if (mac_dev->fm_rtc_get_cnt)
		mac_dev->fm_rtc_get_cnt(mac_dev->fm_dev, &ns);

	ts->tv_sec = div_u64_rem(ns, 1000000000, &remainder);
	ts->tv_nsec = remainder;
	return 0;
}

static int ptp_dpa_settime(struct ptp_clock_info *ptp,
			       const struct timespec64 *ts)
{
	u64 ns;

	ns = ts->tv_sec * 1000000000ULL;
	ns += ts->tv_nsec;

	if (mac_dev->fm_rtc_set_cnt)
		mac_dev->fm_rtc_set_cnt(mac_dev->fm_dev, ns);
	set_fipers();
	return 0;
}

static int ptp_dpa_enable(struct ptp_clock_info *ptp,
			      struct ptp_clock_request *rq, int on)
{
	u32 bit;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		switch (rq->extts.index) {
		case 0:
			bit = ETS1EN;
			break;
		case 1:
			bit = ETS2EN;
			break;
		default:
			return -EINVAL;
		}
		if (on) {
			if (mac_dev->fm_rtc_enable_interrupt)
				mac_dev->fm_rtc_enable_interrupt(
					mac_dev->fm_dev, bit);
		} else {
			if (mac_dev->fm_rtc_disable_interrupt)
				mac_dev->fm_rtc_disable_interrupt(
					mac_dev->fm_dev, bit);
		}
		return 0;

	case PTP_CLK_REQ_PPS:
		if (on) {
			if (mac_dev->fm_rtc_enable_interrupt)
				mac_dev->fm_rtc_enable_interrupt(
					mac_dev->fm_dev, PP1EN);
		} else {
			if (mac_dev->fm_rtc_disable_interrupt)
				mac_dev->fm_rtc_disable_interrupt(
					mac_dev->fm_dev, PP1EN);
		}
		return 0;

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static struct ptp_clock_info ptp_dpa_caps = {
	.owner		= THIS_MODULE,
	.name		= "dpaa clock",
	.max_adj	= 512000,
	.n_alarm	= 0,
	.n_ext_ts	= N_EXT_TS,
	.n_per_out	= 0,
	.pps		= 1,
	.adjfreq	= ptp_dpa_adjfreq,
	.adjtime	= ptp_dpa_adjtime,
	.gettime64	= ptp_dpa_gettime,
	.settime64	= ptp_dpa_settime,
	.enable		= ptp_dpa_enable,
};

static int __init __cold dpa_ptp_load(void)
{
	struct device *ptp_dev;
	struct timespec64 now;
	int dpa_phc_index;
	int err;

	if (!(ptp_priv.of_dev && ptp_priv.mac_dev))
		return -ENODEV;

	ptp_dev = &ptp_priv.of_dev->dev;
	mac_dev = ptp_priv.mac_dev;

	if (mac_dev->fm_rtc_get_drift)
		mac_dev->fm_rtc_get_drift(mac_dev->fm_dev, &freqCompensation);

	getnstimeofday64(&now);
	ptp_dpa_settime(&ptp_dpa_caps, &now);

	clock = ptp_clock_register(&ptp_dpa_caps, ptp_dev);
	if (IS_ERR(clock)) {
		err = PTR_ERR(clock);
		return err;
	}
	dpa_phc_index = ptp_clock_index(clock);
	return 0;
}
module_init(dpa_ptp_load);

static void __exit __cold dpa_ptp_unload(void)
{
	if (mac_dev->fm_rtc_disable_interrupt)
		mac_dev->fm_rtc_disable_interrupt(mac_dev->fm_dev, 0xffffffff);
	ptp_clock_unregister(clock);
}
module_exit(dpa_ptp_unload);
