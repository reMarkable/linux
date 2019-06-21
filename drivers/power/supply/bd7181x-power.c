/*
 * bd7181x-power.c
 * @file ROHM BD71815/BD71817 Charger driver
 *
 * Copyright 2014 Embest Technology Co. Ltd. Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*#define DEBUG*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/mfd/bd7181x.h>
#include <linux/delay.h>

#define JITTER_DEFAULT		10000		/* hope 3s is enough */
#define JITTER_REPORT_CAP	10000		/* 10 seconds */
#define CALIB_CURRENT_A2A3 0xCE9E
#define BD7181X_BATTERY_CAP_MAH	1200
#define BD7181X_BATTERY_CAP	mAh_A10s(BD7181X_BATTERY_CAP_MAH)
#define MAX_VOLTAGE		ocv_table[0]
#define MIN_VOLTAGE		3200000
#define THR_VOLTAGE		4100000
#define MAX_CURRENT		1500000		/* uA */
#define AC_NAME			"bd7181x_ac"
#define BAT_NAME		"bd7181x_bat"
#define BD7181X_BATTERY_FULL	100

#define BY_BAT_VOLT		0
#define BY_VBATLOAD_REG		1
#define INIT_COULOMB		BY_VBATLOAD_REG

#define CALIB_CURRENT_A2A3	0xCE9E

//VBAT Low voltage detection Threshold 
#define VBAT_LOW_TH		0x00D4 // 0x00D4*16mV = 212*0.016 = 3.392v 


#define RS_30mOHM		/* This is for 30mOhm sense resistance */

#ifdef RS_30mOHM
#define A10s_mAh(s)		((s) * 1000 / (360 * 3))
#define mAh_A10s(m)		((m) * (360 * 3) / 1000)
#else
#define A10s_mAh(s)		((s) * 1000 / 360)
#define mAh_A10s(m)		((m) * 360 / 1000)
#endif

#define THR_RELAX_CURRENT	5		/* mA */
#define THR_RELAX_TIME		((60 * 60) - 10)	/* sec. */

#define BD7181X_DGRD_CYC_CAP	15	/* 1 micro Ah unit */

#define BD7181X_DGRD_TEMP_M	25	/* 1 degrees C unit */
#define BD7181X_DGRD_TEMP_L	5	/* 1 degrees C unit */
#define BD7181X_DGRD_TEMP_CAP_H	(0)	/* 1 micro Ah unit */
#define BD7181X_DGRD_TEMP_CAP_M	(0)	/* 1 micro Ah unit */
#define BD7181X_DGRD_TEMP_CAP_L	(0)	/* 1 micro Ah unit */

#define CANCEL_ADJ_COULOMB_SOC_H_1	700	/* unit 0.1% */
#define CANCEL_ADJ_COULOMB_SOC_L_1	550	/* unit 0.1% */
#define CANCEL_ADJ_COULOMB_SOC_H_2	350	/* unit 0.1% */
#define CANCEL_ADJ_COULOMB_SOC_L_2	0	/* unit 0.1% */

#define FORCE_ADJ_COULOMB_TEMP_H	35	/* 1 degrees C unit */
#define FORCE_ADJ_COULOMB_TEMP_L	15	/* 1 degrees C unit */

unsigned int battery_cycle;


static int ocv_table[] = {
	4200000,
	4160000,
	4111638,
	4081966,
	4016618,
	3974931,
	3953040,
	3925422,
	3896285,
	3859529,
	3835339,
	3815914,
	3803244,
	3793558,
	3783886,
	3780000,
	3769083,
	3749738,
	3725590,
	3705524,
	3646077,
	3400000,
	2682635
};	/* unit 1 micro V */

static int soc_table[] = {
	1000,
	1000,
	950,
	900,
	850,
	800,
	750,
	700,
	650,
	600,
	550,
	500,
	450,
	400,
	350,
	300,
	250,
	200,
	150,
	100,
	50,
	0,
	-50
	/* unit 0.1% */
};


/** @brief power deivce */
struct bd7181x_power {
	struct device *dev;
	struct bd7181x *mfd;			/**< parent for access register */
	struct power_supply *ac;		/**< alternating current power */
	struct power_supply_desc ac_desc;
	struct power_supply_config ac_cfg;
	struct power_supply *bat;		/**< battery power */
	struct power_supply_desc bat_desc;
	struct power_supply_config bat_cfg;
	struct delayed_work bd_work;		/**< delayed work for timed work */

	int	reg_index;			/**< register address saved for sysfs */

	int    vbus_status;			/**< last vbus status */
	int    charge_status;			/**< last charge status */
	int    bat_status;			/**< last bat status */

	int	hw_ocv1;			/**< HW ocv1 */
	int	hw_ocv2;			/**< HW ocv2 */
	int	bat_online;			/**< battery connect */
	int	charger_online;			/**< charger connect */
	int	vcell;				/**< battery voltage */
	int	vsys;				/**< system voltage */
	int	vcell_min;			/**< minimum battery voltage */
	int	vsys_min;			/**< minimum system voltage */
	int	rpt_status;			/**< battery status report */
	int	prev_rpt_status;		/**< previous battery status report */
	int	bat_health;			/**< battery health */
	int	designed_cap;			/**< battery designed capacity */
	int	full_cap;			/**< battery capacity */
	int	curr;				/**< battery current from DS-ADC */
	int	curr_sar;			/**< battery current from VM_IBAT */
	int	temp;				/**< battery tempature */
	u32	coulomb_cnt;			/**< Coulomb Counter */
	int	state_machine;			/**< initial-procedure state machine */

	u32	soc_org;			/**< State Of Charge using designed capacity without by load */
	u32	soc_norm;			/**< State Of Charge using full capacity without by load */
	u32	soc;				/**< State Of Charge using full capacity with by load */
	u32	clamp_soc;			/**< Clamped State Of Charge using full capacity with by load */

	int	relax_time;			/**< Relax Time */

	u32	cycle;				/**< Charging and Discharging cycle number */
	volatile int calib_current;		/**< calibration current */
};


#define CALIB_NORM			0
#define CALIB_START			1
#define CALIB_GO			2

enum {
	STAT_POWER_ON,
	STAT_INITIALIZED,
};

static int bd7181x_calc_soc_org(struct bd7181x_power* pwr);

/** @brief read a register group once
 *  @param mfd bd7181x device
 *  @param reg	 register address of lower register
 *  @return register value
 */
#ifdef __BD7181X_REGMAP_H__
static u16 bd7181x_reg_read16(struct bd7181x* mfd, int reg) {
	u16 v;

	v = (u16)bd7181x_reg_read(mfd, reg) << 8;
	v |= (u16)bd7181x_reg_read(mfd, reg + 1) << 0;
	return v;
}
#else
static u16 bd7181x_reg_read16(struct bd7181x* mfd, int reg) {
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return be16_to_cpu(u.long_type);
}
#endif

/** @brief write a register group once
 * @param mfd bd7181x device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd7181x_reg_write16(struct bd7181x *mfd, int reg, u16 val) {
	union {
		u16 long_type;
		char chars[2];
	} u;
	int r;

	u.long_type = cpu_to_be16(val);
	// printk("write16 0x%.4X 0x%.4X\n", val, u.long_type);
#ifdef __BD7181X_REGMAP_H__
	r = mfd->write(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return 0;	
}

/** @brief read quad register once
 *  @param mfd bd7181x device
 *  @param reg	 register address of lower register
 *  @return register value
 */
static int bd7181x_reg_read32(struct bd7181x *mfd, int reg) {
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

#ifdef __BD7181X_REGMAP_H__
	r = mfd->read(mfd, reg, sizeof u.chars, u.chars);
#else
	r = regmap_bulk_read(mfd->regmap, reg, u.chars, sizeof u.chars);
#endif
	if (r) {
		return -1;
	}
	return be32_to_cpu(u.long_type);
}

#if 0
/** @brief write quad register once
 * @param mfd bd7181x device
 * @param reg register address of lower register
 * @param val value to write
 * @retval 0 success
 * @retval -1 fail
 */
static int bd7181x_reg_write32(struct bd7181x *mfd, int reg, unsigned val) {
	union {
		u32 long_type;
		char chars[4];
	} u;
	int r;

	u.long_type = cpu_to_be32(val);
	r = regmap_bulk_write(mfd->regmap, reg, u.chars, sizeof u.chars);
	if (r) {
		return -1;
	}
	return 0;
}
#endif

#if INIT_COULOMB == BY_VBATLOAD_REG
/** @brief get initial battery voltage and current
 * @param pwr power device
 * @return 0
 */
static int bd7181x_get_init_bat_stat(struct bd7181x_power *pwr)
{
	struct bd7181x *mfd = pwr->mfd;
	int vcell;

	vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_OCV_PRE_U) * 1000;
	dev_dbg(pwr->dev, "VM_OCV_PRE = %d\n", vcell);
	pwr->hw_ocv1 = vcell;

	vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_OCV_PST_U) * 1000;
	dev_dbg(pwr->dev, "VM_OCV_PST = %d\n", vcell);
	pwr->hw_ocv2 = vcell;

	return 0;
}
#endif

/** @brief get battery average voltage and current
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @param curr  pointer to return back current in unit uA.
 * @return 0
 */
static int bd7181x_get_vbat_curr(struct bd7181x_power *pwr, int *vcell, int *curr) {
	struct bd7181x* mfd = pwr->mfd;
	int tmp_vcell, tmp_curr;

	tmp_vcell = 0;
	tmp_curr = 0;

	tmp_vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_SA_VBAT_U);
	tmp_curr = bd7181x_reg_read16(mfd, BD7181X_REG_VM_SA_IBAT_U);
	if (tmp_curr & IBAT_SA_DIR_Discharging) {
		tmp_curr = -(tmp_curr & ~IBAT_SA_DIR_Discharging);
	}

	*vcell = tmp_vcell * 1000;
#ifdef RS_30mOHM
	*curr = tmp_curr * 1000 / 3;
#else
	*curr = tmp_curr * 1000;
#endif
	return 0;
}

/** @brief get battery current from DS-ADC
 * @param pwr power device
 * @return current in unit uA
 */
static int bd7181x_get_current_ds_adc(struct bd7181x_power *pwr) {
	int r;
	
	r = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_CC_CURCD_U);
	if (r < 0) {
		return 0;
	}
	if (r & CURDIR_Discharging) {
		r = -(r & ~CURDIR_Discharging);
	}
#ifdef RS_30mOHM
	return r * 1000 / 3;
#else
	return r * 1000;
#endif
}

/** @brief get system average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd7181x_get_vsys(struct bd7181x_power *pwr, int *vsys) {
	struct bd7181x* mfd = pwr->mfd;
	int tmp_vsys;

	tmp_vsys = 0;

	tmp_vsys = bd7181x_reg_read16(mfd, BD7181X_REG_VM_SA_VSYS_U);

	*vsys = tmp_vsys * 1000;

	return 0;
}

/** @brief get battery minimum average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd7181x_get_vbat_min(struct bd7181x_power *pwr, int *vcell) {
	struct bd7181x* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_SA_VBAT_MIN_U);
	bd7181x_set_bits(pwr->mfd, BD7181X_REG_VM_SA_MINMAX_CLR, VBAT_SA_MIN_CLR);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get system minimum average voltage
 * @param pwr power device
 * @param vcell pointer to return back voltage in unit uV.
 * @return 0
 */
static int bd7181x_get_vsys_min(struct bd7181x_power *pwr, int *vcell) {
	struct bd7181x* mfd = pwr->mfd;
	int tmp_vcell;

	tmp_vcell = 0;

	tmp_vcell = bd7181x_reg_read16(mfd, BD7181X_REG_VM_SA_VSYS_MIN_U);
	bd7181x_set_bits(pwr->mfd, BD7181X_REG_VM_SA_MINMAX_CLR, VSYS_SA_MIN_CLR);

	*vcell = tmp_vcell * 1000;

	return 0;
}

/** @brief get battery capacity
 * @param ocv open circuit voltage
 * @return capcity in unit 0.1 percent
 */
static int bd7181x_voltage_to_capacity(int ocv) {
	int i = 0;
	int soc;

	if (ocv > ocv_table[0]) {
		soc = soc_table[0];
	} else {
		i = 0;
		while (soc_table[i] != -50) {
			if ((ocv <= ocv_table[i]) && (ocv > ocv_table[i+1])) {
				soc = (soc_table[i] - soc_table[i+1]) * (ocv - ocv_table[i+1]) / (ocv_table[i] - ocv_table[i+1]);
				soc += soc_table[i+1];
				break;
			}
			i++;
		}
		if (soc_table[i] == -50)
			soc = soc_table[i];
	}
	return soc;
}

/** @brief get battery temperature
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd7181x_get_temp(struct bd7181x_power *pwr) {
	struct bd7181x* mfd = pwr->mfd;
	int t;

	t = 200 - (int)bd7181x_reg_read(mfd, BD7181X_REG_VM_BTMP);

	// battery temperature error
	t = (t > 200)? 200: t;
	
	return t;
}

static int bd7181x_reset_coulomb_count(struct bd7181x_power* pwr);

/** @brief get battery charge status
 * @param pwr power device
 * @return temperature in unit deg.Celsius
 */
static int bd7181x_charge_status(struct bd7181x_power *pwr)
{
	u8 state;
	int ret = 1;

	state = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
	dev_dbg(pwr->dev, "%s(): CHG_STATE %d\n", __func__, state);

	switch (state) {
	case 0x00:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x0E:
		pwr->rpt_status = POWER_SUPPLY_STATUS_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x0F:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_FULL;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
	case 0x14:
	case 0x20:
	case 0x21:
	case 0x22:
	case 0x23:
	case 0x24:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	case 0x30:
	case 0x31:
	case 0x32:
	case 0x40:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case 0x7f:
	default:
		ret = 0;
		pwr->rpt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		pwr->bat_health = POWER_SUPPLY_HEALTH_DEAD;
		break;	
	}

	bd7181x_reset_coulomb_count(pwr);

	pwr->prev_rpt_status = pwr->rpt_status;

	return ret;
}

#if INIT_COULOMB == BY_BAT_VOLT
static int bd7181x_calib_voltage(struct bd7181x_power* pwr, int* ocv) {
	int r, curr, volt;

	bd7181x_get_vbat_curr(pwr, &volt, &curr);

	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
	if (r >= 0 && curr > 0) {
		// voltage increment caused by battery inner resistor
		if (r == 3) volt -= 100 * 1000;
		else if (r == 2) volt -= 50 * 1000;
	}
	*ocv = volt;

	return 0;
}
#endif

/** @brief set initial coulomb counter value from battery voltage
 * @param pwr power device
 * @return 0
 */
static int calibration_coulomb_counter(struct bd7181x_power* pwr)
{
	u32 bcap;
	int soc, ocv;

#if INIT_COULOMB == BY_VBATLOAD_REG
	/* Get init OCV by HW */
	bd7181x_get_init_bat_stat(pwr);

	ocv = (pwr->hw_ocv1 >= pwr->hw_ocv2)? pwr->hw_ocv1: pwr->hw_ocv2;
	dev_dbg(pwr->dev, "ocv %d\n", ocv);
#elif INIT_COULOMB == BY_BAT_VOLT
	bd7181x_calib_voltage(pwr, &ocv);
#endif

	/* Get init soc from ocv/soc table */
	soc = bd7181x_voltage_to_capacity(ocv);
	dev_dbg(pwr->dev, "soc %d[0.1%%]\n", soc);
	if (soc < 0)
		soc = 0;
	bcap = pwr->designed_cap * soc / 1000;

	bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_1, 0);
	bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((bcap + pwr->designed_cap / 200) & 0x1FFFUL));

	pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
	dev_dbg(pwr->dev, "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);

	/* Start canceling offset of the DS ADC. This needs 1 second at least */
	bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCCALIB);

	return 0;
}

/** @brief adjust coulomb counter values at relaxed state
 * @param pwr power device
 * @return 0
 */
static int bd7181x_adjust_coulomb_count(struct bd7181x_power* pwr)
{
	u32 relaxed_coulomb_cnt;

	relaxed_coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_REX_CCNTD_3) & 0x1FFFFFFFUL;
	dev_dbg(pwr->dev,  "%s(): relaxed_coulomb_cnt = 0x%x\n", __func__, relaxed_coulomb_cnt);
	if (relaxed_coulomb_cnt != 0) {
		u32 bcap;
		int soc, ocv;
		int diff_coulomb_cnt;

		/* Get OCV at relaxed state by HW */
		ocv = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_REX_SA_VBAT_U) * 1000;
		dev_dbg(pwr->dev,  "ocv %d\n", ocv);

		/* Clear Relaxed Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_REX_CTRL_1, REX_CLR);

		diff_coulomb_cnt = relaxed_coulomb_cnt - (bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL);
		diff_coulomb_cnt = diff_coulomb_cnt >> 16;
		dev_dbg(pwr->dev,  "diff_coulomb_cnt = %d\n", diff_coulomb_cnt);

		/* Get soc at relaxed state from ocv/soc table */
		soc = bd7181x_voltage_to_capacity(ocv);
		dev_dbg(pwr->dev,  "soc %d[0.1%%]\n", soc);
		if (soc < 0)
			soc = 0;

		if ((soc > CANCEL_ADJ_COULOMB_SOC_H_1) || ((soc < CANCEL_ADJ_COULOMB_SOC_L_1) && (soc > CANCEL_ADJ_COULOMB_SOC_H_2)) || (soc < CANCEL_ADJ_COULOMB_SOC_L_2) || 
			((pwr->temp <= FORCE_ADJ_COULOMB_TEMP_H) && (pwr->temp >= FORCE_ADJ_COULOMB_TEMP_L))) {
			bcap = pwr->designed_cap * soc / 1000;

			/* Stop Coulomb Counter */
			bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

			bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_1, 0);
			bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((bcap + pwr->designed_cap / 200) & 0x1FFFUL) + diff_coulomb_cnt);

			pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
			dev_dbg(pwr->dev,  "Adjust Coulomb Counter at Relaxed State\n");
			dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

			/* Start Coulomb Counter */
			bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

			/* If the following commented out code is enabled, the SOC is not clamped at the relax time. */
			/* Reset SOCs */
			/* bd7181x_calc_soc_org(pwr); */
			/* pwr->soc_norm = pwr->soc_org; */
			/* pwr->soc = pwr->soc_norm; */
			/* pwr->clamp_soc = pwr->soc; */
		}
	}

	return 0;
}

/** @brief reset coulomb counter values at full charged state
 * @param pwr power device
 * @return 0
 */
static int bd7181x_reset_coulomb_count(struct bd7181x_power* pwr)
{
	u32 full_charged_coulomb_cnt;

	full_charged_coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_FULL_CCNTD_3) & 0x1FFFFFFFUL;
	dev_dbg(pwr->dev, "%s(): full_charged_coulomb_cnt=0x%x\n", __func__, full_charged_coulomb_cnt);
	if (full_charged_coulomb_cnt != 0) {
		int diff_coulomb_cnt;

		/* Clear Full Charged Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_FULL_CTRL, FULL_CLR);

		diff_coulomb_cnt = full_charged_coulomb_cnt - (bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL);
		diff_coulomb_cnt = diff_coulomb_cnt >> 16;
		if (diff_coulomb_cnt > 0) {
			diff_coulomb_cnt = 0;
		}
		dev_dbg(pwr->dev,  "diff_coulomb_cnt = %d\n", diff_coulomb_cnt);

		/* Stop Coulomb Counter */
		bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

		bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_1, 0);
		bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((pwr->designed_cap + pwr->designed_cap / 200) & 0x1FFFUL) + diff_coulomb_cnt);

		pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
		dev_dbg(pwr->dev,  "Reset Coulomb Counter at POWER_SUPPLY_STATUS_FULL\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);
	}

	return 0;
}

/** @brief get battery parameters, such as voltages, currents, temperatures.
 * @param pwr power device
 * @return 0
 */
static int bd7181x_get_voltage_current(struct bd7181x_power* pwr)
{

	/* Read detailed vcell and current */
	bd7181x_get_vbat_curr(pwr, &pwr->vcell, &pwr->curr_sar);
	dev_dbg(pwr->dev,  "VM_VBAT = %d\n", pwr->vcell);
	dev_dbg(pwr->dev,  "VM_IBAT = %d\n", pwr->curr_sar);

	pwr->curr = bd7181x_get_current_ds_adc(pwr);
	dev_dbg(pwr->dev,  "CC_CURCD = %d\n", pwr->curr);

	/* Read detailed vsys */
	bd7181x_get_vsys(pwr, &pwr->vsys);
	dev_dbg(pwr->dev,  "VM_VSYS = %d\n", pwr->vsys);

	/* Read detailed vbat_min */
	bd7181x_get_vbat_min(pwr, &pwr->vcell_min);
	dev_dbg(pwr->dev,  "VM_VBAT_MIN = %d\n", pwr->vcell_min);

	/* Read detailed vsys_min */
	bd7181x_get_vsys_min(pwr, &pwr->vsys_min);
	dev_dbg(pwr->dev,  "VM_VSYS_MIN = %d\n", pwr->vsys_min);

	/* Get tempature */
	pwr->temp = bd7181x_get_temp(pwr);
	// dev_dbg(pwr->dev,  "Temperature %d degrees C\n", pwr->temp);

	return 0;
}

/** @brief adjust coulomb counter values at relaxed state by SW
 * @param pwr power device
 * @return 0
 */
static int bd7181x_adjust_coulomb_count_sw(struct bd7181x_power* pwr)
{
	int tmp_curr_mA;

	tmp_curr_mA = pwr->curr / 1000;
	if ((tmp_curr_mA * tmp_curr_mA) <= (THR_RELAX_CURRENT * THR_RELAX_CURRENT)) { /* No load */
		pwr->relax_time += (JITTER_DEFAULT / 1000);
	}
	else {
		pwr->relax_time = 0;
	}
	dev_dbg(pwr->dev,  "%s(): pwr->relax_time = 0x%x\n", __func__, pwr->relax_time);
	if (pwr->relax_time >= THR_RELAX_TIME) { /* Battery is relaxed. */
		u32 bcap;
		int soc, ocv;

		pwr->relax_time = 0;

		/* Get OCV */
		ocv = pwr->vcell;

		/* Get soc at relaxed state from ocv/soc table */
		soc = bd7181x_voltage_to_capacity(ocv);
		dev_dbg(pwr->dev,  "soc %d[0.1%%]\n", soc);
		if (soc < 0)
			soc = 0;

		if ((soc > CANCEL_ADJ_COULOMB_SOC_H_1) || ((soc < CANCEL_ADJ_COULOMB_SOC_L_1) && (soc > CANCEL_ADJ_COULOMB_SOC_H_2)) || (soc < CANCEL_ADJ_COULOMB_SOC_L_2) || 
			((pwr->temp <= FORCE_ADJ_COULOMB_TEMP_H) && (pwr->temp >= FORCE_ADJ_COULOMB_TEMP_L))) {
			bcap = pwr->designed_cap * soc / 1000;

			/* Stop Coulomb Counter */
			bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

			bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_1, 0);
			bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((bcap + pwr->designed_cap / 200) & 0x1FFFUL));

			pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
			dev_dbg(pwr->dev,  "Adjust Coulomb Counter by SW at Relaxed State\n");
			dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

			/* Start Coulomb Counter */
			bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

			/* If the following commented out code is enabled, the SOC is not clamped at the relax time. */
			/* Reset SOCs */
			/* bd7181x_calc_soc_org(pwr); */
			/* pwr->soc_norm = pwr->soc_org; */
			/* pwr->soc = pwr->soc_norm; */
			/* pwr->clamp_soc = pwr->soc; */
		}

	}

	return 0;
}

/** @brief get coulomb counter values
 * @param pwr power device
 * @return 0
 */
static int bd7181x_coulomb_count(struct bd7181x_power* pwr)
{
	dev_dbg(pwr->dev, "%s(): pwr->state_machine = 0x%x\n", __func__, pwr->state_machine);
	if (pwr->state_machine == STAT_POWER_ON) {
		pwr->state_machine = STAT_INITIALIZED;
		/* Start Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);
	} else if (pwr->state_machine == STAT_INITIALIZED) {
		pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
		// dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);
	}
	return 0;
}

/** @brief calc cycle
 * @param pwr power device
 * @return 0
 */
static int bd7181x_update_cycle(struct bd7181x_power* pwr)
{
	int charged_coulomb_cnt;

	charged_coulomb_cnt = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_CCNTD_CHG_3);
	dev_dbg(pwr->dev, "%s(): charged_coulomb_cnt = 0x%x\n", __func__, charged_coulomb_cnt);
	if (charged_coulomb_cnt >= pwr->designed_cap) {
		pwr->cycle++;
		dev_dbg(pwr->dev,  "Update cycle = %d\n", pwr->cycle);
		battery_cycle = pwr->cycle;
		charged_coulomb_cnt -= pwr->designed_cap;
		/* Stop Coulomb Counter */
		bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

		bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CCNTD_CHG_3, charged_coulomb_cnt);

		/* Start Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);
	}
	return 0;
}

/** @brief calc full capacity value by Cycle and Temperature
 * @param pwr power device
 * @return 0
 */
static int bd7181x_calc_full_cap(struct bd7181x_power* pwr)
{
	u32 designed_cap_uAh;
	u32 full_cap_uAh;

	/* Calculate full capacity by cycle */
	designed_cap_uAh = A10s_mAh(pwr->designed_cap) * 1000;
	full_cap_uAh = designed_cap_uAh - BD7181X_DGRD_CYC_CAP * pwr->cycle;
	pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	dev_dbg(pwr->dev,  "Calculate full capacity by cycle\n");
	dev_dbg(pwr->dev,  "%s() pwr->full_cap = %d\n", __func__, pwr->full_cap);

	/* Calculate full capacity by temperature */
	dev_dbg(pwr->dev,  "Temperature = %d\n", pwr->temp);
	if (pwr->temp >= BD7181X_DGRD_TEMP_M) {
		full_cap_uAh += (pwr->temp - BD7181X_DGRD_TEMP_M) * BD7181X_DGRD_TEMP_CAP_H;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	else if (pwr->temp >= BD7181X_DGRD_TEMP_L) {
		full_cap_uAh += (pwr->temp - BD7181X_DGRD_TEMP_M) * BD7181X_DGRD_TEMP_CAP_M;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	else {
		full_cap_uAh += (BD7181X_DGRD_TEMP_L - BD7181X_DGRD_TEMP_M) * BD7181X_DGRD_TEMP_CAP_M;
		full_cap_uAh += (pwr->temp - BD7181X_DGRD_TEMP_L) * BD7181X_DGRD_TEMP_CAP_L;
		pwr->full_cap = mAh_A10s(full_cap_uAh / 1000);
	}
	dev_dbg(pwr->dev,  "Calculate full capacity by cycle and temperature\n");
	dev_dbg(pwr->dev,  "%s() pwr->full_cap = %d\n", __func__, pwr->full_cap);

	return 0;
}

/** @brief calculate SOC values by designed capacity
 * @param pwr power device
 * @return 0
 */
static int bd7181x_calc_soc_org(struct bd7181x_power* pwr)
{
	pwr->soc_org = (pwr->coulomb_cnt >> 16) * 100 /  pwr->designed_cap;
	if (pwr->soc_org > 100) {
		pwr->soc_org = 100;
		/* Stop Coulomb Counter */
		bd7181x_clear_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);

		bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_1, 0);
		bd7181x_reg_write16(pwr->mfd, BD7181X_REG_CC_CCNTD_3, ((pwr->designed_cap + pwr->designed_cap / 200) & 0x1FFFUL));

		pwr->coulomb_cnt = bd7181x_reg_read32(pwr->mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
		dev_dbg(pwr->dev,  "Limit Coulomb Counter\n");
		dev_dbg(pwr->dev,  "CC_CCNTD = %d\n", pwr->coulomb_cnt);

		/* Start Coulomb Counter */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_CC_CTRL, CCNTENB);
	}
	dev_dbg(pwr->dev, "%s(): pwr->soc_org = %d\n", __func__, pwr->soc_org);
	return 0;
}

/** @brief calculate SOC values by full capacity
 * @param pwr power device
 * @return 0
 */
static int bd7181x_calc_soc_norm(struct bd7181x_power* pwr)
{
	int lost_cap;
	int mod_coulomb_cnt;

	lost_cap = pwr->designed_cap - pwr->full_cap;
	dev_dbg(pwr->dev,  "%s() lost_cap = %d\n", __func__, lost_cap);
	mod_coulomb_cnt = (pwr->coulomb_cnt >> 16) - lost_cap;
	if ((mod_coulomb_cnt > 0) && (pwr->full_cap > 0)) {
		pwr->soc_norm = mod_coulomb_cnt * 100 /  pwr->full_cap;
	}
	else {
		pwr->soc_norm = 0;
	}
	if (pwr->soc_norm > 100) {
		pwr->soc_norm = 100;
	}
		dev_dbg(pwr->dev,  "%s() pwr->soc_norm = %d\n", __func__, pwr->soc_norm);
	return 0;
}

/** @brief get OCV value by SOC
 * @param pwr power device
 * @return 0
 */
int bd7181x_get_ocv(struct bd7181x_power* pwr, int dsoc) {
	int i = 0;
	int ocv = 0;

	if (dsoc > soc_table[0]) {
		ocv = MAX_VOLTAGE;
	}
	else if (dsoc == 0) {
			ocv = ocv_table[21];
	}
	else {
		i = 0;
		while (i < 22) {
			if ((dsoc <= soc_table[i]) && (dsoc > soc_table[i+1])) {
				ocv = (ocv_table[i] - ocv_table[i+1]) * (dsoc - soc_table[i+1]) / (soc_table[i] - soc_table[i+1]) + ocv_table[i+1];
				break;
			}
			i++;
		}
		if (i == 22)
			ocv = ocv_table[22];
	}
	dev_dbg(pwr->dev,  "%s() ocv = %d\n", __func__, ocv);
	return ocv;
}

/** @brief calculate SOC value by full_capacity and load
 * @param pwr power device
 * @return OCV
 */
static int bd7181x_calc_soc(struct bd7181x_power* pwr) {
	int ocv_table_load[23];

	pwr->soc = pwr->soc_norm;

	switch (pwr->rpt_status) { /* Adjust for 0% between THR_VOLTAGE and MIN_VOLTAGE */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vsys_min <= THR_VOLTAGE) {
			int i;
			int ocv;
			int lost_cap;
			int mod_coulomb_cnt;
			int dsoc;

			lost_cap = pwr->designed_cap - pwr->full_cap;
			mod_coulomb_cnt = (pwr->coulomb_cnt >> 16) - lost_cap;
			dsoc = mod_coulomb_cnt * 1000 /  pwr->full_cap;
			dev_dbg(pwr->dev,  "%s() dsoc = %d\n", __func__, dsoc);
			ocv = bd7181x_get_ocv(pwr, dsoc);
			for (i = 1; i < 23; i++) {
				ocv_table_load[i] = ocv_table[i] - (ocv - pwr->vsys_min);
				if (ocv_table_load[i] <= MIN_VOLTAGE) {
					dev_dbg(pwr->dev,  "%s() ocv_table_load[%d] = %d\n", __func__, i, ocv_table_load[i]);
					break;
				}
			}
			if (i < 23) {
				int j;
				int dv = (ocv_table_load[i-1] - ocv_table_load[i]) / 5;
				int lost_cap2;
				int mod_coulomb_cnt2, mod_full_cap;
				for (j = 1; j < 5; j++){
					if ((ocv_table_load[i] + dv * j) > MIN_VOLTAGE) {
						break;
					}
				}
				lost_cap2 = ((21 - i) * 5 + (j - 1)) * pwr->full_cap / 100;
				dev_dbg(pwr->dev,  "%s() lost_cap2 = %d\n", __func__, lost_cap2);
				mod_coulomb_cnt2 = mod_coulomb_cnt - lost_cap2;
				mod_full_cap = pwr->full_cap - lost_cap2;
				if ((mod_coulomb_cnt2 > 0) && (mod_full_cap > 0)) {
					pwr->soc = mod_coulomb_cnt2 * 100 / mod_full_cap;
				}
				else {
					pwr->soc = 0;
				}
				dev_dbg(pwr->dev,  "%s() pwr->soc(by load) = %d\n", __func__, pwr->soc);
			}
		}
		break;
	default:
		break;
	}

	switch (pwr->rpt_status) {/* Adjust for 0% and 100% */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->vsys_min <= MIN_VOLTAGE) {
			pwr->soc = 0;
		}
		else {
			if (pwr->soc == 0) {
				pwr->soc = 1;
			}
		}
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		if (pwr->soc == 100) {
			pwr->soc = 99;
		}
		break;
	default:
		break;
	}
	dev_dbg(pwr->dev,  "%s() pwr->soc = %d\n", __func__, pwr->soc);
	return 0;
}

/** @brief calculate Clamped SOC value by full_capacity and load
 * @param pwr power device
 * @return OCV
 */
static int bd7181x_calc_soc_clamp(struct bd7181x_power* pwr) {
	switch (pwr->rpt_status) {/* Adjust for 0% and 100% */
	case POWER_SUPPLY_STATUS_DISCHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (pwr->soc <= pwr->clamp_soc) {
			pwr->clamp_soc = pwr->soc;
		}
		break;
	default:
		pwr->clamp_soc = pwr->soc;
		break;
	}
	dev_dbg(pwr->dev,  "%s() pwr->clamp_soc = %d\n", __func__, pwr->clamp_soc);
	return 0;
}

/** @brief get battery and DC online status
 * @param pwr power device
 * @return 0
 */
static int bd7181x_get_online(struct bd7181x_power* pwr)
{
	int r;

#if 0
#define TS_THRESHOLD_VOLT	0xD9
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_VM_VTH);
	pwr->bat_online = (r > TS_THRESHOLD_VOLT);
#endif
#if 0
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
	if (r >= 0 && (r & BAT_DET_DONE)) {
		pwr->bat_online = (r & BAT_DET) != 0;
	}
#endif
#if 1
#define BAT_OPEN	0x7
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_TEMP);
	pwr->bat_online = (r != BAT_OPEN);
#endif	
	r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_DCIN_STAT);
	if (r >= 0) {
		pwr->charger_online = (r & VBUS_DET) != 0;
	}
	dev_dbg(pwr->dev, "%s(): pwr->bat_online = %d, pwr->charger_online = %d\n", __func__, pwr->bat_online, pwr->charger_online);
	return 0;
}

/** @brief init bd7181x sub module charger
 * @param pwr power device
 * @return 0
 */
static int bd7181x_init_hardware(struct bd7181x_power *pwr)
{
	struct bd7181x *mfd = pwr->mfd;
	int r;

	r = bd7181x_reg_write(mfd, BD7181X_REG_DCIN_CLPS, 0x36);

#define XSTB		0x02
	r = bd7181x_reg_read(mfd, BD7181X_REG_CONF);

#if 0
	for (i = 0; i < 300; i++) {
		r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
		if (r >= 0 && (r & BAT_DET_DONE)) {
			break;
		}
		msleep(5);
	}
#endif
	if ((r & XSTB) == 0x00) {
	//if (r & BAT_DET) {
		/* Init HW, when the battery is inserted. */

		bd7181x_reg_write(mfd, BD7181X_REG_CONF, r | XSTB);

#define TEST_SEQ_00		0x00
#define TEST_SEQ_01		0x76
#define TEST_SEQ_02		0x66
#define TEST_SEQ_03		0x56
#if 0
		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_01);
		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_02);
		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_03);
		bd7181x_reg_write16(pwr->mfd, 0xA2, CALIB_CURRENT_A2A3);
		bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);
#endif

		/* Stop Coulomb Counter */
		bd7181x_clear_bits(mfd, BD7181X_REG_CC_CTRL, CCNTENB);

		/* Set Coulomb Counter Reset bit*/
		bd7181x_set_bits(mfd, BD7181X_REG_CC_CTRL, CCNTRST);

		/* Clear Coulomb Counter Reset bit*/
		bd7181x_clear_bits(mfd, BD7181X_REG_CC_CTRL, CCNTRST);

		/* Set default Battery Capacity */
		pwr->designed_cap = BD7181X_BATTERY_CAP;
		pwr->full_cap = BD7181X_BATTERY_CAP;

		/* Set initial Coulomb Counter by HW OCV */
		calibration_coulomb_counter(pwr);

		/* WDT_FST auto set */
		bd7181x_set_bits(mfd, BD7181X_REG_CHG_SET1, WDT_AUTO);

		/* VBAT Low voltage detection Setting, added by John Zhang*/
		bd7181x_reg_write16(mfd, BD7181X_REG_ALM_VBAT_TH_U, VBAT_LOW_TH); 

		/* Mask Relax decision by PMU STATE */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_REX_CTRL_1, REX_PMU_STATE_MASK);

		/* Set Battery Capacity Monitor threshold1 as 90% */
		bd7181x_reg_write16(mfd, BD7181X_REG_CC_BATCAP1_TH_U, (BD7181X_BATTERY_CAP * 9 / 10)); 
		dev_dbg(pwr->dev,  "BD7181X_REG_CC_BATCAP1_TH = %d\n", (BD7181X_BATTERY_CAP * 9 / 10));

		/* Enable LED ON when charging */
		bd7181x_set_bits(pwr->mfd, BD7181X_REG_LED_CTRL, CHGDONE_LED_EN);

		pwr->state_machine = STAT_POWER_ON;
	} else {
		pwr->designed_cap = BD7181X_BATTERY_CAP;
		pwr->full_cap = BD7181X_BATTERY_CAP;	// bd7181x_reg_read16(pwr->mfd, BD7181X_REG_CC_BATCAP_U);
		pwr->state_machine = STAT_INITIALIZED;	// STAT_INITIALIZED
	}

	pwr->temp = bd7181x_get_temp(pwr);
	dev_dbg(pwr->dev,  "Temperature = %d\n", pwr->temp);
	bd7181x_adjust_coulomb_count(pwr);
	bd7181x_reset_coulomb_count(pwr);
	pwr->coulomb_cnt = bd7181x_reg_read32(mfd, BD7181X_REG_CC_CCNTD_3) & 0x1FFFFFFFUL;
	bd7181x_calc_soc_org(pwr);
	pwr->soc_norm = pwr->soc_org;
	pwr->soc = pwr->soc_norm;
	pwr->clamp_soc = pwr->soc;
	dev_dbg(pwr->dev,  "%s() CC_CCNTD = %d\n", __func__, pwr->coulomb_cnt);
	dev_dbg(pwr->dev,  "%s() pwr->soc = %d\n", __func__, pwr->soc);
	dev_dbg(pwr->dev,  "%s() pwr->clamp_soc = %d\n", __func__, pwr->clamp_soc);

	pwr->cycle = battery_cycle;
	pwr->curr = 0;
	pwr->curr_sar = 0;
	pwr->relax_time = 0;

	return 0;
}

/**@brief timed work function called by system
 *  read battery capacity,
 *  sense change of charge status, etc.
 * @param work work struct
 * @return  void
 */

static void bd_work_callback(struct work_struct *work)
{
	struct bd7181x_power *pwr;
	struct delayed_work *delayed_work;
	int status, changed = 0;
	static int cap_counter = 0;

	delayed_work = container_of(work, struct delayed_work, work);
	pwr = container_of(delayed_work, struct bd7181x_power, bd_work);

	dev_dbg(pwr->dev, "%s(): in\n", __func__);
	status = bd7181x_reg_read(pwr->mfd, BD7181X_REG_DCIN_STAT);
	if (status != pwr->vbus_status) {
		//printk("DCIN_STAT CHANGED from 0x%X to 0x%X\n", pwr->vbus_status, status);
		pwr->vbus_status = status;
		changed = 1;
	}

	status = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
	status &= ~BAT_DET_DONE;
	if (status != pwr->bat_status) {
		dev_dbg(pwr->dev, "BAT_STAT CHANGED from 0x%X to 0x%X\n", pwr->bat_status, status);
		pwr->bat_status = status;
		changed = 1;
	}

	status = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
	if (status != pwr->charge_status) {
		dev_dbg(pwr->dev, "CHG_STATE CHANGED from 0x%X to 0x%X\n", pwr->charge_status, status);
		pwr->charge_status = status;
		//changed = 1;
	}

	bd7181x_get_voltage_current(pwr);
	bd7181x_adjust_coulomb_count(pwr);
	bd7181x_reset_coulomb_count(pwr);
	bd7181x_adjust_coulomb_count_sw(pwr);
	bd7181x_coulomb_count(pwr);
	bd7181x_update_cycle(pwr);
	bd7181x_calc_full_cap(pwr);
	bd7181x_calc_soc_org(pwr);
	bd7181x_calc_soc_norm(pwr);
	bd7181x_calc_soc(pwr);
	bd7181x_calc_soc_clamp(pwr);
	bd7181x_get_online(pwr);
	bd7181x_charge_status(pwr);

	if (changed || cap_counter++ > JITTER_REPORT_CAP / JITTER_DEFAULT) {
		power_supply_changed(pwr->ac);
		power_supply_changed(pwr->bat);
		cap_counter = 0;
	}

	if (pwr->calib_current == CALIB_NORM) {
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(JITTER_DEFAULT));
	} else if (pwr->calib_current == CALIB_START) {
		pwr->calib_current = CALIB_GO;
	}
}

/**@brief bd7181x power interrupt
 * @param irq system irq
 * @param pwrsys bd7181x power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd7181x_power_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd7181x *mfd = dev_get_drvdata(dev->parent);
	// struct bd7181x_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	dev_info(mfd->dev, "bd7181x_power_interrupt() in.\n");
	
	reg = bd7181x_reg_read(mfd, BD7181X_REG_INT_STAT_03);
	if (reg < 0)
		return IRQ_NONE;

	dev_info(mfd->dev, "INT_STAT_03 = 0x%.2X\n", reg);
	if(reg & POWERON_PRESS)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_ONLINE); 
		dev_info(mfd->dev, "POWERON_PRESS\n");
	}
	if(reg & POWERON_SHORT)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_OFFLINE);
		dev_info(mfd->dev, "POWERON_SHORT\n");
	}
	if(reg & POWERON_MID)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_OFFLINE); 
		dev_info(mfd->dev, "POWERON_MID\n");
	}
	if(reg & POWERON_LONG)
	{
		kobject_uevent(&(mfd->dev->kobj), KOBJ_OFFLINE);
		dev_info(mfd->dev, "POWERON_LONG\n");
	}

	r = bd7181x_reg_write(mfd, BD7181X_REG_INT_STAT_03, reg);
	if (r)
		return IRQ_NONE;

	if (reg & DCIN_MON_DET) {
		dev_info(mfd->dev, "\n~~~DCIN removed\n");
	} else if (reg & DCIN_MON_RES) {
		dev_info(mfd->dev, "\n~~~DCIN inserted\n");
	}

	return IRQ_HANDLED;
}

/**@brief bd7181x vbat low voltage detection interrupt
 * @param irq system irq
 * @param pwrsys bd7181x power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added by John Zhang at 2015-07-22
 */
static irqreturn_t bd7181x_vbat_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd7181x *mfd = dev_get_drvdata(dev->parent);
	// struct bd7181x_power *pwr = dev_get_drvdata(dev);
	int reg, r;

	dev_info(mfd->dev, "bd7181x_vbat_interrupt() in.\n");
	
	reg = bd7181x_reg_read(mfd, BD7181X_REG_INT_STAT_08);
	if (reg < 0)
		return IRQ_NONE;

	dev_info(mfd->dev, "INT_STAT_08 = 0x%.2X\n", reg);

	r = bd7181x_reg_write(mfd, BD7181X_REG_INT_STAT_08, reg);
	if (r)
		return IRQ_NONE;

	if (reg & VBAT_MON_DET) {
		dev_info(mfd->dev, "\n~~~ VBAT LOW Detected ... \n");
		
	} else if (reg & VBAT_MON_RES) {
		dev_info(mfd->dev, "\n~~~ VBAT LOW Resumed ... \n");
	}

	return IRQ_HANDLED;
	
}

/**@brief bd7181x int_stat_11 detection interrupt
 * @param irq system irq
 * @param pwrsys bd7181x power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 * added 2015-12-26
 */
static irqreturn_t bd7181x_int_11_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct bd7181x *mfd = dev_get_drvdata(dev->parent);
	// struct bd7181x_power *pwr = dev_get_drvdata(dev);
	int reg, r;
	
	dev_info(mfd->dev, "bd7181x_int_11_interrupt() in.\n");
	
	reg = bd7181x_reg_read(mfd, BD7181X_REG_INT_STAT_11);
	if (reg < 0)
		return IRQ_NONE;

	dev_info(mfd->dev, "INT_STAT_11 = 0x%.2X\n", reg);

	r = bd7181x_reg_write(mfd, BD7181X_REG_INT_STAT_11, reg);
	if (r) {
		return IRQ_NONE;
	}

	if (reg & INT_STAT_11_VF_DET) {
		dev_info(mfd->dev, "\n~~~ VF Detected ... \n");
	} else if (reg & INT_STAT_11_VF_RES) {
		dev_info(mfd->dev, "\n~~~ VF Resumed ... \n");
	} else if (reg & INT_STAT_11_VF125_DET) {
		dev_info(mfd->dev, "\n~~~ VF125 Detected ... \n");
	} else if (reg & INT_STAT_11_VF125_RES) {
		dev_info(mfd->dev, "\n~~~ VF125 Resumed ... \n");
	} else if (reg & INT_STAT_11_OVTMP_DET) {
		dev_info(mfd->dev, "\n~~~ Overtemp Detected ... \n");
	} else if (reg & INT_STAT_11_OVTMP_RES) {
		dev_info(mfd->dev, "\n~~~ Overtemp Detected ... \n");
	} else if (reg & INT_STAT_11_LOTMP_DET) {
		dev_info(mfd->dev, "\n~~~ Lowtemp Detected ... \n");
	} else if (reg & INT_STAT_11_LOTMP_RES) {
		dev_info(mfd->dev, "\n~~~ Lowtemp Detected ... \n");
	}

	return IRQ_HANDLED;

}

/** @brief get property of power supply ac
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */
static int bd7181x_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd7181x_power *pwr = dev_get_drvdata(psy->dev.parent);
	u32 vot;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->charger_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_VM_DCIN_U);
		val->intval = 5000 * vot;		// 5 milli volt steps
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief get property of power supply bat
 *  @param psy power supply deivce
 *  @param psp property to get
 *  @param val property value to return
 *  @retval 0  success
 *  @retval negative fail
 */

static int bd7181x_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp, union power_supply_propval *val)
{
	struct bd7181x_power *pwr = dev_get_drvdata(psy->dev.parent);
	// u32 cap, vot, r;
	// u8 ret;

	switch (psp) {
	/*
	case POWER_SUPPLY_PROP_STATUS:
		r = bd7181x_reg_read(pwr->mfd, BD7181X_REG_CHG_STATE);
		// printk("CHG_STATE = 0x%.2X\n", r);
		switch(r) {
		case CHG_STATE_SUSPEND:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case CHG_STATE_TRICKLE_CHARGE:
		case CHG_STATE_PRE_CHARGE:
		case CHG_STATE_FAST_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case CHG_STATE_TOP_OFF:
		case CHG_STATE_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bd7181x_reg_read(pwr->mfd, BD7181X_REG_BAT_STAT);
		if (ret & DBAT_DET)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else if (ret & VBAT_OV)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		cap = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_CC_BATCAP_U);
		// printk("CC_BATCAP = 0x%.4X\n", cap);
		val->intval = cap * 100 / 0x1FFF;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		vot = bd7181x_reg_read16(pwr->mfd, BD7181X_REG_VM_VBAT_U) * 1000;
		val->intval = vot;
		break;
	*/
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pwr->rpt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pwr->bat_health;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (pwr->rpt_status == POWER_SUPPLY_STATUS_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pwr->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = pwr->clamp_soc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		{
		u32 t;

		t = pwr->coulomb_cnt >> 16;
		t = A10s_mAh(t);
		if (t > A10s_mAh(pwr->designed_cap)) t = A10s_mAh(pwr->designed_cap);
		val->intval = t * 1000;		/* uA to report */
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pwr->bat_online;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = BD7181X_BATTERY_FULL * A10s_mAh(pwr->designed_cap) * 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = BD7181X_BATTERY_FULL * A10s_mAh(pwr->full_cap) * 10;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = pwr->curr_sar;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = pwr->curr;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pwr->temp * 10; /* 0.1 degrees C unit */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = MAX_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = MIN_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = MAX_CURRENT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/** @brief ac properties */
static enum power_supply_property bd7181x_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/** @brief bat properies */
static enum power_supply_property bd7181x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

/** @brief directly set raw value to chip register, format: 'register value' */
static ssize_t bd7181x_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd7181x_power *pwr = container_of(&psy, struct bd7181x_power, bat);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pwr->reg_index = -1;
		dev_err(pwr->dev, "registers set: <reg> <value>\n");
		return count;
	}

	if (ret == 1 && reg <= BD7181X_MAX_REGISTER) {
		pwr->reg_index = reg;
		dev_info(pwr->dev, "registers set: reg=0x%x\n", reg);
		return count;
	}

	if (reg > BD7181X_MAX_REGISTER || val > 255)
		return -EINVAL;

	dev_info(pwr->dev, "registers set: reg=0x%x, val=0x%x\n", reg, val);
	ret = bd7181x_reg_write(pwr->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/** @brief print value of chip register, format: 'register=value' */
static ssize_t bd7181x_sysfs_print_reg(struct bd7181x_power *pwr,
				       u8 reg,
				       char *buf)
{
	int ret = bd7181x_reg_read(pwr->mfd, reg);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/** @brief show all raw values of chip register, format per line: 'register=value' */
static ssize_t bd7181x_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd7181x_power *pwr = container_of(&psy, struct bd7181x_power, bat);
	ssize_t ret = 0;
	int i;

	dev_info(pwr->dev, "register: index[%d]\n", pwr->reg_index);
	if (pwr->reg_index >= 0) {
		ret += bd7181x_sysfs_print_reg(pwr, pwr->reg_index, buf + ret);
	} else {
		for (i = 0; i <= BD7181X_MAX_REGISTER; i++) {
			ret += bd7181x_sysfs_print_reg(pwr, i, buf + ret);
		}
	}
	return ret;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		bd7181x_sysfs_show_registers, bd7181x_sysfs_set_registers);

static int first_offset(struct bd7181x_power *pwr)
{
	unsigned char ra2, ra3, ra6, ra7;
	unsigned char ra2_temp;
	struct bd7181x *mfd = pwr->mfd;

	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_01);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_02);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_03);


	ra2 = bd7181x_reg_read(mfd, 0xA2);	// I want to know initial A2 & A3.
	ra3 = bd7181x_reg_read(mfd, 0xA3);	// I want to know initial A2 & A3.
	ra6 = bd7181x_reg_read(mfd, 0xA6);
	ra7 = bd7181x_reg_read(mfd, 0xA7);

	bd7181x_reg_write(mfd, 0xA2, 0x00);
	bd7181x_reg_write(mfd, 0xA3, 0x00);

	dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2);
	dev_info(pwr->dev, "TEST[A3] = 0x%.2X\n", ra3);
	dev_info(pwr->dev, "TEST[A6] = 0x%.2X\n", ra6);
	dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);

	//-------------- First Step -------------------
	dev_info(pwr->dev, "Frist Step begginning \n");

	// delay some time , Make a state of IBAT=0mA
	// mdelay(1000 * 10);

	ra2_temp = ra2;

	if (ra7 != 0) {
		//if 0<0xA7<20 decrease the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		if ((ra7 > 0) && (ra7 < 20)) {
			do {
				ra2 = bd7181x_reg_read(mfd, 0xA2);
				ra2_temp = ra2 >> 3;
				ra2_temp -= 1;
				ra2_temp <<= 3;
				bd7181x_reg_write(mfd, 0xA2, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd7181x_reg_read(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");

		}		// end if((ra7 > 0)&&(ra7 < 20)) 
		else if ((ra7 > 0xDF) && (ra7 < 0xFF))
			//if DF<0xA7<FF increase the Test register 0xA2[7:3] until 0xA7 becomes 0x00.
		{
			do {
				ra2 = bd7181x_reg_read(mfd, 0xA2);
				ra2_temp = ra2 >> 3;
				ra2_temp += 1;
				ra2_temp <<= 3;

				bd7181x_reg_write(mfd, 0xA2, ra2_temp);
				dev_info(pwr->dev, "TEST[A2] = 0x%.2X\n", ra2_temp);

				ra7 = bd7181x_reg_read(mfd, 0xA7);
				dev_info(pwr->dev, "TEST[A7] = 0x%.2X\n", ra7);
				mdelay(1000);	// 1sec?                           
			} while (ra7);

			dev_info(pwr->dev, "A7 becomes 0 . \n");
		}
	}

	// please use "ra2_temp" at step2.
	return ra2_temp;
}

static int second_step(struct bd7181x_power *pwr, u8 ra2_temp)
{
	u16 ra6, ra7;
	u8 aft_ra2, aft_ra3;
	u8 r79, r7a;
	unsigned int LNRDSA_FUSE;
	long ADC_SIGN;
	long DSADGAIN1_INI;
	struct bd7181x *mfd = pwr->mfd;

	//-------------- Second Step -------------------
	dev_info(pwr->dev, "Second Step begginning \n");

	// need to change boad setting ( input 1A tio 10mohm)
	// delay some time , Make a state of IBAT=1000mA
	// mdelay(1000 * 10);

// rough adjust
	dev_info(pwr->dev, "ra2_temp = 0x%.2X\n", ra2_temp);

	ra6 = bd7181x_reg_read(mfd, 0xA6);
	ra7 = bd7181x_reg_read(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

	bd7181x_reg_write(mfd, 0xA2, ra2_temp);	// this value from step1
	bd7181x_reg_write(mfd, 0xA3, 0x00);

	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);

	r79 = bd7181x_reg_read(mfd, 0x79);
	r7a = bd7181x_reg_read(mfd, 0x7A);

	ADC_SIGN = r79 >> 7;
	ADC_SIGN = 1 - (2 * ADC_SIGN);
	DSADGAIN1_INI = r79 << 8;
	DSADGAIN1_INI = DSADGAIN1_INI + r7a;
	DSADGAIN1_INI = DSADGAIN1_INI & 0x7FFF;
	DSADGAIN1_INI = DSADGAIN1_INI * ADC_SIGN; //  unit 0.001

	// unit 0.000001
	DSADGAIN1_INI *= 1000;
	{
	if (DSADGAIN1_INI > 1000001) {
		DSADGAIN1_INI = 2048000000UL - (DSADGAIN1_INI - 1000000) * 8187;
	} else if (DSADGAIN1_INI < 999999) {
		DSADGAIN1_INI = -(DSADGAIN1_INI - 1000000) * 8187;
	} else {
		DSADGAIN1_INI = 0;
	}
	}

	LNRDSA_FUSE = (int) DSADGAIN1_INI / 1000000;

	dev_info(pwr->dev, "LNRDSA_FUSE = 0x%.8X\n", LNRDSA_FUSE);

	aft_ra2 = (LNRDSA_FUSE >> 8) & 255;
	aft_ra3 = (LNRDSA_FUSE) & 255;

	aft_ra2 = aft_ra2 + ra2_temp;

	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_01);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_02);
	bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_03);

	bd7181x_reg_write(mfd, 0xA2, aft_ra2);
	bd7181x_reg_write(mfd, 0xA3, aft_ra3);

	return 0;
}

static int third_step(struct bd7181x_power *pwr, unsigned thr) {
	u16 ra2_a3, ra6, ra7;
	u8 ra2, ra3;
	u8 aft_ra2, aft_ra3;
	struct bd7181x *mfd = pwr->mfd;

// fine adjust
	ra2 = bd7181x_reg_read(mfd, 0xA2);	//
	ra3 = bd7181x_reg_read(mfd, 0xA3);	//

	ra6 = bd7181x_reg_read(mfd, 0xA6);
	ra7 = bd7181x_reg_read(mfd, 0xA7);
	ra6 <<= 8;
	ra6 |= ra7;		// [0xA6 0xA7]
	dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);


	if (ra6 > thr) {
		do {
			ra2_a3 = bd7181x_reg_read(mfd, 0xA2);
			ra2_a3 <<= 8;
			ra3 = bd7181x_reg_read(mfd, 0xA3);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 -= 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA3, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA2, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd7181x_reg_read(mfd, 0xA6);
			ra7 = bd7181x_reg_read(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);
		} while (ra6 > thr);
	} else if (ra6 < thr) {
		do {
			ra2_a3 = bd7181x_reg_read(mfd, 0xA2);
			ra2_a3 <<= 8;
			ra3 = bd7181x_reg_read(mfd, 0xA3);
			ra2_a3 |= ra3;
			//ra2_a3 >>= 3; // ? 0xA3[7:3] , or 0xA3[7:0]

			ra2_a3 += 1;
			//ra2_a3 <<= 3;
			ra3 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA3, ra3);

			ra2_a3 >>= 8;
			ra2 = ra2_a3;
			bd7181x_reg_write(mfd, 0xA2, ra2);

			dev_info(pwr->dev, "TEST[A2] = 0x%.2X , TEST[A3] = 0x%.2X \n", ra2, ra3);

			mdelay(1000);	// 1sec?

			ra6 = bd7181x_reg_read(mfd, 0xA6);
			ra7 = bd7181x_reg_read(mfd, 0xA7);
			ra6 <<= 8;
			ra6 |= ra7;	// [0xA6 0xA7]
			dev_info(pwr->dev, "TEST[A6,A7] = 0x%.4X\n", ra6);

		} while (ra6 < thr);
	}

	dev_info(pwr->dev, "[0xA6 0xA7] becomes [0x%.4X] . \n", thr);
	dev_info(pwr->dev, " Calibation finished ... \n\n");

	aft_ra2 = bd7181x_reg_read(mfd, 0xA2);	// 
	aft_ra3 = bd7181x_reg_read(mfd, 0xA3);	// I want to know initial A2 & A3.

	dev_info(pwr->dev, "TEST[A2,A3] = 0x%.2X%.2X\n", aft_ra2, aft_ra3);

	// bd7181x_reg_write(mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);

	return 0;
}

static ssize_t bd7181x_sysfs_set_calibrate(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bd7181x_power *pwr = container_of(&psy, struct bd7181x_power, bat);
	ssize_t ret = 0;
	unsigned int val, mA;
	static u8 rA2;

	ret = sscanf(buf, "%d %d", &val, &mA);
	if (ret < 1) {
		dev_err(pwr->dev, "error: write a integer string");
		return count;
	}

	if (val == 1) {
		pwr->calib_current = CALIB_START;
		while (pwr->calib_current != CALIB_GO) {
			msleep(500);
		}
		rA2 = first_offset(pwr);
	}
	if (val == 2) {
		second_step(pwr, rA2);
	}
	if (val == 3) {
		if (ret <= 1) {
			dev_err(pwr->dev, "error: Fine adjust need a mA argument!");
		} else {
		unsigned int ra6_thr;

		ra6_thr = mA * 0xFFFF / 20000;
		dev_info(pwr->dev, "Fine adjust at %d mA, ra6 threshold %d(0x%X)\n", mA, ra6_thr, ra6_thr);
		third_step(pwr, ra6_thr);
		}
	}
	if (val == 4) {
		bd7181x_reg_write(pwr->mfd, BD7181X_REG_TEST_MODE, TEST_SEQ_00);
		pwr->calib_current = CALIB_NORM;
		schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));
	}

	return count;
}

static ssize_t bd7181x_sysfs_show_calibrate(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	// struct power_supply *psy = dev_get_drvdata(dev);
	// struct bd7181x_power *pwr = container_of(psy, struct bd7181x_power, bat);
	ssize_t ret = 0;

	ret = 0;
	ret += sprintf(buf + ret, "write string value\n"
		"\t1      0 mA for step one\n"
		"\t2      1000 mA for rough adjust\n"
		"\t3 <mA> for fine adjust\n"
		"\t4      exit current calibration\n");
	return ret;
}

static DEVICE_ATTR(calibrate, S_IWUSR | S_IRUGO,
		bd7181x_sysfs_show_calibrate, bd7181x_sysfs_set_calibrate);

static struct attribute *bd7181x_sysfs_attributes[] = {
	/*
	 * TODO: some (appropriate) of these attrs should be switched to
	 * use pwr supply class props.
	 */
	&dev_attr_registers.attr,
	&dev_attr_calibrate.attr,
	NULL,
};

static const struct attribute_group bd7181x_sysfs_attr_group = {
	.attrs = bd7181x_sysfs_attributes,
};

/** @brief powers supplied by bd7181x_ac */
static char *bd7181x_ac_supplied_to[] = {
	BAT_NAME,
};

/** @brief probe pwr device 
 * @param pdev platform deivce of bd7181x_power
 * @retval 0 success
 * @retval negative fail
 */
static int __init bd7181x_power_probe(struct platform_device *pdev)
{
	struct bd7181x *bd7181x = dev_get_drvdata(pdev->dev.parent);
	struct bd7181x_power *pwr;
	int irq, ret, reg;

	pwr = kzalloc(sizeof(*pwr), GFP_KERNEL);
	if (pwr == NULL)
		return -ENOMEM;

	pwr->dev = &pdev->dev;
	pwr->mfd = bd7181x;

	platform_set_drvdata(pdev, pwr);

	if (battery_cycle <= 0) {
		battery_cycle = 0;
	}
	dev_err(pwr->dev, "battery_cycle = %d\n", battery_cycle);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to enable Coulomb Counter using following commented out code */
	/* for counting Coulomb when the product is power up(including sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power up time. */
	/* (3) Kernel must call this routin at charging time. */
	/* (4) Must use this code with "Stop Coulomb Counter" code in bd7181x_power_remove() function */
	/* Start Coulomb Counter */
	/* bd7181x_set_bits(pwr->mfd, BD7181x_REG_CC_CTRL, CCNTENB); */

	bd7181x_init_hardware(pwr);

	pwr->bat_desc.name = BAT_NAME;
	pwr->bat_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	pwr->bat_desc.properties = bd7181x_battery_props;
	pwr->bat_desc.num_properties = ARRAY_SIZE(bd7181x_battery_props);
	pwr->bat_desc.get_property = bd7181x_battery_get_property;

	pwr->bat = power_supply_register(&pdev->dev, &pwr->bat_desc, &pwr->bat_cfg);
	if (IS_ERR(pwr->bat)) {
		dev_err(&pdev->dev, "Failed to register battery, %ld\n", PTR_ERR(pwr->bat));
		ret = PTR_ERR(pwr->bat);
		pwr->bat = NULL;
		goto fail_register_bat;
	}

	pwr->ac_desc.name = AC_NAME;
	pwr->ac_desc.type = POWER_SUPPLY_TYPE_MAINS;
	pwr->ac_desc.properties = bd7181x_charger_props;
	pwr->ac_cfg.supplied_to = bd7181x_ac_supplied_to;
	pwr->ac_cfg.num_supplicants = ARRAY_SIZE(bd7181x_ac_supplied_to);
	pwr->ac_desc.num_properties = ARRAY_SIZE(bd7181x_charger_props);
	pwr->ac_desc.get_property = bd7181x_charger_get_property;

	pwr->ac = power_supply_register(&pdev->dev, &pwr->ac_desc, &pwr->ac_cfg);
	if (IS_ERR(pwr->ac)) {
		dev_err(&pdev->dev, "Failed to register ac, %ld\n", PTR_ERR(pwr->ac));
		ret = PTR_ERR(pwr->ac);
		pwr->ac = NULL;
		goto fail_register_ac;
	}

	/*Add DC_IN Inserted and Remove ISR */
	irq  = platform_get_irq(pdev, 0); // get irq number 
#ifdef __BD7181X_REGMAP_H__
	irq += bd7181x->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd7181x_power_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/*add VBAT Low Voltage detection, John Zhang*/
	irq  = platform_get_irq(pdev, 1);
#ifdef __BD7181X_REGMAP_H__
	irq += bd7181x->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd7181x_vbat_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* add INT_STAT_11 */
	irq  = platform_get_irq(pdev, 2);
#ifdef __BD7181X_REGMAP_H__
	irq += bd7181x->irq_base;
#endif
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd7181x_int_11_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);
	}

	/* Enable INT_11 */
	ret = bd7181x_reg_write(bd7181x, BD7181X_REG_INT_EN_11, 0xFF);
	if (ret < 0) {
		dev_warn(&pdev->dev, "Write BD7181X_REG_INT_EN_11 failed\n");
	}
	reg = bd7181x_reg_read(bd7181x, BD7181X_REG_INT_EN_11);
	if (reg < 0) {
		dev_warn(&pdev->dev, "Read BD7181X_REG_INT_EN_11 failed\n");
	}
	dev_info(&pdev->dev, "BD7181X_REG_INT_EN_11=0x%x\n", reg);


	ret = sysfs_create_group(&pwr->bat->dev.kobj, &bd7181x_sysfs_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register sysfs interface\n");
	}

	pwr->reg_index = -1;

	INIT_DELAYED_WORK(&pwr->bd_work, bd_work_callback);

	/* Schedule timer to check current status */
	pwr->calib_current = CALIB_NORM;
	schedule_delayed_work(&pwr->bd_work, msecs_to_jiffies(0));

	return 0;

//error_exit:
	power_supply_unregister(pwr->ac);
fail_register_ac:
	power_supply_unregister(pwr->bat);
fail_register_bat:
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return ret;
}

/** @brief remove pwr device
 * @param pdev platform deivce of bd7181x_power
 * @return 0
 */

static int __exit bd7181x_power_remove(struct platform_device *pdev)
{
	struct bd7181x_power *pwr = platform_get_drvdata(pdev);

	/* If the product often power up/down and the power down time is long, the Coulomb Counter may have a drift. */
	/* If so, it may be better accuracy to disable Coulomb Counter using following commented out code */
	/* for stopping counting Coulomb when the product is power down(without sleep). */
	/* The condition  */
	/* (1) Product often power up and down, the power down time is long and there is no power consumed in power down time. */
	/* (2) Kernel must call this routin at power down time. */
	/* (3) Must use this code with "Start Coulomb Counter" code in bd7181x_power_probe() function */
	/* Stop Coulomb Counter */
	/* bd7181x_clear_bits(pwr->mfd, BD7181x_REG_CC_CTRL, CCNTENB); */

	sysfs_remove_group(&pwr->bat->dev.kobj, &bd7181x_sysfs_attr_group);

	cancel_delayed_work(&pwr->bd_work);

	power_supply_unregister(pwr->bat);
	power_supply_unregister(pwr->ac);
	platform_set_drvdata(pdev, NULL);
	kfree(pwr);

	return 0;
}

static struct platform_driver bd7181x_power_driver = {
	.driver = {
		.name = "bd7181x-power",
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(bd7181x_power_remove),
};

/** @brief module initialize function */
static int __init bd7181x_power_init(void)
{
	return platform_driver_probe(&bd7181x_power_driver, bd7181x_power_probe);
}

module_init(bd7181x_power_init);

/** @brief module deinitialize function */
static void __exit bd7181x_power_exit(void)
{
	platform_driver_unregister(&bd7181x_power_driver);
}

module_exit(bd7181x_power_exit);

module_param(battery_cycle, uint, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(battery_parameters, "battery_cycle:battery charge/discharge cycles");

MODULE_AUTHOR("Tony Luo <luofc@embest-tech.com>");
MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("BD71815/BD71817 Battery Charger Power driver");
MODULE_LICENSE("GPL");


/*-------------------------------------------------------*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define PROCFS_NAME 		"bd7181x_rev"
#define BD7181X_REV			"BD7181x Driver: Rev008\n"

#define BD7181X_BUF_SIZE	1024
static char procfs_buffer[BD7181X_BUF_SIZE];
/**
 * This function is called then the /proc file is read
 *
 */
static int onetime = 0;
static ssize_t bd7181x_proc_read (struct file *file, char __user *buffer, size_t count, loff_t *data)
{
	int ret = 0, error = 0;
	if(onetime==0) {
		onetime = 1;
		memset( procfs_buffer, 0, BD7181X_BUF_SIZE);
		sprintf(procfs_buffer, "%s", BD7181X_REV);
		ret = strlen(procfs_buffer);
		error = raw_copy_to_user(buffer, procfs_buffer, strlen(procfs_buffer));
	} else {
		//Clear for next time
		onetime = 0;
	}
	return (error!=0)?0:ret;
}

#if 0
int bd7181x_debug_mask = 0;
static ssize_t bd7181x_proc_write (struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	sscanf(buffer, "0x%x", &bd7181x_debug_mask);
	printk("BD7181x: bd7181x_debug_mask=0x%08x\n", bd7181x_debug_mask);
	return count;
}
#endif

static const struct file_operations bd7181x_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= bd7181x_proc_read,
	//.write		= bd7181x_proc_write,
};

/**
 *This function is called when the module is loaded
 *
 */
int bd7181x_revision_init(void)
{
	struct proc_dir_entry *bd7181x_proc_entry;

	/* create the /proc/bd7181x_rev */
	bd7181x_proc_entry = proc_create(PROCFS_NAME, 0644, NULL, &bd7181x_proc_fops);
	if (bd7181x_proc_entry == NULL) {
		printk("Error: Could not initialize /proc/%s\n", PROCFS_NAME);
		return -ENOMEM;
	}

	return 0;
}
module_init(bd7181x_revision_init);
/*-------------------------------------------------------*/
