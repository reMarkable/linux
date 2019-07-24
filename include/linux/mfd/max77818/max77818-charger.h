/*
 * Maxim MAX77818 Charger Driver Header
 *
 * Copyright (C) 2014 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77818_CHARGER_H__
#define __MAX77818_CHARGER_H__

enum {
	CHG_INT_BYP_I,
	CHG_INT_BATP_I,
	CHG_INT_BAT_I,
	CHG_INT_CHG_I,
	CHG_INT_WCIN_I,
	CHG_INT_CHGIN_I,
	CHG_INT_AICL_I,
};

struct max77818_charger_platform_data {
	int fast_charge_timer;
	int fast_charge_current;
	int topoff_current;
	int topoff_timer;
	int restart_threshold;
	int termination_voltage;
	int input_current_limit;
};

#endif /* !__MAX77818_CHARGER_H__ */

