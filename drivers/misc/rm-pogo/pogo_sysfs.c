// SPDX-License-Identifier: GPL-2.0-only
/*
 * reMarkable POGO Control
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Steinar Bakkemo <steinar.bakkemo@remarkable.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "pogo_sysfs.h"
#include "pogo_fsm.h"
#include "pogo_dr_mode.h"
#include "pogo_charging_ctrl.h"
#include "pogo_attribute_storage.h"

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/export.h>
#include <linux/power_supply.h>
#include <linux/stddef.h>
#include <linux/glob.h>

#define to_pogo_data(kobj_attr_ptr, kobj_attr_member) \
	container_of(kobj_attr_ptr, struct rm_pogo_data, kobj_attr_member);

#define SYSFS_PARENT_NODE NULL
#define SYSFS_NODE_NAME "pogo"

/* There are 2 types of input emulation.
 *
 * a. Direct character input, which will be remapped to key value directly
 * with a shift. This ease our life.
 *
 * b. Use translation method. This is for those control keys that is not
 *  shown in a string, such as CAPSLOCK, ENTER. And for those characters
 *  that are input from combination of keys, such as @ is KEY_SHIFT + KEY_2.
 *  or those keys that is not in below key_map table.
 *
 *  Any sub-string start with "\" will be translated into keycode with the
 * following 2 characters. For example, "\28" will emulate KEY_ENTER,
 * instead of "\" "2" "8".
 *   #define KEY_ENTER		28
 *   #define KEY_SPACE		57
 *   #define KEY_CAPSLOCK	58
 * More can be found in include/uapi/linux/input-event-codes.h
 *
 *
*/

/* Direct map method from ASCII code with a shift. */
#define KEY_START_IDX 44
u8 key_map[] = {
	KEY_COMMA,	// ,	/* ASCII value start from 44 here */
	KEY_MINUS,	// -
	KEY_DOT,	// .
	KEY_SLASH,	// /
	KEY_0,		// 0
	KEY_1,		// 1
	KEY_2,		// 2
	KEY_3,		// 3
	KEY_4,		// 4
	KEY_5,		// 5
	KEY_6,		// 6
	KEY_7,		// 7
	KEY_8,		// 8
	KEY_9,		// 9
	KEY_RESERVED,	// :	/* Not supported. This is value from keys combination */
	KEY_SEMICOLON,	// ;
	KEY_RESERVED,	// <	/* Not supported. This is value from keys combination */
	KEY_EQUAL,	// =
	KEY_RESERVED,	// >	/* Not supported. This is value from keys combination */
	KEY_RESERVED,	// ?	/* Not supported. This is value from keys combination */
	KEY_RESERVED,	// @	/* Not supported. This is value from keys combination */
	KEY_A,		// A
	KEY_B,		// B
	KEY_C,		// C
	KEY_D,		// D
	KEY_E,		// E
	KEY_F,		// F
	KEY_G,		// G
	KEY_H,		// H
	KEY_I,		// I
	KEY_J,		// J
	KEY_K,		// K
	KEY_L,		// L
	KEY_M,		// M
	KEY_N,		// N
	KEY_O,		// O
	KEY_P,		// P
	KEY_Q,		// Q
	KEY_R,		// R
	KEY_S,		// S
	KEY_T,		// T
	KEY_U,		// U
	KEY_V,		// V
	KEY_W,		// W
	KEY_X,		// X
	KEY_Y,		// Y
	KEY_Z,		// Z
	KEY_LEFTBRACE,	// [
	KEY_RESERVED,	// \	/* Not supported, because we use it for translated key */
	KEY_RIGHTBRACE,	// ]
	KEY_RESERVED,	// ^	/* Not supported. This is value from keys combination */
	KEY_RESERVED,	// _	/* Not supported. This is value from keys combination */
	KEY_GRAVE,	// `
	KEY_A,		// a
	KEY_B,		// b
	KEY_C,		// c
	KEY_D,		// d
	KEY_E,		// e
	KEY_F,		// f
	KEY_G,		// g
	KEY_H,		// h
	KEY_I,		// i
	KEY_J,		// j
	KEY_K,		// k
	KEY_L,		// l
	KEY_M,		// m
	KEY_N,		// n
	KEY_O,		// o
	KEY_P,		// p
	KEY_Q,		// q
	KEY_R,		// r
	KEY_S,		// s
	KEY_T,		// t
	KEY_U,		// u
	KEY_V,		// v
	KEY_W,		// w
	KEY_X,		// x
	KEY_Y,		// y
	KEY_Z,		// z  /* ASCII val: 122 */
};

#define CONTROL_GROUP_ATTRIBUTE_COUNT	20

static struct attribute *control_attrs[CONTROL_GROUP_ATTRIBUTE_COUNT + 1] = {
	/* CONTROL_GROUP_ATTRIBUTE_COUNT number of initializing NULLs */
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,

	/* NULL terminating the list */
	NULL
};

static u8 cmd_dev_app[] =
	{KB_CMD_ENTER_APP_LEN, 0, POGO_CMD_ENTER_APP, POGO_SUCCESS};
/* static u8 cmd_dev_name[] =                                                 */
/*         {13, 0, POGO_CMD_GET_NAME, 'r', 'M', 'k', 'e', 'y', 'b', 'o', 'a', */
/*          'r', 'd', '0', '1', '\0'};                                        */

static u8 cmd_fw_iack[] =
	{KB_CMD_FW_WRITE_INIT_LEN, 0, POGO_CMD_FW_WRITE_INIT, POGO_SUCCESS};

static u8 cmd_fw_ack[] =
	{KB_CMD_FW_WRITE_VALIDATE_IMAGE_LEN, 0, POGO_CMD_FW_WRITE_PACKET, POGO_SUCCESS};

static u8 cmd_fw_vack[] =
	{KB_CMD_FW_WRITE_VALIDATE_CRC_LEN, 0, POGO_CMD_FW_WRITE_VALIDATE_CRC, POGO_SUCCESS};

static u8 cmd_alive[] =
	{0, 0, KB_REPORT_ALIVE};

struct pogo_mcu_cmd {
	char	*name;
	u8	*data;
	u16	len;
};

static struct pogo_mcu_cmd mcu_cmd[] = {
	/* {"name", cmd_dev_name, ARRAY_SIZE(cmd_dev_name)}, */
	{"fw_iack", cmd_fw_iack, ARRAY_SIZE(cmd_fw_iack)},
	{"fw_ack", cmd_fw_ack, ARRAY_SIZE(cmd_fw_ack)},
	{"fw_vack", cmd_fw_vack, ARRAY_SIZE(cmd_fw_vack)},
	{"app", cmd_dev_app, ARRAY_SIZE(cmd_dev_app)},
	{"alive", cmd_alive, ARRAY_SIZE(cmd_alive)},
};

static struct attribute_group control_attr_group = {
	.attrs = control_attrs,
	.name = "control"
};

#define STATUS_GROUP_ATTRIBUTE_COUNT	9

struct attribute *status_attrs[STATUS_GROUP_ATTRIBUTE_COUNT + 1] = {
	/* STATUS_GROUP_ATTRIBUTE_COUNT number of NULLS */
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,

	/* NULL terminating the list */
	NULL
};

#define EMULATION_GROUP_ATTRIBUTE_COUNT 6

struct attribute *emulation_attrs[EMULATION_GROUP_ATTRIBUTE_COUNT + 1] = {
	/* EMULATION_GROUP_ATTRIBUTE_COUNT number of NULLS */
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,

	/* NULL terminating the list */
	NULL
};

static struct attribute_group status_attr_group = {
	.attrs = status_attrs,
	.name = "status"
};

static struct attribute_group emulation_attr_group = {
	.attrs = emulation_attrs,
	.name = "emulation"
};

/* must be aligned with package_kb_language_t */
static char *dev_language[] = {
"ILLEGAL",
"DE",
"ES",
"FR",
"IT",
"NO",
"PT",
"UK",
"US",
};

static ssize_t attribute_show(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      char *buf)
{
	int count, var;
	int i;
	struct rm_pogo_data *pdata;

	if (strcmp(attr->attr.name, "pogo_connected") == 0) {
		pdata = to_pogo_data(attr,
					       pogo_connected_attribute);

		var = pdata->pogo_connected;

		dev_dbg(pdata->dev,
			"%s: Returning cur pogo_connected value (%d)\n",
			__func__,
			var);

	}
	else if (strcmp(attr->attr.name, "pogo_dr_mode") == 0) {
		pdata = to_pogo_data(attr,
					       pogo_dr_mode_attribute);

		dev_dbg(pdata->dev,
			"%s: Returning cur pogo_id_state value (%d)\n",
			__func__,
			pdata->pogo_dr_mode);

		var = pdata->pogo_dr_mode;
	}
	else if (strcmp(attr->attr.name, "pogo_mcu_auth") == 0) {
		pdata = to_pogo_data(attr,
					       pogo_mcu_auth_attribute);

		dev_dbg(pdata->dev,
			"%s: Returning mcu_authenticated value (%d)\n",
			__func__,
			pdata->mcu_authenticated);

		var = pdata->mcu_authenticated;
	}
	else if (strcmp(attr->attr.name, "ack_timeout") == 0) {
		pdata = to_pogo_data(attr,
					       ack_timeout_attribute);

		dev_dbg(pdata->dev, "%s: Returning ack_timeout %dms\n",
			__func__, pdata->ack_timeout);

		var = pdata->ack_timeout;
	}
	else if (strcmp(attr->attr.name, "manual_fsm") == 0) {
		pdata = to_pogo_data(attr,
					       manual_fsm_attribute);

		dev_dbg(pdata->dev, "%s: Returning manual_fsm %dms\n",
			__func__, pdata->manual_fsm);

		var = pdata->manual_fsm;
	}
	else if (strcmp(attr->attr.name, "alive_timeout") == 0) {
		pdata = to_pogo_data(attr,
					       alive_timeout_attribute);

		dev_dbg(pdata->dev, "%s: Returning alive_timeout %dms\n",
			__func__, pdata->alive_timeout);

		var = pdata->alive_timeout;
	}
	else if (strcmp(attr->attr.name, "pogo_chargermode") == 0) {
		pdata = to_pogo_data(attr,
					       pogo_chargermode_attribute);

		count = pogo_get_otg_charger_modes(pdata, buf);

		dev_dbg(pdata->dev,
			"%s: Returning charger mode list: %s\n",
			__func__,
			buf);

		return count;
	}
	else if (strcmp(attr->attr.name, "onewire_pinctrlstate") == 0) {
		pdata = to_pogo_data(attr,
					       onewire_pinctrlstate_attribute);
		dev_dbg(pdata->dev,
			"%s: Returning cur pinctrlstate (%d)\n",
			__func__,
			pdata->onewire_pinctrlstate);

		var = pdata->onewire_pinctrlstate;
	}
	else if (strcmp(attr->attr.name, "pogo_gpio_pinctrl_index") == 0) {
		int cnt;
		pdata = to_pogo_data(attr,
					       pogo_gpio_pinctrl_index_attribute);
		dev_dbg(pdata->dev,
			"%s: print cur gpio pinctrlstate (%d)\n",
			__func__,
			pdata->pogo_gpio_pinctrl_index);

		var = pdata->pogo_gpio_pinctrl_index;
		cnt = sprintf(buf, "%s", otg_onewire_pinctrl_name[var]);
		return cnt;
	}
	else if (strcmp(attr->attr.name, "fsm_state") == 0) {
		pdata = to_pogo_data(attr,
					       fsm_state_attribute);
		dev_dbg(pdata->dev,
			"%s: Returning cur fsm_state (%d)\n",
			__func__,
			pdata->pogo_fsm_state);

		var = pdata->pogo_fsm_state;
	}
	else if (strcmp(attr->attr.name, "lang") == 0) {
		int index, len;
		pdata = to_pogo_data(attr,
					       lang_attribute);

		index = 0xf & pdata->dev_info.language;
		if (!index || index >= ARRAY_SIZE(dev_language))
			return 0;

		len = strlen(dev_language[index]);
		memcpy(buf, dev_language[index], len);
		return len;
	}
	else if (strcmp(attr->attr.name, "keylayout_ver") == 0) {
		pdata = to_pogo_data(attr,
					       keylayout_ver_attribute);

		return sprintf(buf, "%d\n", pdata->dev_info.keylayout);
	}
	else if (strcmp(attr->attr.name, "fw_ver") == 0) {
		pdata = to_pogo_data(attr,
					       fw_ver_attribute);

		return sprintf(buf, "%d.%d\n", pdata->dev_info.fw_version.major,
				pdata->dev_info.fw_version.minor);
	}
	else if (strcmp(attr->attr.name, "dev_id") == 0) {
		u8 *id, cnt, len = 0;
		pdata = to_pogo_data(attr,
					       dev_id_attribute);

		id = (u8*)pdata->dev_info.device_id;
		for (cnt = 0; cnt < sizeof(pdata->dev_info.device_id); cnt++, id++) {
			len += sprintf(buf, "%02x", *id);
			buf+= 2;
		}
		len += sprintf(buf, "\n");
		return len;
	}
	else if (strcmp(attr->attr.name, "mfg_log") == 0) {
		pdata = to_pogo_data(attr, mfg_log_attribute);

		return sprintf(buf, "0x%x", pdata->mfg_log);
	}
	else if (strcmp(attr->attr.name, "onewire_rx_buf") == 0) {
		pdata = to_pogo_data(attr, onewire_rx_buf_attribute);

		mutex_lock(&pdata->lock);
		if ((pdata->onewire_rx_buf_len * 2) + 1 > PAGE_SIZE) {
			count = pdata->onewire_rx_buf_len;
			mutex_unlock(&pdata->lock);
			return sprintf(
				buf,
				"%s: cannot print rx buf, too long (%d) \n",
				__func__, count);
		}

		for (i = 0; i < pdata->onewire_rx_buf_len; ++i) {
			snprintf(buf + (i * 2), 3, "%02hhx",
				 pdata->onewire_rx_buf[i]);
		}
		dev_info(pdata->dev, "%s: onewire_rx_buf: %s\n", __func__, buf);

		// Newline if we have space
		if (((pdata->onewire_rx_buf_len * 2) + 2) < PAGE_SIZE) {
			sprintf(buf + (pdata->onewire_rx_buf_len * 2) + 1,
				"\n");
			count = (pdata->onewire_rx_buf_len * 2) + 2;
		} else {
			count = pdata->onewire_rx_buf_len * 2;
		}
		mutex_unlock(&pdata->lock);

		return count;
	}
	else if (strcmp(attr->attr.name, "fw_write_status") == 0) {
		pdata = to_pogo_data(attr, fw_write_status_attribute);

		mutex_lock(&pdata->lock);

		switch(pdata->fw_write_status) {
		case FW_WRITE_STATUS_NOT_STARTED:
			count = sprintf(buf, "%s\n", "fw-write-not-started");
		break;
		case FW_WRITE_STATUS_UP_TO_DATE:
			count = sprintf(buf, "%s\n", "fw-write-up-to-date");
		break;
		case FW_WRITE_STATUS_STARTED:
			count = sprintf(buf, "%s\n", "fw-write-started");
		break;
		case FW_WRITE_STATUS_FAILED:
			count = sprintf(buf, "%s\n", "fw-write-failed");
		break;
		case FW_WRITE_STATUS_SUCCEEDED:
			count = sprintf(buf, "%s\n", "fw-write-succeeded");
		break;
		default:
			dev_err(pdata->dev,
			"%s: fw_write_status has illegal value:%d\n",
			__func__,
			pdata->fw_write_status);
			mutex_unlock(&pdata->lock);
			return -EINVAL;
		}

		mutex_unlock(&pdata->lock);
		return count;
	}
	else if (strcmp(attr->attr.name, "auto_key") == 0) {
		pdata = to_pogo_data(attr, auto_key_attribute);

		return sprintf(buf, "%d", pdata->auto_key);
	}
	else if (strcmp(attr->attr.name, "emulation_enable") == 0) {
		pdata = to_pogo_data(attr,
					       emulation_enable_attribute);
		dev_dbg(pdata->dev,
			"%s: Returning cur emulation_enable (%d)\n",
			__func__,
			pdata->emulation_enable);

		var = pdata->emulation_enable;
	}
	else if (strcmp(attr->attr.name, "fake_dev_connected") == 0) {
		pdata = to_pogo_data(attr,
					       fake_dev_connected_attribute);
		dev_dbg(pdata->dev,
			"%s: Returning cur fake_dev_connected (%d)\n",
			__func__,
			pdata->fake_dev_connected);

		var = pdata->fake_dev_connected;
	}
	else if (strcmp(attr->attr.name, "fake_fsm_err") == 0) {
		pdata = to_pogo_data(attr,
					       fake_fsm_err_attribute);
		dev_dbg(pdata->dev,
			"%s: Returning cur fake_fsm_err (%d)\n",
			__func__,
			pdata->fsm_err);

		var = pdata->fsm_err;
	}
	else if (strcmp(attr->attr.name, "fake_cmd") == 0) {
		int cnt = sprintf(buf, "available cmd: ");

		for (var = 0; var < ARRAY_SIZE(mcu_cmd); var++)
			cnt += sprintf(buf + cnt, "%s ", mcu_cmd[var].name);

		cnt += sprintf(buf + cnt, "\n");
		return cnt;
	}
	else if (strcmp(attr->attr.name, "user_command_response") == 0) {
		int i, len = 0, packet_length;
		pdata = to_pogo_data(attr, user_command_response_attribute);

		mutex_lock(&pdata->lock);

		packet_length = packet_len(pdata->user_command_response);
		for (i = 0; i < packet_length; i++) {
			len += sprintf(buf, "%02x ", pdata->user_command_response[i]);
			buf += 3;
		}
		mutex_unlock(&pdata->lock);

		len += sprintf(buf, "\n");
		return len;
	}
	else if (strcmp(attr->attr.name, "serial") == 0) {
		const uint8_t serial_len = sizeof(pdata->dev_info.serial);
		pdata = to_pogo_data(attr, serial_attribute);
		dev_dbg(pdata->dev, "%s: Returning current serial: %.*s\n",
			__func__, serial_len,
			pdata->dev_info.serial);

		return snprintf(buf, serial_len + 1, "%.*s\n", serial_len,
				pdata->dev_info.serial);
	} else if (strcmp(attr->attr.name, "mcu_alive_interval") == 0) {
		pdata = to_pogo_data(attr, mcu_alive_interval_attribute);
		dev_dbg(pdata->dev,
			"%s: Returning current MCU alive interval: %d ms\n",
			__func__, pdata->mcu_alive_interval);

		return sprintf(buf, "%d\n", pdata->mcu_alive_interval);
	} else {
		pr_err("%s: Invalid attribute name (%s), returning 0\n",
			__func__,
			attr->attr.name);

		var = 0;
	}

	return sprintf(buf, "%d\n", var);
}

static int send_key_packet(struct rm_pogo_data *pdata, const char *buf,
			   size_t count, bool key_press)
{
	/* Emulated data does not go through serdev, so message
	 * is used, instead of wrapped packet.
	 */
	char msg[] = {1, 0, KB_REPORT_KEY, 0};
	unsigned int row, col, val;

	if (!pdata->emulation_enable)
		return count;

	mutex_lock(&pdata->lock);
	/* col is used to disable warning */
	col = kstrtoint(buf, 16, &val);

	row = 0x7 & (val >> 1);
	col = 0xf & (val >> 4);
	if (row > 6 || col > 14) {
		dev_dbg(pdata->dev, "Unsupported key: 0x%x, row %d col %d",
			val, row, col);
		mutex_unlock(&pdata->lock);
		return count;
	}

	if (key_press)
		val |= 1;
	else
		val &= ~1;

	msg[3] = val;

	kfifo_in(&pdata->read_fifo, msg, 4);
	wake_up_process(pdata->fsm_thread);
	mutex_unlock(&pdata->lock);
	msleep(50);

	return count;
}

static ssize_t attribute_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct rm_pogo_data *pdata;
	int var, ret;

	if (strcmp(attr->attr.name, "pogo_dr_mode") == 0) {
		pdata = to_pogo_data(attr,
					       pogo_dr_mode_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		dev_warn(pdata->dev,
			"%s: Fail to set new pogo dr mode (%d). TODO\n",
			__func__,
			var);
	}
	else if (strcmp(attr->attr.name, "pogo_mcu_auth") == 0) {
		pdata = to_pogo_data(attr,
					       pogo_mcu_auth_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		dev_dbg(pdata->dev,
			"%s: Setting mcu_auth status (%d)\n", __func__, var);

		mutex_lock(&pdata->lock);
		pdata->mcu_authenticated = var;
		wake_up_process(pdata->fsm_thread);
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "pogo_suspend") == 0) {
		pdata = to_pogo_data(attr, pogo_suspend_attribute);

		dev_dbg(pdata->dev,
			"%s: Sending suspend cmd to MCU..\n", __func__);

		mutex_lock(&pdata->lock);
		INIT_USER_COMMAND(POGO_CMD_ENTER_SUSPEND, 0);
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "pogo_reboot") == 0) {
		pdata = to_pogo_data(attr, pogo_reboot_attribute);

		dev_dbg(pdata->dev,
			"%s: Sending reboot cmd to MCU..\n", __func__);

		mutex_lock(&pdata->lock);
		ret = pogo_onewire_write(pdata, POGO_CMD_REBOOT, 0, NULL, 0, false);
		mutex_unlock(&pdata->lock);
		if (ret)
			return ret;
	}
	else if (strcmp(attr->attr.name, "hard_reset") == 0) {
		pdata = to_pogo_data(attr, hard_reset_attribute);

		dev_dbg(pdata->dev,	"%s: Hard reseting MCU\n", __func__);
		mutex_lock(&pdata->lock);
		/*
		 * Hard reset by turning off VBUS.
		 * This is done when entering IDLE state.
		 *
		 * A suspended MCU will not be reset by VBUS being turned off
		 * as it can live from the POGO_1WIRE pull-up, or even the
		 * energy stored in the capacitor only, for a very long time,
		 * typically several seconds. However, the MCU will wake-up
		 * from deep power sleep and reboot by the VBUS being turned back on.
		 */
		pdata->fsm_err = true;
		wake_up_process(pdata->fsm_thread);
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "fw_write_enabled") == 0) {
		pdata = to_pogo_data(attr, fw_update_enabled_attribute);

		dev_dbg(pdata->dev,
			"%s: Setting fw_write_enabled: %s",
			__func__,
			buf);

		pdata->fw_write_enabled = (*buf == '1');
		if (pdata->fw_write_enabled) {
			/*
			 * Trigger reset of MCU to bring it out of APP mode so that
			 * it can be re-enumerated. FW update is part of the enumeration
			 * process, if enabled.
			 */
			mutex_lock(&pdata->lock);
			pdata->fsm_err = true;
			pdata->app_session = APP_SESSION_NONE;
			wake_up_process(pdata->fsm_thread);
			mutex_unlock(&pdata->lock);
		}
	}
	else if (strcmp(attr->attr.name, "user_command") == 0) {
		pdata = to_pogo_data(attr, user_command_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		switch (var) {
		case POGO_CMD_NONE:
			fallthrough;
		case POGO_CMD_FW_WRITE_VALIDATE_IMAGE:
			fallthrough;
		case POGO_CMD_ENTER_APP:
			fallthrough;
		case POGO_CMD_ENTER_SUSPEND:
			fallthrough;
		case POGO_CMD_FW_WRITE_VALIDATE_CRC:
			fallthrough;
		case POGO_CMD_FW_WRITE_INIT:
			fallthrough;
		case POGO_CMD_REBOOT:
			dev_dbg(pdata->dev, "%s: Setting user_command: %d",
				__func__, var);
			break;
		default:
			dev_err(pdata->dev, "%s: Unsupported command: %d",
				__func__, var);
			return -EIO;
		}

		mutex_lock(&pdata->lock);
		INIT_USER_COMMAND(var, 0);
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "ack_timeout") == 0) {
		pdata = to_pogo_data(attr, ack_timeout_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		dev_dbg(pdata->dev, "%s: Set ack_timeout %dms\n", __func__, var);
		pdata->ack_timeout = var;
	}
	else if (strcmp(attr->attr.name, "manual_fsm") == 0) {
		pdata = to_pogo_data(attr, manual_fsm_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		dev_dbg(pdata->dev, "%s: Set manual_fsm %d\n", __func__, var);
		pdata->manual_fsm = var;
	}
	else if (strcmp(attr->attr.name, "alive_timeout") == 0) {
		pdata = to_pogo_data(attr, alive_timeout_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		dev_dbg(pdata->dev, "%s: Set alive_timeout %d\n", __func__, var);
		pdata->alive_timeout = var;
	}
	else if (strcmp(attr->attr.name, "pogo_chargermode") == 0) {
		pdata = to_pogo_data(attr, pogo_chargermode_attribute);

		dev_dbg(pdata->dev,
			"%s: Setting new pogo chargermode: %s",
			__func__,
			buf);

		ret = pogo_change_otg_charger_mode_str(pdata, buf);
	}
	else if (strcmp(attr->attr.name, "onewire_pinctrlstate") == 0) {
		pdata = to_pogo_data(attr,
					       onewire_pinctrlstate_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		dev_dbg(pdata->dev,
			"%s: Setting new pinctrlstate (%d)\n",
			__func__,
			var);

		ret = pogo_switch_one_wire_mux_state(pdata,
							   var);
	}
	else if (strcmp(attr->attr.name, "pogo_gpio_pinctrl_index") == 0) {
		pdata = to_pogo_data(attr,
					       pogo_gpio_pinctrl_index_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		dev_dbg(pdata->dev,
			"%s: Setting new gpio pinctrlstate index (%d): %s\n",
			__func__,
			var, otg_onewire_pinctrl_name[var]);

		pdata->pogo_gpio_pinctrl_index = var;
	}
	else if (strcmp(attr->attr.name, "lang") == 0) {
		char index;
		attribute_storage_id_t *attr_id = NULL;
		attribute_tx_u *write_payload;
		pdata = to_pogo_data(attr, lang_attribute);

		/* Write or fetch? */
		if (count >= 5 && strncmp(buf, "fetch", 5) == 0) {
			dev_dbg(pdata->dev,
				"%s: sending ATTRIBUTE_ID_LANGUAGE read attribute cmd\n",
				__func__);
			mutex_lock(&pdata->lock);

			INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_READ,
					  sizeof(attribute_storage_id_t));

			attr_id = (attribute_storage_id_t *)
					  pdata->user_command_data;
			*attr_id = ATTRIBUTE_ID_LANGUAGE;
			mutex_unlock(&pdata->lock);
			return count;
		}

		for (index = 0; index < ARRAY_SIZE(dev_language); index++)
			if (strncmp(buf, dev_language[(int)index], 2) == 0)
				break;

		if (index >= ARRAY_SIZE(dev_language))
			return -EINVAL;

		dev_info(pdata->dev, "%s: Setting language (%d)%s\n", __func__,
			 index, dev_language[(int)index]);

		mutex_lock(&pdata->lock);

		INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_WRITE,
				  SIZEOF_ATTR_TX_U(ATTRIBUTE_ID_LANGUAGE));

		INIT_ATTR_TX_U(ATTRIBUTE_ID_LANGUAGE, write_payload);

		write_payload->ATTR_VAL_BY_ID(ATTRIBUTE_ID_LANGUAGE) = index;
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "keylayout_ver") == 0) {
		char layout;
		attribute_storage_id_t *attr_id = NULL;
		attribute_tx_u *write_payload;
		pdata = to_pogo_data(attr, keylayout_ver_attribute);

		/* Write or fetch? */
		if (count >= 5 && strncmp(buf, "fetch", 5) == 0) {
			dev_dbg(pdata->dev,
				"%s: sending ATTRIBUTE_ID_KEY_LAYOUT read attribute cmd\n",
				__func__);
			mutex_lock(&pdata->lock);

			INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_READ,
					  sizeof(attribute_storage_id_t));

			attr_id = (attribute_storage_id_t *)
					  pdata->user_command_data;
			*attr_id = ATTRIBUTE_ID_KEY_LAYOUT;
			mutex_unlock(&pdata->lock);
			return count;
		}

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		layout = (char)var;
		dev_dbg(pdata->dev,
			"%s: Setting layout version %d\n", __func__, layout);

		mutex_lock(&pdata->lock);

		INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_WRITE,
				  SIZEOF_ATTR_TX_U(ATTRIBUTE_ID_KEY_LAYOUT));

		INIT_ATTR_TX_U(ATTRIBUTE_ID_KEY_LAYOUT, write_payload);

		write_payload->ATTR_VAL_BY_ID(ATTRIBUTE_ID_KEY_LAYOUT) = layout;
		mutex_unlock(&pdata->lock);
		if (ret)
			return ret;
	}
	else if (strcmp(attr->attr.name, "mfg_log") == 0) {
		char mfg_log;
		attribute_tx_u *write_payload;
		pdata = to_pogo_data(attr, mfg_log_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		if (var > 0xff) {
			dev_dbg(pdata->dev, "Invalid mfg_log 0x%x\n", var);
			return -EINVAL;
		}

		mfg_log = (char)var;
		dev_dbg(pdata->dev, "%s: Setting mfg_log 0x%x\n", __func__,
			mfg_log);

		mutex_lock(&pdata->lock);

		INIT_USER_COMMAND(
			POGO_CMD_ATTRIBUTE_WRITE,
			SIZEOF_ATTR_TX_U(ATTRIBUTE_ID_MFG_LOG));


		INIT_ATTR_TX_U(ATTRIBUTE_ID_MFG_LOG, write_payload);
		write_payload->ATTR_VAL_BY_ID(ATTRIBUTE_ID_MFG_LOG) = mfg_log;
		mutex_unlock(&pdata->lock);
		if (ret)
			return ret;
	}
	else if (strcmp(attr->attr.name, "fetch_mfg_log") == 0) {
		attribute_storage_id_t *attr_id = NULL;
		pdata = to_pogo_data(attr, fetch_mfg_log_attribute);

		dev_dbg(pdata->dev,
			"%s: Requesting mfg_log\n", __func__);

		mutex_lock(&pdata->lock);

		INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_READ,
				  sizeof(attribute_storage_id_t)
				  );

		attr_id = (attribute_storage_id_t*)pdata->user_command_data;
		*attr_id = ATTRIBUTE_ID_MFG_LOG;
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "auto_key") == 0) {
		char auto_key;
		attribute_tx_u *write_payload;
		pdata = to_pogo_data(attr, auto_key_attribute);

		ret = kstrtoint(buf, 10, &var);
		if (ret < 0)
			return ret;

		mutex_lock(&pdata->lock);
		INIT_USER_COMMAND(
			POGO_CMD_ATTRIBUTE_READ,
			SIZEOF_ATTR_TX_U(ATTRIBUTE_ID_AUTO_KEYBOARD_TEST_MODE));
		INIT_ATTR_TX_U(ATTRIBUTE_ID_AUTO_KEYBOARD_TEST_MODE,
				  write_payload);

		pdata->auto_key = auto_key = (char)var;
		write_payload->ATTR_VAL_BY_ID(ATTRIBUTE_ID_AUTO_KEYBOARD_TEST_MODE) =
			auto_key;

		dev_dbg(pdata->dev, "%s: Setting auto_key 0x%x\n", __func__,
			auto_key);
		mutex_unlock(&pdata->lock);
		if (ret)
			return ret;
	}
	else if (strcmp(attr->attr.name, "emulation_enable") == 0) {
		pdata = to_pogo_data(attr, emulation_enable_attribute);

		dev_dbg(pdata->dev,
			"%s: Setting new emulation_enable: %s",
			__func__,
			buf);

		pdata->emulation_enable = (*buf == '1');
	}
	else if (strcmp(attr->attr.name, "fake_dev_connected") == 0) {
		pdata = to_pogo_data(attr, fake_dev_connected_attribute);

		dev_dbg(pdata->dev,
			"%s: Setting new fake_dev_connected: %s",
			__func__,
			buf);

		if (pdata->emulation_enable) {
			pdata->fake_dev_connected = (*buf == '1');
			mutex_lock(&pdata->lock);
			wake_up_process(pdata->fsm_thread);
			mutex_unlock(&pdata->lock);
		}
	}
	else if (strcmp(attr->attr.name, "fake_fsm_err") == 0) {
		pdata = to_pogo_data(attr, fake_fsm_err_attribute);

		dev_dbg(pdata->dev,
			"%s: Setting new fake_fsm_err: %s",
			__func__,
			buf);

		if (mutex_trylock(&pdata->lock)) {
			pdata->fsm_err = (*buf == '1');
			wake_up_process(pdata->fsm_thread);
			mutex_unlock(&pdata->lock);
		} else {
			return	-EAGAIN;
		}
	}
	else if (strcmp(attr->attr.name, "fake_key_input") == 0) {

		pdata = to_pogo_data(attr, fake_key_input_attribute);
		dev_dbg(pdata->dev, "%s: Report key: %s", __func__, buf);

		send_key_packet(pdata, buf, count, true);

		return send_key_packet(pdata, buf, count, false);
	}
	else if (strcmp(attr->attr.name, "fake_key_press") == 0) {
		pdata = to_pogo_data(attr, fake_key_press_attribute);

		dev_dbg(pdata->dev, "%s: Press key: %s", __func__, buf);

		return send_key_packet(pdata, buf, count, true);
	}
	else if (strcmp(attr->attr.name, "fake_key_release") == 0) {
		pdata = to_pogo_data(attr, fake_key_release_attribute);

		dev_dbg(pdata->dev, "%s: Release key: %s", __func__, buf);

		return send_key_packet(pdata, buf, count, false);
	}
	else if (strcmp(attr->attr.name, "fake_cmd") == 0) {
		int idx = -1;

		pdata = to_pogo_data(attr, fake_cmd_attribute);
		mutex_lock(&pdata->lock);
		for (var = 0; var < ARRAY_SIZE(mcu_cmd); var++) {
			if (strncmp(buf, mcu_cmd[var].name,
			    strlen(mcu_cmd[var].name)) == 0) {
				idx = var;
			}
		}

		if (idx >= 0) {
			kfifo_in(&pdata->read_fifo,
				 mcu_cmd[idx].data, mcu_cmd[idx].len);

			dev_dbg(pdata->dev, "fake %s msg:\n", mcu_cmd[idx].name);
			for (var = 0; var < mcu_cmd[idx].len; var++)
				dev_dbg(pdata->dev, "0x%x\n",
					mcu_cmd[idx].data[var]);

			pdata->tx_ack_required = false;
			pdata->tx_ack_timeout = false;
			cancel_delayed_work(
				&pdata->uart_tx_ack_watchdog_work_queue);
			wake_up_process(pdata->fsm_thread);
		}
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "serial") == 0) {
		attribute_tx_u *write_payload = NULL;

		attribute_storage_id_t *attr_id = NULL;
		int serial_len = count;
		pdata = to_pogo_data(attr, serial_attribute);

		/* Write or fetch? */
		if (count >= 5 && strncmp(buf, "fetch", 5) == 0) {
			dev_dbg(pdata->dev,
				"%s: sending ATTRIBUTE_ID_RM_SERIAL_NUMBER read attribute cmd\n",
				__func__);
			mutex_lock(&pdata->lock);

			INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_READ,
					  sizeof(attribute_storage_id_t));

			attr_id = (attribute_storage_id_t *)
					  pdata->user_command_data;
			*attr_id = ATTRIBUTE_ID_RM_SERIAL_NUMBER;
			mutex_unlock(&pdata->lock);
			return count;
		}
		
		/*
		 * we allow either exact length, without new line,
		 * or exact length + new line.
		 * This to allow the serial number to be set programmatically
		 * or from command line.
		 */
		if (buf[serial_len - 1] == '\n')
			serial_len -= 1;

		if (serial_len != sizeof(pdata->dev_info.serial)) {
			dev_err(pdata->dev,
				"%s: incorrect serial number length. expected %d characters, got %d\n",
				__func__, sizeof(pdata->dev_info.serial),
				serial_len);

			return -EINVAL;
		}

		/* The trailing '*' in the pattern is needed in case of a trailing '\n'.
		 * We have already checked for length above so the pattern effectively
		 * covers only what we are interested in.
		 */
		if (glob_match("RM[0-9][0-9][0-9]-[0-9][0-9][0-9]-[0-9][0-9][0-9][0-9][0-9]*", buf)) {
			dev_dbg(pdata->dev, "%s: Setting serial %s\n", __func__, buf);
		} else {
			dev_err(pdata->dev, "%s: Wrong serial format, expecting %s\n",
			        __func__, "RM[0-9][0-9][0-9]-[0-9][0-9][0-9]-[0-9][0-9][0-9][0-9][0-9]");
			return -EINVAL;
		}

		mutex_lock(&pdata->lock);

		INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_WRITE,
				  offsetof(attribute_tx_u, str) +
					  sizeof(pdata->dev_info.serial));

		INIT_ATTR_TX_U(ATTRIBUTE_ID_RM_SERIAL_NUMBER, write_payload);

		write_payload->str_len = sizeof(pdata->dev_info.serial);
		memcpy(write_payload->str, buf, sizeof(pdata->dev_info.serial));
		mutex_unlock(&pdata->lock);
	}
	else if (strcmp(attr->attr.name, "mcu_alive_interval") == 0) {
		uint16_t mcu_alive_interval;
		attribute_storage_id_t *attr_id = NULL;
		attribute_tx_u *write_payload;
		pdata = to_pogo_data(attr, mcu_alive_interval_attribute);

		/* Write or fetch? */
		if (count >= 5 && strncmp(buf, "fetch", 5) == 0) {
			dev_dbg(pdata->dev,
				"%s: sending ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS read attribute cmd\n",
				__func__);
			mutex_lock(&pdata->lock);

			INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_READ,
					  sizeof(attribute_storage_id_t));

			attr_id = (attribute_storage_id_t *)
					  pdata->user_command_data;
			*attr_id = ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS;
			mutex_unlock(&pdata->lock);
			return count;
		}

		ret = kstrtou16(buf, 10, &mcu_alive_interval);
		if (ret < 0)
			return ret;

		if (mcu_alive_interval > U16_MAX) {
			dev_dbg(pdata->dev, "Invalid mcu_alive_interval 0x%x\n", mcu_alive_interval);
			return -EINVAL;
		}

		dev_dbg(pdata->dev, "%s: Setting mcu_alive_interval %d\n", __func__, mcu_alive_interval);

		mutex_lock(&pdata->lock);

		INIT_USER_COMMAND(
			POGO_CMD_ATTRIBUTE_WRITE,
			SIZEOF_ATTR_TX_U(
				ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS));

		INIT_ATTR_TX_U(ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS, write_payload);

		write_payload->ATTR_VAL_BY_ID(ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS) = mcu_alive_interval;
		mutex_unlock(&pdata->lock);
	} else if (strcmp(attr->attr.name, "attribute_rw") == 0) {
		pdata = to_pogo_data(attr, attribute_rw_attribute);
		return exec_attribute_user_command(pdata, buf, count);
	}
	else {
		return -EINVAL;
	}

	return count;
}

void pogo_create_kobj_property(struct kobj_attribute *attr,
				     const char *name,
				     int permission,
				     ssize_t (*show)(struct kobject *kobj,
						     struct kobj_attribute *attr,
						     char *buf),
				     ssize_t (*store)(struct kobject *kobj,
						      struct kobj_attribute *attr,
						      const char *buf,
						      size_t count))
{
	attr->attr.name = name;
	attr->attr.mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO | S_IWUSR);
	attr->show = show;
	attr->store = store;
}

int pogo_init_sysfs_nodes(struct rm_pogo_data *pdata)
{
	struct kobject *pogo_kobj;
	int ret;

	sysfs_attr_init(&pdata->pogo_dr_mode_attribute.attr);
	pogo_create_kobj_property(&pdata->pogo_dr_mode_attribute,
					"pogo_dr_mode",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->pogo_mcu_auth_attribute.attr);
	pogo_create_kobj_property(&pdata->pogo_mcu_auth_attribute,
					"pogo_mcu_auth",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->pogo_chargermode_attribute.attr);
	pogo_create_kobj_property(&pdata->pogo_chargermode_attribute,
					"pogo_chargermode",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->onewire_pinctrlstate_attribute.attr);
	pogo_create_kobj_property(&pdata->onewire_pinctrlstate_attribute,
					"onewire_pinctrlstate",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->pogo_gpio_pinctrl_index_attribute.attr);
	pogo_create_kobj_property(&pdata->pogo_gpio_pinctrl_index_attribute,
					"pogo_gpio_pinctrl_index",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->auto_key_attribute.attr);
	pogo_create_kobj_property(&pdata->auto_key_attribute,
					"auto_key",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->fetch_mfg_log_attribute.attr);
	pogo_create_kobj_property(&pdata->fetch_mfg_log_attribute,
					"fetch_mfg_log",
					S_IRUGO | S_IWUSR,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->pogo_suspend_attribute.attr);
	pogo_create_kobj_property(&pdata->pogo_suspend_attribute,
					"pogo_suspend",
					S_IWUSR,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->pogo_reboot_attribute.attr);
	pogo_create_kobj_property(&pdata->pogo_reboot_attribute,
					"pogo_reboot",
					S_IWUSR,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->ack_timeout_attribute.attr);
	pogo_create_kobj_property(&pdata->ack_timeout_attribute,
					"ack_timeout",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->manual_fsm_attribute.attr);
	pogo_create_kobj_property(&pdata->manual_fsm_attribute,
					"manual_fsm",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->alive_timeout_attribute.attr);
	pogo_create_kobj_property(&pdata->alive_timeout_attribute,
					"alive_timeout",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->hard_reset_attribute.attr);
	pogo_create_kobj_property(&pdata->hard_reset_attribute,
					"hard_reset",
					S_IWUSR,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->fw_update_enabled_attribute.attr);
	pogo_create_kobj_property(&pdata->fw_update_enabled_attribute,
					"fw_write_enabled",
					S_IWUSR,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->user_command_attribute.attr);
	pogo_create_kobj_property(&pdata->user_command_attribute,
					"user_command",
					S_IWUSR,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->user_command_response_attribute.attr);
	pogo_create_kobj_property(&pdata->user_command_response_attribute,
					"user_command_response",
					S_IRUGO,
					attribute_show,
					NULL);

	sysfs_attr_init(&pdata->fake_fsm_err_attribute.attr);
	pogo_create_kobj_property(&pdata->fake_fsm_err_attribute,
					"fake_fsm_err",
					S_IRUGO,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->serial_attribute.attr);
	pogo_create_kobj_property(&pdata->serial_attribute,
					"serial",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->mcu_alive_interval_attribute.attr);
	pogo_create_kobj_property(&pdata->mcu_alive_interval_attribute,
					"mcu_alive_interval",
					S_IRUGO | S_IWUSR,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->attribute_rw_attribute.attr);
	pogo_create_kobj_property(&pdata->attribute_rw_attribute,
					"attribute_rw",
					S_IWUSR,
					NULL,
					attribute_store);

	control_attrs[0] = &pdata->pogo_dr_mode_attribute.attr;
	control_attrs[1] = &pdata->pogo_chargermode_attribute.attr;
	control_attrs[2] = &pdata->onewire_pinctrlstate_attribute.attr;
	control_attrs[3] = &pdata->pogo_mcu_auth_attribute.attr;
	control_attrs[4] = &pdata->pogo_gpio_pinctrl_index_attribute.attr;
	control_attrs[5] = &pdata->auto_key_attribute.attr;
	control_attrs[6] = &pdata->fetch_mfg_log_attribute.attr;
	control_attrs[7] = &pdata->pogo_suspend_attribute.attr;
	control_attrs[8] = &pdata->ack_timeout_attribute.attr;
	control_attrs[9] = &pdata->manual_fsm_attribute.attr;
	control_attrs[10] = &pdata->alive_timeout_attribute.attr;
	control_attrs[11] = &pdata->fake_fsm_err_attribute.attr;
	control_attrs[12] = &pdata->pogo_reboot_attribute.attr;
	control_attrs[13] = &pdata->hard_reset_attribute.attr;
	control_attrs[14] = &pdata->fw_update_enabled_attribute.attr;
	control_attrs[15] = &pdata->user_command_attribute.attr;
	control_attrs[16] = &pdata->user_command_response_attribute.attr;
	control_attrs[17] = &pdata->serial_attribute.attr;
	control_attrs[18] = &pdata->mcu_alive_interval_attribute.attr;
	control_attrs[19] = &pdata->attribute_rw_attribute.attr;
	control_attrs[CONTROL_GROUP_ATTRIBUTE_COUNT] = NULL; /* NULL termination of the list */

	sysfs_attr_init(&pdata->pogo_connected_attribute.attr);
	pogo_create_kobj_property(&pdata->pogo_connected_attribute,
					"pogo_connected",
					S_IRUGO,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->fsm_state_attribute.attr);
	pogo_create_kobj_property(&pdata->fsm_state_attribute,
					"fsm_state",
					S_IRUGO,
					attribute_show,
					NULL);

	sysfs_attr_init(&pdata->lang_attribute.attr);
	pogo_create_kobj_property(&pdata->lang_attribute,
					"lang",
					S_IRUGO,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->keylayout_ver_attribute.attr);
	pogo_create_kobj_property(&pdata->keylayout_ver_attribute,
					"keylayout_ver",
					S_IRUGO,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->fw_ver_attribute.attr);
	pogo_create_kobj_property(&pdata->fw_ver_attribute,
					"fw_ver",
					S_IRUGO,
					attribute_show,
					NULL);

	sysfs_attr_init(&pdata->dev_id_attribute.attr);
	pogo_create_kobj_property(&pdata->dev_id_attribute,
					"dev_id",
					S_IRUGO,
					attribute_show,
					NULL);

	sysfs_attr_init(&pdata->mfg_log_attribute.attr);
	pogo_create_kobj_property(&pdata->mfg_log_attribute,
					"mfg_log",
					S_IRUGO,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->onewire_rx_buf_attribute.attr);
	pogo_create_kobj_property(&pdata->onewire_rx_buf_attribute,
					"onewire_rx_buf",
					S_IRUGO,
					attribute_show,
					NULL);

	sysfs_attr_init(&pdata->fw_write_status_attribute.attr);
	pogo_create_kobj_property(&pdata->fw_write_status_attribute,
					"fw_write_status",
					S_IRUGO,
					attribute_show,
					NULL);

	status_attrs[0] = &pdata->pogo_connected_attribute.attr;
	status_attrs[1] = &pdata->fsm_state_attribute.attr;
	status_attrs[2] = &pdata->lang_attribute.attr;
	status_attrs[3] = &pdata->keylayout_ver_attribute.attr;
	status_attrs[4] = &pdata->fw_ver_attribute.attr;
	status_attrs[5] = &pdata->dev_id_attribute.attr;
	status_attrs[6] = &pdata->mfg_log_attribute.attr;
	status_attrs[7] = &pdata->onewire_rx_buf_attribute.attr;
	status_attrs[8] = &pdata->fw_write_status_attribute.attr;
	status_attrs[STATUS_GROUP_ATTRIBUTE_COUNT] = NULL;	/* NULL termination of the list */

	sysfs_attr_init(&pdata->emulation_enable_attribute.attr);
	pogo_create_kobj_property(&pdata->emulation_enable_attribute,
					"emulation_enable",
					S_IRUGO,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->fake_dev_connected_attribute.attr);
	pogo_create_kobj_property(&pdata->fake_dev_connected_attribute,
					"fake_dev_connected",
					S_IRUGO,
					attribute_show,
					attribute_store);

	sysfs_attr_init(&pdata->fake_key_input_attribute.attr);
	pogo_create_kobj_property(&pdata->fake_key_input_attribute,
					"fake_key_input",
					S_IRUGO,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->fake_key_press_attribute.attr);
	pogo_create_kobj_property(&pdata->fake_key_press_attribute,
					"fake_key_press",
					S_IRUGO,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->fake_key_release_attribute.attr);
	pogo_create_kobj_property(&pdata->fake_key_release_attribute,
					"fake_key_release",
					S_IRUGO,
					NULL,
					attribute_store);

	sysfs_attr_init(&pdata->fake_cmd_attribute.attr);
	pogo_create_kobj_property(&pdata->fake_cmd_attribute,
					"fake_cmd",
					S_IRUGO,
					attribute_show,
					attribute_store);

	emulation_attrs[0] = &pdata->emulation_enable_attribute.attr;
	emulation_attrs[1] = &pdata->fake_dev_connected_attribute.attr;
	emulation_attrs[2] = &pdata->fake_key_input_attribute.attr;
	emulation_attrs[3] = &pdata->fake_key_press_attribute.attr;
	emulation_attrs[4] = &pdata->fake_key_release_attribute.attr;
	emulation_attrs[5] = &pdata->fake_cmd_attribute.attr;
	emulation_attrs[EMULATION_GROUP_ATTRIBUTE_COUNT] = NULL;	/* NULL termination of the list */

	pogo_kobj = kobject_create_and_add(SYSFS_NODE_NAME,
						 SYSFS_PARENT_NODE);
	if (!pogo_kobj) {
		dev_err(pdata->dev,
			"%s: Failed to create 'pogo' kobject\n",
			__func__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(pogo_kobj,
				 &control_attr_group);
	if (ret != 0) {
		dev_err(pdata->dev,
			"%s: Failed to create 'control' attribute group\n",
			__func__);
		goto error_1;
	}

	ret = sysfs_create_group(pogo_kobj,
				 &status_attr_group);
	if (ret != 0) {
		dev_err(pdata->dev,
			"%s: Failed to create 'status' attribute group\n",
			__func__);
		goto error_2;
	}

	ret = sysfs_create_group(pogo_kobj,
				 &emulation_attr_group);
	if (ret != 0) {
		dev_err(pdata->dev,
			"%s: Failed to create 'emulation' attribute group\n",
			__func__);
		goto error_3;
	}

	pdata->kobject = pogo_kobj;
	return ret;

error_3:
	sysfs_remove_group(pdata->kobject, &status_attr_group);

error_2:
	sysfs_remove_group(pdata->kobject, &control_attr_group);

error_1:
	kobject_put(pdata->kobject);
	return ret;
}

void pogo_uninit_sysfs_nodes(struct rm_pogo_data *pdata)
{
	if ((pdata->kobject != NULL) && !IS_ERR(pdata->kobject)) {
		sysfs_remove_group(pdata->kobject, &emulation_attr_group);
		sysfs_remove_group(pdata->kobject, &control_attr_group);
		sysfs_remove_group(pdata->kobject, &status_attr_group);
		kobject_put(pdata->kobject);
		pdata->kobject = NULL;
	}
}
