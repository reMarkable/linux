/*
 * reMarkable POGO Interface Control FSM
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

#include "pogo.h"
#include "pogo_fsm.h"
#include "pogo_charging_ctrl.h"
#include "pogo_dr_mode.h"
#include "pogo_attribute_storage.h"

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/freezer.h>

static bool default_fw_write_enabled = true;
module_param(default_fw_write_enabled, bool, S_IRUSR |  S_IRGRP);
MODULE_PARM_DESC(default_fw_write_enabled, "Controls whether to enable auto FW write or not (default is true)");

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

typedef int (*fsm_event_func) (struct rm_pogo_data *pdata);
typedef void (*fsm_action) (struct rm_pogo_data *pdata);
typedef bool __must_check (*fsm_action_canfail) (struct rm_pogo_data *pdata);

struct state_transit {
	fsm_event_func event_func;
	int to_state;
};

struct fsm_state {
	char			*name;
	fsm_action		routine_func;
	fsm_action_canfail	entry_func;
	fsm_action_canfail	exit_func;
	struct state_transit	*transit_table;
	int			transit_table_size;
};

// TODO Fetch all attributes we care about during enumeration in one step instead of piece-meal
const attribute_storage_id_t dev_info_attrs[] = {
	ATTRIBUTE_ID_FW_VERSION,   ATTRIBUTE_ID_LANGUAGE,
	ATTRIBUTE_ID_DEVICE_CLASS, ATTRIBUTE_ID_IMAGE_START_ADDRESS,
	ATTRIBUTE_ID_KEY_LAYOUT,   ATTRIBUTE_ID_DEVICE_NAME,
	ATTRIBUTE_ID_DEVICE_ID,	   ATTRIBUTE_ID_RM_SERIAL_NUMBER,
};

#define GET_AND_VERIFY_MSG(MSG, MSG_SIZE, RETURN, FSM_ERR)   \
do { \
	count = kfifo_out(&pdata->read_fifo, MSG, MSG_SIZE); \
	if (count < 2) { \
		dev_err(pdata->dev, "Ack message length < 2!\n"); \
		if (FSM_ERR) \
			pdata->fsm_err = true; \
		return RETURN; \
	} \
	len = packet_payload_len(MSG); \
	if (count < len + 3) {\
		dev_err(pdata->dev, "Ack message length:%d, expected:%d!\n", \
				count, len + 3); \
		if (FSM_ERR) \
			pdata->fsm_err = true; \
		return RETURN; \
	} \
} while(0)

/******************************************************************************
 * EVENT FUNCTIONS
 *****************************************************************************/
static int event_always_true(struct rm_pogo_data *pdata)
{
	return 1;
}

static int event_fsm_error(struct rm_pogo_data *pdata)
{
	return pdata->fsm_err == true || pdata->vbus_short == true;
}

static int event_vbus_short(struct rm_pogo_data *pdata)
{
	return pdata->vbus_short == true;
}

static int event_pogo_found(struct rm_pogo_data *pdata)
{
	return !! pdata->pogo_name;
}

static int event_mcu_authenticated(struct rm_pogo_data *pdata)
{
	return !! pdata->mcu_authenticated;
}

static int event_gpio_is_connected(struct rm_pogo_data *pdata)
{
	if (pdata->manual_fsm)
		return false;

	if (pdata->emulation_enable)
		return pdata->fake_dev_connected;

	return pdata->irq_detected && (pdata->vbus_short == false);
}

/******************************************************************************
 * ACTION FUNCTIONS
 *****************************************************************************/
static int fsm_reset_devices(struct rm_pogo_data *pdata)
{
	int ret;

	dev_dbg(pdata->dev, "%s..\n", __func__);
	ret = pogo_change_otg_charger_mode_int(pdata, POGO_CHARGERMODE_CHARGE);
	if (ret > 0) {
		dev_err(pdata->dev,
			"%s: unable to change charger mode\n", __func__);

		return ret;
	}

	ret = pogo_switch_one_wire_mux_state(pdata, POGO_ONEWIRE_STATE__GPIO);
	if (ret > 0)
		dev_err(pdata->dev,
			"%s: failed to set default one-wire pinmux state\n",
			__func__);

	pogo_activate_gpio_irq(pdata);

	return 0;
}

static bool fsm_entry_idle(struct rm_pogo_data *pdata)
{
	int ret;

	pdata->fsm_err = false;
	pdata->app_session = 0;
	pdata->uart_rx_mode = ONE_WIRE_UART_ACK;
	if (pdata->pogo_name) {
		devm_kfree(pdata->dev, pdata->pogo_name);
		pdata->pogo_name = NULL;
	}
	pdata->mcu_authenticated = false;
	pdata->auth_retry = 0;
	pdata->dev_info.device_class = 0;
	memset(&pdata->dev_info, 0, sizeof(pdata->dev_info));
	memset(&pdata->fwu_image_header, 0, sizeof(pdata->fwu_image_header));
	pdata->app_session = APP_SESSION_NONE;
	pdata->fw_write = false;
	pdata->fw = NULL;
	pdata->fw_write_status = FW_WRITE_STATUS_NOT_STARTED;
	pdata->kb_dev = NULL;
	pdata->auto_key = 0;
	pdata->mfg_log = 0;
	pdata->mcu_alive_interval = 0;

	INIT_USER_COMMAND(POGO_CMD_NONE, 0);

	memset(pdata->onewire_rx_buf, 0, ONE_WIRE_MAX_TX_MSG_SIZE);
	pdata->onewire_rx_buf_len = 0;
	pdata->pogo_connected = false;

	if (pdata->serdev_ready)
		pogo_serdev_close(pdata);
	pdata->serdev_ready = false;

	pdata->tx_ack_timeout = false;
	pdata->tx_ack_required = false;
	cancel_delayed_work(
		&pdata->uart_tx_ack_watchdog_work_queue);
	kfifo_reset_out(&pdata->read_fifo);

	del_timer_sync(&pdata->alive_timer);
	pdata->mcu_detach_work_active = false;
	cancel_delayed_work(&pdata->mcu_detach_work);

	ret = fsm_reset_devices(pdata);
	if (ret) {
		dev_err(pdata->dev,
			"%s: fail to reset devices. POGO dies.\n", __func__);
		return false;
	}
	return true;
}

static bool fsm_exit_idle(struct rm_pogo_data *pdata)
{
	pogo_uninit_gpio_irq(pdata);
	return true;
}

static bool fsm_entry_enumerate(struct rm_pogo_data *pdata)
{
	int ret;
	const attribute_storage_id_t request_dev_name_attr[] = {
		ATTRIBUTE_ID_DEVICE_NAME
	};

	 /* OTG POWER ON */
	ret = pogo_change_otg_charger_mode_int(pdata,
					       POGO_CHARGERMODE_OTG);
	if (ret < 0) {
		dev_dbg(pdata->dev,
			"Unable to turn on OTG power, check connections\n");
		return false;
	}

	/* Sleep to allow VBUS to be turned on */
	usleep_range(20000, 21000);

	if (!pogo_serdev_open(pdata))
		pdata->serdev_ready = true;
	else {
		return false;
	}

	/* send a msg to ping dev class */
	kfifo_reset(&pdata->read_fifo);
	dev_dbg(pdata->dev, "Requesting dev name from MCU...\n");

	ret = pogo_onewire_read_attribute(pdata, request_dev_name_attr,
					  ARRAY_SIZE(request_dev_name_attr));
	if (ret) {
		dev_warn(pdata->dev, "%s: failed to send get name cmd to MCU\n", __func__);
		return false;
	}

	/* This starts the timer if it isn't already running */
	mod_timer(&pdata->alive_timer, jiffies +
			msecs_to_jiffies(pdata->alive_timeout));
	return true;
}

static int fsm_parse_dev_name(struct rm_pogo_data *pdata)
{
	u32 count, len;
	u8 msg[ONE_WIRE_MCU_MSG_SIZE];
	int ret = 0;

	GET_AND_VERIFY_MSG(msg, ONE_WIRE_MCU_MSG_SIZE, -EIO, true);

	if (msg[2] == POGO_CMD_ATTRIBUTE_READ && len) {
		ret = update_pdata_from_attribute_read_response(pdata, &msg[3],
								len);
		if (ret != 1) {
			dev_err(pdata->dev, "%s: failed to read device name \n",
				__func__);
			return -EINVAL;
		}

		mod_timer(&pdata->alive_timer,
			  jiffies + msecs_to_jiffies(pdata->alive_timeout));
	} else {
		dev_err(pdata->dev,
			"Invalid cmd 0x%x or len %d for name query.\n", msg[2],
			len);
		return -EINVAL;
	}
	return 0;
}

static void fsm_routine_enumerate(struct rm_pogo_data *pdata)
{
	fsm_parse_dev_name(pdata);
}

static bool fsm_entry_authentication(struct rm_pogo_data *pdata)
{
	int ret = 0;
	dev_dbg(pdata->dev, "Asking MCU for auth key..\n");

	ret = pogo_onewire_write(pdata, POGO_CMD_GET_AUTH_KEY, 0,
			   NULL, 0, true);
	if (ret)
		return false;

	return true;
}

static void fsm_routine_authentication(struct rm_pogo_data *pdata)
{
	u32 count, len;
	u8 msg[ONE_WIRE_MCU_MSG_SIZE];

	GET_AND_VERIFY_MSG(msg, ONE_WIRE_MCU_MSG_SIZE, , true);

	if (msg[2] == POGO_CMD_GET_AUTH_KEY && len == KB_CMD_GET_AUTH_KEY_LEN) {
		if (memcmp(&msg[3], AUTH_KEY_STRING, KB_CMD_GET_AUTH_KEY_LEN) == 0) {
			pdata->mcu_authenticated = true;
			pdata->pogo_connected = true;
		}
	} else {
		dev_err(pdata->dev, "unexpected command 0x%x or len %d\n",
			msg[2], len);
		pdata->fsm_err = true;

	}

	return;
}

static bool fsm_entry_negotiation(struct rm_pogo_data *pdata)
{
	int ret = 0;

	dev_dbg(pdata->dev, "Send dev info query..\n");
	ret = pogo_onewire_read_attribute(pdata, dev_info_attrs,
					  ARRAY_SIZE(dev_info_attrs));
	if (ret) {
		dev_warn(
			pdata->dev,
			"%s: failed to send get info cmd to MCU (%d)\n",
			__func__, ret);
		return false;
	}
	return true;
}

static bool fsm_exit_negotiation(struct rm_pogo_data *pdata)
{
	if (false == pdata->fw_write)
		pogo_release_firmware(pdata);
	return true;
}

static int fsm_get_kb_info(struct rm_pogo_data *pdata)
{
	u32 count, len;
	u8 msg[ONE_WIRE_MCU_MSG_SIZE];
	int got = 0;

	GET_AND_VERIFY_MSG(msg, ONE_WIRE_MCU_MSG_SIZE, -EIO, false);

	/* check cmd type and data length */
	if (msg[2] == POGO_CMD_ATTRIBUTE_READ) {
		got = update_pdata_from_attribute_read_response(pdata, &msg[3],
								len);
		if (got != ARRAY_SIZE(dev_info_attrs)) {
			dev_err(pdata->dev,
				"%s: received unexpected number of attributes. got %d instead of %d\n",
				__func__, got, ARRAY_SIZE(dev_info_attrs));
			pdata->fsm_err = true;
			return -EIO;
		}
	} else {
		dev_err(pdata->dev, "unexpected command 0x%x or len %d\n",
								msg[2], len);
		pdata->fsm_err = true;
		return -EIO;
	}

	dev_info(pdata->dev, "Device class 0x%x,"
		" fw_ver %d.%d, Lang %d Layout %d Image start addr 0x%x\n",
		pdata->dev_info.device_class,
		pdata->dev_info.fw_version.major,pdata->dev_info.fw_version.minor,
		pdata->dev_info.language, pdata->dev_info.keylayout,
		pdata->dev_info.image_start_addr);

	if (POGO_CLASS_UART_IF & pdata->dev_info.device_class) {
		pdata->app_session = APP_SESSION_UART;
		dev_dbg(pdata->dev, "Support UART class\n");
	} else if (POGO_CLASS_OTG_IF & pdata->dev_info.device_class) {
		pdata->app_session = APP_SESSION_USB;
		dev_dbg(pdata->dev, "Support OTG class\n");
	} else {
		pdata->fsm_err = true;
		dev_dbg(pdata->dev, "Unknown class %d:\n",
			pdata->dev_info.device_class);
		return -EIO;
	}

	mod_timer(&pdata->alive_timer,
		jiffies + msecs_to_jiffies(pdata->alive_timeout));
	return 0;
}

static void fsm_routine_negotiation(struct rm_pogo_data *pdata)
{
	char *envp_fw_write_up_to_date[] = { "EVENT=fw-write-up-to-date", NULL };
	int ret;

	ret = fsm_get_kb_info(pdata);
	if (!ret) {
		if (pdata->fw_write_enabled)
			pdata->fw_write = pogo_check_fw_write(pdata);

		if (pdata->fw_write == false) {
			pdata->app_session |= APP_SESSION_READY;
			pdata->fw_write_status = FW_WRITE_STATUS_UP_TO_DATE;
			kobject_uevent_env(&pdata->dev->kobj, KOBJ_CHANGE, envp_fw_write_up_to_date);
		}
	}
}

static bool fsm_entry_fw_write(struct rm_pogo_data *pdata)
{
	int ret;
	dev_dbg(pdata->dev, "Send FW write init..\n");

	ret = pogo_onewire_write(pdata, POGO_CMD_FW_WRITE_INIT, 0, NULL, 0,
				 true);
	if (!ret) {
		pdata->fw_fsm_state = FW_S__INIT;
		pdata->fw_off = 0;
		return true;
	}
	dev_warn(pdata->dev, "%s: failed to send FW write init cmd to MCU\n", __func__);
	return false;
}

static bool fsm_exit_fw_write(struct rm_pogo_data *pdata)
{
	char *envp_fw_write_succeeded[] = { "EVENT=fw-write-succeeded", NULL };
	char *envp_fw_write_failed[] = { "EVENT=fw-write-failed", NULL };

	pdata->fw_write = false;
	pogo_release_firmware(pdata);

	switch (pdata->fw_write_status) {
	case FW_WRITE_STATUS_SUCCEEDED:
		kobject_uevent_env(&pdata->dev->kobj, KOBJ_CHANGE, envp_fw_write_succeeded);
		break;
	case FW_WRITE_STATUS_STARTED:
		/* if the status says started and we are exiting the fw write fsm
		 * then something went wrong and we have to set fw write as failed.
		 * We might have aborted on a command/response error */
		pdata->fw_write_status = FW_WRITE_STATUS_FAILED;
		kobject_uevent_env(&pdata->dev->kobj, KOBJ_CHANGE, envp_fw_write_failed);
		break;
	case FW_WRITE_STATUS_FAILED:
		kobject_uevent_env(&pdata->dev->kobj, KOBJ_CHANGE, envp_fw_write_failed);
		break;

		/* the state should never be one of the two below at this point */
	case FW_WRITE_STATUS_UP_TO_DATE:
		fallthrough;
	case FW_WRITE_STATUS_NOT_STARTED:
		fallthrough;
	default:
		dev_err(pdata->dev,
			"%s: fw_write_status has invalid value:%d\n",
			__func__,
			pdata->fw_write_status);
		break;
	}

	return true;
}

#define FW_WRITE_FW_SENT 0x88
static int fsm_handle_fw_ack(struct rm_pogo_data *pdata)
{
	u32 count, len;
	u8 msg[ONE_WIRE_MCU_MSG_SIZE];
	u8 expected_cmd;
	int ret;

	GET_AND_VERIFY_MSG(msg, ONE_WIRE_MCU_MSG_SIZE, -EIO, false);

	switch(pdata->fw_fsm_state) {
		case FW_S__INIT:
			expected_cmd = POGO_CMD_FW_WRITE_INIT;
			dev_dbg(pdata->dev, "expecting FW init accepted by MCU.\n");
			break;
		case FW_S__MCU_BIN:
			expected_cmd = POGO_CMD_FW_WRITE_PACKET;
			dev_dbg(pdata->dev, "expecting FW packet accepted by MCU.\n");
			break;
		case FW_S__VALIDATE:
			expected_cmd = POGO_CMD_FW_WRITE_VALIDATE_CRC;
			dev_dbg(pdata->dev, "expecting FW validate accepted by MCU.\n");
			break;
		case FW_S__VALIDATE_IMAGE:
			expected_cmd = POGO_CMD_FW_WRITE_VALIDATE_IMAGE;
			dev_dbg(pdata->dev, "expecting FW validate image accepted by MCU.\n");
			break;
		default:
			dev_dbg(pdata->dev, "invalid fw fsm state %d.\n",
									pdata->fw_fsm_state);
			return -EIO;
			break;
	}

	/* check cmd type and data length */
	if (msg[2] == expected_cmd && msg[3] == POGO_SUCCESS) {
		dev_dbg(pdata->dev, "FW command accepted by MCU.\n");
		mod_timer(&pdata->alive_timer,
			jiffies + msecs_to_jiffies(pdata->alive_timeout));

		if (msg[2] == POGO_CMD_FW_WRITE_PACKET) {
			if (pdata->fw_off < pdata->fw->size) {
				ret = pogo_send_fw_packet(pdata);
				if (ret < 0)
					return ret;
			} else {
				return FW_WRITE_FW_SENT;
			}
		}
	} else {
		dev_err(pdata->dev,
			"%s unexpected command %d or response %d (len %d)\n",
			__func__, msg[2], msg[3], len);
		return -EIO;
	}
	return 0;
}

static void fsm_routine_fw_write(struct rm_pogo_data *pdata)
{
	char *envp_fw_write_started[] = { "EVENT=fw-write-started", NULL };
	int ret;
	/*
	* We go to App mode on any fault.
	* Otherwise we might end up in endless update loop if update
	* keeps failing and starts over again
	*/

	dev_dbg(pdata->dev,
		"FW sub-FSM is handling message in state: %d\n",
		pdata->fw_fsm_state);

	switch(pdata->fw_fsm_state) {
		case FW_S__INIT:
			ret = fsm_handle_fw_ack(pdata);
			if (ret != 0) {
				pdata->app_session |= APP_SESSION_READY;
				return;
			}
			pdata->fw_write_status = FW_WRITE_STATUS_STARTED;
			kobject_uevent_env(&pdata->dev->kobj, KOBJ_CHANGE, envp_fw_write_started);
			ret = pogo_send_fw_packet(pdata);
			if (!ret)
				pdata->fw_fsm_state++;
			else
				pdata->app_session |= APP_SESSION_READY;
			break;
		case FW_S__MCU_BIN:
			ret = fsm_handle_fw_ack(pdata);
			if (ret < 0) {
				pdata->app_session |= APP_SESSION_READY;
				return;
			}

			if (!ret)
				/* there are more packets to send,
				 * stay in this state */
				return;

			if (ret == FW_WRITE_FW_SENT) {
				/* the last packet with fw has been sent,
				 * time for CRC validation */
				ret = pogo_send_fw_validate(pdata);
				if (!ret)
					pdata->fw_fsm_state++;
				else
					pdata->app_session |= APP_SESSION_READY;
			}
			break;
		case FW_S__VALIDATE:
			ret = fsm_handle_fw_ack(pdata);

			if (ret != 0) {
				/* CRC validation has failed. Continue to app mode
				 * to avoid looping in case the FW image is corrupted */
				pdata->app_session |= APP_SESSION_READY;
				return;
			}
			/* allow MCU to jump to new image */
			msleep(100);
			dev_dbg(pdata->dev, "Send dev info query..\n");
			ret = pogo_onewire_read_attribute(
				pdata, dev_info_attrs,
				ARRAY_SIZE(dev_info_attrs));
			if (!ret)
				pdata->fw_fsm_state++;
			else
				pdata->fsm_err = true;
			break;
		case FW_S__CHECK_FW_VER:
			ret = fsm_get_kb_info(pdata);
			if (!ret) {
				if (memcmp(&pdata->dev_info.fw_version,
							&pdata->fwu_image_header.fw_version,
							sizeof(pdata->fwu_image_header.fw_version)) != 0) {
					dev_warn(pdata->dev,
						"fw write failed, continues to app mode");
					pdata->app_session |= APP_SESSION_READY;
				} else {
					dev_info(pdata->dev,
						"fw write finished, validating image");
					ret = pogo_send_fw_validate_image(pdata);
					if (!ret) {
						pdata->fw_fsm_state++;
					} else {
						/*
			 			 * We start App mode, otherwise we might end up in endless
						 * write loop if write keeps failing and starts over again
						 */
						pdata->app_session |= APP_SESSION_READY;
					}
				}
			} else {
				/*
				 * if getting kb info fails then it is safest to reset
				 * the fsm here and start over again
				 */
				pdata->fsm_err = true;
			}
			break;
		case FW_S__VALIDATE_IMAGE:
			ret = fsm_handle_fw_ack(pdata);
			if (!ret) {
				dev_info(pdata->dev,
					"fw write succeeded, continues to app mode");
				pdata->fw_write_status = FW_WRITE_STATUS_SUCCEEDED;
			} else {
				dev_warn(pdata->dev,
					"fw write failed, continues to app mode");
			}
			/*
			 * We start App mode regardless whether write succeeded or not.
			 * Otherwise we might end up in endless write loop if write
			 * keeps failing and starts over again
			 */
			pdata->app_session |= APP_SESSION_READY;
			break;
		default:
			pdata->app_session |= APP_SESSION_READY;
			dev_err(pdata->dev, "Unsupported sub-FSM state %d\n", pdata->fw_fsm_state);
			return;
	}
}

static bool fsm_entry_start_app(struct rm_pogo_data *pdata)
{
	int ret = pogo_onewire_write(pdata, POGO_CMD_ENTER_APP, 0, NULL, 0, true);
	if (ret) {
		dev_warn(pdata->dev, "%s: failed to send enter app cmd to MCU (%pe)\n", __func__, ERR_PTR(ret));
		return false;
	}
	return true;
}

static void fsm_routine_start_app(struct rm_pogo_data *pdata)
{
	int count, len;
	u8 msg[ONE_WIRE_MCU_MSG_SIZE];

	GET_AND_VERIFY_MSG(msg, ONE_WIRE_MCU_MSG_SIZE, , true);

	/* check cmd type and data length */
	if (msg[2] == POGO_CMD_ENTER_APP && msg[3] == POGO_SUCCESS) {
		mod_timer(&pdata->alive_timer,
			  jiffies + msecs_to_jiffies(pdata->alive_timeout));
		return;
	}

	dev_err(pdata->dev, "Fail to start app with cmd 0x%x ret %d!\n",
		msg[2], msg[3]);
	pdata->fsm_err = true;
	return;
}

static void clean_up_uart_keyboard(struct rm_pogo_data *pdata)
{
	if (pdata->serdev_ready)
		pogo_serdev_close(pdata);
	pdata->serdev_ready = false;

	if (pdata->kb_dev == NULL)
		return;
	input_unregister_device(pdata->kb_dev);
	pdata->kb_dev = NULL;
}

static bool fsm_entry_uart_keyboard(struct rm_pogo_data *pdata)
{
	int ret;

	/* No ack for any host cmd is needed anymore. */
	pdata->tx_ack_required = false;

	/* keyboard device is still alive */
	if (pdata->kb_dev != NULL)
		return true;

	ret = pogo_register_uart_keyboard(pdata);
	if (ret) {
		dev_err(pdata->dev, "unable to register kb_dev device\n");
		return false;
	}

	return true;
}

static bool fsm_exit_uart_keyboard(struct rm_pogo_data *pdata)
{
	clean_up_uart_keyboard(pdata);
	return true;
}

static void fsm_routine_uart_keyboard(struct rm_pogo_data *pdata)
{
	int count, ret, len, len_max;
	u8 msg[ONE_WIRE_MCU_MSG_SIZE];

	if (!(pdata->app_session & APP_SESSION_UART))
		return;

	GET_AND_VERIFY_MSG(msg, ONE_WIRE_MCU_MSG_SIZE, , false);

	switch (msg[2]) {
	case KB_REPORT_KEY:
		mod_timer(&pdata->alive_timer,
			  jiffies + msecs_to_jiffies(pdata->alive_timeout));
		dev_dbg(pdata->dev, "report key val 0x%x\n", msg[3]);
		pogo_keyboard_report(pdata, msg[3]);
		return;
	case KB_REPORT_ALIVE:
		mod_timer(&pdata->alive_timer,
			  jiffies + msecs_to_jiffies(pdata->alive_timeout));

		if (pdata->user_command != POGO_CMD_NONE) {
			memset(pdata->user_command_response, 0x00,
				sizeof(pdata->user_command_response));
			/* This will work also if there is no data to send.
			 * In that case pdata->user_command == NULL and
			 * pdata->user_command_data_len == 0 */
			dev_dbg(pdata->dev,
				"sending user cmd 0x%x with data of length %d\n",
				pdata->user_command,
				pdata->user_command_data_len);
			ret = pogo_onewire_write(
				pdata, pdata->user_command, 0,
				pdata->user_command_data,
				pdata->user_command_data_len,
				true);
			if (ret != 0) {
				dev_err(pdata->dev,
					"failed sending user cmd 0x%x\n",
					pdata->user_command);
			}
		}
		mod_timer(&pdata->alive_timer,
			  jiffies + msecs_to_jiffies(pdata->alive_timeout));
		return;
	case POGO_CMD_ATTRIBUTE_READ:
		len = packet_payload_len(msg);
		update_pdata_from_attribute_read_response(pdata, &msg[3], len);
		mod_timer(&pdata->alive_timer,
			  jiffies + msecs_to_jiffies(pdata->alive_timeout));
		break;
	case POGO_CMD_ATTRIBUTE_WRITE:
		len = packet_payload_len(msg);
		check_attribute_write_response(pdata, &msg[3], len);
		mod_timer(&pdata->alive_timer,
			  jiffies + msecs_to_jiffies(pdata->alive_timeout));
		break;
	case POGO_CMD_ENTER_SUSPEND:
		/*
		 * The state machine has to be reset here so that the device is
		 * ready for discovery
		 */
		dev_dbg(pdata->dev,
			"%s: received enter suspend command ack, exiting FSM\n",
			__func__);
		pdata->fsm_err = true;
		break;
	default:
		if (pdata->user_command == POGO_CMD_NONE) {
				dev_err(pdata->dev, "Unsupported cmd 0x%x\n",
					msg[2]);
				return;
		} else {
				break;
		}
	}
	/*
	 * We should only get here if the command is anything else than
	 * KB_REPORT_KEY or KB_REPORT_ALIVE as these commands are not allowed
	 * to be sent as user commands
	 */
	if (pdata->user_command != POGO_CMD_NONE) {
		if (msg[2] == pdata->user_command) {
			dev_dbg(pdata->dev, "user command reply 0x%x\n", msg[2]);
			len = packet_len(msg);
			len_max = sizeof(pdata->user_command_response);
			if (len > len_max) {
				dev_warn(pdata->dev,
				"user command reply length (%d) exceeded allowed limit (%d)\n",
					len, len_max);
				/*
				 * We choose not to abort here but to limit the output data to what the
				 * response buffer can hold. This will be useful for debugging purposes.
				 */
				len = len_max;
			}
			memcpy(pdata->user_command_response, msg, len);
		} else {
			dev_dbg(pdata->dev,
				"wrong user command reply 0x%x, expected 0x%x\n",
					msg[2], pdata->user_command);
		}

		INIT_USER_COMMAND(POGO_CMD_NONE, 0);
	}
}

static bool fsm_entry_usb_keyboard(struct rm_pogo_data *pdata)
{
	/* Not implemented */
	dev_err(pdata->dev, "%s: USB keyboard mode not implemented.\n",
		__func__);
	return false;
}

static int event_fw_write(struct rm_pogo_data *pdata)
{
	return pdata->fw_write == true;
}

static int event_enter_app_session(struct rm_pogo_data *pdata)
{
	return !!(pdata->app_session & APP_SESSION_READY);
}

static int event_negotiate_to_uart_dev(struct rm_pogo_data *pdata)
{
	return pdata->app_session == (APP_SESSION_UART | APP_SESSION_READY);
}

static int event_negotiate_to_otg_dev(struct rm_pogo_data *pdata)
{
	return pdata->app_session == (APP_SESSION_USB | APP_SESSION_READY);
}

struct state_transit state_transit_INIT[] = {
	{event_fsm_error,	POGO_STATE__INIT},
	{event_always_true,	POGO_STATE__ONEWIRE_IDLE},
};

struct state_transit state_transit_ONEWIRE_IDLE[] = {
	{event_vbus_short,		POGO_STATE__ONEWIRE_IDLE},
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
	{event_gpio_is_connected,	POGO_STATE__ONEWIRE_ENUM},
};

struct state_transit state_transit_ONEWIRE_ENUM[] = {
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
	{event_pogo_found,		POGO_STATE__ONEWIRE_NEGO},
};

static struct state_transit state_transit_ONEWIRE_NEGO[] = {
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
	{event_fw_write,		POGO_STATE__FW_WRITE},
	{event_enter_app_session,	POGO_STATE__ONEWIRE_AUTH},
};

static struct state_transit state_transit_FW_WRITE[] = {
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
	{event_enter_app_session,	POGO_STATE__ONEWIRE_AUTH},
};

struct state_transit state_transit_ONEWIRE_AUTH[] = {
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
	{event_mcu_authenticated,	POGO_STATE__START_APP},
};

static struct state_transit state_transit_START_APP[] = {
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
	{event_negotiate_to_uart_dev,	POGO_STATE__UART_SESSION},
	{event_negotiate_to_otg_dev,	POGO_STATE__USB_SESSION}
};

static struct state_transit state_transit_UART_SESSION[] = {
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
};

static struct state_transit state_transit_USB_SESSION[] = {
	{event_fsm_error,		POGO_STATE__ONEWIRE_IDLE},
	/* hack to debug. Add usb session logic later */
	{event_always_true,		POGO_STATE__ONEWIRE_IDLE},
};

/* top level FSM states and transition tables */
static struct fsm_state fsm_states[] = {
	{"init",	NULL, NULL, NULL,
		ARRAY_AND_SIZE(state_transit_INIT)},

	{"idle",	NULL, fsm_entry_idle, fsm_exit_idle,
		ARRAY_AND_SIZE(state_transit_ONEWIRE_IDLE)},

	{"enumerate",	fsm_routine_enumerate, fsm_entry_enumerate, NULL,
		ARRAY_AND_SIZE(state_transit_ONEWIRE_ENUM)},

	{"authenticate", fsm_routine_authentication, fsm_entry_authentication, NULL,
		ARRAY_AND_SIZE(state_transit_ONEWIRE_AUTH)},

	{"negotiate",	fsm_routine_negotiation,
		fsm_entry_negotiation, fsm_exit_negotiation,
		ARRAY_AND_SIZE(state_transit_ONEWIRE_NEGO)},

	{"fw_write",	fsm_routine_fw_write,
		fsm_entry_fw_write, fsm_exit_fw_write,
		ARRAY_AND_SIZE(state_transit_FW_WRITE)},

	{"start_app",	fsm_routine_start_app, fsm_entry_start_app,
		NULL,
		ARRAY_AND_SIZE(state_transit_START_APP),},

	{"app_uart",	fsm_routine_uart_keyboard, fsm_entry_uart_keyboard,
		fsm_exit_uart_keyboard,
		ARRAY_AND_SIZE(state_transit_UART_SESSION),},

	{"app_usb",	NULL, fsm_entry_usb_keyboard, NULL,
		ARRAY_AND_SIZE(state_transit_USB_SESSION)},
};

void fsm_run_and_update_state(struct rm_pogo_data *pdata)
{
	struct fsm_state *state = &fsm_states[pdata->pogo_fsm_state];
	struct state_transit *tran_table = state->transit_table;
	int tran_num = state->transit_table_size;
	int idx;

	dev_dbg(pdata->dev, "----FSM run in state: %s\n", state->name);
	if (state->routine_func && !pdata->fsm_err)
		state->routine_func(pdata);

	for (idx = 0; idx < tran_num; idx++) {
		/* check new events */
		if (true == tran_table[idx].event_func(pdata)) {
			dev_dbg(pdata->dev, "The num %d event is true",
				idx + 1);
			if (state->exit_func) {
				dev_dbg(pdata->dev,
					"%s: running exit func of %s\n",
					__func__, state->name);
				if (!state->exit_func(pdata)) {
					dev_warn(
						pdata->dev,
						"%s: exit function of %s failed, resetting\n",
						__func__, state->name);
					pdata->fsm_err = true;
				}
			}
			/* switch to new state on the event triggering */
			pdata->pogo_fsm_state = tran_table[idx].to_state;
			state = &fsm_states[pdata->pogo_fsm_state];
			dev_dbg(pdata->dev, "====FSM switches to state %s..",
				state->name);
			if (state->entry_func) {
				dev_dbg(pdata->dev,
					"%s: running entry func of %s\n",
					__func__, state->name);
				if (!state->entry_func(pdata)) {
					dev_warn(
						pdata->dev,
						"%s: entry function of %s failed, resetting\n",
						__func__, state->name);
					pdata->fsm_err = true;
					continue;
				}
			}
			return;
		}
	}
};

static int pogo_fsm_thread(void *data)
{
	struct rm_pogo_data *pdata = data;

	set_freezable();
	do {
		try_to_freeze();

		set_current_state(TASK_RUNNING);

		mutex_lock(&pdata->lock);
		fsm_run_and_update_state(pdata);

		set_current_state(TASK_INTERRUPTIBLE);

		mutex_unlock(&pdata->lock);

		if (kthread_should_stop()) {
			set_current_state(TASK_RUNNING);
			break;
		}
		schedule();
	} while (1);

	return 0;
}

static void pogo_detach_work(struct work_struct *work)
{
	struct delayed_work *delayed_work =
		container_of(work, struct delayed_work, work);

	struct rm_pogo_data *pdata = container_of(
		delayed_work, struct rm_pogo_data, mcu_detach_work);

	dev_dbg(pdata->dev, "%s: MCU detached.\n", __func__);
	mutex_lock(&pdata->lock);
	/* Don't run detach work if this work has just been canceled/the state
	 * machine doesn't want it to execute anymore. */
	if (pdata->mcu_detach_work_active) {
		pdata->fsm_err = true;
		pdata->mcu_detach_work_active = false;
		wake_up_process(pdata->fsm_thread);
	}
	mutex_unlock(&pdata->lock);
}

static void pogo_queue_detach_work(struct timer_list *t)
{
	struct rm_pogo_data *pdata = from_timer(pdata, t, alive_timer);

	pdata->mcu_detach_work_active = true;
	queue_delayed_work(system_power_efficient_wq, &pdata->mcu_detach_work, 0);
}

int pogo_init_fsm(struct rm_pogo_data *pdata)
{
	spin_lock_init(&pdata->spin_lock);

	pdata->alive_timeout = ALIVE_INTERV_MS;
	INIT_DELAYED_WORK(&pdata->mcu_detach_work, pogo_detach_work);
	timer_setup(&pdata->alive_timer, pogo_queue_detach_work, 0);

	pdata->fsm_thread = kthread_create(pogo_fsm_thread,
					      pdata, "pogo_fsm");
	if (IS_ERR(pdata->fsm_thread)) {
		pr_err("%s: cannot create pogo fsm thread\n", __func__);
		return PTR_ERR(pdata->fsm_thread);
	}

	/* Enabled by default */
	pdata->fw_write_enabled = default_fw_write_enabled;

	/* Shouldn't be necessary during init, but who knows what happens to this code in the future. */
	mutex_lock(&pdata->lock);
	wake_up_process(pdata->fsm_thread);
	mutex_unlock(&pdata->lock);

	return 0;
}

void pogo_exit_fsm(struct rm_pogo_data *pdata)
{
	pdata->fsm_err = true;
	mutex_lock(&pdata->lock);
	wake_up_process(pdata->fsm_thread);
	mutex_unlock(&pdata->lock);
	msleep(2);

	del_timer_sync(&pdata->alive_timer);
	pdata->mcu_detach_work_active = false;
	cancel_delayed_work(&pdata->mcu_detach_work);

	if (!IS_ERR_OR_NULL(pdata->fsm_thread))
		kthread_stop(pdata->fsm_thread);
}

/*
 * The FSM thread should be frozen while this runs. After this callback has run
 * there should be no more timers, scheduled work etc running which might
 * interfere with FSM state. The onewire pin should be in a good state for the
 * relevant target suspend level. IRQs should be ensured to be enabled if we
 * want to be able to wake from onewire IRQs. irq_enable_wake is called one
 * level up instead of here.
 */
void fsm_suspend(struct rm_pogo_data *pdata, bool standby)
{
	if (standby && pdata->pogo_fsm_state == POGO_STATE__UART_SESSION) {
		if (pogo_onewire_write(pdata, POGO_CMD_ENTER_SUSPEND, 0, NULL,
				       0, false)) {
			dev_warn(pdata->dev,
				 "%s: failed to send suspend command to MCU\n",
				 __func__);
		} else
			msleep(1);
	}

	/* Stop all timers and scheduled work. For now we'll require
	 * re-enumeration after resume to simplify things, though we
	 * may want to change this for the future.
	 *
	 * To that end we reset the FSM unless we're already in idle to trigger
	 * a re-enumeration after resume.
	 */
	/* Fix internal state */
	if (pdata->mcu_detach_work_active) {
		pdata->mcu_detach_work_active = false;
		del_timer_sync(&pdata->alive_timer);
		cancel_delayed_work_sync(&pdata->mcu_detach_work);
	}
	if (pdata->tx_ack_required) {
		pdata->tx_ack_required = false;
		cancel_delayed_work_sync(
			&pdata->uart_tx_ack_watchdog_work_queue);
	}
	if (pdata->pogo_fsm_state != POGO_STATE__ONEWIRE_IDLE) {
		pdata->fsm_err = true;
	}
	/* Fix GPIO and power stuff */
	/* We only enable GPIO mode if we enter standby. For deep sleep we set
	 * the onewire pin to a state which will prevent the MCU from draining
	 * enough power to boot and request VBUS to avoid unecessary power
	 * usage.
	 */
	if (standby)
	{
		pogo_switch_one_wire_mux_state(pdata, POGO_ONEWIRE_STATE__GPIO);
	}
	else {
		pogo_switch_one_wire_mux_state(pdata, POGO_ONEWIRE_STATE__GPIO_100K_PD_100K_PD);
	}
	pogo_change_otg_charger_mode_int(pdata, POGO_CHARGERMODE_CHARGE);

	/*
	 * If we're not in idle, we need to enable IRQs for the duration of the sleep.
	 * Only for standby since we can't wake other sleep states via onewire anyways.
	 */
	if (standby && pdata->pogo_fsm_state != POGO_STATE__ONEWIRE_IDLE) {
		pogo_activate_gpio_irq(pdata);
	}
}

/*
 * The FSM thread should be frozen while this runs.
 *
 * Opposite of fsm_suspend. Make sure the FSM can continue and doesn't get
 * stuck or breaks.
 */
void fsm_resume(struct rm_pogo_data *pdata, bool standby)
{
	/*
	 * The FSM will go back to idle. During suspend the charger has been
	 * completely powered off. We need to switch to some other GPIO state
	 * and back to force re-activating the default GPIO state so that we
	 * can be reasonably sure to be in a known good state on the onewire
	 * pin.
	 */
	pogo_switch_one_wire_mux_state(pdata, POGO_ONEWIRE_STATE__GPIO_5K_5K);
	pogo_switch_one_wire_mux_state(pdata, POGO_ONEWIRE_STATE__GPIO);

	/* Clean up having enabled IRQ during suspend only */
	if (standby && pdata->pogo_fsm_state != POGO_STATE__ONEWIRE_IDLE) {
		pogo_uninit_gpio_irq(pdata);
	}
}
