/*
 * reMarkable OTG Control
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

#include "otgcontrol.h"
#include "otgcontrol_fsm.h"
#include "otgcontrol_onewire.h"
#include "otgcontrol_charging_ctrl.h"
#include "otgcontrol_dr_mode.h"

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/power_supply.h>
#include <linux/delay.h>

/******************************************************************************
 * INTERNAL FUNCTIONS
 *****************************************************************************/
static int otgcontrol_reset_fsm(struct rm_otgcontrol_data *otgc_data);

static int otgcontrol_handle_state_MANUAL_CONTROL(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_handle_state_ONEWIRE_AUTH_NOT_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_handle_state_ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_handle_state_ONEWIRE_AUTH_DEVICE_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_handle_state_USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_handle_state_USB_NO_AUTH_NOT_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_handle_state_USB_NO_AUTH_DEVICE_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_handle_state_HOST_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param);

static int otgcontrol_do_controller_mode_change_procedure(
		struct rm_otgcontrol_data *otgc_data,
		void *param);

static int otgcontrol_do_authenticate_calling_application(
		struct rm_otgcontrol_data *otgc_data,
		int mode_requested);

static int otgcontrol_do_verify_application_challenge_reply(
		struct rm_otgcontrol_data *otgc_data,
		void *param);

static int otgcontrol_do_verify_device_challenge_reply(
		struct rm_otgcontrol_data *otgc_data,
		void *param);

static int otgcontrol_do_set_controlmode(
		struct rm_otgcontrol_data *otgc_data,
		int mode);

static int otgcontrol_do_device_connected_procedure(
		struct rm_otgcontrol_data *otgc_data,
		bool authentication_required);

static int otgcontrol_do_device_disconnected_procedure(
		struct rm_otgcontrol_data *otgc_data);

static int otgcontrol_do_start_onewire_authentication(
		struct rm_otgcontrol_data *otgc_data);

static const char *otgcontrol_controllerstate_name(int state)
{
	switch(state)
	{
	case OTG1_STATE__MANUAL_CONTROL:
		return "OTG1_STATE__MANUAL_CONTROL";
		break;
	case OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED:
		return "OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED";
		break;
	case OTG1_STATE__ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS:
		return "OTG1_STATE__ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS";
		break;
	case OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED:
		return "OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED";
		break;
	case OTG1_STATE__USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE:
		return "OTG1_STATE__USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE";
		break;
	case OTG1_STATE__USB_NO_AUTH_NOT_CONNECTED:
		return "OTG1_STATE__USB_NO_AUTH_NOT_CONNECTED";
		break;
	case OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED:
		return "OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED";
		break;
	case OTG1_STATE__HOST_CONNECTED:
		return "OTG1_STATE__HOST_CONNECTED";
		break;
	default:
		return "INVALID STATE";
	}
}

static bool otgcontrol_controllerstate_is_valid(int state)
{
	switch(state)
	{
	case OTG1_STATE__MANUAL_CONTROL:
	case OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED:
	case OTG1_STATE__ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS:
	case OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED:
	case OTG1_STATE__USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE:
	case OTG1_STATE__USB_NO_AUTH_NOT_CONNECTED:
	case OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED:
	case OTG1_STATE__HOST_CONNECTED:
		return true;
	default:
		return false;
	}
}


int otgcontrol_init_fsm(struct rm_otgcontrol_data *otgc_data)
{
	/* For now, only a simple FSM reset is required
	 * init routine has been kept however, to possibly contain other
	 * initiation code when the solution evolves
	 */
	return otgcontrol_reset_fsm(otgc_data);
}

static int otgcontrol_reset_fsm(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	/* Initially, set the DR mode to device, shut off the OTG power, and
	 * mux the onewire at GPIO
	*/
	ret = otgcontrol_change_otg_charger_mode_int(otgc_data,
						     OTG1_CHARGERMODE_CHARGE);
	if (ret > 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to reset FSM "
			"(unable to change charger mode)\n",
			__func__);

		return ret;
	}

	ret = otgcontrol_set_dr_mode(otgc_data, OTG1_DR_MODE__DEVICE);
	if (ret > 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to reset FSM "
			"(unable to set defaul USB OTG DR mode)\n",
			__func__);

		return ret;
	}

	ret = otgcontrol_switch_one_wire_mux_state(otgc_data,
						   OTG1_ONEWIRE_STATE__GPIO);
	if (ret > 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to reset FSM "
			"(unable to set default one-wire pinmux state)\n",
			__func__);

		return ret;
	}

	dev_dbg(otgc_data->dev,
		"%s: Activating one-wire GPIO IRQ\n",
		__func__);

	/* otgcontrol_activate_gpio_irq(otgc_data);*/
	otgcontrol_deactivate_gpio_irq(otgc_data);

	dev_dbg(otgc_data->dev,
		"%s: Checking if device is connected and initiating default "
		"state\n",
		__func__);

	otgc_data->otg1_controllermode = OTG_MODE__ONEWIRE_AUTH;
	if(otgcontrol_get_current_gpio_state(otgc_data) ==
			OTG1_ONEWIRE_GPIO_STATE__DEVICE_CONNECTED) {
		dev_dbg(otgc_data->dev,
			"Device is connected, "
			"doing default authenticated device connection procedure\n");

		ret = otgcontrol_do_device_connected_procedure(otgc_data, true);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"Unable to complete device connection procedure\n");

			dev_dbg(otgc_data->dev,
				"Waiting for device disconnect/connect\n");

			otgc_data->otg_controlstate =
					OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		}
	}
	else {
		dev_dbg(otgc_data->dev,
			"%s: Device is not connected, so wait for device to connect\n",
			__func__);
		otgc_data->otg_controlstate =
				OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
	}

	return 0;
}

int otgcontrol_handleInput(struct rm_otgcontrol_data *otgc_data,
			   int signal,
			   void* param)
{
	switch(otgc_data->otg_controlstate)
	{
	case OTG1_STATE__MANUAL_CONTROL:
		return otgcontrol_handle_state_MANUAL_CONTROL(
					otgc_data,
					signal,
					param);
		break;
	case OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED:
		return otgcontrol_handle_state_ONEWIRE_AUTH_NOT_CONNECTED(
					otgc_data,
					signal,
					param);
		break;
	case OTG1_STATE__ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS:
		return otgcontrol_handle_state_ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS(
					otgc_data,
					signal,
					param);
		break;
	case OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED:
		return otgcontrol_handle_state_ONEWIRE_AUTH_DEVICE_CONNECTED(
					otgc_data,
					signal,
					param);
		break;
	case OTG1_STATE__USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE:
		return otgcontrol_handle_state_USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE(
					otgc_data,
					signal,
					param);
		break;
	case OTG1_STATE__USB_NO_AUTH_NOT_CONNECTED:
		return otgcontrol_handle_state_USB_NO_AUTH_NOT_CONNECTED(
					otgc_data,
					signal,
					param);
		break;
	case OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED:
		return otgcontrol_handle_state_USB_NO_AUTH_DEVICE_CONNECTED(
					otgc_data,
					signal,
					param);
		break;
	case OTG1_STATE__HOST_CONNECTED:
		return otgcontrol_handle_state_HOST_CONNECTED(
					otgc_data,
					signal,
					param);
		break;
	default:
		dev_dbg(otgc_data->dev,
			"%s: Current control state is invalid, "
			"resetting state machine to state "
			"ONEWIRE_AUTH_NOT_CONNECTED\n",
			__func__);
		return otgcontrol_reset_fsm(otgc_data);
	}
}

static int otgcontrol_handle_state_MANUAL_CONTROL(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
	case OTG1_EVENT__CHARGER_DISCONNECTED:
	case OTG1_EVENT__DEVICE_CONNECTED:
	case OTG1_EVENT__DEVICE_DISCONNECTED:
	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: In manual control mode, \
			no automatic action taken\n",
			__func__);
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested\n",
			__func__);

		return otgcontrol_do_controller_mode_change_procedure(
					otgc_data,
					param);
		break;

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d), \
			but in manual mode so ignoring\n",
			__func__, signal);
	}
	return 0;
}

static int otgcontrol_handle_state_ONEWIRE_AUTH_NOT_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is connected, disabling all OTG features "
			"until host is disconnected\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is disconnected (NOT EXPECTED), but just "
			"ignore this and continue to wait for device "
			"connection\n",
			__func__);
		return 0;

	case OTG1_EVENT__DEVICE_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device connected, doing authenticated connection "
			"procedure\n",
			__func__);

		ret = otgcontrol_do_device_connected_procedure(otgc_data,
							       false);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete device connection "
				"procedure, resetting fsm as an attempt to "
				"recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device disconnected, doing disconnection "
			"procedure\n",
			__func__);

		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete disconnection "
				"procedure, resetting fsm as an attempt to "
				"recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected message received from device, it might "
			"be in an unknown state, doing fsm reset procedure to "
			"power-cycle device\n",
			__func__);
		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested, start caller "
			"authentication\n",
			__func__);

		return otgcontrol_do_controller_mode_change_procedure(
					otgc_data,
					param);

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected mode change challenge reply received, "
			"application might be in an unknown state - ignoring\n",
			__func__);
		return 0;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Charger-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: USB OTG DR-mode change requested, not valid in "
			"this state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: TIMEOUT, but not waiting for anyting other than "
			"device connection - ignoring\n",
			__func__);
		return 0;

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d)\n",
			__func__,
			signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is connected, disabling all OTG features "
			"until host is disconnected\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is disconnected (NOT EXPECTED), but just "
			"ignore this and continue to wait for one-wire "
			"handshake response\n",
			__func__);
		return 0;

	case OTG1_EVENT__DEVICE_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device connected (UNEXPECTED), re-starting "
			"authenticated connection procedure\n",
			__func__);

		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete device connection "
				"procedure, resetting fsm as an attempt to "
				"recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device disconnected, doing disconnection "
			"procedure\n",
			__func__);

		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete disconnection "
				"procedure, resetting fsm as an attempt to "
				"recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		dev_dbg(otgc_data->dev,
			"%s: Challenge reply received from connected device\n",
			__func__);

		ret = otgcontrol_do_verify_device_challenge_reply(otgc_data,
								  param);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to verify device challenge reply, "
				"resetting fsm as an attempt to recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested, start caller "
			"authentication\n",
			__func__);

		return otgcontrol_do_controller_mode_change_procedure(otgc_data,
								      param);

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected mode change challenge reply received, "
			"application might be in an unknown state - ignoring\n",
			__func__);
		return 0;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Charger-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: USB OTG DR-mode change requested, not valid in "
			"this state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: TIMEOUT, no response from connected device within "
			"expected amount of time\n",
			__func__);

		dev_dbg(otgc_data->dev,
			"%s: Reset fsm, shutting off power and waiting for "
			"disconnect/re-connect\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d)\n",
			__func__,
			signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_ONEWIRE_AUTH_DEVICE_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is connected, disabling all OTG features "
			"until host is disconnected\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is disconnected (NOT EXPECTED), but just "
			"ignore this and continue being connected to "
			"authenticated device\n",
			__func__);
		return 0;

	case OTG1_EVENT__DEVICE_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device connected (UNEXPECTED), re-starting "
			"authenticated connection procedure\n",
			__func__);

		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete device connection "
				"procedure, resetting fsm as an attempt to "
				"recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device disconnected, doing disconnection "
			"procedure\n",
			__func__);

		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete disconnection procedure, "
				"resetting fsm as an attempt to recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected message received from device, it might "
			"be in an unknown state, doing fsm reset procedure to "
			"power-cycle device\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested\n",
			__func__);

		return otgcontrol_do_controller_mode_change_procedure(otgc_data,
								      param);

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected mode change challenge "
			"reply received, application might be "
			"in an unknown state - ignoring\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Charger-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: USB OTG DR-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: TIMEOUT, but not waiting for anyting other than "
			"device disconnection - ignoring\n",
			__func__);
		return 0;

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d)\n",
			__func__,
			signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is connected, disabling all OTG features until "
			"host is disconnected\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is disconnected (NOT EXPECTED), but just "
			"ignore this and continue to wait for challenge response "
			"from application\n",
			__func__);
		return 0;

	case OTG1_EVENT__DEVICE_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device connected (UNEXPECTED), re-starting "
			"authenticated connection procedure\n",
			__func__);

		ret = otgcontrol_do_device_connected_procedure(otgc_data,
							       false);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete device connection "
				"procedure, resetting fsm as an attempt to "
				"recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device disconnected, doing disconnection "
			"procedure\n",
			__func__);

		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete disconnection procedure, "
				"resetting fsm as an attempt to recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected message received from device, it might "
			"be in an unknown state, doing fsm reset procedure to "
			"power-cycle device\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested while already "
			"waiting for challenge response, restart procedure\n",
			__func__);

		return otgcontrol_do_controller_mode_change_procedure(otgc_data,
								      param);

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		dev_dbg(otgc_data->dev,
			"%s: Received expected challenge response, changing to "
			"unauthenticated USB OTG mode\n",
			__func__);

		return otgcontrol_do_verify_application_challenge_reply(otgc_data,
									param);

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Charger-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: USB OTG DR-mode change requested, not valid in "
			"this state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: TIMEOUT, no response from application requesting "
			"mode change within expected amount of time\n",
			__func__);

		dev_dbg(otgc_data->dev,
			"%s: Reset fsm, going back to authenticated mode, "
			"requiring new mode change request\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d)\n",
			__func__,
			signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_USB_NO_AUTH_NOT_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is connected, disabling all OTG features "
			"until host is disconnected\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is disconnected (NOT EXPECTED), but just "
			"ignore this and continue to wait for device connection\n",
			__func__);
		return 0;

	case OTG1_EVENT__DEVICE_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device connected, doing un-authenticated "
			"connection procedure\n",
			__func__);

		ret = otgcontrol_do_device_connected_procedure(otgc_data,
							       false);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete device connection "
				"procedure, resetting fsm as an attempt to "
				"recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device disconnected, doing disconnection procedure\n",
			__func__);

		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete disconnection procedure, "
				"resetting fsm as an attempt to recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected message received from device, it might "
			"be in an unknown state, doing fsm reset procedure to "
			"power-cycle device\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested, start caller "
			"authentication\n",
			__func__);

		return otgcontrol_do_controller_mode_change_procedure(otgc_data,
								      param);

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected mode change challenge reply received, "
			"application might be in an unknown state - ignoring\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Charger-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: USB OTG DR-mode change requested, not valid in "
			"this state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: TIMEOUT, but not waiting for anyting other than "
			"device connection - ignoring\n",
			__func__);
		return 0;

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d)\n",
			__func__,
			signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_USB_NO_AUTH_DEVICE_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is connected, disabling all OTG features "
			"until host is disconnected\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is disconnected (NOT EXPECTED), but just "
			"ignore this and continue being connected to authenticated "
			"device\n",
			__func__);
		return 0;

	case OTG1_EVENT__DEVICE_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device connected (UNEXPECTED), re-starting "
			"un-authenticated connection procedure\n",
			__func__);

		ret = otgcontrol_do_device_connected_procedure(otgc_data,
							       false);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete device "
				"connection procedure, resetting "
				"fsm as an attempt to recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device disconnected, doing disconnection procedure\n",
			__func__);

		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			dev_err(otgc_data->dev,
				"%s: Failed to complete disconnection procedure, "
				"resetting fsm as an attempt to recover\n",
				__func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected message received from device, it might "
			"be in an unknown state, doing fsm reset procedure to "
			"power-cycle device\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested\n",
			__func__);

		return otgcontrol_do_controller_mode_change_procedure(
					otgc_data,
					param);

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected mode change challenge reply received, "
			"application might be in an unknown state - ignoring\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Charger-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: USB OTG DR-mode change requested, not valid in "
			"this state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: TIMEOUT, but not waiting for anyting other than "
			"device disconnection - ignoring\n",
			__func__);
		return 0;

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d)\n",
			__func__,
			signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_HOST_CONNECTED(
		struct rm_otgcontrol_data *otgc_data,
		int signal,
		void *param)
{
	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host connection detected in already connected "
			"state, keeping all OTG features disabled until host "
			"is disconnected\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Host is disconnected, reset fsm into authenticated "
			"mode\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__DEVICE_CONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device connected (UNEXPECTED), reset fsm into "
			"authenticated mode\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		dev_dbg(otgc_data->dev,
			"%s: Device disconnected (UNEXPECTED), reset fsm into "
			"authenticated mode\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected message received from device, reset "
			"fsm into authenticated mode\n",
			__func__);

		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Controller-mode change requested, not valid in "
			"this state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		dev_dbg(otgc_data->dev,
			"%s: Unexpected mode change challenge reply received, "
			"application might be in an unknown state - ignoring\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: Charger-mode change requested, not valid in this "
			"state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		dev_dbg(otgc_data->dev,
			"%s: USB OTG DR-mode change requested, not valid in "
			"this state\n",
			__func__);
		return -EINVAL;

	case OTG1_EVENT__TIMEOUT:
		dev_dbg(otgc_data->dev,
			"%s: TIMEOUT, but not waiting for anyting other than "
			"host disconnection - ignoring\n",
			__func__);
		return 0;

	default:
		dev_dbg(otgc_data->dev,
			"%s: Unknown signal/event (%d)\n",
			__func__,
			signal);
		return -EINVAL;
	}
}

static int otgcontrol_do_controller_mode_change_procedure(
		struct rm_otgcontrol_data *otgc_data,
		void *param)
{
	int mode_requested = *(int*)param;

	/* Depending on requested mode.. */

	/* 1.
	 * Enable challenge/reply properties, and set challenge to be read by
	 * application */

	/* 2.
	 * Wait for reply for a while */

	/* But for now, just set mode */
	switch(mode_requested)
	{
	case OTG_MODE__MANUAL_CONTROL:
	case OTG_MODE__USB_NO_AUTH:
		dev_err(otgc_data->dev,
			"%s: Change to manual or non-authenticated mode "
			"requires authentication of calling application\n",
			__func__);

		return otgcontrol_do_authenticate_calling_application(
					otgc_data,
					mode_requested);
		break;

	default:
		/* Invalid mode => authenticated mode */
		dev_err(otgc_data->dev,
			"%s: Invalid controller mode or authenticated mode "
			"requested (%d), setting one-wire authenticated mode\n",
			__func__,
			mode_requested);

		otgcontrol_do_set_controlmode(otgc_data,
					      OTG_MODE__ONEWIRE_AUTH);
		return 0;
	}
}

static int otgcontrol_do_authenticate_calling_application(
		struct rm_otgcontrol_data *otgc_data,
		int mode_requested)
{
	switch(mode_requested)
	{
	case OTG_MODE__MANUAL_CONTROL:
	case OTG_MODE__USB_NO_AUTH:
		/* NORMALLY, A CHALLENGE WOULD BE SENT TO CALLING APPLICATION */
		/* IDEA:
		 * SHOW CHALLENGE PROPERTY
		 * WAIT FOR APPLICATION TO READ THIS
		 * SHOW REPLY PROPERTY, TO BE PRESENT WHEN APPLICATION
		 * HAS READ THE CHALLENGE
		 * GET REPLY AND VERIFY
		 * HIDE CHALLENGE/REPLY PROPERTIES
		 *
		 * FOR NOW, JUST CALL REPLY VERIFY ROUTINE, WHICH CURRENTLY
		 * HAS NO VERIFICATION ALGORITHM IMPLEMENTED, AND
		 * WILL DO A SIMPLE CONNECTION CHECK AND SET STATE ACCORDING
		 * TO THE REQUESTED MODE AND THE CURRENT CONNECTION STATE
		 */
		dev_dbg(otgc_data->dev,
			"%s: Authentication of calling application not"
			"implemented, skipping\n",
			__func__);

		otgc_data->mode_requested = mode_requested;
		otgcontrol_do_verify_application_challenge_reply(otgc_data,
								 NULL);
		break;

	case OTG_MODE__ONEWIRE_AUTH:
	default:
		/* No authentication required to enable authenticated mode */
		otgcontrol_do_set_controlmode(otgc_data,
					      mode_requested);
	}
	return 0;
}

static int otgcontrol_do_verify_application_challenge_reply(
		struct rm_otgcontrol_data *otgc_data,
		void *param)
{
	switch(otgc_data->mode_requested)
	{
	case OTG_MODE__MANUAL_CONTROL:
		otgcontrol_do_set_controlmode(otgc_data,
					      otgc_data->mode_requested);
		break;

	case OTG_MODE__USB_NO_AUTH:
		otgcontrol_do_set_controlmode(otgc_data,
					      otgc_data->mode_requested);
		break;

	case OTG_MODE__ONEWIRE_AUTH:
	default:
		otgcontrol_do_set_controlmode(otgc_data,
					      otgc_data->mode_requested);
		break;
	}

	return 0;
}

static int otgcontrol_do_verify_device_challenge_reply(
		struct rm_otgcontrol_data *otgc_data,
		void *param)
{
	int ret;
	dev_dbg(otgc_data->dev,
		"%s: For now, just take the challenge to be good and activate "
		"USB OTG Host mode",
		__func__);

	ret = otgcontrol_set_dr_mode(otgc_data, OTG1_DR_MODE__HOST);
	if (ret < 0) {
		dev_err(otgc_data->dev,
			"%s: Failed to set USB OTG host mode\n",
			__func__);
		return ret;
	}

	otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED;
	return 0;
}

static int otgcontrol_do_set_controlmode(
		struct rm_otgcontrol_data *otgc_data,
		int mode)
{
	switch(mode)
	{
	case OTG_MODE__MANUAL_CONTROL:
		dev_dbg(otgc_data->dev,
			"%s: setting MANUAL_CONTROL mode\n",
			__func__);

		otgc_data->otg_controlstate = OTG1_STATE__MANUAL_CONTROL;
		otgc_data->otg1_controllermode = OTG_MODE__MANUAL_CONTROL;

		otgcontrol_deactivate_gpio_irq(otgc_data);
		break;

	case OTG_MODE__ONEWIRE_AUTH:
		if (otgcontrol_get_current_gpio_state(otgc_data) ==
		    OTG1_ONEWIRE_GPIO_STATE__DEVICE_CONNECTED) {
			dev_dbg(otgc_data->dev,
				"%s: Enabling ONE-WIRE AUTHENTICATED mode "
				"(DEVICE CONNECTED)\n",
				__func__);

			otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED;
		}
		else {
			dev_dbg(otgc_data->dev,
				"%s: Enabling ONE-WIRE AUTHENTICATED mode "
				"(DEVICE NOT CONNECTED)\n",
				__func__);

			otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		}
		otgc_data->otg1_controllermode = OTG_MODE__ONEWIRE_AUTH;

		otgcontrol_deactivate_gpio_irq(otgc_data);

		/* Also do a disconnect procedure, setting the charger back
		 * to charger mode and the OTG USB device back to device mode
		 * until proper authenticated device connection has been
		 * implemented
		 */
		otgcontrol_do_device_disconnected_procedure(otgc_data);
		break;

	case OTG_MODE__USB_NO_AUTH:
		if (otgcontrol_get_current_gpio_state(otgc_data) ==
		    OTG1_ONEWIRE_GPIO_STATE__DEVICE_CONNECTED) {
			dev_dbg(otgc_data->dev,
				"%s: Enabling ONE-WIRE NON-AUTHENTICATED mode "
				"(DEVICE CONNECTED)\n",
				__func__);

			/* Due to the fact that this mode is intended for the
			 * test-application requiring un-authenticated USB
			 * equipment to be connected, just enable OTG mode
			 * since the device is already connected when switching
			 * to this mode, to prevent need for unplug and replug
			 */
			otgcontrol_do_device_connected_procedure(otgc_data, false);
			otgc_data->otg_controlstate = OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED;
		}
		else {
			dev_dbg(otgc_data->dev,
				"%s: Enabling ONE-WIRE NON-AUTHENTICATED mode "
				"(DEVICE NOT CONNECTED)\n",
				__func__);

			/* Due to the fact that this device disconnection could
			 * not be detected in other modes, just disable OTG mode
			 * to make sure the charger mode is enabled since the
			 * device is not connected - just to be sure
			 */
			otgcontrol_do_device_disconnected_procedure(otgc_data);
			otgc_data->otg_controlstate = OTG1_STATE__USB_NO_AUTH_NOT_CONNECTED;
		}
		otgc_data->otg1_controllermode = OTG_MODE__USB_NO_AUTH;

		otgcontrol_activate_gpio_irq(otgc_data);
		break;

	default:
		dev_dbg(otgc_data->dev,
			"%s: unable to set control mode (unknown mode %d)\n",
			__func__,
			mode);
		return -EINVAL;
	}

	return 0;
}

static int otgcontrol_do_device_connected_procedure(
		struct rm_otgcontrol_data *otgc_data,
		bool authentication_required)
{
	int ret;

	if (authentication_required) {
		dev_warn(otgc_data->dev,
			 "Authenticated connection not supported, ignoring..\n");
		return  -ENOTSUPP;
	}
	else
	{
		dev_dbg(otgc_data->dev,
			"Authentication not required, activating USB connection\n");

		dev_dbg(otgc_data->dev,
			"Powering up connected device (if no charger is connected)\n");

		ret = otgcontrol_change_otg_charger_mode_int(
					otgc_data,
					OTG1_CHARGERMODE_OTG); /* OTG POWER ON */
		if (ret < 0) {
			dev_dbg(otgc_data->dev,
				"Unable to turn on OTG power, check connections\n");
			return ret;
		}

		/* Sleep to avoid race, let USB driver handle itself before setting DR mode */
		usleep_range(300000, 400000);

		otgcontrol_set_dr_mode(otgc_data,
				       OTG1_DR_MODE__HOST);
		otgc_data->otg_controlstate = OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED;
		otgc_data->otg1_controllermode = OTG_MODE__USB_NO_AUTH;
		return 0;
	}
}

static int otgcontrol_do_device_disconnected_procedure(
		struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	ret = otgcontrol_change_otg_charger_mode_int(
				otgc_data,
				OTG1_CHARGERMODE_CHARGE); /* OTG POWER OFF */
	if (ret < 0) {
		dev_dbg(otgc_data->dev,
			"%s: Unable to turn off OTG power\n",
			__func__);
		return ret;
	}

	ret = otgcontrol_set_dr_mode(otgc_data,
				     OTG1_DR_MODE__DEVICE);
	if (ret < 0) {
		dev_dbg(otgc_data->dev,
			"%s: Unable to set USB OTG device mode\n",
			__func__);
		return ret;
	}

	/* otgcontrol_activate_gpio_irq(otgc_data);*/
	if (otgc_data->otg1_controllermode == OTG_MODE__USB_NO_AUTH) {
		dev_dbg(otgc_data->dev,
			"%s: Activating one-wire GPIO IRQ\n",
			__func__);

		otgcontrol_activate_gpio_irq(otgc_data);
	}
	return 0;
}

static int otgcontrol_do_start_onewire_authentication(
		struct rm_otgcontrol_data *otgc_data)
{
	dev_dbg(otgc_data->dev,
		"%s: Enter\n",
		__func__);

	dev_dbg(otgc_data->dev,
		"%s: PLEASE IMPLEMENT THIS !\n",
		__func__);
	return 0;
}
