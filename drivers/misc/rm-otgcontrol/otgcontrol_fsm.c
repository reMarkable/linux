#include <linux/rm-otgcontrol.h>

#include "otgcontrol_fsm.h"
#include "otgcontrol_onewire.h"
#include "otgcontrol_charging_ctrl.h"
#include "otgcontrol_dr_mode.h"

#include <linux/printk.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/power_supply.h>
#include <linux/delay.h>

/********************************************************
 * INTERNAL FUNCTIONS
 ********************************************************/
static int otgcontrol_reset_fsm(struct rm_otgcontrol_data *otgc_data);

static int otgcontrol_handle_state_MANUAL_CONTROL(struct rm_otgcontrol_data *otgc_data, int signal, void *param);
static int otgcontrol_handle_state_ONEWIRE_AUTH_NOT_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param);
static int otgcontrol_handle_state_ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS(struct rm_otgcontrol_data *otgc_data, int signal, void *param);
static int otgcontrol_handle_state_ONEWIRE_AUTH_DEVICE_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param);
static int otgcontrol_handle_state_USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE(struct rm_otgcontrol_data *otgc_data, int signal, void *param);
static int otgcontrol_handle_state_USB_NO_AUTH_NOT_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param);
static int otgcontrol_handle_state_USB_NO_AUTH_DEVICE_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param);
static int otgcontrol_handle_state_HOST_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param);

static int otgcontrol_do_controller_mode_change_procedure(struct rm_otgcontrol_data *otgc_data, void *param);
static int otgcontrol_do_authenticate_calling_application(struct rm_otgcontrol_data *otgc_data, int mode_requested);
static int otgcontrol_do_verify_application_challenge_reply(struct rm_otgcontrol_data *otgc_data, void *param);
static int otgcontrol_do_verify_device_challenge_reply(struct rm_otgcontrol_data *otgc_data, void *param);
static int otgcontrol_do_set_controlmode(struct rm_otgcontrol_data *otgc_data, int mode);
static int otgcontrol_do_device_connected_procedure(struct rm_otgcontrol_data *otgc_data, bool authentication_required);
static int otgcontrol_do_device_disconnected_procedure(struct rm_otgcontrol_data *otgc_data);
static int otgcontrol_do_start_onewire_authentication(struct rm_otgcontrol_data *otgc_data);


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
	return otgcontrol_reset_fsm(otgc_data);
}

static int otgcontrol_reset_fsm(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	/* Initially, set the DR mode to device, shut off the OTG power, and mux the onewire at GPIO */
	ret = otgcontrol_change_otg_charge_mode(otgc_data, OTG1_CHARGERMODE_CHARGE);
	if (ret > 0) {
		printk("%s: Failed to reset FSM (unable to change charger mode)\n", __func__);
		return ret;
	}

	ret = otgcontrol_set_dr_mode(otgc_data, OTG1_DR_MODE__DEVICE);
	if (ret > 0) {
		printk("%s: Failed to reset FSM (unable to set defaul USB OTG DR mode)\n", __func__);
		return ret;
	}

	ret = otgcontrol_switch_one_wire_mux_state(otgc_data, OTG1_ONEWIRE_STATE__GPIO);
	if (ret > 0) {
		printk("%s: Failed to reset FSM (unable to set default one-wire pinmux state)\n", __func__);
		return ret;
	}

	printk("%s: Activating one-wire GPIO IRQ\n", __func__);
	otgcontrol_activate_gpio_irq(otgc_data);

	printk("%s: Checking if device is connected\n", __func__);
	if(otgcontrol_get_current_gpio_state(otgc_data) == 0) {
		printk("%s: Device is connected, doing default authenticated device connection procedure", __func__);
		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			printk("%s: Failed to reset FSM (unable to complete device connection procedure)\n", __func__);
			printk("%s: Just wait for device disconnect/connect\n", __func__);
			otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
			return ret;
		}
		return 0;
	}
	else {
		printk("%s: Device is not connected, so wait for device to connect\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		return 0;
	}
}

int otgcontrol_handleInput(struct rm_otgcontrol_data *otgc_data, int signal, void* param)
{
	printk("%s: Enter\n", __func__);

	switch(otgc_data->otg_controlstate)
	{
	case OTG1_STATE__MANUAL_CONTROL:
		return otgcontrol_handle_state_MANUAL_CONTROL(otgc_data, signal, param);
		break;
	case OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED:
		return otgcontrol_handle_state_ONEWIRE_AUTH_NOT_CONNECTED(otgc_data, signal, param);
		break;
	case OTG1_STATE__ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS:
		return otgcontrol_handle_state_ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS(otgc_data, signal, param);
		break;
	case OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED:
		return otgcontrol_handle_state_ONEWIRE_AUTH_DEVICE_CONNECTED(otgc_data, signal, param);
		break;
	case OTG1_STATE__USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE:
		return otgcontrol_handle_state_USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE(otgc_data, signal, param);
		break;
	case OTG1_STATE__USB_NO_AUTH_NOT_CONNECTED:
		return otgcontrol_handle_state_USB_NO_AUTH_NOT_CONNECTED(otgc_data, signal, param);
		break;
	case OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED:
		return otgcontrol_handle_state_USB_NO_AUTH_DEVICE_CONNECTED(otgc_data, signal, param);
		break;
	case OTG1_STATE__HOST_CONNECTED:
		return otgcontrol_handle_state_HOST_CONNECTED(otgc_data, signal, param);
		break;
	default:
		printk("%s: Current control state is invalid, resetting state machine to state ONEWIRE_AUTH_NOT_CONNECTED\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
	}
}

static int otgcontrol_handle_state_MANUAL_CONTROL(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
	case OTG1_EVENT__CHARGER_DISCONNECTED:
	case OTG1_EVENT__DEVICE_CONNECTED:
	case OTG1_EVENT__DEVICE_DISCONNECTED:
	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
	case OTG1_EVENT__TIMEOUT:
		printk("%s: In manual control mode, no automatic action taken\n", __func__);
	default:
		printk("%s: Unknown signal/event (%d), but in manual mode so ignoring\n", __func__, signal);
	}
	return 0;
}

static int otgcontrol_handle_state_ONEWIRE_AUTH_NOT_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		printk("%s: Host is connected, disabling all OTG features until host is disconnected\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;
		break;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		printk("%s: Host is disconnected (NOT EXPECTED), but just ignore this and continue to wait for device connection\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__DEVICE_CONNECTED:
		printk("%s: Device connected, doing authenticated connection procedure\n", __func__);
		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			printk("%s: Failed to complete device connection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		printk("%s: Device disconnected, doing disconnection procedure\n", __func__);
		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			printk("%s: Failed to complete disconnection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		printk("%s: Unexpected message received from device, it might be in an unknown state, doing fsm reset procedure to power-cycle device\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		printk("%s: Controller-mode change requested, start caller authentication\n", __func__);
		return otgcontrol_do_controller_mode_change_procedure(otgc_data, param);
		break;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		printk("%s: Unexpected mode change challenge reply received, application might be in an unknown state - ignoring\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		printk("%s: Charger-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		printk("%s: USB OTG DR-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__TIMEOUT:
		printk("%s: TIMEOUT, but not waiting for anyting other than device connection - ignoring\n", __func__);
		return 0;
		break;

	default:
		printk("%s: Unknown signal/event (%d)\n", __func__, signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		printk("%s: Host is connected, disabling all OTG features until host is disconnected\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;
		break;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		printk("%s: Host is disconnected (NOT EXPECTED), but just ignore this and continue to wait for one-wire handshake response\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__DEVICE_CONNECTED:
		printk("%s: Device connected (UNEXPECTED), re-starting authenticated connection procedure\n", __func__);
		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			printk("%s: Failed to complete device connection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		printk("%s: Device disconnected, doing disconnection procedure\n", __func__);
		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			printk("%s: Failed to complete disconnection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		printk("%s: Challenge reply received from connected device\n", __func__);
		ret = otgcontrol_do_verify_device_challenge_reply(otgc_data, param);
		if (ret < 0) {
			printk("%s: Failed to verify device challenge reply, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		printk("%s: Controller-mode change requested, start caller authentication\n", __func__);
		return otgcontrol_do_controller_mode_change_procedure(otgc_data, param);
		break;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		printk("%s: Unexpected mode change challenge reply received, application might be in an unknown state - ignoring\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		printk("%s: Charger-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		printk("%s: USB OTG DR-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__TIMEOUT:
		printk("%s: TIMEOUT, no response from connected device within expected amount of time\n", __func__);
		printk("%s: Reset fsm, shutting off power and waiting for disconnect/re-connect\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	default:
		printk("%s: Unknown signal/event (%d)\n", __func__, signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_ONEWIRE_AUTH_DEVICE_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		printk("%s: Host is connected, disabling all OTG features until host is disconnected\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;
		break;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		printk("%s: Host is disconnected (NOT EXPECTED), but just ignore this and continue being connected to authenticated device\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__DEVICE_CONNECTED:
		printk("%s: Device connected (UNEXPECTED), re-starting authenticated connection procedure\n", __func__);
		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			printk("%s: Failed to complete device connection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		printk("%s: Device disconnected, doing disconnection procedure\n", __func__);
		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			printk("%s: Failed to complete disconnection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		printk("%s: Unexpected message received from device, it might be in an unknown state, doing fsm reset procedure to power-cycle device\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		printk("%s: Controller-mode change requested\n", __func__);
		return otgcontrol_do_controller_mode_change_procedure(otgc_data, param);
		break;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		printk("%s: Unexpected mode change challenge reply received, application might be in an unknown state - ignoring\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		printk("%s: Charger-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		printk("%s: USB OTG DR-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__TIMEOUT:
		printk("%s: TIMEOUT, but not waiting for anyting other than device disconnection - ignoring\n", __func__);
		return 0;
		break;

	default:
		printk("%s: Unknown signal/event (%d)\n", __func__, signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		printk("%s: Host is connected, disabling all OTG features until host is disconnected\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;
		break;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		printk("%s: Host is disconnected (NOT EXPECTED), but just ignore this and continue to wait for challenge response from application\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__DEVICE_CONNECTED:
		printk("%s: Device connected (UNEXPECTED), re-starting authenticated connection procedure\n", __func__);
		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			printk("%s: Failed to complete device connection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		printk("%s: Device disconnected, doing disconnection procedure\n", __func__);
		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			printk("%s: Failed to complete disconnection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		printk("%s: Unexpected message received from device, it might be in an unknown state, doing fsm reset procedure to power-cycle device\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		printk("%s: Controller-mode change requested while already waiting for challenge response, restart procedure\n", __func__);
		return otgcontrol_do_controller_mode_change_procedure(otgc_data, param);
		break;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		printk("%s: Received expected challenge response, changing to unauthenticated USB OTG mode\n", __func__);
		return otgcontrol_do_verify_application_challenge_reply(otgc_data, param);
		break;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		printk("%s: Charger-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		printk("%s: USB OTG DR-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__TIMEOUT:
		printk("%s: TIMEOUT, no response from application requesting mode change within expected amount of time\n", __func__);
		printk("%s: Reset fsm, going back to authenticated mode, requiring new mode change request\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	default:
		printk("%s: Unknown signal/event (%d)\n", __func__, signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_USB_NO_AUTH_NOT_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		printk("%s: Host is connected, disabling all OTG features until host is disconnected\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;
		break;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		printk("%s: Host is disconnected (NOT EXPECTED), but just ignore this and continue to wait for device connection\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__DEVICE_CONNECTED:
		printk("%s: Device connected, doing un-authenticated connection procedure\n", __func__);
		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			printk("%s: Failed to complete device connection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		printk("%s: Device disconnected, doing disconnection procedure\n", __func__);
		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			printk("%s: Failed to complete disconnection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		printk("%s: Unexpected message received from device, it might be in an unknown state, doing fsm reset procedure to power-cycle device\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		printk("%s: Controller-mode change requested, start caller authentication\n", __func__);
		return otgcontrol_do_controller_mode_change_procedure(otgc_data, param);
		break;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		printk("%s: Unexpected mode change challenge reply received, application might be in an unknown state - ignoring\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		printk("%s: Charger-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		printk("%s: USB OTG DR-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__TIMEOUT:
		printk("%s: TIMEOUT, but not waiting for anyting other than device connection - ignoring\n", __func__);
		return 0;
		break;

	default:
		printk("%s: Unknown signal/event (%d)\n", __func__, signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_USB_NO_AUTH_DEVICE_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	int ret;

	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		printk("%s: Host is connected, disabling all OTG features until host is disconnected\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;
		break;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		printk("%s: Host is disconnected (NOT EXPECTED), but just ignore this and continue being connected to authenticated device\n", __func__);
		return 0;
		break;

	case OTG1_EVENT__DEVICE_CONNECTED:
		printk("%s: Device connected (UNEXPECTED), re-starting un-authenticated connection procedure\n", __func__);
		ret = otgcontrol_do_device_connected_procedure(otgc_data, false);
		if (ret < 0) {
			printk("%s: Failed to complete device connection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		printk("%s: Device disconnected, doing disconnection procedure\n", __func__);
		ret = otgcontrol_do_device_disconnected_procedure(otgc_data);
		if (ret < 0) {
			printk("%s: Failed to complete disconnection procedure, resetting fsm as an attempt to recover\n", __func__);
			return otgcontrol_reset_fsm(otgc_data);
		}
		return 0;
		break;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		printk("%s: Unexpected message received from device, it might be in an unknown state, doing fsm reset procedure to power-cycle device\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		printk("%s: Controller-mode change requested\n", __func__);
		return otgcontrol_do_controller_mode_change_procedure(otgc_data, param);
		break;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		printk("%s: Unexpected mode change challenge reply received, application might be in an unknown state - ignoring\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		printk("%s: Charger-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		printk("%s: USB OTG DR-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__TIMEOUT:
		printk("%s: TIMEOUT, but not waiting for anyting other than device disconnection - ignoring\n", __func__);
		return 0;
		break;

	default:
		printk("%s: Unknown signal/event (%d)\n", __func__, signal);
		return -EINVAL;
	}
}

static int otgcontrol_handle_state_HOST_CONNECTED(struct rm_otgcontrol_data *otgc_data, int signal, void *param)
{
	switch(signal)
	{
	case OTG1_EVENT__CHARGER_CONNECTED:
		printk("%s: Host connection detected in already connected state, keeping all OTG features disabled until host is disconnected\n", __func__);
		otgc_data->otg_controlstate = OTG1_STATE__HOST_CONNECTED;
		return 0;
		break;

	case OTG1_EVENT__CHARGER_DISCONNECTED:
		printk("%s: Host is disconnected, reset fsm into authenticated mode\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__DEVICE_CONNECTED:
		printk("%s: Device connected (UNEXPECTED), reset fsm into authenticated mode\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);

	case OTG1_EVENT__DEVICE_DISCONNECTED:
		printk("%s: Device disconnected (UNEXPECTED), reset fsm into authenticated mode\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__CHALLENGE_REPLY_RECEIVED:
		printk("%s: Unexpected message received from device, reset fsm into authenticated mode\n", __func__);
		return otgcontrol_reset_fsm(otgc_data);
		break;

	case OTG1_EVENT__MODE_CHANGE_REQUESTED:
		printk("%s: Controller-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY:
		printk("%s: Unexpected mode change challenge reply received, application might be in an unknown state - ignoring\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED:
		printk("%s: Charger-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED:
		printk("%s: USB OTG DR-mode change requested, not valid in this state\n", __func__);
		return -EINVAL;
		break;

	case OTG1_EVENT__TIMEOUT:
		printk("%s: TIMEOUT, but not waiting for anyting other than host disconnection - ignoring\n", __func__);
		return 0;
		break;

	default:
		printk("%s: Unknown signal/event (%d)\n", __func__, signal);
		return -EINVAL;
	}
}

static int otgcontrol_do_controller_mode_change_procedure(struct rm_otgcontrol_data *otgc_data, void *param)
{
	int mode_requested = *(int*)param;

	/* Depending on requested mode.. */
	/* Enable challenge/reply properties, and set challenge to be read by application */
	/* Wait for reply for a while */
	/* But for now, just set mode */
	switch(mode_requested)
	{
	case OTG_MODE__MANUAL_CONTROL:
	case OTG_MODE__USB_NO_AUTH:
		return otgcontrol_do_authenticate_calling_application(otgc_data, mode_requested);
		break;
	default:
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		return 0;
	}
}

static int otgcontrol_do_authenticate_calling_application(struct rm_otgcontrol_data *otgc_data, int mode_requested)
{
	printk("%s: SKIPPING CALLING APPLICATION AUTHENTICATION FOR NOW ..\n", __func__);

	switch(mode_requested)
	{
	case OTG_MODE__MANUAL_CONTROL:
		otgc_data->otg_controlstate = OTG1_STATE__MANUAL_CONTROL;
		break;
	case OTG_MODE__ONEWIRE_AUTH:
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		break;
	case OTG_MODE__USB_NO_AUTH:
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		break;
	default:
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
	}
	return 0;
}

static int otgcontrol_do_verify_application_challenge_reply(struct rm_otgcontrol_data *otgc_data, void *param)
{
	printk("%s: For now, just take the challenge to be good and activate requested state\n", __func__);

	switch(otgc_data->mode_requested)
	{
	case OTG_MODE__MANUAL_CONTROL:
		otgc_data->otg_controlstate = OTG1_STATE__MANUAL_CONTROL;
		break;
	case OTG_MODE__ONEWIRE_AUTH:
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		break;
	case OTG_MODE__USB_NO_AUTH:
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
		break;
	default:
		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED;
	}
	return 0;
}

static int otgcontrol_do_verify_device_challenge_reply(struct rm_otgcontrol_data *otgc_data, void *param)
{
	int ret;
	printk("%s: For now, just take the challenge to be good and activate USB OTG Host mode", __func__);
	ret = otgcontrol_set_dr_mode(otgc_data, OTG1_DR_MODE__HOST);
	if (ret < 0) {
		printk("%s: Failed to set USB OTG host mode\n", __func__);
		return ret;
	}

	otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED;
	return 0;
}

static int otgcontrol_do_set_controlmode(struct rm_otgcontrol_data *otgc_data, int mode)
{
	printk("%s: Enter\n", __func__);

	switch(mode)
	{
	case OTG_MODE__MANUAL_CONTROL:
		printk("%s: setting MANUAL_CONTROL mode\n", __func__);
		break;

	case OTG_MODE__ONEWIRE_AUTH:
		printk("%s: setting ONEWIRE_AUTH mode\n", __func__);
		break;

	case OTG_MODE__USB_NO_AUTH:
		printk("%s: setting USB_NO_AUTH mode\n", __func__);
		break;

	default:
		printk("%s: unable to set control mode (unknown mode %d)\n", __func__, mode);
		return -EINVAL;
	}

	return 0;
}

static int otgcontrol_do_device_connected_procedure(struct rm_otgcontrol_data *otgc_data, bool authentication_required)
{
	int i, ret;

	printk("%s: Powering up connected device (if no charger is connected)\n", __func__);
	ret = otgcontrol_change_otg_charge_mode(otgc_data, OTG1_CHARGERMODE_OTG); /* OTG POWER ON */
	if (ret < 0) {
		printk("%s: Unable to turn on OTG power, check connections\n", __func__);
		return ret;
	}

	if (authentication_required) {
		printk("%s: Deactivating GPIO IRQ and sending authentication challenge to connected device\n", __func__);
		otgcontrol_deactivate_gpio_irq(otgc_data);

		/* Wait for device to boot */
		printk("%s: Waiting for device to boot\n", __func__);
		for(i = 0;i < 5000;i++) udelay(1000);

		/* Send challenge response */
		printk("%s: Changing one-wire mux config (UART TX)\n", __func__);
		otgcontrol_switch_one_wire_mux_state(otgc_data, OTG1_ONEWIRE_STATE__UART_TX);

		printk("%s: Sending authentication challenge\n", __func__);
		char tty_device_name[50], buf[100];
		sprintf(tty_device_name, "/dev/%s", otgc_data->one_wire_tty_name);
		ret = otgcontrol_onewire_write_tty(otgc_data, "/dev/ttymxc5", ":0001ff#");
		if (ret < 0)
		{
			printk("%s: Failed to send authentication challenge to connected device\n", __func__);
			return ret;
		}

		/* Wait for message to be sendt before switching direction */
		printk("%s: Waiting 100 ms approx. to let tx message leave before switching direction\n", __func__);
		for(i = 0;i < 100;i++) udelay(1000);

		printk("%s: Changing one-wire mux config (UART RX)\n", __func__);
		ret = otgcontrol_switch_one_wire_mux_state(otgc_data, OTG1_ONEWIRE_STATE__UART_RX);
		if (ret < 0) {
			printk("%s: Unable to switch one-wire mux config, cannot authenticate connected device\n", __func__);
			return ret;
		}

		/* Wait for response or timeout */
		printk("%s: Waiting for response from connected device\n", __func__);
		/* For now, just block here until response has been received */
		int count = otgcontrol_onewire_read_until_cr(otgc_data, "/dev/ttymxc5", buf, 100);
		buf[count] = 0;
		printk("%s: Read '%s'", __func__, buf);

		/* Verify response */
		/* For now just take it to be good and enable USB connection (i.e. enable host mode) */
		printk("%s: Taking response to be good and activating USB connection (i.e enabling host mode)\n", __func__);
		ret = otgcontrol_do_set_controlmode(otgc_data, OTG1_DR_MODE__HOST);
		if (ret < 0) {
			printk("%s: Unable to set USB OTG host mode - disconnect and re-connect connected device\n", __func__);
			return ret;
		}

		otgc_data->otg_controlstate = OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED;
		return 0;
	}
	else
	{
		printk("%s: Authentication not required, activating USB connection (i.e. enabling host mode)\n", __func__);
		otgcontrol_set_dr_mode(otgc_data, OTG1_DR_MODE__HOST);
		otgc_data->otg_controlstate = OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED;
		return 0;
	}
}

static int otgcontrol_do_device_disconnected_procedure(struct rm_otgcontrol_data *otgc_data)
{
	int ret;

	ret = otgcontrol_change_otg_charge_mode(otgc_data, OTG1_CHARGERMODE_CHARGE); /* OTG POWER OFF */
	if (ret < 0) {
		printk("%s: Unable to turn off OTG power\n", __func__);
		return ret;
	}

	ret = otgcontrol_set_dr_mode(otgc_data, OTG1_DR_MODE__DEVICE);
	if (ret < 0) {
		printk("%s: Unable to set USB OTG device mode\n", __func__);
		return ret;
	}

	printk("%s: Activating one-wire GPIO IRQ\n", __func__);
	otgcontrol_activate_gpio_irq(otgc_data);
	return 0;
}

static int otgcontrol_do_start_onewire_authentication(struct rm_otgcontrol_data *otgc_data)
{
	printk("%s: Enter\n", __func__);

	printk("%s: PLEASE IMPLEMENT THIS !\n", __func__);
	return 0;
}
