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

#ifndef __OTGCONTROL_FSM_H__
#define __OTGCONTROL_FSM_H__

#include "otgcontrol.h"

int otgcontrol_init_fsm(struct rm_otgcontrol_data *otgc_data);
int otgcontrol_handleInput(struct rm_otgcontrol_data *otgc_data,
			   int signal,
			   void *param);

#define OTG_MODE__MANUAL_CONTROL					0
#define OTG_MODE__ONEWIRE_AUTH						1
#define OTG_MODE__USB_NO_AUTH						2

#define OTG1_STATE__MANUAL_CONTROL					0
#define OTG1_STATE__ONEWIRE_AUTH_NOT_CONNECTED				1
#define OTG1_STATE__ONEWIRE_AUTH_WAIT_HANDSHAKE_RESPONS			2
#define OTG1_STATE__ONEWIRE_AUTH_DEVICE_CONNECTED			3
#define OTG1_STATE__USB_NO_AUTH_WAITING_CHALLENGE_RESPONSE		4
#define OTG1_STATE__USB_NO_AUTH_NOT_CONNECTED				5
#define OTG1_STATE__USB_NO_AUTH_DEVICE_CONNECTED			6
#define OTG1_STATE__HOST_CONNECTED					7

#define OTG1_EVENT__CHARGER_CONNECTED					0
#define OTG1_EVENT__CHARGER_DISCONNECTED				1
#define OTG1_EVENT__DEVICE_CONNECTED					2
#define OTG1_EVENT__DEVICE_DISCONNECTED					3
#define OTG1_EVENT__CHALLENGE_REPLY_RECEIVED				4
#define OTG1_EVENT__TIMEOUT						5
#define OTG1_EVENT__MODE_CHANGE_REQUESTED				6
#define OTG1_EVENT__MODE_CHANGE_CHALLENGE_REPLY				7
#define OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED			8
#define OTG1_EVENT__OTG_DR_MODE_CHANGE_REQUESTED			9

#endif /* __OTGCONTROL_FSM_H__ */
