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

#ifndef __POGO_FSM_H__
#define __POGO_FSM_H__

#include "pogo.h"

int pogo_init_fsm(struct rm_pogo_data *pogo_data);
int pogo_handleInput(struct rm_pogo_data *pogo_data,
			   int signal,
			   void *param);

#define OTG_MODE__MANUAL_CONTROL					0
#define OTG_MODE__ONEWIRE_AUTH						1
#define OTG_MODE__USB_NO_AUTH						2

#define POGO_STATE__INIT						0
#define POGO_STATE__ONEWIRE_IDLE					1
#define POGO_STATE__ONEWIRE_ENUM					2
#define POGO_STATE__ONEWIRE_AUTH					3
#define POGO_STATE__ONEWIRE_NEGO					4
#define POGO_STATE__FW_WRITE						5
#define POGO_STATE__START_APP						6
#define POGO_STATE__UART_SESSION					7
#define POGO_STATE__USB_SESSION						8

/* Do not change cmd sequence in nego sub-fsm. */
#define NEGO_S__DEV_CLASS						0
#define NEGO_S__DEV_ID							1
#define NEGO_S__DEV_HW_VERSION						2
#define NEGO_S__DEV_FW_VERSION						3
#define NEGO_S__DEV_CONFIGURATION					4
#define NEGO_S__DEV_APP_ENTRY						5

#define FW_S__INIT							0
#define FW_S__MCU_BIN							1
#define FW_S__VALIDATE							2
#define FW_S__CHECK_FW_VER						3
#define FW_S__VALIDATE_IMAGE					4

#define APP_SESSION_NONE						0
#define APP_SESSION_UART						1
#define APP_SESSION_USB							2
#define APP_SESSION_READY						0x8000

#endif /* __POGO_FSM_H__ */
