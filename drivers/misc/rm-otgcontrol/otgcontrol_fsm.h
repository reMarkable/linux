#ifndef __OTGCONTROL_FSM_H__
#define __OTGCONTROL_FSM_H__

#include <linux/rm-otgcontrol.h>

int otgcontrol_init_fsm(struct rm_otgcontrol_data *otgc_data);
int otgcontrol_set_controlmode(struct rm_otgcontrol_data *otgc_data, int mode);
int otgcontrol_handleInput(struct rm_otgcontrol_data *otgc_data, int signal, void *param);

static int otgcontrol_start_onewire_authentication(struct rm_otgcontrol_data *otgc_data);

#define OTG_MODE__MANUAL_CONTROL                        0
#define OTG_MODE__ONEWIRE_AUTH                          1
#define OTG_MODE__USB_NO_AUTH                           2

#define OTG1_STATE__MANUAL_CONTROL                      0
#define OTG1_STATE__ONEWIRE_NOT_CONNECTED               1
#define OTG1_STATE__ONEWIRE_WAIT_HANDSHAKE_RESPONS      2
#define OTG1_STATE__ONEWIRE_DEVICE_CONNECTED            3
#define OTG1_STATE__USB_NOT_CONNECTED                   4
#define OTG1_STATE__USB_DEVICE_CONNECTED                5
#define OTG1_STATE__HOST_CONNECTED                      6

#define OTG1_EVENT__VBUS_CHANGED                        0
#define OTG1_EVENT__ONEWIRE_GPIO_STATE_CHANGED          1
#define OTG1_EVENT__CHALLENGE_REPLY_RECEIVED            2
#define OTG1_EVENT__TIMEOUT                             3
#define OTG1_EVENT__MODE_CHANGE_REQUESTED               4
#define OTG1_EVENT__OTG_CHARGERMODE_CHANGE_REQUESTED    5
#define OTG1_STATE__OTG_ID_STATE_CHANGE_REQUESTED       6

#endif /* __OTGCONTROL_FSM_H__ */
