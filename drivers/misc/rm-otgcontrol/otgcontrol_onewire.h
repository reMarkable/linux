#ifndef __OTGCONTROL_ONE_WIRE_H__
#define __OTGCONTROL_ONE_WIRE_H__

#include <linux/rm-otgcontrol.h>

#define OTG1_ONEWIRE_STATE__GPIO        0
#define OTG1_ONEWIRE_STATE__UART_TX     1
#define OTG1_ONEWIRE_STATE__UART_RX     2

int otgcontrol_init_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_uninit_onw_wire_mux_state(struct rm_otgcontrol_data *otgc_data);

int otgcontrol_switch_one_wire_mux_state(struct rm_otgcontrol_data *otgc_data, int newState);
int otgcontrol_get_current_gpio_state(struct rm_otgcontrol_data *otgc_data);
int otgcontrol_init_gpio_irq(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_activate_gpio_irq(struct rm_otgcontrol_data *otgc_data);
int otgcontrol_deactivate_gpio_irq(struct rm_otgcontrol_data *otgc_data);
int otgcontrol_gpio_irq(struct rm_otgcontrol_data *otgc_data);
void otgcontrol_gpio_irq_work(struct rm_otgcontrol_data *otgc_dataS);

#endif /* __OTGCONTROL_ONE_WIRE_H__ */
