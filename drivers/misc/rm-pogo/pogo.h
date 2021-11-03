/*
 * reMarkable OTG Control
 *
 * Copyright (C) 2019-2022 reMarkable AS - http://www.remarkable.com/
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

#ifndef __RM_POGO_H_
#define __RM_POGO_H_

#include <linux/kobject.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/mutex.h>
#include <linux/extcon.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/serdev.h>
#include <linux/workqueue.h>


#define ONE_WIRE_MAX_TX_MSG_SIZE	2048
#define ONE_WIRE_SHORT_TX_MSG_SHORT	16
#define ONE_WIRE_MCU_MSG_SIZE		280
#define MAX_POGO_DEV_NAME		16

#define ONE_WIRE_UART_ACK		1
#define ONE_WIRE_UART_LISTEN		2
#define POGO_TX_RETRY_LIMIT		3

#define RAND_SECRET_LENGTH		8

/* packet protocol */
#define POGO_CLASS_CHARGER BIT(0)
#define POGO_CLASS_KEYBOARD BIT(1)
#define POGO_CLASS_UART_IF BIT(31)
#define POGO_CLASS_OTG_IF BIT(30)

typedef enum _packet_pogo_command
{
    POGO_CMD_NONE                    = 0x00,
    POGO_CMD_FW_WRITE_VALIDATE_IMAGE = 0x02,
    POGO_CMD_ENTER_APP               = 0x04,
    POGO_CMD_ENTER_SUSPEND           = 0x05,
    POGO_CMD_FW_WRITE_VALIDATE_CRC   = 0x06,
    POGO_CMD_FW_WRITE_PACKET         = 0x07,
    POGO_CMD_FW_WRITE_INIT           = 0x08,
    POGO_CMD_GET_AUTH_KEY            = 0x09,
    POGO_CMD_REBOOT                  = 0x0F,
    POGO_CMD_ATTRIBUTE_READ          = 0x20,
    POGO_CMD_ATTRIBUTE_WRITE         = 0x21,
} packet_pogo_command_t;


typedef enum _packet_kb_report
{
    KB_REPORT_ALIVE   = 0x40,
    KB_REPORT_KEY     = 0x51
} packet_kb_report_t;


/* data length */
#define KB_CMD_ENTER_APP_LEN		1
#define KB_CMD_FW_WRITE_VALIDATE_CRC_LEN	1
#define KB_CMD_FW_WRITE_VALIDATE_IMAGE_LEN 1
#define KB_CMD_FW_WRITE_PACKET_LEN	1
#define KB_CMD_FW_WRITE_INIT_LEN	1
#define KB_CMD_GET_AUTH_KEY_LEN	33

/* The string is 32 char, 0-terminated. Hence KB_CMD_GET_AUTH_KEY_LEN == 33*/
#define AUTH_KEY_STRING ("@O8eO77%o^4*1GE@oeodd#WMa%8Kr6v@")
#define ALIVE_INTERV_MS			1000

/* error codes returned as data in packet */
typedef enum _pogo_cmd_write_response
{
    POGO_SUCCESS                = 0,
    POGO_ERROR                  = 1,
    POGO_FWU_INIT_ERROR         = 2,
    POGO_CRC_CHECK_ERROR        = 3,
    POGO_FLASH_WRITE_ERROR      = 4,
    POGO_FLASH_DELETE_ERROR     = 5,
    POGO_TOO_LARGE_PACKET_ERROR = 6,
    POGO_FWU_IMG_WRONG_SLOT     = 7,
    POGO_FWU_UNALIGNED_PACKET   = 8,
} pogo_cmd_write_response_t;

/* application packet protocol */
enum pogo_onewire_state {
	POGO_ONEWIRE_STATE__GPIO,
	POGO_ONEWIRE_STATE__UART_TX,
	POGO_ONEWIRE_STATE__UART_RX,
	POGO_ONEWIRE_STATE__GPIO_5K_5K,
	POGO_ONEWIRE_STATE__GPIO_5K_47K,
	POGO_ONEWIRE_STATE__GPIO_5K_100K,
	POGO_ONEWIRE_STATE__GPIO_47K_47K,
	POGO_ONEWIRE_STATE__GPIO_47K_100K,
	POGO_ONEWIRE_STATE__GPIO_100K_100K,
	POGO_ONEWIRE_STATE__GPIO_100K_PD_100K_PD,
	POGO_ONEWIRE_STATE_NR,
};

#define POGO_ONEWIRE_GPIO_STATE__POGO_CONNECTED	0
#define POGO_ONEWIRE_GPIO_STATE__POGO_NOT_CONNECTED	1

typedef enum _package_kb_language
{
    DE = 1,
    ES = 2,
    FR = 3,
    IT = 4,
    NO = 5,
    PT = 6,
    UK = 7,
    US = 8
} package_kb_language_t;

typedef struct __attribute__((__packed__)) _fw_version
{
    uint8_t minor;
    uint8_t major;
} fw_version_t;

typedef struct __attribute__((__packed__)) _packet_kb_info
{
    fw_version_t fw_version;
    package_kb_language_t language : 8;
    uint32_t device_class;
    uint32_t device_id[4];
    uint32_t image_start_addr;
    uint8_t keylayout;
    char serial[15]; /* Serial number without \0 byte */
} packet_kb_info_t;

typedef struct __attribute__((__packed__)) _fwu_image_header
{
    uint32_t image_start_address;
    uint32_t image_length;
    fw_version_t fw_version; // LSB Minor version, MSB Major version
} fwu_image_header_t;

typedef enum _fw_write_status {
    FW_WRITE_STATUS_NOT_STARTED	= 0,
    FW_WRITE_STATUS_UP_TO_DATE	= 1,
    FW_WRITE_STATUS_STARTED	= 2,
    FW_WRITE_STATUS_FAILED	= 3,
    FW_WRITE_STATUS_SUCCEEDED	= 4,
}fw_write_status_t;

struct rm_pogo_data {
	struct serdev_device			*serdev;
	struct device           		*dev;
	struct input_dev           		*kb_dev;	/* keyboard dev */
	struct kfifo				read_fifo;

	struct task_struct			*fsm_thread;
	struct task_struct			*uart_rx_thread;

	struct mutex				lock;
	spinlock_t				spin_lock;
	struct notifier_block			vbus_short_nb;
	bool					vbus_short;

	struct extcon_dev			*extcon_dev;
	u8		           		*rand_secret;

	struct work_struct			one_wire_gpio_irq_work_queue;
	int					one_wire_gpio_state;
	void					*onewire_tx_buf;
	u8					*onewire_rx_buf;
	u16					onewire_rx_buf_len;
	u16					tx_data_len;
	u16					tx_retried_count;
	u16					auth_retry;

	/* Reference to charger driver for OTG power control */
	struct power_supply			*vbus_supply;

	/* One-wire gpio config */
	struct gpio_desc			*one_wire_gpio;
	int					one_wire_gpio_irq;
	bool					irq_detected;
	bool					ignore_next_irq;
	bool					suspend;

	int					pogo_fsm_state;
	int					fsm_err;
	int					nego_fsm_state;
	int					fw_fsm_state;

	int					mode_requested;
	int					uart_rx_mode;
	bool					serdev_ready;
	bool					tx_ack_timeout;
	bool					tx_ack_required;
	bool					mcu_authenticated;
	struct delayed_work			uart_tx_ack_watchdog_work_queue;

	char					*pogo_name;

	packet_kb_info_t dev_info;
	fwu_image_header_t fwu_image_header;

	u32					dev_status;
	bool					fw_write;
	bool					fw_write_enabled;
	const struct firmware			*fw;
	u32					fw_off;
	fw_write_status_t 			fw_write_status;

	/* app session */
	int					app_session;
	struct delayed_work			mcu_detach_work;
	bool 					mcu_detach_work_active;
	struct timer_list			alive_timer;
	unsigned int				kb_row_shift;

	u8				user_command;
	u8				*user_command_data;
	int				user_command_data_len;
	u8				user_command_response[ONE_WIRE_MCU_MSG_SIZE];

	struct pinctrl*				one_wire_pinctrl;
	struct pinctrl_state*			one_wire_pinctrl_states[POGO_ONEWIRE_STATE_NR];

	struct kobject*				kobject;

	struct kobj_attribute			pogo_connected_attribute;
	bool					pogo_connected;

	struct kobj_attribute			pogo_dr_mode_attribute;
	int					pogo_dr_mode;

	struct kobj_attribute			rand_secret_attribute;
	struct kobj_attribute			lang_attribute;
	struct kobj_attribute			pogo_mcu_auth_attribute;
	int					pogo_mcu_auth;

	struct kobj_attribute			pogo_chargermode_attribute;
	int					pogo_chargermode;

	struct kobj_attribute			onewire_pinctrlstate_attribute;
	int					onewire_pinctrlstate;

	struct kobj_attribute			pogo_gpio_pinctrl_index_attribute;
	int					pogo_gpio_pinctrl_index;
	struct kobj_attribute			ack_timeout_attribute;
	int					ack_timeout;
	struct kobj_attribute			manual_fsm_attribute;
	int					manual_fsm;
	struct kobj_attribute			alive_timeout_attribute;
	int					alive_timeout;

	struct kobj_attribute			fsm_state_attribute;
	struct kobj_attribute			keylayout_ver_attribute;
	struct kobj_attribute			fw_ver_attribute;
	struct kobj_attribute			dev_id_attribute;
	struct kobj_attribute			auto_key_attribute;
	struct kobj_attribute			fetch_mfg_log_attribute;
	struct kobj_attribute			mfg_log_attribute;
	struct kobj_attribute			onewire_rx_buf_attribute;
	struct kobj_attribute			fw_write_status_attribute;
	int					auto_key;	/* auto key input for test */
	int					mfg_log;	/* manufacture log */
	struct kobj_attribute			pogo_suspend_attribute;
	struct kobj_attribute			pogo_reboot_attribute;
	struct kobj_attribute			hard_reset_attribute;
	struct kobj_attribute			fw_update_enabled_attribute;
	struct kobj_attribute			user_command_attribute;
	struct kobj_attribute			user_command_response_attribute;
	struct kobj_attribute			serial_attribute;
	struct kobj_attribute			mcu_alive_interval_attribute;
	uint16_t				mcu_alive_interval;


	/* fake info for fsm debugging */
	struct kobj_attribute			emulation_enable_attribute;
	int					emulation_enable;

	struct kobj_attribute			fake_dev_connected_attribute;
	int					fake_dev_connected;

	struct kobj_attribute			fake_key_input_attribute;
	struct kobj_attribute			fake_key_press_attribute;
	struct kobj_attribute			fake_key_release_attribute;
	struct kobj_attribute			fake_cmd_attribute;
	struct kobj_attribute			fake_fsm_err_attribute;
};

extern const char *otg_onewire_pinctrl_name[];

bool pogo_check_fw_write(struct rm_pogo_data *pdata);
int pogo_load_fw(struct rm_pogo_data *pdata, char image);
int pogo_send_fw_packet(struct rm_pogo_data *pdata);
int pogo_send_fw_validate(struct rm_pogo_data *pdata);
int pogo_send_fw_validate_image(struct rm_pogo_data *pdata);
void pogo_release_firmware(struct rm_pogo_data *pdata);
void fsm_suspend(struct rm_pogo_data *pdata, bool standby);
void fsm_resume(struct rm_pogo_data *pdata, bool standby);
void pogo_exit_fsm(struct rm_pogo_data *pdata);
int pogo_serdev_open(struct rm_pogo_data *data);
void pogo_serdev_close(struct rm_pogo_data *data);
int pogo_register_uart_keyboard(struct rm_pogo_data *pdata);
void pogo_keyboard_report(struct rm_pogo_data *pdata, u8 val);

int pogo_init_one_wire_mux_state(struct rm_pogo_data *pdata);
int pogo_switch_one_wire_mux_state(struct rm_pogo_data *pdata,
					 int newState);
int pogo_get_current_gpio_state(struct rm_pogo_data *pdata);
const char *pogo_gpio_state_name(int state);
int pogo_init_gpio_irq(struct rm_pogo_data *pdata);
void pogo_uninit_gpio_irq(struct rm_pogo_data *pdata);
void pogo_activate_gpio_irq(struct rm_pogo_data *pdata);
int __must_check pogo_onewire_write(struct rm_pogo_data *data, u8 cmd, u8 ext,
			const unsigned char *msg, size_t count,
			bool ack_required);

static inline u32 packet_payload_len(u8 *msg)
{
	return msg[0] | ((msg[1] & 0xf) << 8);
}

static inline u32 packet_len(u8 *msg)
{
	/* 3 is for two length bytes and command*/
	return packet_payload_len(msg) + 3;
}

#endif /* __RM_POGO_H */
