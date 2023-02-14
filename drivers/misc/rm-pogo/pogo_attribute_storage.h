/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __POGO_ATTRIBUTE_STORAGE_H__
#define __POGO_ATTRIBUTE_STORAGE_H__

#include "pogo.h"

#include <linux/types.h>

/* Macro helpers */
#define CONCAT(a, b) a##b
#define M_CONCAT(a, b) CONCAT(a, b)
#define M__CONCAT(a, b) M_CONCAT(a, b)
#define STRINGIFY2(X) #X
#define STRINGIFY(X) STRINGIFY2(X)

#define INIT_USER_COMMAND(COMMAND, DATA_SIZE)                                  \
	do {                                                                   \
		memset(pdata->user_command_response, 0x00,                     \
		       sizeof(pdata->user_command_response));                  \
		if (pdata->user_command_data) {                                \
			devm_kfree(pdata->dev, pdata->user_command_data);      \
			pdata->user_command_data = NULL;                       \
		}                                                              \
		if (DATA_SIZE > 0)                                             \
			pdata->user_command_data = (u8 *)devm_kmalloc(         \
				pdata->dev, DATA_SIZE, GFP_KERNEL);            \
		pdata->user_command = COMMAND;                                 \
		pdata->user_command_sent = false;                              \
		pdata->user_command_data_len = DATA_SIZE;                      \
	} while (0)

typedef uint8_t attribute_storage_id_t;
typedef uint8_t attribute_storage_status_t;
typedef uint8_t attribute_storage_data_type_t;

/**
 * @brief Return status for write functionality
 *
 * Inspiration of these values are from ZCL
 */
typedef enum attribute_storage_status {
	ATTRIBUTE_STORAGE_SUCCESS = 0x00, /*!< Operation was successful */
	ATTRIBUTE_STORAGE_FAILURE = 0x01, /*!< Operation was not successful */
	ATTRIBUTE_STORAGE_UNSUPPORTED_ATTRIBUTE =
		0x86, /*!< Attribute not found */
	ATTRIBUTE_STORAGE_INVALID_VALUE =
		0x87, /*!< Out of range error, or set to a reserved value */
	ATTRIBUTE_STORAGE_READ_ONLY =
		0x88, /*!< Not allowed to write to this attribute */
	ATTRIBUTE_STORAGE_INSUFFICIENT_SPACE =
		0x89, /*!< An operation failed due to an insufficient amount of free space available */
	ATTRIBUTE_STORAGE_INVALID_DATA_TYPE =
		0x8D, /*!< Attribute have another type than given */
	ATTRIBUTE_STORAGE_WRITE_ONLY =
		0x8F, /*!< Not allowed to read this attribute */
	ATTRIBUTE_STORAGE_STATUS_UNKNOWN = 0x90
} attribute_storage_statuses_t;

static const char __maybe_unused *ATTR_STATUS_NAMES[] = {
	[ATTRIBUTE_STORAGE_SUCCESS] = "ATTRIBUTE_STORAGE_SUCCESS",
	[ATTRIBUTE_STORAGE_FAILURE] = "ATTRIBUTE_STORAGE_FAILURE",
	[ATTRIBUTE_STORAGE_UNSUPPORTED_ATTRIBUTE] =
		"ATTRIBUTE_STORAGE_UNSUPPORTED_ATTRIBUTE",
	[ATTRIBUTE_STORAGE_INVALID_VALUE] = "ATTRIBUTE_STORAGE_INVALID_VALUE",
	[ATTRIBUTE_STORAGE_READ_ONLY] = "ATTRIBUTE_STORAGE_READ_ONLY",
	[ATTRIBUTE_STORAGE_INSUFFICIENT_SPACE] =
		"ATTRIBUTE_STORAGE_INSUFFICIENT_SPACE",
	[ATTRIBUTE_STORAGE_INVALID_DATA_TYPE] =
		"ATTRIBUTE_STORAGE_INVALID_DATA_TYPE",
	[ATTRIBUTE_STORAGE_WRITE_ONLY] = "ATTRIBUTE_STORAGE_WRITE_ONLY",
	[ATTRIBUTE_STORAGE_STATUS_UNKNOWN] = "UNKNOWN_STATUS",
};

/**
 * @brief Data types for attributes
 *
 * Inspiration of these values are from ZCL
 */
typedef enum _attribute_storage_data_type {
	ATTRIBUTE_STORAGE_DATA_8_BIT = 0x08,
	ATTRIBUTE_STORAGE_DATA_16_BIT = 0x09,
	ATTRIBUTE_STORAGE_DATA_32_BIT = 0x0B,
	ATTRIBUTE_STORAGE_DATA_BOOLEAN = 0x10,
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT = 0x18,
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT = 0x19,
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT = 0x1B,
	ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT = 0x30,
	ATTRIBUTE_STORAGE_DATA_OCTET_STRING = 0x41,
	ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING = 0x42,
	ATTRIBUTE_STORAGE_DATA_ARRAY = 0x48,
	ATTRIBUTE_STORAGE_DATA_UNKNOWN,
} attribute_storage_data_types_t;

static const char __maybe_unused *ATTR_DATA_TYPE_NAMES[] = {
	[ATTRIBUTE_STORAGE_DATA_8_BIT] = "ATTRIBUTE_STORAGE_DATA_8_BIT",
	[ATTRIBUTE_STORAGE_DATA_16_BIT] = "ATTRIBUTE_STORAGE_DATA_16_BIT",
	[ATTRIBUTE_STORAGE_DATA_32_BIT] = "ATTRIBUTE_STORAGE_DATA_32_BIT",
	[ATTRIBUTE_STORAGE_DATA_BOOLEAN] = "ATTRIBUTE_STORAGE_DATA_BOOLEAN",
	[ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT] =
		"ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT",
	[ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT] =
		"ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT",
	[ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT] =
		"ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT",
	[ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT] =
		"ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT",
	[ATTRIBUTE_STORAGE_DATA_OCTET_STRING] =
		"ATTRIBUTE_STORAGE_DATA_OCTET_STRING",
	[ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING] =
		"ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING",
	[ATTRIBUTE_STORAGE_DATA_ARRAY] = "ATTRIBUTE_STORAGE_DATA_ARRAY",
	[ATTRIBUTE_STORAGE_DATA_UNKNOWN] = "UNKNOWN_DATA_TYPE",
};

#define GET_ATTRIBUTE_STORAGE_DATA_8_BIT int8_t
#define GET_ATTRIBUTE_STORAGE_DATA_16_BIT int16_t
#define GET_ATTRIBUTE_STORAGE_DATA_32_BIT int32_t
#define GET_ATTRIBUTE_STORAGE_DATA_BOOLEAN bool
#define GET_ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT uint8_t
#define GET_ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT uint16_t
#define GET_ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT uint32_t
#define GET_ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT uint8_t
/* These are all just bytes to me for now. */
#define GET_ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING unsigned char *
#define GET_ATTRIBUTE_STORAGE_DATA_OCTET_STRING unsigned char *
#define GET_ATTRIBUTE_STORAGE_DATA_ARRAY uint8_t *

#define INNER_DATA_STRUCT(TYPE)                                                \
	struct __attribute__((__packed__)) {                                   \
		TYPE value;                                                    \
	} CONCAT(TYPE, _data)

#define DEFINE_ANON_DATA_UNION                                                 \
	union {                                                                \
		INNER_DATA_STRUCT(GET_ATTRIBUTE_STORAGE_DATA_8_BIT);           \
		INNER_DATA_STRUCT(GET_ATTRIBUTE_STORAGE_DATA_16_BIT);          \
		INNER_DATA_STRUCT(GET_ATTRIBUTE_STORAGE_DATA_32_BIT);          \
		INNER_DATA_STRUCT(GET_ATTRIBUTE_STORAGE_DATA_BOOLEAN);         \
		INNER_DATA_STRUCT(GET_ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT);  \
		INNER_DATA_STRUCT(GET_ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT); \
		INNER_DATA_STRUCT(GET_ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT); \
		struct __attribute__((__packed__)) {                           \
			uint8_t str_len;                                       \
			char str[];                                            \
		};                                                             \
		struct __attribute__((__packed__)) {                           \
			attribute_storage_data_type_t subtype;                 \
			uint16_t n_len;                                        \
			uint8_t array_data[];                                  \
		};                                                             \
		uint8_t payload;                                               \
	}

/* Separate read/write structs to be able to easily use offsetof for stuff
 * after status, as status would otherwise have had to be a union member. For
 * write structs, a simple offsetof(attribute_u, str) would've then been
 * incorrect.
 */
typedef struct __attribute__((__packed__)) {
	attribute_storage_id_t id;
	attribute_storage_status_t status;
	attribute_storage_data_type_t type;
	DEFINE_ANON_DATA_UNION;
} attribute_rx_u;

typedef struct __attribute__((__packed__)) {
	attribute_storage_id_t id;
	attribute_storage_data_type_t type;
	DEFINE_ANON_DATA_UNION;
} attribute_tx_u;

typedef struct __attribute__((__packed__)) {
	attribute_storage_status_t status;
	attribute_storage_id_t id;
} attribute_tx_response_t;

#define ATTR_DATA_OFFSET(ATTR_STRUCT_TYPE)                                     \
	(offsetof(ATTR_STRUCT_TYPE, type) +                                    \
	 sizeof(attribute_storage_data_type_t))

#define ATTR_VAL_BY_ID(ATTR_ID)                                                \
	M__CONCAT(ATTR_ID_TO_DATA_TYPE(ATTR_ID), _data).value

#define ATTR_VAL_BY_TYPE(TYPE) M__CONCAT(GET_DATA_TYPE(TYPE), _data).value

#define DATA_TYPE_FOR_ATTRIBUTE_ID_PROTOCOL_VERSION                            \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_LANGUAGE ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_RM_SERIAL_NUMBER                            \
	ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING
#define DATA_TYPE_FOR_ATTRIBUTE_ID_KEY_LAYOUT                                  \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_FW_VERSION                                  \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_HW_VERSION                                  \
	ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING
#define DATA_TYPE_FOR_ATTRIBUTE_ID_DEVICE_CLASS ATTRIBUTE_STORAGE_DATA_32_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_DEVICE_ID ATTRIBUTE_STORAGE_DATA_ARRAY
#define DATA_TYPE_FOR_ATTRIBUTE_ID_IMAGE_START_ADDRESS                         \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_DEVICE_NAME                                 \
	ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING
#define DATA_TYPE_FOR_ATTRIBUTE_ID_MFG_LOG ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_AUTO_KEYBOARD_TEST_MODE                     \
	ATTRIBUTE_STORAGE_DATA_BOOLEAN
#define DATA_TYPE_FOR_ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS                    \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_POGO_PIN_RX_TX_SWITCH_DELAY_US              \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_MATRIX_SCAN_DELAY_US                        \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_LPD_INITIAL_WAIT_US                         \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_LPD_REQUEST_VBUS_PULSE_US                   \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_SUSPEND_WAIT_FOR_VBUS_LOSS_US               \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_COMMUNICATION_TEST_KEYS_DELAY_MS            \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT
#define DATA_TYPE_FOR_ATTRIBUTE_ID_COMMUNICATION_TEST_PRESS_RELEASE_DELAY_MS   \
	ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT

#define GET_DATA_TYPE(_type) CONCAT(GET_, _type)

/* This macro inserts the C data type for an attribute id, where _type is the enum of that id.  */
#define ATTR_ID_TO_DATA_TYPE(_type) GET_DATA_TYPE(CONCAT(DATA_TYPE_FOR_, _type))
/* This macro inserts the enum describing the data type for an attribute id, where _type is the enum of that id.  */
#define ATTR_ID_TO_DATA_TYPE_ENUM(_type) CONCAT(DATA_TYPE_FOR_, _type)

/* Macro assumes pdata variable to be available. */
#define INIT_ATTR_TX_U(ATTR_ID, _struct)                                       \
	_struct = (attribute_tx_u *)pdata->user_command_data;                  \
	_struct->id = ATTR_ID;                                                 \
	_struct->type = ATTR_ID_TO_DATA_TYPE_ENUM(ATTR_ID)

#define SIZEOF_ATTR_TX_U(ATTR_ID)                                              \
	(offsetof(attribute_tx_u, payload) +                                    \
	 get_type_size(ATTR_ID_TO_DATA_TYPE_ENUM(ATTR_ID)))

/*
 * @brief List of valid attributes. Must be kept in-sync with MCU.
 */
typedef enum _attributes_ids {
	/* Read only attributes */
	ATTRIBUTE_ID_PROTOCOL_VERSION = 0x01,
	ATTRIBUTE_ID_FW_VERSION,
	ATTRIBUTE_ID_HW_VERSION,
	ATTRIBUTE_ID_DEVICE_CLASS,
	ATTRIBUTE_ID_DEVICE_ID,
	ATTRIBUTE_ID_IMAGE_START_ADDRESS,
	ATTRIBUTE_ID_DEVICE_NAME,

	/* Read/Write attributes */
	ATTRIBUTE_ID_KEY_LAYOUT = 0x10,
	ATTRIBUTE_ID_LANGUAGE,
	ATTRIBUTE_ID_RM_SERIAL_NUMBER,

	/* Production and test attributes */
	ATTRIBUTE_ID_MFG_LOG = 0x20,
	ATTRIBUTE_ID_AUTO_KEYBOARD_TEST_MODE,
	ATTRIBUTE_ID_COMMUNICATION_TEST_KEYS_DELAY_MS,
	ATTRIBUTE_ID_COMMUNICATION_TEST_PRESS_RELEASE_DELAY_MS,

	/* Communication delays */
	ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS = 0x30,
	ATTRIBUTE_ID_POGO_PIN_RX_TX_SWITCH_DELAY_US,
	ATTRIBUTE_ID_MATRIX_SCAN_DELAY_US,
	ATTRIBUTE_ID_LPD_INITIAL_WAIT_US,
	ATTRIBUTE_ID_LPD_REQUEST_VBUS_PULSE_US,
	ATTRIBUTE_ID_SUSPEND_WAIT_FOR_VBUS_LOSS_US,
	ATTRIBUTE_ID_UNKNOWN,
} attributes_ids_t;

static const char __maybe_unused *ATTR_NAMES[] = {
	[ATTRIBUTE_ID_PROTOCOL_VERSION] = "ATTRIBUTE_ID_PROTOCOL_VERSION",
	[ATTRIBUTE_ID_FW_VERSION] = "ATTRIBUTE_ID_FW_VERSION",
	[ATTRIBUTE_ID_HW_VERSION] = "ATTRIBUTE_ID_HW_VERSION",
	[ATTRIBUTE_ID_DEVICE_CLASS] = "ATTRIBUTE_ID_DEVICE_CLASS",
	[ATTRIBUTE_ID_DEVICE_ID] = "ATTRIBUTE_ID_DEVICE_ID",
	[ATTRIBUTE_ID_IMAGE_START_ADDRESS] = "ATTRIBUTE_ID_IMAGE_START_ADDRESS",
	[ATTRIBUTE_ID_DEVICE_NAME] = "ATTRIBUTE_ID_DEVICE_NAME",

	/* Read/Write attributes */
	[ATTRIBUTE_ID_KEY_LAYOUT] = "ATTRIBUTE_ID_KEY_LAYOUT",
	[ATTRIBUTE_ID_LANGUAGE] = "ATTRIBUTE_ID_LANGUAGE",
	[ATTRIBUTE_ID_RM_SERIAL_NUMBER] = "ATTRIBUTE_ID_RM_SERIAL_NUMBER",

	/* Production and test attributes */
	[ATTRIBUTE_ID_MFG_LOG] = "ATTRIBUTE_ID_MFG_LOG",
	[ATTRIBUTE_ID_AUTO_KEYBOARD_TEST_MODE] =
		"ATTRIBUTE_ID_AUTO_KEYBOARD_TEST_MODE",
	[ATTRIBUTE_ID_COMMUNICATION_TEST_KEYS_DELAY_MS] =
		"ATTRIBUTE_ID_COMMUNICATION_TEST_KEYS_DELAY_MS",
	[ATTRIBUTE_ID_COMMUNICATION_TEST_PRESS_RELEASE_DELAY_MS] =
		"ATTRIBUTE_ID_COMMUNICATION_TEST_PRESS_RELEASE_DELAY_MS",

	/* Communication delays */
	[ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS] =
		"ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS",
	[ATTRIBUTE_ID_POGO_PIN_RX_TX_SWITCH_DELAY_US] =
		"ATTRIBUTE_ID_POGO_PIN_RX_TX_SWITCH_DELAY_US",
	[ATTRIBUTE_ID_MATRIX_SCAN_DELAY_US] =
		"ATTRIBUTE_ID_MATRIX_SCAN_DELAY_US",
	[ATTRIBUTE_ID_LPD_INITIAL_WAIT_US] = "ATTRIBUTE_ID_LPD_INITIAL_WAIT_US",
	[ATTRIBUTE_ID_LPD_REQUEST_VBUS_PULSE_US] =
		"ATTRIBUTE_ID_LPD_REQUEST_VBUS_PULSE_US",
	[ATTRIBUTE_ID_SUSPEND_WAIT_FOR_VBUS_LOSS_US] =
		"ATTRIBUTE_ID_SUSPEND_WAIT_FOR_VBUS_LOSS_US",
	[ATTRIBUTE_ID_UNKNOWN] = "UNKNOWN_ATTR_ID"
};

/*
 * Get a const char name of an attribute id.
 */
const char *const get_attr_name(const attribute_storage_id_t attr);

/*
 * Get a const char name of an attribute status.
 */
const char *const get_attr_status_name(const attribute_storage_status_t status);

/*
 * Sets the value field of an attribute_tx_u union when we only have the raw
 * type and can't use a macro. Reads type field of attribute_tx to determine
 * size of data at buf.
 *
 * Returns return-value from kstr* function used or -EINVAL on complete failure.
 */
int kstr_to_attr_value(attribute_tx_u *attribute_tx, const uint8_t *buf);

static inline bool __must_check
is_known_attr_id(const attribute_storage_id_t attr_id)
{
	return attr_id > 0 && attr_id < ATTRIBUTE_ID_UNKNOWN &&
	       ATTR_NAMES[attr_id] != NULL;
}

static inline bool __must_check
is_known_attr_type(const attribute_storage_data_type_t attr_type)
{
	return attr_type < ATTRIBUTE_STORAGE_DATA_UNKNOWN && attr_type > 0 &&
	       ATTR_DATA_TYPE_NAMES[attr_type] != NULL;
}

/*
 * Get byte size of attribute storage data type
 */
uint8_t get_type_size(const attribute_storage_data_type_t data_type);

/* TODO Make opaque handle? */
typedef struct parsed_received_attributes_handle {
	struct rm_pogo_data *pdata;
	const uint8_t *payload;
	size_t payload_size;
	int payload_offset;
} parsed_received_attributes_handle_t;

/*
 * Helper function to iterate over a read attribute command payload.
 *
 * Is meant to be used with a parsed_received_attributes_handle_t handle. Each
 * time this function is called with the same handle it sets attribute to the
 * next attribute read response.
 *
 * Meant to be called consecutively on the same handle, parsing one attribute
 * at a time. When there are no more attribute responses to parse, the pointer
 * pointed to by attribute is set to NULL.
 *
 * Returns true if another attribute response was found and and the attribute
 * pointer was updated. Returns false when all attributes have been parsed.
 */
bool parse_received_attributes(parsed_received_attributes_handle_t *handle,
			       const attribute_rx_u **attribute);

int pogo_onewire_write_attribute(struct rm_pogo_data *pdata,
				 const uint8_t *const msg, const size_t size);
/*
 * Send a message to the MCU to request a bunch of attributes.
 *
 * @attribute_ids list pointer to the attribute_ids
 * @n 		  number of attribute_ids to be found at attribute_ids
 * @return 	  return value of underlying pogo_onewire_write call
 */
int pogo_onewire_read_attribute(
	struct rm_pogo_data *data,
	const attribute_storage_id_t *const attribute_ids, const size_t n);

int update_pdata_from_attribute_read_response(struct rm_pogo_data *pdata,
					      const uint8_t *payload,
					      const size_t payload_size);

/* 
 * Iterates over all the write responses in a write response message and prints
 * a warning if any write attribute command failed.
 *
 * @payload start of write attribute command response payload.
 * @payload_size size of payload.
 */
void check_attribute_write_response(struct rm_pogo_data *pdata,
				    const uint8_t *payload,
				    const size_t payload_size);

/*
 * Uses the user_command concept to read or write an attribute. This function
 * acquire the pdata->lock.
 *
 * The command, attribute id and potential payload are
 * parsed from the string at buf. See function definition for detailed
 * documentation.
 *
 * Returns count on success or error.
 */
int exec_attribute_user_command(struct rm_pogo_data *pdata, const char *buf,
				const size_t count);

#endif /* __POGO_ATTRIBUTE_STORAGE_H__ */
