// SPDX-License-Identifier: GPL-2.0-only
#include "pogo_attribute_storage.h"

#include <linux/byteorder/generic.h>

const char *const get_attr_name(const attribute_storage_id_t attr)
{
	if (attr >= ATTRIBUTE_ID_UNKNOWN)
		return ATTR_NAMES[ATTRIBUTE_ID_UNKNOWN];

	if (ATTR_NAMES[attr] == NULL)
		return ATTR_NAMES[ATTRIBUTE_ID_UNKNOWN];

	return ATTR_NAMES[attr];
}

const char *const
get_attr_data_type_name(const attribute_storage_data_type_t attr_type)
{
	if (attr_type >= ATTRIBUTE_STORAGE_DATA_UNKNOWN || attr_type < 0)
		return ATTR_DATA_TYPE_NAMES[ATTRIBUTE_STORAGE_DATA_UNKNOWN];

	if (ATTR_DATA_TYPE_NAMES[attr_type] == NULL)
		return ATTR_DATA_TYPE_NAMES[ATTRIBUTE_STORAGE_DATA_UNKNOWN];

	return ATTR_DATA_TYPE_NAMES[attr_type];
}

const char *const get_attr_status_name(const attribute_storage_status_t status)
{
	if (status >= ATTRIBUTE_STORAGE_STATUS_UNKNOWN || status < 0)
		return ATTR_STATUS_NAMES[ATTRIBUTE_STORAGE_STATUS_UNKNOWN];

	if (ATTR_STATUS_NAMES[status] == NULL)
		return ATTR_STATUS_NAMES[ATTRIBUTE_STORAGE_STATUS_UNKNOWN];

	return ATTR_STATUS_NAMES[status];
}

int kstr_to_attr_value(attribute_tx_u *attribute_tx, const uint8_t *buf)
{
	size_t size = 0;
	int ret = -EINVAL;

	if (!is_known_attr_type(attribute_tx->type))
		return ret;

	size = get_type_size(attribute_tx->type);

	switch (attribute_tx->type) {
	case ATTRIBUTE_STORAGE_DATA_8_BIT:
		ret = kstrtos8(buf, 0,
			       &attribute_tx->ATTR_VAL_BY_TYPE(
				       ATTRIBUTE_STORAGE_DATA_8_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_16_BIT:
		ret = kstrtos16(buf, 0,
				&attribute_tx->ATTR_VAL_BY_TYPE(
					ATTRIBUTE_STORAGE_DATA_16_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_32_BIT:
		ret = kstrtos32(buf, 0,
				&attribute_tx->ATTR_VAL_BY_TYPE(
					ATTRIBUTE_STORAGE_DATA_32_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_BOOLEAN:
		ret = kstrtobool(buf, &attribute_tx->ATTR_VAL_BY_TYPE(
					      ATTRIBUTE_STORAGE_DATA_BOOLEAN));
		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT:
		ret = kstrtou8(buf, 0,
			       &attribute_tx->ATTR_VAL_BY_TYPE(
				       ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT:
		ret = kstrtou16(
			buf, 0,
			&attribute_tx->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT:
		ret = kstrtou32(
			buf, 0,
			&attribute_tx->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT:
		ret = kstrtou8(buf, 0,
			       &attribute_tx->ATTR_VAL_BY_TYPE(
				       ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_OCTET_STRING:
		WARN(1,
		     "%s should not be used for ATTRIBUTE_STORAGE_DATA_OCTET_STRING\n",
		     __func__);
		ret = -EINVAL;
		break;
	case ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING:
		WARN(1,
		     "%s should not be used for ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING\n",
		     __func__);
		ret = -EINVAL;
		break;
	case ATTRIBUTE_STORAGE_DATA_ARRAY:
		WARN(1,
		     "%s should not be used for ATTRIBUTE_STORAGE_DATA_ARRAY\n",
		     __func__);
		ret = -EINVAL;
		break;
	default:
		WARN(1, "%s called for unhandled type 0x%x\n", __func__,
		     attribute_tx->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/* Size of the value of an attribute.
 * This function doesn't really make sense for string and array attributes, which are variable length.
 */
uint8_t get_type_size(const attribute_storage_data_type_t data_type)
{
	switch (data_type) {
	case ATTRIBUTE_STORAGE_DATA_8_BIT:
		return sizeof(GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_8_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_16_BIT:
		return sizeof(GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_16_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_32_BIT:
		return sizeof(GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_32_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_BOOLEAN:
		return sizeof(GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_BOOLEAN));
		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT:
		return sizeof(
			GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT:
		return sizeof(
			GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT:
		return sizeof(
			GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT:
		return sizeof(GET_DATA_TYPE(ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_OCTET_STRING:
		WARN(1,
		     "%s: get_type_size should not be used for ATTRIBUTE_STORAGE_DATA_OCTET_STRING\n",
		     __func__);
		return 0;
		break;
	case ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING:
		WARN(1,
		     "%s: get_type_size should not be used for ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING\n",
		     __func__);
		return 0;
		break;
	case ATTRIBUTE_STORAGE_DATA_ARRAY:
		WARN(1,
		     "%s: get_type_size should not be used for ATTRIBUTE_STORAGE_DATA_ARRAY\n",
		     __func__);
		return 0;
		break;
	default:
		WARN(1, "%s: get_type_size called for unhandled type 0x%x\n",
		     __func__, data_type);
		return 0;
	}
}

bool parse_received_attributes(parsed_received_attributes_handle_t *handle,
			       const attribute_rx_u **attribute_ptr)
{
	struct rm_pogo_data *pdata = handle->pdata;
	attribute_rx_u *attribute;

	dev_dbg(pdata->dev,
		"%s: parsing attributes payload_size %d payload_offset %d\n",
		__func__, handle->payload_size, handle->payload_offset);

	/* Are we done? */
	if (handle->payload_offset >= handle->payload_size) {
		*attribute_ptr = NULL;
		return false;
	}

	attribute =
		(attribute_rx_u *)(handle->payload + handle->payload_offset);
	*attribute_ptr = attribute;
	/*
	 * If the status is not success then we cannot handle this response.
	 * The data type and data value fields will be dropped.
	 */
	if (attribute->status != ATTRIBUTE_STORAGE_SUCCESS) {
		dev_warn(
			pdata->dev,
			"%s: received status %s (0x%x) for attribute read request %s (0x%x)\n",
			__func__, get_attr_status_name(attribute->status),
			attribute->status, get_attr_name(attribute->id),
			attribute->id);
		handle->payload_offset += offsetof(attribute_rx_u, type);
		return true;
	}

	dev_dbg(pdata->dev,
		"%s: enum %s (0x%x) status %s (0x%x) data_type %s (0x%x) \n",
		__func__, get_attr_name(attribute->id), attribute->id,
		get_attr_status_name(attribute->status), attribute->status,
		get_attr_data_type_name(attribute->type), attribute->type);

	/*
	 * Update payload_offset according to attr type
	 */
	if (attribute->type == ATTRIBUTE_STORAGE_DATA_OCTET_STRING ||
	    attribute->type == ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING) {
		handle->payload_offset +=
			offsetof(attribute_rx_u, str) + attribute->str_len;

	} else if (attribute->type == ATTRIBUTE_STORAGE_DATA_ARRAY) {
		handle->payload_offset +=
			offsetof(attribute_rx_u, array_data) +
			(attribute->n_len * get_type_size(attribute->subtype));
	} else {
		handle->payload_offset += offsetof(attribute_rx_u, payload) +
					  get_type_size(attribute->type);
	}

	/* Debug printing */
	switch (attribute->type) {
	case ATTRIBUTE_STORAGE_DATA_8_BIT:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_8_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_16_BIT:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_16_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_32_BIT:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_32_BIT));

		break;
	case ATTRIBUTE_STORAGE_DATA_BOOLEAN:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_BOOLEAN));

		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_UNSIGNED_8_BIT));

		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_UNSIGNED_16_BIT));
		break;
	case ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_UNSIGNED_32_BIT));

		break;
	case ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT:
		dev_dbg(pdata->dev, "0x%x\n",
			attribute->ATTR_VAL_BY_TYPE(
				ATTRIBUTE_STORAGE_DATA_ENUM_8_BIT));

		break;
	case ATTRIBUTE_STORAGE_DATA_OCTET_STRING:
		dev_dbg(pdata->dev, "oct string len %d\n", attribute->str_len);
		dev_dbg(pdata->dev, "%.*s\n", attribute->str_len,
			attribute->str);
		break;
	case ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING:
		dev_dbg(pdata->dev, "string len %d\n", attribute->str_len);
		dev_dbg(pdata->dev, "%.*s\n", attribute->str_len,
			attribute->str);
		break;
	case ATTRIBUTE_STORAGE_DATA_ARRAY:
		dev_dbg(pdata->dev, "array %s[%d]",
			get_attr_data_type_name(attribute->subtype),
			attribute->n_len);
		break;
	default:
		dev_warn(pdata->dev, "dunno how to print data type 0x%x\n",
			 attribute->type);
		break;
	};

	return true;
}

int pogo_onewire_write_attribute(struct rm_pogo_data *pdata,
				 const uint8_t *const msg, const size_t size)
{
	return pogo_onewire_write(pdata, POGO_CMD_ATTRIBUTE_WRITE, 0,
				  (unsigned char *)msg, size, false);
}

int pogo_onewire_read_attribute(
	struct rm_pogo_data *pdata,
	const attribute_storage_id_t *const attribute_ids, const size_t n)
{
	dev_dbg(pdata->dev, "%s: reading %d attributes (%d bytes total)\n",
		__func__, n, n * sizeof(attribute_storage_id_t));
	return pogo_onewire_write(pdata, POGO_CMD_ATTRIBUTE_READ, 0,
				  (unsigned char *)attribute_ids,
				  n * sizeof(attribute_storage_id_t), true);
}

/* Return number of attributes succesfully handled */
int update_pdata_from_attribute_read_response(struct rm_pogo_data *pdata,
					      const uint8_t *payload,
					      const size_t payload_size)
{
	bool ret;
	bool handled;
	unsigned int num_handled = 0;
	const attribute_rx_u *current_attr;
	parsed_received_attributes_handle_t attr_parser_handle = {
		.pdata = pdata,
		.payload = payload,
		.payload_size = payload_size,
		.payload_offset = 0
	};

	for (ret = parse_received_attributes(&attr_parser_handle,
					     &current_attr);
	     ret; ret = parse_received_attributes(&attr_parser_handle,
						  &current_attr)) {
		dev_dbg(pdata->dev, "%s: handling %s\n", __func__,
			get_attr_name(current_attr->id));

		if (current_attr->status != ATTRIBUTE_STORAGE_SUCCESS) {
			dev_warn(
				pdata->dev,
				"%s: read attribute returned status 0x%x for attribute %s\n",
				__func__, current_attr->status,
				get_attr_name(current_attr->id));
			continue;
		}

		handled = true;

		switch (current_attr->id) {
		case ATTRIBUTE_ID_DEVICE_CLASS:
			pdata->dev_info.device_class =
				current_attr->ATTR_VAL_BY_ID(
					ATTRIBUTE_ID_DEVICE_CLASS);
			break;
		case ATTRIBUTE_ID_FW_VERSION:
			memcpy(&pdata->dev_info.fw_version,
			       &current_attr->ATTR_VAL_BY_ID(
				       ATTRIBUTE_ID_FW_VERSION),
			       sizeof(pdata->dev_info.fw_version));
			break;
		case ATTRIBUTE_ID_LANGUAGE:
			pdata->dev_info.language = current_attr->ATTR_VAL_BY_ID(
				ATTRIBUTE_ID_LANGUAGE);
			dev_dbg(pdata->dev, "%s: updating language to %d\n",
				__func__, pdata->dev_info.language);
			break;
		case ATTRIBUTE_ID_RM_SERIAL_NUMBER:
			if (sizeof(pdata->dev_info.serial) !=
			    current_attr->str_len) {
				dev_warn(
					pdata->dev,
					"%s: Got serial number with unexpected size. Got %d expected %d\n",
					__func__, current_attr->str_len,
					sizeof(pdata->dev_info.serial));
			} else {
				memcpy(pdata->dev_info.serial,
				       current_attr->str,
				       sizeof(pdata->dev_info.serial));
				dev_dbg(pdata->dev,
					"%s: got serial number %.*s\n",
					__func__,
					sizeof(pdata->dev_info.serial),
					pdata->dev_info.serial);
			}
			break;
		case ATTRIBUTE_ID_KEY_LAYOUT:
			pdata->dev_info.keylayout =
				current_attr->ATTR_VAL_BY_ID(
					ATTRIBUTE_ID_KEY_LAYOUT);
			break;
		case ATTRIBUTE_ID_IMAGE_START_ADDRESS:
			pdata->dev_info.image_start_addr =
				current_attr->ATTR_VAL_BY_ID(
					ATTRIBUTE_ID_IMAGE_START_ADDRESS);
			break;
		case ATTRIBUTE_ID_MFG_LOG:
			pdata->mfg_log = current_attr->ATTR_VAL_BY_ID(
				ATTRIBUTE_ID_MFG_LOG);
			dev_dbg(pdata->dev, "received mfg log from MCU: %d\n",
				pdata->mfg_log);
			break;
		case ATTRIBUTE_ID_DEVICE_NAME:
			if (pdata->pogo_name) {
				devm_kfree(pdata->dev, pdata->pogo_name);
				pdata->pogo_name = NULL;
			}
			pdata->pogo_name =
				devm_kmalloc(pdata->dev,
					     current_attr->str_len + 1,
					     GFP_KERNEL);

			snprintf(pdata->pogo_name, current_attr->str_len + 1,
				 "%s", current_attr->str);

			dev_dbg(pdata->dev, "Found device %s\n",
				pdata->pogo_name);
			break;
		case ATTRIBUTE_ID_DEVICE_ID:
			if (sizeof(pdata->dev_info.device_id) !=
			    current_attr->n_len *
				    get_type_size(current_attr->subtype)) {
				dev_err(pdata->dev,
					"%s: received device_id with unexpected size. Expected %d, got %d\n",
					__func__,
					sizeof(pdata->dev_info.device_id),
					current_attr->n_len *
						get_type_size(
							current_attr->subtype));
				break;
			}
			memcpy(pdata->dev_info.device_id,
			       current_attr->array_data,
			       sizeof(pdata->dev_info.device_id));
			dev_dbg(pdata->dev,
				"%s: received device_id: 0x%x 0x%x 0x%x 0x%x \n",
				__func__, pdata->dev_info.device_id[0],
				pdata->dev_info.device_id[1],
				pdata->dev_info.device_id[2],
				pdata->dev_info.device_id[3]);
			break;
		case ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS:
			pdata->mcu_alive_interval =
				current_attr->ATTR_VAL_BY_ID(
					ATTRIBUTE_ID_ALIVE_MESSAGE_TIMEOUT_MS);
			break;
		default:
			handled = false;
			dev_warn(
				pdata->dev,
				"%s: got attribute we don't update pdata with (%s)\n",
				__func__, get_attr_name(current_attr->id));
		}

		if (handled) {
			++num_handled;
			dev_dbg(pdata->dev,
				"%s: succesfully updated pdata with attribute %s (total handled: %d) \n",
				__func__, get_attr_name(current_attr->id),
				num_handled);
		}
	}
	return num_handled;
}

void check_attribute_write_response(struct rm_pogo_data *pdata,
				    const uint8_t *payload,
				    const size_t payload_size)
{
	attribute_tx_response_t *response = (attribute_tx_response_t *)payload;

	const size_t element_size = sizeof(attribute_storage_status_t) +
				    sizeof(attribute_storage_id_t);
	unsigned int num_attrs = payload_size / element_size;
	int i = 0;

	if (payload_size % element_size != 0) {
		dev_warn(pdata->dev, "%s: invalid payload_size %d\n", __func__,
			 payload_size);
		return;
	}

	for (i = 0; i < num_attrs; ++i) {
		if (response[i].status != ATTRIBUTE_STORAGE_SUCCESS) {
			dev_warn(pdata->dev,
				 "%s: %s (0x%x) status %s (0x%x) \n", __func__,
				 get_attr_name(response[i].id), response[i].id,
				 get_attr_status_name(response[i].status),
				 response[i].status);
		}
	}
}

/*
 * kstrto*() functions are used to convert strings to data
 * and users may use syntax compatible with those functions.
 *
 * hex2bin() is used to convert array payload strings.
 *
 * Read syntax (serial number)
 * --------------------------------
 * CMD:ATTR_ID: 0x20:0x12
 *
 * Write syntax - Simple data types
 * --------------------------------
 * CMD:ATTR_ID:ATTR_DATA_TYPE:VALUE
 *
 * set mfg_log to 0x07
 * 0x21:0x20:0x18:0x07
 *
 * Set alive interval to 900ms
 * 0x21:0x30:0x19:900
 *
 * Set ATTRIBUTE_ID_SUSPEND_WAIT_FOR_VBUS_LOSS_US to 2000001
 * 0x21:53:0x1b:2000001
 *
 * Write syntax - string
 * --------------------------------
 * Write serial number RMAAA-BBB-CCCCD example
 * CMD:ATTR_ID:ATTR_DATA_TYPE:NUM_ELEMENTS::<PAYLOAD in hex>
 * 0x21:0x12:0x42:0x0F:524d4141412d4242422d4343434344
 *
 * Write syntax - array
 * --------------------------------
 * Write array example. Note that there's no
 * writable array attribute at the time of writing, so this example assumes
 * that DEVICE_ID is writeable.
 *
 * WriteCmd:Device Id:array type:sub type:2 bytes n len:<data in hex>
 * 0x21:0x05:0x48:0x0B:0x0004:4370051948809aaf080915140c0500f5
 */
#define MAX_ATTR_USER_CMD_FIELDS 6
int exec_attribute_user_command(struct rm_pogo_data *pdata,
				const char *buf_orig, const size_t count)
{
	int ret = count;
	int i = 0;

	attribute_storage_id_t attr_id;
	uint8_t cmd = 0;
	attribute_storage_data_type_t attr_type;
	attribute_storage_data_type_t attr_data_sub_type;
	uint16_t attr_data_n_len;
	uint8_t attr_str_len = 0;
	uint8_t string_token_length = 0;

	attribute_tx_u *attr_tx = NULL;

	/* Pointer to allocation of working-copy of buf_orig used by strsep */
	char *alloc = NULL;
	/* Pointer manipulated by strsep */
	char *buf = NULL;
	char *tokens[MAX_ATTR_USER_CMD_FIELDS] = { 0 };

	if (buf_orig == NULL || count == 0) {
		dev_err(pdata->dev,
			"%s: syntax error. Read attribute command example: 0x20:0x30\n",
			__func__);
		return -EINVAL;
	}

	alloc = (char *)devm_kzalloc(pdata->dev, count, GFP_KERNEL);
	if (!alloc) {
		dev_err(pdata->dev, "%s: failed to alloc 0x%s\n", __func__,
			alloc);
		return -ENOMEM;
	}

	buf = alloc;
	memcpy(alloc, buf_orig, count);

	for (i = 0; i < MAX_ATTR_USER_CMD_FIELDS; ++i) {
		tokens[i] = strsep(&buf, ":");
		if (tokens[i] != NULL)
			dev_dbg(pdata->dev, "%s: %d %s\n", __func__, i,
				tokens[i]);
		if (buf == NULL)
			break;
	}

	/* Command field */
	if (tokens[0] == NULL || (ret = kstrtou8(tokens[0], 0, &cmd)) < 0) {
		dev_err(pdata->dev, "%s: missing CMD field\n", __func__);
		ret = ret ? ret < 0 : -EINVAL;
		goto free_buf_and_return;
	}

	/* Handle command */
	if (!(cmd == POGO_CMD_ATTRIBUTE_READ ||
	      cmd == POGO_CMD_ATTRIBUTE_WRITE)) {
		dev_err(pdata->dev,
			"%s: command must be either attribute read (0x%x) or attribute write (0x%x)\n",
			__func__, POGO_CMD_ATTRIBUTE_READ,
			POGO_CMD_ATTRIBUTE_WRITE);
		ret = -EINVAL;
		goto free_buf_and_return;
	}

	/* ATTR_ID field */
	if (tokens[1] == NULL || (ret = kstrtou8(tokens[1], 0, &attr_id)) < 0) {
		dev_err(pdata->dev, "%s: missing ATTR_ID field ptr %p ret %x\n",
			__func__, tokens[1], ret);
		if (tokens[1] != NULL)
			dev_err(pdata->dev, "%s: tokens[1] %s\n", __func__,
				tokens[1]);
		ret = ret ? ret < 0 : -EINVAL;
		goto free_buf_and_return;
	}

	/* Handle attribute id */
	if (!is_known_attr_id(attr_id)) {
		dev_err(pdata->dev, "%s: unknown attribute id 0x%x\n", __func__,
			attr_id);
		ret = -EINVAL;
		goto free_buf_and_return;
	}

	/* Validate tokens needed for write commands */
	if (cmd == POGO_CMD_ATTRIBUTE_WRITE) {
		/* Data type */
		if (tokens[2] == NULL ||
		    (ret = kstrtou8(tokens[2], 0, &attr_type)) < 0) {
			dev_err(pdata->dev,
				"%s: write attribute command is missing data type\n",
				__func__);
			ret = ret ? ret < 0 : -EINVAL;
			goto free_buf_and_return;
		}

		if (!is_known_attr_type(attr_type)) {
			dev_warn(pdata->dev, "%s: unsupported data type 0x%x\n",
				 __func__, attr_type);

			ret = -EINVAL;
			goto free_buf_and_return;
		}

		// String type
		if (attr_type == ATTRIBUTE_STORAGE_DATA_CHARACTER_STRING) {
			if (tokens[3] == NULL ||
			    (ret = kstrtou8(tokens[3], 0, &attr_str_len)) < 0) {
				dev_err(pdata->dev,
					"%s: write attribute command is missing string length field\n",
					__func__);
				ret = ret ? ret < 0 : -EINVAL;
				goto free_buf_and_return;
			}

			// Check number of elements
			if (tokens[4] == NULL) {
				dev_err(pdata->dev,
					"%s: write attribute command is missing string data\n",
					__func__);
				ret = -EINVAL;
				goto free_buf_and_return;
			}

			string_token_length = strlen(tokens[4]);
			// String supplied may be larger to account for
			// null-byte or newline
			if (string_token_length < 2 * attr_str_len) {
				dev_err(pdata->dev,
					"%s: string length was specified as %d. expected hex string of length %d, but got %d instead \n",
					__func__, attr_str_len,
					attr_str_len * 2, string_token_length);
				ret = -EINVAL;
				goto free_buf_and_return;
			}

			mutex_lock(&pdata->lock);
			dev_dbg(pdata->dev, "%s: sending write command\n",
				__func__);

			INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_WRITE,
					  offsetof(attribute_tx_u, str) +
						  attr_str_len);
			attr_tx = (attribute_tx_u *)pdata->user_command_data;
			attr_tx->id = attr_id;
			attr_tx->type = attr_type;
			attr_tx->str_len = attr_str_len;

			ret = hex2bin(attr_tx->str, tokens[4], attr_str_len);
			if (ret < 0) {
				dev_err(pdata->dev,
					"%s: failed to parse hex string %.*s\n",
					__func__, string_token_length,
					tokens[4]);
				INIT_USER_COMMAND(POGO_CMD_NONE, 0);
			}
			mutex_unlock(&pdata->lock);
			goto free_buf_and_return;
		}
		// Array type
		// 	- 1 byte sub type
		// 	- 2 bytes num elements
		else if (attr_type == ATTRIBUTE_STORAGE_DATA_ARRAY) {
			if (tokens[3] == NULL ||
			    (ret = kstrtou8(tokens[3], 0, &attr_data_sub_type) <
				   0)) {
				dev_err(pdata->dev,
					"%s: write array sub type field missing or badly formatted\n",
					__func__);
				ret = ret ? ret < 0 : -EINVAL;
				goto free_buf_and_return;
			}

			if (tokens[4] == NULL ||
			    (ret = kstrtou16(tokens[4], 0, &attr_data_n_len) <
				   0)) {
				dev_err(pdata->dev,
					"%s: write array length field missing or badly formatted\n",
					__func__);
				ret = ret ? ret < 0 : -EINVAL;
				goto free_buf_and_return;
			}

			if (tokens[5] == NULL) {
				dev_err(pdata->dev,
					"%s: write array data payload missing\n",
					__func__);
				ret = -EINVAL;
				goto free_buf_and_return;
			}

			string_token_length = strlen(tokens[5]);
			if (string_token_length < 2 * attr_data_n_len) {
				dev_err(pdata->dev,
					"%s: alledged array length %d does not match length of actually-provided hex string %d \n",
					__func__, attr_data_n_len,
					string_token_length / 2);
				ret = -EINVAL;
				goto free_buf_and_return;
			}

			mutex_lock(&pdata->lock);
			dev_dbg(pdata->dev, "%s: sending write command\n",
				__func__);

			INIT_USER_COMMAND(
				POGO_CMD_ATTRIBUTE_WRITE,
				offsetof(attribute_tx_u, array_data) +
					attr_data_n_len *
						get_type_size(
							attr_data_sub_type));

			attr_tx = (attribute_tx_u *)pdata->user_command_data;
			attr_tx->id = attr_id;
			attr_tx->type = attr_type;
			attr_tx->subtype = attr_data_sub_type;
			attr_tx->n_len = attr_data_n_len;

			ret = hex2bin(
				attr_tx->array_data, tokens[5],
				attr_data_n_len *
					get_type_size(attr_data_sub_type));
			if (ret < 0) {
				dev_err(pdata->dev,
					"%s: failed to parse hex string %.*s\n",
					__func__, string_token_length,
					tokens[5]);
				INIT_USER_COMMAND(POGO_CMD_NONE, 0);
			}
			mutex_unlock(&pdata->lock);
			goto free_buf_and_return;

		}
		// Simple data type
		else {
			// Value
			if (tokens[3] == NULL) {
				dev_err(pdata->dev,
					"%s: write attribute command is missing data field\n",
					__func__);
				ret = -EINVAL;
				goto free_buf_and_return;
			}

			mutex_lock(&pdata->lock);
			INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_WRITE,
					  offsetof(attribute_tx_u, payload) +
						  get_type_size(attr_type));

			attr_tx = (attribute_tx_u *)pdata->user_command_data;
			attr_tx->id = attr_id;
			attr_tx->type = attr_type;
			if ((ret = kstr_to_attr_value(attr_tx, tokens[3]))) {
				dev_err(pdata->dev,
					"%s: failed to parse data from string '%s'\n",
					__func__, tokens[3]);
				INIT_USER_COMMAND(POGO_CMD_NONE, 0);
			}
			mutex_unlock(&pdata->lock);
			goto free_buf_and_return;
		}
	} else if (cmd == POGO_CMD_ATTRIBUTE_READ) {
		dev_dbg(pdata->dev, "%s: send attribute read for attr id %s \n",
			__func__, get_attr_name(attr_id));
		mutex_lock(&pdata->lock);
		INIT_USER_COMMAND(POGO_CMD_ATTRIBUTE_READ,
				  sizeof(attribute_storage_id_t));
		*(attribute_storage_id_t *)pdata->user_command_data = attr_id;
		mutex_unlock(&pdata->lock);
		goto free_buf_and_return;

	} else {
		dev_err(pdata->dev,
			"%s: unsupported command 0x%x, must be either write (0x%x) or read (0x%x)\n",
			__func__, cmd, POGO_CMD_ATTRIBUTE_WRITE,
			POGO_CMD_ATTRIBUTE_READ);
		goto free_buf_and_return;
	}

free_buf_and_return:
	if (alloc)
		devm_kfree(pdata->dev, alloc);
	if (ret < 0) {
		return ret;
	} else {
		return count;
	}
}
