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
		handle->payload_offset += ATTR_DATA_OFFSET(attribute_rx_u) +
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
