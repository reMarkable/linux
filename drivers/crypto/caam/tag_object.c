// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2018-2020 NXP
 */

#include <linux/export.h>
#include <linux/string.h>
#include <linux/errno.h>

#include "tag_object.h"
#include "desc.h"

/**
 * is_key_type -	Check if the object is a key
 *
 * @type:		The object type
 *
 * Return:		True if the object is a key (of black or red color),
 *			false otherwise
 */
bool is_key_type(u32 type)
{
	/* Check type bitfield from object type */
	return ((type >> TAG_OBJ_TYPE_OFFSET) & TAG_OBJ_TYPE_MASK) == 0;
}
EXPORT_SYMBOL(is_key_type);

/**
 * is_trusted_type -	Check if the object is a trusted key
 *			Trusted Descriptor Key Encryption Key (TDKEK)
 *
 * @type:		The object type
 *
 * Return:		True if the object is a trusted key,
 *			false otherwise
 */
bool is_trusted_type(u32 type)
{
	/* Check type bitfield from object type */
	return ((type >> TAG_OBJ_TK_OFFSET) & TAG_OBJ_TK_MASK) == 1;
}
EXPORT_SYMBOL(is_trusted_type);

/**
 * is_black_key -	Check if the tag object header is a black key
 * @header:		The tag object header configuration
 *
 * Return:		True if is a black key, false otherwise
 */
bool is_black_key(const struct header_conf *header)
{
	u32 type = header->type;
	/* Check type and color bitfields from tag object type */
	return (type & (BIT(TAG_OBJ_COLOR_OFFSET) |
			BIT(TAG_OBJ_TYPE_OFFSET))) == BIT(TAG_OBJ_COLOR_OFFSET);
}
EXPORT_SYMBOL(is_black_key);

/**
 * is_valid_header_conf - Check if the header configuration is valid
 * @header:		The header configuration
 *
 * Return:		True if the header of the tag object configuration,
 *			has the TAG_OBJECT_MAGIC number and a valid type,
 *			false otherwise
 */
bool is_valid_header_conf(const struct header_conf *header)
{
	return (header->_magic_number == TAG_OBJECT_MAGIC);
}

/**
 * get_tag_object_header_conf - Retrieve the address of tag object
 *				header configuration
 * @buffer:			Buffer containing the tag object
 * @size:			The size of buffer
 * @header:			Returned tag object header configuration
 *
 * Return:			'0' on success, error code otherwise
 */
int get_tag_object_header_conf(const void *buffer, size_t size,
			       struct header_conf **header)
{
	bool valid;

	/* Retrieve the tag object */
	struct tagged_object *tag_obj = (struct tagged_object *)buffer;

	/* Check if one can retrieve the tag object header configuration */
	if (size < TAG_OVERHEAD_SIZE)
		return -EINVAL;

	/* Check tag object header configuration */
	valid = is_valid_header_conf(&tag_obj->header);

	/* Retrieve the tag object header configuration address */
	*header = &tag_obj->header;

	return valid ? 0 : -EINVAL;
}
EXPORT_SYMBOL(get_tag_object_header_conf);

/**
 * get_key_conf -	Retrieve the key configuration,
 *			meaning the length of the black key and
 *			the KEY command parameters needed for CAAM
 * @header:		The tag object header configuration
 * @real_len:		Key length
 * @load_param:		Load parameters for KEY command:
 *			- indicator for encrypted keys: plaintext or black
 *			- indicator for encryption mode: AES-ECB or AES-CCM
 *			- indicator for encryption keys: JDKEK or TDKEK
 */
void get_key_conf(const struct header_conf *header,
		  u32 *real_len, u32 *load_param)
{
	*real_len = header->real_len;
	/* Based on the color of the key, set key encryption bit (ENC) */
	*load_param = ((header->type >> TAG_OBJ_COLOR_OFFSET) &
		       TAG_OBJ_COLOR_MASK) << KEY_ENC_OFFSET;
	/*
	 * For red keys, the TK and EKT bits are ignored.
	 * So we set them anyway, to be valid when the key is black.
	 */
	*load_param |= ((header->type >> TAG_OBJ_TK_OFFSET) &
			 TAG_OBJ_TK_MASK) << KEY_TK_OFFSET;
	*load_param |= ((header->type >> TAG_OBJ_EKT_OFFSET) &
			 TAG_OBJ_EKT_MASK) << KEY_EKT_OFFSET;
}
EXPORT_SYMBOL(get_key_conf);

/**
 * get_tagged_data -	Retrieve the address of the data and size
 *			of the tagged object
 * @tagged_object:		Pointer to tag object
 * @tagged_object_size:		The tagged object size
 * @data:			Returned the address of the data from
 *				the tagged object
 * @data_size:			Returned the size of the data from the
 *				tagged object
 *
 * Return:			'0' on success, error code otherwise
 */
int get_tagged_data(const void *tagged_object, size_t tagged_object_size,
		    const void **data, u32 *data_size)
{
	/* Retrieve the tag object */
	struct tagged_object *tag_obj = (struct tagged_object *)tagged_object;
	/* Check if one can retrieve the data from the tagged object */
	if (tagged_object_size < TAG_OVERHEAD_SIZE)
		return -EINVAL;

	/* Retrieve the address of the data/object from the tagged object */
	*data = &tag_obj->object;
	/* Retrieve the size of the data from the tagged object */
	*data_size = tagged_object_size - TAG_OVERHEAD_SIZE;

	return 0;
}
EXPORT_SYMBOL(get_tagged_data);

/**
 * init_tag_object_header - Initialize the tag object header by setting up
 *			the TAG_OBJECT_MAGIC number, tag object version,
 *			a valid type and the object's length
 * @header:		The header configuration to initialize
 * @version:		The tag object version
 * @type:		The tag object type
 * @len:		The object (actual data) length
 */
void init_tag_object_header(struct header_conf *header, u32 version,
			    u32 type, size_t len)
{
	header->_magic_number = TAG_OBJECT_MAGIC;
	header->version = version;
	header->type = type;
	header->real_len = len;
}
EXPORT_SYMBOL(init_tag_object_header);

/**
 * set_tag_object_header_conf - Set tag object header configuration
 * @header:			The tag object header configuration to set
 * @buffer:			The buffer needed to be tagged
 * @buf_size:			The buffer size
 * @tag_obj_size:		The tagged object size
 *
 * Return:			'0' on success, error code otherwise
 */
int set_tag_object_header_conf(const struct header_conf *header,
			       void *buffer, size_t buf_size, u32 *tag_obj_size)
{
	/* Retrieve the tag object */
	struct tagged_object *tag_obj = (struct tagged_object *)buffer;
	/*
	 * Requested size for the tagged object is the buffer size
	 * and the header configuration size (TAG_OVERHEAD_SIZE)
	 */
	size_t req_size = buf_size + TAG_OVERHEAD_SIZE;

	/*
	 * Check if the configuration can be set,
	 * based on the size of the tagged object
	 */
	if (*tag_obj_size < req_size)
		return -EINVAL;

	/*
	 * Buffers might overlap, use memmove to
	 * copy the buffer into the tagged object
	 */
	memmove(&tag_obj->object, buffer, buf_size);
	/* Copy the tag object header configuration into the tagged object */
	memcpy(&tag_obj->header, header, TAG_OVERHEAD_SIZE);
	/* Set tagged object size */
	*tag_obj_size = req_size;

	return 0;
}
EXPORT_SYMBOL(set_tag_object_header_conf);
