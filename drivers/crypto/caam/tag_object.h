/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2018-2020 NXP
 */

#ifndef _TAG_OBJECT_H_
#define _TAG_OBJECT_H_

#include <linux/types.h>
#include <linux/bitops.h>

/**
 * Magic number to identify the tag object structure
 * 0x54 = 'T'
 * 0x61 = 'a'
 * 0x67 = 'g'
 * 0x4f = 'O'
 */
#define TAG_OBJECT_MAGIC	0x5461674f
#define TAG_MIN_SIZE		(2 * sizeof(struct header_conf))
#define TAG_OVERHEAD_SIZE	sizeof(struct header_conf)

/*
 * Tag object type is a bitfield:
 *
 * EKT:	Encrypted Key Type (AES-ECB or AES-CCM)
 * TK:	Trusted Key (use Job Descriptor Key Encryption Key (JDKEK)
 *	or Trusted Descriptor Key Encryption Key (TDKEK) to
 *	decrypt the key to be loaded into a Key Register).
 *
 *| Denomination | Security state | Memory  | EKT | TK    | Type | Color |
 *| ------------ | -------------- | ------- | --- | ----- | ---- | ----- |
 *| bit(s)       | 5-6            | 4       | 3   | 2     | 1    | 0     |
 *| option 0     | non-secure     | general | ECB | JDKEK | key  | red   |
 *| option 1     | secure         | secure  | CCM | TDKEK | blob | black |
 *| option 2     | trusted        |         |     |       |      |       |
 *
 * CAAM supports two different Black Key encapsulation schemes,
 * one intended for quick decryption (uses AES-ECB encryption),
 * and another intended for high assurance (uses AES-CCM encryption).
 *
 * CAAM implements both Trusted and normal (non-Trusted) Black Keys,
 * which are encrypted with different key-encryption keys.
 * Both Trusted and normal Descriptors are allowed to encrypt or decrypt
 * normal Black Keys, but only Trusted Descriptors are allowed to
 * encrypt or decrypt Trusted Black Keys.
 */
#define TAG_OBJ_COLOR_OFFSET		0
#define TAG_OBJ_COLOR_MASK		0x1
#define TAG_OBJ_TYPE_OFFSET		1
#define TAG_OBJ_TYPE_MASK		0x1
#define TAG_OBJ_TK_OFFSET		2
#define TAG_OBJ_TK_MASK			0x1
#define TAG_OBJ_EKT_OFFSET		3
#define TAG_OBJ_EKT_MASK		0x1
#define TAG_OBJ_MEM_OFFSET		4
#define TAG_OBJ_MEM_MASK		0x1
#define TAG_OBJ_SEC_STATE_OFFSET	5

/**
 * struct header_conf - Header configuration structure, which represents
 *			the metadata (or simply a header) applied to the
 *			actual data (e.g. black key)
 * @_magic_number     : A magic number to identify the structure
 * @version           : The version of the data contained (e.g. tag object)
 * @type              : The type of data contained (e.g. black key, blob, etc.)
 * @real_len          : Length of the object to be loaded by CAAM
 */
struct header_conf {
	u32 _magic_number;
	u32 version;
	u32 type;
	u32 real_len;
};

/**
 * struct tagged_object - Tag object structure, which represents the metadata
 *                        (or simply a header) and the actual data
 *                        (e.g. black key) obtained from hardware
 * @tag                 : The configuration of the data (e.g. header)
 * @object              : The actual data (e.g. black key)
 */
struct tagged_object {
	struct header_conf header;
	char object;
};

bool is_key_type(u32 type);

bool is_trusted_type(u32 type);

bool is_black_key(const struct header_conf * const header);

bool is_black_key(const struct header_conf * const header);

bool is_valid_header_conf(const struct header_conf *header);

int get_tag_object_header_conf(const void *buffer, size_t buffer_size,
			       struct header_conf **header);

void get_key_conf(const struct header_conf *header,
		  u32 *real_len, u32 *load_param);

int get_tagged_data(const void *buffer, size_t buffer_size,
		    const void **data, u32 *data_size);

void init_tag_object_header(struct header_conf *header, u32 version,
			    u32 type, size_t len);

int set_tag_object_header_conf(const struct header_conf *header,
			       void *buffer, size_t obj_size, u32 *to_size);

#endif /* _TAG_OBJECT_H_ */
