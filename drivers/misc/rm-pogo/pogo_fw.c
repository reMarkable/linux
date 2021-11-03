/*
 * reMarkable POGO firmware write
 *
 * Copyright (C) 2021 reMarkable AS - http://www.remarkable.com/
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

#include "pogo_fsm.h"

#include <linux/export.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>

#define FILENAME_LEN_MAX 64
int pogo_load_fw(struct rm_pogo_data *pdata, char image)
{
	char *filename;
	int ret;

	filename = kzalloc(FILENAME_LEN_MAX, GFP_KERNEL);
	if (!filename)
		return -ENOMEM;

	snprintf(filename, FILENAME_LEN_MAX, "%s_fw_%c", pdata->pogo_name, image);

	ret = request_firmware(&pdata->fw, filename, pdata->dev);
	if (ret < 0) {
		dev_info(pdata->dev, "%s: Failed requesting FW %s load\n",
			__func__, filename);
		goto free_name;
	} else {
		dev_dbg(pdata->dev, "%s: FW %s file loading\n",
			__func__, filename);
	}

free_name:
	kfree(filename);
	return ret;
}

/* fw image header in located at this address in the image file */
#define FWU_IMAGE_HEADER_ADDR 0x100
/* start addresses*/
#define FW_IMAGE_A_ADDR 0x800
#define FW_IMAGE_B_ADDR 0x4000

bool pogo_check_fw_write(struct rm_pogo_data *pdata)
{
	int ret;
	u_int32_t crc;
	char image;

	/* Check which is the running image and load the opposite*/

	if (pdata->dev_info.image_start_addr == FW_IMAGE_A_ADDR) {
		image ='b';
	} else if (pdata->dev_info.image_start_addr == FW_IMAGE_B_ADDR) {
		image = 'a';
	} else {
		dev_info(pdata->dev,
					"Invalid image start address in running image,"
					" aborting fw write\n");
			return false;
	}

	ret = pogo_load_fw(pdata, image);
	/*
	 * It is not an error if we for some reason can't load the file.
	 * It could be that there is no new fw yet hence no file
	 */

	if (ret < 0)
		return false;

	memcpy(&crc, &pdata->fw->data[pdata->fw->size - sizeof(crc)],
											sizeof(crc));

	dev_dbg(pdata->dev, "Found FW CRC 0x%08x\n", crc);

	memcpy(&pdata->fwu_image_header, &pdata->fw->data[FWU_IMAGE_HEADER_ADDR],
											sizeof(pdata->fwu_image_header));

	if (pdata->dev_info.image_start_addr == FW_IMAGE_A_ADDR) {
		if (pdata->fwu_image_header.image_start_address == FW_IMAGE_A_ADDR) {
			dev_info(pdata->dev,
					"Invalid image start address in file image (0x04%x),"
					" aborting fw write\n",
					pdata->fwu_image_header.image_start_address);
			return false;
		}
	}
	if (pdata->dev_info.image_start_addr == FW_IMAGE_B_ADDR) {
		if (pdata->fwu_image_header.image_start_address == FW_IMAGE_B_ADDR) {
			dev_info(pdata->dev,
					"Invalid image start address in file image (0x04%x),"
					" aborting fw write\n",
					pdata->fwu_image_header.image_start_address);
			return false;
		}
	}

	dev_info(pdata->dev, "Running image start address 0x%04x\n",
								pdata->dev_info.image_start_addr);
	dev_info(pdata->dev, "Image in file start address 0x%04x\n",
								pdata->fwu_image_header.image_start_address);


	if (memcmp(&pdata->fwu_image_header.fw_version,
						&pdata->dev_info.fw_version,
						sizeof(pdata->dev_info.fw_version)) != 0) {
		dev_info(pdata->dev,
			"Found FW version %d.%d vs current %d.%d\n",
			pdata->fwu_image_header.fw_version.major, pdata->fwu_image_header.fw_version.minor,
			pdata->dev_info.fw_version.major, pdata->dev_info.fw_version.minor);
		return true;
	} else {
		dev_info(pdata->dev, "FW is up to date: %d.%d\n",
			pdata->dev_info.fw_version.major, pdata->dev_info.fw_version.minor);
	}

	return false;
}

#define FW_BULK_SIZE 512
int pogo_send_fw_packet(struct rm_pogo_data *pdata)
{
	int size, ret;
	u16 fw_off;
	u8 *data;

	size = pdata->fw->size - pdata->fw_off;
	size = size > FW_BULK_SIZE ? FW_BULK_SIZE : size;

	/* allocate space for offset and packet */
	data = kzalloc(sizeof(fw_off) + size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	fw_off = pdata->fw_off;

	memcpy(&data[0], &fw_off, sizeof(fw_off));
	memcpy(&data[sizeof(fw_off)], &pdata->fw->data[fw_off], size);

	dev_dbg(pdata->dev, "%s: off %d\n", __func__, fw_off);

	size += sizeof(fw_off);
	ret = pogo_onewire_write(pdata, POGO_CMD_FW_WRITE_PACKET, 0,
								data, size, true);

	if (!ret)
		pdata->fw_off += FW_BULK_SIZE;

	kfree(data);
	return ret;
}

int pogo_send_fw_validate(struct rm_pogo_data *pdata)
{
	dev_dbg(pdata->dev, "Requesting validation of FW ...\n");
	return pogo_onewire_write(pdata, POGO_CMD_FW_WRITE_VALIDATE_CRC, 0,
			   NULL, 0, true);
}

int pogo_send_fw_validate_image(struct rm_pogo_data *pdata)
{
	dev_dbg(pdata->dev, "Requesting validation of FW image...\n");
	return pogo_onewire_write(pdata, POGO_CMD_FW_WRITE_VALIDATE_IMAGE, 0,
			   NULL, 0, true);
}

void pogo_release_firmware(struct rm_pogo_data *pdata)
{
	if (pdata->fw) {
		dev_dbg(pdata->dev, "Releasing FW\n");
		release_firmware(pdata->fw);
		pdata->fw = NULL;
	}
}
