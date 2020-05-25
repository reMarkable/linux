/*
 * Copyright 2018-2020 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file insert_startcode.h
 *
 */
#ifndef __INSERT_STARTCODE_H__
#define __INSERT_STARTCODE_H__

#include "vpu_b0.h"
#include "mediasys_types.h"
#define SCODE_NEW_SEQUENCE 0x31
#define SCODE_NEW_PICTURE 0x32
#define SCODE_NEW_SLICE 0x33

#define IMX_PAYLOAD_HEADER_SIZE		16

#define IMX_CODEC_VERSION_ID		0x1

#define IMX_CODEC_ID_VC1_SIMPLE		0x10
#define IMX_CODEC_ID_VC1_MAIN		0x11
#define IMX_CODEC_ID_ARV8		0x28
#define IMX_CODEC_ID_ARV9		0x29
#define IMX_CODEC_ID_VP6		0x36
#define IMX_CODEC_ID_VP8		0x36
#define IMX_CODEC_ID_DIVX3		0x38
#define IMX_CODEC_ID_SPK		0x39

#define IMX_VC1_RCV_CODEC_V1_VERSION	0x85
#define IMX_VC1_RCV_CODEC_V2_VERSION	0xC5
#define IMX_VC1_RCV_NUM_FRAMES          0xFF
#define IMX_VC1_RCV_SEQ_EXT_DATA_SIZE	4
#define IMX_VC1_RCV_SEQ_HEADER_LEN	20
#define IMX_VC1_RCV_PIC_HEADER_LEN	4
#define IMX_VC1_NAL_HEADER_LEN		4
#define IMX_VC1_IS_NOT_NAL(id)      ((id & 0x00FFFFFF) != 0x00010000)

#define IMX_VP8_IVF_SEQ_HEADER_LEN	32
#define IMX_VP8_IVF_FRAME_HEADER_LEN	8


struct imx_scd_handler {
	unsigned int vdec_std;

	unsigned int (*insert_scd_seq)(struct vpu_ctx *ctx, unsigned int buffer_size, void *data);
	unsigned int (*insert_scd_pic)(struct vpu_ctx *ctx, unsigned int buffer_size, void *data);
	unsigned int (*insert_scd_slice)(struct vpu_ctx *ctx, unsigned int buffer_size, void *data);
};


struct VPU_FMT_INFO_ARV *get_arv_info(struct vpu_ctx *ctx, u_int8 *src, u_int32 size);
void put_arv_info(struct VPU_FMT_INFO_ARV *arv_frame);
u_int32 single_seq_info_format(struct queue_data *q_data);
unsigned int insert_scode(struct vpu_ctx *ctx, unsigned int scd_type,
			  unsigned int buffer_size, void *data);
bool check_free_size_pic(struct vpu_ctx *ctx, unsigned int buffer_size);

#endif
