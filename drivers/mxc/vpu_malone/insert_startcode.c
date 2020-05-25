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
 * @file insert_startcode.c
 *
 * copyright here may be changed later
 *
 *
 */
#include "insert_startcode.h"

static void set_payload_hdr(unsigned char *dst, unsigned int scd_type, unsigned int codec_id,
			unsigned int buffer_size, unsigned int width, unsigned int height)
{
	unsigned int payload_size;
	/* payload_size = buffer_size + itself_size(16) - start_code(4) */
	payload_size = buffer_size + 12;

	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = scd_type;

	/* length */
	dst[4] = ((payload_size>>16)&0xff);
	dst[5] = ((payload_size>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((payload_size>>0)&0xff);

	/* Codec ID and Version */
	dst[8] = codec_id;
	dst[9] = IMX_CODEC_VERSION_ID;

	/* width */
	dst[10] = ((width>>8)&0xff);
	dst[11] = ((width>>0)&0xff);
	dst[12] = 0x58;

	/* height */
	dst[13] = ((height>>8)&0xff);
	dst[14] = ((height>>0)&0xff);
	dst[15] = 0x50;
}

static void set_vc1_rcv_seqhdr(unsigned char *dst, unsigned char *src,
				unsigned int width, unsigned int height)
{
	unsigned int frames = IMX_VC1_RCV_NUM_FRAMES;
	unsigned int ext_data_size = IMX_VC1_RCV_SEQ_EXT_DATA_SIZE;

	/* 0-2 Number of frames, used default value 0xFF */
	dst[0] = (unsigned char)frames;
	dst[1] = (unsigned char)(frames >> 8);
	dst[2] = (unsigned char)(frames >> 16);

	/* 3 RCV version, used V1 */
	dst[3] = (unsigned char)(IMX_VC1_RCV_CODEC_V1_VERSION);

	/* 4-7 extension data size */
	dst[4] = (unsigned char)ext_data_size;
	dst[5] = (unsigned char)(ext_data_size >> 8);
	dst[6] = (unsigned char)(ext_data_size >> 16);
	dst[7] = (unsigned char)(ext_data_size >> 24);
	/* 8-11 extension data */
	dst[8] = src[0];
	dst[9] = src[1];
	dst[10] = src[2];
	dst[11] = src[3];

	/* height */
	dst[12] = (unsigned char)height;
	dst[13] = (unsigned char)(((height >> 8) & 0xff));
	dst[14] = (unsigned char)(((height >> 16) & 0xff));
	dst[15] = (unsigned char)(((height >> 24) & 0xff));
	/* width */
	dst[16] = (unsigned char)width;
	dst[17] = (unsigned char)(((width >> 8) & 0xff));
	dst[18] = (unsigned char)(((width >> 16) & 0xff));
	dst[19] = (unsigned char)(((width >> 24) & 0xff));
}

static void set_vc1_rcv_pichdr(unsigned char *dst, unsigned int buffer_size)
{
	dst[0] = (unsigned char)buffer_size;
	dst[1] = (unsigned char)(buffer_size >> 8);
	dst[2] = (unsigned char)(buffer_size >> 16);
	dst[3] = (unsigned char)(buffer_size >> 24);
}

static int create_vc1_nal_pichdr(unsigned char *dst, void *data)
{
	int len = 0;

	if (IMX_VC1_IS_NOT_NAL(*((unsigned int *)data))) {
		/* need insert nal header: special ID */
		dst[0] = 0x0;
		dst[1] = 0x0;
		dst[2] = 0x01;
		dst[3] = 0x0D;

		len = 4;
	}

	return len;
}

static unsigned int insert_scd_pic_vc1(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;

	if (q_data->fourcc == V4L2_PIX_FMT_VC1_ANNEX_G) {
		unsigned int rcv_pichdr_size = IMX_VC1_RCV_PIC_HEADER_LEN;
		unsigned char rcv_pichdr[IMX_VC1_RCV_PIC_HEADER_LEN] = { 0 };
		unsigned char scd_pichdr[16] = { 0 };

		set_payload_hdr(scd_pichdr, SCODE_NEW_PICTURE, IMX_CODEC_ID_VC1_SIMPLE,
				rcv_pichdr_size + buffer_size, q_data->width, q_data->height);
		length += copy_buffer_to_stream(ctx, scd_pichdr, 16);
		set_vc1_rcv_pichdr(rcv_pichdr, buffer_size);
		length += copy_buffer_to_stream(ctx, rcv_pichdr, rcv_pichdr_size);
	} else {
		unsigned char nal_hdr[IMX_VC1_NAL_HEADER_LEN] = { 0 };
		unsigned int len;

		len = create_vc1_nal_pichdr(nal_hdr, data);
		length += copy_buffer_to_stream(ctx, nal_hdr, len);
	}

	return length;
}

static unsigned int insert_scd_seq_vc1(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned int rvc_seqhdr_size = IMX_VC1_RCV_SEQ_HEADER_LEN;
	unsigned char rcv_seqhdr[IMX_VC1_RCV_SEQ_HEADER_LEN] = { 0 };
	unsigned char scd_seqhdr[16] = { 0 };

	if (q_data->fourcc == V4L2_PIX_FMT_VC1_ANNEX_G) {
		set_payload_hdr(scd_seqhdr, SCODE_NEW_SEQUENCE, IMX_CODEC_ID_VC1_SIMPLE,
					rvc_seqhdr_size, q_data->width, q_data->height);
		length += copy_buffer_to_stream(ctx, scd_seqhdr, 16);

		set_vc1_rcv_seqhdr(rcv_seqhdr, data, q_data->width, q_data->height);
		length += copy_buffer_to_stream(ctx, rcv_seqhdr, rvc_seqhdr_size);
	} else {
		length += copy_buffer_to_stream(ctx, data, buffer_size);
	}

	return length;
}

static unsigned int insert_scd_pic_vp6(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char pic_header[16] = { 0 };

	set_payload_hdr(pic_header, SCODE_NEW_PICTURE, IMX_CODEC_ID_VC1_SIMPLE,
			buffer_size, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, pic_header, 16);

	return length;
}

static unsigned int insert_scd_seq_vp6(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char seq_header[16] = {0};
	unsigned char *src = (unsigned char *)data;

	set_payload_hdr(seq_header, SCODE_NEW_SEQUENCE, IMX_CODEC_ID_VP6,
			0, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, seq_header, 16);

	/* first data include frame data, need to handle them too */
	length += insert_scd_pic_vp6(ctx, buffer_size, data);
	length += copy_buffer_to_stream(ctx, src, buffer_size);

	return length;
}

static void set_vp8_ivf_seqhdr(unsigned char *dst, int width, int height)
{
	/* 0-3byte signature "DKIF" */
	dst[0] = 0x44;
	dst[1] = 0x4b;
	dst[2] = 0x49;
	dst[3] = 0x46;
	/* 4-5byte version: should be 0*/
	dst[4] = 0x00;
	dst[5] = 0x00;
	/* 6-7 length of Header */
	dst[6] = IMX_VP8_IVF_SEQ_HEADER_LEN;
	dst[7] = IMX_VP8_IVF_SEQ_HEADER_LEN >> 8;
	/* 8-11 VP8 fourcc */
	dst[8] = 0x56;
	dst[9] = 0x50;
	dst[10] = 0x38;
	dst[11] = 0x30;
	/* 12-13 width in pixels */
	dst[12] = width;
	dst[13] = width >> 8;
	/* 14-15 height in pixels */
	dst[14] = height;
	dst[15] = height >> 8;
	/* 16-19 frame rate */
	dst[16] = 0xe8;
	dst[17] = 0x03;
	dst[18] = 0x00;
	dst[19] = 0x00;
	/* 20-23 time scale */
	dst[20] = 0x01;
	dst[21] = 0x00;
	dst[22] = 0x00;
	dst[23] = 0x00;
	/* 24-27 number frames */
	dst[24] = 0xdf;
	dst[25] = 0xf9;
	dst[26] = 0x09;
	dst[27] = 0x00;
	/* 28-31 reserved */
}

static void set_vp8_ivf_pichdr(unsigned char *dst, unsigned int frame_size)
{
	/*
	 * firmware just parse 64-bit timestamp(8 bytes).
	 * As not transfer timestamp to firmware, use default value(ZERO).
	 * No need to do anything here
	 */
}

static unsigned int insert_scd_pic_vp8(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned int ivf_pichdr_size = IMX_VP8_IVF_FRAME_HEADER_LEN;
	unsigned char pic_header[16] = { 0 };
	unsigned char ivf_frame_header[IMX_VP8_IVF_FRAME_HEADER_LEN] = { 0 };

	set_payload_hdr(pic_header, SCODE_NEW_PICTURE, IMX_CODEC_ID_VP8,
			ivf_pichdr_size + buffer_size, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, pic_header, 16);
	set_vp8_ivf_pichdr(ivf_frame_header, buffer_size);
	length += copy_buffer_to_stream(ctx, ivf_frame_header, ivf_pichdr_size);

	return length;
}

static unsigned int insert_scd_seq_vp8(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned int ivf_seqhdr_size = IMX_VP8_IVF_SEQ_HEADER_LEN;
	unsigned char scd_seqhdr[16] = { 0 };
	unsigned char ivf_seqhdr[IMX_VP8_IVF_SEQ_HEADER_LEN] = { 0 };
	unsigned char *src = (unsigned char *)data;

	set_payload_hdr(scd_seqhdr, SCODE_NEW_SEQUENCE, IMX_CODEC_ID_VP8,
				ivf_seqhdr_size, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, scd_seqhdr, 16);
	set_vp8_ivf_seqhdr(ivf_seqhdr, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, ivf_seqhdr, ivf_seqhdr_size);

	/* first data include frame data, need to handle them too */
	length += insert_scd_pic_vp8(ctx, buffer_size, data);
	length += copy_buffer_to_stream(ctx, src, buffer_size);

	return length;
}

static unsigned int insert_scd_pic_asp(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char pic_header[16] = { 0 };

	if (q_data->fourcc == VPU_PIX_FMT_DIV3) {
		set_payload_hdr(pic_header, SCODE_NEW_PICTURE, IMX_CODEC_ID_DIVX3,
					buffer_size, q_data->width, q_data->height);
		length += copy_buffer_to_stream(ctx, pic_header, 16);
	}

	return length;
}

static unsigned int insert_scd_seq_asp(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char seq_header[16] = {0};
	unsigned char *src = (unsigned char *)data;

	if (q_data->fourcc == VPU_PIX_FMT_DIV3) {
		set_payload_hdr(seq_header, SCODE_NEW_SEQUENCE, IMX_CODEC_ID_DIVX3,
				0, q_data->width, q_data->height);
		length += copy_buffer_to_stream(ctx, seq_header, 16);

		/* first data include frame data, need to handle them too */
		length += insert_scd_pic_asp(ctx, buffer_size, data);
		length += copy_buffer_to_stream(ctx, src, buffer_size);
	} else {
		/*
		 * other format no sequence or picture header
		 * directly copy frame data to ring buffer
		 */
		length += copy_buffer_to_stream(ctx, src, buffer_size);
	}

	return length;
}

static unsigned int insert_scd_pic_spk(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char pic_header[16] = { 0 };

	set_payload_hdr(pic_header, SCODE_NEW_PICTURE, IMX_CODEC_ID_SPK,
				buffer_size, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, pic_header, 16);

	return length;
}

static unsigned int insert_scd_seq_spk(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char seq_header[16] = {0};
	unsigned char *src = (unsigned char *)data;

	set_payload_hdr(seq_header, SCODE_NEW_SEQUENCE, IMX_CODEC_ID_SPK,
			0, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, seq_header, 16);

	/* first data include frame data, need to handle them too */
	length += insert_scd_pic_spk(ctx, buffer_size, data);
	length += copy_buffer_to_stream(ctx, src, buffer_size);

	return length;
}

static unsigned int insert_slice_arv(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char slice_header[16] = { 0 };
	unsigned int codec_id;

	if (ctx->arv_type == ARV_8)
		codec_id = IMX_CODEC_ID_ARV8;
	else
		codec_id = IMX_CODEC_ID_ARV9;

	set_payload_hdr(slice_header, SCODE_NEW_SLICE, codec_id, buffer_size,
			q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, slice_header, 16);

	return length;
}

static unsigned int insert_scd_pic_arv(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned int slice_num = 0;
	unsigned int packlen = 0;
	unsigned char pic_header[16] = { 0 };
	unsigned char *src = (unsigned char *)data;
	unsigned int codec_id;

	slice_num = ((src[16] << 24) | (src[17] << 16) | (src[18] << 8) | (src[19]));
	packlen = 20 + 8 * slice_num;
	if (ctx->arv_type == ARV_8)
		codec_id = IMX_CODEC_ID_ARV8;
	else
		codec_id = IMX_CODEC_ID_ARV9;

	set_payload_hdr(pic_header, SCODE_NEW_PICTURE, codec_id, packlen,
			q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, pic_header, 16);

	return length;
}

static unsigned int insert_scd_seq_arv(struct vpu_ctx *ctx, unsigned int buffer_size, void *data)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;
	unsigned char seq_header[16] = {0};
	unsigned int codec_id;

	if (strncmp((const char *)(data + 8), "RV30", 4) == 0) {
		ctx->arv_type = ARV_8;
		codec_id = IMX_CODEC_ID_ARV8;
	} else {
		ctx->arv_type = ARV_9;
		codec_id = IMX_CODEC_ID_ARV9;
	}

	set_payload_hdr(seq_header, SCODE_NEW_SEQUENCE, codec_id,
			buffer_size, q_data->width, q_data->height);
	length += copy_buffer_to_stream(ctx, seq_header, 16);
	length += copy_buffer_to_stream(ctx, data, buffer_size);

	return length;
}

u_int32 single_seq_info_format(struct queue_data *q_data)
{
	u_int32 ret = 0;

	switch (q_data->fourcc) {
	case V4L2_PIX_FMT_VC1_ANNEX_G:
	case V4L2_PIX_FMT_VC1_ANNEX_L:
	case VPU_PIX_FMT_RV:
	case V4L2_PIX_FMT_MPEG4:
	case V4L2_PIX_FMT_MPEG2:
	case V4L2_PIX_FMT_XVID:
		ret = 1;
		break;
	default:
		break;
	}

	return ret;
}

bool check_free_size_4_seq(struct vpu_ctx *ctx, u_int32 uPayloadSize)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u_int32 nfreespace = 0;
	u_int32 length = 0;

	switch (q_data->vdec_std) {
	case VPU_VIDEO_VC1:
		length = IMX_VC1_RCV_SEQ_HEADER_LEN + 16;
		break;
	case  VPU_VIDEO_VP6:
		length = uPayloadSize + 32;
		break;
	case VPU_VIDEO_VP8:
		length = uPayloadSize + 72;
		break;
	case VPU_VIDEO_ASP:
		length = uPayloadSize + 16;
		break;
	case VPU_VIDEO_SPK:
		length = uPayloadSize + 32;
		break;
	case VPU_VIDEO_RV:
		length = uPayloadSize + 16;
		break;
	case VPU_VIDEO_AVC:
	case VPU_VIDEO_MPEG2:
	case VPU_VIDEO_AVS:
	case VPU_VIDEO_JPEG:
	case VPU_VIDEO_AVC_MVC:
	case VPU_VIDEO_HEVC:
	case VPU_VIDEO_UNDEFINED:
		length = uPayloadSize;
		break;
	default:
		break;
	}

	pStrBufDesc = get_str_buffer_desc(ctx);
	nfreespace = got_free_space(pStrBufDesc->wptr, pStrBufDesc->rptr,
				    pStrBufDesc->start, pStrBufDesc->end);
	if (nfreespace < (length + MIN_SPACE)) {
		vpu_dbg(LVL_INFO, "buffer_full: the circular buffer freespace < buffer_size\n");
		return false;
	}

	return true;
}

bool check_free_size_pic(struct vpu_ctx *ctx, unsigned int buffer_size)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	unsigned int nfreespace = 0;
	unsigned int length = 0;

	switch (q_data->vdec_std) {
	case VPU_VIDEO_VC1:
		length = buffer_size + IMX_VC1_RCV_PIC_HEADER_LEN
			+ IMX_PAYLOAD_HEADER_SIZE;
		break;
	case VPU_VIDEO_VP8:
		length = buffer_size + IMX_VP8_IVF_FRAME_HEADER_LEN + IMX_PAYLOAD_HEADER_SIZE;
		break;
	case  VPU_VIDEO_VP6:
	case VPU_VIDEO_ASP:
	case VPU_VIDEO_SPK:
		length = buffer_size + IMX_PAYLOAD_HEADER_SIZE;
		break;
	case VPU_VIDEO_RV:
		/* speciall: buffer_size need to include all slice header size*/
		length = buffer_size + IMX_PAYLOAD_HEADER_SIZE;
		break;
	case VPU_VIDEO_AVC:
	case VPU_VIDEO_MPEG2:
	case VPU_VIDEO_AVS:
	case VPU_VIDEO_JPEG:
	case VPU_VIDEO_AVC_MVC:
	case VPU_VIDEO_HEVC:
	case VPU_VIDEO_UNDEFINED:
		length = buffer_size;
		break;
	default:
		break;
	}

	pStrBufDesc = get_str_buffer_desc(ctx);
	nfreespace = got_free_space(pStrBufDesc->wptr, pStrBufDesc->rptr,
				    pStrBufDesc->start, pStrBufDesc->end);
	if (nfreespace < (length + MIN_SPACE)) {
		vpu_dbg(LVL_INFO, "buffer_full: the circular buffer freespace < buffer_size\n");
		return false;
	}

	return true;
}

struct VPU_FMT_INFO_ARV *get_arv_info(struct vpu_ctx *ctx, u_int8 *src, u_int32 size)
{
	u_int32 i;
	struct VPU_FMT_INFO_ARV *arv_frame;

	if (!ctx || !src)
		return NULL;

	if (size < 28)
		return NULL;

	arv_frame = kzalloc(sizeof(struct VPU_FMT_INFO_ARV), GFP_KERNEL);
	if (IS_ERR_OR_NULL(arv_frame)) {
		vpu_err("%s() error: arv_frame alloc failed\n", __func__);
		goto err;
	}

	arv_frame->type = ctx->arv_type;

	arv_frame->data_len = ((src[0] << 24) | (src[1] << 16) | (src[2] << 8) | (src[3]));
	arv_frame->slice_num = ((src[16] << 24) | (src[17] << 16) | (src[18] << 8) | (src[19]));
	if (arv_frame->data_len > size || arv_frame->slice_num > size) {
		vpu_dbg(LVL_WARN, "arv frame info incorrect, data_len=%d  slice_num=%d  buffer_size=%d\n",
			arv_frame->data_len, arv_frame->slice_num, size);
		goto err;
	}

	arv_frame->slice_offset = kcalloc(arv_frame->slice_num, sizeof(u_int32), GFP_KERNEL);
	if (IS_ERR_OR_NULL(arv_frame->slice_offset)) {
		vpu_err("%s() error: slice_offset alloc failed\n", __func__);
		goto err;
	}

	for (i = 0; i < arv_frame->slice_num; i++)
		arv_frame->slice_offset[i] = ((src[20+8*i+4] << 24) | (src[20+8*i+5] << 16) | (src[20+8*i+6] << 8) | (src[20+8*i+7]));

	return arv_frame;

err:
	kfree(arv_frame->slice_offset);
	kfree(arv_frame);
	return NULL;
}

void put_arv_info(struct VPU_FMT_INFO_ARV *arv_frame)
{
	kfree(arv_frame->slice_offset);
	kfree(arv_frame);
}

static const struct imx_scd_handler handlers[] = {
	{.vdec_std = VPU_VIDEO_VC1,
	 .insert_scd_seq = insert_scd_seq_vc1,
	 .insert_scd_pic = insert_scd_pic_vc1,
	},
	{.vdec_std = VPU_VIDEO_VP6,
	 .insert_scd_seq = insert_scd_seq_vp6,
	 .insert_scd_pic = insert_scd_pic_vp6,
	},
	{.vdec_std = VPU_VIDEO_VP8,
	 .insert_scd_seq = insert_scd_seq_vp8,
	 .insert_scd_pic = insert_scd_pic_vp8,
	},
	{.vdec_std = VPU_VIDEO_ASP,
	 .insert_scd_seq = insert_scd_seq_asp,
	 .insert_scd_pic = insert_scd_pic_asp,
	},
	{.vdec_std = VPU_VIDEO_SPK,
	 .insert_scd_seq = insert_scd_seq_spk,
	 .insert_scd_pic = insert_scd_pic_spk,
	},
	{.vdec_std = VPU_VIDEO_RV,
	 .insert_scd_seq = insert_scd_seq_arv,
	 .insert_scd_pic = insert_scd_pic_arv,
	 .insert_scd_slice = insert_slice_arv,
	},
};

unsigned int insert_scode(struct vpu_ctx *ctx, unsigned int scd_type, unsigned int buffer_size, void *data)
{
	const struct imx_scd_handler *handler;
	int i = 0;
	bool found = false;
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	unsigned int length = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		handler = &handlers[i];
		if (handler->vdec_std != q_data->vdec_std)
			continue;
		found = true;
		break;
	}

	if (scd_type ==  SCODE_NEW_SEQUENCE) {
		if (!check_free_size_4_seq(ctx, buffer_size))
			return length;

		/* some format first data is frame data, and no need to add
		 * any header, directly copy it to ring buffer.
		 */
		if (found && handler->insert_scd_seq)
			length = handler->insert_scd_seq(ctx, buffer_size, data);
		else
			length = copy_buffer_to_stream(ctx, data, buffer_size);
	} else if (scd_type ==  SCODE_NEW_PICTURE) {
		if (found && handler->insert_scd_pic)
			length = handler->insert_scd_pic(ctx, buffer_size, data);
	} else if (scd_type ==  SCODE_NEW_SLICE) {
		if (found && handler->insert_scd_slice)
			length = handler->insert_scd_slice(ctx, buffer_size, data);
	} else {
		vpu_dbg(LVL_WARN, "ctx[%d] scd_type invalid\n", ctx->str_index);
	}

	return length;
}
