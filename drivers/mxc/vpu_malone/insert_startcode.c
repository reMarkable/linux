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

static void insert_payload_header_vc1(u_int8 *dst, u_int32 uScodeType, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = uScodeType;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = IMX_VC1_RCV_CODEC_V1_VERSION;
	dst[9] = IMX_CODEC_VERSION_ID;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
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

void vp6_scd_sequence_header(unsigned char *buffer, int pic_width, int pic_height)
{
	int Length = 0;

	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x31;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}

void vp6_scd_frame_header(unsigned char *buffer, int pic_width, int pic_height, int Length)
{
	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x32;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}

void vp8_ivf_sequence_header(unsigned char *buffer, int pic_width, int pic_height)
{
	int Length = 32;

	buffer[0] = 0x44;
	buffer[1] = 0x4b;
	buffer[2] = 0x49;
	buffer[3] = 0x46; //0-3byte signature "DKIF"
	buffer[4] = 0x00;
	buffer[5] = 0x00; //4-5byte version 0
	buffer[6] = Length;
	buffer[7] = Length >> 8; //length of Header
	buffer[8] = 0x56;
	buffer[9] = 0x50;
	buffer[10] = 0x38;
	buffer[11] = 0x30; //VP8 fourcc
	buffer[12] = pic_width;
	buffer[13] = pic_width >> 8;
	buffer[14] = pic_height;
	buffer[15] = pic_height >> 8;
	buffer[16] = 0xe8;
	buffer[17] = 0x03;
	buffer[18] = 0x00;
	buffer[19] = 0x00; //16-19 frame rate
	buffer[20] = 0x01;
	buffer[21] = 0x00;
	buffer[22] = 0x00;
	buffer[23] = 0x00; //20-23 time scale
	buffer[24] = 0xdf;
	buffer[25] = 0xf9;
	buffer[26] = 0x09;
	buffer[27] = 0x00; //24-27 number frames
	//28-31 unused
}

void vp8_ivf_frame_header(unsigned char *buffer, u_int32 FrameSize)
{
	buffer[0] = FrameSize;
	buffer[1] = FrameSize >> 8;
	buffer[2] = FrameSize >> 16;
	buffer[3] = FrameSize >> 24;
	//4-11 timestamp
}

void vp8_scd_sequence_header(unsigned char *buffer, int pic_width, int pic_height)
{
	int Length = 32;

	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x31;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}
void vp8_scd_frame_header(unsigned char *buffer, int pic_width, int pic_height, int Length)
{
	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x32;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}

static void insert_payload_header_divx(u_int8 *dst, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = 0x32;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = 0x38;
	dst[9] = 0x01;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
}

static void insert_seq_header_spk(u_int8 *dst, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = 0x31;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = 0x39;
	dst[9] = 0x01;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
}

static void insert_frame_header_spk(u_int8 *dst, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	uPayloadSize = 0;
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = 0x32;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = 0x39;
	dst[9] = 0x01;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
}

void insert_payload_header_arv(u_int8 *dst, u_int32 uScodeType,
	enum ARV_FRAME_TYPE type, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = uScodeType;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	if (type == ARV_8)
		dst[8] = 0x28;
	else
		dst[8] = 0x29;
	dst[9] = 0x01;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
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

u_int32 insert_scode_4_seq(struct vpu_ctx *ctx, u_int8 *src, u_int32 uPayloadSize)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	u_int32 length = 0;

	if (!check_free_size_4_seq(ctx, uPayloadSize))
		return 0;

	switch (q_data->vdec_std) {
	case VPU_VIDEO_VC1: {
		if (q_data->fourcc == V4L2_PIX_FMT_VC1_ANNEX_G) {
			u_int8 payload_header[32] = {0};
			u_int8 rcv_seqhdr[IMX_VC1_RCV_SEQ_HEADER_LEN] = {0};
			unsigned int rvc_seqhdr_size = IMX_VC1_RCV_SEQ_HEADER_LEN;

			insert_payload_header_vc1(payload_header, SCODE_NEW_SEQUENCE,
					20+12,  q_data->width, q_data->height);
			copy_buffer_to_stream(ctx, payload_header, 16);
			length = 16;
			set_vc1_rcv_seqhdr(rcv_seqhdr, src, q_data->width, q_data->height);
			copy_buffer_to_stream(ctx, rcv_seqhdr, rvc_seqhdr_size);
			length += rvc_seqhdr_size;
		} else {
			length += copy_buffer_to_stream(ctx, src, uPayloadSize);
		}
	}
	break;
	case VPU_VIDEO_VP6: {
		u_int8 seq_header[16] = {0};
		u_int8 frame_header[16] = {0};

		vp6_scd_sequence_header(seq_header, q_data->width, q_data->height);
		copy_buffer_to_stream(ctx, seq_header, 16);
		length = 16;
		vp6_scd_frame_header(frame_header, q_data->width, q_data->height, uPayloadSize);
		copy_buffer_to_stream(ctx, frame_header, 16);
		length += 16;
		copy_buffer_to_stream(ctx, src, uPayloadSize);
		length += uPayloadSize;
	}
	break;
	case VPU_VIDEO_VP8: {
		u_int8 scd_seq_header[16] = {0};
		u_int8 ivf_seq_header[32] = {0};
		u_int8 scd_frame_header[16] = {0};
		u_int8 ivf_frame_header[8] = {0};

		vp8_scd_sequence_header(scd_seq_header, q_data->width, q_data->height);
		copy_buffer_to_stream(ctx, scd_seq_header, 16);
		length = 16;
		vp8_ivf_sequence_header(ivf_seq_header, q_data->width, q_data->height);
		copy_buffer_to_stream(ctx, ivf_seq_header, 32);
		length += 32;
		vp8_scd_frame_header(scd_frame_header, q_data->width, q_data->height, uPayloadSize + 8);
		copy_buffer_to_stream(ctx, scd_frame_header, 16);
		length += 16;
		vp8_ivf_frame_header(ivf_frame_header, uPayloadSize);
		copy_buffer_to_stream(ctx, ivf_frame_header, 8);
		length += 8;
		copy_buffer_to_stream(ctx, src, uPayloadSize);
		length += uPayloadSize;
	}
	break;
	case VPU_VIDEO_ASP: {
		if (q_data->fourcc == VPU_PIX_FMT_DIV3) {
			u_int8 seq_header[16] = {0};

			insert_payload_header_divx(seq_header, uPayloadSize, q_data->width, q_data->height);
			copy_buffer_to_stream(ctx, seq_header, 16);
			length = 16;
			copy_buffer_to_stream(ctx, src, uPayloadSize);
			length += uPayloadSize;
		} else {
			copy_buffer_to_stream(ctx, src, uPayloadSize);
			length = uPayloadSize;
		}
	}
	break;
	case VPU_VIDEO_SPK: {
		u_int8 seq_header[16] = {0};
		u_int8 frame_header[16] = {0};

		insert_seq_header_spk(seq_header, 0, q_data->width, q_data->height);
		copy_buffer_to_stream(ctx, seq_header, 16);
		length = 16;
		insert_frame_header_spk(frame_header, uPayloadSize, q_data->width, q_data->height);
		copy_buffer_to_stream(ctx, frame_header, 16);
		length += 16;
		copy_buffer_to_stream(ctx, src, uPayloadSize);
		length += uPayloadSize;
	}
	break;
	case VPU_VIDEO_RV: {
		u_int8 seq_header[16] = {0};

		if (strncmp((const char *)(src+8), "RV30", 4) == 0)
			ctx->arv_type = ARV_8;
		else
			ctx->arv_type = ARV_9;

		insert_payload_header_arv(seq_header, SCODE_NEW_SEQUENCE, ctx->arv_type, uPayloadSize + 12, q_data->width, q_data->height);
		copy_buffer_to_stream(ctx, seq_header, 16);
		length = 16;
		copy_buffer_to_stream(ctx, src, uPayloadSize);
		length += uPayloadSize;
	}
	break;
	case VPU_VIDEO_AVC:
	case VPU_VIDEO_MPEG2:
	case VPU_VIDEO_AVS:
	case VPU_VIDEO_JPEG:
	case VPU_VIDEO_AVC_MVC:
	case VPU_VIDEO_HEVC:
	case VPU_VIDEO_UNDEFINED: {
		copy_buffer_to_stream(ctx, src, uPayloadSize);
		length = uPayloadSize;
	}
	break;
	default:
		break;
	}
	return length;
}

u_int32 insert_scode_4_pic(struct vpu_ctx *ctx, u_int8 *dst, u_int8 *src, u_int32 vdec_std, u_int32 uPayloadSize)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	u_int32 length = 0;

	switch (vdec_std) {
	case VPU_VIDEO_VC1: {
		if (q_data->fourcc == V4L2_PIX_FMT_VC1_ANNEX_G) {
			u_int8 rcv_pichdr[IMX_VC1_RCV_PIC_HEADER_LEN];
			unsigned int rcv_pichdr_size = IMX_VC1_RCV_PIC_HEADER_LEN;

			insert_payload_header_vc1(dst, SCODE_NEW_PICTURE, uPayloadSize + 16, q_data->width, q_data->height);
			set_vc1_rcv_pichdr(rcv_pichdr, uPayloadSize);
			memcpy(dst+16, rcv_pichdr, rcv_pichdr_size);
			length = 16 + rcv_pichdr_size;
		} else {
			unsigned char nal_hdr[IMX_VC1_NAL_HEADER_LEN] = { 0 };
			unsigned int len;

			len = create_vc1_nal_pichdr(nal_hdr, src);
			memcpy(dst, nal_hdr, len);
			length = len;
		}
	}
	break;
	case VPU_VIDEO_VP6: {
		vp6_scd_frame_header(dst, q_data->width, q_data->height, uPayloadSize);
		length = 16;
	}
	break;
	case VPU_VIDEO_VP8: {
		u_int8 frame_header[8];

		vp8_scd_frame_header(dst, q_data->width, q_data->height, uPayloadSize + 8);
		length = 16;
		vp8_ivf_frame_header(frame_header, uPayloadSize);
		memcpy(dst+length, frame_header, 8);
		length += 8;
	}
	break;
	case VPU_VIDEO_ASP: {
		if (q_data->fourcc == VPU_PIX_FMT_DIV3) {
			insert_payload_header_divx(dst, uPayloadSize, q_data->width, q_data->height);
			length = 16;
		}
	}
	break;
	case VPU_VIDEO_SPK: {
		insert_frame_header_spk(dst, uPayloadSize, q_data->width, q_data->height);
		length = 16;
	}
	break;
	case VPU_VIDEO_RV: {
		u_int32 slice_num;
		u_int32 packlen;

		slice_num = ((src[16] << 24) | (src[17] << 16) | (src[18] << 8) | (src[19]));
		packlen = 20 + 8 * slice_num;
		insert_payload_header_arv(dst, SCODE_NEW_PICTURE, ctx->arv_type, packlen + 12, q_data->width, q_data->height);
		length = 16;
	}
	break;
	default:
	break;
	}
	return length;
}

u_int32 insert_scode_4_arv_slice(struct vpu_ctx *ctx, u_int8 *dst, struct VPU_FMT_INFO_ARV *arv_frame, u_int32 uPayloadSize)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	u_int32 length = 0;

	insert_payload_header_arv(dst, SCODE_NEW_SLICE, arv_frame->type, uPayloadSize, q_data->width, q_data->height);
	length = 16;

	return length;
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

