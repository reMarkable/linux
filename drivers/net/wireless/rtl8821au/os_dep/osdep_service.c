/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/


#define _OSDEP_SERVICE_C_

#include <linux/vmalloc.h>
#include <drv_types.h>

#undef DBG_871X
static inline void DBG_871X(const char *fmt, ...)
{
}

#define RT_TAG	('1178')

u32 rtw_atoi(u8 *s)
{
	int num = 0, flag = 0;
	int i;

	for (i = 0; i <= strlen(s); i++) {
		if (s[i] >= '0' && s[i] <= '9')
			num = num * 10 + s[i] - '0';
		else if (s[0] == '-' && i == 0)
			flag = 1;
		else
			break;
	 }

		if (flag == 1)
			num = num * -1;

	 return num;

}

void *_rtw_vmalloc(u32 sz)
{
	u8 	*pbuf;
	pbuf = vmalloc(sz);

	return pbuf;
}

void *_rtw_zvmalloc(u32 sz)
{
	u8 *pbuf;

	pbuf = _rtw_vmalloc(sz);
	if (pbuf != NULL)
		memset(pbuf, 0, sz);

	return pbuf;
}

inline void _rtw_vmfree(void *pbuf)
{
	vfree(pbuf);
}

void *_rtw_malloc(u32 sz)
{
	u8 *pbuf = NULL;

	pbuf = kmalloc(sz, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

	return pbuf;
}


void *_rtw_zmalloc(u32 sz)
{
	u8 *pbuf = _rtw_malloc(sz);

	if (pbuf != NULL) {
		memset(pbuf, 0, sz);
	}

	return pbuf;
}

void	_rtw_mfree(void *pbuf)
{
	kfree(pbuf);
}


void *rtw_malloc2d(int h, int w, int size)
{
	int j;

	void **a = (void **) rtw_zmalloc(h*sizeof(void *) + h*w*size);

	if (a == NULL) {
		DBG_871X("%s: alloc memory fail!\n", __FUNCTION__);
		return NULL;
	}

	for (j = 0; j < h; j++)
		a[j] = ((char *)(a+h)) + j * w * size;

	return a;
}

void rtw_mfree2d(void *pbuf, int h, int w, int size)
{
	rtw_mfree(pbuf);
}

int _rtw_memcmp(void *dst, void *src, u32 sz)
{
	if (!(memcmp(dst, src, sz)))
		return true;
	else
		return false;
}

void _rtw_init_queue(struct __queue *pqueue)
{

	INIT_LIST_HEAD(&(pqueue->list));

	spin_lock_init(&(pqueue->lock));

}

u32 rtw_end_of_queue_search(struct list_head *head, struct list_head *plist)
{
	if (head == plist)
		return true;
	else
		return false;
}

inline u32 rtw_systime_to_ms(u32 systime)
{
	return systime * 1000 / HZ;
}

inline u32 rtw_ms_to_systime(u32 ms)
{
	return ms * HZ / 1000;
}

/*
 *  the input parameter start use the same unit as returned by jiffies
 */
inline int32_t rtw_get_passing_time_ms(u32 start)
{
	return rtw_systime_to_ms(jiffies-start);
}

inline int32_t rtw_get_time_interval_ms(u32 start, u32 end)
{
	return rtw_systime_to_ms(end-start);
}


void rtw_usleep_os(int us)
{
	if (1 < (us/1000))
		msleep(1);
	else
		msleep((us/1000) + 1);
}

void rtw_yield_os(void)
{
	yield();
}

u64 rtw_modular64(u64 x, u64 y)
{
	return do_div(x, y);
}

void rtw_buf_free(u8 **buf, u32 *buf_len)
{
	u32 ori_len;

	if (!buf || !buf_len)
		return;

	ori_len = *buf_len;

	if (*buf) {
		*buf_len = 0;
		_rtw_mfree(*buf);
		*buf = NULL;
	}
}

void rtw_buf_update(u8 **buf, u32 *buf_len, u8 *src, u32 src_len)
{
	u32 ori_len = 0, dup_len = 0;
	u8 *ori = NULL;
	u8 *dup = NULL;

	if (!buf || !buf_len)
		return;

	if (!src || !src_len)
		goto keep_ori;

	/* duplicate src */
	dup = rtw_malloc(src_len);
	if (dup) {
		dup_len = src_len;
		memcpy(dup, src, dup_len);
	}

keep_ori:
	ori = *buf;
	ori_len = *buf_len;

	/* replace buf with dup */
	*buf_len = 0;
	*buf = dup;
	*buf_len = dup_len;

	/* free ori */
	if (ori && ori_len > 0) {
	/* ULLI check usage of param ori_len */
		_rtw_mfree(ori);
	}
}

