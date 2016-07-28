/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
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

#ifndef _RTW_IO_H_
#define _RTW_IO_H_

/* ULLI : must remove this ?? */

#define rtw_usb_buffer_alloc(dev, size, dma) usb_alloc_coherent((dev), (size), (in_interrupt() ? GFP_ATOMIC : GFP_KERNEL), (dma))
#define rtw_usb_buffer_free(dev, size, addr, dma) usb_free_coherent((dev), (size), (addr), (dma))

#define NUM_IOREQ		8

#define MAX_PROT_SZ	(64-16)

#define _IOREADY			0
#define _IO_WAIT_COMPLETE   1
#define _IO_WAIT_RSP        2

// IO COMMAND TYPE
#define _IOSZ_MASK_		(0x7F)
#define _IO_WRITE_		BIT(7)
#define _IO_FIXED_		BIT(8)
#define _IO_BURST_		BIT(9)
#define _IO_BYTE_		BIT(10)
#define _IO_HW_			BIT(11)
#define _IO_WORD_		BIT(12)
#define _IO_SYNC_		BIT(13)
#define _IO_CMDMASK_	(0x1F80)


/*
	For prompt mode accessing, caller shall free io_req
	Otherwise, io_handler will free io_req
*/



// IO STATUS TYPE
#define _IO_ERR_		BIT(2)
#define _IO_SUCCESS_	BIT(1)
#define _IO_DONE_		BIT(0)


#define IO_RD32			(_IO_SYNC_ | _IO_WORD_)
#define IO_RD16			(_IO_SYNC_ | _IO_HW_)
#define IO_RD8			(_IO_SYNC_ | _IO_BYTE_)

#define IO_RD32_ASYNC	(_IO_WORD_)
#define IO_RD16_ASYNC	(_IO_HW_)
#define IO_RD8_ASYNC	(_IO_BYTE_)

#define IO_WR32			(_IO_WRITE_ | _IO_SYNC_ | _IO_WORD_)
#define IO_WR16			(_IO_WRITE_ | _IO_SYNC_ | _IO_HW_)
#define IO_WR8			(_IO_WRITE_ | _IO_SYNC_ | _IO_BYTE_)

#define IO_WR32_ASYNC	(_IO_WRITE_ | _IO_WORD_)
#define IO_WR16_ASYNC	(_IO_WRITE_ | _IO_HW_)
#define IO_WR8_ASYNC	(_IO_WRITE_ | _IO_BYTE_)

/*

	Only Sync. burst accessing is provided.

*/

#define IO_WR_BURST(x)		(_IO_WRITE_ | _IO_SYNC_ | _IO_BURST_ | ( (x) & _IOSZ_MASK_))
#define IO_RD_BURST(x)		(_IO_SYNC_ | _IO_BURST_ | ( (x) & _IOSZ_MASK_))



//below is for the intf_option bit defition...

#define _INTF_ASYNC_	BIT(0)	//support async io

struct intf_priv;
struct intf_hdl;
struct io_queue;

struct io_req {
	struct list_head	list;
	u32	addr;
	volatile u32	val;
	u32	command;
	u32	status;
	uint8_t	*pbuf;
	struct semaphore	sema;


	void (*_async_io_callback)(struct rtl_priv *padater, struct io_req *pio_req, uint8_t *cnxt);
	uint8_t *cnxt;



};

struct reg_protocol_rd {

#ifdef __LITTLE_ENDIAN

	//DW1
	u32		NumOfTrans:4;
	u32		Reserved1:4;
	u32		Reserved2:24;
	//DW2
	u32		ByteCount:7;
	u32		WriteEnable:1;		//0:read, 1:write
	u32		FixOrContinuous:1;	//0:continuous, 1: Fix
	u32		BurstMode:1;
	u32		Byte1Access:1;
	u32		Byte2Access:1;
	u32		Byte4Access:1;
	u32		Reserved3:3;
	u32		Reserved4:16;
	//DW3
	u32		BusAddress;
	//DW4
	//u32		Value;
#else


//DW1
	u32 Reserved1  :4;
	u32 NumOfTrans :4;

	u32 Reserved2  :24;

	//DW2
	u32 WriteEnable : 1;
	u32 ByteCount :7;


	u32 Reserved3 : 3;
	u32 Byte4Access : 1;

	u32 Byte2Access : 1;
	u32 Byte1Access : 1;
	u32 BurstMode :1 ;
	u32 FixOrContinuous : 1;

	u32 Reserved4 : 16;

	//DW3
	u32		BusAddress;

	//DW4
	//u32		Value;

#endif

};


struct reg_protocol_wt {


#ifdef __LITTLE_ENDIAN

	//DW1
	u32		NumOfTrans:4;
	u32		Reserved1:4;
	u32		Reserved2:24;
	//DW2
	u32		ByteCount:7;
	u32		WriteEnable:1;		//0:read, 1:write
	u32		FixOrContinuous:1;	//0:continuous, 1: Fix
	u32		BurstMode:1;
	u32		Byte1Access:1;
	u32		Byte2Access:1;
	u32		Byte4Access:1;
	u32		Reserved3:3;
	u32		Reserved4:16;
	//DW3
	u32		BusAddress;
	//DW4
	u32		Value;

#else
	//DW1
	u32 Reserved1  :4;
	u32 NumOfTrans :4;

	u32 Reserved2  :24;

	//DW2
	u32 WriteEnable : 1;
	u32 ByteCount :7;

	u32 Reserved3 : 3;
	u32 Byte4Access : 1;

	u32 Byte2Access : 1;
	u32 Byte1Access : 1;
	u32 BurstMode :1 ;
	u32 FixOrContinuous : 1;

	u32 Reserved4 : 16;

	//DW3
	u32		BusAddress;

	//DW4
	u32		Value;

#endif

};



/*
Below is the data structure used by _io_handler

*/

extern uint ioreq_flush(struct rtl_priv *rtlpriv, struct io_queue *ioqueue);
extern void sync_ioreq_enqueue(struct io_req *preq,struct io_queue *ioqueue);
extern uint sync_ioreq_flush(struct rtl_priv *rtlpriv, struct io_queue *ioqueue);


extern uint free_ioreq(struct io_req *preq, struct io_queue *pio_queue);
extern struct io_req *alloc_ioreq(struct io_queue *pio_q);

extern uint register_intf_hdl(uint8_t *dev, struct intf_hdl *pintfhdl);
extern void unregister_intf_hdl(struct intf_hdl *pintfhdl);

extern void _rtw_attrib_read(struct rtl_priv *rtlpriv, u32 addr, u32 cnt, uint8_t *pmem);
extern void _rtw_attrib_write(struct rtl_priv *rtlpriv, u32 addr, u32 cnt, uint8_t *pmem);

extern void rtw_write_scsi(struct rtl_priv *rtlpriv, u32 cnt, uint8_t *pmem);

//ioreq
extern void ioreq_read8(struct rtl_priv *rtlpriv, u32 addr, uint8_t *pval);
extern void ioreq_read16(struct rtl_priv *rtlpriv, u32 addr, u16 *pval);
extern void ioreq_read32(struct rtl_priv *rtlpriv, u32 addr, u32 *pval);
extern void ioreq_write8(struct rtl_priv *rtlpriv, u32 addr, uint8_t val);
extern void ioreq_write16(struct rtl_priv *rtlpriv, u32 addr, u16 val);
extern void ioreq_write32(struct rtl_priv *rtlpriv, u32 addr, u32 val);


extern uint async_read8(struct rtl_priv *rtlpriv, u32 addr, uint8_t *pbuff,
	void (*_async_io_callback)(struct rtl_priv *padater, struct io_req *pio_req, uint8_t *cnxt), uint8_t *cnxt);
extern uint async_read16(struct rtl_priv *rtlpriv, u32 addr,  uint8_t *pbuff,
	void (*_async_io_callback)(struct rtl_priv *padater, struct io_req *pio_req, uint8_t *cnxt), uint8_t *cnxt);
extern uint async_read32(struct rtl_priv *rtlpriv, u32 addr,  uint8_t *pbuff,
	void (*_async_io_callback)(struct rtl_priv *padater, struct io_req *pio_req, uint8_t *cnxt), uint8_t *cnxt);

extern void async_read_mem(struct rtl_priv *rtlpriv, u32 addr, u32 cnt, uint8_t *pmem);
extern void async_read_port(struct rtl_priv *rtlpriv, u32 addr, u32 cnt, uint8_t *pmem);

extern void async_write8(struct rtl_priv *rtlpriv, u32 addr, uint8_t val,
	void (*_async_io_callback)(struct rtl_priv *padater, struct io_req *pio_req, uint8_t *cnxt), uint8_t *cnxt);
extern void async_write16(struct rtl_priv *rtlpriv, u32 addr, u16 val,
	void (*_async_io_callback)(struct rtl_priv *padater, struct io_req *pio_req, uint8_t *cnxt), uint8_t *cnxt);
extern void async_write32(struct rtl_priv *rtlpriv, u32 addr, u32 val,
	void (*_async_io_callback)(struct rtl_priv *padater, struct io_req *pio_req, uint8_t *cnxt), uint8_t *cnxt);

extern void async_write_mem(struct rtl_priv *rtlpriv, u32 addr, u32 cnt, uint8_t *pmem);
extern void async_write_port(struct rtl_priv *rtlpriv, u32 addr, u32 cnt, uint8_t *pmem);

extern uint alloc_io_queue(struct rtl_priv *rtlpriv);
extern void free_io_queue(struct rtl_priv *rtlpriv);
extern void async_bus_io(struct io_queue *pio_q);
extern void bus_sync_io(struct io_queue *pio_q);
extern u32 _ioreq2rwmem(struct io_queue *pio_q);
extern void dev_power_down(struct rtl_priv * rtlpriv, uint8_t bpwrup);

/*
#define RTL_R8(reg)		rtw_read8(rtlpriv, reg)
#define RTL_R16(reg)            rtl_read_word(rtlpriv, reg)
#define RTL_R32(reg)            rtl_read_dword(rtlpriv, reg)
#define RTL_W8(reg, val8)       rtl_write_byte(rtlpriv, reg, val8)
#define RTL_W16(reg, val16)     rtl_write_word(rtlpriv, reg, val16)
#define RTL_W32(reg, val32)     rtl_write_dword(rtlpriv, reg, val32)
*/

/*
#define RTL_W8_ASYNC(reg, val8) rtw_write32_async(rtlpriv, reg, val8)
#define RTL_W16_ASYNC(reg, val16) rtw_write32_async(rtlpriv, reg, val16)
#define RTL_W32_ASYNC(reg, val32) rtw_write32_async(rtlpriv, reg, val32)

#define RTL_WRITE_BB(reg, val32)	phy_SetUsbBBReg(rtlpriv, reg, val32)
#define RTL_READ_BB(reg)	phy_QueryUsbBBReg(rtlpriv, reg)
*/


#endif	//_RTL8711_IO_H_

