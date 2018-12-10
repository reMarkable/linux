/*
 * Copyright 2019-2020 NXP
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
 * @file ppm.h
 *
 * copyright here may be changed later
 *
 *
 */
#ifndef __PPM_H__
#define __PPM_H__

#include <linux/slab.h>
#include "media/demux.h"

#define CONFIG_PROC_FS               1
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/mailbox_client.h>

#include "media/dvbdev.h"
#include "media/dmxdev.h"
#include "media/demux.h"
#include "media/dvb_demux.h"
#include "media/dvb_frontend.h"
#include "media/dvb_net.h"

#define VID_API_COMMAND_LIMIT    64
#define PPM_MESSAGE_LIMIT    256
#define MSG_WORD_LENGTH 3
#define PPM_MAX_INSTANCE 2
#define PPM_TS_188		0
#define PPM_TS_204		1
#define BUF_NUM 36
#define BUF_SIZE (1*1024*1024)
#define LOG_OFFSET (2*1024*1024)
#define MSG_SIZE 4
#define MU_CHANNEL 8
#define PPM_WATCHDOG_INTERVAL_MS 1000

#define M0FW_FILENAME "imx/xuvi/vpu_fw_imx8_xuvi.bin"

enum {
	INIT_DONE = 1,
	RPC_BUF_OFFSET,
	/*PRINT_BUF_OFFSET, */
	BOOT_ADDRESS,
	COMMAND,
	EVENT,
	BUF_TO = 16,
	BUF_FROM,
	ADD_PID,
	RM_PID,
	STOP,
	HARD_FAULT,
	DRAIN_DONE,
	FLUSH_DONE,
	SNAP_SHOT
};

struct event_msg {
	u32 idx;
	u32 msgnum;
	u32 msgid;
	u32 msgdata[MSG_SIZE];
};

struct ppm_buf_list {
	struct list_head list;
	void *vir;
	u32 phy;
	u32 size;
};

struct print_buf_desc {
	u32 start_h_phy;
	u32 start_h_vir;
	u32 start_m;
	u32 bytes;
	u32 read;
	u32 write;
	char buffer[0];
};

struct xuvi_mu_chan {
	struct ppm_dev *dev;
	int idx;
	struct mbox_client cl;
	struct mbox_chan *ch;
};

struct ppm_ctx;
struct ppm_dev {
	struct device *generic_dev;
	struct video_device *pppm_decoder_dev;
	struct platform_device *plat_dev;
	struct firmware *pfw;
	void *fw_space_vir;
	u32 fw_space_phy;
	u32 boot_size;
	struct mutex dev_mutex;
	struct mutex dev_ins_mutex;
	struct mutex cmd_mutex;
	bool fw_is_ready;
	bool fw_started;
	struct completion fw_start_comp;
	struct completion snap_done_comp;
	struct delayed_work watchdog;
	struct workqueue_struct *workqueue;
	struct work_struct msg_work;
	struct clk *ppm_clk;
	struct ppm_buf_list bufs[BUF_NUM];
	struct list_head buf_list;
	struct mutex buf_list_lock;
	struct ppm_buf_list out_bufs[BUF_NUM];
	struct kfifo msg_fifo;

	struct print_buf_desc *print_buf;

	void __iomem *regs_base;
	void __iomem *csr_base;
	struct wait_queue_head buffer_wq;
	struct ppm_ctx *ctx[PPM_MAX_INSTANCE];

	bool suspend;

	struct xuvi_mu_chan mu_chans[MU_CHANNEL];
	struct device *pd_vpu;
	struct device *pd_ts;
	struct device *pd_mu;
	struct device_link *pd_vpu_link;
	struct device_link *pd_ts_link;
	struct device_link *pd_mu_link;
};

struct ppm_ctx {
	struct ppm_dev *dev;

	/*    DVB stuff               */
	struct dvb_adapter dvb_adapter;
	struct dvb_frontend *fe;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dmx_frontend fe_hw;
	struct dmx_frontend fe_mem;
	struct dvb_net dvbnet;

	int idx;
	struct kfifo msg_fifo;
	struct mutex instance_mutex;
	struct work_struct instance_work;
	struct workqueue_struct *instance_wq;
	struct completion stop_comp;
	struct completion eos_comp;
	bool start_flag;
	bool wait_abort_done;
	bool wait_rst_done;
	bool fw_stopped;
	bool stop_start;
	bool fw_finished;
	bool eos_stop_received;
	bool eos_stop_added;
	bool ctx_released;
	bool hang_status;
	bool instance_activated;
	u8 feeds;
};

#endif				// __PPM_H__ //
