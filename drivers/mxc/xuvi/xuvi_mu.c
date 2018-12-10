/*
 * Copyright 2020 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *
 * @file xuvi_mu.c
 *
 */

#include "xuvi_mu.h"

static void xuvi_rx_callback(struct mbox_client *c, void *msg)
{
	struct xuvi_mu_chan *mu_chan =
	    container_of(c, struct xuvi_mu_chan, cl);
	struct ppm_dev *dev = mu_chan->dev;
	static uint32_t msg_mu[4], msg_mask;
	u32 *data = msg;

	msg_mu[mu_chan->idx] = *data;
	msg_mask |= 1 << mu_chan->idx;

	if (msg_mask == 0xF) {
		msg_mask = 0;
		if (kfifo_in(&dev->msg_fifo, msg_mu, sizeof(uint32_t) * 4)
		    != sizeof(uint32_t) * 4) {
			return;
		}

		queue_work(dev->workqueue, &dev->msg_work);
	}
}

int xuvi_mu_request(struct ppm_dev *dev)
{
	struct xuvi_mu_chan *mu_chan;
	struct mbox_client *cl;
	char *chan_name;
	int ret = 0;
	int i;

	for (i = 0; i < MU_CHANNEL; i++) {
		if (i < 4)
			chan_name = kasprintf(GFP_KERNEL, "ts_tx%d", i);
		else
			chan_name =
			    kasprintf(GFP_KERNEL, "ts_rx%d", i - 4);

		if (!chan_name)
			return -ENOMEM;

		mu_chan = &dev->mu_chans[i];
		cl = &mu_chan->cl;
		cl->dev = &dev->plat_dev->dev;
		if (i == 1 || i == 2 || i == 3) {
			cl->tx_block = false;
		} else {
			cl->tx_block = true;
			cl->tx_tout = 1000;
		}
		cl->knows_txdone = false;
		cl->rx_callback = xuvi_rx_callback;

		mu_chan->dev = dev;
		mu_chan->idx = i % 4;
		mu_chan->ch = mbox_request_channel_byname(cl, chan_name);
		if (IS_ERR(mu_chan->ch)) {
			ret = PTR_ERR(mu_chan->ch);
			if (ret != -EPROBE_DEFER)
				printk
				    ("Failed to request mbox chan %s ret %d\n",
				     chan_name, ret);
			kfree(chan_name);
			return ret;
		}

		kfree(chan_name);
	}

	return ret;
}

void xuvi_mu_free(struct ppm_dev *dev)
{
	int i;

	for (i = 0; i < MU_CHANNEL; i++) {
		if (dev->mu_chans[i].ch && !IS_ERR(dev->mu_chans[i].ch))
			mbox_free_channel(dev->mu_chans[i].ch);
		dev->mu_chans[i].ch = NULL;
	}
}

void xuvi_mu_send_msg(struct ppm_dev *dev, uint32_t value0,
		      uint32_t value1, uint32_t value2, uint32_t value3)
{
	mbox_send_message(dev->mu_chans[3].ch, &value3);
	mbox_send_message(dev->mu_chans[2].ch, &value2);
	mbox_send_message(dev->mu_chans[1].ch, &value1);
	mbox_send_message(dev->mu_chans[0].ch, &value0);
}
