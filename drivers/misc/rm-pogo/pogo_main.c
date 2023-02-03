// SPDX-License-Identifier: GPL-2.0-only
/*
 * reMarkable POGO Interface Control
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

#include "pogo.h"
#include "pogo_sysfs.h"
#include "pogo_fsm.h"
#include "pogo_dr_mode.h"
#include "pogo_charging_ctrl.h"

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/kobject.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/power_supply.h>
#include <linux/suspend.h>

static int rm_pogo_init(struct rm_pogo_data *pdata)
{
	int ret = 0;

	ret = pogo_init_sysfs_nodes(pdata);
	if (ret < 0)
		return ret;

	pdata->ack_timeout = 500;
	ret = pogo_init_one_wire_mux_state(pdata);
	if (ret < 0) {
		dev_err(pdata->dev,
			"%s: Failed to initiate onewire pincontrol "
			"configuration\n",
			__func__);
		return ret;
	}

	ret = pogo_init_gpio_irq(pdata);
	if (ret < 0)
		return ret;

	ret = pogo_init_fsm(pdata);
	if (ret)
		goto err_out;

	return 0;

err_out:
	pogo_uninit_gpio_irq(pdata);
	return ret;
}

static int rm_pogo_parse_dt(struct rm_pogo_data *pdata)
{
	struct device *dev = pdata->dev;
	struct device_node *np = dev->of_node;

	dev_dbg(pdata->dev,
		"%s: Enter\n",
		__func__);

	/* TODO: handle vbus later */
	if (of_find_property(np, "vbus-supply", NULL)) {
		pdata->vbus_supply = power_supply_get_by_phandle(np,
								 "vbus-supply");
		if (IS_ERR_OR_NULL(pdata->vbus_supply)) {
			dev_dbg(pdata->dev,
				"%s: vbus supply not ready, defering probe\n",
				__func__);
			return -EPROBE_DEFER;
		}
	} else {
		dev_dbg(pdata->dev,
			"%s: Fail to find vbus-supply property!\n", __func__);
		return -EINVAL;
	}

	if (of_find_property(np, "one-wire-gpios", NULL)) {
		dev_dbg(pdata->dev,
			"%s: Found one-wire-gpio property, trying to read it\n",
			__func__);

		pdata->one_wire_gpio = devm_gpiod_get(pdata->dev,
							  "one-wire",
							  GPIOD_IN);
		if (IS_ERR(pdata->one_wire_gpio)) {
			dev_err(pdata->dev,
				"%s: Failed to read property one-wire-gpio "
				"(code %ld)\n",
				__func__,
				PTR_ERR(pdata->one_wire_gpio));
			return PTR_ERR(pdata->one_wire_gpio);
		}
	}
	else {
		dev_dbg(pdata->dev,
			"%s: Fail to find property one-wire-gpio!\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

/* call this func with pogo mutex lock held.
 *
 * Returns number of bytes written on success and less than count if
 * interrupted or error occurred. */
static int __must_check pogo_onewire_write_raw(struct rm_pogo_data *data,
				  const unsigned char *buf, size_t count,
				  bool ack_required)
{
	struct serdev_device *serdev = data->serdev;
	int ret;
	int i = 0;
	// FIXME Remove, only for debugging
	unsigned char *str_buf =
		kzalloc((ONE_WIRE_MAX_TX_MSG_SIZE * 2) + 1, GFP_KERNEL);
	for (i = 0; i < count; ++i) {
		snprintf(str_buf + (i * 2), 3, "%02hhx", buf[i]);
	}
	dev_dbg(data->dev, "%s: write complete msg: %s\n", __func__, str_buf);
	kfree(str_buf);

	dev_dbg(&serdev->dev, "%s: writing %d bytes\n", __func__, count);

	/* clear rx fifo to read new packet from MCU */
	kfifo_reset(&data->read_fifo);

	data->tx_ack_required = ack_required;

	pogo_switch_one_wire_mux_state(data,
					POGO_ONEWIRE_STATE__UART_TX);

	/* write is only buffered synchronously */
	ret = serdev_device_write(serdev, buf, count, msecs_to_jiffies(data->ack_timeout * 2));
	if (ret < 0 || ret < count)
		return ret;

	serdev_device_poll_sent(serdev, count);
	pogo_switch_one_wire_mux_state(data,
			POGO_ONEWIRE_STATE__UART_RX);

	data->ignore_next_irq = true;

	/* An ack message is expected for message to MCU */
	if (ack_required == true)
	{
		data->tx_ack_timeout = false;
		/* Queue the timeout work if not already queued, otherwise
		 * extend timer. */
		mod_delayed_work(system_power_efficient_wq,
				 &data->uart_tx_ack_watchdog_work_queue,
				 msecs_to_jiffies(data->ack_timeout));
	}

	return count;
}

/* call this func with pogo mutex lock held.
 * returns zero on success */
int __must_check pogo_onewire_write(struct rm_pogo_data *data, u8 cmd, u8 ext,
			const unsigned char *msg,
			size_t count, bool ack_required)
{
	char *buf = data->onewire_tx_buf;
	u16 checksum = 0;
	int i;

	if (msg == NULL && count > 0) {
		dev_err(data->dev,
			"%s: message length > 0, but message == NULL\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(data->dev, "Try to write cmd 0x%02x len %d ack_required %s\n", cmd,
		count, (ack_required ? "TRUE" : "FALSE"));

	if (count > ONE_WIRE_MAX_TX_MSG_SIZE) {
		dev_err(data->dev,
			"%s: failed to send msg to MCU, msg too large (%d vs %d)\n",
			__func__, count, ONE_WIRE_MAX_TX_MSG_SIZE);
		return -EINVAL;
	}

	if (0xf0 & ext) {
		dev_err(data->dev, "Extension exceed 4 bits!!");
		return -EINVAL;
	}

	/* Reset TX timeout state */
	data->tx_retried_count = 0;

	/* Follow Accessory interface description/specification:
	 * start_1 : data_len_1 : data_len_high/ext_1 : cmd_1 : checksum_from_len_1
	 */
	buf[0] = ':';		/* start magic */
	buf[1] = count & 0xff;	/* data len */
	buf[2] = ((count >> 8) & 0xf) | ext;	/* data len | ext */
	checksum += buf[1] + buf[2];
	checksum += buf[3] = cmd;

	for (i = 0; i < count; i++) {
		checksum += msg[i];
		buf[4 + i] = msg[i];
	}
	checksum = 1 + ~checksum;
	buf[4 + count] = checksum & 0xff;
	buf[5 + count] = '\r';	/* end magic */

	data->tx_data_len = 5 + count;
	i = pogo_onewire_write_raw(data, buf, data->tx_data_len, ack_required);
	if (i == 5 + count)
		return 0;
	else
		return -EIO;
}

/*
 * Handle one message. Only used in receive_buf.
 *
 * return -EINVAL on parsing error
 * return -EAGAIN when we haven't received enough bytes given packet header len
 */
static __always_inline int pogo_onewire_receive_buf_handle_message(
	struct rm_pogo_data *data, const unsigned char *buf, const size_t count)
{
	int i = 0;
	unsigned int len = 0;
	uint8_t checksum = 0;
	uint8_t rcv_check = 0;

	/* Follow Accessory interface description/specification:
	 * start_1 : data_len_1 : cmd_2 : datax : checksum_from_len_1 : end_1
	 */
	if (buf[0] != '.') { /* check magic */
		dev_warn(data->dev, "invalid magic 0x%2x\n", buf[0]);
		return -EINVAL;
	}

	if (count < ONE_WIRE_MIN_MSG_SIZE) {
		dev_warn(
			data->dev,
			"%s: %d bytes message too short, a message is at least %d bytes."
			"Waiting for more.\n",
			__func__, count, ONE_WIRE_MIN_MSG_SIZE);
		return -EAGAIN;
	}

	len = buf[1] | ((0xf & buf[2]) << 8);
	if (5 + len > count) {
		dev_warn(data->dev, "%d bytes ready, wait %d bytes\n", count,
			 5 + len);
		return -EAGAIN;
	}

	checksum = buf[1] + buf[2] + buf[3]; /* add len, cmd */

	for (i = 0; i < len; i++) {
		checksum += buf[4 + i];
	}
	rcv_check = buf[4 + len];

	checksum += rcv_check;
	if (checksum) {
		dev_warn(data->dev, "rx packet checksum error\n");
		return -EINVAL;
	}

	// SUCCESS
	return len;
}

static int pogo_onewire_receive_buf(struct serdev_device *serdev,
				const unsigned char *buf, const size_t count)
{
	struct rm_pogo_data *data = serdev_device_get_drvdata(serdev);
	int i, ret = 0;
	int len = 0;
	int bytes_processed = 0;

	const bool tx_ack_required_backup = data->tx_ack_required;

	// FIXME Remove, only for debugging
	unsigned char *str_buf =
		kzalloc((ONE_WIRE_MAX_TX_MSG_SIZE * 2) + 1, GFP_KERNEL);

	/* The tx ack timeout work might run here. To avoid the work triggering
	 * a timeout just as we might be about to/just as we have resent a
	 * message we temporarily set the tx_ack_required flag to false which
	 * should make it ignore the timeout.
	 *
	 * If we resend a message further down, we'll restart a timeout for that.
	 */
	data->tx_ack_timeout = false;
	data->tx_ack_required = false;
	cancel_delayed_work_sync(&data->uart_tx_ack_watchdog_work_queue);
	data->tx_ack_required = tx_ack_required_backup;

	if (!mutex_trylock(&data->lock)) {
		/* When FSM fall back to idle, serdev_device_close() is called.
		 * At the same time TTY core may call pogo_onewire_receive_buf()
		 * to notify pogo driver to handle just received RX data.
		 * serdev_device_close() want to hold tty lock with pogo lock held,
		 * while pogo_onewire_receive_buf() want to hold pogo lock
		 * with tty lock held.
		 */
		dev_warn(data->dev, "%s: mutex_trylock failed\n", __func__);
		return 0;
	}

	dev_dbg(data->dev, "%s: %d bytes data received.\n", __func__, count);

	for (i = 0; i < count; ++i) {
		snprintf(str_buf + (i * 2), 3, "%02hhx", buf[i]);
	}
	dev_dbg(data->dev, "%s: complete recv msg: %s\n", __func__, str_buf);
	kfree(str_buf);

	/* For debugging from user-space */
	if (count <= ONE_WIRE_MAX_TX_MSG_SIZE) {
		memset(data->onewire_rx_buf, 0, ONE_WIRE_MAX_TX_MSG_SIZE);
		memcpy(data->onewire_rx_buf, buf, count);
		data->onewire_rx_buf_len = count;
	}

	/* There may be multiple messages in the RX buffer.
	 * Parse one at a time, signal FSM thread once after.
	 */
	while (count > bytes_processed) {
		ret = pogo_onewire_receive_buf_handle_message(
			data, buf + bytes_processed, count - bytes_processed);

		if (ret == -EINVAL) {
			/* We've received a buffer we can't parse. We don't
			 * want to get the same buffer contents again, so mark
			 * all remaining as processed and drop. 
			 */
			bytes_processed = count;
			if (data->tx_ack_required)
				goto resend_unlock;
			else
				goto ignore_rx;

		} else if (ret == -EAGAIN) {
			/* We are waiting for more data. Don't mark this
			 * message as processed. */
			goto rx_unlock;
		} else if (ret < 0) {
			dev_warn(data->dev,
				 "%s: unknown error %d, ignoring RX\n",
				 __func__, ret);
			bytes_processed = count;
			goto ignore_rx;
		}

		// ret is now the payload len of the most recent packet
		len = ret;

		/* Do we have enough buffer space available? */
		if (kfifo_avail(&data->read_fifo) < (3 + len)) {
			dev_warn(
				data->dev,
				"%s: warning, receiving data faster than we can handle! overflow of %d bytes\n",
				__func__,
				(3 + len) - kfifo_avail(&data->read_fifo));
			// We abort.
			break;
		} else {
			/* send len, cmd and data to FSM layer */
			ret = kfifo_in(&data->read_fifo,
				       (buf + bytes_processed) + 1, 3 + len);
			dev_dbg(data->dev, "%s: processed %d (len %d)\n",
				__func__, 5 + len, len);

			/* Set processed to what's actually processed, so magic
			 * byte, header, payload and CRC. */
			bytes_processed += (5 + len);
		}
	}
	goto rx_unlock;

resend_unlock:
	kfifo_reset(&data->read_fifo);
	if (data->tx_retried_count++ < POGO_TX_RETRY_LIMIT) {
		// Try to resend.
		// If we fail to send what we want, trigger a timeout.
		if (pogo_onewire_write_raw(
			    data, data->onewire_tx_buf, data->tx_data_len,
			    data->tx_ack_required) != data->tx_data_len) {
			dev_warn(
				data->dev,
				"%s: resending cmd 0x%x of length %d failed, timing out.\n",
				__func__, ((uint8_t *)data->onewire_tx_buf)[3],
				data->tx_data_len);
			data->tx_ack_timeout = true;
			wake_up_process(data->fsm_thread);
		}
	} else {
		data->tx_ack_timeout = true;
		wake_up_process(data->fsm_thread);
	}

rx_unlock:
	if (bytes_processed > 0) {
		dev_dbg(data->dev,
			"%s: %d RX bytes processed. Waking up FSM...\n",
			__func__, bytes_processed);
		wake_up_process(data->fsm_thread);
	}

	mutex_unlock(&data->lock);

	/* report the consumed data so those will not be reported again */
	return bytes_processed;

ignore_rx:
	kfifo_reset(&data->read_fifo);
	mutex_unlock(&data->lock);
	return bytes_processed;
}

static const struct serdev_device_ops pogo_serdev_ops = {
	.receive_buf	= pogo_onewire_receive_buf,
	.write_wakeup	= serdev_device_write_wakeup,
};

int pogo_serdev_open(struct rm_pogo_data *data)
{
	int ret = 0;

	ret = serdev_device_open(data->serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(data->serdev, 115200);
	serdev_device_set_flow_control(data->serdev, false);
	return 0;
}

void pogo_serdev_close(struct rm_pogo_data *data)
{
	serdev_device_close(data->serdev);
}

static void pogo_uart_tx_ack_timeout_work(struct work_struct *work)
{
	struct delayed_work *delayed_work =
		container_of(work, struct delayed_work, work);

	struct rm_pogo_data *pdata =
		container_of(delayed_work, struct rm_pogo_data,
			     uart_tx_ack_watchdog_work_queue);

	dev_dbg(pdata->dev,
		"%s: MCU ack timeout. Last sent command was 0x%x \n", __func__,
		((uint8_t *)pdata->onewire_tx_buf)[3]);

	mutex_lock(&pdata->lock);
	if (pdata->tx_ack_required) {
		if (pdata->tx_retried_count++ < POGO_TX_RETRY_LIMIT) {
			/* Retry */
			dev_dbg(pdata->dev, "%s: retry \n", __func__);
			if (pogo_onewire_write_raw(pdata, pdata->onewire_tx_buf,
						   pdata->tx_data_len,
						   pdata->tx_ack_required) !=
			    pdata->tx_data_len) {
				dev_warn(
					pdata->dev,
					"%s: resending cmd 0x%x of length %d failed, timing out.\n",
					__func__,
					((uint8_t *)pdata->onewire_tx_buf)[3],
					pdata->tx_data_len);
				pdata->tx_ack_timeout = true;
				wake_up_process(pdata->fsm_thread);
			}
		} else {
			/* Actual timeout, we give up */
			dev_dbg(pdata->dev, "%s: give up \n", __func__);
			pdata->tx_ack_timeout = true;
			wake_up_process(pdata->fsm_thread);
		}
	}
	mutex_unlock(&pdata->lock);
}

static int pogo_vbus_short_notifier(struct notifier_block *this,
				    unsigned long action, void *data)
{
	struct rm_pogo_data *pdata = container_of(this,
				struct rm_pogo_data, vbus_short_nb);

	/* We cannot hold pogo lock here. Because pogo fsm may be blocked by
	 * pogo_change_otg_charger_mode_int() when max77818_set_property() is
	 * trying to get max77818 mutex while change fgcc mode. Thus a dead lock
	 * happens on racing on pogo lock. */
	pdata->vbus_short = true;
	wake_up_process(pdata->fsm_thread);
	dev_err(pdata->dev, "%s: short circuit detected!\n", __func__);
	return NOTIFY_DONE;
}

static int rm_pogo_probe(struct serdev_device *serdev)
{
	struct rm_pogo_data *pdata;
	struct device *dev = &serdev->dev;

	int ret = 0;

	pdata = devm_kzalloc(dev,
				 sizeof(struct rm_pogo_data),
				 GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->rand_secret = devm_kzalloc(dev, RAND_SECRET_LENGTH + 2, GFP_KERNEL);
	if (!pdata->rand_secret)
		return -ENOMEM;

	pdata->dev = dev;
	pdata->serdev = serdev;
	pdata->serdev_ready = false;
	mutex_init(&pdata->lock);

	serdev_device_set_drvdata(serdev, pdata);
	serdev_device_set_client_ops(serdev, &pogo_serdev_ops);

	pdata->onewire_tx_buf = devm_kzalloc(dev,
						 ONE_WIRE_MAX_TX_MSG_SIZE,
						 GFP_KERNEL);
	if (unlikely(!pdata->onewire_tx_buf)) {
		ret = -ENOMEM;
		goto error_1;
	}

	pdata->onewire_rx_buf = devm_kzalloc(dev,
						 ONE_WIRE_MAX_TX_MSG_SIZE,
						 GFP_KERNEL);
	if (unlikely(!pdata->onewire_rx_buf)) {
		ret = -ENOMEM;
		goto error_1;
	}

	pdata->pogo_name = NULL;
	ret = kfifo_alloc(&pdata->read_fifo,
			  ONE_WIRE_MCU_MSG_SIZE * NUM_MSGS_RECV_BUFFER,
			  GFP_KERNEL);
	if (ret)
		goto error_1;

	INIT_DELAYED_WORK(&pdata->uart_tx_ack_watchdog_work_queue,
			  pogo_uart_tx_ack_timeout_work);

	ret = rm_pogo_parse_dt(pdata);
	if (ret < 0) {
		if (ret == -EPROBE_DEFER) {
			dev_info(dev,
				 "%s: Defering probe due to charger driver not being"
				 "loaded/available yet\n",
				 __func__);
		}
		else {
			dev_err(dev,
				"%s: Failed to load platform data from devicetree, "
				"code %d\n",
				__func__,
				ret);
		}
		goto error_1;
	}

	pdata->vbus_short_nb.notifier_call = pogo_vbus_short_notifier;
	pdata->vbus_short_nb.priority = 255;
	register_max77818_notifier(&pdata->vbus_short_nb);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to vbus short nb, err %d\n",
			__func__, ret);
		goto error_2;
	}

	ret = rm_pogo_init(pdata);
	if (ret < 0) {
		dev_err(dev,
			"%s: Failed to init pogo, err %d\n",
			__func__, ret);
		goto error_2;
	}

	ret = device_init_wakeup(dev, true);
	if (ret < 0) {
		dev_err(dev, "%s: Failed enable wakeup, err %d\n",
			__func__, ret);
		goto error_2;
	}

	dev_info(dev,
		 "Loaded successfully !\n");
	return 0;

error_2:
	pogo_uninit_sysfs_nodes(pdata);
	pogo_uninit_gpio_irq(pdata);
	kfifo_free(&pdata->read_fifo);

error_1:
	/* No need to do explicit calls to devm_kfree */
	return ret;
}

static void rm_pogo_remove(struct serdev_device *serdev)
{
	struct rm_pogo_data *pdata = serdev_device_get_drvdata(serdev);

	unregister_max77818_notifier(&pdata->vbus_short_nb);
	dev_dbg(pdata->dev,
		"%s: Un-initialize gpio irq\n",
		__func__);
	pogo_uninit_gpio_irq(pdata);
	device_init_wakeup(pdata->dev, false);

	pogo_exit_fsm(pdata);

	dev_dbg(pdata->dev,
		"%s: Un-initializing sysfs nodes\n", __func__);

	pogo_uninit_sysfs_nodes(pdata);

	kfifo_free(&pdata->read_fifo);
}

#if defined CONFIG_PM
static int rm_pogo_suspend(struct device *dev)
{
	struct rm_pogo_data *pdata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	fsm_suspend(pdata, pm_suspend_target_state == PM_SUSPEND_STANDBY);

	if (pm_suspend_target_state == PM_SUSPEND_STANDBY &&
	    device_may_wakeup(dev)) {
		enable_irq_wake(pdata->one_wire_gpio_irq);
	}

	pdata->suspend = true;
	return 0;
}

static int rm_pogo_resume(struct device *dev)
{
	struct rm_pogo_data *pdata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	fsm_resume(pdata, pm_suspend_target_state == PM_SUSPEND_STANDBY);

	if (pm_suspend_target_state == PM_SUSPEND_STANDBY &&
	    device_may_wakeup(dev)) {
		disable_irq_wake(pdata->one_wire_gpio_irq);
	}

	pdata->suspend = false;
	return 0;
}
#else
#define rm_pogo_suspend NULL
#define rm_pogo_resume NULL
#endif

void rm_pogo_shutdown(struct device *dev)
{
	struct rm_pogo_data *pdata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s Enter:\n", __func__);

	pdata->tx_ack_required = false;
	cancel_delayed_work_sync(&pdata->uart_tx_ack_watchdog_work_queue);
	cancel_delayed_work_sync(&pdata->mcu_detach_work);
	del_timer_sync(&pdata->alive_timer);

	pogo_uninit_sysfs_nodes(pdata);
	pogo_uninit_gpio_irq(pdata);

	/* disable all energy sources to the MCU */
	pogo_change_otg_charger_mode_int(pdata, POGO_CHARGERMODE_CHARGE);
	pogo_switch_one_wire_mux_state(pdata, POGO_ONEWIRE_STATE__GPIO_100K_PD_100K_PD);

	kfifo_free(&pdata->read_fifo);
}

static struct of_device_id rm_pogo_dt_ids[] = {
	{ .compatible = "rm-pogo" },
	{ }
};
MODULE_DEVICE_TABLE(of, rm_pogo_dt_ids);

static SIMPLE_DEV_PM_OPS(rm_pogo_pm_ops,
			 rm_pogo_suspend,
			 rm_pogo_resume);

static struct serdev_device_driver rm_pogo_driver = {
	.driver = {
		.name = "rm_pogo",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &rm_pogo_pm_ops,
#endif
		.of_match_table = rm_pogo_dt_ids,
		.shutdown = rm_pogo_shutdown,
	},
	.probe = rm_pogo_probe,
	.remove = rm_pogo_remove,
};

module_serdev_device_driver(rm_pogo_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("reMarkable POGO interface control driver");
