// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2013 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <asm/unaligned.h>
#include <linux/kernel.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

// Bitmasks (for data[3])
#define WACOM_TIP_SWITCH_bm         (1 << 0)
#define WACOM_BARREL_SWITCH_bm      (1 << 1)
#define WACOM_ERASER_bm             (1 << 2)
#define WACOM_INVERT_bm             (1 << 3)

// Registers
#define WACOM_COMMAND_LSB   0x04
#define WACOM_COMMAND_MSB   0x00

#define WACOM_DATA_LSB      0x05
#define WACOM_DATA_MSB      0x00

// Report types
#define REPORT_INPUT        0x10
#define REPORT_OUTPUT       0x20
#define REPORT_FEATURE      0x30

// Requests / operations
#define OPCODE_RESET        0x01
#define OPCODE_GET_REPORT   0x02
#define OPCODE_SET_REPORT   0x03
#define OPCODE_SET_POWER    0x08

// Power settings
#define POWER_ON            0x00
#define POWER_SLEEP         0x01

// Input report ids
#define WACOM_PEN_DATA_REPORT           2
#define WACOM_SHINONOME_REPORT          26

// Feature report ids
#define WACOM_DEVICE_MODE_REPORT        2
#define WACOM_QUERY_REPORT              3
#define WACOM_PEN_INPUT_FORMAT_REPORT   9
#define WACOM_POSITION_RATE_REPORT      25
#define WACOM_ORIGIN_OFFSET_REPORT      22
#define WACOM_SIDE_SWITCH_REPORT        21

#define WACOM_MAX_DATA_SIZE     22

struct wacom_features {
	int x_max;
	int y_max;
	int pressure_max;
	int distance_max;
	int distance_physical_max;
	int tilt_x_max;
	int tilt_y_max;
	char fw_version;
};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input;
	struct wacom_features features;
	struct regulator *vdd;
	u8 data[WACOM_MAX_DATA_SIZE];
	bool prox;
	int tool;

	bool flip_tilt_x;
	bool flip_tilt_y;
	bool flip_pos_x;
	bool flip_pos_y;
	bool flip_distance;
	bool flip_pressure;
};

u8 reset_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		0x00,
		OPCODE_RESET,
};

u8 wakeup_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		POWER_ON,
		OPCODE_SET_POWER,
};

u8 sleep_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		POWER_SLEEP,
		OPCODE_SET_POWER,
};

static int wacom_query_device(struct i2c_client *client,
				  struct wacom_features *features)
{
	int ret;
	u8 data[WACOM_MAX_DATA_SIZE];
	struct reset_control *rstc;

	u8 get_query_data_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		REPORT_FEATURE | WACOM_QUERY_REPORT,
		OPCODE_GET_REPORT,
		WACOM_DATA_LSB,
		WACOM_DATA_MSB,
	};

	struct i2c_msg msgs[] = {
		// Request reading of feature ReportID: 3 (Pen Query Data)
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(get_query_data_cmd),
			.buf = get_query_data_cmd,
		},
		// Read 21 bytes
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 21,
			.buf = data,
		},
	};

	rstc = devm_reset_control_get_optional_exclusive(&client->dev, NULL);
	if (IS_ERR(rstc)) {
		dev_err(&client->dev, "Failed to get reset control before init\n");
	}
	else {
		reset_control_reset(rstc);
	}

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		return -EIO;
	}

	features->x_max = get_unaligned_le16(&data[3]);
	features->y_max = get_unaligned_le16(&data[5]);
	features->pressure_max = get_unaligned_le16(&data[11]);
	features->fw_version = get_unaligned_le16(&data[13]);
	features->distance_max = data[15];
	features->distance_physical_max = data[16];
	features->tilt_x_max = get_unaligned_le16(&data[17]);
	features->tilt_y_max = get_unaligned_le16(&data[19]);

	dev_dbg(&client->dev,
		"x_max:%d, y_max:%d, pressure:%d, fw:%d, "
		"distance: %d, phys distance: %d"
		"tilt_x_max: %d, tilt_y_max: %d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version,
		features->distance_max, features->distance_physical_max,
		features->tilt_x_max, features->tilt_y_max);

	return 0;
}

#ifdef CONFIG_OF
static void wacom_of_read(struct wacom_i2c *wac_i2c)
{
	struct i2c_client *client = wac_i2c->client;

	wac_i2c->flip_tilt_x = of_property_read_bool(client->dev.of_node, "flip-tilt-x");
	wac_i2c->flip_tilt_y = of_property_read_bool(client->dev.of_node, "flip-tilt-y");
	wac_i2c->flip_pos_x = of_property_read_bool(client->dev.of_node, "flip-pos-x");
	wac_i2c->flip_pos_y = of_property_read_bool(client->dev.of_node, "flip-pos-y");
	wac_i2c->flip_distance = of_property_read_bool(client->dev.of_node, "flip-distance");
	wac_i2c->flip_pressure = of_property_read_bool(client->dev.of_node, "flip-pressure");
}
#endif

static int wacom_setup_device(struct i2c_client *client)
{
	int ret;
	u8 data[WACOM_MAX_DATA_SIZE];

	u8 get_sample_rate_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		REPORT_FEATURE | 0xF,
		OPCODE_GET_REPORT,
		WACOM_POSITION_RATE_REPORT,
		WACOM_DATA_LSB,
		WACOM_DATA_MSB,
	};

	u8 set_sample_rate_cmd[] = {
		WACOM_COMMAND_LSB,
		WACOM_COMMAND_MSB,
		REPORT_FEATURE | 0xF,
		OPCODE_SET_REPORT,
		WACOM_POSITION_RATE_REPORT,
		WACOM_DATA_LSB,
		WACOM_DATA_MSB,
		0x04, // LENGTH LSB
		0x00, // LENGTH MSB
		WACOM_POSITION_RATE_REPORT,
		0x06, // Selected rate (1=240pps, 3=360pps (def), 5=480pps, 6=540pps)
	};

	struct i2c_msg msgs[] = {
		// Request writing of feature, ReportID 25 / Position Report Rate
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(set_sample_rate_cmd),
			.buf = set_sample_rate_cmd,
		},
	};

	struct i2c_msg msgs_readback[] = {
		// Request reading of feature, ReportID 25 / Position Report Rate
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(get_sample_rate_cmd),
			.buf = get_sample_rate_cmd,
		},
		// Read 4 bytes
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 4,
			.buf = data,
		},
	};

	// Write position rate report
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		return -EIO;
	}

	// Read position rate report
	ret = i2c_transfer(client->adapter, msgs_readback, ARRAY_SIZE(msgs_readback));
	if (ret < 0) {
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs_readback)) {
		return -EIO;
	}

	return 0;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	ktime_t timestamp = ktime_get();

	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	struct wacom_features *features = &wac_i2c->features;
	u8 *data = wac_i2c->data;
	unsigned int x, y, pressure;
	unsigned char tip, f1, f2, eraser, transducer = 0;
	short tilt_x, tilt_y, distance;
	int error;

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, 17);
	if (error < 0) {
		goto out;
	}

	// data[0] == Length LSB
	// data[1] == Length MSB
	// data[2] == ReportID (2 | 26)
	tip = data[3] & WACOM_TIP_SWITCH_bm;
	eraser = data[3] & WACOM_ERASER_bm;
	f1 = data[3] & WACOM_BARREL_SWITCH_bm;
	f2 = data[3] & WACOM_INVERT_bm;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);

	// Shinonome Refill has a transducer index field:
	if (data[2] == WACOM_SHINONOME_REPORT) {
		transducer = (data[3] >> 6);
	}

	// Tilt (signed)
	tilt_x = le16_to_cpup((__le16 *)&data[11]);
	tilt_y = le16_to_cpup((__le16 *)&data[13]);

	// Hover height
	distance = le16_to_cpup((__le16 *)&data[15]);

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = (data[3] & 0x20) ? true : false;

	// Flippin'
	pressure = wac_i2c->flip_pressure ? (features->pressure_max - pressure) : pressure;
	distance = wac_i2c->flip_distance ? -distance : distance;
	x = wac_i2c->flip_pos_x ? (features->x_max - x) : x;
	y = wac_i2c->flip_pos_y ? (features->y_max - y) : y;
	tilt_x = wac_i2c->flip_tilt_x ? -tilt_x : tilt_x;
	tilt_y = wac_i2c->flip_tilt_y ? -tilt_y : tilt_y;

	input_report_key(input, BTN_TOUCH, tip || eraser);
	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_abs(input, ABS_DISTANCE, distance);
	input_report_abs(input, ABS_TILT_X, tilt_x);
	input_report_abs(input, ABS_TILT_Y, tilt_y);
	input_set_timestamp(input, timestamp);
	input_sync(input);

out:
	return IRQ_HANDLED;
}

static int wacom_i2c_open(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	enable_irq(client->irq);

	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	struct wacom_i2c *wac_i2c = input_get_drvdata(dev);
	struct i2c_client *client = wac_i2c->client;

	disable_irq(client->irq);
}

static int wacom_i2c_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	struct wacom_features *features;
	struct input_dev *input;
	int error;

	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	if (!wac_i2c) {
		return -ENOMEM;
	}
	features = &wac_i2c->features;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	wac_i2c->vdd = regulator_get(&client->dev, "vdd");
	if (IS_ERR(wac_i2c->vdd)) {
		error = PTR_ERR(wac_i2c->vdd);
		goto err_free_mem;
	}

	error = regulator_enable(wac_i2c->vdd);
	if (error)
		goto err_put_vdd;

	error = wacom_query_device(client, features);
	if (error)
		goto err_disable_vdd;

	error = wacom_setup_device(client);
	if (error)
		goto err_disable_vdd;

	input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto err_disable_vdd;
	}

	wac_i2c->client = client;
	wac_i2c->input = input;

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = features->fw_version;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, features->x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, features->y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
				 0, features->pressure_max, 0, 0);

	input_set_abs_params(input, ABS_DISTANCE, 0, features->distance_max, 0, 0);
	input_set_abs_params(input, ABS_TILT_X, -features->tilt_x_max, features->tilt_x_max, 0, 0);
	input_set_abs_params(input, ABS_TILT_Y, -features->tilt_y_max, features->tilt_y_max, 0, 0);

	input_set_drvdata(input, wac_i2c);

	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
					 IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					 "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_moremem;
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	disable_irq(client->irq);

	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	i2c_set_clientdata(client, wac_i2c);

#ifdef CONFIG_OF
	wacom_of_read(wac_i2c);
#endif

	device_init_wakeup(&client->dev, true);

	return 0;

err_free_irq:
	free_irq(client->irq, wac_i2c);
err_free_moremem:
	input_free_device(input);
err_disable_vdd:
	regulator_disable(wac_i2c->vdd);
err_put_vdd:
	regulator_put(wac_i2c->vdd);
err_free_mem:
	kfree(wac_i2c);

	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input);
	regulator_disable(wac_i2c->vdd);
	regulator_put(wac_i2c->vdd);
	kfree(wac_i2c);

	return 0;
}

static int __maybe_unused wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	if (pm_suspend_target_state == PM_SUSPEND_MEM) {
		regulator_disable(wac_i2c->vdd);
		pinctrl_pm_select_sleep_state(dev);
	}

	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);
	disable_irq(client->irq);

	return 0;
}

static int __maybe_unused wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	int ret;

	enable_irq(client->irq);
	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	if (pm_suspend_target_state == PM_SUSPEND_MEM) {
		pinctrl_pm_select_default_state(dev);

		ret = regulator_enable(wac_i2c->vdd);
		if (ret)
			return ret;

		ret = wacom_setup_device(client);
		if (ret)
			return ret;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "WAC_I2C_EMR", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id wacom_i2c_of_match_table[] = {
	{ .compatible = "wacom,wacom-i2c" },
	{}
};
MODULE_DEVICE_TABLE(of, wacom_i2c_of_match_table);
#endif

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.pm	= &wacom_i2c_pm,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(wacom_i2c_of_match_table),
#endif
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};
module_i2c_driver(wacom_i2c_driver);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
