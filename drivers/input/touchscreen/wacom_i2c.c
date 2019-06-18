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
	u8 data[WACOM_MAX_DATA_SIZE];
	bool prox;
	int tool;
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

//#define GPIO_TEST_LED   174

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
    u8 read_data[] = {
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
#if 0
        // Setup for reading data
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(read_data),
			.buf = read_data,
		},
#endif
        // Read 21 bytes
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 21,
			.buf = data,
		},
	};


    printk("[---- SBA ----] Asserting/deasserting WACOM reset ..\n");
    rstc = devm_reset_control_get_optional_exclusive(&client->dev, NULL);
    if (IS_ERR(rstc)) {
        printk("[---- SBA ----] Failed to get reset control while trying to reset WACOM device before init !\n");
    }
    else {
        reset_control_reset(rstc);
    }

	printk("[---- SBA ----] Sending cmd1, cmd2 and data over I2C ..\n");
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		printk("[---- SBA ----] i2c_transfer returned with errorcode %d\n", ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		printk("[---- SBA ----] i2c_transfer returned with unexpected return value %d\n", ret);
		return -EIO;
	}

    printk("[---- SBA ----] query report (%d) length: %d\n", data[2], data[0] + 256*data[1]);

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

    printk("[WACOM] x_max:%d, y_max:%d, pressure:%d, fw:%d, "
		"distance: %d, phys distance: %d"
		"tilt_x_max: %d, tilt_y_max: %d\n",
		features->x_max, features->y_max,
		features->pressure_max, features->fw_version,
		features->distance_max, features->distance_physical_max,
		features->tilt_x_max, features->tilt_y_max);

	return 0;
}

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
	printk("[---- SBA ----] Writing sample rate..\n");
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		printk("[---- SBA ----] i2c_transfer returned with errorcode %d\n", ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		printk("[---- SBA ----] i2c_transfer returned with unexpected return value %d\n", ret);
		return -EIO;
	}

    // Read position rate report
    printk("[---- SBA ----] Reading sample rate..\n");
	ret = i2c_transfer(client->adapter, msgs_readback, ARRAY_SIZE(msgs_readback));
	if (ret < 0) {
		printk("[---- SBA ----] i2c_transfer returned with errorcode %d\n", ret);
		return ret;
	}
	if (ret != ARRAY_SIZE(msgs_readback)) {
		printk("[---- SBA ----] i2c_transfer returned with unexpected return value %d\n", ret);
		return -EIO;
	}

    printk("[---- SBA ----] Rate report: %x %x %x %x ..\n", data[0], data[1], data[2], data[3]);

	return 0;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{

	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	u8 *data = wac_i2c->data;
	unsigned int x, y, pressure;
	unsigned char tip, f1, f2, eraser, distance, transducer = 0;
	short tilt_x, tilt_y;
	int error;

// TODO: Should continue to read packets until DIGITIZER_INT is pulled high

    //gpio_set_value(GPIO_TEST_LED, 1);
#if 1
	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, 17);
#else
    // Skip bus locking
    const struct i2c_client *client = wac_i2c->client;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = 17;
	msg.buf = wac_i2c->data;
	error = __i2c_transfer(adap, &msg, 1);
#endif

    //gpio_set_value(GPIO_TEST_LED, 0);

	if (error < 0) {
        //printk("[----- SBA -----] error = %d\n", error);
		goto out;
    }

    // data[0] == Length LSB
    // data[1] == Length MSB
    // data[2] == ReportID (2 | 26)
	tip = data[3] & 0x01;
	eraser = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
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

    // Toggle LED GPIO:
#if 0
    if (x > 10484) {
        gpio_set_value(GPIO_TEST_LED, 1);
    } else {
        gpio_set_value(GPIO_TEST_LED, 0);
    }
#endif

	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

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
	struct input_dev *input;
	struct wacom_features features = { 0 };
    struct device_node *np;
	int error;
    unsigned int flags, ret, reset_gpio;

    // Setup GPIO_TEST_LED
//    gpio_request(GPIO_TEST_LED, "test_led");
//    gpio_direction_output(GPIO_TEST_LED, 0);

	printk("[---- SBA ----] wacom_i2c_probe enter\n");
	printk("[---- SBA ----] Stack trace:\n");
    dump_stack();
    printk("[---- SBA ----] client->addr: 0x%02X\n", client->addr);
    printk("[---- SBA ----] client->irq: %d\n", client->irq);
    printk("[---- SBA ----] client->name: %s\n", client->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	printk("[---- SBA ----] Reading reset-gpio config if present..\n");
    np = client->dev.of_node;
	reset_gpio = of_get_named_gpio_flags(np, "reset-gpio", 0, &flags);
    if (reset_gpio == -EPROBE_DEFER) {
        printk("[---- SBA ----] of_get_named_gpio_flags returned EPROBE_DEFER, ignoring reset-gpio..\n");
    } else if (!gpio_is_valid(reset_gpio)) {
        printk("[---- SBA ----] reset-gpio found: %d\n", reset_gpio);
        printk("[---- SBA ----] Invalid reset gpio: %d\n", reset_gpio);
    }

    printk("[---- SBA ----] Requesting gpio %d ..\n", reset_gpio);
    ret = devm_gpio_request_one(&client->dev, reset_gpio, flags, NULL);
    if (ret < 0) {
        printk("[---- SBA ----] Failed to request gpio %d: %d\n", reset_gpio, ret);
    }

    printk("[---- SBA ----] Setting requested GPIO as output ..\n");
    gpio_direction_output(reset_gpio, 1);
    printk("[---- SBA ----] Trying to pull reset GPIO low ..\n");
    gpio_set_value(reset_gpio, 0);
    printk("[---- SBA ----] Waiting 500 us..\n");
    udelay(500);
    printk("[---- SBA ----] Trying to pull reset GPIO high ..\n");
    gpio_set_value(reset_gpio, 1);
    printk("[---- SBA ----] Waiting 500 us..\n");
    udelay(500);

    printk("[---- SBA ----] Querying device ..\n");
	error = wacom_query_device(client, &features);
	if (error)
		return error;

    printk("[---- SBA ----] Setting up device ..\n");
	error = wacom_setup_device(client);
	if (error)
		return error;

	printk("[---- SBA ----] Allocating memory to hold device info..\n");
	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);

	printk("[---- SBA ----] Allocating input device in input subsystem..\n");
	input = input_allocate_device();
	if (!wac_i2c || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	wac_i2c->client = client;
	wac_i2c->input = input;

	printk("[---- SBA ----] Registering information for wacom device as new input..\n");
	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = features.fw_version;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, features.x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, features.y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
			     0, features.pressure_max, 0, 0);

	input_set_abs_params(input, ABS_DISTANCE, 0, features.distance_max, 0, 0);
	input_set_abs_params(input, ABS_TILT_X, -features.tilt_x_max, features.tilt_x_max, 0, 0);
	input_set_abs_params(input, ABS_TILT_Y, -features.tilt_y_max, features.tilt_y_max, 0, 0);

	input_set_drvdata(input, wac_i2c);

	printk("[---- SBA ----] Requesting threaded irq (client->irq=%d) ..\n", client->irq);
	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_mem;
	}

	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
    printk("[---- SBA ----] Disabling IRQ for now (to be enabled in wac_i2c_open) ..\n");
	disable_irq(client->irq);

	printk("[---- SBA ----] Register wacom input device ..\n");
	error = input_register_device(wac_i2c->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
		goto err_free_irq;
	}

	printk("[---- SBA ----] Set wacom device data in i2c client structure ..\n");
	i2c_set_clientdata(client, wac_i2c);
	return 0;

err_free_irq:
	free_irq(client->irq, wac_i2c);
err_free_mem:
	input_free_device(input);
	kfree(wac_i2c);

	printk("[---- SBA ----] wacom_i2c_probe exit\n");
	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input);
	kfree(wac_i2c);

	return 0;
}

static int __maybe_unused wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);

	return 0;
}

static int __maybe_unused wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);

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
