/* drivers/input/touchscreen/novatek-nt11206.c
 *
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * Novatek NT11206 Capacitive Touchscreen Driver
 *
 * This driver provides full support for the Novatek NT11206 touchscreen 
 * controller, handling I2C communication, interrupt management, input 
 * event reporting, and device initialization.
 *
 * Key Features:
 * - Multi-touch support (up to 10 fingers)
 * - Dynamic touch point tracking
 * - Flexible GPIO and regulator management
 * - Robust error handling and retry mechanisms
 *
 * Supported Platforms:
 * - Linux kernel 6.x
 * - I2C-based embedded systems
 *
 * Device Tree Configuration:
 * Required properties:
 * - reset-gpios: Reset GPIO pin
 * - irq-gpios: Interrupt GPIO pin
 * - vcc, iovcc: Regulators for power management
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#define TOUCH_MAX_WIDTH 720
#define TOUCH_MAX_HEIGHT 1280
#define TOUCH_MAX_FINGER_NUM 10

#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

// I2C Analysis
uint8_t read_buffer[2000] = { 0 };

static int read_index = 1;
module_param(read_index, int, 0644);
MODULE_PARM_DESC(read_index, "Index of data to read via I2C");

static int read_length = 256;
module_param(read_length, int, 0644);
MODULE_PARM_DESC(read_length, "Length of data to read via I2C");

/**
 * struct nvt_ts_data - Comprehensive touchscreen device state management
 * 
 * Encapsulates all critical data and configuration for the Novatek touchscreen device.
 * Provides a centralized structure for tracking device properties, I2C communication,
 * input handling, and power management.
 */
struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct regulator_bulk_data regulators[2];
	struct regulator *vcc;
	struct regulator *iovcc;
	struct gpio_desc *reset_gpio;
};
struct nvt_ts_data *data;

struct nvt_ts_i2c_chip_data {
	u8 wake_type;
	u8 chip_id;
};

/**
 * enum RST_COMPLETE_STATE - Touchscreen Reset State Tracking
 * 
 * Defines the possible states during the device reset process.
 * Helps track and validate the reset sequence for the touchscreen controller.
 *
 * @RESET_STATE_INIT: Initial reset state, device is being initialized
 * @RESET_STATE_REK: Reconfiguration state
 * @RESET_STATE_REK_FINISH: Reconfiguration completed
 * @RESET_STATE_NORMAL_RUN: Device is in normal operational mode
 */
typedef enum {
	RESET_STATE_INIT = 0xA0,
	RESET_STATE_REK,
	RESET_STATE_REK_FINISH,
	RESET_STATE_NORMAL_RUN
} RST_COMPLETE_STATE;

int32_t novatek_i2c_read(struct i2c_client *client, uint16_t address,
			 uint8_t *buf, uint16_t len);
int32_t novatek_i2c_read_dummy(struct i2c_client *client, uint16_t address);

int32_t novatek_i2c_write(struct i2c_client *client, uint16_t address,
			  uint8_t *buf, uint16_t len);

/**
 * novatek_i2c_read - Read data from the Novatek CTP device via I2C.
 * @client: Pointer to the I2C client structure.
 * @address: I2C address to read from.
 * @buf: Buffer to store the read data.
 * @len: Length of data to read.
 *
 * Returns:
 * 0 on success, or a negative error code on failure.
 */
int32_t novatek_i2c_read(struct i2c_client *client, uint16_t address,
			 uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg[2] = { {
					  .addr = address,
					  .flags = !I2C_M_RD,
					  .len = 1,
					  .buf = &buf[0],
				  },
				  {
					  .addr = address,
					  .flags = I2C_M_RD,
					  .len = len - 1,
					  .buf = &buf[1],
				  } };
	int error = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (error != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "I2C read error: %d\n", error);
		return (error < 0) ? error : -EIO;
	}

	dev_info(&client->dev, "Read %d bytes from register 0x%02x: %*ph",
		 len - 1, address, len - 1, buf);

	return 0;
}

/**
 * novatek_i2c_read_dummy - Perform a dummy read to wake up the Novatek CTP device.
 * @client: Pointer to the I2C client structure.
 * @address: I2C address to read from.
 *
 * This function sends a dummy read command to the Novatek CTP device to ensure it is awake
 * and ready to accept further commands. It performs multiple retries in case of failure.
 *
 * Returns:
 * Number of messages successfully transferred, or a negative error code on failure.
 */
int32_t novatek_i2c_read_dummy(struct i2c_client *client, uint16_t address)
{
	struct i2c_msg msg;
	uint8_t buf[8] = { 0 };
	int32_t error = -1;

	msg.flags = I2C_M_RD; // Read message flag
	msg.addr = address; // I2C address to read from
	msg.len = 1; // Length of the read message
	msg.buf = buf; // Buffer to store the read data

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error == 1) {
		dev_dbg(&client->dev, "Dummy read 1 byte from 0x%02x: %*ph\n",
			address, 1, buf);
	}

	return error;
}

/**
 * novatek_i2c_write - Write data to the Novatek CTP device via I2C.
 * @client: Pointer to the I2C client structure.
 * @address: I2C address to write to.
 * @buf: Buffer containing the data to write.
 * @len: Length of data to write.
 *
 * Returns:
 * 0 on success, or a negative error code on failure.
 */
int32_t novatek_i2c_write(struct i2c_client *client, uint16_t address,
			  uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;

	msg.flags = !I2C_M_RD; // Write message flag
	msg.addr = address; // I2C address to write to
	msg.len = len; // Length of the write message
	msg.buf = buf; // Buffer containing the data to write

	// Perform I2C transfer
	dev_info(&client->dev, "Writing %d bytes to register 0x%02x: %*ph", len,
		 address, len, buf);
	int error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0) {
		dev_err(&client->dev, "I2C write error: %d\n", error);
		return error;
	}

	return 0;
}

/**
 * novatek_touchscreen_work_func - Asynchronous Touch Event Processing Handler
 * 
 * Background worker function responsible for processing and reporting 
 * touch events received from the Novatek touchscreen device. Designed 
 * to handle multi-touch input with high performance and reliability.
 */
static irqreturn_t novatek_touchscreen_irq(int irq, void *dev_id)
{
	dev_info(&data->client->dev, "Interrupt received\n");
	struct nvt_ts_data *data = dev_id;
	struct device *dev = &data->client->dev;

	int32_t error = -1;
	uint8_t point_data[70] = { 0 };
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = { 0 };

	int32_t i = 0;
	int32_t finger_cnt = 0;

	error = novatek_i2c_read(data->client, I2C_FW_Address, point_data,
				 65 + 1);
	if (error)
		return IRQ_HANDLED;

	finger_cnt = 0;
	input_id = (uint8_t)(point_data[1] >> 3);

	for (i = 0; i < TOUCH_MAX_FINGER_NUM; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if (input_id > TOUCH_MAX_FINGER_NUM)
			continue;

		if (((point_data[position] & 0x07) == 0x01) ||
		    ((point_data[position] & 0x07) ==
		     0x02)) { //finger down (enter & moving)
			input_x = (uint32_t)(point_data[position + 1] << 4) +
				  (uint32_t)(point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) +
				  (uint32_t)(point_data[position + 3] & 0x0F);
			input_w = (uint32_t)(point_data[position + 4]) + 10;
			if (input_w > 255)
				input_w = 255;

			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > TOUCH_MAX_WIDTH) ||
			    (input_y > TOUCH_MAX_HEIGHT))
				continue;

			press_id[input_id - 1] = 1;
			input_mt_slot(data->input_dev, input_id - 1);
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, true);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 input_x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 input_y);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 input_w);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					 input_w);

			finger_cnt++;
		}
	}

	for (i = 0; i < TOUCH_MAX_FINGER_NUM; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(data->input_dev, i);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 0);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, false);
		}
	}

	input_report_key(data->input_dev, BTN_TOUCH, (finger_cnt > 0));

	input_sync(data->input_dev);

	return IRQ_HANDLED;
}

static void probe_i2c(struct device *dev, struct i2c_client *client,
		      const char *message)
{
	dev_info(dev, "%s", message);
	novatek_i2c_read(client, read_index, read_buffer, read_length);
}

/**
 * nvt_ts_probe - Probe function for the Novatek touchscreen driver.
 * @client: Pointer to the I2C client structure.
 *
 * Returns:
 * 0 on success, negative error code on failure.
 */
static int nvt_ts_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int error;
	const struct nvt_ts_i2c_chip_data *chip;
	uint8_t buf[8] = { 0 };

	// Validate inputs
	if (!client->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}
	dev_info(dev, "Initial irq %d\n", client->irq);

	// Allocate and zero-initialize memory for touchscreen device data
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	chip = device_get_match_data(dev);
	if (!chip) {
		return -EINVAL;
	}

	// Configure power regulators
	data->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(data->vcc)) {
		error = PTR_ERR(data->vcc);
		dev_err(dev, "Failed to get vcc regulator: %d\n", error);
		return error;
	}

	data->iovcc = devm_regulator_get(dev, "iovcc");
	if (IS_ERR(data->iovcc)) {
		error = PTR_ERR(data->iovcc);
		dev_err(dev, "Failed to get iovcc regulator: %d\n", error);
		return error;
	}

	error = regulator_enable(data->vcc);
	if (error) {
		dev_err(dev, "Failed to enable vcc regulator: %d\n", error);
		return error;
	}

	error = regulator_enable(data->iovcc);
	if (error) {
		dev_err(dev, "Failed to enable iovcc regulator: %d\n", error);
		regulator_disable(data->vcc);
		return error;
	}

	// Request GPIO descriptors using the new gpiod API
	data->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);

	if (IS_ERR(data->reset_gpio)) {
		dev_err(dev, "Failed to request GPIOs: reset=%ld\n",
			PTR_ERR(data->reset_gpio));
		return PTR_ERR(data->reset_gpio);
	}
	if (!data->reset_gpio) {
		dev_err(dev, "Failed to get reset GPIO\n");
		return -ENODEV;
	}

	msleep(50);

	// Configure the reset GPIO as output with an initial high value
	error = gpiod_direction_output(data->reset_gpio, 1);
	if (error) {
		dev_err(dev, "Failed to configure reset GPIO: %d\n", error);
		regulator_disable(data->iovcc);
		regulator_disable(data->vcc);
		return error;
	}

	// Delay 10ms after Power-On Reset (POR) to ensure device stability
	msleep(10);

	// Perform dummy read to resume touchscreen before sending commands
	dev_info(dev, "Performing dummy read to resume touchscreen\n");
	error = novatek_i2c_read_dummy(client, I2C_FW_Address);
	if (error < 0) {
		dev_err(dev, "Dummy read failed: %d\n", error);
		regulator_disable(data->iovcc);
		regulator_disable(data->vcc);
		return error;
	}

	// Reset idle to keep default addr 0x01 to read chipid
	dev_info(dev, "Reset idle to keep default addr 0x01 to read chipid\n");
	buf[0] = 0x00;
	buf[1] = 0xA5;
	error = novatek_i2c_write(data->client, I2C_HW_Address, buf, 2);
	if (error < 0) {
		dev_err(dev, "Reset write failed: %d\n", error);
		return error;
	}

	msleep(100);

	// Write i2c index to 0x1F000
	dev_info(dev, "Writing index to 0x1F000\n");
	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0xF0;
	error = novatek_i2c_write(data->client, I2C_FW_Address, buf, 3);
	if (error < 0) {
		dev_err(dev, "Failed to write index: %d\n", error);
		return error;
	}

	// Read hw chip id
	dev_info(dev, "Reading chip ID\n");
	buf[0] = 0x00;
	buf[1] = 0x00;
	error = novatek_i2c_read(data->client, I2C_FW_Address, buf, 3);
	if (error < 0) {
		dev_err(dev, "Failed to read chip ID: %d\n", error);
		return error;
	}

	if (buf[1] != 0x26) {
		dev_err(dev, "Invalid chip ID: 0x%02X\n", buf[1]);
		return -ENODEV;
	}

	// Allocate input device
	data->input_dev = devm_input_allocate_device(dev);
	if (!data->input_dev) {
		dev_err(dev, "Failed to allocate input device\n");
		regulator_disable(data->iovcc);
		regulator_disable(data->vcc);
		return -ENOMEM;
	}

	// Set input device parameters
	data->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |
				    BIT_MASK(EV_ABS);
	data->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	data->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_mt_init_slots(data->input_dev, TOUCH_MAX_FINGER_NUM, 0);

	input_set_abs_params(data->input_dev, ABS_MT_POSITION_X, 0,
			     TOUCH_MAX_WIDTH, 0, 0);
	input_set_abs_params(data->input_dev, ABS_MT_POSITION_Y, 0,
			     TOUCH_MAX_HEIGHT, 0, 0);

	data->input_dev->name = client->name;
	data->input_dev->id.bustype = BUS_I2C;

	// Register input device
	error = input_register_device(data->input_dev);
	if (error) {
		dev_err(dev, "Failed to register input device: %d\n", error);
		return error;
	}

	error = devm_request_threaded_irq(dev, client->irq, NULL,
					  novatek_touchscreen_irq,
					  IRQF_ONESHOT | IRQF_TRIGGER_RISING,
					  client->name, data);
	if (error) {
		dev_err(dev, "Failed to request IRQ: %d\n", error);
		return error;
	}

	gpiod_set_value(data->reset_gpio, 1);
	msleep(50);
	gpiod_set_value(data->reset_gpio, 0);
	msleep(50);
	gpiod_set_value(data->reset_gpio, 1);
	msleep(100);

	// Bootloader reset
	dev_info(dev, "Resetting bootloader\n");
	buf[0] = 0x00;
	buf[1] = 0x69;
	error = novatek_i2c_write(data->client, I2C_HW_Address, buf, 2);
	if (error < 0) {
		dev_err(dev, "Bootloader reset failed: %d\n", error);
		return error;
	}

	msleep(35);

	// Check firmware reset state
	dev_info(dev, "Checking reset state\n");
	unsigned long timeout =
		jiffies + msecs_to_jiffies(5000); // 5 seconds timeout
	while (1) {
		msleep(10);

		buf[0] = 0x60;
		buf[1] = 0x00;
		error = novatek_i2c_read(data->client, I2C_FW_Address, buf, 2);
		if (error < 0) {
			dev_err(dev, "Failed to read reset state: %d\n", error);
			return error;
		}

		if ((buf[1] >= RESET_STATE_INIT) && (buf[1] < 0xFF))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dev, "Timeout while waiting for reset state\n");
			return -ETIMEDOUT;
		}
	}
	dev_info(dev, "Probe completed successfully\n");
	return 0;
}

static const struct nvt_ts_i2c_chip_data nvt_nt11206_ts_data = {
	.wake_type = 0x05,
	.chip_id = 0x26,
};

static const struct of_device_id nvt_ts_of_match[] = {
	{ .compatible = "novatek,nt11206-ts", .data = &nvt_nt11206_ts_data },
	{}
};
MODULE_DEVICE_TABLE(of, nvt_ts_of_match);

static const struct i2c_device_id nvt_ts_i2c_id[] = {
	{ "nt11206-ts", (unsigned long)&nvt_nt11206_ts_data },
	{}
};
MODULE_DEVICE_TABLE(i2c, nvt_ts_i2c_id);

static struct i2c_driver nvt_ts_driver = {
	.driver = {
		.name	= "novatek-nt11206-ts",
		.owner	= THIS_MODULE,
		.of_match_table = nvt_ts_of_match,
	},
	.probe		= nvt_ts_probe,
	.id_table	= nvt_ts_i2c_id,

};

module_i2c_driver(nvt_ts_driver);

MODULE_DESCRIPTION("Novatek NT11206 touchscreen driver");
MODULE_LICENSE("GPL");
