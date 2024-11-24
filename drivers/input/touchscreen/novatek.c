/* drivers/input/touchscreen/novatek.c
 *
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 *
 * $Revision: 6200 $
 * $Date: 2016-09-02 15:52:16 +0800 (?��?, 02 九�? 2016) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>


#define BBOX_TP_I2C_READ_FAIL	do {printk("BBox;%s: TP i2c read fail!\n", __func__); printk("BBox::UEC;7::1\n");} while (0);
#define BBOX_TP_I2C_WRITE_FAIL	do {printk("BBox;%s: TP i2c write fail!\n", __func__); printk("BBox::UEC;7::2\n");} while (0);
#define BBOX_TP_PROBE_FAIL	do {printk("BBox;%s: TP firmware update fail!\n", __func__); printk("BBox::UEC;7::0\n");} while (0);
#define DEVICE_NAME	"NVTflash"
#define NOVATEK_I2C_NAME "NVT-ts"
#define IRQ_TYPE_EDGE_RISING 1
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62
#define NOVATEK_TS_NAME "NVTCapacitiveTouchScreen"
#define TOUCH_MAX_WIDTH  720
#define TOUCH_MAX_HEIGHT 1280
#define TOUCH_MAX_FINGER_NUM 10


struct novatek_touchscreen_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct novatek_work;
	struct delayed_work novatek_fwu_work;
	uint16_t addr;
	int8_t phys[32];
	struct notifier_block fb_notif;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	struct regulator *avdd;
	struct regulator *vdd;
	bool power_on;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	struct work_struct	work;
};

struct novatek_flash_data{
	rwlock_t lock;
	struct i2c_client *client;
};

typedef enum {
	RESET_STATE_INIT = 0xA0,
	RESET_STATE_REK,		
	RESET_STATE_REK_FINISH,	
	RESET_STATE_NORMAL_RUN	
} RST_COMPLETE_STATE;

// Prototypes
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf,
		     uint16_t len);
int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address);
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf,
		      uint16_t len);
int32_t novatek_clear_fw_status(void);
int32_t novatek_check_fw_status(void);
int32_t novatek_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
void novatek_hw_reset(void);
void novatek_set_i2c_debounce(void);
void novatek_sw_reset_idle(void);
void novatek_bootloader_reset(void);

static ssize_t novatek_flash_read(struct file *file, char __user *buff,
			      size_t count, loff_t *offp);
static int32_t novatek_flash_open(struct inode *inode, struct file *file);
static int32_t novatek_flash_close(struct inode *inode, struct file *file);

static const struct proc_ops novatek_flash_proc_ops = {
	.proc_read = novatek_flash_read,
	.proc_open = novatek_flash_open,
	.proc_release = novatek_flash_close,
	.proc_lseek = generic_file_llseek,
};

// Declarations
struct novatek_touchscreen_data *ts;
static struct proc_dir_entry *NOVATEK_proc_entry;
static struct workqueue_struct *novatek_workqueue;
static uint8_t bTouchIsAwake = 0;


int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf,
		     uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = address;
	msgs[0].len = 1;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = address;
	msgs[1].len = len - 1;
	msgs[1].buf = &buf[1];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		dev_err(&client->dev, "%s: error, ret=%d\n", __func__, ret);
		BBOX_TP_I2C_READ_FAIL
		ret = -EIO;
	}

	return ret;
}

int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	struct i2c_msg msg;
	uint8_t buf[8] = { 0 };
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = I2C_M_RD;
	msg.addr = address;
	msg.len = 1;
	msg.buf = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		dev_err(&client->dev, "%s: error, ret=%d\n", __func__, ret);
		BBOX_TP_I2C_READ_FAIL
		ret = -EIO;
	}

	return ret;
}

int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf,
		      uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr = address;
	msg.len = len;
	msg.buf = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		dev_err(&client->dev, "%s: error, ret=%d\n", __func__, ret);
		BBOX_TP_I2C_WRITE_FAIL
		ret = -EIO;
	}

	return ret;
}

void novatek_hw_reset(void)
{
	//---trigger rst-pin to reset---
	gpio_set_value(ts->reset_gpio, 1);
	msleep(20);
	gpio_set_value(ts->reset_gpio, 0);
	msleep(20);
	gpio_set_value(ts->reset_gpio, 1);
	msleep(20);
}

void novatek_set_i2c_debounce(void)
{
	uint8_t buf[8] = { 0 };
	uint8_t reg1_val = 0;
	uint8_t reg2_val = 0;
	uint32_t retry = 0;

	do {
		msleep(10);

		// set xdata index to 0x1F000
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF0;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		// set i2c debounce 34ns
		buf[0] = 0x15;
		buf[1] = 0x17;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		buf[0] = 0x15;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		reg1_val = buf[1];

		// set schmitt trigger enable
		buf[0] = 0x3E;
		buf[1] = 0x07;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		buf[0] = 0x3E;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);
		reg2_val = buf[1];
	} while (((reg1_val != 0x17) || (reg2_val != 0x07)) && (retry++ < 20));

	if (retry == 20) {
		dev_err(&ts->client->dev,
			"%s: set i2c debounce failed, reg1_val=0x%02X, reg2_val=0x%02X\n",
			__func__, reg1_val, reg2_val);
	}
}

void novatek_sw_reset_idle(void)
{
	uint8_t buf[4] = { 0 };

	//---write i2c cmds to reset idle---
	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);

	novatek_set_i2c_debounce();
}

void novatek_bootloader_reset(void)
{
	uint8_t buf[8] = { 0 };

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 35ms delay after bootloader reset
	msleep(35);
}

int32_t novatek_clear_fw_status(void)
{
	uint8_t buf[8] = { 0 };
	int32_t i = 0;
	const int32_t retry = 10;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	for (i = 0; i < retry; i++) {
		//---set xdata index to 0x14700---
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x47;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---clear fw status---
		buf[0] = 0x51;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = 0x51;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry)
		return -1;
	else
		return 0;
}

int32_t novatek_check_fw_status(void)
{
	uint8_t buf[8] = { 0 };
	int32_t i = 0;
	const int32_t retry = 10;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	for (i = 0; i < retry; i++) {
		//---set xdata index to 0x14700---
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x47;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read fw status---
		buf[0] = 0x51;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry)
		return -1;
	else
		return 0;
}

int32_t novatek_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = { 0 };
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		//---read reset state---
		buf[0] = 0x60;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] >= check_reset_state) && (buf[1] < 0xFF)) {
			ret = 0;
			break;
		}

		retry++;
		if (unlikely(retry > 100)) {
			ret = -1;
			dev_err(&ts->client->dev,
				"%s: error, retry=%d, buf[1]=0x%02X\n",
				__func__, retry, buf[1]);
			break;
		}
	}

	return ret;
}

static ssize_t novatek_flash_read(struct file *file, char __user *buff,
				  size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];
	uint8_t str[64] = { 0 };
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str))
		return -EFAULT;

	if (copy_from_user(str, buff, count))
		return -EFAULT;

	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) { //I2C write
		msgs[0].flags = !I2C_M_RD;
		msgs[0].addr = str[0] & 0x7F;
		msgs[0].len = str[1];
		msgs[0].buf = &str[2];

		while (retries < 20) {
			ret = i2c_transfer(ts->client->adapter, msgs, 1);
			if (ret == 1)
				break;
			else
				dev_err(&ts->client->dev,
					"%s: error, retries=%d, ret=%d\n",
					__func__, retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			dev_err(&ts->client->dev, "%s: error, ret = %d\n",
				__func__, ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == 1) { //I2C read
		msgs[0].flags = !I2C_M_RD;
		msgs[0].addr = str[0] & 0x7F;
		msgs[0].len = 1;
		msgs[0].buf = &str[2];

		msgs[1].flags = I2C_M_RD;
		msgs[1].addr = str[0] & 0x7F;
		msgs[1].len = str[1] - 1;
		msgs[1].buf = &str[3];

		while (retries < 20) {
			ret = i2c_transfer(ts->client->adapter, msgs, 2);
			if (ret == 2)
				break;
			else
				dev_err(&ts->client->dev,
					"%s: error, retries=%d, ret=%d\n",
					__func__, retries, ret);

			retries++;
		}

		// copy buff to user if i2c transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			dev_err(&ts->client->dev, "%s: error, ret = %d\n",
				__func__, ret);
			return -EIO;
		}

		return ret;
	} else {
		dev_err(&ts->client->dev, "%s: Call error, str[0]=%d\n",
			__func__, str[0]);
		return -EFAULT;
	}
}

static int32_t novatek_flash_open(struct inode *inode, struct file *file)
{
	struct novatek_flash_data *dev;

	dev = kmalloc(sizeof(struct novatek_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&ts->client->dev,
			"%s: Failed to allocate memory for nvt flash data\n",
			__func__);
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

static int32_t novatek_flash_close(struct inode *inode, struct file *file)
{
	struct novatek_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static int32_t novatek_flash_proc_init(void)
{
	NOVATEK_proc_entry =
		proc_create(DEVICE_NAME, 0444, NULL, &novatek_flash_proc_ops);
	if (NOVATEK_proc_entry == NULL) {
		dev_err(&ts->client->dev, "%s: Failed!\n", __func__);
		return -ENOMEM;
	} else {
		dev_info(&ts->client->dev, "%s: Succeeded!\n", __func__);
	}

	dev_info(
		&ts->client->dev,
		"============================================================\n");
	dev_info(&ts->client->dev, "Create /proc/NVTflash\n");
	dev_info(
		&ts->client->dev,
		"============================================================\n");

	return 0;
}

static int32_t novatek_power_on(struct novatek_touchscreen_data *ts)
{
	int32_t ret = 0;

	if (ts->power_on) {
		dev_info(&ts->client->dev, "Device already power on\n");
		return 0;
	}

	if (!IS_ERR(ts->avdd)) {
		ret = regulator_enable(ts->avdd);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator avdd enable failed ret=%d\n", ret);
			goto err_enable_avdd;
		}
	}

	if (!IS_ERR(ts->vdd)) {
		ret = regulator_enable(ts->vdd);
		if (ret) {
			dev_err(&ts->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			goto err_enable_vdd;
		}
	}

	ts->power_on = true;
	return 0;

err_enable_vdd:
	if (!IS_ERR(ts->avdd))
		regulator_disable(ts->avdd);
err_enable_avdd:
	ts->power_on = false;
	return ret;
}

static int32_t novatek_power_off(struct novatek_touchscreen_data *ts)
{
	int32_t ret = 0;

	if (!ts->power_on) {
		dev_info(&ts->client->dev, "Device already power off\n");
		return 0;
	}

	if (!IS_ERR(ts->vdd)) {
		ret = regulator_disable(ts->vdd);
		if (ret)
			dev_err(&ts->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
	}

	if (!IS_ERR(ts->avdd)) {
		ret = regulator_disable(ts->avdd);
		if (ret)
			dev_err(&ts->client->dev,
				"Regulator avdd disable failed ret=%d\n", ret);
	}

	ts->power_on = false;
	return 0;
}

static int32_t novatek_power_init(struct novatek_touchscreen_data *ts)
{
	int32_t ret = 0;

	ts->avdd = regulator_get(&ts->client->dev, "avdd");
	if (IS_ERR(ts->avdd)) {
		ret = PTR_ERR(ts->avdd);
		dev_err(&ts->client->dev, "Regulator get failed avdd ret=%d\n",
			ret);
	}

	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		dev_err(&ts->client->dev, "Regulator get failed vdd ret=%d\n",
			ret);
	}

	return 0;
}

static int32_t novatek_power_remove(struct novatek_touchscreen_data *ts)
{
	regulator_put(ts->vdd);
	regulator_put(ts->avdd);

	return 0;
}

static void novatek_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;

	ts->reset_gpio = of_get_named_gpio(np, "novatek,reset-gpio", 0);
	ts->irq_gpio = of_get_named_gpio(np, "novatek,irq-gpio", 0);

	dev_info(dev, "%s: novatek,reset-gpio=%d, novatek,irq-gpio=%d\n",
		 __func__, ts->reset_gpio, ts->irq_gpio);
}

static void novatek_touchscreen_work_func(struct work_struct *work)
{
	//struct novatek_touchscreen_data *ts = container_of(work, struct novatek_touchscreen_data, work);
	struct i2c_client *client = ts->client;

	int32_t ret = -1;
	uint8_t point_data[70] = { 0 };
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint8_t input_id = 0;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = { 0 };

	int32_t i = 0;
	int32_t finger_cnt = 0;

	mutex_lock(&ts->lock);

	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, 65 + 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: CTP_I2C_READ failed.(%d)\n",
			__func__, ret);
		goto XFER_ERROR;
	}

	finger_cnt = 0;
	input_id = (uint8_t)(point_data[1] >> 3);

	for (i = 0; i < ts->max_touch_num; i++) {
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
			if ((input_x > ts->abs_x_max) ||
			    (input_y > ts->abs_y_max))
				continue;

			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, true);

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					 input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					 input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					 input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					 input_w);

			finger_cnt++;
		}
	}

	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));

	input_sync(ts->input_dev);

XFER_ERROR:
	enable_irq(ts->client->irq);

	mutex_unlock(&ts->lock);
}

static irqreturn_t novatek_touchscreen_irq_handler(int32_t irq, void *dev_id)
{
	//struct novatek_touchscreen_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);

	queue_work(novatek_workqueue, &ts->novatek_work);

	return IRQ_HANDLED;
}

static int32_t novatek_touchscreen_chip_version(void)
{
	uint8_t buf[8] = { 0 };
	int32_t ret = 0;
	uint8_t cut_number = 0;
	uint8_t cut_version = 0;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	//write i2c index to 0x1F000
	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0xF0;
	ret = CTP_I2C_WRITE(ts->client, 0x01, buf, 3);
	if (ret < 0) {
		dev_err(&ts->client->dev, "%s: write i2c index error!!(%d)\n",
			__func__, ret);
		return ret;
	}

	//read chip version
	buf[0] = 0x01;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, 0x01, buf, 3);
	if (ret < 0) {
		dev_err(&ts->client->dev, "%s: read chip version error!!(%d)\n",
			__func__, ret);
		return ret;
	}

	// [4:0]: Cut Number
	cut_number = buf[1] & 0x1F;
	// [7:5]: Cut Version
	cut_version = (buf[1] & 0xE0) >> 5;

	dev_info(&ts->client->dev,
		 "chip version: cut number=%d, cut version=%d\n", cut_number,
		 cut_version);

	return ret;
}

static uint8_t novatek_touchscreen_read_chipid(void)
{
	uint8_t buf[8] = { 0 };
	int32_t retry = 0;

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_HW_Address);

	// reset idle to keep default addr 0x01 to read chipid
	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(10);

	//---Check NT11206 for 5 times---
	for (retry = 5; retry > 0; retry--) {
		//write i2c index to 0x1F000
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF0;
		CTP_I2C_WRITE(ts->client, 0x01, buf, 3);

		//read hw chip id
		buf[0] = 0x00;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, 0x01, buf, 3);

		if (buf[1] == 0x26)
			break;
	}

	return buf[1];
}

static int novatek_touchscreen_probe(struct i2c_client *client)
{
	int32_t ret = 0;

	dev_info(&client->dev, "%s:11/08 999 start.......\n", __func__);
	ts = kmalloc(sizeof(struct novatek_touchscreen_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev,
			"%s: failed to allocated memory for nvt ts data\n",
			__func__);
		BBOX_TP_PROBE_FAIL
		return -ENOMEM;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);

	//---parse dts---
	novatek_parse_dt(&client->dev);

	ts->power_on = false;

	//---request regulator for DragonBoard---
	ret = novatek_power_init(ts);
	if (ret) {
		dev_err(&client->dev, "nvt power init failed\n");
		goto err_power_init;
	}

	//---turn on regulator for DragonBoard---
	ret = novatek_power_on(ts);
	if (ret) {
		dev_err(&client->dev, "nvt power on failed\n");
		goto err_novatek_power_on;
	}

	//---request RST-pin & INT-pin---
	ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-rst");
	if (ret)
		dev_err(&client->dev, "Failed to get NVT-rst GPIO\n");

	ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
	if (ret)
		dev_err(&client->dev, "Failed to get NVT-int GPIO\n");

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip id---
	ret = novatek_touchscreen_read_chipid();
	if (ret != 0x26) {
		dev_err(&client->dev,
			"novatek_touchscreen_read_chipid is not 0x26. ret=0x%02X\n",
			ret);
		ret = -EINVAL;
		goto err_chipid_failed;
	}

	//---check chip version---
	ret = novatek_touchscreen_chip_version();

	mutex_init(&ts->lock);

	//---create workqueue---
	novatek_workqueue = create_workqueue("novatek_workqueue");
	if (!novatek_workqueue) {
		dev_err(&client->dev,
			"%s: novatek_workqueue create workqueue failed\n",
			__func__);
		ret = -ENOMEM;
		goto err_create_novatek_workqueue_failed;
	}
	INIT_WORK(&ts->novatek_work, novatek_touchscreen_work_func);

	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		dev_err(&client->dev, "%s: allocate input device failed\n",
			__func__);
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->abs_x_max = TOUCH_MAX_WIDTH;
	ts->abs_y_max = TOUCH_MAX_HEIGHT;
	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

	ts->int_trigger_type = INT_TRIGGER_TYPE;

	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |
				  BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0,
			     0); //pressure = 255

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0,
			     0); //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max,
			     0, 0);
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,
			     ts->max_touch_num, 0, 0);
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NOVATEK_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,
			"register input device (%s) failed. ret=%d\n",
			ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	//---set int-pin & request irq---
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		dev_info(&client->dev, "int_trigger_type=%d\n",
			 ts->int_trigger_type);

		ret = request_irq(client->irq, novatek_touchscreen_irq_handler,
				  ts->int_trigger_type, client->name, ts);
		if (ret != 0) {
			dev_err(&client->dev, "request irq failed. ret=%d\n",
				ret);
			goto err_int_request_failed;
		} else {
			disable_irq(client->irq);
			dev_info(&client->dev, "request irq %d succeed\n",
				 client->irq);
		}
	}

	mutex_lock(&ts->lock);
	novatek_hw_reset();
	msleep(5);
	novatek_bootloader_reset();
	novatek_check_fw_reset_state(RESET_STATE_INIT);
	mutex_unlock(&ts->lock);

	//---set device node---
	ret = novatek_flash_proc_init();
	if (ret != 0) {
		dev_err(&client->dev, "nvt flash proc init failed. ret=%d\n",
			ret);
		goto err_init_NOVATEK_ts;
	}

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret) {
		dev_err(&client->dev, "register fb_notifier failed. ret=%d\n",
			ret);
		goto err_register_fb_notif_failed;
	}

	enable_irq(client->irq);
	bTouchIsAwake = 1;
	dev_info(&client->dev, "%s: finished\n", __func__);

	return 0;

err_register_fb_notif_failed:
err_init_NOVATEK_ts:
	free_irq(client->irq, ts);
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_create_novatek_workqueue_failed:
	mutex_destroy(&ts->lock);
err_chipid_failed:
err_check_functionality_failed:
	novatek_power_off(ts);
err_novatek_power_on:
	novatek_power_remove(ts);
err_power_init:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	BBOX_TP_PROBE_FAIL
	return ret;
}

static void novatek_touchscreen_remove(struct i2c_client *client)
{
	//struct novatek_touchscreen_data *ts = i2c_get_clientdata(client);

	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&ts->client->dev,
			"Error occurred while unregistering fb_notifier.\n");

	mutex_destroy(&ts->lock);

	dev_info(&client->dev, "%s: removing driver...\n", __func__);

	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);
}

static int32_t novatek_touchscreen_suspend(struct device *dev)
{
	uint8_t buf[4] = { 0 };

	if (!bTouchIsAwake) {
		dev_info(dev, "%s: Touch is already suspend\n", __func__);
		return 0;
	}

	mutex_lock(&ts->lock);

	dev_info(dev, "%s: begin...\n", __func__);

	bTouchIsAwake = 0;

	disable_irq(ts->client->irq);

	//---dummy read to resume TP before writing command---
	CTP_I2C_READ_DUMMY(ts->client, I2C_FW_Address);

	//---write i2c command to enter "deep sleep mode"---
	buf[0] = 0x50;
	buf[1] = 0x12;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

	msleep(50);

	novatek_power_off(ts);

	mutex_unlock(&ts->lock);

	dev_info(dev, "%s: end\n", __func__);

	return 0;
}

static int32_t novatek_touchscreen_resume(struct device *dev)
{
	if (bTouchIsAwake) {
		dev_info(dev, "%s: Touch is already resume\n", __func__);
		return 0;
	}

	mutex_lock(&ts->lock);

	dev_info(dev, "%s: begin...\n", __func__);

	if (novatek_power_on(ts)) {
		dev_err(dev, "nvt power on failed\n");
		goto err_novatek_power_on;
	}
	msleep(10);

	novatek_hw_reset();
	novatek_bootloader_reset();
	novatek_check_fw_reset_state(RESET_STATE_INIT);
	enable_irq(ts->client->irq);

	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);

	dev_info(dev, "%s: end\n", __func__);

	return 0;

err_novatek_power_on:
	novatek_power_remove(ts);
	return -1;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct novatek_touchscreen_data *ts =
		container_of(self, struct novatek_touchscreen_data, fb_notif);

	if (evdata && evdata->data && event == 0x10) {
		pr_debug("[HL]%s: event == FB_EARLY_EVENT_BLANK\n", __func__);
		blank = evdata->data;
		pr_debug("[HL]%s: *blank = (%d)\n", __func__, *blank);
		if ((*blank == FB_BLANK_UNBLANK) ||
		    (*blank == FB_BLANK_NORMAL)) {
			pr_debug(
				"[HL]%s: (*blank == FB_BLANK_UNBLANK) || (*blank == FB_BLANK_NORMAL)\n",
				__func__);
		} else //FB_BLANK_VSYNC_SUSPEND or FB_BLANK_HSYNC_SUSPEND or FB_BLANK_POWERDOWN
		{
			pr_debug(
				"[HL]%s: *blank = FB_BLANK_VSYNC_SUSPEND or FB_BLANK_HSYNC_SUSPEND or FB_BLANK_POWERDOWN\n",
				__func__);
			novatek_touchscreen_suspend(&ts->client->dev);
		}
	} else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		pr_debug("[HL]%s: event == FB_EVENT_BLANK\n", __func__);
		blank = evdata->data;
		pr_debug("[HL]%s: *blank = (%d)\n", __func__, *blank);
		if ((*blank == FB_BLANK_UNBLANK) ||
		    (*blank == FB_BLANK_NORMAL)) {
			pr_debug(
				"[HL]%s: (*blank == FB_BLANK_UNBLANK) || (*blank == FB_BLANK_NORMAL)\n",
				__func__);
			novatek_touchscreen_resume(&ts->client->dev);
		} else //FB_BLANK_VSYNC_SUSPEND or FB_BLANK_HSYNC_SUSPEND or FB_BLANK_POWERDOWN
		{
			pr_debug(
				"[HL]%s: *blank = FB_BLANK_VSYNC_SUSPEND or FB_BLANK_HSYNC_SUSPEND or FB_BLANK_POWERDOWN\n",
				__func__);
		}
	}

	return 0;
}

// CODIGO REESCRITO ABAJO

static const struct i2c_device_id novatek_touchscreen_id[] = {
	{ NOVATEK_I2C_NAME, 0 },
	{}
};

static struct of_device_id novatek_match_table[] = {
	{
		.compatible = "novatek,NVT-ts",
	},
	{},
};

static struct i2c_driver novatek_i2c_driver = {
	.probe		= novatek_touchscreen_probe,
	.remove		= novatek_touchscreen_remove,
//	.suspend	= novatek_touchscreen_suspend,
//	.resume		= novatek_touchscreen_resume,
	.id_table	= novatek_touchscreen_id,
	.driver = {
		.name	= NOVATEK_I2C_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = novatek_match_table,
	},
};

static int32_t __init novatek_driver_init(void)
{
	int32_t ret = 0;

	ret = i2c_add_driver(&novatek_i2c_driver);
	if (ret) {
		pr_err("%s: failed to add i2c driver", __func__);
		return ret;
	}

	pr_info("%s: finished\n", __func__);
	return ret;
}

static void __exit novatek_driver_exit(void)
{
	i2c_del_driver(&novatek_i2c_driver);

	if (novatek_workqueue)
		destroy_workqueue(novatek_workqueue);

	novatek_power_off(ts);
	novatek_power_remove(ts);
}

module_init(novatek_driver_init);
module_exit(novatek_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
