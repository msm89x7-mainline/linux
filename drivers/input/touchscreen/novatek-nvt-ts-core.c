// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Novatek touchscreen controller.
 *
 * Copyright (c) 2023 Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/unaligned.h>

#include "novatek-nvt-ts.h"

static const int nvt_ts_irq_type[4] = {
	IRQF_TRIGGER_RISING,
	IRQF_TRIGGER_FALLING,
	IRQF_TRIGGER_LOW,
	IRQF_TRIGGER_HIGH
};

static irqreturn_t nvt_ts_irq(int irq, void *dev_id)
{
	struct nvt_ts_data *data = dev_id;
	const struct device *dev = data->dev;
	int i, error, slot, x, y;
	bool active;
	u8 *touch;

	error = regmap_raw_read(data->regmap,
				data->event_addr + NVT_TS_TOUCH_START,
				data->buf,
				data->max_touches * NVT_TS_TOUCH_SIZE);
	if (error)
		return IRQ_HANDLED;

	for (i = 0; i < data->max_touches; i++) {
		touch = &data->buf[i * NVT_TS_TOUCH_SIZE];

		if (touch[0] == NVT_TS_TOUCH_INVALID)
			continue;

		slot = touch[0] >> NVT_TS_TOUCH_SLOT_SHIFT;
		if (slot < 1 || slot > data->max_touches) {
			dev_warn(dev, "slot %d out of range, ignoring\n", slot);
			continue;
		}

		switch (touch[0] & NVT_TS_TOUCH_TYPE_MASK) {
		case NVT_TS_TOUCH_NEW:
		case NVT_TS_TOUCH_UPDATE:
			active = true;
			break;
		case NVT_TS_TOUCH_RELEASE:
			active = false;
			break;
		default:
			dev_warn(dev, "slot %d unknown state %d\n", slot, touch[0] & 7);
			continue;
		}

		slot--;
		x = (touch[1] << 4) | (touch[3] >> 4);
		y = (touch[2] << 4) | (touch[3] & 0x0f);

		input_mt_slot(data->input, slot);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, active);
		touchscreen_report_pos(data->input, &data->prop, x, y, true);
	}

	input_mt_sync_frame(data->input);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static int nvt_ts_upload_firmware(struct nvt_ts_data *data)
{
	const struct firmware *firmware = NULL;
	struct device *dev = data->dev;
	int error;

	if (!data->upload_firmware || !data->fw_name)
		return 0;

	error = request_firmware(&firmware, data->fw_name, dev);
	if (error) {
		dev_err(dev, "unable to load firmware: %d\n", error);
		return error;
	}

	error = data->upload_firmware(data, firmware);

	release_firmware(firmware);
	return error;
}

static int nvt_ts_start(struct input_dev *dev)
{
	struct nvt_ts_data *data = input_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(data->regulators), data->regulators);
	if (ret) {
		dev_err(data->dev, "failed to enable regulators\n");
		return ret;
	}

	enable_irq(data->irq);
	gpiod_set_value_cansleep(data->reset_gpio, 0);

	ret = nvt_ts_upload_firmware(data);
	if (ret) {
		dev_err(data->dev, "failed to upload firmware: %d\n", ret);
		disable_irq(data->irq);
		gpiod_set_value_cansleep(data->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
		return ret;
	}

	return 0;
}

static void nvt_ts_stop(struct input_dev *dev)
{
	struct nvt_ts_data *data = input_get_drvdata(dev);

	disable_irq(data->irq);
	gpiod_set_value_cansleep(data->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
}

static void nvt_ts_panel_prepare_work(struct work_struct *work)
{
	struct nvt_ts_data *data = container_of(work, struct nvt_ts_data, panel_follower_prepare_work);

	int ret = nvt_ts_start(data->input);
	if (!ret) {
		WRITE_ONCE(data->prepare_work_finished, true);
		smp_wmb();
	}
}

static int nvt_ts_input_stop(const struct nvt_ts_data *data)
{
	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		nvt_ts_stop(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int nvt_ts_suspend(struct device *dev)
{
	struct nvt_ts_data *data = dev_get_drvdata(dev);

	if (drm_is_panel_follower(dev))
		return 0;

	cancel_work_sync(&data->panel_follower_prepare_work);

	smp_rmb();
	if (!READ_ONCE(data->prepare_work_finished))
		return 0;

	return nvt_ts_input_stop(data);
}

static int nvt_ts_resume(struct device *dev)
{
	struct nvt_ts_data *data = dev_get_drvdata(dev);

	if (drm_is_panel_follower(dev))
		return 0;

	WRITE_ONCE(data->prepare_work_finished, false);
	schedule_work(&data->panel_follower_prepare_work);

	return 0;
}

static int panel_prepared(struct drm_panel_follower *follower)
{
	struct nvt_ts_data *data = container_of(follower, struct nvt_ts_data, panel_follower);

	WRITE_ONCE(data->prepare_work_finished, false);
	schedule_work(&data->panel_follower_prepare_work);

	return 0;
}

static int panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nvt_ts_data *data = container_of(follower, struct nvt_ts_data, panel_follower);

	cancel_work_sync(&data->panel_follower_prepare_work);

	smp_rmb();
	if (!READ_ONCE(data->prepare_work_finished))
		return 0;

	return nvt_ts_input_stop(data);
}

static struct drm_panel_follower_funcs nvt_ts_panel_follower_funcs = {
	.panel_prepared = panel_prepared,
	.panel_unpreparing = panel_unpreparing,
};

EXPORT_GPL_SIMPLE_DEV_PM_OPS(nvt_ts_pm_ops, nvt_ts_suspend, nvt_ts_resume);

int nvt_ts_probe(struct device *dev, int irq, struct regmap *regmap,
		 const struct input_id *id,
		 nvt_ts_upload_firmware_impl upload_firmware)
{
	int error, width, height, irq_type;
	struct nvt_ts_data *data;
	const struct nvt_ts_chip_data *chip;
	struct input_dev *input;

	if (!irq)
		return dev_err_probe(dev, -EINVAL, "missing irq\n");

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	chip = device_get_match_data(dev);
	if (!chip)
		return -EINVAL;

	if (upload_firmware) {
		error = device_property_read_string(dev, "firmware-name", &data->fw_name);
		if (error)
			return dev_err_probe(dev, error, "unable to get firmware name\n");
	}

	data->dev = dev;
	data->regmap = regmap;
	data->irq = irq;
	data->upload_firmware = upload_firmware;
	data->memory_map = chip->memory_map;
	data->event_addr = chip->memory_map ? chip->memory_map->event_buffer : 0;
	dev_set_drvdata(dev, data);

	/*
	 * VCC is the analog voltage supply
	 * IOVCC is the digital voltage supply
	 */
	data->regulators[0].supply = "vcc";
	data->regulators[1].supply = "iovcc";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->regulators), data->regulators);
	if (error)
		return dev_err_probe(dev, error, "cannot get regulators\n");

	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error)
		return dev_err_probe(dev, error, "failed to enable regulators\n");

	data->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	error = PTR_ERR_OR_ZERO(data->reset_gpio);
	if (error) {
		regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
		return dev_err_probe(dev, error, "failed to request reset GPIO\n");
	}

	/* Wait for power supplies to stabilize and chip to power on */
	msleep(50);

	error = nvt_ts_upload_firmware(data);
	if (error) {
		gpiod_set_value_cansleep(data->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
		return dev_err_probe(dev, error, "failed to upload firmware\n");
	}

	/* Wait for controller to come out of reset before params read */
	msleep(100);
	error = regmap_raw_read(data->regmap,
				data->event_addr + NVT_TS_PARAMETERS_START,
				data->buf, NVT_TS_PARAMS_SIZE);
	gpiod_set_value_cansleep(data->reset_gpio, 1); /* Put back in reset */
	regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
	if (error)
		return error;

	width  = get_unaligned_be16(&data->buf[NVT_TS_PARAMS_WIDTH]);
	height = get_unaligned_be16(&data->buf[NVT_TS_PARAMS_HEIGHT]);
	data->max_touches = data->buf[NVT_TS_PARAMS_MAX_TOUCH];
	irq_type = data->buf[NVT_TS_PARAMS_IRQ_TYPE];

	if (width > NVT_TS_MAX_SIZE || height >= NVT_TS_MAX_SIZE ||
	    data->max_touches > NVT_TS_MAX_TOUCHES ||
	    irq_type >= ARRAY_SIZE(nvt_ts_irq_type) ||
	    data->buf[NVT_TS_PARAMS_WAKE_TYPE] != chip->wake_type ||
	    data->buf[NVT_TS_PARAMS_CHIP_ID] != chip->chip_id) {
		dev_err(dev, "Unsupported touchscreen parameters: %*ph\n",
			NVT_TS_PARAMS_SIZE, data->buf);
		return -EIO;
	}

	dev_dbg(dev, "Detected %dx%d touchscreen with %d max touches\n",
		width, height, data->max_touches);

	if (data->buf[NVT_TS_PARAMS_MAX_BUTTONS])
		dev_warn(dev, "Touchscreen buttons are not supported\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = dev_name(dev);
	input->id = *id;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, width - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, height - 1, 0, 0);
	touchscreen_parse_properties(input, true, &data->prop);

	error = input_mt_init_slots(input, data->max_touches,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	data->input = input;
	input_set_drvdata(input, data);

	error = devm_request_threaded_irq(dev, irq, NULL, nvt_ts_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN |
						nvt_ts_irq_type[irq_type],
					  dev_name(dev), data);
	if (error)
		return dev_err_probe(dev, error, "failed to request irq\n");

	if (drm_is_panel_follower(dev)) {
		INIT_WORK(&data->panel_follower_prepare_work, nvt_ts_panel_prepare_work);
		data->panel_follower.funcs = &nvt_ts_panel_follower_funcs;
		devm_drm_panel_add_follower(dev, &data->panel_follower);
	} else {
		input->open = nvt_ts_start;
		input->close = nvt_ts_stop;
	}

	error = input_register_device(input);
	if (error)
		return dev_err_probe(dev, error, "failed to register input device\n");

	return 0;
}
EXPORT_SYMBOL_GPL(nvt_ts_probe);

MODULE_DESCRIPTION("Novatek core touchscreen driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
