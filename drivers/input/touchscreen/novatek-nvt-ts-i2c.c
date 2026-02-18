// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Novatek touchscreen controller.
 *
 * Copyright (c) 2023 Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/i2c.h>

#include "novatek-nvt-ts.h"

static const struct regmap_config nvt_ts_i2c_regmap_conf = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct input_id nvt_ts_i2c_input_id = {
	.bustype = BUS_I2C,
};

static int nvt_ts_i2c_probe(struct i2c_client *i2c)
{
	struct regmap *regmap =
		devm_regmap_init_i2c(i2c, &nvt_ts_i2c_regmap_conf);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return nvt_ts_probe(&i2c->dev, i2c->irq, regmap, &nvt_ts_i2c_input_id,
			    NULL);
}

static const struct nvt_ts_chip_data nvt_nt11205_ts_data = {
	.wake_type = 0x05,
	.chip_id = 0x05,
};

static const struct nvt_ts_chip_data nvt_nt36672a_ts_data = {
	.wake_type = 0x01,
	.chip_id = 0x08,
};

static const struct of_device_id nvt_ts_i2c_of_match[] = {
	{ .compatible = "novatek,nt11205-ts", .data = &nvt_nt11205_ts_data },
	{ .compatible = "novatek,nt36672a-ts", .data = &nvt_nt36672a_ts_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nvt_ts_i2c_of_match);

static const struct i2c_device_id nvt_ts_i2c_id[] = {
	{ "nt11205-ts", (unsigned long)&nvt_nt11205_ts_data },
	{ "nt36672a-ts", (unsigned long)&nvt_nt36672a_ts_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, nvt_ts_i2c_id);

static struct i2c_driver nvt_ts_i2c_driver = {
	.driver = {
		.name	= "novatek-nvt-ts-i2c",
		.pm	= pm_sleep_ptr(&nvt_ts_pm_ops),
		.of_match_table = nvt_ts_i2c_of_match,
	},
	.probe = nvt_ts_i2c_probe,
	.id_table = nvt_ts_i2c_id,
};
module_i2c_driver(nvt_ts_i2c_driver);

MODULE_DESCRIPTION("Novatek i2c touchscreen driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
