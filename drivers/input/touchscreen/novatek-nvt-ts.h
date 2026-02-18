/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __NOVATEK_NVT_TS_H_
#define __NOVATEK_NVT_TS_H_

#include <drm/drm_panel.h>

#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define NVT_TS_TOUCH_START		0x00
#define NVT_TS_TOUCH_SIZE		6

#define NVT_TS_HOST_COMMAND_START	0x50

#define NVT_TS_RESET_STATUS_START	0x60

#define NVT_TS_PARAMETERS_START		0x78
/* These are offsets from NVT_TS_PARAMETERS_START */
#define NVT_TS_PARAMS_WIDTH		0x04
#define NVT_TS_PARAMS_HEIGHT		0x06
#define NVT_TS_PARAMS_MAX_TOUCH		0x09
#define NVT_TS_PARAMS_MAX_BUTTONS	0x0a
#define NVT_TS_PARAMS_IRQ_TYPE		0x0b
#define NVT_TS_PARAMS_WAKE_TYPE		0x0c
#define NVT_TS_PARAMS_CHIP_ID		0x0e
#define NVT_TS_PARAMS_SIZE		0x0f

#define NVT_TS_MAX_TOUCHES		10
#define NVT_TS_MAX_SIZE			4096

#define NVT_TS_TOUCH_INVALID		0xff
#define NVT_TS_TOUCH_SLOT_SHIFT		3
#define NVT_TS_TOUCH_TYPE_MASK		GENMASK(2, 0)
#define NVT_TS_TOUCH_NEW		1
#define NVT_TS_TOUCH_UPDATE		2
#define NVT_TS_TOUCH_RELEASE		3

struct nvt_ts_data;

struct nvt_ts_memory_map {
	u32 event_buffer;
	u32 software_reset_n8;
	u32 spi_rd_fast;
	u32 bootloader_crc_enable;
	u32 boot_ready;
	u32 eng_reset;
	u32 por_cd;
	struct {
		u32 ilm;
		u32 dlm;
	} length;
	struct {
		u32 ilm;
		u32 dlm;
	} destination;
	struct {
		u32 ilm;
		u32 dlm;
	} golden_checksum;
};

typedef int (*nvt_ts_upload_firmware_impl)(struct nvt_ts_data *data,
					   const struct firmware *firmware);

struct nvt_ts_data {
	struct device *dev;
	struct regmap *regmap;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data regulators[2];
	struct touchscreen_properties prop;
	int max_touches;
	int irq;
	nvt_ts_upload_firmware_impl upload_firmware;
	const struct nvt_ts_memory_map *memory_map;
	u32 event_addr;
	struct drm_panel_follower panel_follower;
	struct work_struct panel_follower_prepare_work;
	bool prepare_work_finished;
	const char *fw_name;
	u8 buf[NVT_TS_TOUCH_SIZE * NVT_TS_MAX_TOUCHES];
};

struct nvt_ts_chip_data {
	const struct nvt_ts_memory_map *memory_map;
	u8 wake_type;
	u8 chip_id;
};

int nvt_ts_probe(struct device *dev, int irq, struct regmap *regmap,
		 const struct input_id *id,
		 nvt_ts_upload_firmware_impl upload_firmware);

extern const struct dev_pm_ops nvt_ts_pm_ops;

#endif
