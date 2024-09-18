// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct ili9881c_tianma_c3b {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	bool prepared;
};

static inline
struct ili9881c_tianma_c3b *to_ili9881c_tianma_c3b(struct drm_panel *panel)
{
	return container_of(panel, struct ili9881c_tianma_c3b, panel);
}

static void ili9881c_tianma_c3b_reset(struct ili9881c_tianma_c3b *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(80);
}

static int ili9881c_tianma_c3b_on(struct ili9881c_tianma_c3b *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x11, 0x00);
	msleep(120);
	mipi_dsi_dcs_write_seq(dsi, 0x29, 0x00);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x04);
	mipi_dsi_dcs_write_seq(dsi, 0x92, 0x0f);
	mipi_dsi_dcs_write_seq(dsi, 0x21, 0xb0);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x05);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x7f, 0x81);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x06);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x81);
	mipi_dsi_dcs_write_seq(dsi, 0x7f, 0x83);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x07);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x7f, 0x89);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x08);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x89);
	mipi_dsi_dcs_write_seq(dsi, 0x7f, 0x93);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x09);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x7f, 0x93);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x0a);
	mipi_dsi_dcs_write_seq(dsi, 0x00, 0x93);
	mipi_dsi_dcs_write_seq(dsi, 0x7f, 0xb3);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x00);

	return 0;
}

static int ili9881c_tianma_c3b_off(struct ili9881c_tianma_c3b *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x00);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(31);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(140);

	return 0;
}

static int ili9881c_tianma_c3b_prepare(struct drm_panel *panel)
{
	struct ili9881c_tianma_c3b *ctx = to_ili9881c_tianma_c3b(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	ili9881c_tianma_c3b_reset(ctx);

	ret = ili9881c_tianma_c3b_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int ili9881c_tianma_c3b_unprepare(struct drm_panel *panel)
{
	struct ili9881c_tianma_c3b *ctx = to_ili9881c_tianma_c3b(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = ili9881c_tianma_c3b_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode ili9881c_tianma_c3b_mode = {
	.clock = (720 + 112 + 8 + 160) * (1280 + 40 + 4 + 18) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 112,
	.hsync_end = 720 + 112 + 8,
	.htotal = 720 + 112 + 8 + 160,
	.vdisplay = 1280,
	.vsync_start = 1280 + 40,
	.vsync_end = 1280 + 40 + 4,
	.vtotal = 1280 + 40 + 4 + 18,
	.width_mm = 62,
	.height_mm = 110,
};

static int ili9881c_tianma_c3b_get_modes(struct drm_panel *panel,
					 struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ili9881c_tianma_c3b_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs ili9881c_tianma_c3b_panel_funcs = {
	.prepare = ili9881c_tianma_c3b_prepare,
	.unprepare = ili9881c_tianma_c3b_unprepare,
	.get_modes = ili9881c_tianma_c3b_get_modes,
};

static int ili9881c_tianma_c3b_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ili9881c_tianma_c3b *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->supplies[0].supply = "vsn";
	ctx->supplies[1].supply = "vsp";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &ili9881c_tianma_c3b_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void ili9881c_tianma_c3b_remove(struct mipi_dsi_device *dsi)
{
	struct ili9881c_tianma_c3b *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ili9881c_tianma_c3b_of_match[] = {
	{ .compatible = "xiaomi,riva-ili9881c-tianma" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ili9881c_tianma_c3b_of_match);

static struct mipi_dsi_driver ili9881c_tianma_c3b_driver = {
	.probe = ili9881c_tianma_c3b_probe,
	.remove = ili9881c_tianma_c3b_remove,
	.driver = {
		.name = "panel-xiaomi-riva-ili9881c-tianma",
		.of_match_table = ili9881c_tianma_c3b_of_match,
	},
};
module_mipi_dsi_driver(ili9881c_tianma_c3b_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for ili9881c_HD720p_video_tianma_c3b");
MODULE_LICENSE("GPL");
