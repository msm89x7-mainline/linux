// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct sc_ili9881c {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *backlight_gpio;
	bool prepared;
};

static inline struct sc_ili9881c *to_sc_ili9881c(struct drm_panel *panel)
{
	return container_of(panel, struct sc_ili9881c, panel);
}

static void sc_ili9881c_reset(struct sc_ili9881c *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int sc_ili9881c_on(struct sc_ili9881c *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x11, 0x00);
	msleep(30);
	mipi_dsi_dcs_write_seq(dsi, 0x29, 0x00);
	usleep_range(10000, 11000);
	mipi_dsi_dcs_write_seq(dsi, 0x68, 0x04, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x35, 0x00);
	mipi_dsi_dcs_write_seq(dsi, 0x53, 0x2c);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x02);
	mipi_dsi_dcs_write_seq(dsi, 0x06, 0x40);
	mipi_dsi_dcs_write_seq(dsi, 0x07, 0x05);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x03);
	mipi_dsi_dcs_write_seq(dsi, 0x2c, 0x0c);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0x98, 0x81, 0x00);

	return 0;
}

static int sc_ili9881c_off(struct sc_ili9881c *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	usleep_range(16000, 17000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}
	msleep(120);

	return 0;
}

static int sc_ili9881c_prepare(struct drm_panel *panel)
{
	struct sc_ili9881c *ctx = to_sc_ili9881c(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	sc_ili9881c_reset(ctx);

	ret = sc_ili9881c_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int sc_ili9881c_unprepare(struct drm_panel *panel)
{
	struct sc_ili9881c *ctx = to_sc_ili9881c(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = sc_ili9881c_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	ctx->prepared = false;
	return 0;
}

static const struct drm_display_mode sc_ili9881c_mode = {
	.clock = (720 + 100 + 10 + 100) * (1280 + 36 + 10 + 36) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 100,
	.hsync_end = 720 + 100 + 10,
	.htotal = 720 + 100 + 10 + 100,
	.vdisplay = 1280,
	.vsync_start = 1280 + 36,
	.vsync_end = 1280 + 36 + 10,
	.vtotal = 1280 + 36 + 10 + 36,
	.width_mm = 68,
	.height_mm = 121,
};

static int sc_ili9881c_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &sc_ili9881c_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs sc_ili9881c_panel_funcs = {
	.prepare = sc_ili9881c_prepare,
	.unprepare = sc_ili9881c_unprepare,
	.get_modes = sc_ili9881c_get_modes,
};

static int sc_ili9881c_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct sc_ili9881c *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	gpiod_set_value_cansleep(ctx->backlight_gpio, !!brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops sc_ili9881c_bl_ops = {
	.update_status = sc_ili9881c_bl_update_status,
};

static struct backlight_device *
sc_ili9881c_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &sc_ili9881c_bl_ops, &props);
}

static int sc_ili9881c_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct sc_ili9881c *ctx;
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

	ctx->backlight_gpio = devm_gpiod_get(dev, "backlight", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->backlight_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->backlight_gpio),
				     "Failed to get backlight-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_HSE |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel, dev, &sc_ili9881c_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->panel.backlight = sc_ili9881c_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void sc_ili9881c_remove(struct mipi_dsi_device *dsi)
{
	struct sc_ili9881c *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id sc_ili9881c_of_match[] = {
	{ .compatible = "xiaomi,ugglite-ili9881c-sc" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sc_ili9881c_of_match);

static struct mipi_dsi_driver sc_ili9881c_driver = {
	.probe = sc_ili9881c_probe,
	.remove = sc_ili9881c_remove,
	.driver = {
		.name = "panel-xiaomi-ugglite-ili9881c-sc",
		.of_match_table = sc_ili9881c_of_match,
	},
};
module_mipi_dsi_driver(sc_ili9881c_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for shenchao ili9881c 720p video mode dsi panel");
MODULE_LICENSE("GPL");
