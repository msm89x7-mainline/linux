// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct mdss_dsi_nt35521s {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline
struct mdss_dsi_nt35521s *to_mdss_dsi_nt35521s(struct drm_panel *panel)
{
	return container_of(panel, struct mdss_dsi_nt35521s, panel);
}

static void mdss_dsi_nt35521s_reset(struct mdss_dsi_nt35521s *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(25);
}

static int mdss_dsi_nt35521s_on(struct mdss_dsi_nt35521s *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	mipi_dsi_dcs_write_seq(dsi, 0xff, 0xaa, 0x55, 0x25, 0x01);
	mipi_dsi_dcs_write_seq(dsi, 0x6f, 0x16);
	mipi_dsi_dcs_write_seq(dsi, 0xf7, 0x10);
	mipi_dsi_dcs_write_seq(dsi, 0xff, 0xaa, 0x55, 0x25, 0x01);
	usleep_range(10000, 11000);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}
	msleep(105);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	usleep_range(15000, 16000);

	ret = mipi_dsi_dcs_set_display_brightness(dsi, 0x00ff);
	if (ret < 0) {
		dev_err(dev, "Failed to set display brightness: %d\n", ret);
		return ret;
	}

	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);

	return 0;
}

static int mdss_dsi_nt35521s_off(struct mdss_dsi_nt35521s *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(120);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}

	return 0;
}

static int mdss_dsi_nt35521s_prepare(struct drm_panel *panel)
{
	struct mdss_dsi_nt35521s *ctx = to_mdss_dsi_nt35521s(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	mdss_dsi_nt35521s_reset(ctx);

	ret = mdss_dsi_nt35521s_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int mdss_dsi_nt35521s_unprepare(struct drm_panel *panel)
{
	struct mdss_dsi_nt35521s *ctx = to_mdss_dsi_nt35521s(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = mdss_dsi_nt35521s_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode mdss_dsi_nt35521s_mode = {
	.clock = (720 + 112 + 8 + 112) * (1280 + 20 + 2 + 12) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 112,
	.hsync_end = 720 + 112 + 8,
	.htotal = 720 + 112 + 8 + 112,
	.vdisplay = 1280,
	.vsync_start = 1280 + 20,
	.vsync_end = 1280 + 20 + 2,
	.vtotal = 1280 + 20 + 2 + 12,
	.width_mm = 64,
	.height_mm = 115,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int mdss_dsi_nt35521s_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &mdss_dsi_nt35521s_mode);
}

static const struct drm_panel_funcs mdss_dsi_nt35521s_panel_funcs = {
	.prepare = mdss_dsi_nt35521s_prepare,
	.unprepare = mdss_dsi_nt35521s_unprepare,
	.get_modes = mdss_dsi_nt35521s_get_modes,
};

static int mdss_dsi_nt35521s_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct mdss_dsi_nt35521s *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 3;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &mdss_dsi_nt35521s_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void mdss_dsi_nt35521s_remove(struct mipi_dsi_device *dsi)
{
	struct mdss_dsi_nt35521s *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id mdss_dsi_nt35521s_of_match[] = {
	{ .compatible = "nokia,nd1-nt35521s" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdss_dsi_nt35521s_of_match);

static struct mipi_dsi_driver mdss_dsi_nt35521s_driver = {
	.probe = mdss_dsi_nt35521s_probe,
	.remove = mdss_dsi_nt35521s_remove,
	.driver = {
		.name = "panel-nokia-nd1-nt35521s",
		.of_match_table = mdss_dsi_nt35521s_of_match,
	},
};
module_mipi_dsi_driver(mdss_dsi_nt35521s_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for nt35521s 720p video mode dsi panel");
MODULE_LICENSE("GPL");
