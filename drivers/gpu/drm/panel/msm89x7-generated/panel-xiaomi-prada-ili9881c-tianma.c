// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2025 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct ili9881c_tianma {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline
struct ili9881c_tianma *to_ili9881c_tianma(struct drm_panel *panel)
{
	return container_of(panel, struct ili9881c_tianma, panel);
}

static void ili9881c_tianma_reset(struct ili9881c_tianma *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int ili9881c_tianma_on(struct ili9881c_tianma *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x00);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x21, 0xb0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x89);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x89);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x8c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x00);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0xff0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x98, 0x81, 0x00);

	return dsi_ctx.accum_err;
}

static int ili9881c_tianma_off(struct ili9881c_tianma *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int ili9881c_tianma_prepare(struct drm_panel *panel)
{
	struct ili9881c_tianma *ctx = to_ili9881c_tianma(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ili9881c_tianma_reset(ctx);

	ret = ili9881c_tianma_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int ili9881c_tianma_unprepare(struct drm_panel *panel)
{
	struct ili9881c_tianma *ctx = to_ili9881c_tianma(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = ili9881c_tianma_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode ili9881c_tianma_mode = {
	.clock = (720 + 124 + 10 + 180) * (1280 + 40 + 4 + 18) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 124,
	.hsync_end = 720 + 124 + 10,
	.htotal = 720 + 124 + 10 + 180,
	.vdisplay = 1280,
	.vsync_start = 1280 + 40,
	.vsync_end = 1280 + 40 + 4,
	.vtotal = 1280 + 40 + 4 + 18,
	.width_mm = 62,
	.height_mm = 110,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ili9881c_tianma_get_modes(struct drm_panel *panel,
				     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ili9881c_tianma_mode);
}

static const struct drm_panel_funcs ili9881c_tianma_panel_funcs = {
	.prepare = ili9881c_tianma_prepare,
	.unprepare = ili9881c_tianma_unprepare,
	.get_modes = ili9881c_tianma_get_modes,
};

static int ili9881c_tianma_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ili9881c_tianma *ctx;
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

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_VIDEO_NO_HBP;

	drm_panel_init(&ctx->panel, dev, &ili9881c_tianma_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

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

static void ili9881c_tianma_remove(struct mipi_dsi_device *dsi)
{
	struct ili9881c_tianma *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ili9881c_tianma_of_match[] = {
	{ .compatible = "xiaomi,prada-ili9881c-tianma" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ili9881c_tianma_of_match);

static struct mipi_dsi_driver ili9881c_tianma_driver = {
	.probe = ili9881c_tianma_probe,
	.remove = ili9881c_tianma_remove,
	.driver = {
		.name = "panel-xiaomi-prada-ili9881c-tianma",
		.of_match_table = ili9881c_tianma_of_match,
	},
};
module_mipi_dsi_driver(ili9881c_tianma_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for Tianma_ili9881c_720p_video_mode_dsi_panel");
MODULE_LICENSE("GPL");
