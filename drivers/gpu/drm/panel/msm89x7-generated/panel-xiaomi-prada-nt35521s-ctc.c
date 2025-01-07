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

struct nt35521s_ctc {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline struct nt35521s_ctc *to_nt35521s_ctc(struct drm_panel *panel)
{
	return container_of(panel, struct nt35521s_ctc, panel);
}

static void nt35521s_ctc_reset(struct nt35521s_ctc *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static int nt35521s_ctc_on(struct nt35521s_ctc *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xee, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x00ff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x2c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x64);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0x25, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x1d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0x25, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0x07);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 5000, 6000);

	return dsi_ctx.accum_err;
}

static int nt35521s_ctc_off(struct nt35521s_ctc *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 5000, 6000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	return dsi_ctx.accum_err;
}

static int nt35521s_ctc_prepare(struct drm_panel *panel)
{
	struct nt35521s_ctc *ctx = to_nt35521s_ctc(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	nt35521s_ctc_reset(ctx);

	ret = nt35521s_ctc_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int nt35521s_ctc_unprepare(struct drm_panel *panel)
{
	struct nt35521s_ctc *ctx = to_nt35521s_ctc(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = nt35521s_ctc_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode nt35521s_ctc_mode = {
	.clock = (720 + 150 + 20 + 146) * (1280 + 35 + 2 + 28) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 150,
	.hsync_end = 720 + 150 + 20,
	.htotal = 720 + 150 + 20 + 146,
	.vdisplay = 1280,
	.vsync_start = 1280 + 35,
	.vsync_end = 1280 + 35 + 2,
	.vtotal = 1280 + 35 + 2 + 28,
	.width_mm = 62,
	.height_mm = 110,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int nt35521s_ctc_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &nt35521s_ctc_mode);
}

static const struct drm_panel_funcs nt35521s_ctc_panel_funcs = {
	.prepare = nt35521s_ctc_prepare,
	.unprepare = nt35521s_ctc_unprepare,
	.get_modes = nt35521s_ctc_get_modes,
};

static int nt35521s_ctc_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt35521s_ctc *ctx;
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
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &nt35521s_ctc_panel_funcs,
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

static void nt35521s_ctc_remove(struct mipi_dsi_device *dsi)
{
	struct nt35521s_ctc *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt35521s_ctc_of_match[] = {
	{ .compatible = "xiaomi,prada-nt35521s-ctc" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt35521s_ctc_of_match);

static struct mipi_dsi_driver nt35521s_ctc_driver = {
	.probe = nt35521s_ctc_probe,
	.remove = nt35521s_ctc_remove,
	.driver = {
		.name = "panel-xiaomi-prada-nt35521s-ctc",
		.of_match_table = nt35521s_ctc_of_match,
	},
};
module_mipi_dsi_driver(nt35521s_ctc_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for CTC_nt35521s_720p_video_mode_dsi_panel");
MODULE_LICENSE("GPL");
