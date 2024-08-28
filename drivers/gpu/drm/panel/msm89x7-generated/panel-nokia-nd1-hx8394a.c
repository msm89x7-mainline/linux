// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct mdss_dsi_hx8394a {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline
struct mdss_dsi_hx8394a *to_mdss_dsi_hx8394a(struct drm_panel *panel)
{
	return container_of_const(panel, struct mdss_dsi_hx8394a, panel);
}

static void mdss_dsi_hx8394a_reset(struct mdss_dsi_hx8394a *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static int mdss_dsi_hx8394a_on(struct mdss_dsi_hx8394a *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0xff, 0x83, 0x94);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xba,
				     0x13, 0x82, 0x00, 0x16, 0xc5, 0x00, 0x10,
				     0xff, 0x0f, 0x24, 0x03, 0x21, 0x24, 0x25,
				     0x20, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1,
				     0x01, 0x00, 0x04, 0x0b, 0x16, 0x20, 0x28,
				     0x2f, 0x36, 0x3f, 0x47, 0x4f, 0x57, 0x5f,
				     0x67, 0x6f, 0x76, 0x7e, 0x86, 0x8e, 0x96,
				     0x9e, 0xa6, 0xae, 0xb6, 0xbe, 0xc6, 0xce,
				     0xd6, 0xde, 0xe6, 0xee, 0xf8, 0xff, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x08, 0x10, 0x17, 0x21, 0x29,
				     0x30, 0x37, 0x40, 0x48, 0x51, 0x58, 0x60,
				     0x67, 0x70, 0x76, 0x7e, 0x86, 0x8e, 0x96,
				     0x9e, 0xa6, 0xae, 0xb6, 0xbe, 0xc6, 0xce,
				     0xd6, 0xde, 0xe6, 0xee, 0xf8, 0xff, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x08, 0x13, 0x18, 0x21, 0x29,
				     0x30, 0x37, 0x40, 0x48, 0x51, 0x59, 0x61,
				     0x69, 0x72, 0x79, 0x81, 0x89, 0x92, 0x9a,
				     0xa2, 0xab, 0xb3, 0xbb, 0xc3, 0xcc, 0xd3,
				     0xdb, 0xe3, 0xeb, 0xf1, 0xf8, 0xff, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1,
				     0x01, 0x00, 0x37, 0x87, 0x01, 0x11, 0x11,
				     0x22, 0x2a, 0x19, 0x3f, 0x57, 0x02, 0x00,
				     0xe6, 0xe2);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0x00, 0xc8, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbf, 0x06, 0x00, 0x10, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb4,
				     0x80, 0x08, 0x32, 0x10, 0x0d, 0x32, 0x10,
				     0x08, 0x22, 0x10, 0x08, 0x37, 0x04, 0x4a,
				     0x11, 0x37, 0x04, 0x40, 0x06, 0x5a, 0x5a,
				     0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd5,
				     0x00, 0x00, 0x00, 0x10, 0x0a, 0x00, 0x01,
				     0x33, 0x00, 0x00, 0x33, 0x00, 0x23, 0x01,
				     0x67, 0x45, 0x01, 0x23, 0x88, 0x88, 0x88,
				     0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
				     0x88, 0x45, 0x99, 0x88, 0x54, 0x76, 0x10,
				     0x32, 0x32, 0x10, 0x88, 0x88, 0x88, 0x88,
				     0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
				     0x54, 0x88, 0x99);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7, 0x00, 0x10, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0,
				     0x00, 0x0a, 0x14, 0x39, 0x3e, 0x3f, 0x24,
				     0x44, 0x04, 0x0b, 0x0f, 0x13, 0x14, 0x14,
				     0x15, 0x18, 0x1c, 0x00, 0x0a, 0x14, 0x39,
				     0x3e, 0x3f, 0x24, 0x44, 0x04, 0x0b, 0x0f,
				     0x13, 0x16, 0x14, 0x15, 0x18, 0x1c, 0x08,
				     0x17, 0x0a, 0x1a, 0x08, 0x17, 0x0a, 0x1a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x0c, 0x17);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcc, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd4, 0x32);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb6, 0x24);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VHBLANK);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 125);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int mdss_dsi_hx8394a_off(struct mdss_dsi_hx8394a *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 125);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int mdss_dsi_hx8394a_prepare(struct drm_panel *panel)
{
	struct mdss_dsi_hx8394a *ctx = to_mdss_dsi_hx8394a(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	mdss_dsi_hx8394a_reset(ctx);

	ret = mdss_dsi_hx8394a_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int mdss_dsi_hx8394a_unprepare(struct drm_panel *panel)
{
	struct mdss_dsi_hx8394a *ctx = to_mdss_dsi_hx8394a(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = mdss_dsi_hx8394a_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode mdss_dsi_hx8394a_mode = {
	.clock = (720 + 96 + 40 + 92) * (1280 + 8 + 4 + 15) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 96,
	.hsync_end = 720 + 96 + 40,
	.htotal = 720 + 96 + 40 + 92,
	.vdisplay = 1280,
	.vsync_start = 1280 + 8,
	.vsync_end = 1280 + 8 + 4,
	.vtotal = 1280 + 8 + 4 + 15,
	.width_mm = 62,
	.height_mm = 110,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int mdss_dsi_hx8394a_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &mdss_dsi_hx8394a_mode);
}

static const struct drm_panel_funcs mdss_dsi_hx8394a_panel_funcs = {
	.prepare = mdss_dsi_hx8394a_prepare,
	.unprepare = mdss_dsi_hx8394a_unprepare,
	.get_modes = mdss_dsi_hx8394a_get_modes,
};

static int mdss_dsi_hx8394a_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct mdss_dsi_hx8394a *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct mdss_dsi_hx8394a, panel,
				   &mdss_dsi_hx8394a_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

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
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

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

static void mdss_dsi_hx8394a_remove(struct mipi_dsi_device *dsi)
{
	struct mdss_dsi_hx8394a *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id mdss_dsi_hx8394a_of_match[] = {
	{ .compatible = "nokia,nd1-hx8394a" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdss_dsi_hx8394a_of_match);

static struct mipi_dsi_driver mdss_dsi_hx8394a_driver = {
	.probe = mdss_dsi_hx8394a_probe,
	.remove = mdss_dsi_hx8394a_remove,
	.driver = {
		.name = "panel-nokia-nd1-hx8394a",
		.of_match_table = mdss_dsi_hx8394a_of_match,
	},
};
module_mipi_dsi_driver(mdss_dsi_hx8394a_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for hx8394a 720p video mode dsi panel");
MODULE_LICENSE("GPL");
