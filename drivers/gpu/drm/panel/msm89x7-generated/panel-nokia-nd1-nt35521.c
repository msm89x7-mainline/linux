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

struct mdss_dsi_nt35521 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline
struct mdss_dsi_nt35521 *to_mdss_dsi_nt35521(struct drm_panel *panel)
{
	return container_of_const(panel, struct mdss_dsi_nt35521, panel);
}

static void mdss_dsi_nt35521_reset(struct mdss_dsi_nt35521 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(25);
}

static int mdss_dsi_nt35521_on(struct mdss_dsi_nt35521 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int mdss_dsi_nt35521_off(struct mdss_dsi_nt35521 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int mdss_dsi_nt35521_prepare(struct drm_panel *panel)
{
	struct mdss_dsi_nt35521 *ctx = to_mdss_dsi_nt35521(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	mdss_dsi_nt35521_reset(ctx);

	ret = mdss_dsi_nt35521_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int mdss_dsi_nt35521_unprepare(struct drm_panel *panel)
{
	struct mdss_dsi_nt35521 *ctx = to_mdss_dsi_nt35521(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = mdss_dsi_nt35521_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode mdss_dsi_nt35521_mode = {
	.clock = (720 + 52 + 4 + 52) * (1280 + 50 + 1 + 50) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 52,
	.hsync_end = 720 + 52 + 4,
	.htotal = 720 + 52 + 4 + 52,
	.vdisplay = 1280,
	.vsync_start = 1280 + 50,
	.vsync_end = 1280 + 50 + 1,
	.vtotal = 1280 + 50 + 1 + 50,
	.width_mm = 62,
	.height_mm = 110,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int mdss_dsi_nt35521_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &mdss_dsi_nt35521_mode);
}

static const struct drm_panel_funcs mdss_dsi_nt35521_panel_funcs = {
	.prepare = mdss_dsi_nt35521_prepare,
	.unprepare = mdss_dsi_nt35521_unprepare,
	.get_modes = mdss_dsi_nt35521_get_modes,
};

static int mdss_dsi_nt35521_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct mdss_dsi_nt35521 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct mdss_dsi_nt35521, panel,
				   &mdss_dsi_nt35521_panel_funcs,
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

static void mdss_dsi_nt35521_remove(struct mipi_dsi_device *dsi)
{
	struct mdss_dsi_nt35521 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id mdss_dsi_nt35521_of_match[] = {
	{ .compatible = "nokia,nd1-nt35521" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdss_dsi_nt35521_of_match);

static struct mipi_dsi_driver mdss_dsi_nt35521_driver = {
	.probe = mdss_dsi_nt35521_probe,
	.remove = mdss_dsi_nt35521_remove,
	.driver = {
		.name = "panel-nokia-nd1-nt35521",
		.of_match_table = mdss_dsi_nt35521_of_match,
	},
};
module_mipi_dsi_driver(mdss_dsi_nt35521_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for nt35521 720p video mode dsi panel");
MODULE_LICENSE("GPL");
