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

struct mdss_dsi_pj050ia_nt35521s {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline
struct mdss_dsi_pj050ia_nt35521s *to_mdss_dsi_pj050ia_nt35521s(struct drm_panel *panel)
{
	return container_of_const(panel, struct mdss_dsi_pj050ia_nt35521s, panel);
}

static void mdss_dsi_pj050ia_nt35521s_reset(struct mdss_dsi_pj050ia_nt35521s *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int mdss_dsi_pj050ia_nt35521s_on(struct mdss_dsi_pj050ia_nt35521s *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0,
				     0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);

	return dsi_ctx.accum_err;
}

static int mdss_dsi_pj050ia_nt35521s_off(struct mdss_dsi_pj050ia_nt35521s *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 150);

	return dsi_ctx.accum_err;
}

static int mdss_dsi_pj050ia_nt35521s_prepare(struct drm_panel *panel)
{
	struct mdss_dsi_pj050ia_nt35521s *ctx = to_mdss_dsi_pj050ia_nt35521s(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	mdss_dsi_pj050ia_nt35521s_reset(ctx);

	ret = mdss_dsi_pj050ia_nt35521s_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int mdss_dsi_pj050ia_nt35521s_unprepare(struct drm_panel *panel)
{
	struct mdss_dsi_pj050ia_nt35521s *ctx = to_mdss_dsi_pj050ia_nt35521s(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = mdss_dsi_pj050ia_nt35521s_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode mdss_dsi_pj050ia_nt35521s_mode = {
	.clock = (720 + 100 + 20 + 100) * (1280 + 120 + 2 + 60) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 100,
	.hsync_end = 720 + 100 + 20,
	.htotal = 720 + 100 + 20 + 100,
	.vdisplay = 1280,
	.vsync_start = 1280 + 120,
	.vsync_end = 1280 + 120 + 2,
	.vtotal = 1280 + 120 + 2 + 60,
	.width_mm = 64,
	.height_mm = 117,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int mdss_dsi_pj050ia_nt35521s_get_modes(struct drm_panel *panel,
					       struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &mdss_dsi_pj050ia_nt35521s_mode);
}

static const struct drm_panel_funcs mdss_dsi_pj050ia_nt35521s_panel_funcs = {
	.prepare = mdss_dsi_pj050ia_nt35521s_prepare,
	.unprepare = mdss_dsi_pj050ia_nt35521s_unprepare,
	.get_modes = mdss_dsi_pj050ia_nt35521s_get_modes,
};

static int mdss_dsi_pj050ia_nt35521s_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct mdss_dsi_pj050ia_nt35521s *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct mdss_dsi_pj050ia_nt35521s, panel,
				   &mdss_dsi_pj050ia_nt35521s_panel_funcs,
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

static void mdss_dsi_pj050ia_nt35521s_remove(struct mipi_dsi_device *dsi)
{
	struct mdss_dsi_pj050ia_nt35521s *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id mdss_dsi_pj050ia_nt35521s_of_match[] = {
	{ .compatible = "nokia,nd1-nt35521s-pj050ia" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdss_dsi_pj050ia_nt35521s_of_match);

static struct mipi_dsi_driver mdss_dsi_pj050ia_nt35521s_driver = {
	.probe = mdss_dsi_pj050ia_nt35521s_probe,
	.remove = mdss_dsi_pj050ia_nt35521s_remove,
	.driver = {
		.name = "panel-nokia-nd1-nt35521s-pj050ia",
		.of_match_table = mdss_dsi_pj050ia_nt35521s_of_match,
	},
};
module_mipi_dsi_driver(mdss_dsi_pj050ia_nt35521s_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for PA050IA nt35521s 720p video mode dsi panel");
MODULE_LICENSE("GPL");
