// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct nt35521s {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
};

static inline struct nt35521s *to_nt35521s(struct drm_panel *panel)
{
	return container_of_const(panel, struct nt35521s, panel);
}

static void nt35521s_reset(struct nt35521s *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int nt35521s_on(struct nt35521s *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x68, 0x07);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb6, 0x08);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb8, 0x08);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbb, 0x54, 0x44);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc, 0x05, 0x05);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc7, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd,
					 0x02, 0xb0, 0x1e, 0x1e, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc5, 0x01, 0x07);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc8, 0x83);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x05, 0x05);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x05, 0x05);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc, 0x90, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd, 0x90, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xca, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc0, 0x04);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3, 0x28, 0x28);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb4, 0x12, 0x12);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb9, 0x45, 0x45);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba, 0x14, 0x14);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xee, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xef,
					 0x09, 0x06, 0x15, 0x18);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0,
					 0x00, 0x00, 0x00, 0x0f, 0x00, 0x28);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0,
					 0x00, 0x47, 0x00, 0x58, 0x00, 0x76);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x0c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0,
					 0x00, 0x9b, 0x00, 0xcd);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1,
					 0x00, 0xfc, 0x01, 0x3e, 0x01, 0x7c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1,
					 0x01, 0xd3, 0x02, 0x1a, 0x02, 0x1c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x0c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1,
					 0x02, 0x5f, 0x02, 0xbf);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2,
					 0x02, 0xf6, 0x03, 0x34, 0x03, 0x57);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2,
					 0x03, 0x86, 0x03, 0x8f, 0x03, 0xac);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x0c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2,
					 0x03, 0xbd, 0x03, 0xe4);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3,
					 0x03, 0xf7, 0x03, 0xff);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2, 0x2e, 0x09);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3, 0x2a, 0x29);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb4, 0x1b, 0x19);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x17, 0x15);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb6, 0x13, 0x11);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb7, 0x01, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb8, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb9, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbb, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc, 0x2e, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd, 0x10, 0x12);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbe, 0x14, 0x16);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbf, 0x18, 0x1a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc0, 0x29, 0x2a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc1, 0x08, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc2, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc3, 0x2e, 0x0a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe5, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc4, 0x0a, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc5, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc6, 0x2e, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc7, 0x2a, 0x29);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc8, 0x10, 0x12);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc9, 0x14, 0x16);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xca, 0x18, 0x1a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcb, 0x08, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcc, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcd, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xce, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcf, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd0, 0x2e, 0x09);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd1, 0x1b, 0x19);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd2, 0x17, 0x15);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd3, 0x13, 0x11);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd4, 0x29, 0x2a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd5, 0x01, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd6, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd7, 0x2e, 0x0b);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe6, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd8,
					 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd9,
					 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe7, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x20, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x20, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2,
					 0x05, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb6,
					 0x05, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb7,
					 0x05, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba,
					 0x57, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbb,
					 0x57, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc0,
					 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc1,
					 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc4, 0x60);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc5, 0x40);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x05);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd,
					 0x03, 0x01, 0x03, 0x03, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x17, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x17, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2, 0x17, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3, 0x17, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb4, 0x17, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x17, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb8, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb9, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbb, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc0, 0x07);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc4, 0x80);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc5, 0xa4);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc8, 0x05, 0x30);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc9, 0x01, 0x31);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcc, 0x00, 0x00, 0x3c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcd, 0x00, 0x00, 0x3c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd1,
					 0x00, 0x05, 0x09, 0x07, 0x10);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd2,
					 0x00, 0x05, 0x0e, 0x07, 0x10);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe5, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe6, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe7, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe8, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe9, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xea, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xed, 0x30);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x6f, 0x11);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf3, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x51, 0xff);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x53, 0x2c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int nt35521s_off(struct nt35521s *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int nt35521s_prepare(struct drm_panel *panel)
{
	struct nt35521s *ctx = to_nt35521s(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	nt35521s_reset(ctx);

	ret = nt35521s_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->supply);
		return ret;
	}

	return 0;
}

static int nt35521s_unprepare(struct drm_panel *panel)
{
	struct nt35521s *ctx = to_nt35521s(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = nt35521s_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->supply);

	return 0;
}

static const struct drm_display_mode nt35521s_mode = {
	.clock = (800 + 80 + 4 + 80) * (1280 + 20 + 4 + 16) * 60 / 1000,
	.hdisplay = 800,
	.hsync_start = 800 + 80,
	.hsync_end = 800 + 80 + 4,
	.htotal = 800 + 80 + 4 + 80,
	.vdisplay = 1280,
	.vsync_start = 1280 + 20,
	.vsync_end = 1280 + 20 + 4,
	.vtotal = 1280 + 20 + 4 + 16,
	.width_mm = 0,
	.height_mm = 0,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int nt35521s_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &nt35521s_mode);
}

static const struct drm_panel_funcs nt35521s_panel_funcs = {
	.prepare = nt35521s_prepare,
	.unprepare = nt35521s_unprepare,
	.get_modes = nt35521s_get_modes,
};

static int nt35521s_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt35521s *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct nt35521s, panel,
				   &nt35521s_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "Failed to get power regulator\n");

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

static void nt35521s_remove(struct mipi_dsi_device *dsi)
{
	struct nt35521s *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt35521s_of_match[] = {
	{ .compatible = "lenovo,tbx304x-nt35521s" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt35521s_of_match);

static struct mipi_dsi_driver nt35521s_driver = {
	.probe = nt35521s_probe,
	.remove = nt35521s_remove,
	.driver = {
		.name = "panel-lenovo-tbx304x-nt35521s",
		.of_match_table = nt35521s_of_match,
	},
};
module_mipi_dsi_driver(nt35521s_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for nt35521s wxga video mode dsi panel");
MODULE_LICENSE("GPL");
