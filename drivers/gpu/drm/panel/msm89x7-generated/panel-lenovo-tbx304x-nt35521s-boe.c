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

struct nt35521s_boe {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
};

static inline struct nt35521s_boe *to_nt35521s_boe(struct drm_panel *panel)
{
	return container_of_const(panel, struct nt35521s_boe, panel);
}

static void nt35521s_boe_reset(struct nt35521s_boe *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);
}

static int nt35521s_boe_on(struct nt35521s_boe *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xff,
					 0xaa, 0x55, 0x25, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfc, 0x08);
	mipi_dsi_usleep_range(&dsi_ctx, 1000, 2000);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfc, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xff,
					 0xaa, 0x55, 0x25, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x68, 0x07);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb8,
					 0x01, 0x02, 0x02, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb6, 0x11);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbb, 0x63, 0x63);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd,
					 0x02, 0xbb, 0x11, 0x10, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc8, 0x80);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3, 0x28, 0x28);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb4, 0x14, 0x14);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb9, 0x44, 0x44);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba, 0x24, 0x24);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc, 0x78, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd, 0x78, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xee, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0,
					 0x00, 0x00, 0x00, 0x1a, 0x00, 0x40,
					 0x00, 0x5c, 0x00, 0x75, 0x00, 0x9d,
					 0x00, 0xbd, 0x00, 0xf1);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1,
					 0x01, 0x1a, 0x01, 0x59, 0x01, 0x8a,
					 0x01, 0xd6, 0x02, 0x13, 0x02, 0x15,
					 0x02, 0x4b, 0x02, 0x84);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2,
					 0x02, 0xa8, 0x02, 0xdc, 0x02, 0xf6,
					 0x03, 0x26, 0x03, 0x43, 0x03, 0x6c,
					 0x03, 0x86, 0x03, 0xa1);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3,
					 0x03, 0xba, 0x03, 0xff);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2,
					 0x05, 0x01, 0x55, 0x00, 0x1a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba,
					 0x53, 0x01, 0x55, 0x00, 0x3a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc4, 0x60);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc5, 0x40);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x04);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1,
					 0x03, 0x02, 0x05, 0x07);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x05);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd,
					 0x01, 0x01, 0x00, 0x00, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc0, 0x09);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc8, 0x07, 0x38);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd1,
					 0x00, 0x05, 0x01, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe5, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe6, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe9, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xea, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x08, 0x06);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x2d, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x01, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb2, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb3, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb4, 0x2e, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x2d, 0x13);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb6, 0x13, 0x11);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb7, 0x11, 0x36);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb8, 0x36, 0x15);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb9, 0x15, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xba, 0x2e, 0x14);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbb, 0x14, 0x35);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbc, 0x35, 0x10);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbd, 0x10, 0x12);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbe, 0x12, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xbf, 0x2d, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc0, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc1, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc2, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc3, 0x2d, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc4, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc5, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc6, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc7, 0x2d, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc8, 0x2e, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xc9, 0x2d, 0x14);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xca, 0x14, 0x35);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcb, 0x35, 0x10);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcc, 0x10, 0x12);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcd, 0x12, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xce, 0x2e, 0x13);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xcf, 0x13, 0x11);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd0, 0x11, 0x36);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd1, 0x36, 0x15);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd2, 0x15, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd3, 0x2d, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd4, 0x2d, 0x2d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd5, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd6, 0x01, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd7, 0x2e, 0x2e);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd8,
					 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xd9,
					 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe5, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe6, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe8, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xe9, 0x34, 0x34);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0,
					 0x55, 0xaa, 0x52, 0x00, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x62, 0x01);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int nt35521s_boe_off(struct nt35521s_boe *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int nt35521s_boe_prepare(struct drm_panel *panel)
{
	struct nt35521s_boe *ctx = to_nt35521s_boe(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	nt35521s_boe_reset(ctx);

	ret = nt35521s_boe_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->supply);
		return ret;
	}

	return 0;
}

static int nt35521s_boe_unprepare(struct drm_panel *panel)
{
	struct nt35521s_boe *ctx = to_nt35521s_boe(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = nt35521s_boe_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->supply);

	return 0;
}

static const struct drm_display_mode nt35521s_boe_mode = {
	.clock = (800 + 200 + 4 + 80) * (1280 + 20 + 4 + 16) * 60 / 1000,
	.hdisplay = 800,
	.hsync_start = 800 + 200,
	.hsync_end = 800 + 200 + 4,
	.htotal = 800 + 200 + 4 + 80,
	.vdisplay = 1280,
	.vsync_start = 1280 + 20,
	.vsync_end = 1280 + 20 + 4,
	.vtotal = 1280 + 20 + 4 + 16,
	.width_mm = 135,
	.height_mm = 216,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int nt35521s_boe_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &nt35521s_boe_mode);
}

static const struct drm_panel_funcs nt35521s_boe_panel_funcs = {
	.prepare = nt35521s_boe_prepare,
	.unprepare = nt35521s_boe_unprepare,
	.get_modes = nt35521s_boe_get_modes,
};

static int nt35521s_boe_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt35521s_boe *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct nt35521s_boe, panel,
				   &nt35521s_boe_panel_funcs,
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
			  MIPI_DSI_MODE_VIDEO_NO_HFP;

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

static void nt35521s_boe_remove(struct mipi_dsi_device *dsi)
{
	struct nt35521s_boe *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt35521s_boe_of_match[] = {
	{ .compatible = "lenovo,tbx304x-nt35521s-boe" }, // FIXME
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt35521s_boe_of_match);

static struct mipi_dsi_driver nt35521s_boe_driver = {
	.probe = nt35521s_boe_probe,
	.remove = nt35521s_boe_remove,
	.driver = {
		.name = "panel-lenovo-tbx304x-nt35521s-boe",
		.of_match_table = nt35521s_boe_of_match,
	},
};
module_mipi_dsi_driver(nt35521s_boe_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for nt35521s boe wxga video mode dsi panel");
MODULE_LICENSE("GPL");
