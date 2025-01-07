// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024, Vasiliy Doylov (NekoCWD) <nekodevelopper@gmail.com>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct huawei_agassi_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
};

static inline struct huawei_agassi_panel *to_huawei_agassi_panel(struct drm_panel *panel)
{
	return container_of(panel, struct huawei_agassi_panel, panel);
}

static int huawei_agassi_panel_enable(struct drm_panel *panel)
{
	struct huawei_agassi_panel *ctx = to_huawei_agassi_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	mipi_dsi_dcs_write_seq(dsi, 0x83, 0xaa);
	mipi_dsi_dcs_write_seq(dsi, 0x84, 0x55);
	mipi_dsi_dcs_write_seq(dsi, 0x1, 0x0);
	msleep(20);
	mipi_dsi_dcs_write_seq(dsi, 0x83, 0x96);
	mipi_dsi_dcs_write_seq(dsi, 0x84, 0x69);
	mipi_dsi_dcs_write_seq(dsi, 0xa9, 0xff);
	mipi_dsi_dcs_write_seq(dsi, 0xaa, 0xfe);
	mipi_dsi_dcs_write_seq(dsi, 0xab, 0xfa);
	mipi_dsi_dcs_write_seq(dsi, 0xac, 0xf7);
	mipi_dsi_dcs_write_seq(dsi, 0xad, 0xf3);
	mipi_dsi_dcs_write_seq(dsi, 0xae, 0xf1);
	mipi_dsi_dcs_write_seq(dsi, 0xaf, 0xed);
	mipi_dsi_dcs_write_seq(dsi, 0xb0, 0xeb);
	mipi_dsi_dcs_write_seq(dsi, 0xb1, 0xe9);
	mipi_dsi_dcs_write_seq(dsi, 0xb2, 0xe6);
	mipi_dsi_dcs_write_seq(dsi, 0xbd, 0x1c);
	mipi_dsi_dcs_write_seq(dsi, 0xbe, 0x30);
	mipi_dsi_dcs_write_seq(dsi, 0xbf, 0x30);
	mipi_dsi_dcs_write_seq(dsi, 0x98, 0xc);
	mipi_dsi_dcs_write_seq(dsi, 0x97, 0x80);
	mipi_dsi_dcs_write_seq(dsi, 0x96, 0x63);
	mipi_dsi_dcs_write_seq(dsi, 0x95, 0x0);
	mipi_dsi_dcs_write_seq(dsi, 0x83, 0x0);
	mipi_dsi_dcs_write_seq(dsi, 0x84, 0x0);
	mipi_dsi_dcs_write_seq(dsi, 0x96, 0xc0);
	mipi_dsi_dcs_write_seq(dsi, 0x10, 0x0);

	return 0;
}

static int huawei_agassi_panel_disable(struct drm_panel *panel)
{
	struct huawei_agassi_panel *ctx = to_huawei_agassi_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	mipi_dsi_dcs_write_seq(dsi, 0x11, 0x00);
	msleep(100);

	return 0;
}

static int huawei_agassi_panel_backlight_update_status(struct backlight_device *backlight)
{
	struct mipi_dsi_device *dsi = bl_get_data(backlight);
	u16 brightness = backlight_get_brightness(backlight);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_write(dsi, 0xf5, &brightness, 1);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (ret < 0) {
		dev_alert(&backlight->dev, "Error setting brightness");
		return ret;
	}

	return 0;
}

static const struct backlight_ops huawei_agassi_panel_backlight_ops = {
	.update_status = huawei_agassi_panel_backlight_update_status,
};

static struct backlight_device *
huawei_agassi_panel_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &huawei_agassi_panel_backlight_ops, &props);
}

static const struct drm_display_mode huawei_agassi_panel_mode = {
	.clock = (800 + 152 + 8 + 128) * (1280 + 18 + 1 + 23) * 60 / 1000,
	.hdisplay = 800,
	.hsync_start = 800 + 152,
	.hsync_end = 800 + 152 + 8,
	.htotal = 800 + 152 + 8 + 128,
	.vdisplay = 1280,
	.vsync_start = 1280 + 18,
	.vsync_end = 1280 + 18 + 1,
	.vtotal = 1280 + 18 + 1 + 23,
	.width_mm = 129,
	.height_mm = 207,
};

static int huawei_agassi_panel_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &huawei_agassi_panel_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs huawei_agassi_panel_panel_funcs = {
	.get_modes = huawei_agassi_panel_get_modes,
	.disable = huawei_agassi_panel_disable,
	.enable = huawei_agassi_panel_enable,
};

static int huawei_agassi_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct huawei_agassi_panel *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =
		MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_MODE_VIDEO_NO_HFP |
		MIPI_DSI_MODE_VIDEO_NO_HBP | MIPI_DSI_MODE_VIDEO_NO_HSA;

	drm_panel_init(&ctx->panel, dev, &huawei_agassi_panel_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	if (!ctx->panel.backlight) {
		ctx->panel.backlight = huawei_agassi_panel_create_backlight(dsi);
		if (IS_ERR(ctx->panel.backlight))
			return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
					     "Failed to create backlight\n");
	}

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void huawei_agassi_panel_remove(struct mipi_dsi_device *dsi)
{
	struct huawei_agassi_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id huawei_agassi_panel_of_match[] = {
	{ .compatible = "huawei,agassi-panel" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, huawei_agassi_panel_of_match);

static struct mipi_dsi_driver huawei_agassi_panel_driver = {
	.probe = huawei_agassi_panel_probe,
	.remove = huawei_agassi_panel_remove,
	.driver = {
		.name = "panel-huawei-agassi",
		.of_match_table = huawei_agassi_panel_of_match,
	},
};

module_mipi_dsi_driver(huawei_agassi_panel_driver);

MODULE_AUTHOR("NekoCWD <nekodevelopper@gmail.com");
MODULE_DESCRIPTION("DRM driver for huawei agassi panel");
MODULE_LICENSE("GPL");
