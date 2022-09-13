// SPDX-License-Identifier: GPL-2.0
/*
 * KOE Hitachi TX13D100VM0EAA panel driver
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define KOE_MACP 		0xB0 /* Manufacturer CMD Protect */

#define KOE_INVERSION		0xC1
#define KOE_GAMMA_SET_A		0xC8 /* Gamma Setting A */
#define KOE_GAMMA_SET_B		0xC9 /* Gamma Setting B */
#define KOE_GAMMA_SET_C		0xCA /* Gamma Setting C */
#define KOE_CONTRAST_SET	0xCC

struct koe_display {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct regulator *vcc_supply;
	struct regulator *iovcc_supply;

	struct gpio_desc *reset_gpio;

	bool prepared;

	bool dig_cont_adj;
	bool inversion;
	int gamma;
};

static const u8 macp_on[] = {
	KOE_MACP, 0x03
};

static const u8 macp_off[] = {
	KOE_MACP, 0x04
};

static const u8 address_mode[] = {
	MIPI_DCS_SET_ADDRESS_MODE
};

static const u8 contrast_setting[] = {
	KOE_CONTRAST_SET,
	0xDC, 0xB4, 0xFF
};

static const u8 column_inversion[] = {
	KOE_INVERSION,
	0x00, 0x50, 0x03, 0x22,
	0x16, 0x06, 0x60, 0x11
};

static const u8 line_inversion[] = {
	KOE_INVERSION,
	0x00, 0x10, 0x03, 0x22,
	0x16, 0x06, 0x60, 0x01
};

static const u8 gamma_setting[][25] = {
	{},
	{
		KOE_GAMMA_SET_A,
		0x00, 0x06, 0x0A, 0x0F,
		0x14, 0x1F, 0x1F, 0x17,
		0x12, 0x0C, 0x09, 0x06,
		0x00, 0x06, 0x0A, 0x0F,
		0x14, 0x1F, 0x1F, 0x17,
		0x12, 0x0C, 0x09, 0x06
	},
	{
		KOE_GAMMA_SET_A,
		0x00, 0x05, 0x0B, 0x0F,
		0x11, 0x1D, 0x20, 0x18,
		0x18, 0x09, 0x07, 0x06,
		0x00, 0x05, 0x0B, 0x0F,
		0x11, 0x1D, 0x20, 0x18,
		0x18, 0x09, 0x07, 0x06
	},
	{
		KOE_GAMMA_SET_A,
		0x0B, 0x0D, 0x10, 0x14,	
		0x13, 0x1D, 0x20, 0x18,
		0x12, 0x09, 0x07, 0x06,
		0x0A, 0x0C, 0x10, 0x14,
		0x13, 0x1D, 0x20, 0x18,
		0x12, 0x09, 0x07, 0x06
	},
};

static inline struct koe_display *to_koe_display(struct drm_panel *panel)
{
	return container_of(panel, struct koe_display, panel);
}

static void koe_display_reset(struct koe_display *priv)
{
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(2000, 3000);
}

static int koe_display_prepare(struct drm_panel *panel)
{
	struct koe_display *priv = to_koe_display(panel);
	struct device *dev = &priv->dsi->dev;
	int ret;

	if (priv->prepared)
		return 0;

	ret = regulator_enable(priv->vcc_supply);
	if (ret < 0) {
		dev_err(dev, "failed to enable vcc power supply\n");
		return ret;
	}

	mdelay(2);

	ret = regulator_enable(priv->iovcc_supply);
	if (ret < 0) {
		dev_err(dev, "failed to enable iovcc power supply\n");
		return ret;
	}

	mdelay(2);

	koe_display_reset(priv);

	priv->prepared = true;
	return 0;
}

static int koe_display_enable(struct drm_panel *panel)
{
	struct koe_display *priv = to_koe_display(panel);
	struct mipi_dsi_device *dsi = priv->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	msleep(80);

	mipi_dsi_dcs_write_buffer(dsi, address_mode,
			sizeof(address_mode));

	mdelay(20);

	ret = mipi_dsi_dcs_set_pixel_format(dsi, MIPI_DCS_PIXEL_FMT_24BIT << 4);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format: %d\n", ret);
		return ret;
	}

	/* MACP Off */
	mipi_dsi_generic_write(dsi, macp_off, sizeof(macp_off));

	if (priv->dig_cont_adj)
		mipi_dsi_generic_write(dsi, contrast_setting,
				sizeof(contrast_setting));

	if (priv->gamma)
		mipi_dsi_generic_write(dsi, gamma_setting[priv->gamma],
				sizeof(gamma_setting[priv->gamma]));

	if (priv->inversion)
		mipi_dsi_generic_write(dsi, column_inversion,
				sizeof(column_inversion));
	else
		mipi_dsi_generic_write(dsi, line_inversion,
				sizeof(line_inversion));

	/* MACP On */
	mipi_dsi_generic_write(dsi, macp_on, sizeof(macp_on));

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}
	msleep(50);

	return 0;
}

static int koe_display_disable(struct drm_panel *panel)
{
	struct koe_display *priv = to_koe_display(panel);
	struct mipi_dsi_device *dsi = priv->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}
	msleep(100);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}

	return 0;
}

static int koe_display_unprepare(struct drm_panel *panel)
{
	struct koe_display *priv = to_koe_display(panel);

	if (!priv->prepared)
		return 0;

	mdelay(10);

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	mdelay(5);

	regulator_disable(priv->iovcc_supply);
	mdelay(2);
	regulator_disable(priv->vcc_supply);

	priv->prepared = false;
	return 0;
}

static const struct drm_display_mode koe_display_mode = {
	.clock = (768 + 116 + 81 + 5) * (1024 + 24 + 8 + 2) * 60 / 1000,
	.hdisplay = 768,
	.hsync_start = 768 + 116,
	.hsync_end = 768 + 116 + 81,
	.htotal = 768 + 116 + 81 + 5,
	.vdisplay = 1024,
	.vsync_start = 1024 + 24,
	.vsync_end = 1024 + 24 + 8,
	.vtotal = 1024 + 24 + 8 + 2,
	.width_mm = 76,
	.height_mm = 101,
};

static int koe_display_get_modes(struct drm_panel *panel,
				   struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &koe_display_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs koe_display_panel_funcs = {
	.prepare = koe_display_prepare,
	.enable = koe_display_enable,
	.disable = koe_display_disable,
	.unprepare = koe_display_unprepare,
	.get_modes = koe_display_get_modes,
};

static int koe_display_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct koe_display *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->vcc_supply = devm_regulator_get(dev, "vcc");
	if (IS_ERR(priv->vcc_supply))
		return PTR_ERR(priv->vcc_supply);

	priv->iovcc_supply = devm_regulator_get(dev, "iovcc");
	if (IS_ERR(priv->iovcc_supply))
		return PTR_ERR(priv->iovcc_supply);

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	if (device_property_read_bool(dev, "koe,inversion"))
		priv->inversion = true;

	if (device_property_read_bool(dev, "koe,contrast"))
		priv->dig_cont_adj = true;

	ret = device_property_read_u32(dev, "koe,gamma", &priv->gamma);
	if (ret)
		priv->gamma = 0;

	priv->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, priv);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
//			  MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_LPM;

	drm_panel_init(&priv->panel, dev, &koe_display_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

//	ret = drm_panel_of_backlight(&priv->panel);
//	if (ret)
//		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&priv->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&priv->panel);
		return ret;
	}

	return 0;
}

static int koe_display_remove(struct mipi_dsi_device *dsi)
{
	struct koe_display *priv = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev,
			"Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&priv->panel);

	return 0;
}

static const struct of_device_id koe_display_of_match[] = {
	{ .compatible = "koe,tx13d100vm0eaa" },
	{ .compatible = "hitachi,tx13d100vm0eaa" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, koe_display_of_match);

static struct mipi_dsi_driver koe_display_driver = {
	.probe = koe_display_probe,
	.remove = koe_display_remove,
	.driver = {
		.name = "panel-koe-tx13d100vm0eaa",
		.of_match_table = koe_display_of_match,
	},
};
module_mipi_dsi_driver(koe_display_driver);

MODULE_AUTHOR("Maxim Schwalm <maxim.schwalm@gmail.com>");
MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("KOE Hitachi TX13D100VM0EAA panel driver");
MODULE_LICENSE("GPL v2");
