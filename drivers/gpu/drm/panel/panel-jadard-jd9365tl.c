// SPDX-License-Identifier: GPL-2.0+
/*
 * Derived from panel-jadard-jd9365da-h3.c
 *
 * Copyright (c) 2026 Elettronica GF s.r.l.
 *
 * Author:
 * - Andrea Collamati <andrea.collamati@elettronicagf.it>
 */

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>


/*
 * Paste this at file scope (above your enable callback), or inside the callback
 * before use. It relies on 'dev' and 'ret' being visible in scope.
 */
#define DSI_WRITE_SEQ(_dsi, /* cmd then params... */ ...)                             \
    do {                                                                              \
        static const u8 __seq[] = { __VA_ARGS__ };                                    \
        ret = mipi_dsi_generic_write((_dsi), __seq, sizeof(__seq));                   \
        if (ret < 0) {                                                                \
            dev_err(dev, "DSI generic write failed (%d) at %s:%d\n",                  \
                    ret, __func__, __LINE__);                                         \
            return ret;                                                               \
        }                                                                             \
    } while (0)




struct jadard_panel_desc {
	const struct drm_display_mode mode;
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
};

struct jadard {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct jadard_panel_desc *desc;

	struct regulator *vdd;
	struct regulator *vccio;
	struct gpio_desc *reset;
};

static inline struct jadard *panel_to_jadard(struct drm_panel *panel)
{
	return container_of(panel, struct jadard, panel);
}


static int jadard_enable(struct drm_panel *panel)
{
	struct device *dev = panel->dev;
	struct jadard *jadard = panel_to_jadard(panel);
	struct mipi_dsi_device *dsi = jadard->dsi;

    int ret;

    /* Ensure regulators/backlight/reset have been configured earlier.
     * Keep the link in LPM for command writes (dsi->mode_flags typically
     * include MIPI_DSI_MODE_LPM by default for command-phase).
     */

    /* --- vendor generic init sequence (from meta_language_init.txt) --- */

    /* SSD_Number(0x04); SSD_CMD(0xDF); 0x90,0x66,0xF6; Delayms(1); */
    DSI_WRITE_SEQ(dsi, 0xDF, 0x90, 0x66, 0xF6);
    usleep_range(1000, 2000);

    /* SSD_Number(0x02); SSD_CMD(0xDE); 0x00; Delayms(1); */
    DSI_WRITE_SEQ(dsi, 0xDE, 0x00);
    usleep_range(1000, 2000);

    /* SSD_Number(0x0A); SSD_CMD(0xB2); +9 params; Delayms(1); */
    DSI_WRITE_SEQ(dsi, 0xB2, 0x01, 0x23, 0x60, 0x60, 0x88, 0xDB, 0x5A, 0x07, 0x00);
    usleep_range(1000, 2000);

    /* SSD_Number(0x08); SSD_CMD(0xBB); +7 params */
    DSI_WRITE_SEQ(dsi, 0xBB, 0x00, 0x22, 0x43, 0x50, 0x5A, 0x55, 0x55);

    /* SSD_Number(0x03); SSD_CMD(0xBD); +2 params */
    DSI_WRITE_SEQ(dsi, 0xBD, 0x00, 0x4C);

    /* SSD_Number(0x05); SSD_CMD(0xBF); +4 params */
    DSI_WRITE_SEQ(dsi, 0xBF, 0x46, 0x5A, 0x30, 0xC3);

    /* SSD_Number(0x05); SSD_CMD(0xC0); +4 params */
    DSI_WRITE_SEQ(dsi, 0xC0, 0x01, 0x85, 0x01, 0x85);

    /* SSD_Number(0x2B); SSD_CMD(0xCB); +42 params; Delayms(1) */
    DSI_WRITE_SEQ(dsi, 0xCB,
        0x7C, 0x69, 0x5D, 0x4D, 0x3F, 0x3C, 0x2E, 0x32, 0x1D, 0x37, 0x35, 0x34, 0x50, 0x3C, 0x42, 0x33, 0x30, 0x24, 0x13, 0x0A, 0x06,
        0x7C, 0x69, 0x5D, 0x4D, 0x3F, 0x3C, 0x2E, 0x32, 0x1D, 0x37, 0x35, 0x34, 0x50, 0x3C, 0x42, 0x33, 0x30, 0x24, 0x13, 0x0A, 0x06);
    usleep_range(1000, 2000);

    /* SSD_Number(0x09); SSD_CMD(0xC3); +8 params */
    DSI_WRITE_SEQ(dsi, 0xC3, 0x03, 0x01, 0x06, 0x01, 0x06, 0xFF, 0x07, 0xFF);

    /* SSD_Number(0x03); SSD_CMD(0xC4); +2 params */
    DSI_WRITE_SEQ(dsi, 0xC4, 0x00, 0x00);

    /* SSD_Number(0x0A); SSD_CMD(0xC6); +9 params */
    DSI_WRITE_SEQ(dsi, 0xC6, 0x00, 0xB4, 0x00, 0xB4, 0x00, 0x1F, 0x16, 0x82, 0x00);

    /* SSD_Number(0x04); SSD_CMD(0xC8); +3 params */
    DSI_WRITE_SEQ(dsi, 0xC8, 0x22, 0x00, 0x96);

    /* SSD_Number(0x05); SSD_CMD(0xCD); +4 params */
    DSI_WRITE_SEQ(dsi, 0xCD, 0x00, 0x00, 0x00, 0x00);

    /* SSD_Number(0x2D); SSD_CMD(0xCE); +44 params */
    DSI_WRITE_SEQ(dsi, 0xCE,
        0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00);

    /* SSD_Number(0x13); SSD_CMD(0xCF); +18 params */
    DSI_WRITE_SEQ(dsi, 0xCF,
        0x40,0x00,0x00,0x00,0x1F,0xFF,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x1F,0xFF,0x00,0x00,0x00);

    /* SSD_Number(0x18); SSD_CMD(0xD0); +23 params */
    DSI_WRITE_SEQ(dsi, 0xD0,
        0x00,0xA0,0xA2,0x9F,0x9F,0x97,0xD7,0x84,0x86,0x88,0x8A,0x8C,0x8E,0x80,
        0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24);

    /* SSD_Number(0x18); SSD_CMD(0xD1); +23 params */
    DSI_WRITE_SEQ(dsi, 0xD1,
        0x00,0xA1,0xA3,0x9F,0x9F,0x97,0xD7,0x85,0x87,0x89,0x8B,0x8D,0x8F,0x81,
        0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24);

    /* SSD_Number(0x18); SSD_CMD(0xD2); +23 params */
    DSI_WRITE_SEQ(dsi, 0xD2,
        0x00,0x81,0xA3,0x9F,0x9F,0x97,0xD7,0x87,0x85,0x8F,0x8D,0x8B,0x89,0xA1,
        0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24);

    /* SSD_Number(0x18); SSD_CMD(0xD3); +23 params */
    DSI_WRITE_SEQ(dsi, 0xD3,
        0x00,0x80,0xA2,0x9F,0x9F,0x97,0xD7,0x86,0x84,0x8E,0x8C,0x8A,0x88,0xA0,
        0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24);

    /* SSD_Number(0x41); SSD_CMD(0xD4); +64 params */
    DSI_WRITE_SEQ(dsi, 0xD4,
        0x00,0x20,0x1A,0x01,0x00,0x03,0x20,0x04,0x00,0x00,0x00,0x04,0x04,0x81,0x04,0x1F,
        0x01,0x00,0x03,0x05,0x20,0x64,0x04,0x04,0x40,0xE4,0x1C,0x03,0x04,0x00,0x18,0x00,
        0x0A,0x04,0x28,0x00,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x01,0x00,0x00,0x20,0x00,0x00);

    /* SSD_Number(0x21); SSD_CMD(0xD5); +32 params */
    DSI_WRITE_SEQ(dsi, 0xD5,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA0,0x00,0x00,0x00,0x07,0x32,0x5A,
        0x00,0x08,0x3C,0x00,0x00,0x04,0xA3,0xB4,0x00,0x1C,0x00,0x0C,0x71,0x20,0x04,0x10);

    /* extra tail from the same 0xD5 block (remaining bytes): */
    DSI_WRITE_SEQ(dsi, 0xD5,
        0x00,0x00,0x00,0x00,0x00,0x00, /* six zeros continue the table */
        0x00,0x0F,0x0F,0x00 /* last four as in your file */
    );

    /* SSD_Number(0x10); SSD_CMD(0xD7); +15 params */
    DSI_WRITE_SEQ(dsi, 0xD7,
        0x00,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x00,0xFF,0x00,0xFF,0x00,0xFF,0xBA,0xAA);

    /* SSD_Number(0x02); SSD_CMD(0xDE); +1 param */
    DSI_WRITE_SEQ(dsi, 0xDE, 0x02);

    /* SSD_Number(0x04); SSD_CMD(0xE6); +3 params */
    DSI_WRITE_SEQ(dsi, 0xE6, 0x10, 0x10, 0x78);

    /* SSD_Number(0x05); SSD_CMD(0xC6); +4 params */
    DSI_WRITE_SEQ(dsi, 0xC6, 0x42, 0x01, 0x40, 0x11);

    /* SSD_Number(0x02); SSD_CMD(0xDE); +1 param (restore) */
    DSI_WRITE_SEQ(dsi, 0xDE, 0x00);

    /* ---- end of vendor init ---- */

      mipi_dsi_dcs_exit_sleep_mode(dsi);
      msleep(120);
      mipi_dsi_dcs_set_display_on(dsi);
      msleep(20);
     

    return 0;
}

static int jadard_disable(struct drm_panel *panel)
{
	struct device *dev = panel->dev;
	struct jadard *jadard = panel_to_jadard(panel);
	int ret;

	ret = mipi_dsi_dcs_set_display_off(jadard->dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(jadard->dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "failed to enter sleep mode: %d\n", ret);

	return 0;
}

static int jadard_prepare(struct drm_panel *panel)
{
	struct jadard *jadard = panel_to_jadard(panel);
	int ret;

	ret = regulator_enable(jadard->vccio);
	if (ret)
		return ret;

	ret = regulator_enable(jadard->vdd);
	if (ret)
		return ret;

	gpiod_set_value(jadard->reset, 1);
	msleep(5);

	gpiod_set_value(jadard->reset, 0);
	msleep(10);

	gpiod_set_value(jadard->reset, 1);
	msleep(120);

	return 0;
}

static int jadard_unprepare(struct drm_panel *panel)
{
	struct jadard *jadard = panel_to_jadard(panel);

	gpiod_set_value(jadard->reset, 1);
	msleep(120);

	regulator_disable(jadard->vdd);
	regulator_disable(jadard->vccio);

	return 0;
}

static int jadard_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct jadard *jadard = panel_to_jadard(panel);
	const struct drm_display_mode *desc_mode = &jadard->desc->mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, desc_mode);
	if (!mode) {
		DRM_DEV_ERROR(&jadard->dsi->dev, "failed to add mode %ux%ux@%u\n",
			      desc_mode->hdisplay, desc_mode->vdisplay,
			      drm_mode_vrefresh(desc_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs jadard_funcs = {
	.disable = jadard_disable,
	.unprepare = jadard_unprepare,
	.prepare = jadard_prepare,
	.enable = jadard_enable,
	.get_modes = jadard_get_modes,
};

/* HengCheng 7" 600x1024 with incell technology */
static const struct jadard_panel_desc blc1256_desc = {
	.mode = {
			.clock = 51200,
			.hdisplay = 600,
			.hsync_start = 600 + 17,
			.hsync_end = 600 + 17 + 17,
			.htotal = 600 + 17 + 17 + 10,
			.vdisplay = 1024,
			.vsync_start = 1024 + 160,
			.vsync_end = 1024 + 160 + 160,
			.vtotal = 1024 + 160 + 160 + 70,
	},
	
	.lanes = 4,
	.format = MIPI_DSI_FMT_RGB888,
};

static int jadard_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct jadard_panel_desc *desc;
	struct jadard *jadard;
	int ret;

	jadard = devm_kzalloc(&dsi->dev, sizeof(*jadard), GFP_KERNEL);
	if (!jadard)
		return -ENOMEM;

	desc = of_device_get_match_data(dev);
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_NO_EOT_PACKET ;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	jadard->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(jadard->reset)) {
		DRM_DEV_ERROR(&dsi->dev, "failed to get our reset GPIO\n");
		return PTR_ERR(jadard->reset);
	}

	jadard->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(jadard->vdd)) {
		DRM_DEV_ERROR(&dsi->dev, "failed to get vdd regulator\n");
		return PTR_ERR(jadard->vdd);
	}

	jadard->vccio = devm_regulator_get(dev, "vccio");
	if (IS_ERR(jadard->vccio)) {
		DRM_DEV_ERROR(&dsi->dev, "failed to get vccio regulator\n");
		return PTR_ERR(jadard->vccio);
	}

	drm_panel_init(&jadard->panel, dev, &jadard_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&jadard->panel);
	if (ret)
		return ret;

	drm_panel_add(&jadard->panel);

	mipi_dsi_set_drvdata(dsi, jadard);
	jadard->dsi = dsi;
	jadard->desc = desc;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&jadard->panel);

	return ret;
}

static void jadard_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct jadard *jadard = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&jadard->panel);
}

static const struct of_device_id jadard_of_match[] = {
	{
		.compatible = "egf,blc1256",
		.data = &blc1256_desc
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jadard_of_match);

static struct mipi_dsi_driver jadard_driver = {
	.probe = jadard_dsi_probe,
	.remove = jadard_dsi_remove,
	.driver = {
		.name = "jadard-jd9365tl",
		.of_match_table = jadard_of_match,
	},
};
module_mipi_dsi_driver(jadard_driver);

MODULE_AUTHOR("Andrea Collamati <andrea.collamati@elettronicagf.it>");
MODULE_DESCRIPTION("Jadard EGF JD9365TL DSI panel");
MODULE_LICENSE("GPL");
