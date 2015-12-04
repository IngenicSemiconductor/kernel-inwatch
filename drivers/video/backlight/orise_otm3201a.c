/* *
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *
 * Orise OTM3201A panel driver.
 *
 * Inki Dae, <inki.dae@samsung.com>
 * Donghwa Lee, <dh09.lee@samsung.com>
 * MaoLei.Wang, <maolei.wang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/lcd.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <mach/jz_dsim.h>

#define POWER_IS_ON(pwr)	((pwr) == FB_BLANK_UNBLANK)
#define POWER_IS_OFF(pwr)	((pwr) == FB_BLANK_POWERDOWN)
#define POWER_IS_NRM(pwr)	((pwr) == FB_BLANK_NORMAL)

#define lcd_to_master(a)	(a->dsim_dev->master)
#define lcd_to_master_ops(a)	((lcd_to_master(a))->master_ops)

struct orise_otm3201a {
	struct device   *dev;
	unsigned int    power;
	unsigned int    id;

	struct lcd_device        *ld;
	struct backlight_device  *bd;

	struct mipi_dsim_lcd_device  *dsim_dev;
	struct lcd_platform_data     *ddi_pd;

	struct mutex lock;
	bool  enabled;
};


static void orise_otm3201a_sleep_in(struct orise_otm3201a *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x10, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void orise_otm3201a_sleep_out(struct orise_otm3201a *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x11, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void orise_otm3201a_display_on(struct orise_otm3201a *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x29, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void orise_otm3201a_display_off(struct orise_otm3201a *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x28, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void orise_otm3201a_panel_init(struct orise_otm3201a *lcd)
{
	int array_size;
	int VSA=2;
	int VBP=10;
	int VFP=2;
	int HSA=10;
	int HBP=42;
	int HFP=26;
	unsigned char orise_otm3201a_cmd_list[] = {
		0x39, 0x03, 0x00, 0xF0,0x54,0x47, // Enable CMD2
		0x39, 0x02, 0x00, 0xA0,0x00, // Read Mode Disable
		0x39, 0x02, 0x00, 0xB1,0x22,
		//{0x39, 0x06, 0x00, {0xB3,0x02,0x0a,0x1A,0x2a,0x2a}},
		0x39, 0x06, 0x00, 0xB3,VFP,VBP,HFP,HBP,(VSA*16+HSA),
		0x39, 0x03, 0x00, 0xB4,0x00,0x00,
		0x39, 0x04, 0x00, 0xBD,0x00,0x11,0x31,
		0x39, 0x05, 0x00, 0xBA,0x05,0x15,0x2B,0x01,
		0x39, 0x02, 0x00, 0xE9,0x46, // Set RTN
		0x39, 0x02, 0x00, 0xE2,0xf5,

		//Gamma 2.4" From INX setting CHECK Analog Gamma(C0h ~ C5h)
		0x39, 0x05, 0x00, 0xB5,0x5C,0x5C,0x7A,0xFA,
		0x39, 0x12, 0x00, 0xC0,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f,
		0x39, 0x12, 0x00, 0xC1,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f,
		0x39, 0x12, 0x00, 0xC2,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f,
		0x39, 0x12, 0x00, 0xC3,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f,
		0x39, 0x12, 0x00, 0xC4,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f,
		0x39, 0x12, 0x00, 0xC5,0x00,0x01,0x0b,0x11,0x16,0x2b,0x0c,0x0a,0x09,0x0d,0x0b,0x32,0x0e,0x13,0x1a,0x3f,0x3f,
	};
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);

	array_size = ARRAY_SIZE(orise_otm3201a_cmd_list);
	ops->cmd_write(dsi, orise_otm3201a_cmd_list, array_size);
}

static int orise_otm3201a_set_power(struct lcd_device *ld, int power)
{
	return 0;
}

static int orise_otm3201a_get_power(struct lcd_device *ld)
{
	struct orise_otm3201a *lcd = lcd_get_data(ld);

	return lcd->power;
}


static struct lcd_ops orise_otm3201a_lcd_ops = {
	.set_power = orise_otm3201a_set_power,
	.get_power = orise_otm3201a_get_power,
};


static void orise_otm3201a_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct orise_otm3201a *lcd = dev_get_drvdata(&dsim_dev->dev);

	if (power == POWER_ON_BL)
		return;

	mutex_lock(&lcd->lock);

	if (!lcd->ddi_pd->lcd_enabled) {
		/* lcd power on */
		if (lcd->ddi_pd->power_on)
			lcd->ddi_pd->power_on(lcd->ld, power);

		/* lcd reset */
		if (lcd->ddi_pd->reset)
			lcd->ddi_pd->reset(lcd->ld);
	}

	mutex_unlock(&lcd->lock);
}

static void orise_otm3201a_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct orise_otm3201a *lcd = dev_get_drvdata(&dsim_dev->dev);

	mutex_lock(&lcd->lock);

	if (!lcd->ddi_pd->lcd_enabled) {
		orise_otm3201a_panel_init(lcd);
		mdelay(20);
		orise_otm3201a_sleep_out(lcd);
		mdelay(20);
		orise_otm3201a_display_on(lcd);
		mdelay(20);
		lcd->power = FB_BLANK_UNBLANK;
	}
	mutex_unlock(&lcd->lock);
}

static int orise_otm3201a_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct orise_otm3201a *lcd;
	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct orise_otm3201a), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate orise_otm3201a structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->ddi_pd = (struct lcd_platform_data *)dsim_dev->platform_data;
	lcd->dev = &dsim_dev->dev;

	lcd->ld = lcd_device_register("orise_otm3201a", lcd->dev, lcd,
			&orise_otm3201a_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register orise_otm3201a lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	mutex_init(&lcd->lock);
	dev_set_drvdata(&dsim_dev->dev, lcd);

	dev_dbg(lcd->dev, "probed orise_otm3201a panel driver.\n");

	return 0;
}

#ifdef CONFIG_PM
static int orise_otm3201a_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct orise_otm3201a *lcd = dev_get_drvdata(&dsim_dev->dev);
	lcd->ddi_pd->lcd_enabled = 0;
	orise_otm3201a_sleep_in(lcd);
	msleep(lcd->ddi_pd->power_off_delay);
	orise_otm3201a_display_off(lcd);

	orise_otm3201a_power_on(dsim_dev, 0);

	return 0;
}

static int orise_otm3201a_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct orise_otm3201a *lcd = dev_get_drvdata(&dsim_dev->dev);

	orise_otm3201a_sleep_out(lcd);
	msleep(lcd->ddi_pd->power_on_delay);

	orise_otm3201a_power_on(dsim_dev, 1);
	orise_otm3201a_set_sequence(dsim_dev);

	return 0;
}
#else
#define orise_otm3201a_suspend		NULL
#define orise_otm3201a_resume		NULL
#endif

static struct mipi_dsim_lcd_driver orise_otm3201a_dsim_ddi_driver = {
	.name = "orise_otm3201a-lcd",
	.id = -1,

	.power_on = orise_otm3201a_power_on,
	.set_sequence = orise_otm3201a_set_sequence,
	.probe = orise_otm3201a_probe,
	.suspend = orise_otm3201a_suspend,
	.resume = orise_otm3201a_resume,
};

static int orise_otm3201a_init(void)
{
	mipi_dsi_register_lcd_driver(&orise_otm3201a_dsim_ddi_driver);
	return 0;
}

static void orise_otm3201a_exit(void)
{
	return;
}

module_init(orise_otm3201a_init);
module_exit(orise_otm3201a_exit);

MODULE_DESCRIPTION("Orise OTM3201A panel drive");
MODULE_LICENSE("GPL");
