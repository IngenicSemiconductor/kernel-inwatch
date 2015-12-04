/*
 * Copyright (C) 2015 Ingenic Electronics
 *
 * BOE 1.54 320*320 TFT LCD Driver
 *
 * Model : BV015Z2M-N00-2B00
 *
 * Author: MaoLei.Wang <maolei.wang@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
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

#include <video/mipi_display.h>
#include <mach/jz_dsim.h>

#define POWER_IS_ON(pwr)	((pwr) == FB_BLANK_UNBLANK)
#define POWER_IS_OFF(pwr)	((pwr) == FB_BLANK_POWERDOWN)
#define POWER_IS_NRM(pwr)	((pwr) == FB_BLANK_NORMAL)

#define lcd_to_master(a)	(a->dsim_dev->master)
#define lcd_to_master_ops(a)	((lcd_to_master(a))->master_ops)

struct h160_tft320320_dev {
	struct device *dev;
	unsigned int id;
	unsigned int power;

	struct lcd_device *ld;
	struct mipi_dsim_lcd_device *dsim_dev;
	struct lcd_platform_data    *ddi_pd;
	struct mutex lock;
};

#ifdef CONFIG_PM
static void h160_tft320320_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power);
#endif

static void h160_tft320320_sleep_in(struct h160_tft320320_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x10, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void h160_tft320320_sleep_out(struct h160_tft320320_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x11, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void h160_tft320320_display_on(struct h160_tft320320_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x29, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void h160_tft320320_display_off(struct h160_tft320320_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x28, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void h160_tft320320_panel_condition_setting(struct h160_tft320320_dev *lcd)
{
	int array_size;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);

	unsigned char h160_tft320320_cmd_list[] = {
		0x39, 0x02, 0x00, 0x36, 0xc0,
		0x39, 0x05, 0x00, 0x2a, 0x00, 0x00, 319 >> 8, 319 & 0xff,
		0x39, 0x05, 0x00, 0x2b, 160 >> 8, 160 & 0xff, 479 >> 8, 479 & 0xff,
		0x39, 0x03, 0x00, 0x44, 320 >> 8, 320 & 0xff,
		0x05, 0x20, 0x00,
		0x39, 0x02, 0x00, 0xe1, 0x02,
		0x05, 0x35, 0x00,
		0x39, 0x02, 0x00, 0x3a, 0x77,
		/* {0x05, 0x29, 0x00}, */
	};

	h160_tft320320_sleep_out(lcd);
	mdelay(10);
	array_size = ARRAY_SIZE(h160_tft320320_cmd_list);
	ops->cmd_write(dsi, h160_tft320320_cmd_list, array_size);
	return;
}

static void h160_tft320320_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct h160_tft320320_dev *lcd = dev_get_drvdata(&dsim_dev->dev);

	if (!lcd->ddi_pd->lcd_enabled) {
		mutex_lock(&lcd->lock);
		h160_tft320320_panel_condition_setting(lcd);
		lcd->power = FB_BLANK_UNBLANK;
		mutex_unlock(&lcd->lock);
	}

	return;
}

static int h160_tft320320_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	struct h160_tft320320_dev *lcd = dev_get_drvdata(&dsim_dev->dev);

	if (!lcd) {
		pr_err(" h160_tft320320_ioctl get drv failed\n");
		return -EFAULT;
	}

	mutex_lock(&lcd->lock);
	switch (cmd) {
	case CMD_MIPI_DISPLAY_ON:
		h160_tft320320_display_on(lcd);
#ifdef CONFIG_PM
		h160_tft320320_power_on(dsim_dev, POWER_ON_BL);
#endif
		break;
	default:
		break;
	}
	mutex_unlock(&lcd->lock);

	return 0;
}

static int h160_tft320320_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct h160_tft320320_dev *lcd = NULL;

	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct h160_tft320320_dev), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate h160_tft320320_dev structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->ddi_pd = (struct lcd_platform_data *)dsim_dev->platform_data;
	lcd->dev = &dsim_dev->dev;

	mutex_init(&lcd->lock);

	lcd->ld = lcd_device_register("h160_tft320320_dev", lcd->dev, lcd, NULL);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);
	dev_dbg(lcd->dev, "probed h160_tft320320_dev panel driver.\n");

	return 0;
}

#ifdef CONFIG_PM
static void h160_tft320320_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct h160_tft320320_dev *lcd = dev_get_drvdata(&dsim_dev->dev);

	if (!lcd->ddi_pd->lcd_enabled) {
		/* lcd power on */
		if (lcd->ddi_pd->power_on) {
			lcd->ddi_pd->power_on(lcd->ld, power);
		}

		if (power == POWER_ON_LCD) {
			/* lcd reset */
			if (lcd->ddi_pd->reset) {
				lcd->ddi_pd->reset(lcd->ld);
			}
		}
	}

	return;
}

static int h160_tft320320_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct h160_tft320320_dev *lcd = dev_get_drvdata(&dsim_dev->dev);

	lcd->ddi_pd->lcd_enabled = 0;

	mutex_lock(&lcd->lock);
	
	h160_tft320320_display_off(lcd);
	h160_tft320320_sleep_in(lcd);
	mdelay(50);
	h160_tft320320_power_on(dsim_dev, !POWER_ON_LCD);
	mutex_unlock(&lcd->lock);

	return 0;
}
#else
#define h160_tft320320_suspend		NULL
#define h160_tft320320_resume		NULL
#define h160_tft320320_power_on		NULL
#endif


static struct mipi_dsim_lcd_driver h160_tft320320_dsim_ddi_driver = {
	.name = "h160_tft320320-lcd",
	.id   = 0,
	.set_sequence = h160_tft320320_set_sequence,
	.ioctl    = h160_tft320320_ioctl,
	.probe    = h160_tft320320_probe,
	.suspend  = h160_tft320320_suspend,
	.power_on = h160_tft320320_power_on,
};

static int h160_tft320320_init(void)
{
	mipi_dsi_register_lcd_driver(&h160_tft320320_dsim_ddi_driver);
	return 0;
}

static void h160_tft320320_exit(void)
{
	return;
}

module_init(h160_tft320320_init);
module_exit(h160_tft320320_exit);

MODULE_DESCRIPTION("BOE TFT320320 lcd driver");
MODULE_LICENSE("GPL");
