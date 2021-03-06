/*
 * Copyright (C) 2015 Ingenic Electronics
 *
 * ARS NT35350 LTPS TFT 360x360 LCD Driver
 *
 * Model : ARS-Y1300A
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

struct ars_nt35350_dev {
	struct device *dev;
	unsigned int id;
	unsigned int power;

	struct lcd_device *ld;
	struct mipi_dsim_lcd_device *dsim_dev;
	struct lcd_platform_data    *ddi_pd;
	struct mutex lock;
};

#ifdef CONFIG_PM
static void ars_nt35350_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power);
#endif

static void ars_nt35350_sleep_in(struct ars_nt35350_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x10, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void ars_nt35350_sleep_out(struct ars_nt35350_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x11, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void ars_nt35350_display_on(struct ars_nt35350_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x29, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void ars_nt35350_display_off(struct ars_nt35350_dev *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x28, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void ars_nt35350_panel_condition_setting(struct ars_nt35350_dev *lcd)
{
	int array_size;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);

	unsigned char ars_nt35350_cmd_list[] = {
		0x39, 0x02, 0x00, 0xFF, 0x10,
		0x39, 0x02, 0x00, 0xB3, 0x00,
		0x39, 0x02, 0x00, 0xC0, 0x01,
		0x39, 0x02, 0x00, 0xBB, 0x10,
		0x39, 0x02, 0x00, 0xFF, 0x24,
		0x39, 0x02, 0x00, 0xCF, 0x0A,
		0x39, 0x02, 0x00, 0xD0, 0x06,
		0x39, 0x02, 0x00, 0xD2, 0xBA,
		0x39, 0x02, 0x00, 0xD3, 0xBA,
		0x39, 0x02, 0x00, 0xD4, 0xBA,
		0x39, 0x02, 0x00, 0xD5, 0x01,
		0x39, 0x02, 0x00, 0xD8, 0x41,
		0x39, 0x02, 0x00, 0x92, 0x00,
		0x39, 0x02, 0x00, 0x93, 0x00,
		0x39, 0x02, 0x00, 0x02, 0x00,
		0x39, 0x02, 0x00, 0x03, 0x00,
		0x39, 0x02, 0x00, 0x04, 0x00,
		0x39, 0x02, 0x00, 0x05, 0x03,
		0x39, 0x02, 0x00, 0x06, 0x8C,
		0x39, 0x02, 0x00, 0x07, 0x0C,
		0x39, 0x02, 0x00, 0x08, 0x09,
		0x39, 0x02, 0x00, 0x09, 0x07,
		0x39, 0x02, 0x00, 0x0A, 0x00,
		0x39, 0x02, 0x00, 0x0B, 0x00,
		0x39, 0x02, 0x00, 0x0C, 0x00,
		0x39, 0x02, 0x00, 0x0D, 0x00,
		0x39, 0x02, 0x00, 0x0E, 0x00,
		0x39, 0x02, 0x00, 0x0F, 0x00,
		0x39, 0x02, 0x00, 0x10, 0x00,
		0x39, 0x02, 0x00, 0x11, 0x00,
		0x39, 0x02, 0x00, 0x12, 0x00,
		0x39, 0x02, 0x00, 0x13, 0x00,
		0x39, 0x02, 0x00, 0x14, 0x00,
		0x39, 0x02, 0x00, 0x15, 0x00,
		0x39, 0x02, 0x00, 0x16, 0x00,
		0x39, 0x02, 0x00, 0x17, 0x00,
		0x39, 0x02, 0x00, 0x18, 0x00,
		0x39, 0x02, 0x00, 0x19, 0x00,
		0x39, 0x02, 0x00, 0x1A, 0x00,
		0x39, 0x02, 0x00, 0x1D, 0x00,
		0x39, 0x02, 0x00, 0x1E, 0x00,
		0x39, 0x02, 0x00, 0x1F, 0x00,
		0x39, 0x02, 0x00, 0x20, 0x00,
		0x39, 0x02, 0x00, 0x21, 0x00,
		0x39, 0x02, 0x00, 0x22, 0x00,
		0x39, 0x02, 0x00, 0x23, 0x00,
		0x39, 0x02, 0x00, 0x24, 0x00,
		0x39, 0x02, 0x00, 0x25, 0x00,
		0x39, 0x02, 0x00, 0x26, 0x00,
		0x39, 0x02, 0x00, 0x27, 0x02,
		0x39, 0x02, 0x00, 0x28, 0x08,
		0x39, 0x02, 0x00, 0x29, 0x06,
		0x39, 0x02, 0x00, 0x2A, 0x02,
		0x39, 0x02, 0x00, 0x2B, 0x03,
		0x39, 0x02, 0x00, 0x2D, 0x04,
		0x39, 0x02, 0x00, 0x2F, 0x05,
		0x39, 0x02, 0x00, 0x30, 0x06,
		0x39, 0x02, 0x00, 0x31, 0x07,
		0x39, 0x02, 0x00, 0x32, 0x00,
		0x39, 0x02, 0x00, 0x33, 0x00,
		0x39, 0x02, 0x00, 0x34, 0x00,
		0x39, 0x02, 0x00, 0x36, 0x00,
		0x39, 0x02, 0x00, 0x37, 0x00,
		0x39, 0x02, 0x00, 0x38, 0x80,
		0x39, 0x02, 0x00, 0x39, 0x00,
		0x39, 0x02, 0x00, 0x3A, 0x40,
		0x39, 0x02, 0x00, 0x3B, 0x02,
		0x39, 0x02, 0x00, 0x3D, 0xA3,
		0x39, 0x02, 0x00, 0x42, 0xAD,
		0x39, 0x02, 0x00, 0x43, 0x50,
		0x39, 0x02, 0x00, 0x44, 0x22,
		0x39, 0x02, 0x00, 0x45, 0x00,
		0x39, 0x02, 0x00, 0x46, 0x04,
		0x39, 0x02, 0x00, 0x47, 0x00,
		0x39, 0x02, 0x00, 0x48, 0x48,
		0x39, 0x02, 0x00, 0x49, 0x19,
		0x39, 0x02, 0x00, 0x4A, 0x00,
		0x39, 0x02, 0x00, 0x4B, 0xA3,
		0x39, 0x02, 0x00, 0x4C, 0x30,
		0x39, 0x02, 0x00, 0x4D, 0x05,
		0x39, 0x02, 0x00, 0x4E, 0x20,
		0x39, 0x02, 0x00, 0x65, 0x25,
		0x39, 0x02, 0x00, 0x66, 0x64,
		0x39, 0x02, 0x00, 0x6D, 0x04,
		0x39, 0x02, 0x00, 0x6E, 0x08,
		0x39, 0x02, 0x00, 0x80, 0x21,
		0x39, 0x02, 0x00, 0x81, 0x0C,
		0x39, 0x02, 0x00, 0xFF, 0x20,
		0x39, 0x02, 0x00, 0x07, 0x44,
		0x39, 0x02, 0x00, 0x16, 0x1A,
		0x39, 0x02, 0x00, 0x53, 0x2C,
		0x39, 0x02, 0x00, 0x54, 0x01,
		0x39, 0x02, 0x00, 0x55, 0x2C,
		0x39, 0x02, 0x00, 0x56, 0x01,
		0x39, 0x02, 0x00, 0x57, 0x00,
		0x39, 0x02, 0x00, 0x58, 0x00,
		0x39, 0x02, 0x00, 0x59, 0x2C,
		0x39, 0x02, 0x00, 0x5A, 0x01,
		0x39, 0x02, 0x00, 0x5B, 0x2C,
		0x39, 0x02, 0x00, 0x5C, 0x01,
		0x39, 0x02, 0x00, 0x61, 0x00,
		0x39, 0x02, 0x00, 0x62, 0x00,
		0x39, 0x02, 0x00, 0x0E, 0x13,
		0x39, 0x02, 0x00, 0x0F, 0x13,
		0x39, 0x02, 0x00, 0x10, 0x13,
		0x39, 0x02, 0x00, 0x11, 0x13,
		0x39, 0x02, 0x00, 0x14, 0xCF,
		0x39, 0x02, 0x00, 0x0B, 0xA4,
		0x39, 0x02, 0x00, 0x0C, 0x90,
		0x39, 0x02, 0x00, 0xFF, 0x24,
		0x39, 0x02, 0x00, 0xE5, 0x2F,
		0x39, 0x02, 0x00, 0x83, 0x01,
		0x39, 0x02, 0x00, 0x8E, 0x00,
		0x39, 0x02, 0x00, 0x97, 0x00,
		0x39, 0x02, 0x00, 0x9B, 0x01,
		0x39, 0x02, 0x00, 0x9C, 0x39,
		0x39, 0x02, 0x00, 0x9D, 0x06,
		0x39, 0x02, 0x00, 0x9E, 0x02,
		0x39, 0x02, 0x00, 0xA2, 0x03,
		0x39, 0x02, 0x00, 0xA5, 0x12,
		0x39, 0x02, 0x00, 0xA6, 0x45,
		0x39, 0x02, 0x00, 0xA7, 0x31,
		0x39, 0x02, 0x00, 0xA9, 0x64,
		0x39, 0x02, 0x00, 0xAA, 0x23,
		0x39, 0x02, 0x00, 0xAB, 0x56,
		0x39, 0x02, 0x00, 0xB2, 0x21,
		0x39, 0x02, 0x00, 0xB3, 0x43,
		0x39, 0x02, 0x00, 0xB4, 0x65,
		0x39, 0x02, 0x00, 0xB8, 0x00,
		0x39, 0x02, 0x00, 0xB9, 0x40,
		0x39, 0x02, 0x00, 0xBA, 0x10,
		0x39, 0x02, 0x00, 0xC0, 0x07,
		0x39, 0x02, 0x00, 0xC1, 0x17,
		0x39, 0x02, 0x00, 0xFF, 0x10,
		0x39, 0x02, 0x00, 0xFB, 0x01,
		0x39, 0x02, 0x00, 0xFF, 0x20,
		0x39, 0x02, 0x00, 0xFB, 0x01,
		0x39, 0x02, 0x00, 0xFF, 0x24,
		0x39, 0x02, 0x00, 0xFB, 0x01,
		0x39, 0x02, 0x00, 0xFF, 0x10,
		0x39, 0x02, 0x00, 0x35, 0x00,
	};

	array_size = ARRAY_SIZE(ars_nt35350_cmd_list);
	ops->cmd_write(dsi, ars_nt35350_cmd_list, array_size);

	ars_nt35350_sleep_out(lcd);
	msleep(120);
	ars_nt35350_display_on(lcd);

	return;
}

static void ars_nt35350_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{

	struct ars_nt35350_dev *lcd = dev_get_drvdata(&dsim_dev->dev);
	printk(KERN_DEBUG "%s %s %d\n", __FILE__, __func__, __LINE__);

	if (!lcd->ddi_pd->lcd_enabled) {
		mutex_lock(&lcd->lock);
		ars_nt35350_panel_condition_setting(lcd);
		lcd->power = FB_BLANK_UNBLANK;
		mutex_unlock(&lcd->lock);
	}

	return;
}

static int ars_nt35350_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	struct ars_nt35350_dev *lcd = dev_get_drvdata(&dsim_dev->dev);
	printk(KERN_DEBUG "%s %s %d\n", __FILE__, __func__, __LINE__);
	if (!lcd) {
		pr_err(" ars_nt35350_ioctl get drv failed\n");
		return -EFAULT;
	}

	mutex_lock(&lcd->lock);
	switch (cmd) {
	case CMD_MIPI_DISPLAY_ON:
		ars_nt35350_display_on(lcd);
#ifdef CONFIG_PM
		ars_nt35350_power_on(dsim_dev, POWER_ON_BL);
#endif
		break;
	default:
		break;
	}
	mutex_unlock(&lcd->lock);

	return 0;
}

static int ars_nt35350_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct ars_nt35350_dev *lcd = NULL;
	printk(KERN_DEBUG "%s %s %d\n", __FILE__, __func__, __LINE__);
	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct ars_nt35350_dev), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate ars_nt35350_dev structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->ddi_pd = (struct lcd_platform_data *)dsim_dev->platform_data;
	lcd->dev = &dsim_dev->dev;

	mutex_init(&lcd->lock);

	lcd->ld = lcd_device_register("ars_nt35350", lcd->dev, lcd, NULL);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);
	dev_dbg(lcd->dev, "probed ars_nt35350_dev panel driver.\n");

	return 0;
}

#ifdef CONFIG_PM
static void ars_nt35350_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct ars_nt35350_dev *lcd = dev_get_drvdata(&dsim_dev->dev);
	printk(KERN_DEBUG "%s %s %d\n", __FILE__, __func__, __LINE__);
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

static int ars_nt35350_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct ars_nt35350_dev *lcd = dev_get_drvdata(&dsim_dev->dev);
	printk(KERN_DEBUG "%s %s %d\n", __FILE__, __func__, __LINE__);
	lcd->ddi_pd->lcd_enabled = 0;

	mutex_lock(&lcd->lock);
	ars_nt35350_power_on(dsim_dev, !POWER_ON_LCD);
	mutex_unlock(&lcd->lock);

	return 0;
}
#else
#define ars_nt35350_suspend		NULL
#define ars_nt35350_resume		NULL
#define ars_nt35350_power_on		NULL
#endif


static struct mipi_dsim_lcd_driver ars_nt35350_dsim_ddi_driver = {
	.name = "ars_nt35350-lcd",
	.id   = 0,
	.set_sequence = ars_nt35350_set_sequence,
	.ioctl    = ars_nt35350_ioctl,
	.probe    = ars_nt35350_probe,
	.suspend  = ars_nt35350_suspend,
	.power_on = ars_nt35350_power_on,
};

static int ars_nt35350_init(void)
{
	mipi_dsi_register_lcd_driver(&ars_nt35350_dsim_ddi_driver);
	return 0;
}

static void ars_nt35350_exit(void)
{
	return;
}

module_init(ars_nt35350_init);
module_exit(ars_nt35350_exit);

MODULE_DESCRIPTION("ARS NT35350 LTPS TFT LCD Driver");
MODULE_LICENSE("GPL");
