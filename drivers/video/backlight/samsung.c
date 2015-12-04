/* linux/drivers/video/exynos/samsung.c
 *
 * MIPI-DSI based samsung AMOLED lcd 4.65 inch panel driver.
 *
 * Inki Dae, <inki.dae@samsung.com>
 * Donghwa Lee, <dh09.lee@samsung.com>
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
#include <linux/gpio.h>

#include <video/mipi_display.h>
#include <mach/jz_dsim.h>

#define POWER_IS_ON(pwr)	((pwr) == FB_BLANK_UNBLANK)
#define POWER_IS_OFF(pwr)	((pwr) == FB_BLANK_POWERDOWN)
#define POWER_IS_NRM(pwr)	((pwr) == FB_BLANK_NORMAL)

#define lcd_to_master(a)	(a->dsim_dev->master)
#define lcd_to_master_ops(a)	((lcd_to_master(a))->master_ops)

struct samsung {
	struct device	*dev;
	unsigned int			power;
	unsigned int			id;

	struct lcd_device	*ld;
	struct backlight_device	*bd;

	struct mipi_dsim_lcd_device	*dsim_dev;
	struct lcd_platform_data	*ddi_pd;
	struct mutex			lock;
	struct regulator *lcd_vcc_reg;
	struct regulator *lcd_vcc_reg1;
	bool  enabled;
};


static void samsung_regulator_enable(struct samsung *lcd)
{
	int ret = 0;
	struct lcd_platform_data *pd = NULL;

	pd = lcd->ddi_pd;
	mutex_lock(&lcd->lock);
	if (!lcd->enabled) {
		ret = regulator_enable(lcd->lcd_vcc_reg1);
		if (ret){
            printk("can't get lcd_vcc_reg1\n");
			goto out;
        }
		ret = regulator_enable(lcd->lcd_vcc_reg);
		if (ret){
            printk("can't get lcd_vcc_reg\n");
			goto out;
        }
		lcd->enabled = true;
	}
	msleep(pd->power_on_delay);
out:
	mutex_unlock(&lcd->lock);
}

static void samsung_regulator_disable(struct samsung *lcd)
{
	int ret = 0;

	mutex_lock(&lcd->lock);
	if (lcd->enabled) {
		ret = regulator_disable(lcd->lcd_vcc_reg);
		if (ret){
            printk("can't disable lcd_vcc_reg\n");
			goto out;
        }

		ret = regulator_disable(lcd->lcd_vcc_reg1);
		if (ret){
            printk("can't disable lcd_vcc_reg1\n");
			goto out;
        }
		lcd->enabled = false;
	}
out:
	mutex_unlock(&lcd->lock);
}

static void samsung_sleep_in(struct samsung *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x10, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void samsung_sleep_out(struct samsung *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x11, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void samsung_memory_access(struct samsung *lcd)
{
	unsigned char data_to_send[] = {0x39, 0x01, 0x00, 0x2c};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}


static void samsung_display_on(struct samsung *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x29, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void samsung_display_off(struct samsung *lcd)
{
	unsigned char data_to_send[] = {0x05, 0x28, 0x00};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}


static int samsung_set_power(struct lcd_device *ld, int power)
{
    return 0;
}

static int samsung_get_power(struct lcd_device *ld)
{
	struct samsung *lcd = lcd_get_data(ld);

	return lcd->power;
}

static void samsung_brightness_setting(struct samsung *lcd, unsigned long value)
{
    int array_size;
    struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
    struct dsi_device *dsi = lcd_to_master(lcd);
    unsigned char samsung_cmd_bl[] = { 
		0x39, 0x02, 0x00, 0x51, value,
	    0x39, 0x02, 0x00, 0x53, 0x20
    };  

	array_size = ARRAY_SIZE(samsung_cmd_bl);
	ops->cmd_write(dsi, samsung_cmd_bl, array_size);
}

static void samsung_backlight_update_status(struct backlight_device *bd)
{
	struct samsung *lcd = dev_get_drvdata(&bd->dev);
	int brightness = bd->props.brightness;

	samsung_brightness_setting(lcd, brightness);
}

static struct lcd_ops samsung_lcd_ops = {
	.set_power = samsung_set_power,
	.get_power = samsung_get_power,
};

static const struct backlight_ops samsung_backlight_ops = { 
	.update_status  = samsung_backlight_update_status,
};

static void samsung_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct samsung *lcd = dev_get_drvdata(&dsim_dev->dev);

	if (power)
		samsung_regulator_enable(lcd);
	else
		samsung_regulator_disable(lcd);

	/* lcd power on */
	if (lcd->ddi_pd->power_on)
		lcd->ddi_pd->power_on(lcd->ld, power);
	/* lcd reset */
	if (lcd->ddi_pd->reset)
		lcd->ddi_pd->reset(lcd->ld);
}
unsigned char samsung_cmd_list1[] = {
    0x39, 0x03, 0x00, 0xF2, 0x1C, 0x28,
    0x39, 0x04, 0x00, 0xB5, 0x0A, 0x01, 0x00,/* Frame Freq = 60Hz, ALPM CTL Boosting */
    0x39, 0x05, 0x00, 0x2A, 0x00,0x14,0x01,0x53,/* column address */
    0x39, 0x05, 0x00, 0x2B, 0x01,0x40,0x00,0x01,/* Page address */
    0x39, 0x0E, 0x00, 0xF8, 0x08, 0x08, 0x08, 0x17, 0x00, 0x2A, 0x02, 0x26, 0x00, 0x00, 0x02, 0x00, 0x00,
    0x39, 0x02, 0x00, 0xF7, 0x02,/* Normal2 + BICTL0*/
    0x39, 0x02, 0x00, 0x35, 0x01,/* TE */
};
unsigned char samsung_cmd_list2[] = {
    0x39, 0x02, 0x00, 0x51, 0xEA,/* Write Display Brightness */
    0x39, 0x02, 0x00, 0x53, 0x20,/* Write ControlDisplay */
};
unsigned char samsung_cmd_list3[] = {
    0x39, 0x03, 0x00, 0xB1, 0x00, 0x09,/* ELVSS Condition SET */
    0x39, 0x02, 0x00, 0x36, 0x80,/* reversal left and right */
};

static void samsung_panel_test_key_enable(struct samsung *lcd)
{
    int array_size;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	unsigned char data_to_send0[] = {0x39, 0x03, 0x00, 0xf0, 0x5a, 0x5a};
	unsigned char data_to_send1[] = {0x39, 0x03, 0x00, 0xf1, 0x5a, 0x5a};

	array_size = ARRAY_SIZE(data_to_send0);
	ops->cmd_write(dsi, data_to_send0, array_size);
	array_size = ARRAY_SIZE(data_to_send1);
	ops->cmd_write(dsi, data_to_send1, array_size);
}


static void samsung_panel_test_key_disable(struct samsung *lcd)
{
	unsigned char data_to_send[] = {0x39, 0x03, 0x00, 0xf1, 0xa5, 0xa5};
	int array_size = ARRAY_SIZE(data_to_send);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, data_to_send, array_size);
}

static void samsung_panel_condition_setting(struct samsung *lcd)
{
    int array_size = ARRAY_SIZE(samsung_cmd_list1);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, samsung_cmd_list1, array_size);
}

static void samsung_etc_setting(struct samsung *lcd)
{
    int array_size = ARRAY_SIZE(samsung_cmd_list3);
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	ops->cmd_write(dsi, samsung_cmd_list3, array_size);
}

static void samsung_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
    struct samsung *lcd = dev_get_drvdata(&dsim_dev->dev);
    samsung_panel_test_key_enable(lcd);
    samsung_panel_condition_setting(lcd);
    samsung_sleep_out(lcd);
    msleep(120);
    samsung_brightness_setting(lcd, 0xEA);
    samsung_etc_setting(lcd);
    samsung_memory_access(lcd);
    samsung_panel_test_key_disable(lcd);
    samsung_display_on(lcd);
    lcd->power = FB_BLANK_UNBLANK;
}

static int samsung_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
    	int ret;
	struct samsung *lcd;
	struct backlight_properties props;
	
	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct samsung), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate samsung structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->ddi_pd = (struct lcd_platform_data *)dsim_dev->platform_data;
	lcd->dev = &dsim_dev->dev;

	mutex_init(&lcd->lock);

	lcd->lcd_vcc_reg = regulator_get(NULL, "lcd_1v8");
	if (IS_ERR(lcd->lcd_vcc_reg)) {
		dev_err(lcd->dev, "failed to get regulator vlcd\n");
		return PTR_ERR(lcd->lcd_vcc_reg);
	}

	lcd->lcd_vcc_reg1 = regulator_get(NULL, "lcd_3v0");
	if (IS_ERR(lcd->lcd_vcc_reg1)) {
		dev_err(lcd->dev, "failed to get regulator lcd_3v0\n");
		return PTR_ERR(lcd->lcd_vcc_reg1);
	}

	lcd->ld = lcd_device_register("samsung", lcd->dev, lcd,
			&samsung_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		return PTR_ERR(lcd->ld);
	}
	
	lcd->bd = backlight_device_register("pwm-backlight.0", lcd->dev, lcd,
			                 &samsung_backlight_ops, &props);
	if (IS_ERR(lcd->bd)) {
		dev_err(lcd->dev, "failed to register backlight\n");
		ret = PTR_ERR(lcd->bd);
		return ret;
	}
	
	dev_set_drvdata(&dsim_dev->dev, lcd);

	dev_dbg(lcd->dev, "probed samsung panel driver.\n");

	return 0;
}

#ifdef CONFIG_PM
static int samsung_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct samsung *lcd = dev_get_drvdata(&dsim_dev->dev);

	samsung_display_off(lcd);
	samsung_sleep_in(lcd);
        msleep(120);

	/* lcd reset */
	if (lcd->ddi_pd->reset)
		lcd->ddi_pd->reset(lcd->ld);
	samsung_regulator_disable(lcd);

	return 0;
}

static int samsung_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct samsung *lcd = dev_get_drvdata(&dsim_dev->dev);

	samsung_regulator_enable(lcd);
	return 0;
}
#else
#define samsung_suspend		NULL
#define samsung_resume		NULL
#endif

static struct mipi_dsim_lcd_driver samsung_dsim_ddi_driver = {
	.name = "samsung-lcd",
	.id = 0,

	.power_on = samsung_power_on,
	.set_sequence = samsung_set_sequence,
	.probe = samsung_probe,
	.suspend = samsung_suspend,
	.resume = samsung_resume,
};

static int samsung_init(void)
{
	mipi_dsi_register_lcd_driver(&samsung_dsim_ddi_driver);
	return 0;
}

static void samsung_exit(void)
{
	return;
}

module_init(samsung_init);
module_exit(samsung_exit);

MODULE_DESCRIPTION("samsung lcd driver");
MODULE_LICENSE("GPL");
