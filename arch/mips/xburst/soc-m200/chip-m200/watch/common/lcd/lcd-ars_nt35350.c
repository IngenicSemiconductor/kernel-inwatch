/*
 * Copyright (C) 2015 Ingenic Electronics
 *
 * ARS NT35350 LTPS TFT 360x360 LCD Driver (driver's data part)
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>

#include <mach/jzfb.h>
#include <mach/jz_dsim.h>
#include "../board_base.h"

static struct regulator *lcd_vcc_reg = NULL;
static struct regulator *lcd_io_reg = NULL;
static bool is_init = 0;

int ars_nt35350_init(struct lcd_device *lcd)
{
	int ret = 0;

	lcd_vcc_reg = regulator_get(NULL, VCC_LCD_2V8_NAME);
	if (IS_ERR(lcd_vcc_reg)) {
		dev_err(&lcd->dev, "failed to get regulator %s\n", VCC_LCD_2V8_NAME);
		return PTR_ERR(lcd_vcc_reg);
	}

	lcd_io_reg = regulator_get(NULL, VCC_LCD_1V8_NAME);
	if (IS_ERR(lcd_io_reg)) {
		dev_err(&lcd->dev, "failed to get regulator %s\n", VCC_LCD_1V8_NAME);
		return PTR_ERR(lcd_io_reg);
	}

	ret = gpio_request(GPIO_MIPI_RST_N, "mipi reset pin");
	if (ret) {
		dev_err(&lcd->dev,"can't request mipi reset pin\n");
		return ret;
	}

	ret = gpio_request(GPIO_LCD_BLK_EN, "bl power enable pin");
	if (ret) {
		dev_err(&lcd->dev,"can't request backlight power enable pin\n");
		return ret;
	}

	is_init = 1;

	return ret;
}

int ars_nt35350_reset(struct lcd_device *lcd)
{
	gpio_direction_output(GPIO_MIPI_RST_N, 1);
	udelay(100);
	gpio_direction_output(GPIO_MIPI_RST_N, 0);
	udelay(100);
	gpio_direction_output(GPIO_MIPI_RST_N, 1);
	udelay(300);
	return 0;
}

int ars_nt35350_power_on(struct lcd_device *lcd, int enable)
{
	if(!is_init && ars_nt35350_init(lcd))
		return -EFAULT;

	if (enable == POWER_ON_LCD) {
		regulator_enable(lcd_vcc_reg);
		regulator_enable(lcd_io_reg);
	} else if (enable == POWER_ON_BL){
		gpio_direction_output(GPIO_LCD_BLK_EN, 1);
	} else {
		/* power off the power of LCD and it's Backlight */
		gpio_direction_output(GPIO_MIPI_RST_N, 0);
		regulator_disable(lcd_io_reg);
		regulator_disable(lcd_vcc_reg);
		gpio_direction_output(GPIO_LCD_BLK_EN, 0);
	}

	return 0;
}

struct lcd_platform_data ars_nt35350_data = {
	.reset    = ars_nt35350_reset,
	.power_on = ars_nt35350_power_on,
	.lcd_enabled = 1, // lcd panel was enabled from uboot
};

struct mipi_dsim_lcd_device ars_nt35350_device={
	.name = "ars_nt35350-lcd",
	.id   = 0,
	.platform_data = &ars_nt35350_data,
};

unsigned long ars_nt35350_cmd_buf[]= {
	0x2C2C2C2C,
};

struct fb_videomode jzfb_videomode = {
	.name = "ars_nt35350-lcd",
	.refresh = 60,
	.xres = 360,
	.yres = 360,
	.pixclock = KHZ2PICOS(7776),
	.left_margin  = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = ~FB_SYNC_HOR_HIGH_ACT & ~FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

struct jzdsi_data jzdsi_pdata = {
	.modes = &jzfb_videomode,
	.video_config.no_of_lanes = 1,
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
	.video_config.receive_ack_packets = 0,	/* enable receiving of ack packets */
	.video_config.is_18_loosely = 0, /*loosely: R0R1R2R3R4R5__G0G1G2G3G4G5G6__B0B1B2B3B4B5B6, not loosely: R0R1R2R3R4R5G0G1G2G3G4G5B0B1B2B3B4B5*/
	.video_config.data_en_polarity = 1,

	.dsi_config.max_lanes = 1,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.max_bps = 500, /* 500Mbps */
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.te_gpio = DSI_TE_GPIO,
	.dsi_config.te_irq_level = IRQF_TRIGGER_RISING,
};

struct jzfb_platform_data jzfb_pdata = {
	.name = "ars_nt35350-lcd",
	.num_modes = 1,
	.modes = &jzfb_videomode,
	.dsi_pdata = &jzdsi_pdata,

	.lcd_type = LCD_TYPE_SLCD,
	.bpp = 24,
	.width = 31,
	.height = 31,

	.smart_config.clkply_active_rising = 0,
	.smart_config.rsply_cmd_high = 0,
	.smart_config.csply_active_high = 0,
	.smart_config.write_gram_cmd = ars_nt35350_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(ars_nt35350_cmd_buf),
	.smart_config.bus_width = 8,
	.dither_enable = 1,
	.dither.dither_red   = 1,	/* 6bit */
	.dither.dither_green = 1,	/* 6bit */
	.dither.dither_blue  = 1,	/* 6bit */

	.lcd_desc = "CIRCULARITY_360_360_1.3_TFT", /* use to create sysfs node */
};

/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM
#ifndef CONFIG_CLEAR_PWM_OUTPUT
#warning "Not define CONFIG_CLEAR_PWM_OUTPUT at 'make menuconfig' !"
#endif
static int backlight_init(struct device *dev)
{
	int ret = 0;

	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request PWM for Backlight\n");
		return ret;
	}

	return 0;
}

static void backlight_exit(struct device *dev)
{
	gpio_free(GPIO_LCD_PWM);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 120,
	.pwm_period_ns	= 30000,
	.init		= backlight_init,
	.exit		= backlight_exit,
};

struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &backlight_data,
	},
};
#endif

