/*
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 *  dorado board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/regulator/consumer.h>

#include <mach/jzfb.h>
#include "../board_base.h"

static struct regulator *lcd_vcc_reg = NULL;
static struct regulator *lcd_io_reg = NULL;
static bool is_init = 0;

int orise_otm3201a_init(struct lcd_device *lcd)
{
	int ret = 0;

	lcd_vcc_reg = regulator_get(NULL, "2V8_LED_VDD");
	if (IS_ERR(lcd_vcc_reg)) {
		dev_err(&lcd->dev, "failed to get regulator 2V8_LED_VDD\n");
		return PTR_ERR(lcd_vcc_reg);
	}

	lcd_io_reg = regulator_get(NULL, "1V8_LED_IO");
	if (IS_ERR(lcd_io_reg)) {
		dev_err(&lcd->dev, "failed to get regulator 1V8_LED_IO\n");
		return PTR_ERR(lcd_io_reg);
	}

	ret = gpio_request(GPIO_MIPI_RST_N, "mipi reset pin");
	if (ret) {
		dev_err(&lcd->dev,"can's request mipi reset pin\n");
		return ret;
	}
	is_init = 1;

	return ret;
}

int orise_otm3201a_reset(struct lcd_device *lcd)
{
	gpio_direction_output(GPIO_MIPI_RST_N, 1);
	mdelay(300);
	gpio_direction_output(GPIO_MIPI_RST_N, 0);
	msleep(10);
	gpio_direction_output(GPIO_MIPI_RST_N, 1);
	mdelay(50);
	return 0;
}

int orise_otm3201a_power_on(struct lcd_device *lcd, int enable)
{
	int ret;

	if(!is_init && orise_otm3201a_init(lcd))
		return -EFAULT;

	if (enable) {
		ret = regulator_enable(lcd_vcc_reg);
		if (ret)
			printk(KERN_ERR "failed to enable lcd vcc reg\n");
		ret = regulator_enable(lcd_io_reg);
		if (ret)
			printk(KERN_ERR "failed to enable lcd io reg\n");
	} else {
		regulator_disable(lcd_io_reg);
		regulator_disable(lcd_vcc_reg);
	}

	return 0;
}
struct lcd_platform_data orise_otm3201a_data = {
	.reset = orise_otm3201a_reset,
	.power_on= orise_otm3201a_power_on,
	.lcd_enabled = 1, //lcd is enabled from bootloader
};

struct mipi_dsim_lcd_device orise_otm3201a_device={
	.name = "orise_otm3201a-lcd",
	.id   = 0,
	.platform_data = &orise_otm3201a_data,
};

/**
 * .pixclock comes from DataSheet(DATA SHEET_OTM3201A_V03_Truly.pdf)
 * .left_margin = THBP
 * .right_margin= THFP
 * .upper_margin= TVBP
 * .lower_margin= TVFP
 * .hsync_len = THS
 * .vsync_len = TVS
 */
struct fb_videomode jzfb_videomode = {
	.name = "orise_otm3201a-lcd",
	.refresh = 60,
	.xres = 320,
	.yres = 320,
	.pixclock = KHZ2PICOS(5310), //PCLK Frequency: 5.31MHz
	.left_margin = 42, // HBP
	.right_margin = 26, // HFP
	.upper_margin = 10, // VBP
	.lower_margin = 2, // VFP
	.hsync_len = 10, // HSA
	.vsync_len = 2, // VSA
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
	/*loosely: R0R1R2R3R4R5__G0G1G2G3G4G5G6__B0B1B2B3B4B5B6,
	 * not loosely: R0R1R2R3R4R5G0G1G2G3G4G5B0B1B2B3B4B5*/
	.video_config.pixel_clock = 5310,	/* dpi_clock */
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,

	.dsi_config.max_lanes = 1,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.max_bps = 500, /* 500Mbps */
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
};

struct jzfb_platform_data jzfb_pdata = {
	.name = "orise_otm3201a-lcd",
	.num_modes = 1,
	.modes = &jzfb_videomode,
	.dsi_pdata = &jzdsi_pdata,

	.lcd_type = LCD_TYPE_GENERIC_24_BIT,
	.bpp = 24,
	.width = 31,
	.height = 31,

	.pixclk_falling_edge = 0,
	.data_enable_active_low = 0,
};

/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM
static int backlight_init(struct device *dev)
{
//	int ret;
//	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
//	if (ret) {
//		return ret;
//	}

//	gpio_direction_output(GPIO_BL_PWR_EN, 1);

	return 0;
}

static void backlight_exit(struct device *dev)
{
//	gpio_free(GPIO_LCD_PWM);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 2,
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
