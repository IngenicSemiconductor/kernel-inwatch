/*
 * Copyright (c) 2014 Engenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * JZ-M200 orion board lcd setup routines.
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

/*ifdef is 18bit,6-6-6 ,ifndef default 5-6-6*/
//#define CONFIG_SLCD_TRULY_18BIT

#ifdef	CONFIG_SLCD_TRULY_18BIT
static int slcd_inited = 1;
#else
static int slcd_inited = 0;
#endif

struct truly_tft320320_power{
	struct regulator *vlcdio;
	struct regulator *vlcdvcc;
	int inited;
};

static struct truly_tft320320_power lcd_power = {
	NULL,
    NULL,
    0
};

int truly_tft320320_power_init(struct lcd_device *ld)
{
    int ret ;
    printk("======truly_tft320320_power_init==============\n");
    if(GPIO_LCD_RST > 0){
        ret = gpio_request(GPIO_LCD_RST, "lcd rst");
        if (ret) {
            printk(KERN_ERR "can's request lcd rst\n");
            return ret;
        }
    }
    if(GPIO_LCD_CS > 0){
        ret = gpio_request(GPIO_LCD_CS, "lcd cs");
        if (ret) {
            printk(KERN_ERR "can's request lcd cs\n");
            return ret;
        }
    }
    if(GPIO_LCD_RD > 0){
        ret = gpio_request(GPIO_LCD_RD, "lcd rd");
        if (ret) {
            printk(KERN_ERR "can's request lcd rd\n");
            return ret;
        }
    }
    printk("set lcd_power.inited  =======1 \n");
    lcd_power.inited = 1;
    return 0;
}

int truly_tft320320_power_reset(struct lcd_device *ld)
{
	if (!lcd_power.inited)
		return -EFAULT;
	gpio_direction_output(GPIO_LCD_RST, 0);
	mdelay(20);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(10);

	return 0;
}

int truly_tft320320_power_on(struct lcd_device *ld, int enable)
{
	if (!lcd_power.inited && truly_tft320320_power_init(ld))
		return -EFAULT;

	if (enable) {
		gpio_direction_output(GPIO_LCD_CS, 1);
		gpio_direction_output(GPIO_LCD_RD, 1);

		truly_tft320320_power_reset(ld);

		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);

	} else {
		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);
		gpio_direction_output(GPIO_LCD_RD, 0);
		gpio_direction_output(GPIO_LCD_RST, 0);
		slcd_inited = 0;
	}
	return 0;
}

struct lcd_platform_data truly_tft320320_pdata = {
	.reset    = truly_tft320320_power_reset,
	.power_on = truly_tft320320_power_on,
};

/* LCD Panel Device */
struct platform_device truly_tft320320_device = {
	.name		= "truly_tft320320_slcd",
	.dev		= {
		.platform_data	= &truly_tft320320_pdata,
	},
};

static struct smart_lcd_data_table truly_tft320320_data_table[] = {
    {SMART_CONFIG_CMD, 0x38}, ///Exit idle mode

    {SMART_CONFIG_CMD, 0x11},
    {SMART_CONFIG_UDELAY, 200000},

	{SMART_CONFIG_CMD, 0xB9},  // SET password
	{SMART_CONFIG_DATA, 0xFF},
	{SMART_CONFIG_DATA, 0x83},
	{SMART_CONFIG_DATA, 0x57},
    {SMART_CONFIG_UDELAY, 1000},

    {SMART_CONFIG_CMD, 0xB0},
	{SMART_CONFIG_DATA, 0x66},//change lcd internal refresh rate

	{SMART_CONFIG_CMD, 0xCC},
	{SMART_CONFIG_DATA, 0x09},



	{SMART_CONFIG_CMD, 0x3A},
	{SMART_CONFIG_DATA, 0x55},



	{SMART_CONFIG_CMD, 0xB1},   //SETPower
	{SMART_CONFIG_DATA, 0x00},	  //Ref from 57-C BOE code
	{SMART_CONFIG_DATA, 0x14},//TRI XDK BT
	{SMART_CONFIG_DATA, 0x1E},//VRH
	{SMART_CONFIG_DATA, 0x1E},//NVRH
	{SMART_CONFIG_DATA, 0xC3},//GAS_EN  FP_DDVDH AP
	{SMART_CONFIG_DATA, 0x44},//FS1 FS0
    {SMART_CONFIG_UDELAY, 1000},

	{SMART_CONFIG_CMD, 0xB2},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x83},
	{SMART_CONFIG_DATA, 0x27},//3B

	{SMART_CONFIG_CMD, 0xB6},
	{SMART_CONFIG_DATA, 0x50},

	{SMART_CONFIG_CMD, 0xB5},   //SETCYC
	{SMART_CONFIG_DATA, 0x0B},	  //VREF=4.8V
	{SMART_CONFIG_DATA, 0x0B},    //NVREF=-4.8V
	{SMART_CONFIG_DATA, 0x67},    //VPP-VVDHS ,VSP=5.475V

	{SMART_CONFIG_CMD, 0xB4},   //SETCYC
	{SMART_CONFIG_DATA, 0x22},	  //2-dot
	{SMART_CONFIG_DATA, 0x40},//RTN
	{SMART_CONFIG_DATA, 0x00},//DIV
	{SMART_CONFIG_DATA, 0x2A},//DUM
	{SMART_CONFIG_DATA, 0x2A},//DUM
	{SMART_CONFIG_DATA, 0x20},//GDON
	{SMART_CONFIG_DATA, 0x91},//GDOFF

	{SMART_CONFIG_CMD, 0xC0},   //
	{SMART_CONFIG_DATA, 0x60},	  //
	{SMART_CONFIG_DATA, 0x60},//OPON
	{SMART_CONFIG_DATA, 0x01},	//
	{SMART_CONFIG_DATA, 0x3C},
	{SMART_CONFIG_DATA, 0xC8},
	{SMART_CONFIG_DATA, 0x08},

	{SMART_CONFIG_CMD, 0xC2},   //
	{SMART_CONFIG_DATA, 0x00},	  //
	{SMART_CONFIG_DATA, 0x08},
	{SMART_CONFIG_DATA, 0x05},	//

    {SMART_CONFIG_CMD, 0xE3},   //
	{SMART_CONFIG_DATA, 0x17},	  //
	{SMART_CONFIG_DATA, 0x0F},

	{SMART_CONFIG_CMD, 0xE0},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_DATA, 0x0A},
	{SMART_CONFIG_DATA, 0x20},
	{SMART_CONFIG_DATA, 0x32},
	{SMART_CONFIG_DATA, 0x3F},
	{SMART_CONFIG_DATA, 0x54},
	{SMART_CONFIG_DATA, 0x4A},
	{SMART_CONFIG_DATA, 0x40},
	{SMART_CONFIG_DATA, 0x33},
	{SMART_CONFIG_DATA, 0x2F},
	{SMART_CONFIG_DATA, 0x27},
	{SMART_CONFIG_DATA, 0x23},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},
	{SMART_CONFIG_DATA, 0x02},
	{SMART_CONFIG_DATA, 0x04},
	{SMART_CONFIG_DATA, 0x0A},
	{SMART_CONFIG_DATA, 0x20},
	{SMART_CONFIG_DATA, 0x32},
	{SMART_CONFIG_DATA, 0x3F},
	{SMART_CONFIG_DATA, 0x54},
	{SMART_CONFIG_DATA, 0x4A},
	{SMART_CONFIG_DATA, 0x40},
	{SMART_CONFIG_DATA, 0x33},
	{SMART_CONFIG_DATA, 0x2F},
	{SMART_CONFIG_DATA, 0x27},
	{SMART_CONFIG_DATA, 0x23},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x00},
	{SMART_CONFIG_DATA, 0x01},
    {SMART_CONFIG_UDELAY, 1000},

    {SMART_CONFIG_CMD, 0x35},
    {SMART_CONFIG_DATA, 0x00},  //enable TE

	{SMART_CONFIG_CMD, 0x29},
    {SMART_CONFIG_UDELAY, 10000},
//#endif
	/*Display on */
	{SMART_CONFIG_CMD, 0x02},
	{SMART_CONFIG_DATA, 0x01},

	/* red draw */
	{SMART_CONFIG_CMD, 0x0c},

};
unsigned long truly_cmd_buf[]= {
	0x2C2C2C2C,
};

struct fb_videomode jzfb0_videomode = {
	.name = "320x320",
	.refresh = 45,
	.xres = 320,
	.yres = 320,
	.pixclock = KHZ2PICOS(5760),
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};


struct jzfb_platform_data jzfb_pdata = {
	.name = "truly320320"
	.num_modes = 1,
	.modes = &jzfb0_videomode,
	.lcd_type = LCD_TYPE_SLCD,
	.bpp = 18,		/* set 32bpp framebuffer rgba8888 */
	.width = 31,
	.height = 31,
	.pinmd  = 0,

	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	.smart_config.newcfg_fmt_conv =  1,
	/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
	.smart_config.write_gram_cmd = truly_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(truly_cmd_buf),
	.smart_config.bus_width = 8,
	.smart_config.length_data_table =  ARRAY_SIZE(truly_tft320320_data_table),
	.smart_config.data_table = truly_tft320320_data_table,
	.smart_config.te_gpio = SLCD_TE_GPIO,
	.smart_config.te_irq_level = IRQF_TRIGGER_RISING,
	.dither_enable = 0,
};
/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM
static int backlight_init(struct device *dev)
{
	int ret;
	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request GPF for PWM-OUT1\n");
		return ret;
	}

	ret = gpio_request(GPIO_BL_PWR_EN, "BL PWR");
	if (ret) {
		printk(KERN_ERR "failed to reqeust BL PWR\n");
		return ret;
	}
	gpio_direction_output(GPIO_BL_PWR_EN, 1);
	return 0;
}

static int backlight_notify(struct device *dev, int brightness)
{
	if (brightness)
		gpio_direction_output(GPIO_BL_PWR_EN, 1);
	else
		gpio_direction_output(GPIO_BL_PWR_EN, 0);

	return brightness;
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
	.notify		= backlight_notify,
};

struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &backlight_data,
	},
};
#endif
