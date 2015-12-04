/*
 * drivers/input/touchscreen/focaltech.c
 *
 * FocalTech focaltech TouchScreen driver.
 *
 * Copyright (c) 2014  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *    1.1		  2014-09			mshl
 *
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/suspend.h>

#include <linux/tsc.h>

#if(defined(CONFIG_I2C_SPRD) || defined(CONFIG_I2C_SPRD_V1))
#include <mach/i2c-sprd.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//#define FTS_DBG
#ifdef FTS_DBG
#define ENTER printk(KERN_INFO "[FTS_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[FTS_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FTS_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FTS_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FTS_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FTS_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FTS_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FTS_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

//#define FTS_GESTRUE //no test for use

#include <linux/i2c/focaltech.h>
#include <linux/i2c/focaltech_ex_fun.h>
#include <linux/i2c/focaltech_ctl.h>

#define	USE_WAIT_QUEUE	0
#define	USE_THREADED_IRQ	0
#define	USE_WORK_QUEUE	1

//#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B 0
#define	TS_MAX_FINGER		5

#define	FTS_PACKET_LENGTH	128

#if USE_WAIT_QUEUE
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;
#endif

static struct fts_data *g_fts;
static struct i2c_client *this_client;
static 	struct regulator *reg_vdd;

/*static unsigned char suspend_flag = 0;*/

struct Upgrade_Info fts_updateinfo[] =
{
	{0x0e,"FT3x0x",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},

};

struct Upgrade_Info fts_updateinfo_curr;
#ifdef FTS_GESTRUE
#include "ft_gesture_lib.h"
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

short pointnum = 0;
unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};

extern int fetch_object_sample(unsigned char *buf,short pointnum);
extern void init_para(int x_pixel,int y_pixel,int time_slot,int cut_x_pixel,int cut_y_pixel);
suspend_state_t get_suspend_state(void);
#endif

#if defined(READ_FW_VER)
static unsigned char fts_read_fw_ver(void);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fts_ts_suspend(struct early_suspend *handler);
static void fts_ts_resume(struct early_suspend *handler);
#endif

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct fts_data {
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	struct ts_event	event;
#if USE_WORK_QUEUE
	struct work_struct	pen_event_work;
	struct workqueue_struct	*fts_workqueue;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct work_struct       resume_work;
	struct workqueue_struct *fts_resume_workqueue;
	struct early_suspend	early_suspend;
#endif
	struct fts_platform_data	*platform_data;
};

#ifdef TOUCH_VIRTUAL_KEYS

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct fts_data *data = i2c_get_clientdata(this_client);
	struct fts_platform_data *pdata = data->platform_data;
	return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
		,__stringify(EV_KEY), __stringify(KEY_MENU),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
		,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
		,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.focaltech_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void fts_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    pr_info("[FST] %s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

#endif

/***********************************************************************************************
Name	:	 fts_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
#if defined(READ_FW_VER)
static unsigned char fts_read_fw_ver(void)
{
	unsigned char ver;
	fts_read_reg(this_client, FTS_REG_FIRMID, &ver);
	return(ver);
}
#endif

static void fts_clear_report_data(struct fts_data *fts)
{
	int i;

	for(i = 0; i < TS_MAX_FINGER; i++) {
	#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(fts->input_dev, i);
		input_mt_report_slot_state(fts->input_dev, MT_TOOL_FINGER, false);
	#endif
	}
	input_report_key(fts->input_dev, BTN_TOUCH, 0);
	#if !MULTI_PROTOCOL_TYPE_B
		input_mt_sync(fts->input_dev);
	#endif
	input_sync(fts->input_dev);
}

static int fts_update_data(void)
{
	struct fts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[33] = {0};
	int ret = -1;
	int i;
	u16 x , y;
	u8 ft_pressure , ft_size;

	ret = fts_i2c_Read(this_client, buf, 1, buf, 33);

	if (ret < 0) {
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	for(i = 0; i < TS_MAX_FINGER; i++) {
		if((buf[6*i+3] & 0xc0) == 0xc0)
			continue;
		x = (s16)(buf[6*i+3] & 0x0F)<<8 | (s16)buf[6*i+4];
		y = (s16)(buf[6*i+5] & 0x0F)<<8 | (s16)buf[6*i+6];

#ifdef CONFIG_TSC_SWAP_XY
		tsc_swap_xy(&x,&y);
#endif

#ifdef CONFIG_TSC_SWAP_X
		tsc_swap_x(&x,data->platform_data->TP_MAX_X);
#endif

#ifdef CONFIG_TSC_SWAP_Y
		tsc_swap_y(&y,data->platform_data->TP_MAX_Y);
#endif
		ft_pressure = buf[6*i+7];
		if(ft_pressure > 127 || ft_pressure == 0)
			ft_pressure = 127;
		ft_size = (buf[6*i+8]>>4) & 0x0F;
		if(ft_size == 0)
		{
			ft_size = 0x09;
		}
		if((buf[6*i+3] & 0x40) == 0x0) {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
		#else
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, buf[6*i+5]>>4);
		#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, ft_pressure);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
			input_report_key(data->input_dev, BTN_TOUCH, 1);

		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif
			pr_debug("===x%d = %d,y%d = %d ====",i, x, i, y);
		}
		else {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		#endif
		}
	}
	if(0 == event->touch_point) {
		for(i = 0; i < TS_MAX_FINGER; i ++) {
			#if MULTI_PROTOCOL_TYPE_B
                            input_mt_slot(data->input_dev, i);
                            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			#endif
		}
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif

	}

	input_sync(data->input_dev);
	return 0;
}

#ifdef FTS_GESTRUE
static void check_gesture(int gesture_id)
{
	struct fts_data *data = i2c_get_clientdata(this_client);
	printk("kaka gesture_id==0x%x\n ",gesture_id);
	switch(gesture_id)
	{
		case GESTURE_LEFT:
			input_report_key(data->input_dev, KEY_LEFT, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_LEFT, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_RIGHT:
			input_report_key(data->input_dev, KEY_RIGHT, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_RIGHT, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_UP:
			input_report_key(data->input_dev, KEY_UP, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_UP, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_DOWN:
			input_report_key(data->input_dev, KEY_DOWN, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_DOWN, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_DOUBLECLICK:
			input_report_key(data->input_dev, KEY_U, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_U, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_O:
			input_report_key(data->input_dev, KEY_O, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_O, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_W:
			input_report_key(data->input_dev, KEY_W, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_W, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_M:
			input_report_key(data->input_dev, KEY_M, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_M, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_E:
			input_report_key(data->input_dev, KEY_E, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_E, 0);
			input_sync(data->input_dev);
			break;
		case GESTURE_C:
			input_report_key(data->input_dev, KEY_C, 1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev, KEY_C, 0);
			input_sync(data->input_dev);
			break;
		default:
			break;
	}
}
static int fts_read_Gestruedata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
    int ret = -1;
    int i = 0;
    int gestrue_id = 0;
    buf[0] = 0xd3;

    pointnum = 0;

    ret = fts_i2c_Read(this_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
                printk( "tpd read FTS_GESTRUE_POINTS_HEADER.\n");

    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }

    /* FW */
     if (fts_updateinfo_curr.CHIP_ID==0x54)
     {
		gestrue_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;

		if((pointnum * 4 + 8)<255)
		{
	                ret = fts_i2c_Read(this_client, buf, 1, buf, (pointnum * 4 + 8));
		}
		else
		{
	                ret = fts_i2c_Read(this_client, buf, 1, buf, 255);
	                ret = fts_i2c_Read(this_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
		}
		if (ret < 0)
		{
	                printk( "%s read touchdata failed.\n", __func__);
	                return ret;
		}
		check_gesture(gestrue_id);
		for(i = 0;i < pointnum;i++)
		{
	                coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
	                8 | (((s16) buf[1 + (4 * i)])& 0xFF);
	                coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
	                8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
		}
		for(i = 0; i < pointnum; i++)
		{
	                printk("coordinate_x[%d]  = %d",i,coordinate_x[i] );
		}
		return -1;
    }

    if (0x24 == buf[0])

    {
        gestrue_id = 0x24;
        check_gesture(gestrue_id);
        return -1;
    }

    pointnum = (short)(buf[1]) & 0xff;
    buf[0] = 0xd3;

    if((pointnum * 4 + 8)<255)
    {
         ret = fts_i2c_Read(this_client, buf, 1, buf, (pointnum * 4 + 8));
    }
    else
    {
         ret = fts_i2c_Read(this_client, buf, 1, buf, 255);
         ret = fts_i2c_Read(this_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
    }
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }

   gestrue_id = fetch_object_sample(buf, pointnum);
   check_gesture(gestrue_id);

    for(i = 0;i < pointnum;i++)
    {
        coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[1 + (4 * i)])& 0xFF);
        coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
    }
    return -1;
}
#endif
#if USE_WAIT_QUEUE
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 5 };
	u8 state;
	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, (0 != tpd_flag));
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
#ifdef FTS_GESTRUE
		i2c_smbus_read_i2c_block_data(this_client, 0xd0, 1, &state);
		if( state ==1)
		{
	                fts_read_Gestruedata();
	                continue;
		}
#endif
		fts_update_data();

	} while (!kthread_should_stop());

	return 0;
}
#endif

#if USE_WORK_QUEUE
static void fts_pen_irq_work(struct work_struct *work)
{
	fts_update_data();
	enable_irq(this_client->irq);
}
#endif

static irqreturn_t fts_interrupt(int irq, void *dev_id)
{

#if USE_WAIT_QUEUE
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
#endif

#if USE_WORK_QUEUE
	struct fts_data *fts = (struct fts_data *)dev_id;

	disable_irq_nosync(this_client->irq);

	if (!work_pending(&fts->pen_event_work)) {
		queue_work(fts->fts_workqueue, &fts->pen_event_work);
	}
	return IRQ_HANDLED;
#endif

#if USE_THREADED_IRQ
	fts_update_data();
	return IRQ_HANDLED;
#endif

}

static void fts_reset(void)
{
	struct fts_platform_data *pdata = g_fts->platform_data;

	gpio_direction_output(pdata->reset_gpio_number, 1);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(200);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fts_ts_suspend(struct early_suspend *handler)
{
	int ret = -1;
	pr_info("==%s==\n", __FUNCTION__);
#ifdef FTS_GESTRUE
	fts_write_reg(this_client, 0xd0, 0x01);
if (fts_updateinfo_curr.CHIP_ID==0x54)
{
	fts_write_reg(this_client, 0xd1, 0xff);
	fts_write_reg(this_client, 0xd2, 0xff);
	fts_write_reg(this_client, 0xd5, 0xff);
	fts_write_reg(this_client, 0xd6, 0xff);
	fts_write_reg(this_client, 0xd7, 0xff);
	fts_write_reg(this_client, 0xd8, 0xff);
}
	/*return 0;*/
	return;
#endif
	ret = fts_write_reg(this_client, FTS_REG_PMODE, PMODE_HIBERNATE);

	if(ret < 0){
		PRINT_ERR("==fts_suspend==  fts_write_reg fail\n");
	}

	disable_irq(this_client->irq);
	fts_clear_report_data(g_fts);
	regulator_disable(reg_vdd);
}

static void fts_ts_resume(struct early_suspend *handler)
{
	struct fts_data  *fts = (struct fts_data *)i2c_get_clientdata(this_client);

	regulator_enable(reg_vdd);
	msleep(100);

	queue_work(fts->fts_resume_workqueue, &fts->resume_work);
}

static void fts_resume_work(struct work_struct *work)
{
	pr_info("==%s==\n", __FUNCTION__);
#ifdef FTS_GESTRUE
	fts_write_reg(this_client,0xD0,0x00);
#endif
	fts_reset();
	enable_irq(this_client->irq);
	msleep(2);
	fts_clear_report_data(g_fts);
}
#endif

static void fts_hw_init(struct fts_data *fts)
{
	struct i2c_client *client = fts->client;
	struct fts_platform_data *pdata = fts->platform_data;

	pr_info("[FST] %s [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
	gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);

	reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	if (!WARN(IS_ERR(reg_vdd), "[FST] fts_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		regulator_set_voltage(reg_vdd, 2800000, 2800000);
		regulator_enable(reg_vdd);
	}
	msleep(100);
	fts_reset();
}

void focaltech_get_upgrade_array(struct i2c_client *client)
{

	u8 chip_id;
	u32 i;

	i2c_smbus_read_i2c_block_data(client,FT_REG_CHIP_ID,1,&chip_id);

	printk("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

#ifdef CONFIG_OF
static struct fts_platform_data *fts_parse_dt(struct device *dev)
{
	struct fts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct fts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", &pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif


static int fts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fts_data *fts;
	struct input_dev *input_dev;
	struct fts_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	unsigned char uc_reg_value;

	pr_info("[FST] %s: probe\n",__func__);

#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	if (np && !pdata){
		pdata = fts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	fts = kzalloc(sizeof(*fts), GFP_KERNEL);
	if (!fts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_fts = fts;
	fts->platform_data = pdata;
	this_client = client;
	fts->client = client;
	fts_hw_init(fts);
	i2c_set_clientdata(client, fts);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	#if(defined(CONFIG_I2C_SPRD) ||defined(CONFIG_I2C_SPRD_V1))
	sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000);
	#endif

	err = fts_read_reg(this_client, FTS_REG_CIPHER, &uc_reg_value);
	if (err < 0)
	{
		pr_err("[FST] read chip id error %x\n", uc_reg_value);
		err = -ENODEV;
		goto exit_chip_check_failed;
	}

#if USE_WORK_QUEUE
	INIT_WORK(&fts->pen_event_work, fts_pen_irq_work);

	fts->fts_workqueue = create_singlethread_workqueue("focal-work-queue");
	if (!fts->fts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	INIT_WORK(&fts->resume_work, fts_resume_work);
	fts->fts_resume_workqueue = create_singlethread_workqueue("fts_resume_work");
	if (!fts->fts_resume_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[FST] failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	fts_virtual_keys_init();
#endif
	fts->input_dev = input_dev;

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER);
#endif
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_PRESSURE, 0, 127, 0, 0);
#if !MULTI_PROTOCOL_TYPE_B
	input_set_abs_params(input_dev,ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name = FOCALTECH_TS_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"[FST] fts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#if USE_THREADED_IRQ
	err = request_threaded_irq(client->irq, NULL, fts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, fts);
#else
	err = request_irq(client->irq, fts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, fts);
#endif
	if (err < 0) {
		dev_err(&client->dev, "[FST] ft5x0x_probe: request irq failed %d\n",err);
		goto exit_irq_request_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	fts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	fts->early_suspend.suspend = fts_ts_suspend;
	fts->early_suspend.resume	= fts_ts_resume;
	register_early_suspend(&fts->early_suspend);
#endif

focaltech_get_upgrade_array(client);

#ifdef CONFIG_FOCALTECH_SYSFS_DEBUG
fts_create_sysfs(client);
#endif

#ifdef CONFIG_FOCALTECH_FTS_CTL_IIC
if (ft_rw_iic_drv_init(client) < 0)
{
	dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",	__func__);
}
#endif

#ifdef FTS_GESTRUE
	init_para(720,1280,0,0,0);
#endif
#ifdef CONFIG_FOCALTECH_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(client);
#endif

#ifdef CONFIG_FOCALTECH_APK_DEBUG
	fts_create_apk_debug_channel(client);
#endif

#if USE_WAIT_QUEUE
	thread = kthread_run(touch_event_handler, 0, "focal-wait-queue");
	if (IS_ERR(thread))
	{
		err = PTR_ERR(thread);
		PRINT_ERR("failed to create kernel thread: %d\n", err);
	}
#endif

	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_create_singlethread:
exit_chip_check_failed:
	gpio_free(pdata->irq_gpio_number);
	gpio_free(pdata->reset_gpio_number);
	kfree(fts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	fts = NULL;
	i2c_set_clientdata(client, fts);
#ifdef CONFIG_OF
exit_alloc_platform_data_failed:
#endif
	return err;
}

static int fts_remove(struct i2c_client *client)
{
	struct fts_data *fts = i2c_get_clientdata(client);

	pr_info("==fts_remove=\n");

	#ifdef CONFIG_FOCALTECH_SYSFS_DEBUG
	fts_release_sysfs(client);
	#endif
	#ifdef CONFIG_FOCALTECH_FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	#ifdef CONFIG_FOCALTECH_APK_DEBUG
	fts_release_apk_debug_channel();
	#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&fts->early_suspend);
#endif
	free_irq(client->irq, fts);
	input_unregister_device(fts->input_dev);
	input_free_device(fts->input_dev);
#if USE_WORK_QUEUE
	cancel_work_sync(&fts->pen_event_work);
	destroy_workqueue(fts->fts_workqueue);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	cancel_work_sync(&fts->resume_work);
	destroy_workqueue(fts->fts_resume_workqueue);
#endif
	kfree(fts);
	fts = NULL;
	i2c_set_clientdata(client, fts);

	return 0;
}

static const struct i2c_device_id fts_id[] = {
	{ FOCALTECH_TS_NAME, 0 },{ }
};

static int fts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	PRINT_INFO("fts_suspend\n");
	return 0;
}
static int fts_resume(struct i2c_client *client)
{
	PRINT_INFO("fts_resume\n");
	return 0;
}

MODULE_DEVICE_TABLE(i2c, fts_id);

static const struct of_device_id focaltech_of_match[] = {
       { .compatible = "focaltech,focaltech_ts", },
       { }
};
MODULE_DEVICE_TABLE(of, focaltech_of_match);
static struct i2c_driver fts_driver = {
	.probe		= fts_probe,
	.remove		= fts_remove,
	.id_table	= fts_id,
	.driver	= {
		.name	= FOCALTECH_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = focaltech_of_match,
	},
	.suspend = fts_suspend,
	.resume = fts_resume,
};

static int __init fts_init(void)
{
	return i2c_add_driver(&fts_driver);
}

static void __exit fts_exit(void)
{
	i2c_del_driver(&fts_driver);
}

module_init(fts_init);
module_exit(fts_exit);

MODULE_AUTHOR("<mshl>");
MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL");
