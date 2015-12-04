/*
 *  mma955xl-i2c.c - Linux kernel modules for 3-Axis Smart Orientation/ Motion Sensor
 *  Version		: 01.00
 *  Time			: Dec.26, 2012
 *  Author		: rick zhang <rick.zhang@freescale.com>
 *
 *  Copyright (C) 2010-2011 Freescale Semiconductor.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include "mma955xl.h"

static int mma955xl_i2c_write(struct mma955xl_data *pdata, u8* buf, int len){
	struct i2c_client * client = (struct i2c_client *)pdata->bus_priv;
	return i2c_master_send(client,buf,len);
}

static int mma955xl_i2c_read (struct mma955xl_data *pdata, u8* buf, int len){
	struct i2c_client * client = (struct i2c_client *)pdata->bus_priv;
	return i2c_master_recv(client,buf,len);
}

static int  mma955xl_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	mma955xl_dev.bus_priv = client;
	mma955xl_dev.bus_type = BUS_I2C;
	mma955xl_dev.read  = mma955xl_i2c_read;
	mma955xl_dev.write = mma955xl_i2c_write;
	i2c_set_clientdata(client,&mma955xl_dev);
	return mma955xl_device_init(&mma955xl_dev);
}

static int  mma955xl_i2c_remove(struct i2c_client *client)
{
	return mma955xl_device_remove(&mma955xl_dev);
}

#ifdef CONFIG_PM
static int mma955xl_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return mma955xl_suspend(i2c_get_clientdata(client));
}

static int mma955xl_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return mma955xl_resume(i2c_get_clientdata(client));
}

#else
#define mma955xl_i2c_suspend	NULL
#define mma955xl_i2c_resume		NULL

#endif

static const struct i2c_device_id mma955xl_i2c_id[] = {
	{"mma955xl", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mma955xl_i2c_id);

static SIMPLE_DEV_PM_OPS(mma955xl_pm_ops, mma955xl_i2c_suspend, mma955xl_i2c_resume);

static struct i2c_driver mma955xl_i2c_driver = {
	.driver = {
		   .name = "mma955xl",
		   .owner = THIS_MODULE,
		   .pm = &mma955xl_pm_ops,
		   },
	.probe = mma955xl_i2c_probe,
	.remove = mma955xl_i2c_remove,
	.id_table = mma955xl_i2c_id,
};

static int __init mma955xl_i2c_init(void)
{
	/* register driver */
	int res;
	res = i2c_add_driver(&mma955xl_i2c_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma955xl i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit mma955xl_i2c_exit(void)
{
	i2c_del_driver(&mma955xl_i2c_driver);
}

module_init(mma955xl_i2c_init);
module_exit(mma955xl_i2c_exit);

MODULE_AUTHOR("Johnson Sun <johnson.sun@freescale.com>");
MODULE_DESCRIPTION("mma955xl 3-Axis Smart Orientation and Motion Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

