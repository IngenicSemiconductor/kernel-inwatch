/*
 *  mma955xl-spi.c - Linux kernel modules for 3-Axis Smart Orientation/ Motion Sensor
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
#include <linux/spi/spi.h>

#include "mma955xl.h"
#define MMA955XL_SPI_WRITE   	0x80
#define MMA955XL_SPI_READ   	0x00
#define MMA955XL_SPI_REG(reg) 	((reg) << 1)
static int mma955xl_spi_write(struct mma955xl_data *pdata, u8* buf, int len)
{
	int ret = 0;
	struct spi_device * spi = (struct spi_device *)pdata->bus_priv;
	buf[0] = (MMA955XL_SPI_WRITE|MMA955XL_SPI_REG(buf[0]));
	ret = spi_write(spi,buf,len);
	if(ret < 0){
		printk(KERN_ERR "%s ,write data error\n",__FUNCTION__);
		return -EINVAL;
	}
	return len;

}
static int mma955xl_spi_read (struct mma955xl_data *pdata, u8* buf, int len)
{
	int ret = 0;
	u8 tmp_tx;
	struct spi_device * spi = (struct spi_device *)pdata->bus_priv;
	tmp_tx = (MMA955XL_SPI_READ |MMA955XL_SPI_REG(0));
	ret = spi_write_then_read(spi,&tmp_tx,1,buf,len);
	if(ret < 0){
			printk(KERN_ERR "%s ,read data error  ret %d\n",__FUNCTION__,ret);
			return -EINVAL;
	}
	return len;

}

static int __devinit mma955xl_spi_probe(struct spi_device *spi)
{
	int ret;
	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if(ret)
		return ret;
	mma955xl_dev.bus_priv = spi;
	mma955xl_dev.bus_type = BUS_SPI;
	mma955xl_dev.read  = mma955xl_spi_read;
	mma955xl_dev.write = mma955xl_spi_write;
	spi_set_drvdata(spi, &mma955xl_dev);
	return mma955xl_device_init(&mma955xl_dev);
}
static int __devexit mma955xl_spi_remove(struct spi_device *client)
{
	return mma955xl_device_remove(&mma955xl_dev);
}

#ifdef CONFIG_PM
static int mma955xl_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	return mma955xl_suspend(spi_get_drvdata(spi));
}

static int mma955xl_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	return mma955xl_resume(spi_get_drvdata(spi));
}

#else
#define mma955xl_spi_suspend	NULL
#define mma955xl_spi_resume	NULL
#endif

static SIMPLE_DEV_PM_OPS(mma955xl_pm_ops, mma955xl_spi_suspend, mma955xl_spi_resume);

static struct spi_driver mma955xl_spi_driver = {
	.driver	 = {
		.name   = "mma955xl",
		.owner  = THIS_MODULE,
		.pm = &mma955xl_pm_ops,
	},
	.probe   = mma955xl_spi_probe,
	.remove  = __devexit_p(mma955xl_spi_remove),
};

static int __init mma955xl_spi_init(void)
{
	/* register driver */
	int res;
	res = spi_register_driver(&mma955xl_spi_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma955xl spi driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit mma955xl_spi_exit(void)
{
	spi_unregister_driver(&mma955xl_spi_driver);
}

module_init(mma955xl_spi_init);
module_exit(mma955xl_spi_exit);

MODULE_AUTHOR("Johnson Sun <johnson.sun@freescale.com>");
MODULE_DESCRIPTION("mma955xl 3-Axis Smart Orientation and Motion Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

