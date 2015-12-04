/*
 *  Copyright (C) 2014 Fighter Sun <wanmyqawdr@126.com>
 *  Copyright (C) 2014 Wu Jiao <jwu@ingenic.cn>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/slpt.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/slpt_battery.h>
#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/slpt_cache.h>
#include <linux/rtc.h>

#include "fifo_ring.h"

static struct fifo_ring data_fifo;

/* 先进后出 */
int slpt_adc_get_data_stack(unsigned int *voltage, unsigned int *time) {
	struct time_vol_pair data;
	int ret = fifo_ring_get_reverse(&data_fifo, &data);

	if (ret) {
		*voltage = data.voltage;
		*time = data.time;
	}

	return ret;
}
EXPORT_SYMBOL(slpt_adc_get_data_stack);

/* 先进先出 */
int slpt_adc_get_data_reverse(unsigned int *voltage, unsigned int *time) {
	struct time_vol_pair data;
	int ret = fifo_ring_get(&data_fifo, &data);

	if (ret) {
		*voltage = data.voltage;
		*time = data.time;
	}

	return ret;
}
EXPORT_SYMBOL(slpt_adc_get_data_reverse);

void slpt_adc_add_data(unsigned int time, unsigned int voltage) {
	struct time_vol_pair data = {.time = time, .voltage = voltage};
	fifo_ring_add(&data_fifo, data);
}
EXPORT_SYMBOL(slpt_adc_add_data);

void *slpt_adc_get_fifo_ring(void) {
	return (void *)KSEG1ADDR((unsigned long)&data_fifo);
}
EXPORT_SYMBOL(slpt_adc_get_fifo_ring);

static ssize_t dump_data_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct time_vol_pair data;
	struct rtc_time tm;
	struct fifo_ring tmp;
	
	tmp = data_fifo;
	while (fifo_ring_get_reverse(&data_fifo, &data)) {
		rtc_time_to_tm(data.time, &tm);
		if (data.voltage & (1 << 31)) { /* it means pmu regs */
			pr_err("%04u-%02u-%02u %02u:%02u:%02u : 0x%02x 0x%02x\n", tm.tm_year, tm.tm_mon, tm.tm_mday,
				   tm.tm_hour, tm.tm_min, tm.tm_sec,
				   ((data.voltage >> 8) & 0xff), (data.voltage & 0xff));
		} else {
			pr_err("%04u-%02u-%02u %02u:%02u:%02u : %u mV\n", tm.tm_year, tm.tm_mon, tm.tm_mday,
				   tm.tm_hour, tm.tm_min, tm.tm_sec, data.voltage);
		}
	}
	data_fifo = tmp;

	return 0;
}

static ssize_t dump_data_w(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(dump_data, S_IRUGO|S_IWUGO, dump_data_r, dump_data_w);

static struct attribute *slpt_adc_test_attrs[] = {
	&dev_attr_dump_data.attr,
	NULL,
};

static struct attribute_group slpt_adc_test_attr_group = {
	.name	= "debug",
	.attrs	= slpt_adc_test_attrs,
};

static volatile struct time_vol_pair data_array[2];

static int slpt_adc_test_probe(struct platform_device *pdev) {
	int ret;
	int size = 2;
	
	struct time_vol_pair *data = kmalloc(sizeof(struct time_vol_pair) * size, GFP_KERNEL);
	BUG_ON(data == NULL);

	fifo_ring_init(&data_fifo, (void *)KSEG1ADDR((unsigned int )data_array), 2);

	ret = sysfs_create_group(&pdev->dev.kobj, &slpt_adc_test_attr_group);
	BUG_ON(ret != 0);

	return 0;
}

static int slpt_adc_test_remove(struct platform_device *pdev) {
	return 0;
}

int slpt_adc_test_suspend(struct platform_device *pdev, pm_message_t state) {
	return 0;
}

int slpt_adc_test_resume(struct platform_device *pdev) {
	return 0;
}

void slpt_adc_test_shutdown(struct platform_device *pdev) {
	
}

static struct platform_driver slpt_adc_test_driver = {
	.probe = slpt_adc_test_probe,
	.remove  = slpt_adc_test_remove,
	.driver.name = "slpt_adc_test",
	.suspend = slpt_adc_test_suspend,
	.resume = slpt_adc_test_resume,
	.shutdown = slpt_adc_test_shutdown,
};

static struct platform_device slpt_adc_test_device = {
	.name = "slpt_adc_test",
};

int __init slpt_adc_test_init(void) {
	int ret;
	
	ret = platform_driver_register(&slpt_adc_test_driver);
	if (ret) {
		pr_err("slpt_adc_test: slpt_adc_test driver register failed\n");
		ret = -EINVAL;
		goto error_platform_driver_register_failed;
	}
	
	ret = platform_device_register(&slpt_adc_test_device);
	if (ret) {
		pr_err("slpt_adc_test: slpt_adc_test device register failed\n");
		ret = -EINVAL;
		goto error_platform_device_register_failed;
	}
	
	return 0;
error_platform_driver_register_failed:
	;/* ToDo : write your error deal code here */
error_platform_device_register_failed:
	platform_driver_unregister(&slpt_adc_test_driver);
return ret;
}
module_init(slpt_adc_test_init);

void __exit slpt_adc_test_exit(void) {
	platform_device_unregister(&slpt_adc_test_device);
	platform_driver_unregister(&slpt_adc_test_driver);
}
module_exit(slpt_adc_test_exit);



