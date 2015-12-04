/*
 *  Copyright (C) 2015 Wu Jiao <jiao.wu@ingenic.com wujiaososo@qq.com>
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
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/completion.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/jz_adc.h>

#include "fifo_ring.h"

#define SENSOR_VDD_REGULATOR "sensor3v3"
#define DEV_NAME "jz-hwmon"

#ifndef ADCFG_CMD_AUX1
#define ADCFG_CMD_AUX1 ADCFG_CMD_AUX(1)
#endif

#ifdef CONFIG_MFD_JZ_SADC_HEART_RATE_AUX1
#error "MFD_JZ_SADC_HEART_RATE_AUX1 and CONFIG_MFD_JZ_SADC_AUX1_HRTIMER can not select both of them"
#endif

#undef assert
#define assert(cond)											\
	do {														\
		if (!(cond)) {											\
			pr_err("#####assert condition failed (%s)#####\n", #cond);	\
			BUG_ON(1);											\
		}														\
	} while (0)

#define MAG_MAX_POS             214748364       /** Max positive value */
#define MAG_MAX_NEG             214748364       /** Max negative value */

#define HRADC_FIFO_SIZE         100

enum {
	REPORT_BY_NONE = 0,
	REPORT_BY_DWORK = 1 << 0,
	REPORT_BY_SYSFS = 1 << 1,
	REPORT_BOTH = (REPORT_BY_DWORK | REPORT_BY_SYSFS),
};

struct hradc {
	struct platform_device *pdev;
	const struct mfd_cell *cell;
	int irq;
	struct resource *mem;
	void __iomem *base;

	struct hrtimer timer;
	struct regulator *reg_vdd;
	struct input_dev	*idev;
	struct delayed_work input_work;
	struct completion read_completion;
	struct mutex mlock;
	spinlock_t slock;

	int irq_enabled;
	int data_report_mode;

	unsigned int enable;
	unsigned int period;		/* in ms */
	ktime_t last_expires;

	unsigned long fifo_data_dwork[HRADC_FIFO_SIZE];
	struct fifo_ring fifo_dwork;

	unsigned long fifo_data_sysfs[HRADC_FIFO_SIZE];
	struct fifo_ring fifo_sysfs;

	unsigned int is_enabled_before_suspend;
};

struct hradc *mhradc = NULL;

extern void jz_adc_fast_disable(struct device *dev);
extern void jz_adc_fast_enable(struct device *dev, unsigned int aux_config);

static inline void hradc_enable_irq(void) {
	if (!mhradc->irq_enabled) {
		enable_irq(mhradc->irq);
		mhradc->irq_enabled = 1;
	}
}

static inline void hradc_disable_irq(void) {
	if (mhradc->irq_enabled) {
		disable_irq_nosync(mhradc->irq);
		mhradc->irq_enabled = 0;
	}
}

static inline void hradc_start_sample(void) {
	jz_adc_fast_enable(mhradc->pdev->dev.parent, ADCFG_CMD_AUX1);
	hradc_enable_irq();
}

static inline void hradc_stop_sample(void) {
	jz_adc_fast_disable(mhradc->pdev->dev.parent);
	hradc_disable_irq();
}

static inline void hradc_set_dwork_voltage(unsigned int voltage) {
	fifo_ring_add(&mhradc->fifo_dwork, voltage);
}

static inline void hradc_set_sysfs_voltage(unsigned int voltage) {
	fifo_ring_add(&mhradc->fifo_sysfs, voltage);
}

static inline void set_expires(struct hrtimer *timer, ktime_t *last_expires, unsigned long long period_ms) {
	ktime_t period = ns_to_ktime(period_ms * 1000 * 1000);
	*last_expires = ktime_add(*last_expires, period);
	hrtimer_set_expires(timer, *last_expires);
}

static enum hrtimer_restart hradc_hrtimer_callback(struct hrtimer *timer) {
	set_expires(&mhradc->timer, &mhradc->last_expires, mhradc->period);
	if (!mhradc->enable)
		return HRTIMER_NORESTART;
	hradc_start_sample();
	return HRTIMER_RESTART;		/* we want restart after callback is return */
}

static void hradc_start_timer(void) {
	mhradc->last_expires = ktime_get();
	set_expires(&mhradc->timer, &mhradc->last_expires, mhradc->period);
	hrtimer_restart(&mhradc->timer);
}

static void hradc_stop_timer(void) {
	hrtimer_cancel(&mhradc->timer);
}

static inline void hradc_set_power(unsigned int enable) {
	if (enable && mhradc->reg_vdd) {
		regulator_enable(mhradc->reg_vdd);
	} else if (!enable && mhradc->reg_vdd) {
		regulator_disable(mhradc->reg_vdd);
	}
}

static void hradc_enable(void) {
	fifo_ring_clear(&mhradc->fifo_dwork);
	fifo_ring_clear(&mhradc->fifo_sysfs);
	hradc_set_power(1);
	hradc_start_timer();
}

static void hradc_disable(void) {
	hradc_set_power(0);
	hradc_stop_timer();
	fifo_ring_clear(&mhradc->fifo_dwork);
	fifo_ring_clear(&mhradc->fifo_sysfs);
}

static irqreturn_t hradc_irq_handler(int irq, void *devid)
{
	struct hradc *hradc = (struct hradc *)devid;
	unsigned int voltage = readw(hradc->base);

	hradc_stop_sample();

	if (hradc->data_report_mode & REPORT_BY_DWORK) {
		hradc_set_dwork_voltage(voltage);
		schedule_delayed_work(&hradc->input_work, 0);
	}

	if (hradc->data_report_mode & REPORT_BY_SYSFS) {
		hradc_set_sysfs_voltage(voltage);
		if (!completion_done(&hradc->read_completion))
			complete(&hradc->read_completion);
	}

	return IRQ_HANDLED;
}

void hradc_input_work(struct work_struct *work) {
	unsigned long flags;
	struct fifo_ring tmp;
	unsigned long voltage;
	unsigned long data[HRADC_FIFO_SIZE];

	fifo_ring_init(&tmp, data, ARRAY_SIZE(data));
	mutex_lock(&mhradc->mlock);

	spin_lock_irqsave(&mhradc->slock, flags);
	fifo_ring_move(&tmp, &mhradc->fifo_dwork);
	spin_unlock_irqrestore(&mhradc->slock, flags);

	while (fifo_ring_get(&tmp, &voltage)) {
		mhradc->idev->absinfo[ABS_GAS].value = voltage - 1;
		input_report_abs(mhradc->idev, ABS_GAS, voltage);
		input_sync(mhradc->idev);
	}
	mutex_unlock(&mhradc->mlock);
}

static ssize_t report_mode_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int n = 0;
	const char *str;

	switch (mhradc->data_report_mode) {
	case REPORT_BY_DWORK: str = "dwork";break;
	case REPORT_BOTH: str = "both";break;
	case REPORT_BY_SYSFS: str = "sysfs";break;
	case REPORT_BY_NONE: str = "none";break;
	default: str = "";break;
	}
	n = sprintf(buf, "%s", str) + 1;

	return n;
}

static ssize_t report_mode_w(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int mode = -1;
	unsigned long flags;

	if (!strncmp(buf, "sysfs", strlen("sysfs"))) {
		mode = REPORT_BY_SYSFS;
	} else if (!strncmp(buf, "dwork", strlen("dwork"))) {
		mode = REPORT_BY_DWORK;
	} else if (!strncmp(buf, "both", strlen("both"))) {
		mode = REPORT_BOTH;
	} if (!strncmp(buf, "none", strlen("none"))) {
		mode = REPORT_BY_NONE;
	}

	if (mode != -1) {
		spin_lock_irqsave(&mhradc->slock, flags);
		mhradc->data_report_mode = mode;
		spin_unlock_irqrestore(&mhradc->slock, flags);
	}

	return count;
}

static ssize_t enable_adc_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int n = 0;

	n = sprintf(buf, "%d", mhradc->enable) + 1;

	return n;
}

static ssize_t enable_adc_w(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int enable = -1;
	unsigned long flags;

	sscanf(buf, "%d", &enable);

	if (enable != -1) {
		enable = !!enable;
		if ((mhradc->enable && enable) || (!mhradc->enable && !enable))
			return count;
		spin_lock_irqsave(&mhradc->slock, flags);
		mhradc->enable = enable;
		spin_unlock_irqrestore(&mhradc->slock, flags);
		if (enable)
			hradc_enable();
		else
			hradc_disable();
	}

	return count;
}

static ssize_t sample_period_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int n = 0;

	n = sprintf(buf, "%d", mhradc->period) + 1;

	return n;
}

static ssize_t sample_period_w(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int period = -1;
	unsigned long flags;

	sscanf(buf, "%d", &period);

	if (period != -1) {
		spin_lock_irqsave(&mhradc->slock, flags);
		mhradc->period = period;
		spin_unlock_irqrestore(&mhradc->slock, flags);
	}

	return count;
}

static ssize_t adc_data_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int n = 0;
	unsigned long flags;
	unsigned long voltage;
	unsigned int is_empty;

	if (!mhradc->enable) {
		pr_err("heart rate: is not enabled\n");
		return 0;
	}

	spin_lock_irqsave(&mhradc->slock, flags);
	is_empty = fifo_ring_is_empty(&mhradc->fifo_sysfs);
	spin_unlock_irqrestore(&mhradc->slock, flags);

	if (is_empty)
		wait_for_completion_interruptible_timeout(&mhradc->read_completion, HZ);

	spin_lock_irqsave(&mhradc->slock, flags);
	while (fifo_ring_get(&mhradc->fifo_sysfs, &voltage)) {
		n += sprintf(buf + n, "%ld\n", voltage);
		if (n >= (PAGE_SIZE - 100)) {
			break;
		}
	}
	spin_unlock_irqrestore(&mhradc->slock, flags);

	return n;
}

static ssize_t adc_data_w(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(report_mode, S_IRUGO|S_IWUGO, report_mode_r, report_mode_w);
static DEVICE_ATTR(enable_adc, S_IRUGO|S_IWUGO, enable_adc_r, enable_adc_w);
static DEVICE_ATTR(adc_data, S_IRUGO|S_IWUGO, adc_data_r, adc_data_w);
static DEVICE_ATTR(sample_period, S_IRUGO|S_IWUGO, sample_period_r, sample_period_w);

static struct attribute *hradc_attrs[] = {
	&dev_attr_report_mode.attr,
	&dev_attr_sample_period.attr,
	&dev_attr_adc_data.attr,
	&dev_attr_enable_adc.attr,
	NULL,
};

static struct attribute_group hradc_attrs_group = {
	.name	= "config",
	.attrs	= hradc_attrs,
};

static int hradc_probe(struct platform_device *pdev) {
	int ret;

	assert(!mhradc);

	mhradc = kzalloc(sizeof(*mhradc), GFP_KERNEL);
	assert(mhradc);

	mhradc->pdev = pdev;
	mhradc->data_report_mode = REPORT_BY_NONE;
	mhradc->period = 40;
	mhradc->enable = 0;
	fifo_ring_init(&mhradc->fifo_dwork, mhradc->fifo_data_dwork, ARRAY_SIZE(mhradc->fifo_data_dwork));
	fifo_ring_init(&mhradc->fifo_sysfs, mhradc->fifo_data_sysfs, ARRAY_SIZE(mhradc->fifo_data_sysfs));

	platform_set_drvdata(pdev, mhradc);

	mhradc->cell = mfd_get_cell(pdev);
	assert(mhradc->cell);

	mhradc->irq = platform_get_irq(pdev, 0);
	assert(mhradc->irq > 0);

	mhradc->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	assert(mhradc->mem);

	mhradc->mem = request_mem_region(mhradc->mem->start, resource_size(mhradc->mem), pdev->name);
	assert(mhradc->mem);

	mhradc->base = ioremap_nocache(mhradc->mem->start, resource_size(mhradc->mem));
	assert(mhradc->base);

	hrtimer_init(&mhradc->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	mhradc->timer.function = hradc_hrtimer_callback;
	INIT_DELAYED_WORK(&mhradc->input_work, hradc_input_work);
	init_completion(&mhradc->read_completion);
	mutex_init(&mhradc->mlock);
	spin_lock_init(&mhradc->slock);

	ret = request_irq(mhradc->irq, hradc_irq_handler, IRQF_DISABLED, pdev->name, mhradc);
	assert(!ret);
	disable_irq(mhradc->irq);
	mhradc->irq_enabled = 0;

	mhradc->reg_vdd = regulator_get(&pdev->dev, SENSOR_VDD_REGULATOR);
	if (IS_ERR(mhradc->reg_vdd)) {
		mhradc->reg_vdd = NULL;
		pr_err("hradc: failed to get regulator: [%s]\n", "evg_vdd");
	}

	mhradc->idev = input_allocate_device();
	assert(mhradc->idev);

	mhradc->idev->name = "hrtimer-adc-aux1";
	mhradc->idev->id.bustype = BUS_I2C;
	mhradc->idev->dev.parent = &pdev->dev;

	set_bit(EV_ABS, mhradc->idev->evbit);

	input_set_abs_params(mhradc->idev, ABS_GAS,
						 -MAG_MAX_NEG, MAG_MAX_POS, 0, 0);

	ret = input_register_device(mhradc->idev);
	assert(!ret);

	ret = sysfs_create_group(&pdev->dev.kobj, &hradc_attrs_group);
	assert(!ret);

	return 0;
}

static int hradc_remove(struct platform_device *pdev) {
	hradc_disable();
	sysfs_remove_group(&mhradc->pdev->dev.kobj, &hradc_attrs_group);
	free_irq(mhradc->irq, mhradc);
	if (mhradc->reg_vdd)
		regulator_put(mhradc->reg_vdd);
	input_unregister_device(mhradc->idev);
	input_free_device(mhradc->idev);
	iounmap(mhradc->base);
	release_mem_region(mhradc->mem->start, resource_size(mhradc->mem));
	kfree(mhradc);
	mhradc = NULL;

	return 0;
}

int hradc_suspend(struct platform_device *pdev, pm_message_t state) {
	mhradc->is_enabled_before_suspend = mhradc->enable;
	if (mhradc->enable) {
		hradc_disable();
	}

	return 0;
}

int hradc_resume(struct platform_device *pdev) {
	if (mhradc->is_enabled_before_suspend) {
		hradc_enable();
	}

	return 0;
}

void hradc_shutdown(struct platform_device *pdev) {

}

static struct platform_driver hradc_driver = {
	.probe = hradc_probe,
	.remove  = hradc_remove,
	.driver.name = DEV_NAME,
	.suspend = hradc_suspend,
	.resume = hradc_resume,
	.shutdown = hradc_shutdown,
};

int __init hradc_init(void) {
	int ret;

	ret = platform_driver_register(&hradc_driver);
	if (ret) {
		pr_err("hradc: hradc driver register failed\n");
		ret = -EINVAL;
		goto error_platform_driver_register_failed;
	}

	return 0;
error_platform_driver_register_failed:
	;/* ToDo : write your error deal code here */
return ret;
}
device_initcall_sync(hradc_init);

void __exit hradc_exit(void) {
	platform_driver_unregister(&hradc_driver);
}
module_exit(hradc_exit);
