/**
 * drivers/mfd/sadc_aux_interface.c
 *
 * aux1 aux2 channels voltage sample interface for Ingenic SoC
 *
 * Copyright(C)2012 Ingenic Semiconductor Co., LTD.
 * http://www.ingenic.cn
 * Aaron Wang <hfwang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/jz_adc.h>
#include <linux/linux_sensors.h>
#include <linux/input.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regulator/consumer.h>

#define MAG_MAX_POS             214748364       /** Max positive value */
#define MAG_MAX_NEG             214748364       /** Max negative value */

struct hwmon {
	struct platform_device *pdev;
	struct resource *mem;
	struct completion read_completion;
	struct mutex lock;
	struct input_dev	*idev;
	struct delayed_work input_hwork;
	struct device *hwmon_dev;  // Hardware device structure
	const struct mfd_cell *cell;

	void __iomem *base;
	int irq;
	unsigned int poll_interval_volt;
	atomic_t enabled;
	atomic_t heart_enabled;
	unsigned int voltage;
};

static struct regulator *neurosky_power_vio = NULL;
static atomic_t neurosky_powered = ATOMIC_INIT(0);

struct hwmon *hwmon = NULL;

static int hwmon_enable(struct hwmon *hwmon, int sensor);
static int hwmon_disable(struct hwmon *hwmon, int sensor);

static irqreturn_t jz_hwmon_irq_handler(int irq, void *devid)
{
	struct hwmon *hwmon = (struct hwmon *)devid;
	complete(&hwmon->read_completion);
	return IRQ_HANDLED;
}

static unsigned int hwmon_read_value(struct hwmon *hwmon, uint8_t config)
{
	unsigned long tmp;
	unsigned int value;

	mutex_lock(&hwmon->lock);
	INIT_COMPLETION(hwmon->read_completion);
	jz_adc_set_config(hwmon->pdev->dev.parent, ADCFG_CMD_MASK, config);
	hwmon->cell->enable(hwmon->pdev);
	enable_irq(hwmon->irq);

	tmp = wait_for_completion_interruptible_timeout(&hwmon->read_completion, HZ);
	if (tmp > 0) {
		value = readw(hwmon->base) & 0xfff;
	} else {
		value = tmp ? tmp : -ETIMEDOUT;
	}

	disable_irq(hwmon->irq);
	hwmon->cell->disable(hwmon->pdev);
	mutex_unlock(&hwmon->lock);

	return value;
}

int hwmon_sample_volt(enum aux_chan chan)
{
	unsigned int value = 0, voltage = 0;
	uint8_t config = 0;
	if (!hwmon) {
		printk("hwmon is null ! return\n");
		return -EINVAL;
	}

	switch (chan) {
		case SADC_AUX1:
			config = ADCFG_CMD_AUX(1);
			break;
		case SADC_AUX2:
			config = ADCFG_CMD_AUX(2);
			break;
		default:
			break;
	}

	value = hwmon_read_value(hwmon, config);
	voltage = value * 3300 / 4096;

	return voltage;
}
EXPORT_SYMBOL(hwmon_sample_volt);

static int hwmon_open(struct inode *inode, struct file *file)
{
        return 0;
}

static int hwmon_release(struct inode *inode, struct file *filp)
{
        return 0;
}

#define	HWMONIO	0x4d
#define	HWMON_IOCTL_GET_VOLT_AUX1		_IOR(HWMONIO, 0x11, int)
#define	HWMON_IOCTL_GET_VOLT_AUX2		_IOR(HWMONIO, 0x12, int)
#define SENSOR_TYPE_HWMON                       30
static struct proc_dir_entry *proc_hwmon_dir;
static struct linux_sensor_t hwmon_hw = {
	                  "hwmon heart-beat sensor",
	                   "Ingenic",
			  SENSOR_TYPE_HWMON,0,16384,1, 1, { }
};

static long hwmon_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        int ret = 0;
	void __user *argp = (void __user *)arg;
	switch (cmd) {
	case HWMON_IOCTL_GET_VOLT_AUX1:
	case HWMON_IOCTL_GET_VOLT_AUX2:
	        if (cmd == HWMON_IOCTL_GET_VOLT_AUX1)
		    ret = hwmon_sample_volt(SADC_AUX1);
		else
		    ret = hwmon_sample_volt(SADC_AUX2);
		if (ret == -EINVAL)
		    return -EINVAL;
		if (copy_to_user(argp, &ret, sizeof(ret))) {
		    dev_err(&hwmon->pdev->dev, "copy_to_user error!!!\n");
		    return -EFAULT;
		}
		break;
	case SENSOR_IOCTL_GET_DATA:
	    if (copy_to_user(argp, &hwmon_hw, sizeof(hwmon_hw))) {
		dev_err(&hwmon->pdev->dev, "copy_to_user error!!!\n");
		return -EINVAL;
	    }
	    break;
	default:
	        dev_err(&hwmon->pdev->dev, "invalid command: 0x%08x\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static ssize_t hwmon_show_volt(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret=0;
	ret = hwmon_sample_volt(SADC_AUX1);
	ret = sprintf(buf, "%d\n", ret);

	return ret;
}

static ssize_t hwmon_show_enable_heart(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret;
	struct hwmon *hwmon = dev_get_drvdata(dev);
	ret = sprintf(buf, "%d\n", atomic_read(&hwmon->heart_enabled));

	return ret;
}

static ssize_t hwmon_store_enable_heart(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	unsigned long enable;
	struct hwmon *hwmon = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable) {
		ret = hwmon_enable(hwmon, 1);
	} else {
		ret = hwmon_disable(hwmon, 1);
	}

	return size;
}

static ssize_t hwmon_show_heart_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = hwmon->poll_interval_volt;
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t hwmon_store_heart_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long interval_ms;
	struct hwmon *hwmon = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	mutex_lock(&hwmon->lock);
	hwmon->poll_interval_volt = (unsigned int)interval_ms;
	mutex_unlock(&hwmon->lock);
	return size;
}


static SENSOR_DEVICE_ATTR(voltage, S_IRUGO,  hwmon_show_volt, NULL, 0);


static SENSOR_DEVICE_ATTR(enable_heart, S_IRUGO | S_IWUGO | S_IXUGO,
		hwmon_show_enable_heart, hwmon_store_enable_heart, 0);


static SENSOR_DEVICE_ATTR(poll_period_ms_heart, S_IRUGO | S_IWUGO | S_IXUGO,
		hwmon_show_heart_polling_rate, hwmon_store_heart_polling_rate, 0);

static struct attribute *hwmon_attributes[] = {

	&sensor_dev_attr_voltage.dev_attr.attr,
	&sensor_dev_attr_enable_heart.dev_attr.attr,
	&sensor_dev_attr_poll_period_ms_heart.dev_attr.attr,
	NULL
};

static const struct attribute_group hwmon_attribute_group = {
	.attrs = hwmon_attributes,
};


static void hwmon_input_hwork_func(struct work_struct *work)
{
	int ret = 0;
	struct hwmon *hwmon = container_of((struct delayed_work *)work,
			struct hwmon, input_hwork);

	ret = hwmon_sample_volt(SADC_AUX1);
//	printk("jim gao  hwmon_input_hwork_func-------------------%d---\n",ret);
	input_report_abs(hwmon->idev, ABS_GAS, ret);
	input_sync(hwmon->idev);

	schedule_delayed_work(&hwmon->input_hwork,
			msecs_to_jiffies(hwmon->poll_interval_volt));
}


static void hwmon_device_power_off(struct hwmon *hwmon)
{
	int err;
	printk(" hwmon_device_power_off==================\n");
	if (atomic_read(&neurosky_powered)) {
		if (!IS_ERR(neurosky_power_vio)) {
			if(err =regulator_disable(neurosky_power_vio)){
				printk("disable_power failed res=%d\n",err);
			}
		}
		atomic_set(&neurosky_powered, 0);
	}

	atomic_set(&hwmon->enabled, 0);
}

static int hwmon_device_power_on(struct hwmon *hwmon)
{
	int err;
	printk(" hwmon_device_power_on==================\n");
	if (!atomic_read(&neurosky_powered)) {
		if (!IS_ERR(neurosky_power_vio)) {

			if(err =regulator_enable(neurosky_power_vio)){
				printk("evg_enable_power  regulator_enable failed\n");
				return -1;
			}
		}
		atomic_set(&neurosky_powered, 1);
	}

	atomic_set(&hwmon->enabled, 1);

	return 0;
}

static int hwmon_enable(struct hwmon *hwmon, int sensor)
{
	int err;


	if (!atomic_cmpxchg(&hwmon->heart_enabled, 0, 1)) {
		err = hwmon_device_power_on(hwmon);
		if (err < 0) {
			atomic_set(&hwmon->heart_enabled, 0);
			return err;
		}
		schedule_delayed_work(&hwmon->input_hwork,
							  msecs_to_jiffies(hwmon->poll_interval_volt));
	}

	return 0;
}

static int hwmon_disable(struct hwmon *hwmon, int sensor)
{

	if (atomic_cmpxchg(&hwmon->heart_enabled, 1, 0)) {
		cancel_delayed_work_sync(&hwmon->input_hwork);
	}


	if (!atomic_read(&hwmon->heart_enabled)) {
		hwmon_device_power_off(hwmon);
	}

	return 0;
}

static struct file_operations hwmon_ops = {
	.owner     	=    THIS_MODULE,
	.open      	=    hwmon_open,
	.release   	=    hwmon_release,
	.unlocked_ioctl =    hwmon_ioctl,
};

static int __devexit hwmon_remove(struct platform_device *pdev)
{
	struct hwmon *hwmon = platform_get_drvdata(pdev);

	free_irq(hwmon->irq, hwmon);

	iounmap(hwmon->base);
	release_mem_region(hwmon->mem->start,resource_size(hwmon->mem));
	kfree(hwmon);

	return 0;
}

static int __devinit hwmon_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct proc_dir_entry *entry;

	pr_debug("=============hwmon_probe=============\n");
	hwmon = kzalloc(sizeof(*hwmon), GFP_KERNEL);

	if (!hwmon) {
		dev_err(&pdev->dev, "Failed to allocate driver structre\n");
		return -ENOMEM;
	}

	hwmon->cell = mfd_get_cell(pdev);
	if (!hwmon->cell) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get mfd cell for hwmon!\n");
		goto err_free;
	}

	hwmon->irq = platform_get_irq(pdev, 0);
	if (hwmon->irq < 0) {
		ret = hwmon->irq;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free;
	}

	hwmon->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!hwmon->mem) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform mmio resource\n");
		goto err_free;
	}

	hwmon->mem = request_mem_region(hwmon->mem->start,
			resource_size(hwmon->mem), pdev->name);
	if (!hwmon->mem) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to request mmio memory region\n");
		goto err_free;
	}

	hwmon->base = ioremap_nocache(hwmon->mem->start,resource_size(hwmon->mem));
	if (!hwmon->base) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		goto err_free;
	}

	hwmon->pdev = pdev;

	init_completion(&hwmon->read_completion);
	mutex_init(&hwmon->lock);

	ret = request_irq(hwmon->irq, jz_hwmon_irq_handler, 0, pdev->name, hwmon);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", ret);
		goto err_free;
	}

	disable_irq(hwmon->irq);


	platform_set_drvdata(pdev, hwmon);

	proc_hwmon_dir = proc_mkdir("hwmon", NULL);

	if (!proc_hwmon_dir) {
		dev_err(&pdev->dev, "Failed to mkdir hwmon\n");
		return -ENOMEM;
	}

	entry = proc_create("devices", S_IRUGO | S_IWUGO, proc_hwmon_dir,
			    &hwmon_ops);

	if (!entry)
		goto fail;

//jim
	if (!neurosky_power_vio) {
		neurosky_power_vio = regulator_get(NULL, "sensor3v3");
		if (IS_ERR(neurosky_power_vio)) {
			printk("%s -> get regulator VIO failed\n",__func__);
			neurosky_power_vio = NULL;
			return -ENODEV;
		}
	}

	INIT_DELAYED_WORK(&hwmon->input_hwork, hwmon_input_hwork_func);

	hwmon->idev = input_allocate_device();
	if (!hwmon->idev) {
		dev_err(&pdev->dev, "Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto  err_free;
	}

	hwmon->idev->name = "ADC_heart_rate";
	hwmon->idev->id.bustype = BUS_I2C;
	hwmon->idev->dev.parent = &pdev->dev;

	input_set_drvdata(hwmon->idev, hwmon);

	set_bit(EV_ABS, hwmon->idev->evbit);

	input_set_abs_params(hwmon->idev, ABS_GAS,
						 -MAG_MAX_NEG, MAG_MAX_POS, 0, 0);

	ret = input_register_device(hwmon->idev);
	if (ret) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", ret);
		goto out_reg;
	}

	hwmon->poll_interval_volt = 100;

//	ret = hwmon_enable(hwmon, 1);

	ret = sysfs_create_group(&pdev->dev.kobj, &hwmon_attribute_group);
	if (ret) {
		dev_dbg(&pdev->dev, "could not create sysfs files\n");
		goto fail;
	}

	hwmon->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(hwmon->hwmon_dev)) {
		dev_dbg(&pdev->dev, "unable to register hwmon device\n");
		ret = PTR_ERR(hwmon->hwmon_dev);
		goto fail;
	}

	return 0;


out_reg:
	input_free_device(hwmon->idev);


err_free :
	kfree(hwmon);
	return ret;
fail:
	remove_proc_entry("devices", proc_hwmon_dir);
	remove_proc_entry("hwmon", NULL);
	return -ENOMEM;
}

static struct platform_driver hwmon_driver = {
	.probe	= hwmon_probe,
	.driver = {
		.name	= "jz-hwmon",
		.owner	= THIS_MODULE,
	},
};

static int __init hwmon_init(void)
{
	platform_driver_register(&hwmon_driver);

	return 0;
}

static void __exit hwmon_exit(void)
{
	platform_driver_unregister(&hwmon_driver);
	remove_proc_entry("devices", proc_hwmon_dir);
	remove_proc_entry("hwmon", NULL);
}

device_initcall_sync(hwmon_init);
module_exit(hwmon_exit);

MODULE_ALIAS("platform:jz-hwmon");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aaron Wang <hfwang@ingenic.cn>");
MODULE_DESCRIPTION("JZ SoC aux sample driver");
