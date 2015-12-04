#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <soc/gpio.h>
#include <asm/atomic.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/input-polldev.h>
#include <linux/timer.h>
#include <linux/input/pac7673.h>
#include <linux/i2c/i2c_power_manager.h>

#define PAC7673_NAME "pac7673"
#define MAG_MIN_POS     0
#define MAG_MAX_POS     256

typedef struct {
    struct i2c_client *client;
    struct pac7673_platform_data *pdata;
    struct class *pac7673_class;
    struct device *pac7673_device;
    struct input_dev *input_dev;
    struct kobject *kobj;
    struct delayed_work pac7673_work;
    unsigned int major_id;
    int irq;
    atomic_t enabled;
    atomic_t period_ms;
    struct wake_lock irq_lock;
} pac7673_data_t;
static pac7673_data_t pac7673data;

static struct i2c_power_device *device = NULL;
static struct i2c_control_operations func;

static int pac7673_i2c_write(unsigned char reg, unsigned char * data, int len) {
    unsigned char buf[20];
    int rc;
    int ret = 0;
    int i;

    buf[0] = reg;
    if (len >= 20) {
        printk("%s (%d) : FAILED: buffer size is limitted(20) %d\n", __func__,
                __LINE__, len);
        return -1;
    }

    for (i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }

    rc = i2c_master_send(pac7673data.client, buf, len + 1);

    if (rc != len + 1) {
        printk("%s (%d) : FAILED: writing to reg 0x%x\n", __func__, __LINE__,
                reg);
        ret = -1;
    }

    return ret;
}

static int pac7673_i2c_read(unsigned char reg, unsigned char * data) {
    unsigned char buf[20];
    int rc;

    buf[0] = reg;

    rc = i2c_master_send(pac7673data.client, buf, 1);
    if (rc != 1) {
        printk("%s (%d) : FAILED: writing to address 0x%x\n", __func__,
                __LINE__, reg);
        return -1;
    }

    rc = i2c_master_recv(pac7673data.client, buf, 1);
    if (rc != 1) {
        printk("%s (%d) : FAILED: reading data\n", __func__, __LINE__);
        return -1;
    }

    *data = buf[0];
    return 0;
}

unsigned char pac7673_write_reg(unsigned char addr, unsigned char data) {
    int ret = pac7673_i2c_write(addr, &data, 1);

    if (ret != 0)
        return false;
    else
        return true;
}

unsigned char pac7673_read_reg(unsigned char addr, unsigned char *data) {
    int ret = pac7673_i2c_read(addr, data);

    if (ret != 0)
        return false;
    else
        return true;
}

static int pac7673_enable(int enable) {
    unsigned char i = 0;
    unsigned char reg_value = 0;
    unsigned char ret = 0;

    if (enable) {
        for (i = 0; i < 10; i++) {
            ret = pac7673_read_reg(0x00, &reg_value);
            if (reg_value == 0x63)
                break;
            msleep(10);
            return 0;
        }
    } else
        pac7673_write_reg(0x03, 0x20);

    return 1;
}

void pac7673_enable_ps(int enable) //enable operation
{
    unsigned char reg_value = 0;

    pac7673_read_reg(0x03, &reg_value);
    if (enable) {
        pac7673_write_reg(0x03, reg_value | 0x01);
    } else {
        pac7673_write_reg(0x03, reg_value & 0xFE);
    }
}

void pac7673ee_init(void) {
    msleep(1);
    pac7673_enable(1);
    msleep(1);
    pac7673_write_reg(0x03, 0x0d);

    pac7673_write_reg(0x04, 0x10); //als intergration time, set idle state step to 1.66ms
    pac7673_write_reg(0x05, 0xC8); //PS LED Pulse On Time = [7:0] x 2us
    pac7673_write_reg(0x06, 0x2f); //PS  intergration time 16*1.66
    pac7673_write_reg(0x07, 0x2a); //idle time 	60-16 =42
    pac7673_write_reg(0x0c, 0x1E); //PS high threshold
    pac7673_write_reg(0x0d, 0x32); //PS low threshould
    pac7673_write_reg(0x11, 0x0c); //Disable PS INT
    pac7673_enable_ps(1);
}

static int pac7673_open(struct inode *inode, struct file *filp) {
    dev_info(&pac7673data.client->dev, "%s\n", __FUNCTION__);
    return 0;
}

static int pac7673_release(struct inode *inode, struct file *filp) {
    dev_info(&pac7673data.client->dev, "%s\n", __FUNCTION__);
    return 0;
}

static struct file_operations pac7673_fops = {
        .owner = THIS_MODULE,
        .open = pac7673_open,
        .release = pac7673_release,
};

static irqreturn_t pac7673_irq_func(int irq, void *data) {
    disable_irq_nosync(pac7673data.irq);
    wake_lock(&pac7673data.irq_lock);
    schedule_delayed_work(&pac7673data.pac7673_work, msecs_to_jiffies(atomic_read(&pac7673data.period_ms)));

    return IRQ_HANDLED;
}

void pac7673_work_func(struct work_struct *work) {
    u8 val = 0;
    u8 int_status = 0;

    pac7673_read_reg(0x10, &val);
    printk(KERN_DEBUG "====pac7673 report val is %d=====\n",val);

    input_report_abs(pac7673data.input_dev, ABS_GAS, val);
    input_sync(pac7673data.input_dev);

    pac7673_read_reg(0x09, &int_status);
    pac7673_write_reg(0x09, int_status & 0xDB);
    enable_irq(pac7673data.irq);
    wake_unlock(&pac7673data.irq_lock);
}

static ssize_t pac7673_show_enable(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", atomic_read(&pac7673data.enabled));

    return ret;
}

static ssize_t pac7673_store_enable(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    unsigned long enable;

    if (strict_strtoul(buf, 10, &enable))
        return -EINVAL;

    if (atomic_read(&pac7673data.enabled) == !!enable) {
        return size;
    }

    atomic_set(&pac7673data.enabled, !!enable);

    if (!!enable) {
        enable_irq(pac7673data.irq);
        pac7673_write_reg(0x11, 0x8c);
    } else {
        pac7673_write_reg(0x11, 0x0c);
        disable_irq(pac7673data.irq);
    }

    return size;
}

static ssize_t pac7673_show_polling_rate(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", atomic_read(&pac7673data.period_ms));

    return ret;
}

static ssize_t pac7673_store_polling_rate(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    unsigned long interval_ms;

    if (strict_strtoul(buf, 10, &interval_ms))
        return -EINVAL;

    if (interval_ms < 10)
        interval_ms = 10;

    if (interval_ms > 500)
        interval_ms = 500;

    atomic_set(&pac7673data.period_ms, interval_ms);

    return size;
}

static SENSOR_DEVICE_ATTR(enable_proximity, S_IRUGO | S_IWUGO,
        pac7673_show_enable, pac7673_store_enable, 0);

static SENSOR_DEVICE_ATTR(poll_period_ms_proximity, S_IRUGO | S_IWUGO,
        pac7673_show_polling_rate, pac7673_store_polling_rate, 0);

static struct attribute *pac7673_attributes[] = {
        &sensor_dev_attr_enable_proximity.dev_attr.attr,
        &sensor_dev_attr_poll_period_ms_proximity.dev_attr.attr,
        NULL
};

static const struct attribute_group pac7673_attribute_group = {
        .attrs = pac7673_attributes,
};

static void pac7673_power_on(struct pac7673_platform_data *pdata)
{
    if (pdata->power_on)
        pdata->power_on();
}

static void pac7673_power_off(struct pac7673_platform_data *pdata)
{
    if (pdata->power_off)
        pdata->power_off();
}

static void pac7673_init_chip(void)
{
    jzgpio_ctrl_pull(GPIO_PORT_C, 1, 22);
    pac7673ee_init();
}

static int pac7673_i2c_probe(struct i2c_client *client,
        const struct i2c_device_id *id) {
    struct pac7673_platform_data *pdata = NULL;
    struct input_dev *input_dev = NULL;
    int err = 0;
    int i = 0;
    u8 read_reg = 0;

    pdata = (struct pac7673_platform_data *) client->dev.platform_data;
    pac7673data.client = client;
    pac7673data.pdata = pdata;

    if (pdata->board_init) {
        err = pdata->board_init(&client->dev);
        if (err < 0) {
            pr_err("board_init failed ! errno = %d\n", err);
            return err;
        }
    }

    func.power_on = pac7673_power_on;
    func.power_off = pac7673_power_off;
    func.init_chip = pac7673_init_chip;

    device = register_i2c_power_device(i2c_adapter_id(client->adapter), &func, pdata);
    if (!device) {
        err = -1;
        goto board_exit;
    }

    i2c_power_device_on(device);
    msleep(2);

    for (i = 0; i < 5; i++) {
        err = pac7673_read_reg(0x0, &read_reg);

        if (i == 4 && err == 0) {
            printk("pac7673 error: Can not read register 0x0\n");
            err = -1;
            goto failed_to_read_reg;
        }
    }

    err = gpio_request(pdata->gpio_int, "pac7673_gpio_int");
    if (err < 0) {
        printk("pac7673: Unable to request INT GPIO %d\n", pdata->gpio_int);
        goto failed_request_gpio;
    }
    gpio_direction_input(pdata->gpio_int);
    jzgpio_ctrl_pull(pdata->gpio_int / 32, 1, pdata->gpio_int % 32);

    pac7673data.irq = gpio_to_irq(pdata->gpio_int);
    if (pac7673data.irq > 0) {
        err = request_any_context_irq(pac7673data.irq, pac7673_irq_func,
                IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "pac7673_irq",
                &pac7673data);

        if (err) {
            printk(KERN_ERR "Failed to request irq: %d\n", err);
            goto failed_request_gpio;
        } else {
            enable_irq_wake(pac7673data.irq);
            disable_irq(pac7673data.irq);
        }
    }

    err = register_chrdev(0, PAC7673_NAME, &pac7673_fops);
    if (err < 0) {
        printk(KERN_WARNING "pac7673 : Can't get major\n");
    } else {
        pac7673data.major_id = err;
        printk("pac7673 : Success to register character device %d\n", pac7673data.major_id);
        pac7673data.pac7673_class = class_create(THIS_MODULE, PAC7673_NAME);

        if (IS_ERR(pac7673data.pac7673_class)) {
            printk(KERN_ERR "class_create() failed for ofn_class\n");
            err = -1;
            goto free_chrdev;
        }

        pac7673data.pac7673_device = device_create(pac7673data.pac7673_class,
                NULL,MKDEV(pac7673data.major_id, 0), NULL,
                PAC7673_NAME);
    }

    pac7673data.kobj = kobject_create_and_add("pac7673", kernel_kobj);
    if (!pac7673data.kobj) {
        goto failed_kobj_create;
    }

    err = sysfs_create_group(pac7673data.kobj, &pac7673_attribute_group);
    if (err) {
        goto failed_sysfs_create_group;
    }

    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto failed_input_dev_alloc;
    }

    pac7673data.input_dev = input_dev;
    set_bit(EV_ABS, pac7673data.input_dev->evbit);
    input_set_abs_params(pac7673data.input_dev, ABS_GAS,
                         -MAG_MIN_POS, MAG_MAX_POS, 0, 0);
    pac7673data.input_dev->name = PAC7673_NAME;

    err = input_register_device(pac7673data.input_dev);
    if (err) {
        printk(KERN_ERR "pac7673_probe: failed to register input devices\n");
        goto failed_input_register_device;
    }

    atomic_set(&pac7673data.enabled, 0);
    atomic_set(&pac7673data.period_ms, 80);

    INIT_DELAYED_WORK(&pac7673data.pac7673_work, pac7673_work_func);
    wake_lock_init(&pac7673data.irq_lock, WAKE_LOCK_SUSPEND, "pac7673_irq");

    printk("pac7673_i2c_probe success\n");

    return 0;

failed_request_gpio:
    input_unregister_device(pac7673data.input_dev);
failed_input_register_device:
    input_free_device(pac7673data.input_dev);
failed_input_dev_alloc:
    sysfs_remove_group(pac7673data.kobj, &pac7673_attribute_group);
failed_sysfs_create_group:
    kobject_put(pac7673data.kobj);
failed_kobj_create:
    device_destroy(pac7673data.pac7673_class, MKDEV(pac7673data.major_id, 0));
    class_destroy(pac7673data.pac7673_class);
free_chrdev:
    unregister_chrdev(pac7673data.major_id, PAC7673_NAME);
failed_to_read_reg:
    i2c_power_device_off(device);
    unregister_i2c_power_device(device);
board_exit:
    pdata->board_exit(&client->dev);

    return err;
}

static int pac7673_i2c_remove(struct i2c_client *client) {
    return 0;
}

static int pac7673_suspend(struct device *dev)
{
    return 0;
}

static int pac7673_resume(struct device *dev)
{
    return 0;
}

static const struct i2c_device_id pac7673_device_id[] =
        { { "pac7673", 0 }, { } };

static const struct dev_pm_ops pac7673_pm_ops = {
        .suspend = pac7673_suspend,
        .resume = pac7673_resume,
};

static struct i2c_driver pac7673_i2c_driver = {
        .driver = {
                .name = "pac7673",
                .owner = THIS_MODULE,
                .pm = &pac7673_pm_ops,
        },
        .probe = pac7673_i2c_probe,
        .remove = pac7673_i2c_remove,
        .id_table = pac7673_device_id,
};

static int __init pac7673_init(void)
{
    int retval;

    retval = i2c_add_driver(&pac7673_i2c_driver);
    if (retval) {
        printk("*****pac7673 add i2c driver failed *****\n");
    }

    return retval;
}

static void __exit pac7673_exit(void)
{
    device_destroy(pac7673data.pac7673_class, MKDEV(pac7673data.major_id, 0)); //delete device node under /dev
    class_destroy(pac7673data.pac7673_class);//delete class created by us
    unregister_chrdev(pac7673data.major_id, PAC7673_NAME);
    i2c_del_driver(&pac7673_i2c_driver);
}

module_init( pac7673_init);
module_exit( pac7673_exit);
MODULE_DESCRIPTION("pac7673 driver");
MODULE_LICENSE("GPL");
