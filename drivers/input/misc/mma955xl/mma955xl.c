/*
 *  mma955xl.c - Linux kernel modules for 3-Axis Smart Orientation/ Motion Sensor
 *  Version     : 01.00
 *  Time            : Dec.26, 2012
 *  Author      : rick zhang <rick.zhang@freescale.com>
 *
 *  Copyright (C) 2012 Freescale Semiconductor.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/i2c/mma955xl_pedometer.h>

#include "mma955xl.h"

struct mma955xl_data mma955xl_dev;
EXPORT_SYMBOL_GPL(mma955xl_dev);

/*cmd contains command and high bits of  offset*/
static int mma955xl_send_command(struct mma955xl_data *pdata, int appid,
        int cmd, int offset, char *data, int size) {
    unsigned char tx_buf[32] = { 0x00 };
    static unsigned char rx_buf[1024] = { 0x00 };
    int ret = -1;
    int retval = -1;
    int retry = 50;
    struct mma955xl_cmd_h_t * pcmd = (struct mma955xl_cmd_h_t *) tx_buf;
    struct mma955xl_rep_h_t * presp;
    int len = sizeof(struct mma955xl_cmd_h_t);
    memset(tx_buf, 0, sizeof(tx_buf));
    pcmd->rsvd = 0;
    pcmd->appId = appid;
    pcmd->cmd.val = cmd;
    pcmd->offset = offset;
    pcmd->count = size;
    if (pcmd->cmd.bits.cmd != MMA955XL_FCI_CONFIG_R
            && pcmd->cmd.bits.cmd != MMA955XL_FCI_DATA_R) // Config write Cmd.
    {
        if (size < 20) {
            memcpy(pcmd->data, data, size);
            len += size;
        } else {
            printk("mma9550::invalid size of array \r\n");
        }
    }
    pdata->write(pdata, (u8*) pcmd, len);
    do {
        msleep(2);
        retval = -1;
        if (ret <= 0) {
            memset(rx_buf, 0, sizeof(rx_buf));
            presp = (struct mma955xl_rep_h_t *) rx_buf;
            if (pcmd->cmd.bits.cmd == MMA955XL_FCI_CONFIG_R
                    || pcmd->cmd.bits.cmd == MMA955XL_FCI_DATA_R) {
                ret = pdata->read(pdata, (u8 *) presp,
                        size + sizeof(struct mma955xl_rep_h_t));
            } else {
                ret = pdata->read(pdata, (u8 *) presp, 4);
            }
            if (presp->cc == 0x01 && presp->error == 0x00
                    && presp->bytes_read == presp->bytes_rqst) {
                if (pcmd->cmd.bits.cmd == MMA955XL_FCI_CONFIG_R
                        || pcmd->cmd.bits.cmd == MMA955XL_FCI_DATA_R) {
                    memcpy(data, presp->data, size);
                }
                retval = 0;
                break;
            }
        }
        retry--;
    } while (retry > 0);
    return retval;
}
static int mma955xl_write(struct mma955xl_data *pdata, u8 *mb_frame,
        int buf_len) {
    int appid, cmd, offset, size;
    u8 *data;
    if (!pdata || !mb_frame)
        return -EINVAL;
    appid = mb_frame[0];
    cmd = mb_frame[1];
    offset = mb_frame[2];
    size = mb_frame[3];
    data = &mb_frame[4];
    if (buf_len < (sizeof(struct mma955xl_cmd_h_t) + size)) /*the frame buffer less*/
    {
        printk("ma955xl:error1\n");
        return -EINVAL;
    }
    return mma955xl_send_command(pdata, appid, cmd, offset, data, size);
}
static int mma955xl_read(struct mma955xl_data *pdata, u8 *mb_frame, int buf_len) {
    int appid, cmd, offset, size;
    u8 *data;
    if (!pdata || !mb_frame)
        return -EINVAL;
    appid = mb_frame[0];
    cmd = mb_frame[1];
    offset = mb_frame[2];
    size = mb_frame[3];
    data = &mb_frame[4];
    if (buf_len < (sizeof(struct mma955xl_cmd_h_t) + size)) /*the frame buffer less*/
        return -EINVAL;
    return mma955xl_send_command(pdata, appid, cmd, offset, data, size);
}
static int mma955xl_set_wsmode(struct mma955xl_data *pdata, int mode) {
    u8 wake_frame[32] = { 0x12, 0x20, 0x06, 0x01, 0x00 };
    int ret;
    if (mode == MMA955XL_MODE_WAKE) {
        ret = mma955xl_write(pdata, wake_frame, sizeof(wake_frame));
        if (ret < 0) {
            printk(KERN_ERR "mma955xl  write error when init device wake\n");
            return ret;
        }
        printk(KERN_INFO "mma955xl_set_wsmode set awake\n");
        pdata->wsmode = MMA955XL_MODE_WAKE;

    } else {
        wake_frame[4] = 0x01;
        ret = mma955xl_write(pdata, wake_frame, sizeof(wake_frame));
        if (ret < 0) {
            printk(KERN_ERR "mma955xl  write error when init device sleep\n");
            return ret;
        }
        printk(KERN_INFO "mma955xl_set_wsmode set sleep\n");
        pdata->wsmode = MMA955XL_MODE_SLEEP;
    }
    return 0;
}
/*set normal  mode*/
static int mma955xl_set_nr(struct mma955xl_data *pdata) {
    int ret;
    u8 nr_frame_set[32] = { 0x18, 0x20, 0x00, 0x01, 0x80 };
    u8 afe_frame[32] = { 0x06, 0x20, 0x00, 0x0A, 0x02, 0x10, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00 };
    ret = mma955xl_write(pdata, nr_frame_set, sizeof(nr_frame_set));
    if (ret < 0)
        return ret;
    ret = mma955xl_write(pdata, afe_frame, sizeof(afe_frame));
    if (ret < 0)
        return ret;

    return 0;
}

static int mma955xl_pedometer_conf(struct mma955xl_data *pdata) {
    int ret;
    u8 pedo_conf[32] = { 0x15, 0x20, 0x00, 0x10, 0x0C, 0xE0, 0x13, 0x20, 0x00,
            0x5A, 0xE0, 0x64, 0xB4, 0x4E, 0x01, 0x83, 0x05, 0x01, 0x00, 0x00 };
    ret = mma955xl_write(pdata, pedo_conf, sizeof(pedo_conf));
    if (ret < 0)
        return ret;
    return 0;
}
static int mma955xl_pedometer_status(struct mma955xl_data *pdata,
        struct mma955xl_pedometer_status *pstatus) {
    int ret;
    u8 pedo_status[32] = { 0x15, 0x30, 0x00, 0x0C };
    u8* p;
    ret = mma955xl_read(pdata, pedo_status, sizeof(pedo_status));
    if (ret < 0)
        return ret;
    p = MMA955XL_DATA_OF_RESPONSE(pedo_status);
    pstatus->status = ((p[0] << 8) & 0xff00) | p[1];
    pstatus->step_count = ((p[2] << 8) & 0xff00) | p[3];
    pstatus->distance = ((p[4] << 8) & 0xff00) | p[5];
    pstatus->speed = ((p[6] << 8) & 0xff00) | p[7];
    pstatus->calories = ((p[8] << 8) & 0xff00) | p[9];
    pstatus->sleep_count = ((p[10] << 8) & 0xff00) | p[11];
    return 0;
}
static int mma955xl_get_version(struct mma955xl_data *pdata) {
    u8 msg[32] = { 0x00, 0x00, 0x00, 0x0c };
    int ret = 0;
    u8 *v = NULL;
    struct mma955xl_rep_h_t *presp = (struct mma955xl_rep_h_t *) msg;
    ret = pdata->write(pdata, msg, 5);
    if (ret < 0) {
        printk("mma955xl_get_version:write failed ! ret=%d\n !", ret);
        return -1;
    }

    memset(msg, 0, 32);
    msleep(10);
    ret = pdata->read(pdata, msg, 16);
    if (ret < 0) {
        printk("mma955xl_get_version:read failed !");
        return -1;
    }

    v = MMA955XL_DATA_OF_RESPONSE(msg);
    printk("mma955xl_get_version:presp->cc = %d \n", presp->cc);
    if (presp->cc == 0x01) {
        printk(KERN_INFO "mma955xl device ID [0x%x%x%x%x]\n", v[0], v[1], v[2],
                v[3]);
        printk(KERN_INFO "mma955xl ROM[%x.%x],FW[%x.%x]\n", v[4], v[5], v[6],
                v[7]);
        printk(KERN_INFO "mma955xl HW[%x.%x],FB[%x,%x]\n", v[8], v[9], v[10],
                v[11]);
    }
    return 0;
}
static int mma955xl_chip_init(struct mma955xl_data *pdata) {
    mma955xl_set_wsmode(pdata, MMA955XL_MODE_SLEEP);
    mma955xl_set_nr(pdata);
    mma955xl_pedometer_conf(pdata);
    return 0;
}

static void mma955xl_report_data(struct mma955xl_data* pdata) {
    int ret = -1;
    struct input_polled_dev * poll_dev = pdata->poll_dev;
    struct mma955xl_pedometer_status status;
    if (pdata->wsmode == MMA955XL_MODE_WAKE) {

        ret = mma955xl_pedometer_status(pdata, &status);
        if (!ret) {
            input_report_abs(poll_dev->input, ABS_RX, status.step_count);
            input_sync(poll_dev->input);
        }

    }
}

static void mma955xl_dev_poll(struct input_polled_dev *dev) {
    struct mma955xl_data* pdata = (struct mma955xl_data*) dev->private;
    mma955xl_report_data(pdata);
}

static ssize_t mma955xl_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf) {
    struct input_polled_dev *poll_dev = mma955xl_dev.poll_dev;
    struct mma955xl_data *pdata = (struct mma955xl_data *) (poll_dev->private);
    int enable = 0;
    mutex_lock(&pdata->data_lock);
    enable = (pdata->wsmode == MMA955XL_MODE_WAKE ? 1 : 0);
    mutex_unlock(&pdata->data_lock);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t mma955xl_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count) {
    struct input_polled_dev *poll_dev = mma955xl_dev.poll_dev;
    struct mma955xl_data *pdata = (struct mma955xl_data *) (poll_dev->private);

    unsigned long enable;
    int ret;

    mutex_lock(&pdata->data_lock);
    enable = simple_strtoul(buf, NULL, 10);
    enable = (enable > 0) ? 1 : 0;

    if (enable && pdata->wsmode == MMA955XL_MODE_SLEEP) {
        ret = mma955xl_set_wsmode(pdata, MMA955XL_MODE_WAKE);
        if (ret < 0) {
            printk("Failed to set wake mode\n");
            return ret;
        }
    } else if (enable == 0 && pdata->wsmode == MMA955XL_MODE_WAKE) {
        ret = mma955xl_set_wsmode(pdata, MMA955XL_MODE_SLEEP);
        if (ret < 0) {
            printk("Failed to set sleep mode\n");
            return ret;
        }
    }
    mutex_unlock(&pdata->data_lock);
    return count;
}

static ssize_t mma955xl_position_show(struct device *dev,
        struct device_attribute *attr, char *buf) {
    struct input_polled_dev *poll_dev = mma955xl_dev.poll_dev;
    struct mma955xl_data *pdata = (struct mma955xl_data *) (poll_dev->private);
    int position = 0;
    mutex_lock(&pdata->data_lock);
    position = pdata->position;
    mutex_unlock(&pdata->data_lock);
    return sprintf(buf, "%d\n", position);
}

static ssize_t mma955xl_position_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count) {
    struct input_polled_dev *poll_dev = mma955xl_dev.poll_dev;
    struct mma955xl_data *pdata = (struct mma955xl_data *) (poll_dev->private);
    int position;
    position = simple_strtoul(buf, NULL, 10);
    mutex_lock(&pdata->data_lock);
    pdata->position = position;
    mutex_unlock(&pdata->data_lock);
    return count;
}

static ssize_t mma955xl_poll_interval_show(struct device *dev,
        struct device_attribute *attr, char *buf) {
    struct input_polled_dev *poll_dev = mma955xl_dev.poll_dev;
    return sprintf(buf, "%d\n", poll_dev->poll_interval);
}

static ssize_t mma955xl_poll_interval_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count) {
    struct input_polled_dev *poll_dev = mma955xl_dev.poll_dev;
    struct mma955xl_data *pdata = (struct mma955xl_data *) (poll_dev->private);
    unsigned int interval = simple_strtoul(buf, NULL, 10);
    mutex_lock(&pdata->data_lock);
    poll_dev->poll_interval = interval;
    mutex_unlock(&pdata->data_lock);
    return count;
}

static SENSOR_DEVICE_ATTR(enable_pedometer, S_IRUGO | S_IWUGO,
        mma955xl_enable_show, mma955xl_enable_store,0);
static SENSOR_DEVICE_ATTR(position, S_IRUGO | S_IWUGO,
        mma955xl_position_show, mma955xl_position_store,0);
static SENSOR_DEVICE_ATTR(poll_interval_pedometer, S_IRUGO | S_IWUGO,
        mma955xl_poll_interval_show, mma955xl_poll_interval_store,0);

static struct attribute *mma955xl_attributes[] = {
        &sensor_dev_attr_enable_pedometer.dev_attr.attr,
        &sensor_dev_attr_position.dev_attr.attr,
        &sensor_dev_attr_poll_interval_pedometer.dev_attr.attr, NULL };

static const struct attribute_group mma955xl_attr_group = { .attrs =
        mma955xl_attributes, };
int mma955xl_suspend(struct mma955xl_data *pdata) {
    if (pdata->wsmode == MMA955XL_MODE_WAKE)
        mma955xl_set_wsmode(pdata, MMA955XL_MODE_SLEEP);
    return 0;
}
EXPORT_SYMBOL_GPL(mma955xl_suspend);

int mma955xl_resume(struct mma955xl_data *pdata) {
    if (pdata->wsmode == MMA955XL_MODE_SLEEP)
        mma955xl_set_wsmode(pdata, MMA955XL_MODE_WAKE);
    return 0;
}
EXPORT_SYMBOL_GPL(mma955xl_resume);

int mma955xl_device_init(struct mma955xl_data *pdata) {
    int result = -1;
    struct input_dev *idev;
    struct input_polled_dev *poll_dev;
    struct mma955xl_platform_data *p = NULL;
    struct i2c_client *pbus = (struct i2c_client *) pdata->bus_priv;

    if (!pdata)
        goto err_out;
    p = (struct mma955xl_platform_data *) dev_get_platdata(&pbus->dev);

    result = gpio_request(p->mcu_wakeup_ap, "mcu_wakeup_ap");
    if (result < 0) {
        printk("Failed to request gpio for mcu_wakeup_ap: %d\n", result);
        goto err_out;
    }
    result = gpio_request(p->ap_wakeup_mcu, "ap_wakeup_mcu");
    if (result < 0) {
        printk("Failed to request gpio for ap_wakeup_mcu: %d\n", result);
        goto err_gpio_0;
    }
    gpio_request(p->sensor_int0_n, "sensor_int0_n");
    if (result < 0) {
        printk("Failed to request gpio for sensor_int0_n: %d\n", result);
        goto err_gpio_1;
    }
    result = gpio_direction_input(p->mcu_wakeup_ap);
    if (result < 0) {
        printk("Failed to set mcu_wakeup_ap input \n");
        goto err_gpio_2;
    }
    result = gpio_direction_input(p->sensor_int0_n);
    if (result < 0) {
        printk("Failed to set sensor_int0_n input \n");
        goto err_gpio_2;
    }
    result = gpio_direction_output(p->ap_wakeup_mcu, 0);
    if (result < 0) {
        printk("Failed to set ap_wakeup_mcu output:0 \n");
        goto err_gpio_2;
    }

    msleep(500);
    result = gpio_direction_output(p->ap_wakeup_mcu, 1);
    if (result < 0) {
        printk("Failed to set ap_wakeup_mcu output:1 \n");
        goto err_gpio_2;
    }

    msleep(100);
    result = mma955xl_get_version(pdata);
    if (result < 0) {
        printk("mma955xl_get_version failed \n");
        goto err_gpio_2;
    }

    pdata->mode = MODE_2G;
    pdata->position = 0;
    pdata->wsmode = MMA955XL_MODE_SLEEP;
    mutex_init(&pdata->data_lock);

    poll_dev = input_allocate_polled_device();
    if (!poll_dev) {
        result = -ENOMEM;
        printk(KERN_ERR "mma955xl alloc poll device failed!\n");
        goto err_alloc_poll_device;
    }
    poll_dev->poll = mma955xl_dev_poll;
    poll_dev->poll_interval = POLL_INTERVAL;
    poll_dev->poll_interval_min = POLL_INTERVAL_MIN;
    poll_dev->poll_interval_max = POLL_INTERVAL_MAX;
    poll_dev->private = pdata;
    idev = poll_dev->input;
    idev->name = "FreescaleAccelerometer";
    idev->uniq = "mma955xl";
    idev->id.bustype = pdata->bus_type;
    idev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_REL);
    input_set_abs_params(idev, ABS_X, -0x7fff, 0x7fff, 0, 0);
    input_set_abs_params(idev, ABS_Y, -0x7fff, 0x7fff, 0, 0);
    input_set_abs_params(idev, ABS_Z, -0x7fff, 0x7fff, 0, 0);
    input_set_abs_params(idev, ABS_RX, 0, 0xffff, 0, 0); // for pedometer step count
    input_set_abs_params(idev, ABS_RY, 0, 0xffff, 0, 0); // for pedometer
    pdata->poll_dev = poll_dev;
    result = input_register_polled_device(pdata->poll_dev);
    if (result) {
        printk(KERN_ERR "mma955xl register poll device failed!\n");
        goto err_register_polled_device;
    }

    pdata->kobj = kobject_create_and_add("mma955xl", kernel_kobj);
    if (!pdata->kobj) {
        printk("kobject_create_and_add failed !\n");
        goto failed_kobj_create;
    }

    result = sysfs_create_group(pdata->kobj, &mma955xl_attr_group);
    if (result) {
        printk(KERN_ERR "mma955xl create device file failed!\n");
        result = -EINVAL;
        goto err_create_sysfs;
    }

    result = mma955xl_chip_init(pdata);
    if (result < 0) {
        printk(KERN_ERR "mma955xl sensor init error!\n");
        goto error_init;
    }
    printk("mma955xl device driver init successfully\n");
    return 0;

    error_init: sysfs_remove_group(&idev->dev.kobj, &mma955xl_attr_group);
    err_create_sysfs: kobject_put(pdata->kobj);
    failed_kobj_create: input_unregister_polled_device(pdata->poll_dev);
    err_register_polled_device: input_free_polled_device(poll_dev);
    err_alloc_poll_device: kfree(pdata);
    err_gpio_2: gpio_free(p->sensor_int0_n);
    err_gpio_1: gpio_free(p->ap_wakeup_mcu);
    err_gpio_0: gpio_free(p->mcu_wakeup_ap);
    err_out: return result;
}
EXPORT_SYMBOL_GPL(mma955xl_device_init);

int mma955xl_device_remove(struct mma955xl_data *pdata) {
    struct input_polled_dev *poll_dev = pdata->poll_dev;
    if (pdata) {
        input_unregister_polled_device(poll_dev);
        input_free_polled_device(poll_dev);
        kfree(pdata);
    }
    return 0;
}
EXPORT_SYMBOL_GPL(mma955xl_device_remove);

MODULE_DESCRIPTION("Freescale three-axis digital accelerometer smart driver");
MODULE_AUTHOR("Rick");
MODULE_LICENSE("GPL");

