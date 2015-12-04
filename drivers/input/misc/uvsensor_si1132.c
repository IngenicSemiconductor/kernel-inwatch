/**
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
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
#include <linux/hwmon.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uv_sensor.h>
#include "uvsensor_si1132.h"

#define SI1132_DRV_NAME		"si1132"
#define SI1132_ID		0x32

struct si1132_data {
	struct input_dev            *input_dev;
	struct i2c_client *client;
	struct uvsensor_platform_data *pdata;
	struct mutex lock;
	atomic_t enabled;
	unsigned int                 delay_ms;
	unsigned long                delay_jiffies;
	struct delayed_work          work_data;

	struct kobject *si1132_kobj;
};

static int si1132_i2c_read(struct si1132_data *ssi,u8* reg,u8 *buf, int len)
{
	int err;
	struct i2c_msg msgs[] = {
			{
					.addr = ssi->client->addr,
					.flags =  0,
					.len = 1,
					.buf = reg,
			},
			{
					.addr = ssi->client->addr,
					.flags = I2C_M_RD,
					.len = len,
					.buf = buf,
			},
	};

	err = i2c_transfer(ssi->client->adapter, msgs, 2);
	if (err < 0) {
		dev_err(&ssi->client->dev, "Read msg error\n");
		dev_err(&ssi->client->dev, "Read msg error, err = %d\n", err);
		dev_err(&ssi->client->dev, "Read msg error, ssi->client->addr = 0x%x\n", ssi->client->addr);
	}
	return  0;
}

static int si1132_i2c_write(struct si1132_data *ssi,u8 *buf, int len)
{
	int err;
	struct i2c_msg msgs[] = {
			{
					.addr = ssi->client->addr,
					.flags = 0,
					.len = len,
					.buf = buf,
			},
	};

	err = i2c_transfer(ssi->client->adapter, msgs, 1);
	if (err < 0) {
		dev_err(&ssi->client->dev, "Write msg errors\n");
	}
	return  0;
}

static int si1132_i2c_read_data(struct si1132_data *ssi,u8 reg,u8* rxData, int length)
{
	int ret;
	ret = si1132_i2c_read(ssi,&reg, rxData, length);
	if (ret < 0)
		return ret;
	else
		return 0;
}

static int si1132_i2c_write_data(struct si1132_data *ssi,u8 reg,char *txData, int length)
{
	char buf[80];
	int ret;

	buf[0] = reg;
	memcpy(buf+1, txData, length);
	ret = si1132_i2c_write(ssi,buf, length+1);
	if (ret < 0)
		return ret;
	else
		return 0;
}

s16 si1132_write_register(struct si1132_data *ssi, u8 address, u8 data)
{
	return si1132_i2c_write_data(ssi, address, &data, 1);
}

u8 si1132_read_register(struct si1132_data *ssi, u8 address)
{
	u8 data;
	si1132_i2c_read_data(ssi, address, &data, 1);
	return data;
}

s16 si1132_block_write(struct si1132_data *ssi,
		u8 address, u8 length, u8 *values)
{
	return si1132_i2c_write_data(ssi, address, values, length);
}

s16 si1132_block_read(struct si1132_data *ssi,
		u8 address, u8 length, u8 *values)
{
	return si1132_i2c_read_data(ssi, address, values, length);
}

static s16 wait_until_sleep(struct si1132_data *ssi)
{
	s16 retval;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	do {
		msleep(50);
		retval = si1132_read_register(ssi, REG_CHIP_STAT);
		if (retval == 1)
			break;
		if (retval <  0)
			return retval;
	} while (time_before(jiffies, timeout));

	if (jiffies >= timeout) {
		dev_info(&ssi->client->dev, "%s time out\n", __func__);
	}
	return 0;
}

static s16 send_cmd(struct si1132_data *ssi, u8 command)
{
	s16  response;
	s16  retval;
	unsigned long timeout;

	if ((response=si1132_read_register(ssi, REG_RESPONSE)) < 0)
		return response;

	timeout = jiffies + msecs_to_jiffies(1000);
	do {
		if ((retval = wait_until_sleep(ssi)) != 0)
			return retval;

		if (command == 0)
			break;

		msleep(50);
		retval = si1132_read_register(ssi, REG_RESPONSE);
		if (retval == response)
			break;
		else if (retval < 0)
			return retval;
		else response = retval;
	} while (time_before(jiffies, timeout));

	if (jiffies >= timeout) {
		dev_info(&ssi->client->dev, "%s time out REG_RESPONSE\n", __func__);
	}

	/* Send the Command */
	if ((retval = si1132_write_register(ssi, REG_COMMAND, command)) != 0)
		return retval;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		if(command == 0)
			break;

		msleep(50);
		retval = si1132_read_register(ssi, REG_RESPONSE);

		if (retval != response)
			break;
		else if (retval < 0)
			return retval;
	} while (time_before(jiffies, timeout));

	if (jiffies >= timeout) {
		dev_info(&ssi->client->dev, "%s time out \n", __func__);
	}
	return 0;
}

static s16 si1132_nop(struct si1132_data *ssi)
{
	return send_cmd(ssi, 0x00);
}

static s16 si1132_als_force(struct si1132_data *ssi)
{
	return send_cmd(ssi, 0x06);
}

static s16 si1132_als_auto(struct si1132_data *ssi)
{
	return send_cmd(ssi, 0x0E);
}

static s16 si1132_param_set(struct si1132_data *ssi, u8 address, u8 data)
{
	s16     retval;
	u8      buffer[2];
	unsigned long timeout;

	if ((retval = wait_until_sleep(ssi)) != 0)
		return retval;
	buffer[0]= data;
	buffer[1]= 0xA0 + (address & 0x1F);
	retval=si1132_block_write(ssi, REG_PARAM_WR, 2, (u8 *)buffer);
	if (retval != 0)
		return retval;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		msleep(50);
		retval = si1132_read_register(ssi, REG_PARAM_RD);
		if (retval == data)
			break;
	} while (time_before(jiffies, timeout));

	if (jiffies >= timeout) {
		dev_info(&ssi->client->dev, "%s time out\n", __func__);
	}
	return 0;
}

static s16 ps_als_pause (struct si1132_data *ssi)
{
	return send_cmd(ssi, 0x0B);
}

s16 si1132_pause_all(struct si1132_data *ssi)
{
	u8 count = 10;
	unsigned long timeout2;
	do {
		timeout2 = jiffies + msecs_to_jiffies(1000);
		do {
			msleep(50);
			if ((si1132_read_register(ssi, REG_RESPONSE))==0)
				break;
			else
				si1132_nop(ssi);
		} while (time_before(jiffies, timeout2));

		if (jiffies >= timeout2) {
			dev_info(&ssi->client->dev, "%s time out\n", __func__);
		}

		ps_als_pause(ssi);

		timeout2 = jiffies + msecs_to_jiffies(500);
		do {
			msleep(50);
			if ((si1132_read_register(ssi, REG_RESPONSE))!=0)
				break;
		} while (time_before(jiffies, timeout2));

		if (jiffies >= timeout2) {
			dev_info(&ssi->client->dev, "%s time out\n", __func__);
		}

		msleep(50);
		if ((si1132_read_register(ssi, REG_RESPONSE)) == 1)
			break;
		count--;
	} while (count);

	if (count == 0) {
		dev_info(&ssi->client->dev, "%s time out\n", __func__);
	}
	return 0;
}

static s16 si1132_reset(struct si1132_data *ssi)
{
	s32 retval = 0;

	retval += si1132_write_register(ssi, REG_MEAS_RATE, 0x00);
	retval += si1132_write_register(ssi, REG_ALS_RATE, 0x0);
	retval += si1132_pause_all(ssi);

	retval += si1132_write_register(ssi, REG_MEAS_RATE, 0x00);
	retval += si1132_write_register(ssi, REG_IRQ_ENABLE, 0x00);
	retval += si1132_write_register(ssi, REG_IRQ_MODE1, 0x00);
	retval += si1132_write_register(ssi, REG_IRQ_MODE2, 0x00);
	retval += si1132_write_register(ssi, REG_INT_CFG, 0x00);
	retval += si1132_write_register(ssi, REG_IRQ_STATUS, 0xFF);

	retval += si1132_write_register(ssi, REG_COMMAND, 1);

	/* This delay is needed to allow the Si114x to perform internal reset sequence. */
	mdelay(10);

	retval += si1132_write_register(ssi, REG_HW_KEY, HW_KEY_VAL0);

	return retval;
}

static s16 si114x_get_calibration(struct si1132_data *ssi, SI114X_CAL_S *si114x_cal, char security)
{
	return 0;
}

static s16 si114x_set_ucoef(struct si1132_data *ssi, u8 ref_ucoef[], SI114X_CAL_S *si114x_cal )
{
	s16 response;
	u8 ucoef[4] = { 0x29, 0x89, 0x02, 0x00 };

	response = si1132_block_write(ssi, REG_UCOEF0, 4, &ucoef[0]);
	return response;
}

static int si1132_configure_detection(struct si1132_data *ssi, int automode)
{
	short retval = 0;
	SI114X_CAL_S si114x_cal;

	retval += si1132_reset(ssi);

	retval += si1132_write_register(ssi, REG_PS_LED21, 0);
	retval += si1132_write_register(ssi, REG_PS_LED3, 0);

	/* UV Coefficients */
	si114x_get_calibration(ssi, &si114x_cal, 1);
	si114x_set_ucoef(ssi, 0, &si114x_cal);

	retval += si1132_param_set(ssi, PARAM_CH_LIST, UV_TASKLIST);

	if (automode)
		retval += si1132_write_register(ssi, REG_INT_CFG, ICG_INTOE);
	else
		retval += si1132_write_register(ssi, REG_INT_CFG, 0);

	retval += si1132_write_register(ssi, REG_IRQ_ENABLE, IE_ALS_EVRYSAMPLE);
	retval += si1132_param_set(ssi, PARAM_ALSIR_ADC_MISC, RANGE_EN);
	retval += si1132_param_set(ssi, PARAM_ALSVIS_ADC_MISC, RANGE_EN);

	if (automode) {
		/* Set up how often the device wakes up to make measurements (10ms)  multiplied by 31.25us*/
		retval += si1132_write_register(ssi, REG_MEAS_RATE_MSB, (MEASRATE_FAST & 0xff00) >> 8);
		retval += si1132_write_register(ssi, REG_MEAS_RATE_LSB, MEASRATE_FAST & 0x00ff);

		retval += si1132_als_auto(ssi);
	}

	return retval;
}

static int si1132_get_uv_data(struct si1132_data *ssi)
{
	int r_data = 0;
	u8 timeout = 0x14;

	mutex_lock(&ssi->lock);

	si1132_write_register(ssi, REG_MEAS_RATE_MSB, 0);
	si1132_write_register(ssi, REG_MEAS_RATE_LSB, 0);
	si1132_als_force(ssi);

	do {
		msleep(50);
	} while (((si1132_read_register(ssi, REG_IRQ_STATUS) & 1) == 0) && (--timeout));

	if (timeout <= 0) {
		dev_info(&ssi->client->dev, "%s read time out\n", __func__);
		r_data = -2;
		goto out;
	}

	r_data =  si1132_read_register(ssi, REG_AUX_DATA0);
	r_data |= si1132_read_register(ssi, REG_AUX_DATA1) << 8;
	if (r_data < 0)
		r_data = -1;

	out:
	si1132_write_register(ssi, REG_IRQ_STATUS, 0xff);
	mutex_unlock(&ssi->lock);

	return r_data;
}

static int si1132_report_data(struct si1132_data *ssi)
{
	int rc = 0;
	int data = si1132_get_uv_data(ssi);
	if (data < 0)
		return data;

	input_event(ssi->input_dev, EV_MSC, MSC_RAW, data);
	input_sync(ssi->input_dev);

	return rc;
}

static int si1132_enable_disable(struct si1132_data *ssi, int enable)
{
	if (enable) {
		atomic_set(&ssi->enabled, 1);
		ssi->pdata->power_on();
		si1132_configure_detection(ssi, 0);
	} else {
		atomic_set(&ssi->enabled, 0);
		ssi->pdata->power_off();
	}

	return 0;
}

static void si1132_work_f(struct work_struct *work)
{
	struct si1132_data *ssi = container_of(work, struct si1132_data,
			work_data.work);

	si1132_enable_disable(ssi, 1);
	si1132_report_data(ssi);
	si1132_enable_disable(ssi, 0);

	schedule_delayed_work(&ssi->work_data, ssi->delay_jiffies);
}

static void si1132_enable_work(struct si1132_data *ssi, int enable)
{
	if (enable)
		schedule_delayed_work(&ssi->work_data, ssi->delay_jiffies);
	else
		cancel_delayed_work_sync(&ssi->work_data);
}

static ssize_t si1132_show_uv_data(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int ret = 0;
	int r_data = 0;
	struct si1132_data *ssi = dev_get_drvdata(dev);

	si1132_enable_disable(ssi, 1);
	r_data = si1132_get_uv_data(ssi);
	si1132_enable_disable(ssi, 0);

	sprintf(buf, "%d\n", r_data);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t si1132_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct si1132_data *ssi = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&ssi->enabled));
}

static ssize_t si1132_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	unsigned long val;
	struct si1132_data *ssi = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &val);
	if (rc)
		return rc;

	if(val <= 0) {
		si1132_enable_work(ssi, 0);
	} else {
		si1132_enable_work(ssi, 1);
	}

	return count;
}

static ssize_t si1132_delay_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct si1132_data *ssi = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", ssi->delay_ms);
}

static ssize_t si1132_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc;
	unsigned long val;
	struct si1132_data *ssi = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &val);
	if (rc)
		return rc;

	if (val >= 10) {
		ssi->delay_ms = (unsigned int)val;
		ssi->delay_jiffies = msecs_to_jiffies(ssi->delay_ms);

		return strnlen(buf, count);
	}

	return -EINVAL;
}

static SENSOR_DEVICE_ATTR(uv_sensor_data, S_IRUGO | S_IWUGO | S_IXUGO,
		si1132_show_uv_data, NULL, 0);
static SENSOR_DEVICE_ATTR(enable_uv_sensor, S_IRUGO | S_IWUGO | S_IXUGO,
		si1132_enable_show, si1132_enable_store, 1);
static SENSOR_DEVICE_ATTR(poll_period_ms_uv, S_IRUGO | S_IWUGO | S_IXUGO,
		si1132_delay_show, si1132_delay_store, 2);

static struct attribute *si1132_attributes[] = {
		&sensor_dev_attr_uv_sensor_data.dev_attr.attr,
		&sensor_dev_attr_enable_uv_sensor.dev_attr.attr,
		&sensor_dev_attr_poll_period_ms_uv.dev_attr.attr,
		NULL,
};

static const struct attribute_group si1132_attribute_group = {
		.attrs = si1132_attributes,
};

static int si1132_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct si1132_data *ssi;
	u8 buf[3] = {0,0,0};
	int result = -1;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client not i2c capable\n");
		result = -ENODEV;
		goto i2c_client_err;
	}

	ssi = kzalloc(sizeof(*ssi), GFP_KERNEL);
	if (ssi == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data\n");
		result = -ENOMEM;
		goto i2c_client_err;
	}

	mutex_init(&ssi->lock);
	ssi->pdata = kmalloc(sizeof(*ssi->pdata), GFP_KERNEL);
	if(ssi->pdata == NULL)
		goto kfree_ssi;
	memcpy(ssi->pdata, client->dev.platform_data, sizeof(*ssi->pdata));
	ssi->client = client;

	/* power manage*/
	client->dev.init_name = client->name;
	if (ssi->pdata->board_init) {
		result = ssi->pdata->board_init(&client->dev);
		if (result < 0) {
			dev_err(&client->dev,"board_init failed ! errno = %d\n",result);
			goto kfree_pdata;
		}
	}

	if (ssi->pdata->power_off)
		ssi->pdata->power_off();
	//gpio_set_pull(ssi->pdata->gpio_int);

	if (ssi->pdata->power_on)
		ssi->pdata->power_on();

	atomic_set(&ssi->enabled, 1);
	/*read id must add to load si1132 */
	si1132_i2c_read_data(ssi, REG_PART_ID, buf, 1);
	if (SI1132_ID != buf[0]) {
		dev_err(&client->dev, "read si1132 chip ID failed ,buf[0] = 0x%x\n", buf[0]);
		goto exit_power_off;
	}
	dev_info(&client->dev, "UV sensor is si1132\n");
	i2c_set_clientdata(client, ssi);

	si1132_enable_disable(ssi, 0);

	ssi->delay_ms = 1000;
	INIT_DELAYED_WORK(&ssi->work_data, si1132_work_f);

	ssi->input_dev = input_allocate_device();
	if (!ssi->input_dev) {
		result = -ENOMEM;
		dev_err(&ssi->client->dev, "input device allocate failed\n");
		goto exit_power_off;
	}
	ssi->input_dev->name = "uv_sensor"; // must conform to android hal
	ssi->input_dev->id.bustype = BUS_I2C;
	input_set_capability(ssi->input_dev, EV_MSC, MSC_RAW);
	input_set_drvdata(ssi->input_dev, ssi);

	result = input_register_device(ssi->input_dev);
	if (result < 0) {
		dev_err(&ssi->client->dev,
				"unable to register input polled device %s\n",
				ssi->input_dev->name);
		goto exit_power_off;
	}

	result = sysfs_create_group(&client->dev.kobj,
				&si1132_attribute_group);
	if (result) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto exit_power_off;
	}

	return 0;

exit_power_off:
	ssi->pdata->power_off();
kfree_pdata:
	kfree(ssi->pdata);
kfree_ssi:
	kfree(ssi);
i2c_client_err:
	return result;
}

static int si1132_remove(struct i2c_client *client)
{
	struct si1132_data *ssi = i2c_get_clientdata(client);

	si1132_enable_disable(ssi, 0);
	sysfs_remove_group(&client->dev.kobj,
				&si1132_attribute_group);
	kfree(ssi);
	return 0;
}

static const struct i2c_device_id si1132_id[] = {
		{ SI1132_DRV_NAME, 0 },
		{ }
};
MODULE_DEVICE_TABLE(i2c, si1132_id);

static struct i2c_driver si1132_driver = {
		.driver = {
				.name	= SI1132_DRV_NAME,
				.owner	= THIS_MODULE,
		},
		.probe	= si1132_probe,
		.remove	= si1132_remove,
		.id_table = si1132_id,
};
module_i2c_driver(si1132_driver);

MODULE_AUTHOR("Huanglihong <lihong.huang@ingenic.com>");
MODULE_DESCRIPTION("Silibs labs Uv index and Ambient light Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

