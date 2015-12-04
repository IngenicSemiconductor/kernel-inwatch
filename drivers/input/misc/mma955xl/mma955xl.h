/*
 *  mma955xl.h - Linux kernel modules for 3-Axis Smart Orientation/ Motion Sensor
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
#ifndef _MMA955XL_H
#define _MMA955XL_H

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
#include <linux/input-polldev.h>

#define POLL_INTERVAL_MIN   1
#define POLL_INTERVAL_MAX   5000
#define POLL_INTERVAL       1000/* msecs */
#define POLL_STOP_TIME      200

#define MMA955XL_FCI_VER        0x00        // Used to request version information of the device such as ROM, Firmware and Hardware versions
#define MMA955XL_FCI_CONFIG_R           0x01        // Used to read configuration data from a specific application or gesture within the device.
#define MMA955XL_FCI_CONFIG_W           0x02        // Used to write configuration data from a specific application or gesture within the device.
#define MMA955XL_FCI_DATA_R                 0x03

#define MMA955XL_DRV_NAME       "mma955xl"
#define MODE_CHANGE_DELAY_MS    100
#define MMA955XL_COCOBYTE_ADDR  0x01
#define MMA955XL_COCOBIT        0x80

/*sleep/awake*/
#define MMA955XL_MODE_WAKE      1
#define MMA955XL_MODE_SLEEP     0

/*quick-read mode or  normal*/
#define MMA955XL_DATA_OF_RESPONSE(x)  (((struct mma955xl_rep_h_t *)(x))->data);
struct mma955xl_cmd_h_t {
    unsigned char rsvd;
    unsigned char appId;
    union {
        unsigned char val;
        struct {
            unsigned char offset_h :4;
            unsigned char cmd :3;
            unsigned char cc :1;
        } bits;
    } cmd;
    unsigned char offset;
    unsigned char count;
    unsigned char data[0];

};

struct mma955xl_rep_h_t {
    unsigned char appId;
    unsigned char error :7;
    unsigned char cc :1;
    unsigned char bytes_read;
    unsigned char bytes_rqst;
    unsigned char data[0];
};
struct mma955xl_data {
    void * bus_priv;
    u16 bus_type;
    int (*write)(struct mma955xl_data *pdata, u8* buf, int len);
    int (*read)(struct mma955xl_data *pdata, u8* buf, int len);
    struct input_polled_dev *poll_dev;
    struct mutex data_lock;
    int mode;
    int position;
    int wsmode; //wake or sleep
    struct kobject *kobj;
};
struct mma955xl_data_axis {
    short x;
    short y;
    short z;
};

struct mma955xl_pedometer_status {
    short status;
    short step_count;
    short distance;
    short speed;
    short calories;
    short sleep_count;
};
/* enum for mma955xl */
enum {
    MODE_2G = 0, MODE_4G, MODE_8G,
};

extern struct mma955xl_data mma955xl_dev;
int mma955xl_device_init(struct mma955xl_data *pdata);
int mma955xl_device_remove(struct mma955xl_data *pdata);
int mma955xl_suspend(struct mma955xl_data *pdata);
int mma955xl_resume(struct mma955xl_data *pdata);
#endif

