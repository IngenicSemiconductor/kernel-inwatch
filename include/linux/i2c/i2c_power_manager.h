/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Author: Li Zidong <lizidong.marco@gmail.com>
 */

#include <linux/list.h>

#define POWER_OFF       0
#define POWER_ON        1

struct i2c_control_operations {
    int (*power_on)(void *data);
    int (*power_off)(void *data);
    int (*init_chip)(void *data);
};

struct i2c_power_manager {
    struct list_head handlers;
    struct list_head link;
    int bus_num;
    int device_count;
    int request_off_count;
    unsigned int state:1;
};

struct i2c_power_device {
    struct list_head link;
    int bus_num;
    struct i2c_control_operations *ops;
    unsigned int request_state:1;
    unsigned int state:1;
    void *data;
};

extern struct i2c_power_device *register_i2c_power_device(int bus_num,
        struct i2c_control_operations *ops, void *data);
extern void unregister_i2c_power_device(struct i2c_power_device *device);
extern int i2c_power_device_on(struct i2c_power_device *device);
extern int i2c_power_device_off(struct i2c_power_device *device);
