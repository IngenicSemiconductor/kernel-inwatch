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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/i2c/i2c_power_manager.h>

static LIST_HEAD(i2c_power_manager_list);
static DEFINE_MUTEX(lock);

static inline struct i2c_power_manager *find_manager(int bus_num)
{
    struct list_head *pos = NULL;
    struct i2c_power_manager *manager = NULL;

    list_for_each(pos, &i2c_power_manager_list) {
        manager = list_entry(pos, struct i2c_power_manager, link);
        if (manager->bus_num == bus_num)
            return manager;
    }

    return NULL;
}

static inline struct i2c_power_device *find_device(struct i2c_power_manager *manager,
        struct i2c_power_device *device)
{
    struct list_head *pos = NULL;

    if (manager == NULL || device == NULL)
        return NULL;

    list_for_each(pos, &manager->handlers) {
        struct i2c_power_device *d =
                list_entry(pos, struct i2c_power_device, link);

        if (d == device)
            return d;
    }

    return NULL;
}

/**
 * 申请注册I2C上的一个设备，利用i2c_power_manager进行管理。
 * 注意事项：使用该子系统的I2C设备的供电端都必须采用BOOT_ON，即开机时自动上电。
 *         原因是同一I2C上的设备如果存在一个设备没有正确上电，可能导致I2C总线被拉低，从而导致I2C无法正常通信。
 *         而这也是编写i2c_power_manager用于管理电源的初衷。
 *
 *         在正常情况下，注册之后，应该调用i2c_power_device_on()进行上电。
 *
 * @param bus_num:
 *          申请设备所处的I2C总线
 * @param ops:
 *          申请设备的上电、芯片初始化、关电等函数的结构体指针。
 *          上电、芯片初始化、关电函数必须初始化。
 *          其中上电函数应该只做单纯的上电功能，关电函数应该只做单纯的关电功能。
 * @param data:
 *          用户设备注册的私有数据指针，将会作为上电、芯片初始化、关电函数的传参回调。
 *
 * @return: 返回一个i2c_power_device类型的结构体指针
 */
struct i2c_power_device *register_i2c_power_device(int bus_num,
        struct i2c_control_operations *ops, void *data)
{
    struct i2c_power_device *device = NULL;
    struct i2c_power_manager *manager = NULL;

    if (ops == NULL)
        return NULL;

    if (ops->power_on == NULL || ops->power_off == NULL || ops->init_chip == NULL)
        return NULL;

    device = kmalloc(sizeof(struct i2c_power_device), GFP_KERNEL);
    if (device == NULL)
        return NULL;

    device->bus_num = bus_num;
    device->request_state = POWER_OFF;
    device->state = POWER_OFF;
    device->ops = ops;
    device->data = data;

    mutex_lock(&lock);

    manager = find_manager(bus_num);
    if (manager == NULL) {
        manager = kmalloc(sizeof(struct i2c_power_manager), GFP_KERNEL);
        if (manager == NULL) {
            kfree(device);
            mutex_unlock(&lock);
            return NULL;
        }

        manager->bus_num = bus_num;
        manager->device_count = 0;
        manager->request_off_count = 0;
        manager->state = POWER_OFF;
        INIT_LIST_HEAD(&manager->handlers);
        list_add_tail(&manager->link, &i2c_power_manager_list);
    }

    list_add_tail(&device->link, &manager->handlers);
    manager->device_count++;
    manager->request_off_count++;

    mutex_unlock(&lock);
    return device;
}
EXPORT_SYMBOL(register_i2c_power_device);

/**
 * 注销I2C上的一个设备。
 *
 * 由于注销一个设备时，会导致该设备的上电情况不明，这种情况下可能造成其他设备不能正常使用，所以不建议调用。
 * 建议调用的情景应该是在系统开机，某一个设备进行注册，在探索设备具体连接在某一条I2C总线上时，遇到错误时调用。
 */
void unregister_i2c_power_device(struct i2c_power_device *device)
{
    struct i2c_power_manager *manager = NULL;

    if (device == NULL)
        return;

    mutex_lock(&lock);

    manager = find_manager(device->bus_num);
    if (manager == NULL) {
        mutex_unlock(&lock);
        return;
    }

    if (find_device(manager, device)) {
        if (device->request_state == POWER_OFF)
            manager->request_off_count--;
        list_del(&device->link);
        kfree(device);
        manager->device_count--;
    }

    mutex_unlock(&lock);
}
EXPORT_SYMBOL(unregister_i2c_power_device);

/**
 * 设备申请进行上电。
 *
 * 只要有一个设备申请上电，所有连接到这条I2C上的设备都会处于上电状态。
 */
int i2c_power_device_on(struct i2c_power_device *device)
{
    struct list_head *pos = NULL;
    struct i2c_power_manager *manager = NULL;

    if (device == NULL)
        return -1;

    mutex_lock(&lock);

    manager = find_manager(device->bus_num);
    if (manager == NULL) {
        mutex_unlock(&lock);
        return -1;
    }

    if (find_device(manager, device) == NULL) {
        mutex_unlock(&lock);
        return -1;
    }

    if (device->request_state != POWER_ON) {
        manager->request_off_count--;
        device->request_state = POWER_ON;
    }

    if (device->state == POWER_ON) {
        mutex_unlock(&lock);
        return 0;
    }

    //如果该总线的设备都没上电，该总线上的所有设备执行先上电，后初始化
    if (manager->state != POWER_ON) {
        struct i2c_power_device *d = NULL;

        list_for_each(pos, &manager->handlers) {
            d = list_entry(pos, struct i2c_power_device, link);

            d->ops->power_on(d->data);
            d->state = POWER_ON;
        }

        list_for_each(pos, &manager->handlers) {
            d = list_entry(pos, struct i2c_power_device, link);

            d->ops->init_chip(d->data);
        }
        manager->state = POWER_ON;
    } else {
        device->ops->power_on(device->data);
        device->ops->init_chip(device->data);
        device->state = POWER_ON;
    }

    mutex_unlock(&lock);
    return 0;
}
EXPORT_SYMBOL(i2c_power_device_on);

/**
 * 设备申请关电。
 *
 * 只有当一条I2C上所连接的设备都申请关电，这条I2C上的设备才会全部被关电。
 */
int i2c_power_device_off(struct i2c_power_device *device)
{
    struct list_head *pos2 = NULL;
    struct i2c_power_manager *manager = NULL;

    if (device == NULL)
        return -1;

    mutex_lock(&lock);

    manager = find_manager(device->bus_num);
    if (find_device(manager, device) == NULL) {
        mutex_unlock(&lock);
        return -1;
    }

    if (device->request_state != POWER_OFF) {
        manager->request_off_count++;
        device->request_state = POWER_OFF;
    }

    if (device->state == POWER_OFF) {
        mutex_unlock(&lock);
        return 0;
    }

    if (manager->state == POWER_ON &&
            manager->request_off_count == manager->device_count) {
        list_for_each(pos2, &manager->handlers) {
            struct i2c_power_device *d = list_entry(pos2,
                    struct i2c_power_device, link);

            d->state = POWER_OFF;
            d->ops->power_off(d->data);
        }
        manager->state = POWER_OFF;
    }

    mutex_unlock(&lock);
    return 0;
}
EXPORT_SYMBOL(i2c_power_device_off);

static int __init i2c_power_manager_init(void)
{
    return 0;
}

static void __exit i2c_power_manager_exit(void)
{
}

subsys_initcall(i2c_power_manager_init);
module_exit(i2c_power_manager_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i2c_power_manager");
