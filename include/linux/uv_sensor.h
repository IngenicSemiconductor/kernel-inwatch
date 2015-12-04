#ifndef __UV_SENSORS_H__
#define __UV_SENSORS_H__
#include <linux/gpio.h>

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#endif

struct uvsensor_platform_data {
	int gpio_int;
	int (*board_init)(struct device *dev);
	int (*board_exit)(struct device *dev);
	int (*power_on)(void);
	int (*power_off)(void);
};
#endif
