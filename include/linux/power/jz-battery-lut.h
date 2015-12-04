/* include/linux/power/jz4780-battery.h
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 *	http://www.ingenic.com
 *	Sun Jiwei<jwsun@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __JZ4780_BATTERY_LUT_H
#define __JZ4780_BATTERY_LUT_H

#include <linux/power_supply.h>

struct jz_battery_info {

	int battery_max_cpt;
	int ac_chg_current;
	int usb_chg_current;

	unsigned int	sleep_current;
	int (*curve)[3];
};

struct jz_battery_platform_data {
	struct jz_battery_info info;
};

enum {
	USB,
	AC,
	STATUS,
};

struct jz_battery {
	struct jz_battery_platform_data *pdata;
	struct platform_device *pdev;

	struct resource *mem;
	void __iomem *base;

	int irq;

	const struct mfd_cell *cell;

	/* get from PMU driver, represent the current status of charging or discharging */
	int status;

	unsigned int voltage;

	struct completion read_completion;
	struct completion get_status_completion;

	struct power_supply battery;
	struct delayed_work work;
	struct delayed_work init_work;
	struct delayed_work resume_work;

	struct mutex lock;

	unsigned int next_scan_time;
	unsigned int usb;
	unsigned int ac;

	/* Online charger modified by PMU driver */
	unsigned int charger;
	struct regulator *ucharger;

	unsigned int ac_charge_time;
	unsigned int usb_charge_time;

	struct wake_lock work_wake_lock;

	__kernel_time_t resume_time;
	__kernel_time_t suspend_time;
	__kernel_time_t last_update_time;

	int capacity_show;
	int capacity_real;

	void *pmu_interface;
	int (*get_pmu_status)(void *pmu_interface, int status);
	void (*pmu_work_enable)(void *pmu_interface);
};

#define get_charger_online(bat, n)	((bat->charger & (1 << n)) ? 1 : 0)
#define set_charger_online(bat, n)	(bat->charger |= (1 << n))
#define set_charger_offline(bat, n)	(bat->charger &= ~(1 << n))

#endif
