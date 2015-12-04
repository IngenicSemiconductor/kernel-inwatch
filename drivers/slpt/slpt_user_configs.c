/*
 *  Copyright (C) 2015 Wu Jiao <jwu@ingenic.cn wujiaoosos@qq.com>
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
#include <linux/list.h>
#include <linux/string.h>
#include <linux/slpt.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/slpt_battery.h>
#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/slpt_cache.h>

static DEFINE_MUTEX(slpt_configs_lock);

#ifndef CONFIG_LCD_BRIGHTNESS_ALWAYS_ON_LEVEL
#define CONFIG_LCD_BRIGHTNESS_ALWAYS_ON_LEVEL 102
#endif

int fb_always_on = 0;
int brightness_always_on = 0;
unsigned int brightness_always_on_level = CONFIG_LCD_BRIGHTNESS_ALWAYS_ON_LEVEL;

int brightness_is_always_on(void) {
	return brightness_always_on;
}

int fb_is_always_on(void) {
	return fb_always_on;
}

unsigned int get_brightness_always_on_level(void) {
	return brightness_always_on_level;
}

void set_fb_always_on(int on) {
	fb_always_on = !!on;
}

void set_brightness_always_on(int on) {
	brightness_always_on = !!on;
}

void set_brightness_always_on_level(unsigned int level) {
	brightness_always_on_level = level;
}

/*
 * config: fb_always_on
 */
static ssize_t fb_always_on_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", !!fb_always_on);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t fb_always_on_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_fb_always_on(enable);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(fb_always_on);

/*
 * config: brightness_always_on
 */
static ssize_t brightness_always_on_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", !!brightness_always_on);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t brightness_always_on_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_brightness_always_on(enable);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(brightness_always_on);

/*
 * config: brightness_always_on_level
 */
static ssize_t brightness_always_on_level_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_configs_lock);
	count = sprintf(buf, "%d\n", brightness_always_on_level);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

static ssize_t brightness_always_on_level_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int level = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_configs_lock);
	set_brightness_always_on_level(level);
	mutex_unlock(&slpt_configs_lock);

	return count;
}

slpt_attr(brightness_always_on_level);

static struct attribute *slpt_configs_attributes[] = {
	&fb_always_on_attr.attr,
	&brightness_always_on_attr.attr,
	&brightness_always_on_level_attr.attr,
	NULL,
};

static struct attribute_group slpt_configs_attrs_g = {
	.attrs = slpt_configs_attributes,
	.name = NULL,
};

int __init slpt_configs_init(void) {
	int ret;

	ret = sysfs_create_group(slpt_configs_kobj, &slpt_configs_attrs_g);
	if (ret) {
		pr_err("SLPT: error: slpt configs sysfs group create failed\n");
	}

	return ret;
}

int __exit slpt_configs_exit(void) {
	sysfs_remove_group(slpt_configs_kobj, &slpt_configs_attrs_g);
	
	return 0;
}
