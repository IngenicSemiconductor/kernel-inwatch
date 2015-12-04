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

#include <linux/backlight.h>
#include <linux/mutex.h>
#include <linux/default_backlight.h>
#include <linux/fb.h>

struct backlight_device *default_backlight = NULL;
static DEFINE_MUTEX(m_lock);

static void do_backlight_set_brightness(struct backlight_device *bd, unsigned int brightness) {
	if (!bd)
		return;

	if (brightness) {
		bd->props.power = FB_BLANK_UNBLANK;
		bd->props.state &= ~BL_CORE_FBBLANK;
		bd->props.state &= ~BL_CORE_SUSPENDED;
	}

	bd->props.brightness = brightness;
	backlight_update_status(bd);
}

void set_brightness_of_default_backlight(unsigned int brightness) {
	do_backlight_set_brightness(default_backlight, brightness);
}

void register_default_backlight(struct backlight_device *bd) {
	mutex_lock(&m_lock);
	BUG_ON(!bd);
	BUG_ON(default_backlight);

	default_backlight = bd;
	mutex_unlock(&m_lock);
}

void unregister_default_backlight(struct backlight_device *bd) {
	mutex_lock(&m_lock);
	BUG_ON(!default_backlight);
	BUG_ON(default_backlight != bd);

	default_backlight = NULL;
	mutex_unlock(&m_lock);
}
