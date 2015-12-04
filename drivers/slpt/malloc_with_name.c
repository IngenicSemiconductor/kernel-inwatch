/*
 *  Copyright (C) 2015 Wu Jiao <jiao.wu@ingenic.com wujiaososo@qq.com>
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

#include <linux/slab.h>

void *malloc_with_name(unsigned int size, const char *name) {
	unsigned int len = strlen(name) + 1;
	char *p = kmalloc(size + len, GFP_KERNEL);
	if (p) {
		memcpy(p + size, name, len);
	}

	return p;
}
