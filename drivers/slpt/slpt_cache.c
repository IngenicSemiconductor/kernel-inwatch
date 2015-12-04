/*
 *  Copyright (C) 2015 Wu Jiao <jwu@ingenic.cn, wujiaososo@qq.com>
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

#include <linux/slpt_cache.h>

unsigned long slpt_icache_line_size;
unsigned long slpt_scache_line_size;
unsigned long slpt_dcache_line_size;

unsigned long slpt_icache_waysize;
unsigned long slpt_scache_waysize;
unsigned long slpt_dcache_waysize;

unsigned long slpt_icache_waybit;
unsigned long slpt_scache_waybit;
unsigned long slpt_dcache_waybit;

unsigned long slpt_icache_ways;
unsigned long slpt_scache_ways;
unsigned long slpt_dcache_ways;

static unsigned long test = 10;

/**
 * slpt_cache_init() - init the important argument of slpt cache operations
 */
int slpt_cache_init(void) {
	pr_info("test: %ld\n", KSEG1VALUE(test));

	slpt_icache_line_size = cpu_icache_line_size();
	slpt_dcache_line_size = cpu_dcache_line_size();
	slpt_scache_line_size = cpu_scache_line_size();

	slpt_icache_waysize = current_cpu_data.icache.waysize;
	slpt_dcache_waysize = current_cpu_data.dcache.waysize;
	slpt_scache_waysize = current_cpu_data.scache.waysize;

	slpt_icache_waybit = current_cpu_data.icache.waybit;
	slpt_dcache_waybit = current_cpu_data.dcache.waybit;
	slpt_scache_waybit = current_cpu_data.scache.waybit;

	slpt_icache_ways = current_cpu_data.icache.ways;
	slpt_dcache_ways = current_cpu_data.dcache.ways;
	slpt_scache_ways = current_cpu_data.scache.ways;

	return 0;
}
