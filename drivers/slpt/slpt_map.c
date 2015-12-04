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
#include <linux/vmalloc.h>
#include <linux/gfp.h>
#include <asm/r4kcache.h>

#ifdef CONFIG_SLPT_MAP_TO_KSEG2

static struct page **pages;
static unsigned int nr_pages = 0;
static struct vm_struct slpt_remap_vm;
static void *vm_addr;

/**
 * slpt_register_vm_area_early() - register vm area for slpt in kernel
 *
 * reserve the virtual address of kseg2 for slpt
 */
void slpt_register_vm_area_early(void) {
	BUG_ON(ALIGN(SLPT_LIMIT_SIZE, PAGE_SIZE) != SLPT_LIMIT_SIZE);
	nr_pages = SLPT_LIMIT_SIZE / PAGE_SIZE;

	slpt_remap_vm.flags = VM_ALLOC;
	slpt_remap_vm.size = nr_pages << PAGE_SHIFT;
	vm_area_register_early(&slpt_remap_vm, PAGE_SIZE);
	vm_addr = slpt_remap_vm.addr;
	BUG_ON(((unsigned long) vm_addr) != SLPT_RESERVE_ADDR);
}

/**
 * slpt_map_reserve_mem() - map slpt reserve memory to the reserved address
 */
void slpt_map_reserve_mem(void) {
	int ret;

	ret = map_vm_area(&slpt_remap_vm, PAGE_KERNEL, &pages);
	BUG_ON(ret < 0);
}

/**
 * slpt_unmap_reserve_mem() - unmap slpt reserve memory from the reserved address
 */
void slpt_unmap_reserve_mem(void) {
	unmap_kernel_range(SLPT_RESERVE_ADDR, SLPT_LIMIT_SIZE);
}

/**
 * slpt_alloc_maped_memory() - allocate the memory need by slpt
 *
 * allocate the memory need by slpt, will map to kseg2
 */
void slpt_alloc_maped_memory(void) {
	unsigned int i;
	volatile void *addr;

	pages = kmalloc(nr_pages * sizeof(void *), GFP_KERNEL);
	BUG_ON(!pages);
	memset(pages, 0, nr_pages * sizeof(void *));

	pr_info("=======================================\n");
	pr_info("slpt: reserved mem: %p\n", slpt_reserve_mem);
	pr_info("slpt: page addr: %p\n", virt_to_page(slpt_reserve_mem));
	pr_info("=======================================\n");

	addr = slpt_reserve_mem;

	for (i = 0; i < nr_pages; ++i) {
		pages[i] = virt_to_page(addr);
		addr += 1 << PAGE_SHIFT;
	}

	slpt_map_reserve_mem();
}
#else
void slpt_register_vm_area_early(void) {

}

void slpt_alloc_maped_memory(void) {

}

void slpt_map_reserve_mem(void) {

}

void slpt_unmap_reserve_mem(void) {

}
#endif
