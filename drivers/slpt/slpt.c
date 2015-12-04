/*
 *  Copyright (C) 2014 Fighter Sun <wanmyqawdr@126.com>
 *  Copyright (C) 2014 Wu Jiao <jwu@ingenic.cn>
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

#define KB (1024)
#define MB (1024 * 1024)

#ifdef CONFIG_SLPT_MAP_TO_KSEG2
volatile char __attribute__((aligned(2 * MB))) slpt_reserve_mem[SLPT_LIMIT_SIZE];
#endif

unsigned int slpt_task_is_enabled = 0;

#define TCSM_BASE 	(0xb3422000)
#define RETURN_ADDR 	(TCSM_BASE+0)
#define REG_ADDR 	(TCSM_BASE+4)

#define reg_write(reg, value)					\
	do {										\
		(*(volatile unsigned int *)reg) = value;	\
	} while (0)

#define reg_read(reg) (*(volatile unsigned int *)reg)

static DEFINE_MUTEX(slpt_lock);
static LIST_HEAD(slpt_handlers);

struct kset *slpt_kset;
struct kobject *slpt_kobj;
struct kobject *slpt_apps_kobj;
struct kobject *slpt_res_kobj;
struct kobject *slpt_configs_kobj;

/* currently runing task */
static struct slpt_task *slpt_cur = NULL;

/* currently been selected task, when suspend we run it, if enable */
struct slpt_task *slpt_select = NULL;

#define STATUS_SIZE (100)
#define SLPT_NAME_SIZE (100)
#define SLPT_EMPTY_NAME ("")

static char slpt_select_status[STATUS_SIZE] = "none\n";
static char slpt_select_request_name[SLPT_NAME_SIZE] = SLPT_EMPTY_NAME;
static const char *slpt_select_apply_name = SLPT_EMPTY_NAME;

/* enable the selected task to be run when suspend */
static int slpt_select_enable = 0;

/* saving the soc implemented suspend pm_enter function */
int (*slpt_save_pm_enter_func)(suspend_state_t state);

#define slpt_size_check(len) (((len) >= SLPT_NAME_SIZE) || (!(len)))

static int slpt_create_task_sysfs_file(struct slpt_task* task);
static void slpt_remove_task_sysfs_file(struct slpt_task *task);

static inline void slpt_set_cur(struct slpt_task *task) {
	slpt_cur = task;
}

/**
 * slpt_get_cur() - get current slpt task
 *
 * Just called by slpt*.c
 */
struct slpt_task *slpt_get_cur(void) {
	return slpt_cur;
}
EXPORT_SYMBOL(slpt_get_cur);

static inline int slpt_select_task(struct slpt_task *task) {
	if (!task) {
		slpt_select_apply_name = SLPT_EMPTY_NAME;
		slpt_select = NULL;
		slpt_select_enable = 0;
	} else {
		slpt_select_apply_name = task->name;
		slpt_select = task;
		slpt_select_enable = 0;
	}

	return 0;
}

static inline void slpt_set_select_enable(int enable) {
	if (slpt_select)
		slpt_select_enable = enable;
}

void slpt_enable_task(struct slpt_task *task, bool enable) {
	slpt_select_task(task);
	slpt_set_select_enable(enable);
}

static int slpt_probe(struct platform_device *pdev) {
	return 0;
}

static int slpt_remove(struct platform_device *pdev) {
	return 0;
}

int slpt_suspend(struct platform_device *pdev, pm_message_t state) {
	return 0;
}

int slpt_resume(struct platform_device *pdev) {
	return 0;
}

void slpt_shutdown(struct platform_device *pdev) {

}

static struct platform_driver slpt_driver = {
	.probe = slpt_probe,
	.remove  = slpt_remove,
	.driver.name = "slpt",
	.suspend = slpt_suspend,
	.resume = slpt_resume,
	.shutdown = slpt_shutdown,
};

static struct platform_device slpt_device = {
	.name = "slpt",
};

void kernel_dma_cache_sync(unsigned long addr, unsigned long size) {
	dma_cache_sync(&slpt_device.dev, (void *)addr, size, DMA_TO_DEVICE);
}
EXPORT_SYMBOL(kernel_dma_cache_sync);

static inline int slpt_is_loaded(struct slpt_task *task) {
	return task->state.is_loaded;
}

static inline int slpt_is_registered(struct slpt_task *task) {
	return task->state.is_registered;
}

static int slpt_is_inited(struct slpt_task *task) {
	return task->state.is_inited;
}

static int copy_pages_to_addr(void *dest, struct page **page, size_t size) {
	int i;
	void *page_data;

	for (i = 0; i < PFN_DOWN(size); ++i) {
		page_data = kmap(page[i]);
		memcpy(dest, page_data, PAGE_SIZE);
		kunmap(page[i]);
		dest += PAGE_SIZE;
	}
	if (size % PAGE_SIZE) {
		page_data = kmap(page[i]);
		memcpy(dest, page_data, size % PAGE_SIZE);
		kunmap(page[i]);
	}
	pr_info("SLPT: size: %d, pages: %d", size, i);
	return 0;
}

static int load_task(struct slpt_task *task) {
	const struct firmware *fw = task->task_fw;

	if (!fw || !fw->size || !task->load_at_addr)
		return -ENOMEM;

	if (fw->data) {
		memcpy(task->load_at_addr, fw->data, fw->size);
	} else if (fw->pages) {
		copy_pages_to_addr(task->load_at_addr, fw->pages, fw->size);
	} else {
		return -ENOMEM;
	}
	pr_info("SLPT: cp \"%s\"  %d bytes to addr %p\n", task->bin_desc, fw->size, task->load_at_addr);
	return 0;
}

static int slpt_load_task(struct slpt_task *task) {

	pr_info("SLPT: %s", __FUNCTION__);

	if (!slpt_is_registered(task)) {
		pr_err("SLPT: error: Load no registered slpt task: %s", task->name);
		return -ENODEV;
	}

	if (slpt_is_loaded(task))
		return 0;

	if (load_task(task)) {
		pr_err("SLPT: error: Failed load task fw: %s\n", task->bin_desc);
		return -ENOMEM;
	}

	return 0;
}

#define bigger_out(x, start, size) ((x) >= ((start) + (size)))
#define no_intersection(x1, s1, x2, s2) ((bigger_out(x1, x2, s2)) || (bigger_out(x2, x1, s1)))

#define in_area(x, start, size) (((x) >= (start)) && ((x) < (start) + (size)))

static inline int in_reserve_area(size_t x1, size_t s1) {
	return  in_area(x1, SLPT_RESERVE_ADDR, SLPT_LIMIT_SIZE) && in_area(x1 + s1 - 1, SLPT_RESERVE_ADDR, SLPT_LIMIT_SIZE);
}

static int slpt_get_fw_info(struct slpt_task *task) {
	unsigned int *a;

	task->size = task->task_fw->size;
	if (!(task->load_at_addr && task->run_at_addr)) {
		a = (unsigned int *)(task->task_fw->data ? task->task_fw->data : page_address(task->task_fw->pages[0]));
		a = (unsigned int *)((unsigned char *)a + 0x380);
		task->load_at_addr = (void *)a[0];
		task->run_at_addr = (void *)a[1];
		task->init_addr = (void *)a[2];
		task->exit_addr = (void *)a[3];
		task->run_every_time_addr = (void *)a[4];
		task->cache_prefetch_addr = (void *)a[5];
	}
	if (!in_area(task->run_at_addr, task->load_at_addr, task->size)) {
		return -EFAULT;
	}

	return 0;
}

static struct slpt_task *name_to_slpt_task_internal(const char *name) {
	struct list_head *pos;

	list_for_each(pos, &slpt_handlers) {
		struct slpt_task *t = list_entry(pos, struct slpt_task, link);
		pr_info("SLPT: --> %s\n", t->name);
		if (!strcmp(t->name, name))
			return t;
	}
	return NULL;
}

struct slpt_task *name_to_slpt_task(const char *name) {
	struct slpt_task *t;

	mutex_lock(&slpt_lock);
	t = name_to_slpt_task_internal(name);
	mutex_unlock(&slpt_lock);

	return t;
}
EXPORT_SYMBOL(name_to_slpt_task);

static int slpt_init_task_internal(struct slpt_task *task) {
	int (*init)(unsigned long api_addr, struct slpt_task *task);
	int ret;

	if (slpt_is_inited(task))
		return 0;

	init = (int (*)(unsigned long api_addr, struct slpt_task *task)) task->init_addr;
	if (!init) {
		pr_info("SLPT: info: task:%s no init addr\n", task->name);
		ret = -ENODEV;
		goto return_ret;
	}

	ret = init((unsigned long) slpt_app_get_api, task);
	if (ret) {
		pr_err("SLPT: error: slpt task :%s init failed\n", task->name);
		goto return_ret;
	}
	task->state.is_inited = 1;

return_ret:
	return ret;
}

int slpt_init_task(struct slpt_task *task) {
	int ret;

	mutex_lock(&slpt_lock);
	ret = slpt_init_task_internal(task);
	mutex_unlock(&slpt_lock);

	return ret;
}
EXPORT_SYMBOL(slpt_init_task);

static int slpt_exit_task_internal(struct slpt_task *task) {
	int (*exit)(unsigned long api_addr, struct slpt_task *task);

	if (!slpt_is_inited(task))
		return 0;

	exit = (int (*)(unsigned long api_addr, struct slpt_task *task)) task->exit_addr;
	if (!exit) {
		pr_info("SLPT: info: task:%s no exit addr\n", task->name);
		return -ENODEV;
	}

	task->state.is_inited = 0;

	return exit((unsigned long) slpt_app_get_api, task);
}

int slpt_exit_task(struct slpt_task *task) {
	int ret;

	mutex_lock(&slpt_lock);
	ret = slpt_exit_task_internal(task);
	mutex_unlock(&slpt_lock);

	return ret;
}
EXPORT_SYMBOL(slpt_exit_task);

/**
 * slpt_register_task() - register and slpt task
 *
 * @task->load_at_addr: address to load firmware, and if NULL, we will find it in firmware
 * @task->run_at_addr:  address to app's entrypiont, and if NULL, we will find it in firmware
 * @task->init_addr: address to app's init address
 * @task->exit_addr:  address to app's init address
 * @task->bin_desc:  descriptor of your firmware, use to get your firmware
 * @task->name:      task name, is a unique identification for the task
 *
 * @return_value: 0 if successed, nonzero if failed.
 */
int slpt_register_task(struct slpt_task *task, int init, bool need_request_firmware) {
	int ret = 0;
	size_t name_len;
	struct list_head *pos;
	struct slpt_task *t;

	mutex_lock(&slpt_lock);

	task->state.is_loaded = 0;
	task->state.is_registered = 0;
	task->state.is_inited = 0;

	if (!task->name) {
		ret = -EINVAL;
		pr_err("SLPT: error: task name must not be null\n");
		goto unlock;
	}

	name_len = strlen(task->name) + 1;
	if (slpt_size_check(name_len)) {
		ret = -EINVAL;
		pr_err("SLPT: error: task name len(%u) should in the range of (0 to %d)\n", name_len, SLPT_NAME_SIZE);
		goto unlock;
	}

	if (!task->bin_desc) {
		ret = -EINVAL;
		pr_err("SLPT: error: bin desc must not be null\n");
		goto unlock;
	}

	t = name_to_slpt_task_internal(task->name);
	if (t) {
		ret = -EINVAL;
		pr_err("SLPT: error: name already registered by (%s, %p, %d)\n", t->bin_desc, t->load_at_addr, t->size);
		goto unlock;
	}
	if (need_request_firmware) {
		ret = request_firmware(&task->task_fw, task->bin_desc, &slpt_device.dev);
		if (ret) {
			pr_err("SLPT: error: Failed to request firmware : (%s)\n", task->bin_desc);
			ret = -ENODEV;
			goto unlock;
		}
	}
	if (slpt_get_fw_info(task)) {
		pr_err("SLPT: error: Failed to get firmware info : (%s)\n", task->bin_desc);
		ret = -EFAULT;
		goto error_get_fw_info_failed;
	}

	if (!in_reserve_area((size_t)task->load_at_addr, task->size)) {
		pr_err("SLPT: error: Firmware is not in reserve area\n");
		pr_err("SLPT: error: (%s %p %d)\n", task->bin_desc, task->load_at_addr, task->size);
		ret = -EINVAL;
		goto error_not_in_reserve_area;
	}

	list_for_each(pos, &slpt_handlers) {
		t = list_entry(pos, struct slpt_task, link);
		if (!no_intersection(t->load_at_addr, t->size, task->load_at_addr, task->size)) {
			pr_err("SLPT: error: The bins has intersection\n");
			pr_err("SLPT: error: (%s %p %d) and (%s %p %d)\n",
				   t->bin_desc, t->load_at_addr, t->size, task->bin_desc, task->load_at_addr, task->size);
			ret = -EFAULT;
			goto error_bins_has_intersection;
		}
		if (t->load_at_addr > task->load_at_addr)
			break;
	}

	list_add_tail(&task->link, pos);
	task->state.is_registered = 1;

	INIT_LIST_HEAD(&task->res_handlers);
	INIT_LIST_HEAD(&task->method_handlers);

	if (slpt_load_task(task)) {
		pr_err("SLPT: error: Failed to load task fw\n");
		ret = -EINVAL;
		goto error_load_task_failed;
	}
	task->state.is_loaded = 1;

	ret = slpt_create_task_sysfs_file(task);
	if (ret) {
		pr_err("SLPT: error: Failed to create task sysfs file: %s\n", task->name);
		ret = -ENOMEM;
		goto error_create_task_sysfs_file_failed;
	}

	if (init) {
		ret = slpt_init_task_internal(task);
		if (ret) {
			pr_info("SLPT: info: slpt_init_task retrun with error code: (%s, %d)\n", task->name, ret);
			goto error_init_task_failed;
		}
	}

	goto free_firmware;
error_init_task_failed:
	slpt_remove_task_sysfs_file(task);
error_create_task_sysfs_file_failed:
	task->state.is_loaded = 0;
error_load_task_failed:
	task->state.is_registered = 0;
	list_del(&task->link);
error_bins_has_intersection:
error_not_in_reserve_area:
error_get_fw_info_failed:
free_firmware:
	if (need_request_firmware)
		release_firmware(task->task_fw);

	task->task_fw = NULL;
unlock:
	mutex_unlock(&slpt_lock);
	return ret;
}
EXPORT_SYMBOL(slpt_register_task);


void slpt_unregister_task(struct slpt_task *task) {
	mutex_lock(&slpt_lock);
	slpt_remove_task_sysfs_file(task);
	if (slpt_cur == task)
		slpt_set_cur(NULL);
	if (slpt_select == task)
		slpt_select_task(NULL);
	list_del(&task->link);
	slpt_exit_task_internal(task);
	task->state.is_registered = 0;
	task->state.is_loaded = 0;
	task->state.is_inited = 0;
	kobject_put(&task->kobj_res);
	kobject_put(&task->kobj);
	mutex_unlock(&slpt_lock);
}
EXPORT_SYMBOL(slpt_unregister_task);

#if 1
static int slpt_run_target_task(struct slpt_task *task) {
	void (*run)(unsigned long api_addr, struct slpt_task *task);

	pr_info("SLPT: info: %s is running---\n", task->name);
	pr_info("SLPT: run at %p\n", task->run_at_addr);

	run = (void (*)(unsigned long api_addr, struct slpt_task *task))task->run_at_addr;
	run((unsigned long)slpt_app_get_api, task);
	pr_info("SLPT: info: done\n");
	return 0;
}
#endif

/**
 * slpt_run_task() - run slpt task
 *
 * @task: task must a registered task by call slpt_register_task()
 */
int slpt_run_task(struct slpt_task *task) {
	int ret = 0;

	mutex_lock(&slpt_lock);

	if (!task) {
		pr_err("SLPT: error: slpt task can not be null\n");
		ret = -EINVAL;
		goto unlock;
	}

	ret = slpt_load_task(task);
	if (ret) {
		pr_err("SLPT: error: Failed to load task: %s\n", task->name);
		ret = -EINVAL;
		goto unlock;
	}

	slpt_set_cur(task);

	ret = slpt_run_target_task(task);
	if (ret) {
		pr_err("SLPT: error: Failed to run task: %s\n", task->name);
		ret = -EINVAL;
		goto unlock;
	}

unlock:
	mutex_unlock(&slpt_lock);
	return ret;
}
EXPORT_SYMBOL(slpt_run_task);

size_t slpt_print_task_info(char *buf, struct slpt_task *task) {
	char *p = buf;

	p += sprintf(p, "task : %s\n", task->name);
	p += sprintf(p, "fw : %s\n", task->bin_desc);
	p += sprintf(p, "size : %u\n", task->size);
	p += sprintf(p, "load at : %p\n", task->load_at_addr);
	p += sprintf(p, "run at : %p\n", task->run_at_addr);
	p += sprintf(p, "init at : %p\n", task->init_addr);
	p += sprintf(p, "exit at : %p\n", task->exit_addr);

	return p - buf;
}

int slpt_name_len_check(const char *buf, size_t count, size_t *lenp) {
	char *p;
	size_t len;

	p = memchr(buf, '\n', count);
	len = p ? p - buf : strlen(buf);
	*lenp = len;

	return  slpt_size_check(len) ? -EINVAL : 0;
}

int slpt_scanf_task_name(const char *buf, size_t count, char **name, char *status) {
	size_t len;
	char *str;

	if (slpt_name_len_check(buf, count, &len)) {
		pr_err("SLPT: error: request task name size not valid: size:%u\n", len);
		if (status)
			sprintf(status, "%s\n", "size_invalid");
		return -1;
	}

	str = *name ? *name : kmalloc(len + 1, GFP_KERNEL);
	if (!str) {
		pr_err("SLPT: error: Allocate slpt name failed\n");
		if (status)
			sprintf(status, "%s\n", "no_mem");
		return -2;
	}

	memcpy(str, buf, len);
	str[len] = '\0';
	*name = str;
	return 0;
}

#if 0
static int slpt_reload_all_task(void) {
	struct list_head *pos;

	mutex_lock(&slpt_lock);
	list_for_each(pos, &slpt_handlers) {
		struct slpt_task *t = list_entry(pos, struct slpt_task, link);
		pr_info("SLPT: --> %s\n", t->name);
		load_task(t);
	}
	mutex_unlock(&slpt_lock);
	return 0;
}
#endif

static ssize_t reload_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	return 0;
}

static ssize_t reload_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	struct slpt_task *task = container_of(kobj, struct slpt_task, kobj);

	mutex_lock(&slpt_lock);
	load_task(task);
	mutex_unlock(&slpt_lock);
	return count;
}

slpt_attr(reload);

static ssize_t info_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	struct slpt_task *task = container_of(kobj, struct slpt_task, kobj);

	return slpt_print_task_info(buf, task);
}

static ssize_t info_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	struct slpt_task *task = container_of(kobj, struct slpt_task, kobj);

	pr_info("SLPT: info: task name :%s\n", task->name);
	return count;
}

slpt_attr(info);

static char slpt_new_res_status[STATUS_SIZE] = "none\n";
#define RES_LENGHT 200

static ssize_t add_res_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {

	return sprintf(buf, "%s", slpt_new_res_status);
}

static ssize_t add_res_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	struct slpt_task *task = container_of(kobj, struct slpt_task, kobj);
	char name[SLPT_NAME_SIZE];
	char *p = name;
	struct slpt_app_res *res;
	struct slpt_res *sr;
	size_t len;

	pr_info("SLPT: info: test: (%s) count:%u strlenbuf:%u\n", buf, count, strlen(buf));

	mutex_lock(&slpt_lock);

	if (slpt_scanf_task_name(buf, count, &p, slpt_new_res_status)) {
		pr_err("SLPT: error: Failed to get task name from buf\n");
		goto return_count;
	}

	len = strlen(name) + 1;
	res = kzalloc(sizeof(*res) + len + RES_LENGHT, GFP_KERNEL);
	if (!res) {
		pr_err("SLPT: error: Allocate task memory failed\n");
		sprintf(slpt_new_res_status, "%s\n", "no_mem");
		goto return_count;
	}
	res->name = p = (char *)&res[1];
	memcpy(p, name, len);

	res->length = RES_LENGHT;
	res->type = SLPT_RES_MEM;
	res->addr = (void *)res + sizeof(*res) + len;

	sr = slpt_register_res(res, 1, task);
	if (!sr) {
		pr_err("SLPT: error: register res failed\n");
		sprintf(slpt_new_res_status, "%s\n", "failed");
		goto error_register_res_failed;
	}

	sprintf(slpt_new_res_status, "%s\n", "success");

	goto return_count;
error_register_res_failed:
	kfree(res);
return_count:
	mutex_unlock(&slpt_lock);
	return count;
}

slpt_attr(add_res);

static char slpt_rm_res_status[STATUS_SIZE] = "none\n";

static ssize_t rm_res_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_lock);
	count = sprintf(buf, "%s", slpt_rm_res_status);
	mutex_unlock(&slpt_lock);

	return count;
}

static ssize_t rm_res_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	char name[SLPT_NAME_SIZE];
	char *p = name;
	struct slpt_res *sr;
	struct slpt_task *task = container_of(kobj, struct slpt_task, kobj);

	if (slpt_scanf_task_name(buf, count, &p, slpt_rm_res_status)) {
		pr_err("SLPT: error: Failed to get task name from buf\n");
		goto return_count;
	}

	sr = name_to_slpt_res(name, task);
	if (!sr) {
		pr_info("SLPT: error: Failed to get slpt res:%s\n", name);
		sprintf(slpt_rm_res_status, "%s\n", "no_res");
		goto return_count;
	}

	slpt_unregister_res(sr);
	kfree(sr);
	sprintf(slpt_rm_res_status, "%s\n", "success");

	goto return_count;
return_count:
	return count;
}

slpt_attr(rm_res);

#define exit_show NULL

static ssize_t exit_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	struct slpt_task *task = container_of(kobj, struct slpt_task, kobj);

	slpt_exit_task(task);

	return count;
}

slpt_attr(exit);

#define init_show NULL

static ssize_t init_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	struct slpt_task *task = container_of(kobj, struct slpt_task, kobj);

	slpt_init_task(task);

	return count;
}

slpt_attr(init);

static struct attribute *slpt_app_g[] = {
	&info_attr.attr,
	&add_res_attr.attr,
	&rm_res_attr.attr,
	&init_attr.attr,
	&exit_attr.attr,
	&reload_attr.attr,
	NULL,
};

static void slpt_kobj_release(struct kobject *kobj) {
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
}

struct kobj_type slpt_kobj_ktype = {
	.release = slpt_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

static int slpt_create_task_sysfs_file(struct slpt_task* task) {
	int ret;

	ret = kobject_init_and_add(&task->kobj, &slpt_kobj_ktype, slpt_apps_kobj, "%s", task->name);
	if (ret) {
		pr_err("SLPT: error: Failed to add slpt task kobj: %s\n", task->name);
		ret =  -ENOMEM;
		goto error_task_kobj_create_failed;
	}

	ret = kobject_init_and_add(&task->kobj_res, &slpt_kobj_ktype, &task->kobj, "%s", "res");
	if (ret) {
		pr_err("SLPT: error: Failed to add slpt task res kobj: %s\n", task->name);
		ret = -ENOMEM;
		goto error_task_res_kobj_create_failed;
	}

	ret = slpt_register_method_kobj(task);
	if (ret) {
		pr_err("SLPT: error: Failed to add slpt task method kobj: %s\n", task->name);
		ret = -ENOMEM;
		goto error_task_method_kobj_create_failed;
	}

	task->group.name = NULL;
	task->group.attrs = slpt_app_g;

	ret = sysfs_create_group(&task->kobj, &task->group);
	if (ret) {
		pr_err("SLPT: error: slpt apps kobject create failed\n");
		ret = -ENOMEM;
		goto error_task_sys_group_create_failed;
	}

	return 0;
error_task_sys_group_create_failed:
	kobject_put(&task->kobj_method);
error_task_method_kobj_create_failed:
	kobject_put(&task->kobj_res);
error_task_res_kobj_create_failed:
	kobject_put(&task->kobj);
error_task_kobj_create_failed:
	return ret;
}

static void slpt_remove_task_sysfs_file(struct slpt_task *task) {
	sysfs_remove_group(&task->kobj, &task->group);
	kobject_put(&task->kobj);
}

static char slpt_run_status[STATUS_SIZE] = "none\n";

static ssize_t run_task_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {

	return 0;
}

static DEFINE_SPINLOCK(slpt_spin_lock);

static ssize_t run_task_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	unsigned long flags;
	struct slpt_task *task;
	char name[SLPT_NAME_SIZE];
	char *p = name;

	pr_info("p:%p\n", p);
	if (slpt_scanf_task_name(buf, count, &p, slpt_run_status)) {
		pr_err("SLPT: error: Failed to get task name from buf\n");
		goto return_count;
	}

	task = name_to_slpt_task(name);
	if (!task) {
		pr_err("SLPT: error: No task named (%s)\n", name);
		sprintf(slpt_run_status, "%s\n", "no_task");
		goto return_count;
	}

	spin_lock_irqsave(&slpt_spin_lock, flags);
	slpt_run_task(task);
	spin_unlock_irqrestore(&slpt_spin_lock, flags);

return_count:
	return count;
}

slpt_attr(run_task);

static ssize_t task_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_lock);
	count = sprintf(buf, "%s", slpt_select_status);
	mutex_unlock(&slpt_lock);

	return count;
}

static ssize_t task_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	struct slpt_task *task;
	char *p = slpt_select_request_name;
	pr_info("SLPT: info: test: (%s) count:%u strlenbuf:%u\n", buf, count, strlen(buf));

	mutex_lock(&slpt_lock);

	pr_info("p:%p\n", p);
	if (slpt_scanf_task_name(buf, count, &p, slpt_select_status)) {
		pr_err("SLPT: error: Failed to get task name from buf\n");
		goto unlock;
	}

	task = name_to_slpt_task_internal(slpt_select_request_name);
	if (!task) {
		pr_err("SLPT: error: No task name is (%s) be found\n", slpt_select_request_name);
		sprintf(slpt_select_status, "%s\n", "no_task");
		goto unlock;
	} else {
		/* task be found */
		sprintf(slpt_select_status, "%s\n", "success");
		slpt_select_task(task);
	}

unlock:
	mutex_unlock(&slpt_lock);
	return count;
}

slpt_attr(task);

static ssize_t apply_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_lock);
	count = sprintf(buf, "%s\n", slpt_select_apply_name);
	mutex_unlock(&slpt_lock);

	return count;
}

#define apply_store NULL

slpt_attr(apply);

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_lock);
	count = sprintf(buf, "%d\n", slpt_select_enable);
	mutex_unlock(&slpt_lock);

	return count;
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	int enable = simple_strtol(buf, NULL, 10);

	mutex_lock(&slpt_lock);
	if (enable == 0 || enable == 1) {
		slpt_set_select_enable(enable);
	}
	mutex_unlock(&slpt_lock);

	return count;
}

slpt_attr(enable);

static char slpt_add_status[STATUS_SIZE] = "none\n";

static ssize_t add_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_lock);
	count = sprintf(buf, "%s", slpt_add_status);
	mutex_unlock(&slpt_lock);

	return count;
}

static void fix_name(char *str) {
	while (*str != '\0') {
		if (*str == '.' || *str == '\\')
			*str = '-';
		++str;
	}
}

static ssize_t add_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	char name[SLPT_NAME_SIZE];
	char *p = name;
	struct slpt_task *task;
	size_t len;
	int ret;

	if (slpt_scanf_task_name(buf, count, &p, slpt_add_status)) {
		pr_err("SLPT: error: Failed to get task name from buf\n");
		goto return_count;
	}

	len = strlen(name) + 1;
	task = kzalloc(sizeof(*task) + len * 2, GFP_KERNEL);
	if (!task) {
		pr_err("SLPT: error: Allocate task memory failed\n");
		sprintf(slpt_add_status, "%s\n", "no_mem");
		goto return_count;
	}

	task->bin_desc = p = (char *)&task[1];
	memcpy(p, name, len);
	task->name = p = (char *)&task[1] + len;
	fix_name(name);
	memcpy(p, name, len);

	ret = slpt_register_task(task, 1, 1);
	if (ret) {
		pr_info("SLPT: error: Failed to register slpt task:%s\n", name);
		sprintf(slpt_add_status, "%s\n", "failed");
		goto error_slpt_register_task_failed;
	}
	sprintf(slpt_add_status, "%s\n", "success");

	goto return_count;
error_slpt_register_task_failed:
	kfree(task);
return_count:
	return count;
}

slpt_attr(add);

static char slpt_remove_status[STATUS_SIZE] = "none\n";

static ssize_t remove_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	size_t count;

	mutex_lock(&slpt_lock);
	count = sprintf(buf, "%s", slpt_remove_status);
	mutex_unlock(&slpt_lock);

	return count;
}

static ssize_t remove_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	char name[SLPT_NAME_SIZE];
	char *p = name;
	struct slpt_task *task;

	if (slpt_scanf_task_name(buf, count, &p, slpt_remove_status)) {
		pr_err("SLPT: error: Failed to get task name from buf\n");
		goto return_count;
	}

	task = name_to_slpt_task(name);
	if (!task) {
		pr_info("SLPT: error: Failed to get slpt task:%s\n", name);
		sprintf(slpt_remove_status, "%s\n", "no_task");
		goto return_count;
	}

	slpt_unregister_task(task);
	kfree(task);
	sprintf(slpt_remove_status, "%s\n", "success");

	goto return_count;
return_count:
	return count;
}

slpt_attr(remove);

ssize_t task_info_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	char *p = buf;
	struct list_head *pos;

	list_for_each(pos, &slpt_handlers) {
		struct slpt_task *task = list_entry(pos, struct slpt_task, link);

		p += slpt_print_task_info(p, task);
		p += sprintf(p, "\n");
	}

	return p - buf;
}

ssize_t task_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {

	return count;
}

slpt_attr(task_info);

static struct attribute *test_g[] = {
	&run_task_attr.attr,
	&task_attr.attr,
	&apply_attr.attr,
	&enable_attr.attr,
	&add_attr.attr,
	&remove_attr.attr,
	&task_info_attr.attr,
	NULL,
};

static struct attribute_group slpt_attrs_g = {
	.attrs = test_g,
	.name = NULL,
};

static char data_buf[] = "it is a test data buf\n";
#define DATA_BUF_SIZE (sizeof(data_buf))

static ssize_t firmware_data_read(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buffer, loff_t offset, size_t count)
{
	if (offset >= DATA_BUF_SIZE) {
		return 0;
	}

	if (count + offset > DATA_BUF_SIZE) {
		count = DATA_BUF_SIZE - offset;
	}

	if (count)
		memcpy(buffer, data_buf + offset, count);

	return count;
}

static ssize_t firmware_data_write(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr,
				   char *buffer, loff_t offset, size_t count)
{
	size_t retval = count;

	if (offset >= DATA_BUF_SIZE) {
		return retval;
	}

	if (count + offset > DATA_BUF_SIZE) {
		count = DATA_BUF_SIZE - offset;
	}

	if (count)
		memcpy(data_buf + offset, buffer, count);

	return retval;
}

static struct bin_attribute test_bin_attr = {
	.attr = { .name = "test-bin", .mode = 0644 },
	.size = 0,
	.read = firmware_data_read,
	.write = firmware_data_write,
};

extern void wlan_pw_en_disable(void);
void __weak wlan_pw_en_disable(void) {

}

static int slpt_notify_sys(struct notifier_block *this, unsigned long code,
	void *unused)
{
	if (code == SYS_POWER_OFF) {
		if (slpt_get_battery_capacity() == 0) {
			sys_sync();

#if defined(CONFIG_SLPT) && defined(CONFIG_SLPT_SHUTDOWN)
#if defined(CONFIG_SLPT_POWERDOWN_DEVICE)
            if (wlan_pw_en_disable)
                wlan_pw_en_disable();
#endif
           slpt_set_power_state(SLPT_V_LOW_POWER_SHUTDOWN);
            pm_suspend(PM_SUSPEND_MEM);
#endif
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block slpt_notifier = {
	.notifier_call	= slpt_notify_sys,
};

static int __init slpt_init(void) {
	int ret;

	slpt_alloc_maped_memory();

	slpt_cache_init();

	slpt_set_power_state(SLPT_V_POWER_NORMAL);

	ret = register_reboot_notifier(&slpt_notifier);
	if (ret) {
		pr_err("cannot register reboot notifier (err=%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&slpt_driver);
	if (ret) {
		pr_err("SLPT: error: slpt driver register failed\n");
		ret = -EINVAL;
		goto error_platform_driver_register_failed;
	}

	ret = platform_device_register(&slpt_device);
	if (ret) {
		pr_err("SLPT: error: slpt device register failed\n");
		ret = -EINVAL;
		goto error_platform_device_register_failed;
	}

	slpt_kset = kset_create_and_add("slpt", NULL, NULL);
	if (!slpt_kset) {
		pr_err("SLPT: error: slpt kset create failed\n");
		ret = -ENOMEM;
		goto error_slpt_kset_create_failed;
	}
	slpt_kobj = &slpt_kset->kobj;
	slpt_kobj->kset = slpt_kset;

	slpt_apps_kobj = kobject_create_and_add("apps", slpt_kobj);
	if (!slpt_apps_kobj) {
		pr_err("SLPT: error: slpt apps kobject create failed\n");
		ret = -ENOMEM;
		goto error_slpt_apps_kobj_create_failed;
	}

	slpt_res_kobj = kobject_create_and_add("res", slpt_kobj);
	if (!slpt_res_kobj) {
		pr_err("SLPT: error: slpt res kobject create failed\n");
		ret = -ENOMEM;
		goto error_slpt_res_kobj_create_failed;
	}

	slpt_configs_kobj = kobject_create_and_add("configs", slpt_kobj);
	if (!slpt_configs_kobj) {
		pr_err("SLPT: error: slpt configs kobject create failed\n");
		ret = -ENOMEM;
		goto error_slpt_configs_kobj_create_failed;
	}

	ret = slpt_configs_init();
	if (ret) {
		pr_err("SLPT: error: slpt configs init failed\n");
		ret = -ENOMEM;
		goto error_configs_init_failed;
	}

	ret = sysfs_create_group(slpt_kobj, &slpt_attrs_g);
	if (ret) {
		pr_err("SLPT: error: slpt sysfs group create failed\n");
		ret = -ENOMEM;
		goto error_create_slpt_g_failed;
	}

	ret = sysfs_create_bin_file(slpt_kobj, &test_bin_attr);
	if (ret) {
		pr_err("SLPT: error: slpt test bin sysfs create failed\n");
		ret = -ENOMEM;
		goto error_create_slpt_test_bin_failed;
	}

	return 0;
error_create_slpt_test_bin_failed:
	sysfs_remove_group(slpt_kobj, &slpt_attrs_g);
error_create_slpt_g_failed:
	slpt_configs_exit();
error_configs_init_failed:
	kobject_put(slpt_configs_kobj);
error_slpt_configs_kobj_create_failed:
	kobject_put(slpt_res_kobj);
error_slpt_res_kobj_create_failed:
	kobject_put(slpt_apps_kobj);
error_slpt_apps_kobj_create_failed:
	kset_unregister(slpt_kset);
error_slpt_kset_create_failed:
	platform_device_unregister(&slpt_device);
error_platform_device_register_failed:
	platform_driver_unregister(&slpt_driver);
error_platform_driver_register_failed:
	return ret;
}

static void __exit slpt_exit(void) {
	sysfs_remove_group(slpt_kobj, &slpt_attrs_g);
	kobject_put(slpt_apps_kobj);
	kobject_put(slpt_res_kobj);
	slpt_configs_exit();
	kobject_put(slpt_configs_kobj);
	kobject_put(slpt_kobj);
	platform_driver_unregister(&slpt_driver);
	platform_device_unregister(&slpt_device);
}

core_initcall_sync(slpt_init);
module_exit(slpt_exit);

extern void pm_p0_setup_tlb(unsigned int addr_fr0, unsigned int addr_fr1, unsigned int map_addr);

int slpt_pm_enter(suspend_state_t state) {
	int error = 0;
	int task_enable = 0;

	/* notfiy slpt fb's state */
	slpt_set_fb_on(fb_is_always_on());

	pr_info("SLPT: slect task: %s\n", slpt_select ? slpt_select->name : "null");
	pr_info("SLPT: enable: %d\n", slpt_select_enable);
	pr_info("SLPT: fb %s\n", fb_is_always_on() ? "on" : "off");
	pr_info("SLPT: sample adc %s\n", slpt_get_sample_adc_for_kernel() ? "yes" : "no");

	if (slpt_select && slpt_get_sample_adc_for_kernel())
		task_enable = 1;

	if (slpt_select && slpt_select_enable)
		task_enable = 1;

	if (task_enable) {
		slpt_task_is_enabled = 1;
#ifdef CONFIG_SLPT_MAP_TO_KSEG2
		error = slpt_suspend_in_kernel(state);
#else
		slpt_run_task(slpt_select);
#endif
	} else {
		slpt_task_is_enabled = 0;
		error = slpt_suspend_in_kernel(state);
	}

	return 0;
}

void slpt_set_suspend_ops(struct platform_suspend_ops *ops) {
	if (ops) {
		slpt_save_pm_enter_func = ops->enter;
		ops->enter = slpt_pm_enter;
	}

	suspend_set_ops(ops);
}

#ifdef CONFIG_SLPT_MAP_TO_KSEG2
void slpt_task_init_everytime(void) {
	void (*volatile run)(unsigned long api_addr, struct slpt_task *task);
	unsigned int addr = (unsigned int)virt_to_phys(slpt_reserve_mem);

	if (slpt_task_is_enabled) {
		slpt_setup_tlb(addr, addr + 1 * 1024 * 1024, SLPT_RESERVE_ADDR);
		slpt_cache_prefetch();

		run = (void (*)(unsigned long api_addr, struct slpt_task *task))slpt_select->run_at_addr;
		run((unsigned long)slpt_app_get_api, slpt_select);
	}
}

void slpt_cache_prefetch_ops(void) {
	void (*volatile run)(void);

	if (slpt_task_is_enabled) {
		run = (void (*)(void)) slpt_select->cache_prefetch_addr;
		run();
	}
}
#endif
