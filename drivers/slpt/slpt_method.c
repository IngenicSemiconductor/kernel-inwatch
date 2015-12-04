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

#include <linux/module.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/slpt.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include "argv_creator.h"

extern void *malloc_with_name(unsigned int size, const char *name);

#undef assert
#define assert(cond)											\
	do {														\
		if (!(cond)) {											\
			pr_err("#####assert condition failed (%s)#####\n", #cond);	\
			BUG_ON(1);											\
		}														\
	} while (0)

struct slpt_method {
	struct attribute attr;
	unsigned long addr;
	struct list_head link;
};

DEFINE_MUTEX(slpt_method_lock);
static int method_ret_value = 0;

static ssize_t slpt_method_show(struct kobject *kobj, struct slpt_method *method, char *buf) {
	size_t count = 0;

	mutex_lock(&slpt_method_lock);
	count += sprintf(buf, "%d", method_ret_value) + 1;
	mutex_unlock(&slpt_method_lock);

	return count;
}

static ssize_t slpt_method_store(struct kobject *kobj, struct slpt_method *method, const char *buf, size_t count) {
	int (*func)(int argc, char *argv[]);
	struct argv_creator *ac = NULL;
	int argc;
	char **argv;
	char *buf_tmp;

	if (count == 0) {
		argc = 0;
		argv = NULL;
	} else {
		buf_tmp = kmalloc(count + 1, GFP_KERNEL);
		if (!buf_tmp) {
			pr_err("SLPT: failed to alloc method tmp buffer\n");
			return count;
		}
		memcpy(buf_tmp, buf, count);
		buf_tmp[count] = '\0';
		ac = alloc_argv_creator(buf_tmp);
		if (!ac) {
			pr_err("SLPT: failed to alloc argv creator\n");
			kfree(buf_tmp);
			return count;
		}
		argc = ac->argc;
		argv = ac->argv;
	}

	mutex_lock(&slpt_method_lock);
	func = (void *)method->addr;
	method_ret_value = func(argc, argv);
	mutex_unlock(&slpt_method_lock);

	if (ac) {
		argv_creator_free(ac);
		kfree(buf_tmp);
	}

	return count;
}

/* slpt method kobject attribute operations */
static ssize_t slpt_method_kobj_attr_show(struct kobject *kobj, struct attribute *attr,
			      char *buf)
{
	struct slpt_method *method;
	ssize_t ret = -EIO;

	method = container_of(attr, struct slpt_method, attr);
	ret = slpt_method_show(kobj, method, buf);

	return ret;
}

static ssize_t slpt_method_kobj_attr_store(struct kobject *kobj, struct attribute *attr,
			       const char *buf, size_t count)
{
	struct slpt_method *method;
	ssize_t ret = -EIO;

	method = container_of(attr, struct slpt_method, attr);
	slpt_method_store(kobj, method, buf, count);

	return ret;
}

const struct sysfs_ops slpt_method_kobj_sysfs_ops = {
	.show	= slpt_method_kobj_attr_show,
	.store	= slpt_method_kobj_attr_store,
};

static void slpt_method_kobj_release(struct kobject *kobj) {
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
}

struct kobj_type slpt_method_kobj_ktype = {
	.release = slpt_method_kobj_release,
	.sysfs_ops = &slpt_method_kobj_sysfs_ops,
};

int slpt_register_method_kobj(struct slpt_task *task) {
	return kobject_init_and_add(&task->kobj_method, &slpt_method_kobj_ktype, &task->kobj, "%s", "method");
}

/**
 * slpt_register_method() - register method to kernel, slpt will use it
 * @task: the slpt task
 * @name: method name show to user
 * @addr: method address in slpt
 *
 * @return_val: return 0 if success, nagative value if failed.
 */
int slpt_register_method(struct slpt_task *task, const char *name, unsigned long addr) {
	struct slpt_method *method;
	struct list_head *pos;
	int ret;

	assert(task && addr && name);

	method = malloc_with_name(sizeof(*method), name);
	if (!method) {
		pr_err("SLPT: failed to alloc method: [%s] [0x%08lx]\n", name, addr);
		return -ENOMEM;
	}

	method->attr.name = (void *)method + sizeof(*method);
	method->addr = addr;
	method->attr.mode = 0666;

	mutex_lock(&slpt_method_lock);

	ret = sysfs_create_file(&task->kobj_method, &method->attr);
	if (ret) {
		pr_err("SLPT: failed to create method file : [%s] [%d]\n", name, ret);
		goto error_create_method_file_failed;
	}

	list_for_each(pos, &task->method_handlers) {
		struct slpt_method *m = list_entry(pos, struct slpt_method, link);
		if (strcmp(m->attr.name, name) > 0)
			break;
	}
	list_add_tail(&method->link, pos);

	goto unlock;
error_create_method_file_failed:
	kfree(method);
unlock:
	mutex_unlock(&slpt_method_lock);	
	return ret;
}
EXPORT_SYMBOL(slpt_register_method);

void slpt_unregister_method(struct slpt_task *task, const char *name) {
	struct list_head *pos;

	mutex_lock(&slpt_method_lock);

	list_for_each(pos, &task->method_handlers) {
		struct slpt_method *m = list_entry(pos, struct slpt_method, link);
		if (strcmp(m->attr.name, name) == 0) {
			list_del(&m->link);
			sysfs_remove_file(&task->kobj_method, &m->attr);
			kfree(m);
			break;
		}
	}

	mutex_unlock(&slpt_method_lock);
}

/* test functions */
#if 0
void slpt_method_test_func1(int argc, char *argv[]) {
	pr_info("%s: called\n", __func__);
}

void slpt_method_test_func2(int argc, char *argv[]) {
	pr_info("%s: called\n", __func__);	
}

void slpt_method_test_func3(int argc, char *argv[]) {
	pr_info("%s: called\n", __func__);
}

void slpt_method_test_func4(int argc, char *argv[]) {
	unsigned int i;

	for (i = 0; i < argc; ++i) {
		pr_info("%s\n", argv[i]);
	}

	pr_info("%s: called\n", __func__);
}

void slpt_method_test(struct slpt_task *task) {
	slpt_register_method(task, "slpt_method_test_func1", (unsigned long )slpt_method_test_func1);
	slpt_register_method(task, "slpt_method_test_func2", (unsigned long )slpt_method_test_func2);
	slpt_register_method(task, "slpt_method_test_func3", (unsigned long )slpt_method_test_func3);
	slpt_register_method(task, "slpt_method_test_func4", (unsigned long )slpt_method_test_func4);
	slpt_unregister_method(task, "slpt_method_test_func2");
	slpt_unregister_method(task, "slpt_method_test_func1");
	slpt_unregister_method(task, "slpt_method_test_func3");
}
#endif
