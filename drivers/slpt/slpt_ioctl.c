/*
 *  Copyright (C) 2015 HarvisWang <maolei.wang@ingenic.com>
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/slpt.h>
#include <linux/string.h>
#include <linux/miscdevice.h>

#define SLPT_CMD_LOAD_FW     _IOW('S', 0x121, int)
#define SLPT_CMD_ENABLE_FW   _IOW('S', 0x122, int)
#define SLPT_CMD_DISABLE_FW  _IOW('S', 0x123, int)

#define TAG_SIZE 4
struct slpt_data {
	char tag[TAG_SIZE];
	unsigned int hdr_len;
	unsigned int mem_len;
	void *hdr;
	void *mem;
};

static long slpt_ioctl_load_firmware(void *hdr, unsigned int hdr_len,void *mem, unsigned int mem_len)
{
	struct firmware firmware;
	struct slpt_task *task;
	const char *name = hdr;
	unsigned int name_len = hdr_len;
	long err;

	if (hdr == NULL || mem == NULL)
		return -EINVAL;

	task = name_to_slpt_task(name);
	if (task != NULL) {
		slpt_unregister_task(task);
		kfree(task);
	}

	firmware.size = mem_len;
	firmware.data = mem;
	firmware.pages = NULL;
	pr_debug("firmware->size = %u\n", firmware.size);

	task = kzalloc(sizeof(struct slpt_task) + name_len, GFP_KERNEL);
	if (!task) {
		pr_err("slpt-ioctl: allocate task memory for slpt_task failed\n");
		return -ENOMEM;
	}

	task->name = (char *)&task[1];
	memcpy((char *)task->name, name, name_len);
	task->bin_desc = task->name;

	task->task_fw = &firmware;
	err = slpt_register_task(task, 1, 0);
	if (err) {
		pr_err("slpt-ioctl: error: Failed to register slpt task\n");
		return err;
	}

	return 0;
}

static long enable_task_inner(char *task_name, bool enable)
{
	struct slpt_task *task;

	if (task_name == NULL)
		return -EINVAL;

	task = name_to_slpt_task(task_name);
	if (!task) {
		pr_err("slpt-ioctl: error: No task name is (%s) be found\n", task_name);
		return -ENODEV;
	}

	slpt_enable_task(task, enable);

	return 0;
}

static long slpt_ioctl_enable_task(void *hdr, unsigned int hdr_len,void *mem, unsigned int mem_len)
{
	return enable_task_inner(hdr, 1);
}


static long slpt_ioctl_disable_task(void *hdr, unsigned int hdr_len,void *mem, unsigned int mem_len)
{
	return enable_task_inner(hdr, 0);
}

static long slpt_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	struct slpt_data head;
	struct slpt_data *user_data = (struct slpt_data *)args;
	void *mem = NULL, *hdr = NULL;
	long err;
	int (*ioctl)(void *hdr, unsigned hdr_len, void *mem, unsigned int mem_len, unsigned int cmd);

	err = copy_from_user(&head, user_data, sizeof(head));
	if (err) {
		pr_err("slpt-ioctl: copy struct slpt_data from user space failed.\n");
		return err;
	}

	if (head.tag[0] != 'S' || head.tag[1] != 'L' || head.tag[2] != 'P' || head.tag[3] != 'T') {
		pr_err("slpt-ioctl: tag error\n");
		return -EFAULT;
	}

	if ((head.hdr_len != 0 && head.hdr == NULL) || (head.mem_len != 0 && head.mem == NULL)) {
		pr_err("slpt-ioctl: no attached data error\n");
		return -EFAULT;
	}

	if (head.hdr_len != 0) {
		hdr = kmalloc(head.hdr_len, GFP_KERNEL);
		if (hdr == NULL) {
			pr_err("slpt-ioctl: allocate header memory failed.\n");
			err = -ENOMEM;
			goto out;
		}

		err = copy_from_user(hdr, head.hdr, head.hdr_len);
		if (err) {
			pr_err("slpt-ioctl: copy from header failed.\n");
			goto out_hdr;
		}
	}

	if (head.mem_len != 0) {
		if (head.mem_len > PAGE_SIZE)
			mem = vmalloc(head.mem_len);
	    else
			mem = kmalloc(head.mem_len, GFP_KERNEL);

		if (mem == NULL) {
			pr_err("slpt-ioctl: allocate body memory failed.\n");
			err = -ENOMEM;
			goto out_hdr;
		}

		err = copy_from_user(mem, head.mem, head.mem_len);
		if (err) {
			pr_err("slpt-ioctl: copy from body failed.\n");
			goto out_mem;
		}
	}

	switch (cmd) {
	case SLPT_CMD_LOAD_FW:
		err = slpt_ioctl_load_firmware(head.hdr, head.hdr_len, head.mem, head.mem_len);
		break;
	case SLPT_CMD_ENABLE_FW:
		err = slpt_ioctl_enable_task(head.hdr, head.hdr_len, head.mem, head.mem_len);
		break;
	case SLPT_CMD_DISABLE_FW:
		err = slpt_ioctl_disable_task(head.hdr, head.hdr_len, head.mem, head.mem_len);
		break;

	default:
		ioctl = slpt_get_ioctl();
		if (ioctl)
			err = ioctl(head.hdr, head.hdr_len, head.mem, head.mem_len, cmd);
		else
			err = -ENODEV;
		break;
	}

out_mem:
	if (mem) {
		if (head.mem_len > PAGE_SIZE)
			vfree(mem);
		else
			kfree(mem);
	}
out_hdr:
	if (hdr)
		kfree(hdr);
out:
	return err;
}

static ssize_t slpt_open(struct inode *inode, struct file *filp)
{
	pr_debug("%s ...\n", __func__);
	return 0;
}

static ssize_t slpt_close(struct inode *inode, struct file *filp)
{
	pr_debug("%s ...\n", __func__);
	return 0;
}

static ssize_t slpt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t slpt_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static struct file_operations slpt_file_ops = {
	.owner = THIS_MODULE,
	.open = slpt_open,
	.release = slpt_close,
	.write = slpt_write,
	.read = slpt_read,
	.unlocked_ioctl = slpt_ioctl,
};

static struct miscdevice dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &slpt_file_ops,
	.name = "slpt",
};

static int __init slpt_init(void)
{
	return	misc_register(&dev);
}

static void __exit slpt_exit(void)
{
	misc_deregister(&dev);
}

module_init(slpt_init);
module_exit(slpt_exit);


MODULE_AUTHOR("HarvisWang<maolei.wang@ingenic.com>");
MODULE_DESCRIPTION("/dev/slpt driver");
MODULE_LICENSE("GPL");
