/*
 *  Copyright (C) 2015 Wu Jiao <jwu@ingenic.cn>
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
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/workqueue.h>

static ssize_t write_file(const char *filename, void *buf, 	ssize_t lenght) {
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	ssize_t ret;

	fp = filp_open(filename, O_WRONLY, 0666);
	if (IS_ERR(fp)) {
		pr_err("%s failed to open file %s\n", __FUNCTION__, filename);
		return PTR_ERR(fp);
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	ret = vfs_write(fp, buf, lenght, &pos);
	filp_close(fp, NULL);
	set_fs(fs);

	return ret;
}

static ssize_t write_string_to_file(const char *filename, void *buf) {
	ssize_t lenght;

	if (buf == NULL)
		return -EINVAL;

	lenght = strlen(buf) + 1;

	return write_file(filename, buf, lenght);
}

static void slpt_loader_work_func(struct work_struct *work) {
	unsigned int sample_period = 60;

	pr_err("%s %d\n", __FUNCTION__, __LINE__);

	/*
	 *  echo slpt-app > /sys/slpt/add
	 *  echo slpt-app > /sys/slpt/task
	 *  echo 1 > /sys/slpt/enable
	 *  echo -e "\x3c\x00\x00\x00" > /sys/slpt/apps/slpt-app/res/sample-period/data
	 */
	write_string_to_file("/sys/slpt/add", "slpt-app");
	write_string_to_file("/sys/slpt/task", "slpt-app");
	write_string_to_file("/sys/slpt/enable", "1");
	write_file("/sys/slpt/apps/slpt-app/res/sample-period/data", &sample_period, 4);
}

static DECLARE_DELAYED_WORK(slpt_loader_work, slpt_loader_work_func);

/* 现在kernel里面默认编译了slpt-app的固件，
 * 所以再用一个脚本来执行load操作显得有点多余，
 * 为了不改最大程度的省事儿，用kernel来写sysfs文件接口完成slpt的加载动作
 */
int slpt_load_self(void) {

	schedule_delayed_work(&slpt_loader_work, msecs_to_jiffies(5 * 1000));

	return 0;
}

late_initcall_sync(slpt_load_self);
