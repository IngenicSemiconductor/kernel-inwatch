#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/fb.h>

static enum {
	POWER_STATE_ACTIVE = 0,
	POWER_STATE_EARLY_SUSPEND,
	POWER_STATE_SUSPEND,
	POWER_STATE_ENTER,

	/* keep last */
	POWER_STATE_REALSE,
	POWER_STATE_NUMS,
} power_state;

static enum {
	POWER_REQUEST_EARLY_SUSPEND,
	POWER_REQUEST_NONE,
} power_request;

const char *request_states[] = {
	[POWER_REQUEST_EARLY_SUSPEND] = "request-earlysuspend",
	[POWER_REQUEST_NONE] = "request-none",
};

const char *power_states[POWER_STATE_NUMS] = {
	[POWER_STATE_ACTIVE] = "active",
	[POWER_STATE_EARLY_SUSPEND] = "early_suspend",
	[POWER_STATE_SUSPEND] = "suspend",
	[POWER_STATE_ENTER] = "enter_suspend",
	[POWER_STATE_REALSE] = "device_relase",
};

static DEFINE_SPINLOCK(power_state_lock);
static wait_queue_head_t power_state_wq;
static wait_queue_head_t power_request_wq;
static int debug_info;
module_param(debug_info,  int, 0644);

#undef pr_debug

#define pr_debug(x...)                          \
    do {                                        \
        if (debug_info)                         \
            pr_info(x);                         \
    } while (0)

static inline void print_state(const char *tag) {
	pr_debug("%s ---> %s %s\n", tag, power_states[power_state], request_states[power_request]);
}

static ssize_t power_state_show(struct device *device, struct device_attribute *attr, char *buf) {
	char *s = buf;

	s += sprintf(buf, "%s", power_states[power_state]);

	return s - buf;
}
#define power_state_store NULL

static ssize_t request_earlysuspend_show(struct device *device, struct device_attribute *attr, char *buf) {
	int ret;
	char *s = buf;
	unsigned long irq_flags;

	print_state("request earlysuspend before");

	spin_lock_irqsave(&power_state_lock, irq_flags);
	power_request = POWER_REQUEST_EARLY_SUSPEND;
	spin_unlock_irqrestore(&power_state_lock, irq_flags);

	ret = wait_event_interruptible(power_state_wq, power_state == POWER_STATE_EARLY_SUSPEND);
	print_state("request earlysuspend after");

	if (ret < 0 && power_state != POWER_STATE_EARLY_SUSPEND) {
		power_request = POWER_REQUEST_NONE;
		return ret;
	} else {
		if (ret < 0)
			pr_info("power_state: wait has been interrupt, but state is early suspend\n");
		s += sprintf(buf, "%s", power_states[POWER_STATE_EARLY_SUSPEND]);
	}

	return s - buf;
}
#define request_earlysuspend_store NULL

static ssize_t release_earlysuspend_show(struct device *device, struct device_attribute *attr, char *buf) {
	unsigned long irq_flags;

	print_state("release earlysuspend");

	spin_lock_irqsave(&power_state_lock, irq_flags);
	power_request = POWER_REQUEST_NONE;
	spin_unlock_irqrestore(&power_state_lock, irq_flags);

	wake_up_all(&power_request_wq);

	return 0;
}
#define release_earlysuspend_store NULL

static struct device_attribute power_state_deivce_attrs[] = {
	__ATTR(power_state, S_IRUSR|S_IRGRP|S_IROTH, power_state_show, power_state_store),
	__ATTR(request_earlysuspend, S_IRUSR|S_IRGRP|S_IROTH, request_earlysuspend_show, request_earlysuspend_store),
	__ATTR(release_earlysuspend, S_IRUSR|S_IRGRP|S_IROTH, release_earlysuspend_show, release_earlysuspend_store),
};

static void remove_device_files(struct device *dev, struct device_attribute *attrs, unsigned int len) {
	unsigned int i;

	for (i = 0; i < len; ++i) {
		device_remove_file(dev, &attrs[i]);
	}
}

static int create_device_files(struct device *dev, struct device_attribute *attrs, unsigned int len) {
	unsigned int i;
	int ret;

	for (i = 0; i < len; ++i) {
		ret = device_create_file(dev, &attrs[i]);
		if (ret) {
			pr_err("Failed to create device files\n");
			remove_device_files(dev, attrs, i);
			return ret;
		}
	}
	return 0;
}

static void power_state_early_suspend(void) {
	unsigned long irq_flags;

	spin_lock_irqsave(&power_state_lock, irq_flags);
	power_state = POWER_STATE_EARLY_SUSPEND;
	spin_unlock_irqrestore(&power_state_lock, irq_flags);

	wake_up_all(&power_state_wq);

	print_state("early suspend");
}

void wait_slpt_linux_release_fb(void) {
	int ret;
	int not_sync = 0;
	int timeout = 3 * HZ;
	unsigned long irq_flags;

	print_state("late resume");
	ret = wait_event_interruptible_timeout(power_request_wq, power_request != POWER_REQUEST_EARLY_SUSPEND, timeout);

	spin_lock_irqsave(&power_state_lock, irq_flags);
	if (power_request == POWER_REQUEST_EARLY_SUSPEND) {
		power_request = POWER_REQUEST_NONE;
		not_sync = 1;
	}
	power_state = POWER_STATE_ACTIVE;
	spin_unlock_irqrestore(&power_state_lock, irq_flags);

	if (ret < 0 && not_sync) {
		pr_info("power_state: failed to wait power_request state to !(ealry suspend)\n");
	}

	pr_info("%s called timeout:%d ret:%d notsync:%d\n", __FUNCTION__, timeout, ret, not_sync);
}

static int power_state_fb_notifier_call(struct notifier_block *self,unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int mode;

	/* If we aren't interested in this event, skip it immediately ... */
	switch (event) {
		case FB_EVENT_BLANK:
		case FB_EVENT_MODE_CHANGE:
		case FB_EVENT_MODE_CHANGE_ALL:
		case FB_EARLY_EVENT_BLANK:
		case FB_R_EARLY_EVENT_BLANK:
			break;
		default:
			return 0;
	}

	mode = *(int *)evdata->data;
	if(event == FB_EVENT_BLANK){
		if(mode)
			power_state_early_suspend();
		//else
			//power_state_late_resume();
	}
	return 0;
}

static struct notifier_block power_state_fb_notifier = {
	.notifier_call = power_state_fb_notifier_call,
	.priority = 0,
};

int power_state_pm_notifier_call(struct notifier_block *nb, unsigned long msg, void *data) {
	int ret = 0;
	int not_sync = 0;
	int timeout = 1 * HZ;
	unsigned long irq_flags;

	if (msg == PM_SUSPEND_PREPARE) {
		ret = wait_event_interruptible_timeout(power_request_wq, power_request != POWER_REQUEST_EARLY_SUSPEND, timeout);

		spin_lock_irqsave(&power_state_lock, irq_flags);
		if (power_request == POWER_REQUEST_EARLY_SUSPEND) {
			power_request = POWER_REQUEST_NONE;
			not_sync = 1;
		}
		power_state = POWER_STATE_SUSPEND;
		spin_unlock_irqrestore(&power_state_lock, irq_flags);

		print_state("notify prepare");
	}else if (msg == PM_POST_SUSPEND) {
		spin_lock_irqsave(&power_state_lock, irq_flags);
		power_state = POWER_STATE_EARLY_SUSPEND;
		spin_unlock_irqrestore(&power_state_lock, irq_flags);

		wake_up_all(&power_state_wq);
		print_state("notify post");
	}

	if (ret < 0 && not_sync) {
		pr_info("power_state: failed to wait power_request state to !(ealry suspend)\n");
	}

	return NOTIFY_DONE;
}

struct notifier_block power_state_pm_notifier = {
	.notifier_call = power_state_pm_notifier_call,
	.priority = 0,
};

static int power_state_suspend(struct platform_device *pdev, pm_message_t state) {
	unsigned long irq_flags;

	spin_lock_irqsave(&power_state_lock, irq_flags);
	power_state = POWER_STATE_ENTER;
	spin_unlock_irqrestore(&power_state_lock, irq_flags);

	wake_up_all(&power_state_wq);

	return 0;
}

static int power_state_resume(struct platform_device *pdev) {
	unsigned long irq_flags;

	spin_lock_irqsave(&power_state_lock, irq_flags);
	power_state = POWER_STATE_SUSPEND;
	spin_unlock_irqrestore(&power_state_lock, irq_flags);

	return 0;
}

static int power_state_probe(struct platform_device *pdev) {
	int ret;

	pr_info("%s called\n", __FUNCTION__);

	power_state = POWER_STATE_ACTIVE;
	power_request = POWER_REQUEST_NONE;

	init_waitqueue_head(&power_state_wq);
	init_waitqueue_head(&power_request_wq);

	ret = fb_register_client(&power_state_fb_notifier);
	if (ret) {
		pr_err("power_state: Failed to register fb notifier\n");
		return ret;
	}

	ret = register_pm_notifier(&power_state_pm_notifier);
	if (ret) {
		pr_err("power_state: Failed to register pm notifier\n");
		goto remove_early_suspend;
	}

	ret = create_device_files(&pdev->dev, power_state_deivce_attrs, ARRAY_SIZE(power_state_deivce_attrs));
	if (ret) {
		pr_err("power_state: Failed to register pm notifier\n");
		goto remove_pm_notifier;
	}

	return 0;
remove_pm_notifier:
	unregister_pm_notifier(&power_state_pm_notifier);
remove_early_suspend:
	fb_unregister_client(&power_state_fb_notifier);
	return ret;
}

static int power_state_remove(struct platform_device *pdev) {
	unsigned long irq_flags;

	spin_lock_irqsave(&power_state_lock, irq_flags);
	power_state = POWER_STATE_REALSE;
	spin_unlock_irqrestore(&power_state_lock, irq_flags);

	pr_info("%s called\n", __FUNCTION__);
	remove_device_files(&pdev->dev, power_state_deivce_attrs, ARRAY_SIZE(power_state_deivce_attrs));
	unregister_pm_notifier(&power_state_pm_notifier);
	fb_unregister_client(&power_state_fb_notifier);
	return 0;
}

static struct platform_driver power_state_driver = {
	.probe = power_state_probe,
	.remove = power_state_remove,
	.suspend = power_state_suspend,
	.resume = power_state_resume,
	.driver = {
		.name = "power_state",
	},
};

static void power_state_release(struct device * dev)
{
	pr_info("%s called\n", __FUNCTION__);
	return ;
}

static struct platform_device power_state_device = {
	.name = "power_state",
	.dev = {
		.release = power_state_release,
	},
};

static int __init power_state_init(void) {
	int ret;

	ret = platform_device_register(&power_state_device);
	if (ret) {
		pr_err("power_state: failed to register platform device\n");
		return ret;
	}

	ret = platform_driver_register(&power_state_driver);
	if (ret) {
		pr_err("power_state: failed to register platform driver\n");
		goto unregister_platform_device;
	}

	return 0;
unregister_platform_device:
	platform_device_unregister(&power_state_device);
	return ret;
}

static void __exit power_state_exit(void) {
	platform_device_unregister(&power_state_device);
	platform_driver_unregister(&power_state_driver);
}

module_init(power_state_init);
module_exit(power_state_exit);
MODULE_AUTHOR("wu jiao");
MODULE_LICENSE("GPL");
