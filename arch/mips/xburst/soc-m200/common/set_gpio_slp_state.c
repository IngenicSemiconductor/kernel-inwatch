#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

extern int jzgpio_str2gpio(char *str);
extern void gpio_sleep_state(unsigned int group, unsigned int index,unsigned int state);

static void reset_gpio_sleep_state(unsigned int gpio_num,  int sleep_state)
{
	unsigned int group, index, state;

	if (gpio_num > GSS_TABLET_END) {
		printk("error: gpio_num is invalid!\n");
		return;
	}
	group = gpio_num / 32;
	index = gpio_num % 32;
	state = sleep_state;
	gpio_sleep_state(group, index, state);
}

static char *endchar_to_NUL(char * str, int len)
{
	char *ptr = NULL;
	if (!str)
		return NULL;
	ptr = str;
	while (--len && *str != '\n') {
		str++;
	}
	*str = '\0';
	return ptr;
}

static int stricmp(const char *s1, const char *s2)
{
	int c1, c2;

	do {
		c1 = tolower(*s1++);
		c2 = tolower(*s2++);
	} while (c1 == c2 && c1 != 0);
	return c1 - c2;
}

static int str_to_sleep_type(char *str)
{
	int sleep_type = 0;

	if(!str){
		pr_err("%s: str is null.\n", __func__);
		return sleep_type;
	}
	if (0 == stricmp(str, "LOW") || 0 == stricmp(str, "LO")) {
		sleep_type = GSS_OUTPUT_LOW;
	} else if (0 == stricmp(str, "high") || 0 == stricmp(str, "hi")) {
		sleep_type = GSS_OUTPUT_HIGH;
	} else if (0 == stricmp(str, "pull") || 0 == stricmp(str, "pu")) {
		sleep_type = GSS_INPUT_PULL;
	} else if (0 == stricmp(str, "nopull") || 0 == stricmp(str, "nop")) {
		sleep_type = GSS_INPUT_NOPULL;
	} else if (0 == stricmp(str, "ignore") || 0 == stricmp(str, "ign")) {
		sleep_type = GSS_IGNORE;
	} else {
		return -EINVAL;
	}

	return sleep_type;
}

static ssize_t gpio_set_sleep_mode_write(struct file *file, 
							const char __user * buf, size_t len, loff_t * off)
{
	char *str = NULL;
	char sleep_mode[64] = "";
	char gpio_port[64] = "";
	int gpio_num = 0, sleep_type;

	if (!buf) {
		pr_err("%s: buf is null!\n", __func__);
		return -EINVAL;
	}
 
	if (len > 64) {
		pr_err("%s: length of %s is %d, bigger then 64.\n", __func__, buf, len);
		return -EINVAL;
	}

	str = (char *)buf;
	str = endchar_to_NUL(str, len);
	sscanf(str, "%s %s", gpio_port, sleep_mode);
	if (*gpio_port == '\0' || *sleep_mode == '\0') {
		pr_err("%s: %s of write buf is invalid.\n", __func__, str);
		return -EINVAL;
	}

	gpio_num = jzgpio_str2gpio(gpio_port);
	if (gpio_num < 0) {
		pr_err("%s: %s is invalid.\n", __func__, gpio_port);
		return -EINVAL;
	}

	sleep_type = str_to_sleep_type(sleep_mode);
	if (sleep_type < 0) {
		pr_err("%s: sleep_mode[%s] is invalid\n", __func__, sleep_mode);
		return -EINVAL;
	}

	reset_gpio_sleep_state(gpio_num, sleep_type);
	return len;
}

static int gpio_proc_sleep_show(struct seq_file *m, void *v)
{
	return 0;
}

static int gpio_sleep_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpio_proc_sleep_show, PDE_DATA(inode));
}

struct file_operations gpios_sleep_mode_fops = {
	.read = seq_read,
	.write = gpio_set_sleep_mode_write,
	.open = gpio_sleep_open,
};
