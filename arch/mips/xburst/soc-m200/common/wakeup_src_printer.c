#include <linux/printk.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/moduleparam.h>

struct intc_regs {
	volatile unsigned int ICSR0;
	volatile unsigned int ICMR0;
	volatile unsigned int ICMSR0;
	volatile unsigned int ICMCR0;
	volatile unsigned int ICPR0;
	volatile unsigned int reserved[3];
	volatile unsigned int ICSR1;
	volatile unsigned int ICMR1;
	volatile unsigned int ICMSR1;
	volatile unsigned int ICMCR1;
	volatile unsigned int ICPR1;
	volatile unsigned int DSR0;
	volatile unsigned int DMR0;
	volatile unsigned int DPR0;
	volatile unsigned int DSR1;
	volatile unsigned int DMR1;
	volatile unsigned int DPR1;
};

#define INTC_IO_BASE 0xB0001000

extern void dump_gpio_port(int port);

static const char *intc0_src_name[32] = {
	"DMIC",
	"AIC0",
	"BCH",
	"DSI",
	"CSI",
	"OHCI",
	"IPU",
	"SSI1",
	"SSI0",
	"RESERVED",
	"PDMA",
	"RESERVED",
	"GPIO5",
	"GPIO4",
	"GPIO3",
	"GPIO2",
	"GPIO1",
	"GPIO0",
	"SADC",
	"EPDC",
	"EHCI",
	"OTG",
	"HASH",
	"AES",
	"RESERVED",
	"TCU2",
	"TCU1",
	"TCU0",
	"RESERVED",
	"ISP",
	"DELAY_LINE",
	"LCD",
};

static const char *intc1_src_name[32] = {
	"RTC",
	"RESERVED",
	"UART4",
	"MSC2",
	"MSC1",
	"MSC0",
	"RESERVED",
	"NFI",
	"PCM0",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"HARB2",
	"HARB1",
	"HARB0",
	"CPM",
	"UART3",
	"UART2",
	"UART1",
	"UART0",
	"DDR",
	"RESERVED",
	"EFUSE",
	"ETHC",
	"RESERVED",
	"I2C3",
	"I2C2",
	"I2C1",
	"I2C0",
	"PDMAM",
	"VPU",
	"GPU",
};

void show_gpio_wakeup_sources(int port);

struct rtc_time tm_suspend;
unsigned long total_suspend_time = 0;

unsigned int mask_rtc_wakeup = 0;
core_param(mask_rtc_wakeup, mask_rtc_wakeup, int, 0644);

void record_suspend_time(void) {
	struct rtc_device *rtc_dev = alarmtimer_get_rtcdev();
	struct intc_regs *intc = (void *)0xB0001000;

	if (mask_rtc_wakeup) {
		intc->ICMSR1 = (1 << 0);
	}

	if (rtc_dev == NULL)
		return;

	rtc_read_time(rtc_dev, &tm_suspend);
}

void show_suspend_time(void) {
	unsigned int days, hours, minutes, seconds;
	struct rtc_device *rtc_dev = alarmtimer_get_rtcdev();
	struct rtc_time tm;
	unsigned long old_time, new_time, delta;

	if (rtc_dev == NULL)
		return;

	rtc_read_time(rtc_dev, &tm);
	rtc_tm_to_time(&tm_suspend, &old_time);
	rtc_tm_to_time(&tm, &new_time);
	delta = new_time - old_time;

	days = delta / (24 * 60 * 60);
	hours = delta / (60 * 60) % 24;
	minutes = delta / 60 % 60;
	seconds = delta % 60;

	total_suspend_time += delta;

	pr_err("Total suspend: %ddays %dhours %dmintues %dseconds\n", days, hours, minutes, seconds);
	pr_err("Total suspend: all : %lu seconds\n", total_suspend_time);
}

void show_wakeup_sources(void) {
	struct intc_regs *intc = (void *)0xB0001000;
	int i;

	unsigned int reg = intc->ICPR0;

	pr_err("-----------------");
	show_suspend_time();

	for (i = 0; i < 32; ++i) {
		if (reg & (1 << i)) {
			pr_err("WAKE UP0: %d-> %s\n", i, intc0_src_name[i]);
			if (i >= 12 && i <= 17)
				show_gpio_wakeup_sources(17 - i);
		}
	}

	reg = intc->ICPR1;

	for (i = 0; i < 32; ++i) {
		if (reg & (1 << i)) {
			pr_err("WAKE UP1: %d-> %s\n", i, intc1_src_name[i]);
		}
	}
	pr_err("-----------------\n");

	if (mask_rtc_wakeup) {
		intc->ICMCR1 = (1 << 0);
	}
}
