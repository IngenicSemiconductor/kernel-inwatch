/* drivers/power/jz4780-battery.c
 *
 * Battery measurement code for Ingenic JZ SoC
 *
 * Copyright(C)2012 Ingenic Semiconductor Co., LTD.
 *	http://www.ingenic.cn
 *	Sun Jiwei <jwsun@ingenic.cn>
 * Based on JZ4740-battery.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/proc_fs.h>
#include <linux/mfd/core.h>
#include <linux/power_supply.h>

#include <linux/power/jz-battery-lut.h>
#include <linux/mfd/act8600-private.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_SLPT
#include <linux/slpt_battery.h>
#include <linux/slpt.h>
#endif

//#define LOG_TAG "Battery driver: "
#define LOG_TAG "\e[0;91mBattery driver:\e[0m "

#ifndef CONFIG_SLPT
static struct alarm alarm;
#endif

#ifndef CONFIG_SLPT_LOW_BATTERY_VOLTAGE
#define CONFIG_SLPT_LOW_BATTERY_VOLTAGE 3624 /* for 16t is 10% */
#endif

#ifndef CONFIG_SLPT_LOW_BATTERY_WARN_VOLTAGE
#define CONFIG_SLPT_LOW_BATTERY_WARN_VOLTAGE 3408 /* for 16t is 10% */
#endif

#if CONFIG_SLPT_LOW_BATTERY_WARN_VOLTAGE > CONFIG_SLPT_LOW_BATTERY_VOLTAGE
#error "CONFIG_SLPT_LOW_BATTERY_WARN_VOLTAGE can not bigger than CONFIG_SLPT_LOW_BATTERY_VOLTAGE"
#endif

#ifdef CONFIG_SLPT
static unsigned int low_battery_voltage = CONFIG_SLPT_LOW_BATTERY_VOLTAGE;
static unsigned int low_battery_warn_voltage = CONFIG_SLPT_LOW_BATTERY_WARN_VOLTAGE;
core_param(low_battery_voltage, low_battery_voltage, int, 0644);
core_param(low_battery_warn_voltage, low_battery_warn_voltage, int, 0644);
#endif

static bool external_cb_en = 0;
static bool force_full = 0;

static inline struct jz_battery *psy_to_jz_battery(struct power_supply *psy) {
    return container_of(psy, struct jz_battery, battery);
}

static const char *status2str(int status) {
    switch (status) {
    case POWER_SUPPLY_STATUS_CHARGING:
        return "CHARGING";
    case POWER_SUPPLY_STATUS_DISCHARGING:
        return "DISCHARGING";
    case POWER_SUPPLY_STATUS_NOT_CHARGING:
        return "NOT_CHARGING";
    case POWER_SUPPLY_STATUS_FULL:
        return "FULL";
    default:
        return "UNKNOWN";
    }
}

static int is_charging(int status) {
    switch (status) {
    case POWER_SUPPLY_STATUS_CHARGING:
    case POWER_SUPPLY_STATUS_FULL:
        return 1;
    default:
        return 0;
    }
}

static int get_status(struct jz_battery *bat) {
    int status;
    status = bat->get_pmu_status(bat->pmu_interface, STATUS);
    if ((status == POWER_SUPPLY_STATUS_CHARGING) && force_full
            && (bat->capacity_real > 85)) {

        pr_info(LOG_TAG "Force FULL\n");
        return POWER_SUPPLY_STATUS_FULL;
    }
    if (status == POWER_SUPPLY_STATUS_FULL)
        force_full = 1;
    return status;
}

/* ADC */

static long slop;
static long cut;

#define REG_EFSCTL        0xb34100dc
#define REG_EFUSTATE0     0xb34100e0
#define REG_EFUSTATE1     0xb34100e4
#define REG_EFUSTATE2     0xb34100e8
#define REG_EFUSTATE3     0xb34100ec
#define REG_EFUSTATE4     0xb34100f0
#define REG_EFUSTATE5     0xb34100f4
#define REG_EFUSTATE6     0xb34100f8
#define REG_EFUSTATE7     0xb34100fc

#define REG_READ(addr)          ((unsigned int)(*(volatile unsigned int *)(addr)))
#define REG_WRITE(value, addr)	(*(volatile unsigned int *)(addr) = (value))

static void get_slop_cut(void) {
    unsigned int tmp_new, tmp_old;
    char slop_r;
    char cut_r;

    tmp_new = REG_READ(REG_EFUSTATE4);
    tmp_old = REG_READ(REG_EFUSTATE2);

    if (tmp_new) {
        slop_r = (tmp_new >> 8) & 0xff;
        cut_r = tmp_new & 0xff;

        slop = 2 * slop_r + 2939;
        cut = 1250 * cut_r + 52000;
        pr_debug(LOG_TAG "get new slop = %ld cut = %ld\n", slop, cut);
    } else if (tmp_old) {
        slop_r = (tmp_old >> 8) & 0xff;
        cut_r = tmp_old & 0xff;

        slop = 2 * slop_r + 2935;
        cut = 1000 * cut_r + 70000;
        pr_debug(LOG_TAG "get old slop = %ld cut = %ld\n", slop, cut);
    } else {
        slop = 0;
        cut = 0;
        pr_debug(LOG_TAG "slop cut is zero\n");
    }
}

static irqreturn_t adc_irq_handler(int irq, void *devid) {
    struct jz_battery *bat = devid;

    complete(&bat->read_completion);
    return IRQ_HANDLED;
}

static unsigned int get_adc_value(struct jz_battery *bat) {
    unsigned long tmp;
    unsigned int value;

    mutex_lock(&bat->lock);

    INIT_COMPLETION(bat->read_completion);

    bat->cell->enable(bat->pdev);
    enable_irq(bat->irq);

    tmp = wait_for_completion_interruptible_timeout(&bat->read_completion, HZ);
    if (tmp > 0) {
        value = readw(bat->base) & 0xfff;
    } else {
        value = tmp ? tmp : -ETIMEDOUT;
    }

    disable_irq(bat->irq);
    bat->cell->disable(bat->pdev);

    mutex_unlock(&bat->lock);

    return value;
}

static int get_adc_voltage(struct jz_battery *bat) {
    unsigned int current_value[12], final_value = 0;
    unsigned int value_sum = 0;
    unsigned int max_value, min_value;
    unsigned int i, j = 0, temp;
    unsigned int max_index = 0, min_index = 0;
    unsigned int real_voltage = 0;

    current_value[0] = get_adc_value(bat);

    value_sum = max_value = min_value = current_value[0];

    for (i = 1; i < 12; i++) {
        current_value[i] = get_adc_value(bat);
        value_sum += current_value[i];
        if (max_value < current_value[i]) {
            max_value = current_value[i];
            max_index = i;
        }
        if (min_value > current_value[i]) {
            min_value = current_value[i];
            min_index = i;
        }
    }

    value_sum -= (max_value + min_value);
    final_value = value_sum / 10;

    for (i = 0; i < 12; i++) {
        if (i == min_index)
            continue;
        if (i == max_index)
            continue;

        temp = abs(current_value[i] - final_value);
        if (temp > 4) {
            j++;
            value_sum -= current_value[i];
        }
    }

    if (j < 10) {
        final_value = value_sum / (10 - j);
    }

    real_voltage = final_value * 1200 / 4096;
    real_voltage *= 4;

    if ((slop == 0) && (cut == 0)) {
        pr_info(LOG_TAG "ADC voltage is %dmV\n", real_voltage);
    } else {
        unsigned int origin_voltage;
        origin_voltage = real_voltage;
        real_voltage = (final_value * slop + cut) / 10000;
        real_voltage *= 4;
        pr_info(LOG_TAG "ADC voltage is %d(%d)mV\n",
                real_voltage, origin_voltage);
    }

    return real_voltage;
}

/* major */

extern const int jz_cv_discharging[101];
extern const int jz_cv_charging[101];

static int lookup_capacity(int charging, int volt) {
    int i;
    const int *cv;

    if (charging)
        cv = jz_cv_charging;
    else
        cv = jz_cv_discharging;

    for (i = 100; i > 0; --i) {
        if (volt >= cv[i])
            return i;
    }
    return 0;
}

#define VOLTAGE_BOUNDARY 80

#define MAX_CT 120
#define MIN_CT 8

static void jz_battery_next_scantime_falling(struct jz_battery *bat) {
    if (bat->capacity_show >= 90) {
        bat->next_scan_time = 120;
    } else if (bat->capacity_show >= 15) {
        bat->next_scan_time = 60;
    } else if (bat->capacity_show >= 8) {
        bat->next_scan_time = 30;
    } else {
        bat->next_scan_time = 10;
    }

    if (bat->capacity_real < bat->capacity_show - 10)
        bat->next_scan_time = 30;
}

static void jz_battery_capacity_falling(struct jz_battery *bat) {
    jz_battery_next_scantime_falling(bat);

    if (bat->capacity_real < bat->capacity_show)
        if (bat->capacity_show != 0)
            bat->capacity_show--;
}

static void jz_battery_next_scantime_rising(struct jz_battery *bat) {
    bat->next_scan_time = 40;

    if (bat->capacity_show >= 100) {
        bat->next_scan_time = 60;
        bat->capacity_show = 99;
        return;
    } else if (bat->capacity_show == 99) {
        bat->next_scan_time = 60;
        return;
    }

    if (bat->capacity_real > bat->capacity_show + 5)
        bat->next_scan_time = 50;
    if (bat->capacity_real > bat->capacity_show + 10)
        bat->next_scan_time = 30;
}

static void jz_battery_capacity_rising(struct jz_battery *bat) {
    jz_battery_next_scantime_rising(bat);

    if (bat->capacity_real > bat->capacity_show)
        if (bat->capacity_show != 100)
            bat->capacity_show++;
}

static void jz_battery_capacity_full(struct jz_battery *bat) {
    int old_status = bat->status;

    if (bat->capacity_show >= 99) {
        bat->capacity_show = 100;
        bat->status = POWER_SUPPLY_STATUS_FULL;
        bat->next_scan_time = 5 * 60;
    } else {
        bat->next_scan_time = 45;
        bat->capacity_show++;
        if (old_status == POWER_SUPPLY_STATUS_FULL) {
            bat->status = POWER_SUPPLY_STATUS_CHARGING;
            pr_info(LOG_TAG "status force changed from %s to %s,"
            " because cap show not 100!\n",
                    status2str(old_status), status2str(bat->status));
        }
    }
}

static void jz_battery_work(struct work_struct *work) {
    struct jz_battery *bat = container_of(work, struct jz_battery, work.work);
    int old_status = bat->status;
    struct timeval battery_time;

    if (bat->get_pmu_status == NULL) {
        bat->status = POWER_SUPPLY_STATUS_CHARGING;
        pr_err(
                LOG_TAG "Changer device not yet registered ! Force Charging !\n");
    } else {
        bat->status = get_status(bat);
    }

    if (old_status != bat->status)
        pr_info(LOG_TAG "old_status: %s\n", status2str(old_status));
    bat->voltage = get_adc_voltage(bat);
    bat->capacity_real = lookup_capacity(is_charging(bat->status),
            bat->voltage);

    switch (bat->status) {
    case POWER_SUPPLY_STATUS_CHARGING:
        jz_battery_capacity_rising(bat);
        break;
    case POWER_SUPPLY_STATUS_FULL:
        jz_battery_capacity_full(bat);
        break;
    case POWER_SUPPLY_STATUS_DISCHARGING:
    case POWER_SUPPLY_STATUS_NOT_CHARGING:
        jz_battery_capacity_falling(bat);
        break;
    case POWER_SUPPLY_STATUS_UNKNOWN:
        bat->next_scan_time = 60;
        break;
    }

    pr_info(LOG_TAG "%s, cap real: %d, cap show: %d, volt: %dmV, next %ds\n",
            status2str(bat->status), bat->capacity_real, bat->capacity_show, bat->voltage, bat->next_scan_time);

#ifdef CONFIG_SLPT
    slpt_set_battery_low_voltage(low_battery_voltage);
    slpt_set_battery_voltage(bat->voltage, bat->capacity_show);
    slpt_set_low_battery_warn_voltage(low_battery_warn_voltage);
#endif

    /* update last update time */
    do_gettimeofday(&battery_time);
    bat->last_update_time = battery_time.tv_sec;

    power_supply_changed(&bat->battery);
    schedule_delayed_work(&bat->work, bat->next_scan_time * HZ);
}

/* PM */
#ifndef CONFIG_SLPT
static void jz_battery_set_resume_time(struct jz_battery *bat) {
    struct timespec alarm_time;
    unsigned long interval = 60 * 60;

    getnstimeofday(&alarm_time);
    printk(LOG_TAG "time now is %ld\n", alarm_time.tv_sec);

    if (bat->pdata->info.sleep_current == 0) {
        interval = 60 * 60;

    } else if (bat->capacity_show > 10) {
        interval = ((bat->capacity_show - 10) * bat->pdata->info.battery_max_cpt
                * 36) / (bat->pdata->info.sleep_current);
    } else if (bat->capacity_show > 3) {
        interval = ((bat->capacity_show - 3) * bat->pdata->info.battery_max_cpt
                * 36) / (bat->pdata->info.sleep_current);
    } else if (bat->capacity_show > 1) {
        interval = ((bat->capacity_show - 1) * bat->pdata->info.battery_max_cpt
                * 36) / (bat->pdata->info.sleep_current);
    } else if (bat->capacity_show == 1) {
        interval = (bat->pdata->info.battery_max_cpt * 36)
                / bat->pdata->info.sleep_current;
    }

    if (interval == 0)
        interval = 60 * 60;

    alarm_time.tv_sec += interval;
    printk(LOG_TAG "set resume time %ld(delta %ld, %ldh %ldm %lds)\n",
            alarm_time.tv_sec, interval, interval / 3600, (interval / 60) % 60,
            interval % 60);
    alarm_start_range(&alarm, timespec_to_ktime(alarm_time),
            timespec_to_ktime(alarm_time));
}
#endif

static void jz_battery_resume_work(struct work_struct *work) {
    struct delayed_work *delayed_work =
            container_of(work, struct delayed_work, work);
    struct jz_battery *bat =
            container_of(delayed_work, struct jz_battery, resume_work);
    struct timeval battery_time;
    unsigned long time_delta;
    unsigned int next_scan_time;
    unsigned int battery_is_updated = 1;

    pr_info(LOG_TAG "%s\n", __FUNCTION__);

    if (bat->get_pmu_status == NULL) {
        bat->status = POWER_SUPPLY_STATUS_CHARGING;
        pr_err(
                LOG_TAG "Changer device not yet registered ! Force Charging !\n");
    } else {
        bat->status = get_status(bat);
    }
#ifdef CONFIG_SLPT
    bat->voltage = slpt_get_battery_voltage();
    if (bat->voltage <= low_battery_voltage && bat->status != POWER_SUPPLY_STATUS_CHARGING) {
        pr_info(LOG_TAG "SLPT: low capacity(%dmV), shutting down...\n", bat->voltage);
        bat->capacity_show = bat->capacity_real = 0;
        power_supply_changed(&bat->battery);
        while (1)
        msleep(1000);
    }
#else
    bat->voltage = get_adc_voltage(bat);
#endif

    /*
     * TODO: to create a table for SLPT capacity lookup specifically
     */
    bat->capacity_real = lookup_capacity(is_charging(bat->status),
            bat->voltage);

    do_gettimeofday(&battery_time);
    bat->resume_time = battery_time.tv_sec;
    pr_info(LOG_TAG "resume_time is %ld\n", battery_time.tv_sec);
    time_delta = bat->resume_time - bat->suspend_time;
    pr_info(LOG_TAG "time_delta is %ld, %ldh %ldm %lds\n",
            time_delta, time_delta/3600, (time_delta/60)%60, time_delta%60);

    next_scan_time = bat->next_scan_time;
    bat->next_scan_time = 15;

    if (bat->last_update_time == 0)
        bat->last_update_time = bat->resume_time;

    pr_info(LOG_TAG "suspend: %ld last:%ld  next_scan:%d delta:%ld\n",
            bat->suspend_time, bat->last_update_time, next_scan_time, bat->suspend_time - bat->last_update_time);

    if (bat->status == POWER_SUPPLY_STATUS_FULL) {
        bat->capacity_show = bat->capacity_real = 100;

    } else if (bat->status == POWER_SUPPLY_STATUS_CHARGING) {

        if ((bat->capacity_real > bat->capacity_show)
                && (time_delta > 10 * 60)) {
            bat->capacity_show = bat->capacity_real;
        } else if (time_delta > 3 * 60) {
            jz_battery_capacity_rising(bat);
        } else if ((bat->suspend_time - bat->last_update_time)
                > next_scan_time) {
            jz_battery_capacity_rising(bat);
        } else {
            jz_battery_next_scantime_rising(bat);
            battery_is_updated = 0;
        }

    } else { /* DISCHARGING || NOT_CHARGING */

        if ((bat->capacity_real < bat->capacity_show)
                && (time_delta > 10 * 60)) {
            bat->capacity_show = bat->capacity_real;
        } else if (time_delta > 3 * 60) {
            jz_battery_capacity_falling(bat);
        } else if ((bat->suspend_time - bat->last_update_time)
                > next_scan_time) {
            jz_battery_capacity_falling(bat);
        } else {
            jz_battery_next_scantime_falling(bat);
            battery_is_updated = 0;
        }
    }

    if (battery_is_updated) {
        bat->last_update_time = bat->resume_time;
        power_supply_changed(&bat->battery);
    }

    pr_info(LOG_TAG "%s, cap real: %d, cap show: %d, volt: %dmV, next %ds\n",
            status2str(bat->status), bat->capacity_real, bat->capacity_show, bat->voltage, bat->next_scan_time);

    schedule_delayed_work(&bat->work, bat->next_scan_time * HZ);

    external_cb_en = 1;

#ifndef CONFIG_SLPT
    jz_battery_set_resume_time(bat);
#endif
    wake_unlock(&bat->work_wake_lock);
}

#ifdef CONFIG_PM
static int jz_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct jz_battery *bat = platform_get_drvdata(pdev);
    struct timeval battery_time;
    pr_info(LOG_TAG "%s\n", __FUNCTION__);

    if (bat->get_pmu_status == NULL) {
        bat->status = POWER_SUPPLY_STATUS_CHARGING;
        pr_err(LOG_TAG "Changer device not yet registered ! Force Charging !\n");
    } else {
        bat->status = get_status(bat);
    }

    pr_info(LOG_TAG "%s, cap real: %d, cap show: %d, volt: %dmV\n",
            status2str(bat->status), bat->capacity_real,
            bat->capacity_show, bat->voltage);

    cancel_delayed_work_sync(&bat->resume_work);
    cancel_delayed_work_sync(&bat->work);

    do_gettimeofday(&battery_time);
    bat->suspend_time = battery_time.tv_sec;
    pr_info(LOG_TAG "suspend_time is %ld\n", battery_time.tv_sec);
    external_cb_en = 0;

    return 0;
}

static int jz_battery_resume(struct platform_device *pdev)
{
    struct jz_battery *bat = platform_get_drvdata(pdev);

    cancel_delayed_work(&bat->resume_work);
    wake_lock(&bat->work_wake_lock);
#ifdef CONFIG_SLPT
    schedule_delayed_work(&bat->resume_work, 0 * HZ);
#else
    schedule_delayed_work(&bat->resume_work, 1 * HZ);
#endif
    return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND

static void jz_battery_early_suspend(struct early_suspend *early_suspend)
{
#ifndef CONFIG_SLPT
    struct jz_battery *bat;
    pr_info(LOG_TAG "%s\n", __FUNCTION__);

    bat = container_of(early_suspend, struct jz_battery, early_suspend);

    jz_battery_set_resume_time(bat);
#endif
}

static void jz_battery_late_resume(struct early_suspend *early_suspend)
{
    ;
}
#endif

/* INIT */
#ifndef CONFIG_SLPT
static void wake_up_fun(struct alarm *alarm) {
    ;
}
#endif

static void jz_battery_init_work(struct work_struct *work) {
    struct delayed_work *delayed_work =
            container_of(work, struct delayed_work, work);
    struct jz_battery *bat =
            container_of(delayed_work, struct jz_battery, init_work);
    pr_info(LOG_TAG "%s\n", __FUNCTION__);

    if (bat->get_pmu_status == NULL) {
        bat->status = POWER_SUPPLY_STATUS_CHARGING;
        pr_err(
                LOG_TAG "Changer device not yet registered ! Force Charging !\n");
    } else {
        bat->status = get_status(bat);
    }

    bat->voltage = get_adc_voltage(bat);
    if (bat->status != POWER_SUPPLY_STATUS_FULL) {
        bat->capacity_real = lookup_capacity(is_charging(bat->status),
                bat->voltage);
        bat->capacity_show = bat->capacity_real;
    } else
        bat->capacity_real = bat->capacity_show = 100;

    pr_info(LOG_TAG "%s, cap real: %d, cap show: %d, volt: %dmV\n",
            status2str(bat->status), bat->capacity_real, bat->capacity_show, bat->voltage);

    power_supply_changed(&bat->battery);

    cancel_delayed_work_sync(&bat->work);
    schedule_delayed_work(&bat->work, 20 * HZ);
#ifndef CONFIG_SLPT
    alarm_init(&alarm, ANDROID_ALARM_RTC_WAKEUP, wake_up_fun);
#endif

    if (bat->pmu_work_enable != NULL) {
        bat->pmu_work_enable(bat->pmu_interface);
        external_cb_en = 1;
    }
}

static enum power_supply_property jz_battery_properties[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
    POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
    POWER_SUPPLY_PROP_PRESENT,
};

static int proc_set_capacity(struct file *file, const char *buf,
        unsigned long count, void *data) {
    struct jz_battery *bat = (struct jz_battery *) data;
    int cap;
    char str[10];
    str[9] = '\0';

    if (copy_from_user(str, buf, max(count, 8UL)))
        return -EFAULT;
    cap = simple_strtol(str, 0, 10);

    pr_info(LOG_TAG "Set capacity from %d to %d\n", bat->capacity_show, cap);

    if ((bat->status == POWER_SUPPLY_STATUS_FULL) && (cap <= 85))
        pr_info(LOG_TAG "skip because status is FULL\n");
    else if (abs(bat->capacity_real - cap) >= 35)
        pr_info(LOG_TAG "skip because abs >= 35\n");
    else
        bat->capacity_show = cap;

    cancel_delayed_work(&bat->work);
    schedule_delayed_work(&bat->work, 0);

    return count;
}

static int proc_read_status(char *buf, char **start, off_t off, int count,
        int *eof, void *data) {
    struct jz_battery *bat = (struct jz_battery *) data;

    return sprintf(buf,
            "%s, cap real: %d, cap show: %d, volt: %dmV, next: %ds\n",
            status2str(bat->status), bat->capacity_real, bat->capacity_show,
            bat->voltage, bat->next_scan_time);
}

static int jz_battery_get_property(struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val) {
    struct jz_battery *bat = psy_to_jz_battery(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        if ((POWER_SUPPLY_STATUS_FULL == bat->status)
                && (100 != bat->capacity_show))
            val->intval = POWER_SUPPLY_STATUS_CHARGING;
        else
            val->intval = bat->status;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        if ((bat->status == POWER_SUPPLY_STATUS_CHARGING)
                && (bat->capacity_show == 0)) {
            val->intval = 1;
        } else
            val->intval = bat->capacity_show;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = bat->voltage;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
        val->intval = jz_cv_discharging[100];
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
        val->intval = jz_cv_discharging[0];
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = 1;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static void jz_battery_external_power_changed(struct power_supply *psy) {
    struct jz_battery *bat = psy_to_jz_battery(psy);
    force_full = 0;

    /* enable after init_work & resume_work */
    if (!external_cb_en) {
        pr_info(LOG_TAG "ignore external_power_changed\n");
        return;
    }
    pr_info(LOG_TAG "external_power_changed\n");
    cancel_delayed_work(&bat->work);
    schedule_delayed_work(&bat->work, 0);
}

static const struct file_operations capacity_proc_fops = {
    .owner = THIS_MODULE,
    .write = proc_set_capacity,
};

static const struct file_operations status_proc_fops = {
    .owner = THIS_MODULE,
    .read = proc_read_status,
};

static int jz_battery_probe(struct platform_device *pdev) {
    int ret = 0;
    struct jz_battery *bat;
    struct power_supply *battery;
    struct proc_dir_entry *root;
    struct proc_dir_entry *res_capacity;
    struct proc_dir_entry *res_status;
    struct jz_battery_info *info;

    struct jz_battery_platform_data *pdata = kzalloc(
            sizeof(struct jz_battery_platform_data), GFP_KERNEL);
    if (!pdata) {
        dev_err(&pdev->dev, "Failed to allocate platform_data structre\n");
        return -ENOMEM;
    }

    bat = kzalloc(sizeof(*bat), GFP_KERNEL);
    if (!bat) {
        dev_err(&pdev->dev, "Failed to allocate driver structre\n");
        return -ENOMEM;
    }

    bat->cell = mfd_get_cell(pdev);
    info = bat->cell->platform_data;
    if (!info) {
        dev_err(&pdev->dev, "no cell data,cat attach\n");
        return -EINVAL;
    }
    pdata->info = *info;

    bat->ucharger = regulator_get(&pdev->dev, "ucharger");
    if (!bat->ucharger) {
        pr_info("Missing regulator ucharger\n");
    }

    bat->irq = platform_get_irq(pdev, 0);
    if (bat->irq < 0) {
        ret = bat->irq;
        dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
        goto err_free;
    }

    bat->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!bat->mem) {
        ret = -ENOENT;
        dev_err(&pdev->dev, "Failed to get platform mmio resource\n");
        goto err_free;
    }

    bat->mem =
            request_mem_region(bat->mem->start, resource_size(bat->mem), pdev->name);
    if (!bat->mem) {
        ret = -EBUSY;
        dev_err(&pdev->dev, "Failed to request mmio memory region\n");
        goto err_free;
    }

    bat->base = ioremap_nocache(bat->mem->start, resource_size(bat->mem));
    if (!bat->base) {
        ret = -EBUSY;
        dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
        goto err_release_mem_region;
    }

    get_slop_cut();

    battery = &bat->battery;
    battery->name = "battery";
    battery->type = POWER_SUPPLY_TYPE_BATTERY;
    battery->properties = jz_battery_properties;
    battery->num_properties = ARRAY_SIZE(jz_battery_properties);
    battery->get_property = jz_battery_get_property;
    battery->external_power_changed = jz_battery_external_power_changed;
    battery->use_for_apm = 1;

    bat->pdata = pdata;
    bat->pdev = pdev;

#ifdef CONFIG_HAS_EARLYSUSPEND
    bat->early_suspend.suspend = jz_battery_early_suspend;
    bat->early_suspend.resume = jz_battery_late_resume;
    bat->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    register_early_suspend(&bat->early_suspend);
#endif
    init_completion(&bat->read_completion);
    mutex_init(&bat->lock);

    INIT_DELAYED_WORK(&bat->work, jz_battery_work);
    INIT_DELAYED_WORK(&bat->init_work, jz_battery_init_work);
    INIT_DELAYED_WORK(&bat->resume_work, jz_battery_resume_work);
    wake_lock_init(&bat->work_wake_lock, WAKE_LOCK_SUSPEND, "jz_battery");

    ret = request_irq(bat->irq, adc_irq_handler, 0, pdev->name, bat);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request irq %d\n", ret);
        goto err_iounmap;
    }
    disable_irq(bat->irq);

    ret = power_supply_register(&pdev->dev, &bat->battery);
    if (ret) {
        dev_err(&pdev->dev, "Power supply battery register failed.\n");
        goto err_iounmap;
    }

    platform_set_drvdata(pdev, bat);

    if (pdata->info.usb_chg_current) {
        bat->usb_charge_time = pdata->info.battery_max_cpt * 36
                / pdata->info.usb_chg_current;
    }

    if (bat->usb_charge_time < 90)
        bat->usb_charge_time = 90;

    bat->next_scan_time = 15;
    /* init_work init function is only called once,
     * 5 seconds delay waiting Charger Device registration
     */
    schedule_delayed_work(&bat->init_work, 5 * HZ);

    root = proc_mkdir("power_supply", 0);

    res_capacity =
            proc_create_data("capacity", 0200, root, &capacity_proc_fops, bat);
    if (!res_capacity) {
        dev_err(&pdev->dev, "Failed to Create proc capacity\n");
        goto err_proc_capacity;
    }

    res_status = proc_create_data("status", 0444, root, &status_proc_fops, bat);
    if (!res_status) {
        dev_err(&pdev->dev, "Failed to Create proc status\n");
        goto err_proc_status;
    }

    pr_debug(LOG_TAG "discharging max voltage is %d\n", jz_cv_discharging[100]);
    pr_debug(LOG_TAG "discharging min voltage is %d\n", jz_cv_discharging[0]);
    pr_debug(LOG_TAG "charging max voltage is %d\n", jz_cv_charging[100]);
    pr_debug(LOG_TAG "charging min voltage is %d\n", jz_cv_charging[0]);
    pr_debug(LOG_TAG "battery_max_cpt is %d\n", pdata->info.battery_max_cpt);
    pr_debug(LOG_TAG "usb_chg_current is %d\n", pdata->info.usb_chg_current);
    pr_debug(LOG_TAG "sleep_current is %d\n", pdata->info.sleep_current);

    pr_info(LOG_TAG "registers driver over!\n");

    return 0;

err_proc_status:
    proc_remove(res_capacity);

err_proc_capacity:
err_iounmap:
#ifndef CONFIG_SLPT
    wake_lock_destroy(&bat->work_wake_lock);
#endif
    platform_set_drvdata(pdev, NULL);
    iounmap(bat->base);

err_release_mem_region:
    release_mem_region(bat->mem->start, resource_size(bat->mem));

err_free:
    if (bat->ucharger)
        regulator_put(bat->ucharger);
    kfree(bat);
    return ret;
}

static int jz_battery_remove(struct platform_device *pdev) {
    struct jz_battery *bat = platform_get_drvdata(pdev);

    cancel_delayed_work_sync(&bat->work);
    cancel_delayed_work_sync(&bat->resume_work);
#ifndef CONFIG_SLPT
    wake_lock_destroy(&bat->work_wake_lock);
#endif
    power_supply_unregister(&bat->battery);

    free_irq(bat->irq, bat);

    iounmap(bat->base);
    release_mem_region(bat->mem->start, resource_size(bat->mem));
    kfree(bat);

    return 0;
}

static struct platform_driver jz_battery_driver = {
    .probe = jz_battery_probe,
    .remove = jz_battery_remove,
    .driver = {
        .name = "jz-battery",
        .owner = THIS_MODULE,
    },
    .suspend = jz_battery_suspend,
    .resume = jz_battery_resume,
};

#ifndef CONFIG_BATTERY_DS2782
static
#endif
int jz_battery_init(void) {
    return platform_driver_register(&jz_battery_driver);
}
#ifndef CONFIG_BATTERY_DS2782
module_init(jz_battery_init);
#endif

static void jz_battery_exit(void) {
    platform_driver_unregister(&jz_battery_driver);
}
module_exit(jz_battery_exit);

MODULE_ALIAS("platform:jz4775-battery");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sun Jiwei<jwsun@ingenic.cn>");
MODULE_DESCRIPTION("JZ4780 SoC battery driver");
