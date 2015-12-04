#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/errno.h>

#include <linux/slpt.h>

#ifdef CONFIG_BATTERY_RICOH619
#define SAMPLE_ADC_FOR_KERNEL 1
#else
#define SAMPLE_ADC_FOR_KERNEL 0
#endif

#undef KEY
#define KEY(index, n, v) [index] = {.id = index, .name = n, .val = (unsigned long)(v)}

struct slpt_key slpt_key_list[SLPT_K_NUMS] = {
	KEY(SLPT_K_POWER_STATE, "power-state", ""),
	KEY(SLPT_K_CHARGER_GPIO, "charger-gpio", -1),
	KEY(SLPT_K_CHARGER_LEVEL, "charger-level", -1),
	KEY(SLPT_K_LOW_BATTERY_WARN_VOL, "low-bat-warn-vol", CONFIG_SLPT_LOW_BATTERY_WARN_VOLTAGE),
	KEY(SLPT_K_BATTERY_VOLTAGE, "battery-voltage", 0),
	KEY(SLPT_K_BATTERY_LOW_VOLTAGE, "battery-low-voltage", CONFIG_SLPT_LOW_BATTERY_VOLTAGE),
	KEY(SLPT_K_BATTERY_CAPACITY, "battery-capacity", 100),
#ifdef CONFIG_SLPT_MAP_TO_KSEG2
	KEY(SLPT_K_RESERVE_MEM_ADDR, "reserve-mem-addr", slpt_reserve_mem),
#else
	KEY(SLPT_K_RESERVE_MEM_ADDR, "reserve-mem-addr", SLPT_RESERVE_ADDR),
#endif

	KEY(SLPT_K_GO_KERNEL, "go-kernel", 1),
	KEY(SLPT_K_VOICE_TRIGGER_STATE, "voice-trigger-state", 0),
	KEY(SLPT_K_FRIZZ_PEDO, "frizz_pedo", 0),
	KEY(SLPT_K_FRIZZ_GESTURE, "frizz_gesture", 0),
	KEY(SLPT_K_SLEEP_MOTION, "sleep_motion", 0),
	KEY(SLPT_K_SLEEP_MOTION_ENABLE, "sleep_motion_enable", 0),
	KEY(SLPT_K_IOCTL, "ioctl", NULL),
	KEY(SLPT_K_FB_ON, "fb_on", 0),
	KEY(SLPT_K_SAMPLE_ADC_FOR_KERNEL, "sample_adc_for_kernel", SAMPLE_ADC_FOR_KERNEL),
};

static inline int slpt_find_key_id_internal(const char *name) {
	int i;

	for (i = 0; i < SLPT_K_NUMS; ++i) {
		if (!strcmp(slpt_key_list[i].name, name))
			return i;
	}

	return -ENODEV;
}

int slpt_get_key_id(const char *name, unsigned long **val_addr) {
	int id;

	if (!name) {
		return -EINVAL;
	}

	id = slpt_find_key_id_internal(name);
	if (id < 0)
		return -ENODEV;

	if (val_addr)
		*val_addr = &slpt_key_list[id].val;

	return id;
}
EXPORT_SYMBOL(slpt_get_key_id);

int slpt_set_key_by_id(unsigned int id, unsigned long val) {
	BUG_ON(!(id < SLPT_K_NUMS));

	slpt_key_list[id].val = val;

	return 0;
}
EXPORT_SYMBOL(slpt_set_key_by_id);

int slpt_get_key_by_id(unsigned int id, unsigned long *val) {
	BUG_ON(!(id < SLPT_K_NUMS));

	*val = slpt_key_list[id].val;

	return 0;
}
EXPORT_SYMBOL(slpt_get_key_by_id);

static inline struct slpt_key *slpt_find_key_interal(const char *name) {
	int i;

	for (i = 0; i < SLPT_K_NUMS; ++i) {
		if (!strcmp(slpt_key_list[i].name, name))
			return &slpt_key_list[i];
	}

	return NULL;
}

struct slpt_key *slpt_find_key(const char *name) {
	return name ? slpt_find_key_interal(name) : NULL;
}
EXPORT_SYMBOL(slpt_find_key);

int slpt_set_key_by_name(const char *name, unsigned long val) {
	struct slpt_key *key;

	if (!name) {
		return -EINVAL;
	}

	key = slpt_find_key_interal(name);
	if (!key) {
		return -ENODEV;
	}

	key->val = val;

	return 0;
}
EXPORT_SYMBOL(slpt_set_key_by_name);

int slpt_get_key_by_name(const char *name, unsigned long *val) {
	struct slpt_key *key;

	if (!name) {
		return -EINVAL;
	}

	key = slpt_find_key_interal(name);
	if (!key) {
		return -ENODEV;
	}

	*val = key->val;

	return 0;
}
EXPORT_SYMBOL(slpt_get_key_by_name);
