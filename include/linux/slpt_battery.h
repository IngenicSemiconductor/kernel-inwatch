#ifndef _SLPT_BATTERY_H_
#define _SLPT_BATTERY_H_

#include <linux/slpt.h>

static inline void slpt_set_battery_voltage(unsigned int voltage, unsigned int capacity) {
	SLPT_SET_KEY(SLPT_K_BATTERY_VOLTAGE, voltage);
	SLPT_SET_KEY(SLPT_K_BATTERY_CAPACITY, capacity);
}

static inline unsigned int slpt_get_battery_voltage(void) {
	unsigned int voltage;

	SLPT_GET_KEY(SLPT_K_BATTERY_VOLTAGE, &voltage);
	return voltage;
}

static inline unsigned int slpt_get_battery_capacity(void) {
	unsigned int capacity;

	SLPT_GET_KEY(SLPT_K_BATTERY_CAPACITY, &capacity);
	return capacity;
}

static inline void slpt_set_battery_low_voltage(unsigned int voltage) {
	SLPT_SET_KEY(SLPT_K_BATTERY_LOW_VOLTAGE, voltage);
}

static inline void slpt_set_low_battery_warn_voltage(unsigned int voltage) {
	SLPT_SET_KEY(SLPT_K_LOW_BATTERY_WARN_VOL, voltage);
}

#endif /* _SLPT_BATTERY_H_ */
