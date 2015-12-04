#include <linux/module.h>
#include <linux/slpt_battery.h>

static struct slpt_battery slpt_battery;

/* for kernel driver */
void slpt_set_battery_struct(struct slpt_battery *battery) {
	if (!battery) {
		slpt_battery = *battery;
	}
}
EXPORT_SYMBOL(slpt_set_battery_struct);

void slpt_set_battery_voltage(unsigned int voltage, unsigned int capacity) {
	slpt_battery.voltage = voltage;
	slpt_battery.capacity = capacity;
}
EXPORT_SYMBOL(slpt_set_battery_voltage);

unsigned int slpt_get_battery_voltage() {
	return slpt_battery.voltage;
}
EXPORT_SYMBOL(slpt_get_battery_voltage);

unsigned int slpt_get_battery_capacity() {
	return slpt_battery.capacity;
}
EXPORT_SYMBOL(slpt_get_battery_capacity);

void slpt_set_battery_low_voltage(unsigned int voltage) {
	slpt_battery.low_battery_voltage = voltage;
}
EXPORT_SYMBOL(slpt_set_battery_low_voltage);

/* for slpt app */
void slpt_app_set_battery_struct(void *data) {
	struct slpt_battery	*battery = (struct slpt_battery	*)data;

	if (!battery) {
		slpt_battery = *battery;
	}
}
EXPORT_SYMBOL(slpt_app_set_battery_struct);

void slpt_app_set_battery_voltage(unsigned int voltage) {
	slpt_battery.voltage = voltage;
}
EXPORT_SYMBOL(slpt_app_set_battery_voltage);

int slpt_app_get_battery_low_voltage(void) {
	return slpt_battery.low_battery_voltage;
}
EXPORT_SYMBOL(slpt_app_get_battery_low_voltage);
