#ifndef __GPIO_H__
#define __GPIO_H__

void gpio_set_value(int port, int pin, int value);
int gpio_get_value(int port, int pin);
void gpio_direction_input(int port, int pin);
void gpio_direction_output(int port, int pin, int value);

#endif
