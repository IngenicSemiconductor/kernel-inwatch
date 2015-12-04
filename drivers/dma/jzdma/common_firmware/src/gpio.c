#include <asm/mach/jz4785-base.h>
#include <asm/mach/gpio.h>

#define writel(value, addr)    (*(volatile unsigned int *) (addr) = (value))
#define readl( addr)    (*(volatile unsigned int *) (addr))

void gpio_set_value(int port, int pin, int value)
{
	if (value)
		writel(1 << pin, GPIO_PXPAT0S(port));
	else
		writel(1 << pin, GPIO_PXPAT0C(port));
}

int gpio_get_value(int port, int pin)
{
	
	return (readl(GPIO_PXPIN(port)) & (1 << pin)) ? 1 : 0;
}

void gpio_direction_input(int port, int pin)
{
	writel(1 << pin, GPIO_PXINTC(port));
	writel(1 << pin, GPIO_PXMASKS(port));
	writel(1 << pin, GPIO_PXPAT1S(port));
}

void gpio_direction_output(int port, int pin, int value)
{
	writel(1 << pin, GPIO_PXINTC(port));
	writel(1 << pin, GPIO_PXMASKS(port));
	writel(1 << pin, GPIO_PXPAT1C(port));

	gpio_set_value(port, pin, value);
}
