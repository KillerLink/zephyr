#include <zephyr.h>
#include <misc/printk.h>
#include <string.h>
#include <device.h>
#include <gpio.h>
#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sys_ctrl.h>

#define LED_PIN (0)

struct device * led_gpio_dev = NULL;

void init_gpio(void) {
	led_gpio_dev = device_get_binding("GPIO_PORTC");
	gpio_pin_configure(led_gpio_dev, LED_PIN, GPIO_DIR_OUT);
}

void main(void) {
	init_gpio();

	while (1) {
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
		gpio_pin_write(led_gpio_dev, LED_PIN, 0);
		gpio_pin_write(led_gpio_dev, LED_PIN, 1);
	}
}
