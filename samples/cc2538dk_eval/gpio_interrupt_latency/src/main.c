#include <zephyr.h>
#include <misc/printk.h>
#include <string.h>
#include <device.h>
#include <gpio.h>
#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sys_ctrl.h>

#define LED_PIN (0)
#define BTN_PIN (3)

struct device * led_gpio_dev = NULL;
struct device * btn_gpio_dev = NULL;
struct gpio_callback btn_callbacks;
void btn_handler(struct device *port, struct gpio_callback *cb, u32_t pins);

void init_gpio(void) {
	led_gpio_dev = device_get_binding("GPIO_PORTC");
	btn_gpio_dev = device_get_binding("GPIO_PORTA");
	gpio_pin_configure(led_gpio_dev, LED_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(btn_gpio_dev, BTN_PIN, GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE | GPIO_PUD_PULL_UP);
	gpio_init_callback(&btn_callbacks, btn_handler, (1<<BTN_PIN));
	gpio_add_callback(btn_gpio_dev, &btn_callbacks);
	gpio_pin_enable_callback(btn_gpio_dev, BTN_PIN);
}

void main(void) {
	init_gpio();

	while (1) {
		//nothing
		k_sleep(1 * 1000);
	}
}

int cnt = 0;
void btn_handler(struct device *port, struct gpio_callback *cb, u32_t pins) {
		gpio_pin_write(led_gpio_dev, LED_PIN, (++cnt)%2);
}
