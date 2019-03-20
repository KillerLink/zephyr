#include <zephyr.h>
#include <misc/printk.h>
#include <string.h>
#include <device.h>
#include <gpio.h>
#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sys_ctrl.h>
#include "main.h"
#define SECONDS_TO_SLEEP (1)

struct device * led_gpio_dev = NULL;
struct device * btn_gpio_dev = NULL;
struct gpio_callback btn_callbacks;
void btn_handler(struct device *port, struct gpio_callback *cb, u32_t pins);

void init_gpio(void) {
	led_gpio_dev = device_get_binding(PORT_LEDS);
	btn_gpio_dev = device_get_binding(PORT_BTNS);
	gpio_pin_configure(led_gpio_dev, PIN_LED1, GPIO_DIR_OUT);
	gpio_pin_configure(led_gpio_dev, PIN_LED2, GPIO_DIR_OUT);
	gpio_pin_configure(led_gpio_dev, PIN_LED3, GPIO_DIR_OUT);
	gpio_pin_configure(led_gpio_dev, PIN_LED4, GPIO_DIR_OUT);
	gpio_pin_configure(btn_gpio_dev, PIN_BTN1, GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP);
	gpio_pin_configure(btn_gpio_dev, PIN_BTN2, GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP);
	gpio_pin_configure(btn_gpio_dev, PIN_BTN3, GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP);
	gpio_pin_configure(btn_gpio_dev, PIN_BTN4, GPIO_DIR_IN);
	gpio_init_callback(&btn_callbacks, btn_handler, MASK_BTN1 | MASK_BTN2 | MASK_BTN3);
	int x2 = gpio_add_callback(btn_gpio_dev, &btn_callbacks);
	int x3 = gpio_pin_enable_callback(btn_gpio_dev, PIN_BTN1);
	int x4 = gpio_pin_enable_callback(btn_gpio_dev, PIN_BTN2);
	int x5 = gpio_pin_enable_callback(btn_gpio_dev, PIN_BTN3);
	printk("btn callback setup: %d %d %d %d\n", x2, x3, x4, x5);
}

/* Application main Thread */
void main(void) {
	init_gpio();
	printk("main()\n");
	printk("LED1 on\n");
	gpio_pin_write(led_gpio_dev, PIN_LED1, 1);

	while (1) {
		printk("LED2 on\n");
		gpio_pin_write(led_gpio_dev, PIN_LED2, 1);
		k_sleep(SECONDS_TO_SLEEP * 1000);

		printk("LED2 off\n");
		gpio_pin_write(led_gpio_dev, PIN_LED2, 0);
		k_sleep(SECONDS_TO_SLEEP * 1000);

		printk("LED3 toggle\n");
		uint32_t l4 = 0;
		gpio_pin_read(led_gpio_dev, PIN_LED3, &l4);
		gpio_pin_write(led_gpio_dev, PIN_LED3, l4==0?1:0);
		k_sleep(SECONDS_TO_SLEEP * 1000);
	}
}

void btn_handler(struct device *port, struct gpio_callback *cb, u32_t pins) {
	printk("[Interrupt]\n");
	if (pins & MASK_BTN1) {
		gpio_pin_write(led_gpio_dev, PIN_LED4, 1);
	}
	if (pins & MASK_BTN2) {
		gpio_pin_write(led_gpio_dev, PIN_LED4, 0);
	}
	if (pins & MASK_BTN3) {
		printk("[BTN3 Interrupt!]\n");
	}
}
