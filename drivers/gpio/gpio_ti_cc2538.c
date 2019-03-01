#include <errno.h>

#include <device.h>
#include <gpio.h>
#include <init.h>
#include <kernel.h>
#include <sys_io.h>

/* Driverlib includes */
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ioc.h>
#include <driverlib/rom.h>
#undef __GPIO_H__  /* Zephyr and CC2538SDK gpio.h conflict */
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/ioc.h>

#include "gpio_utils.h"

struct gpio_ti_cc2538_config_t {
	const unsigned long port_base; // base address of GPIO port
	const unsigned long irq_num; 	// GPIO IRQ number
	const unsigned long irq_pri;
};

struct gpio_ti_cc2538_data_t {
	sys_slist_t callbacks; //list of registered callbacks
	u32_t pin_callback_enables; //callback enable pin bitmask
};

#define DEV_CFG(dev) ((const struct gpio_ti_cc2538_config_t *)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct gpio_ti_cc2538_data_t *)(dev)->driver_data)

static inline int gpio_ti_cc2538_config(struct device *port, int access_op, u32_t pin, int flags) {
	const struct gpio_ti_cc2538_config_t *gpio_config = DEV_CFG(port);
	unsigned long port_base = gpio_config->port_base;
	unsigned long int_type;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	/* runtime interrupt type config */
	if (flags & GPIO_INT) {
		if (flags & GPIO_INT_EDGE) {
			if (flags & GPIO_INT_ACTIVE_HIGH) {
				int_type = GPIO_RISING_EDGE;
			} else if (flags & GPIO_INT_DOUBLE_EDGE) {
				int_type = GPIO_BOTH_EDGES;
			} else {
				int_type = GPIO_FALLING_EDGE;
			}
		} else { /* GPIO_INT_LEVEL */
			if (flags & GPIO_INT_ACTIVE_HIGH) {
				int_type = GPIO_HIGH_LEVEL;
			} else {
				int_type = GPIO_LOW_LEVEL;
			}
		}
		GPIOIntTypeSet(port_base, (1<<pin), int_type);
		GPIOPinIntClear(port_base, (1<<pin));
		GPIOPinIntEnable(port_base, (1<<pin));
	} else {
		GPIOPinIntDisable(port_base, (1<<pin));
	}

	//TODO: ? evaluate programmed behaviour to be desirable
	//Since there is no different flag for "make input" and "make output"
	//the target must be assumed. Therefore, this will also clear
	//alternate function settings, such that this function indeed can be used
	//to setup a GPIO pin as expected. Sadly this has ugly side effects such as:
	//-> Unexpected Change to Input (because GPIO_DIR_OUT was not explicitly set)
	//-> Removal of Alternate Pinfunction (because pin mode assumed to be either GPIO_IN or GPIO_OUT)
	if (flags & GPIO_DIR_OUT) {
		GPIOPinTypeGPIOOutput(port_base, (1<<pin));
	} else {
		GPIOPinTypeGPIOInput(port_base, (1<<pin));
	}

	//Pull Up/Down Control
	if ( (flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL ) {
		IOCPadConfigSet(port_base, (1<<pin), IOC_OVERRIDE_DIS);
	} else if ( (flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP ) {
		IOCPadConfigSet(port_base, (1<<pin), IOC_OVERRIDE_PUE);
	} else if ( (flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN ) {
		IOCPadConfigSet(port_base, (1<<pin), IOC_OVERRIDE_PDE);
	}

	return 0;
}

static inline int gpio_ti_cc2538_write(struct device *port, int access_op, u32_t pin, u32_t value) {
	const struct gpio_ti_cc2538_config_t *gpio_config = DEV_CFG(port);
	unsigned long port_base = gpio_config->port_base;

	if (access_op == GPIO_ACCESS_BY_PIN) {
		value = value << pin;
		pin = 1 << pin;
		GPIOPinWrite(port_base, (unsigned char)pin, value);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static inline int gpio_ti_cc2538_read(struct device *port, int access_op, u32_t pin, u32_t *value) {
	const struct gpio_ti_cc2538_config_t *gpio_config = DEV_CFG(port);
	unsigned long port_base = gpio_config->port_base;
	long status;
	unsigned char pin_packed;

	if (access_op == GPIO_ACCESS_BY_PIN) {
		pin_packed = 1 << pin;
		status =  GPIOPinRead(port_base, pin_packed);
		*value = status >> pin;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_ti_cc2538_manage_callback(struct device *dev, struct gpio_callback *callback, bool set) {
	struct gpio_ti_cc2538_data_t *data = DEV_DATA(dev);
	_gpio_manage_callback(&data->callbacks, callback, set);
	return 0;
}


static int gpio_ti_cc2538_enable_callback(struct device *dev, int access_op, u32_t pin) {
	struct gpio_ti_cc2538_data_t *data = DEV_DATA(dev);
	if (access_op == GPIO_ACCESS_BY_PIN) {
		data->pin_callback_enables |= (1 << pin);
	} else {
		data->pin_callback_enables = 0xFFFFFFFF;
	}
	return 0;
}


static int gpio_ti_cc2538_disable_callback(struct device *dev, int access_op, u32_t pin) {
	struct gpio_ti_cc2538_data_t *data = DEV_DATA(dev);
	if (access_op == GPIO_ACCESS_BY_PIN) {
		data->pin_callback_enables &= ~(1 << pin);
	} else {
		data->pin_callback_enables = 0;
	}
	return 0;
}

static void gpio_cc2538_port_isr(void *arg) {
	struct device *dev = arg;
	const struct gpio_ti_cc2538_config_t *config = DEV_CFG(dev);
	struct gpio_ti_cc2538_data_t *data = DEV_DATA(dev);

	// See which interrupts triggered
	u32_t int_status  = (u32_t)GPIOPinIntStatus(config->port_base, 1);
	u32_t enabled_int = int_status & data->pin_callback_enables;

	// Clear and Disable GPIO Interrupt
	GPIOPinIntDisable(config->port_base, enabled_int);
	GPIOPinIntClear(config->port_base, enabled_int);

	// Call the registered callbacks
	_gpio_fire_callbacks(&data->callbacks, (struct device *)dev, enabled_int);

	// Re-enable the interrupts
	GPIOPinIntEnable(config->port_base, enabled_int);
}

static const struct gpio_driver_api api_funcs = {
	.config = gpio_ti_cc2538_config,
	.write = gpio_ti_cc2538_write,
	.read = gpio_ti_cc2538_read,
	.manage_callback = gpio_ti_cc2538_manage_callback,
	.enable_callback = gpio_ti_cc2538_enable_callback,
	.disable_callback = gpio_ti_cc2538_disable_callback,

};


//=============================================================================================
//==== PORTA ==================================================================================
//=============================================================================================
#ifdef CONFIG_GPIO_TI_CC2538_PORTA

static struct device DEVICE_NAME_GET(gpio_ti_cc2538_porta_dev);
static struct gpio_ti_cc2538_data_t gpio_ti_cc2538_data_porta;

static const struct gpio_ti_cc2538_config_t gpio_ti_cc2538_config_porta = {
	.port_base = PORTA_BASE_ADDRESS,
	.irq_num = PORTA_IRQ,
	.irq_pri = PORTA_IRQ_PRIORITY
};

static int gpio_ti_cc2538_init(struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(PORTA_IRQ, PORTA_IRQ_PRIORITY,
		gpio_cc2538_port_isr, DEVICE_GET(gpio_ti_cc2538_porta_dev),	0);
	irq_enable(PORTA_IRQ);
	return 0;
}

DEVICE_AND_API_INIT(
	gpio_ti_cc2538_porta_dev,
	PORTA_LABEL,
	gpio_ti_cc2538_init,
	&gpio_ti_cc2538_data_porta,
	&gpio_ti_cc2538_config_porta,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api_funcs);

#endif /* CONFIG_GPIO_TI_CC2538_PORTA */


//=============================================================================================
//==== PORTB ==================================================================================
//=============================================================================================
#ifdef CONFIG_GPIO_TI_CC2538_PORTB

static struct device DEVICE_NAME_GET(gpio_ti_cc2538_portb_dev);
static struct gpio_ti_cc2538_data_t gpio_ti_cc2538_data_portb;

static const struct gpio_ti_cc2538_config_t gpio_ti_cc2538_config_portb = {
	.port_base = PORTB_BASE_ADDRESS,
	.irq_num = PORTB_IRQ,
	.irq_pri = PORTB_IRQ_PRIORITY
};

static int gpio_ti_cc2538_init_portb(struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(PORTB_IRQ, PORTB_IRQ_PRIORITY,
		gpio_cc2538_port_isr, DEVICE_GET(gpio_ti_cc2538_portb_dev),	0);
	irq_enable(PORTB_IRQ);
	return 0;
}

DEVICE_AND_API_INIT(
	gpio_ti_cc2538_portb_dev,
	PORTB_LABEL,
	gpio_ti_cc2538_init_portb,
	&gpio_ti_cc2538_data_portb,
	&gpio_ti_cc2538_config_portb,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api_funcs);

#endif /* CONFIG_GPIO_TI_CC2538_PORTB */


//=============================================================================================
//==== PORTC ==================================================================================
//=============================================================================================
#ifdef CONFIG_GPIO_TI_CC2538_PORTC

static struct device DEVICE_NAME_GET(gpio_ti_cc2538_portc_dev);
static struct gpio_ti_cc2538_data_t gpio_ti_cc2538_data_portc;

static const struct gpio_ti_cc2538_config_t gpio_ti_cc2538_config_portc = {
	.port_base = PORTC_BASE_ADDRESS,
	.irq_num = PORTC_IRQ,
	.irq_pri = PORTC_IRQ_PRIORITY
};

static int gpio_ti_cc2538_init_portc(struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(PORTC_IRQ, PORTC_IRQ_PRIORITY,
		gpio_cc2538_port_isr, DEVICE_GET(gpio_ti_cc2538_portc_dev),	0);
	irq_enable(PORTC_IRQ);
	return 0;
}

DEVICE_AND_API_INIT(
	gpio_ti_cc2538_portc_dev,
	PORTC_LABEL,
	gpio_ti_cc2538_init_portc,
	&gpio_ti_cc2538_data_portc,
	&gpio_ti_cc2538_config_portc,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api_funcs);

#endif /* CONFIG_GPIO_TI_CC2538_PORTC */


//=============================================================================================
//==== PORTD ==================================================================================
//=============================================================================================
#ifdef CONFIG_GPIO_TI_CC2538_PORTD

static struct device DEVICE_NAME_GET(gpio_ti_cc2538_portd_dev);
static struct gpio_ti_cc2538_data_t gpio_ti_cc2538_data_portd;

static const struct gpio_ti_cc2538_config_t gpio_ti_cc2538_config_portd = {
	.port_base = PORTD_BASE_ADDRESS,
	.irq_num = PORTD_IRQ,
	.irq_pri = PORTD_IRQ_PRIORITY
};

static int gpio_ti_cc2538_init_portd(struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(PORTD_IRQ, PORTD_IRQ_PRIORITY,
		gpio_cc2538_port_isr, DEVICE_GET(gpio_ti_cc2538_portd_dev),	0);
	irq_enable(PORTD_IRQ);
	return 0;
}

DEVICE_AND_API_INIT(
	gpio_ti_cc2538_portd_dev,
	PORTD_LABEL,
	gpio_ti_cc2538_init_portd,
	&gpio_ti_cc2538_data_portd,
	&gpio_ti_cc2538_config_portd,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api_funcs);

#endif /* CONFIG_GPIO_TI_CC2538_PORTD */
