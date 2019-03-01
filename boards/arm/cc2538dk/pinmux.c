// (Created by [TheM])

#include <init.h>

#include "pinmux.h"

#include <driverlib/gpio.h>
#include <driverlib/ioc.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>

int cc2538dk_pinmux_initialize(struct device *port)
{
	ARG_UNUSED(port);

#ifdef CONFIG_GPIO_TI_CC2538_PORTA
	/* Enable Peripheral Clocks */
	//MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);

	/* The following enables the 3 LEDs for the blinking samples */

	/* Configure PIN_64 for GPIOOutput */
	//MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
	//MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

	/* Configure PIN_01 for GPIOOutput */
	//MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
	//MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);

	/* Configure PIN_02 for GPIOOutput */
	//MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
	//MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);

	/* SW3: Configure PIN_04 (GPIO13) for GPIOInput */
	//MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
	//MAP_GPIODirModeSet(GPIOA1_BASE, 0x20, GPIO_DIR_MODE_IN);
#endif

#ifdef CONFIG_GPIO_TI_CC2538_PORTB
	//MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);

	/* SW2: Configure PIN_15 (GPIO22) for GPIOInput */
	//MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
	//MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_IN);
#endif

#ifdef CONFIG_GPIO_TI_CC2538_PORTC
	//MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
#endif

#ifdef CONFIG_GPIO_TI_CC2538_PORTD
	//MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
#endif

	return 0;
}

SYS_INIT(cc2538dk_pinmux_initialize, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
