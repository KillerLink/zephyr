// (Created by [TheM])

/**
 * @file
 * @brief System/hardware module for ti_cc2538 platform
 *
 * This module provides routines to initialize and support
 * board-level hardware for the ti_lm3s6965 platform.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <arch/cpu.h>

#include <driverlib/sys_ctrl.h>
#include <driverlib/uart.h>
#include <driverlib/ioc.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sys_ctrl.h>
#include <logging/sys_log.h>


/**
 *
 * @brief Perform basic hardware initialization
 *
 * @return 0
 */

static int ti_cc2538_init(struct device *arg)
{
	ARG_UNUSED(arg);

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* Configure the system clock, io clock, power mode */
	//TODO: make those available via kconfig
	uint8_t bExternalOsc32kHz = 0; //use external 32kHz oscillator? -> no
	uint8_t bInternalOsc = 0; //use internal 1-16MHz oscillator or the external 0-32MHz oscillator? -> external
	uint32_t ui32SysDiv = SYS_CTRL_SYSDIV_32MHZ; //IF internal RC oscillator is used, this selects its frequency
	SysCtrlClockSet(bExternalOsc32kHz, bInternalOsc, ui32SysDiv);
	//The IO Clock should be configured according to the used oscillator
	SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
	IntMasterEnable();

	return 0;
}

SYS_INIT(ti_cc2538_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
