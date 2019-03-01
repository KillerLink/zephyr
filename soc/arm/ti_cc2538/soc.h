// (Created by [TheM])
/**
 * @file
 * @brief Board configuration macros for the QEMU for arm platform
 *
 */

#ifndef _BOARD__H_
#define _BOARD__H_

#include <misc/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* default system clock */

#define SYSCLK_DEFAULT_IOSC_HZ MHZ(16)

/* IRQs */
//defined as expected by zephyr which are offset by 16 to the 
//real interrupt numbers from the datasheet
//TODO: implement configurable alternate map
#define IRQ_GPIO_PORTA 0
#define IRQ_GPIO_PORTB 1
#define IRQ_GPIO_PORTC 2
#define IRQ_GPIO_PORTD 3
//4 to 4 not in use
#define IRQ_UART0 5
#define IRQ_UART1 6
#define IRQ_SSI0 7
#define IRQ_I2C0 8
//9 to 13 not in use
#define IRQ_ADC0 14
//15 to 17 not in use
#define IRQ_WATCHDOG 18
#define IRQ_WATCHDOG0 18
#define IRQ_TIMER0A 19
#define IRQ_TIMER0B 20
#define IRQ_TIMER1A 21
#define IRQ_TIMER1B 22
#define IRQ_TIMER2A 23
#define IRQ_TIMER2B 24
#define IRQ_COMP0 25
//26 to 28 only in use for alternate map
#ifdef CC2538_USE_ALTERNATE_INTERRUPT_MAP
	#define IRQ_RFCORERTX 26
	#define IRQ_RFCOREERR 27
	#define IRQ_ICEPICK 28
#endif //CC2538_USE_ALTERNATE_INTERRUPT_MAP
#define IRQ_FLASH 29
//30 to 33 only in use for alternate map
#ifdef CC2538_USE_ALTERNATE_INTERRUPT_MAP
	#define IRQ_AES 30
	#define IRQ_PKA 31
	#define IRQ_SMTIM 32
	#define IRQ_MACTIMR 33
#endif //CC2538_USE_ALTERNATE_INTERRUPT_MAP
#define IRQ_SSI1 34
#define IRQ_TIMER3A 35
#define IRQ_TIMER3B 36
//37 to 43 not in use
//44 to 44 only in use for alternate map
#ifdef CC2538_USE_ALTERNATE_INTERRUPT_MAP
	#define IRQ_USB2538 44
#endif //CC2538_USE_ALTERNATE_INTERRUPT_MAP
//45 to 45 not in use
#define IRQ_UDMA 46
#define IRQ_UDMAERR 47
// 48 to 139 not in use
#ifndef CC2538_USE_ALTERNATE_INTERRUPT_MAP
	#define IRQ_USB2538 140
	#define IRQ_RFCORERTX 141
	#define IRQ_RFOREERR 142
	#define IRQ_AES 143
	#define IRQ_PKA 144
	#define IRQ_SMTIM 145
	#define IRQ_MACTIMR 146
#endif // not CC2538_USE_ALTERNATE_INTERRUPT_MAP

#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <random/rand32.h>

#endif /* !_ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* _BOARD__H_ */
