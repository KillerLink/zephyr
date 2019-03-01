#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>

/* Driverlib includes */
#include <inc/hw_types.h>
#include <driverlib/rom.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/uart.h>
#include <driverlib/ioc.h>
#include <driverlib/gpio.h>
#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>
#include <inc/hw_sys_ctrl.h>

//TODO: extend support to uart1
//TODO: make pin assignment kconfig choosable

#define DEV_CFG(dev) ((const struct uart_ti_cc2538_config_t * const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct uart_ti_cc2538_data_t * const)(dev)->driver_data)

#if defined(CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN)
static void uart_cc2538_isr(void *arg);
#endif // CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN

struct uart_ti_cc2538_config_t {
	unsigned long uart_base; // base address of UART port
	unsigned long irq_num; 	// UART IRQ number
};

struct uart_ti_cc2538_data_t {
#if defined(CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN)
	uart_irq_callback_t cb; // Callback function pointer
#endif // CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN
};


/*
 *  CC2538 UART has a configurable FIFO length, from 1 to 8 characters.
 *  However, the Zephyr console driver, and the Zephyr uart sample test, assume
 *  a RX FIFO depth of one: meaning, one interrupt == one character received.
 *  Keeping with this assumption, this driver leaves the FIFOs disabled,
 *  and at depth 1.
 */

#ifdef CONFIG_UART_TI_CC2538_UART0
static int uart_ti_cc2538_init_uart0 (struct device *dev) {

	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	unsigned long uart_base = (unsigned long)config->uart_base;

	//Disable
	UARTDisable(uart_base);

	//Perform Reset
	SysCtrlPeripheralReset(SYS_CTRL_PERIPH_UART0);

	//PinMux Settings
	//RX/TX
	IOCPinConfigPeriphOutput(GPIO_A_BASE, GPIO_PIN_1, IOC_MUX_OUT_SEL_UART0_TXD); //tx output
	GPIOPinTypeUARTOutput(GPIO_A_BASE, GPIO_PIN_1);
	IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_0, IOC_UARTRXD_UART0); //rx input
	GPIOPinTypeUARTInput(GPIO_A_BASE, GPIO_PIN_0);
	//XDS100v3 RTS/CTS //TODO: make config? //TODO: is this mandatory
	GPIODirModeSet(GPIO_B_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN); //xds100v3 cts
	GPIODirModeSet(GPIO_D_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT); //xds100v3 rts
	GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_3, 1);


	//Allow to continue operation in (Deep)SleepMode
	#ifdef CONFIG_UART_TI_CC2538_UART0_INSLEEP
	SysCtrlPeripheralSleepEnable(SYS_CTRL_PERIPH_UART0);
	#else
	#warning "UART0 wont work correctly in sleep!"
	#endif
	#ifdef CONFIG_UART_TI_CC2538_UART0_INDEEPSLEEP
	SysCtrlPeripheralDeepSleepEnable(SYS_CTRL_PERIPH_UART0);
	#else
	#warning "UART0 wont work correctly in deepsleep!"
	#endif

	//Enable Peripheral Clock to Uart 0
	SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);

	//Define Clock (Systemclock or PIOSC ?? IO Clock)
	#if defined(CONFIG_UART_TI_CC2538_UART0_CLOCK_SYSTEM)
	UARTClockSourceSet(uart_base, UART_CLOCK_SYSTEM);
	uint32_t baudclock = SysCtrlClockGet();
	#elif defined(CONFIG_UART_TI_CC2538_UART0_CLOCK_PIOSC)
	UARTClockSourceSet(uart_base, UART_CLOCK_PIOSC);
	uint32_t baudclock = SysCtrlIOClockGet();
	#else
	#warning "No clock source for UART0 selected"
	#endif

	//Setup Configuration Parameters
	UARTConfigSetExpClk(
		uart_base,
		baudclock,
		UART0_CURRENT_SPEED,
		(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));

	//Fifo Settings IF used
	UARTFIFOLevelSet(uart_base, UART_FIFO_TX7_8, UART_FIFO_RX7_8);

	//Enable
	UARTEnable(uart_base);

	//No Fifo
	#ifndef CONFIG_UART_TI_CC2538_UART0_FIFOS
	UARTFIFODisable(uart_base);
	#endif

	#ifdef CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN
	//Clear Interrupts
	UARTIntClear(uart_base, (UART_INT_RX | UART_INT_TX));
	IRQ_CONNECT(
		UART0_IRQ,
		UART0_IRQ_PRIORITY,
		uart_ti_cc2538_isr,
		DEVICE_GET(uart_ti_cc2538_uart0_dev),
		0);
	irq_enable(UART0_IRQ);
	#endif

	return 0;
}
#endif

static int uart_ti_cc2538_poll_in(struct device *dev, unsigned char *c) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	unsigned long uart_base = (unsigned long)config->uart_base;
	if (UARTCharsAvail(uart_base)) {
		*c = UARTCharGetNonBlocking(uart_base);
	} else {
		return (-1);
	}
	return 0;
}

static unsigned char uart_ti_cc2538_poll_out(struct device *dev, unsigned char c) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	unsigned long uart_base = (unsigned long)config->uart_base;
	UARTCharPut(uart_base, c);
	#if !defined(CONFIG_UART_TI_CC2538_UART0_INSLEEP) ||  !defined(CONFIG_UART_TI_CC2538_UART0_INDEEPSLEEP)
	//If the uart module does not continue operation in (deep)sleep,
	//every transmition must be finished on function return.
	//Else there will be some strange errors (incorrect transmitted characters)
	while ((volatile char)(UARTBusy(uart_base))); //can solve problem with printk->k_sleep
	#endif
	return c;
}

static int uart_ti_cc2538_err_check(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
        unsigned long uart_base = (unsigned long)config->uart_base;
	unsigned long cc2538_errs = 0L;
	unsigned int z_err = 0;
	cc2538_errs = UARTRxErrorGet(uart_base);
	// Map cc2538 SDK uart.h defines to zephyr uart.h defines
	z_err = 0;
	z_err |= (cc2538_errs & UART_RXERROR_OVERRUN) ? UART_ERROR_OVERRUN : 0;
	z_err |= (cc2538_errs & UART_RXERROR_BREAK) ? UART_ERROR_BREAK : 0;
	z_err |= (cc2538_errs & UART_RXERROR_PARITY) ? UART_ERROR_PARITY : 0;
	z_err |= (cc2538_errs & UART_RXERROR_FRAMING) ? UART_ERROR_FRAMING : 0;
	UARTRxErrorClear(uart_base);
	return (int)z_err;
}

#ifdef CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN

static int uart_ti_cc2538_fifo_fill(struct device *dev, const u8_t *tx_data, int size) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	unsigned int num_tx = 0;
	while ((size - num_tx) > 0) {
		/* Send a character */
		if (UARTCharPutNonBlocking((unsigned long)config->uart_base, tx_data[num_tx])) {
			num_tx++;
		} else {
			break;
		}
	}
	return (int)num_tx;
}

static int uart_ti_cc2538_fifo_read(struct device *dev, u8_t *rx_data, const int size) {
	const struct uart_ti_cc2538_config *config = DEV_CFG(dev);
	unsigned int num_rx = 0;
	while (((size - num_rx) > 0) &&
		UARTCharsAvail((unsigned long)config->uart_base)) {
		/* Receive a character */
		rx_data[num_rx++] = UARTCharGetNonBlocking((unsigned long)config->uart_base);
	}
	return num_rx;
}

static void uart_ti_cc2538_irq_tx_enable(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	UARTIntEnable((unsigned long)config->uart_base, UART_INT_TX);
}

static void uart_ti_cc2538_irq_tx_disable(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	UARTIntDisable((unsigned long)config->uart_base, UART_INT_TX);
}

static int uart_ti_cc2538_irq_tx_ready(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	unsigned int int_status;
	int_status = UARTIntStatus((unsigned long)config->uart_base, 1);
	return (int_status & UART_INT_TX);
}

static void uart_ti_cc2538_irq_rx_enable(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	// FIFOs are left disabled from reset, so UART_INT_RT flag not used.
	UARTIntEnable((unsigned long)config->uart_base, UART_INT_RX);
}

static void uart_ti_cc2538_irq_rx_disable(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	UARTIntDisable((unsigned long)config->uart_base, UART_INT_RX);
}

static int uart_ti_cc2538_irq_tx_complete(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	return (!UARTBusy((unsigned long)config->uart_base));
}

static int uart_ti_cc2538_irq_rx_ready(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	unsigned int int_status;
	int_status = UARTIntStatus((unsigned long)config->uart_base, 1);
	return (int_status & UART_INT_RX);
}

static void uart_ti_cc2538_irq_err_enable(struct device *dev) {
	// Not yet used in zephyr
}

static void uart_ti_cc2538_irq_err_disable(struct device *dev) {
	// Not yet used in zephyr
}

static int uart_cc2538_irq_is_pending(struct device *dev) {
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	unsigned int int_status;
	int_status = UARTIntStatus((unsigned long)config->uart_base, 1);
	return (int_status & (UART_INT_TX | UART_INT_RX));
}

static int uart_ti_cc2538_irq_update(struct device *dev) {
	return 1;
}

static void uart_ti_cc2538_irq_callback_set(struct device *dev, uart_irq_callback_t cb) {
	struct uart_ti_cc2538_data_t * const dev_data = DEV_DATA(dev);
	dev_data->cb = cb;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * Note: CC2538 UART Tx interrupts when ready to send; Rx interrupts when char
 * received.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
static void uart_cc2538_isr(void *arg)
{
	struct device *dev = arg;
	const struct uart_ti_cc2538_config_t *config = DEV_CFG(dev);
	struct uart_ti_cc2538_data_t * const dev_data = DEV_DATA(dev);

	unsigned long intStatus = UARTIntStatus((unsigned long)config->uart_base,1);

	if (dev_data->cb) {
		dev_data->cb(dev);
	}
	/*
	 * Clear interrupts only after cb called, as Zephyr UART clients expect
	 * to check interrupt status during the callback.
	 */
	UARTIntClear((unsigned long)config->uart_base, intStatus);
}
#endif /* CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_ti_cc2538_driver_api = {
	.poll_in = uart_ti_cc2538_poll_in,
	.poll_out = uart_ti_cc2538_poll_out,
	.err_check = uart_ti_cc2538_err_check,
#ifdef CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN
	.fifo_fill	  = uart_ti_cc2538_fifo_fill,
	.fifo_read	  = uart_ti_cc2538_fifo_read,
	.irq_tx_enable	  = uart_ti_cc2538_irq_tx_enable,
	.irq_tx_disable	  = uart_ti_cc2538_irq_tx_disable,
	.irq_tx_ready	  = uart_ti_cc2538_irq_tx_ready,
	.irq_rx_enable	  = uart_ti_cc2538_irq_rx_enable,
	.irq_rx_disable	  = uart_ti_cc2538_irq_rx_disable,
	.irq_tx_complete  = uart_ti_cc2538_irq_tx_complete,
	.irq_rx_ready	  = uart_ti_cc2538_irq_rx_ready,
	.irq_err_enable	  = uart_ti_cc2538_irq_err_enable,
	.irq_err_disable  = uart_ti_cc2538_irq_err_disable,
	.irq_is_pending	  = uart_ti_cc2538_irq_is_pending,
	.irq_update	  = uart_ti_cc2538_irq_update,
	.irq_callback_set = uart_ti_cc2538_irq_callback_set,
#endif /* CONFIG_UART_TI_CC2538_UART0_INTERRUPT_DRIVEN */
};



#ifdef CONFIG_UART_TI_CC2538_UART0

static struct device DEVICE_NAME_GET(uart_ti_cc2538_uart0_dev);
static struct uart_ti_cc2538_data_t uart_ti_cc2538_data_uart0 = {
#ifdef CONFIG_UART_TI_CC2538_INTERRUPT_DRIVEN
	.cb = NULL,
#endif
};

static const struct uart_ti_cc2538_config_t uart_ti_cc2538_config_uart0 = {
	.uart_base = UART0_BASE_ADDRESS,
	.irq_num = UART0_IRQ,
};

DEVICE_AND_API_INIT(
	uart_ti_cc2538_uart0_dev,
	UART0_LABEL,
	uart_ti_cc2538_init_uart0,
	&uart_ti_cc2538_data_uart0,
	&uart_ti_cc2538_config_uart0,
	PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	(void *)&uart_ti_cc2538_driver_api);
#endif /* CONFIG_UART_TI_CC2538_UART0 */
