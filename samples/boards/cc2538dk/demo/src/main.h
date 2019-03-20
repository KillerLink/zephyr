#ifndef __INC_MAIN_H
#define __INC_MAIN_H

/* Onboard LEDs and Switches */
#define PORT_LEDS ("GPIO_PORTC")
#define PIN_LED1 (0)
#define PIN_LED2 (1)
#define PIN_LED3 (2)
#define PIN_LED4 (3)
#define PORT_BTNS ("GPIO_PORTC")
#define PIN_BTN1 (4)
#define PIN_BTN2 (5)
#define PIN_BTN3 (6)
#define PIN_BTN4 (7)

#define MASK_BTN1 (1<<PIN_BTN1)
#define MASK_BTN2 (1<<PIN_BTN2)
#define MASK_BTN3 (1<<PIN_BTN3)
#define MASK_BTN4 (1<<PIN_BTN4)

#endif /* __INC_MAIN_H */
