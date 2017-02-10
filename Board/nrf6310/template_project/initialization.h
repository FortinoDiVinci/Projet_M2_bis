
#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#define DELAY_MS               100        /*!< Timer Delay in milli-seconds */
#define LED 18
#define LED2 19
#define BUTTON 17
#define PIN_BUCK 0
#define DEBUG_PIN 2
#define DEBUG_UART_RX   1
#define DEBUG_UART_TX   3


void gpiote_init(void);
void timerSPI_init();
void timerADC_init();
void timerVib_init();

#endif

