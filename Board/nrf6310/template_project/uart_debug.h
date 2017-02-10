
#ifndef SIMPLE_UART_H
#define SIMPLE_UART_H

/*lint ++flb "Enter library region" */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "nrf.h"


/** Reads a character from UART.
Execution is blocked until UART peripheral detects character has been received.
\return cr Received character.
*/
uint8_t uart_get(void);

/** Reads a character from UART with timeout on how long to wait for the byte to be received
Execution is blocked until UART peripheral detects character has been received or until the timeout expires, which even occurs first
\return bool True, if byte is received before timeout, else returns False.
@param timeout_ms maximum time to wait for the data.
@param rx_data pointer to the memory where the received data is stored.
*/
bool uart_get_with_timeout(int32_t timeout_ms, uint8_t *rx_data);

/** Sends a character to UART.
Execution is blocked until UART peripheral reports character to have been send.
@param cr Character to send.
*/
void uart_put(uint8_t cr);

/** Sends a string to UART.
Execution is blocked until UART peripheral reports all characters to have been send.
Maximum string length is 254 characters including null character in the end.
@param str Null terminated string to send.
*/
void uart_putstring(const uint8_t *str);

/** Configures UART to use 38400 baud rate.
@param rts_pin_number Chip pin number to be used for UART RTS
@param txd_pin_number Chip pin number to be used for UART TXD
@param cts_pin_number Chip pin number to be used for UART CTS
@param rxd_pin_number Chip pin number to be used for UART RXD
@param hwfc Enable hardware flow control
*/
void uart_config();


/*lint --flb "Leave library region" */
#endif
