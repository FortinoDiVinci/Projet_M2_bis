/****************************************************** 
 * File : uart_debug.h                                *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about UART    *
 * transmission, used to debug the program.           *
 *                                                    *
 ******************************************************/


#ifndef DEBUG_UART_H
#define DEBUG_UART_H

/************************
*       INCLUDES        *
*************************/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf.h"

/************************
*    GLOBAL VARIABLES   *
*************************/

const char *h="0123456789";

/************************
*       FUNCTIONS       *
*************************/

uint8_t uart_get(void);

/** Reads a character from UART.
* Execution is blocked until UART peripheral detects character has been received.
* \return cr Received character.
*/

bool uart_get_with_timeout(int32_t timeout_ms, uint8_t *rx_data);

/** Reads a character from UART with timeout on how long to wait for the byte to be received
Execution is blocked until UART peripheral detects character has been received or until the timeout expires, which even occurs first
\return bool True, if byte is received before timeout, else returns False.
@param timeout_ms maximum time to wait for the data.
@param rx_data pointer to the memory where the received data is stored.
*/

void uart_put(uint8_t cr);

/** Sends a character to UART.
Execution is blocked until UART peripheral reports character to have been send.
@param cr Character to send.
*/

void uart_putstring(const uint8_t *str);

/** Sends a string to UART.
Execution is blocked until UART peripheral reports all characters to have been send.
Maximum string length is 254 characters including null character in the end.
@param str Null terminated string to send.
*/

void uart_config();

/** Configures UART to use 115200 baud rate.
* @param : None
*
*/


#endif
