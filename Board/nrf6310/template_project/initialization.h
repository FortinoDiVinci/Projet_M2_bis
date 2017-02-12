/****************************************************** 
 * File : initialization.h                            *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the     *
 * initializations of all the gpio of the NRF51822,   * 
 * as well as the Timers and Interruption on GPIO     *
 *                                                    *
 ******************************************************/

#ifndef INITIALIZATION_H
#define INITIALIZATION_H

/************************
*       INCLUDES        *
*************************/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "adc.h"

/************************
*       DEFINES       *
*************************/

#define DELAY_MS                100        
#define LED                     18
#define LED2                    19
#define BUTTON                  17
#define PIN_BUCK                0
#define DEBUG_PIN               2
#define DEBUG_UART_RX           1
#define DEBUG_UART_TX           3
#define PIN_ADC_ON              24

/************************
*       FUNCTIONS       *
*************************/

void gpiot_init(void);
/*
 * Initializes all the gpio of the NRF51822
 *
 * @param : None
 * @return : None
 */

void timerSPI_init();
/*
 * Initializes the Timer for the synchronisation of SPI transmissions
 *
 * @param : None
 * @return : None
 */

void timerADC_init();
/*
 * Initializes the Timer for the synchronisation of ADC task
 *
 * @param : None
 * @return : None
 */


void timerVib_init();
/*
 * Initializes the Timer for the synchronisation of Vibration sensor tasks
 *
 * @param : None
 * @return : None
 */

#endif

