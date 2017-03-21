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

/* PINS DEFINITION */

#define BUCK_ON                 0
#define LED                     2
#define INT2_MEMS               3
#define BAT_LVL                 4 // Analog pin 5
#define INT1_MEMS               5
#define VIB                     6
#define SEL                    17
#define SCK                    19
#define MISO                   21
#define MOSI                   23
#define BAT_LVL_ON              24

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

