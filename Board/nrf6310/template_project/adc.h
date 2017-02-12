/****************************************************** 
 * File : adc.h                                       *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the ADC *
 * used to detect the level of the battery.           *
 *                                                    *
 ******************************************************/

#ifndef ADC_H
#define ADC_H

/************************
*       INCLUDES        *
*************************/

#include <stdint.h>
#include "nrf_gpio.h"
#include "initialization.h"

/************************
*       FUNCTIONS       *
*************************/

void init_adc(void);
/*
 * Initialization of the ADC
 *
 * @param : None
 * @return : None
 */

uint8_t start_sampling();

/*
 * Start sampling task and return the value of the ADC
 *
 * @param : None
 * @return 
 * @retval uint8_t value of the ADC sampling
 */

#endif

