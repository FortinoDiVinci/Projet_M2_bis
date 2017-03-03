/****************************************************** 
 * File : adc.c                                       *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the ADC *
 * used to detect the level of the battery.           *
 *                                                    *
 ******************************************************/

/************************
*       INCLUDES        *
*************************/

#include "adc.h"

/************************
*       FUNCTIONS       *
*************************/

void init_adc()
{
   
  /* Configuration of the ADC 8 bits */
  
  NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos)
                                        | (ADC_CONFIG_INPSEL_AnalogInputNoPrescaling << ADC_CONFIG_INPSEL_Pos)
                                        | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)
                                        | (ADC_CONFIG_PSEL_AnalogInput5<< ADC_CONFIG_PSEL_Pos)
                                        | (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos); 
  
  /* Enable ADC */
  
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled<<ADC_ENABLE_ENABLE_Pos;
}

uint8_t start_sampling()
{
  nrf_gpio_pin_set(PIN_ADC_ON);
  
  nrf_delay_us(750);
    
  while(NRF_ADC->BUSY);
  
  NRF_ADC->TASKS_START = 1U;
  NRF_ADC->EVENTS_END = 0U;
  
  while(NRF_ADC->EVENTS_END==0U)
  {
  }
  
  nrf_gpio_pin_clear(PIN_ADC_ON);
  
  return (uint8_t) (0x0000FF & NRF_ADC->RESULT); 
}