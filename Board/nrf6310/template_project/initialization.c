/****************************************************** 
 * File : initialization.c                            *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the     *
 * initializations of all the gpio of the NRF51822,   * 
 * as well as the Timers and Interruption on GPIO     *
 *                                                    *
 ******************************************************/


/************************
*       INCLUDES        *
*************************/

#include "initialization.h"


/************************
*       FUNCTIONS       *
*************************/

 void gpiot_init(void)
{ 
   /* Configuration of BUCK pin */
  nrf_gpio_cfg_output(BUCK_ON);
  nrf_gpio_pin_clear(BUCK_ON);
  nrf_delay_us(700);    
 
  
  /* Configuration of all other pins 
   *
   * SPI pins are initialized in the 
   * spi_master_config file           */
  
  
  // BUCK
  nrf_gpio_cfg_output(1);                       // unused
  nrf_gpio_cfg_output(LED);
  nrf_gpio_cfg_output(INT2_MEMS);
  // BAT_LVL (Analog5)
  nrf_gpio_cfg_output(INT1_MEMS);
  // VIB (vibrasensor)
  nrf_gpio_cfg_output(7);                       // unused
  nrf_gpio_cfg_output(8);                       // unused
  nrf_gpio_cfg_output(9);                       // unused
  nrf_gpio_cfg_output(10);                      // unused
  nrf_gpio_cfg_output(11);                      // unused
  nrf_gpio_cfg_output(12);                      // unused
  nrf_gpio_cfg_output(13);                      // unused
  nrf_gpio_cfg_output(14);                      // unused
  nrf_gpio_cfg_output(15);                      // unused
  nrf_gpio_cfg_output(16);                      // unused
  // SPI SEL
  nrf_gpio_cfg_output(18);                      // unused
  // SPI SCK
  nrf_gpio_cfg_output(20);                      // unused
  // SPI MISO
  nrf_gpio_cfg_output(22);                      // unused
  // SPI MOSI
  nrf_gpio_cfg_output(BAT_LVL_ON);
  nrf_gpio_cfg_output(25);                      // unused
  nrf_gpio_cfg_output(26);                      // unused
  nrf_gpio_cfg_output(27);                      // unused
  nrf_gpio_cfg_output(28);                      // unused
  nrf_gpio_cfg_output(29);                      // unused
  nrf_gpio_cfg_output(30);                      // unused 
  
  
  // BUCK
  nrf_gpio_pin_clear(1);                       // unused
  nrf_gpio_pin_clear(LED);
  nrf_gpio_pin_clear(INT2_MEMS);
  // BAT_LVL (Analog5)
  nrf_gpio_pin_clear(INT1_MEMS);
  // VIB (vibrasensor)
  nrf_gpio_pin_clear(7);                       // unused
  nrf_gpio_pin_clear(8);                       // unused
  nrf_gpio_pin_clear(9);                       // unused
  nrf_gpio_pin_clear(10);                      // unused
  nrf_gpio_pin_clear(11);                      // unused
  nrf_gpio_pin_clear(12);                      // unused
  nrf_gpio_pin_clear(13);                      // unused
  nrf_gpio_pin_clear(14);                      // unused
  nrf_gpio_pin_clear(15);                      // unused
  nrf_gpio_pin_clear(16);                      // unused
  // SPI SEL
  nrf_gpio_pin_clear(18);                      // unused
  // SPI SCK
  nrf_gpio_pin_clear(20);                      // unused
  // SPI MISO
  nrf_gpio_pin_clear(22);                      // unused
  // SPI MOSI
  nrf_gpio_pin_clear(BAT_LVL_ON);
  nrf_gpio_pin_clear(25);                      // unused
  nrf_gpio_pin_clear(26);                      // unused
  nrf_gpio_pin_clear(27);                      // unused
  nrf_gpio_pin_clear(28);                      // unused
  nrf_gpio_pin_clear(29);                      // unused
  nrf_gpio_pin_clear(30);                      // unused 
  
  /* Configure GPIOTE channel Vibrations sensor to generate event when pin 6 is in a high state */

  NRF_GPIO->PIN_CNF[VIB]=(GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

  /* Enable interrupt on PORT for GPIOTE */
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos; 
}

   void timerVib_init()
{
  NRF_TIMER1->TASKS_STOP = 1;
  NRF_TIMER1->PRESCALER = 0x9UL;                                                  // initiliaze the prescaler to the value 9
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;                // Mode timer
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos; // Mode 16 bits
  NRF_TIMER1->CC[0] = 0x1E85;                                                     //500 ms period
  NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos ;
  NRF_TIMER1->TASKS_CLEAR = 1;
  NRF_TIMER1->EVENTS_COMPARE[0] = 0;
  NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  
}


void timerSPI_init()
{
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->PRESCALER = 0x9UL;                                                        // initiliaze the prescaler to the value 9
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;                      // Mode timer
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;       // Mode 16 bits
  NRF_TIMER2->CC[0] = 0x139;                                                            // 20 ms period
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos ;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled<< TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  
}
void timerADC_init()
{
  NRF_TIMER0->TASKS_STOP = 1;
  NRF_TIMER0->PRESCALER = 0x9UL;                                                        // initiliaze the prescaler to the value 9
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;                      // Mode timer
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;       // Mode 16 bits
  NRF_TIMER0->CC[1] = 0x00C3;//006B49D1;                                                // 1 hour period
  NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos ;
  NRF_TIMER0->TASKS_CLEAR = 1;
  NRF_TIMER0->EVENTS_COMPARE[0] = 0;
  NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos;
}