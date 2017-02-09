#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "initialization.h"
#include "adc.h"




 void gpiote_init(void)
{
//  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

//  nrf_gpio_cfg_input(6,NRF_GPIO_PIN_NOPULL);
  // Configure GPIOTE channel BUUTTON to generate event when MOTION_INTERRUPT_PIN_NUMBER goes from Low to High

  NRF_GPIO->PIN_CNF[6]=(GPIO_PIN_CNF_SENSE_High<< GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
  
  //nrf_gpiote_event_config(0, 6, NRF_GPIOTE_POLARITY_LOTOHI);


  // Enable interrupt for NRF_GPIOTE->EVENTS_IN[0] event
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled<<GPIOTE_INTENSET_PORT_Pos;
  //NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled<<GPIOTE_INTENSET_IN0_Pos;
  
  
  /* Configuration of  UART pins (TX and RX) */
  
  nrf_gpio_cfg_output(DEBUG_UART_TX);
  nrf_gpio_cfg_input(DEBUG_UART_RX, NRF_GPIO_PIN_NOPULL);  
  NRF_UART0->PSELTXD = DEBUG_UART_TX;
  NRF_UART0->PSELRXD = DEBUG_UART_RX;
  
  
  nrf_gpio_cfg_output(DEBEUG_PIN);
  nrf_gpio_pin_clear(DEBEUG_PIN);                                    
  nrf_gpio_cfg_output(LED2);
  nrf_gpio_pin_clear(LED2);
  nrf_gpio_cfg_output(PIN_ADC_ON);
  nrf_gpio_pin_clear(PIN_ADC_ON);
  nrf_gpio_cfg_output(LED);
  nrf_gpio_pin_clear(LED);
  nrf_gpio_cfg_output(PIN_BUCK);
  nrf_gpio_pin_clear(PIN_BUCK); // we will change it when the consuption will be ok

//  nrf_gpio_cfg_output(1);
//  nrf_gpio_cfg_output(3);
  nrf_gpio_cfg_output(4);
  nrf_gpio_cfg_output(5);
  nrf_gpio_cfg_output(7);
  nrf_gpio_cfg_output(12);
  nrf_gpio_cfg_output(13);
  nrf_gpio_cfg_output(14);
  nrf_gpio_cfg_output(15);
  nrf_gpio_cfg_output(16);
  nrf_gpio_cfg_output(17);
  nrf_gpio_cfg_output(20);
  nrf_gpio_cfg_output(21);
  nrf_gpio_cfg_output(22);
  nrf_gpio_cfg_output(23);
  nrf_gpio_cfg_output(25);
  nrf_gpio_cfg_output(26);
  nrf_gpio_cfg_output(27);
  nrf_gpio_cfg_output(28);
  nrf_gpio_cfg_output(29);
  nrf_gpio_cfg_output(30);
//  nrf_gpio_pin_clear(1);
//  nrf_gpio_pin_clear(3);
  nrf_gpio_pin_clear(4);
  nrf_gpio_pin_clear(5);
  nrf_gpio_pin_clear(7);
  nrf_gpio_pin_clear(12);
  nrf_gpio_pin_clear(13);
  nrf_gpio_pin_clear(14);
  nrf_gpio_pin_clear(15);
  nrf_gpio_pin_clear(16);
  nrf_gpio_pin_clear(17);
  nrf_gpio_pin_clear(20);
  nrf_gpio_pin_clear(21);
  nrf_gpio_pin_clear(22);
  nrf_gpio_pin_clear(23);
  nrf_gpio_pin_clear(25);
  nrf_gpio_pin_clear(26);
  nrf_gpio_pin_clear(27);
  nrf_gpio_pin_clear(28);
  nrf_gpio_pin_clear(29);
  nrf_gpio_pin_clear(30);
  
}

   void timerVib_init()
{
  NRF_TIMER1->TASKS_STOP = 1;
  NRF_TIMER1->PRESCALER = 0x9UL; // initiliaze the prescaler to the value 9
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos; // Mode timer
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos; // Mode 16 bits
  NRF_TIMER1->CC[0] = 0x1E85; //500 ms period
  NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos ;
  NRF_TIMER1->TASKS_CLEAR = 1;
  NRF_TIMER1->EVENTS_COMPARE[0] = 0;
  NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  
}


void timerSPI_init()
{
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->PRESCALER = 0x9UL; // initiliaze the prescaler to the value 9
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos; // Mode timer
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos; // Mode 16 bits
  NRF_TIMER2->CC[0] = 0x139; // 20 ms period
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos ;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
           
}
void timerADC_init()
{
  NRF_TIMER0->TASKS_STOP=1;
  NRF_TIMER0->PRESCALER=0x9UL; // initiliaze the prescaler to the value 9
  NRF_TIMER0->MODE=TIMER_MODE_MODE_Timer<< TIMER_MODE_MODE_Pos; // Mode timer
  NRF_TIMER0->BITMODE=TIMER_BITMODE_BITMODE_32Bit<< TIMER_BITMODE_BITMODE_Pos; // Mode 16 bits
  NRF_TIMER0->CC[1]=0x00C3;//006B49D1; // 1 hour period
  NRF_TIMER0->INTENSET=TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos ;
  NRF_TIMER0->TASKS_CLEAR = 1;
  NRF_TIMER0->EVENTS_COMPARE[0]=0;
  NRF_TIMER0->SHORTS=TIMER_SHORTS_COMPARE1_CLEAR_Enabled<< TIMER_SHORTS_COMPARE1_CLEAR_Pos;
}