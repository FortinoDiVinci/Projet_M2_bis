/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example template
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief Example template.
*
*/

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "nrf.h"
#include "nrf51.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "adc.h"
   
#include "initialization.h"
#include "spi_master.h"
#include "common.h"
#include "spi_master_config.h"
#include "radio_config.h"

#define MAX_LENGTH_SAMPLE 10
#define INC 4
#define DEC 1
#define THRESH 20
#define MAX 150
#define SIZE_PACKET 15


uint8_t sample_count = 1;
uint8_t start = 0;
uint8_t sleep_count=0;

static int16_t x_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration x samples*/
static int16_t y_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration y samples */
static int16_t z_acc_samples[MAX_LENGTH_SAMPLE]; /*acceleration z samples */

/**
 * main function
 * \return 0. int return type required by ANSI/ISO standard. 
 */

void GPIOTE_IRQHandler(void);

//void TIMER0_IRQHandler(void);

void TIMER1_IRQHandler(void);

//void TIMER2_IRQHandler(void);

int main(void)
{
/** GPIOTE interrupt handler.
* Triggered on motion interrupt pin input low-to-high transition.
*/
 
   gpiote_init();
   timerSPI_init();
//   timerADC_init();
   NRF_POWER->DCDCEN=POWER_DCDCEN_DCDCEN_Disabled<<POWER_DCDCEN_DCDCEN_Pos;
   NRF_POWER->TASKS_LOWPWR=1;
   
  // Enable GPIOTE interrupt in Nested Vector Interrupt Controller
   NVIC_EnableIRQ(GPIOTE_IRQn);
 
//    NRF_POWER->SYSTEMOFF=POWER_SYSTEMOFF_SYSTEMOFF_Enter<<POWER_SYSTEMOFF_SYSTEMOFF_Pos;
  
  //SPI0  
    write_data(0x3F,0X10);  // set accelrometre (get mesure : 52 hz; scall:+-16g filter :50hz)
    //write_data(0x33,0x10);     // set accelerometre (get mesure: 52hz scall:+-2g filter :50hz)
    read_data(0x10);        // check value 
    write_data(0x10,0x15);  // disable high-performance mode for accelerometre 
    while (true)
    {     
      if( start == 1)
      { 
        NRF_TIMER1->TASKS_STOP=0;
        NRF_TIMER1->TASKS_START=1;
        NVIC_EnableIRQ(TIMER1_IRQn);
        while(start == 1)
        {
//          __WFE();
//          __WFE();
          __WFI();
          if(sample_count == MAX_LENGTH_SAMPLE)
          {
            uint16_t x_acc=0;
            uint16_t y_acc=0;
            uint16_t z_acc=0;
            uint8_t data_to_send[SIZE_PACKET];
            int32_t x_acceleration=0,y_acceleration=0,z_acceleration=0;
            for (uint8_t i =0; i< MAX_LENGTH_SAMPLE; i++)
            {
               x_acceleration += x_acc_samples[i];
               y_acceleration += y_acc_samples[i];
               z_acceleration += z_acc_samples[i];
            }
            x_acc = x_acceleration/MAX_LENGTH_SAMPLE;
            y_acc = y_acceleration/MAX_LENGTH_SAMPLE;
            z_acc = z_acceleration/MAX_LENGTH_SAMPLE;
            sample_count = 1;
            
            data_to_send[0] = 0x05;                         // Set Length to 5 bytes
            data_to_send[1] = 0xFF;                         // Write 1's to S1, for debug purposes
            data_to_send[2] = (uint8_t) x_acc;
            data_to_send[3] = (uint8_t) (x_acc >> 8);
            data_to_send[4] = (uint8_t) y_acc;
            data_to_send[5] = (uint8_t) (y_acc >> 8);
            data_to_send[6] = (uint8_t) z_acc;
            data_to_send[7] = (uint8_t) (z_acc>>8);
            rf_send(data_to_send);
          }
        }
      }
      else 
      {
        NRF_TIMER1->TASKS_START=0;
        NRF_TIMER1->TASKS_STOP=1;
        NVIC_DisableIRQ(TIMER1_IRQn);
        __WFI();
//        __WFE();
//        __WFE();
//        NRF_POWER->SYSTEMOFF=POWER_SYSTEMOFF_SYSTEMOFF_Enter<<POWER_SYSTEMOFF_SYSTEMOFF_Pos;
      }
    }
}

/**
 *@}
 **/

void GPIOTE_IRQHandler(void)
{
  nrf_gpio_pin_toggle(LED2);
  start =1;

  // Event causing the interrupt must be cleared
  NRF_GPIOTE->EVENTS_PORT = 0;
  NVIC_DisableIRQ(GPIOTE_IRQn);
}



void TIMER1_IRQHandler(void)
{
  //static uint8_t sleep_count=0;
  if ((NRF_GPIO->IN&0x00000040)==(0x00000000))
  {
    //nrf_gpio_pin_set(LED);
    sleep_count ++; 
  }
  else
  {
    sleep_count = 0;
  }
  if(sleep_count > THRESH)
  {
    sleep_count = 0;
    //nrf_gpio_pin_clear(LED);
    start = 0;
    NRF_TIMER1->TASKS_START=0;
    NRF_TIMER1->TASKS_STOP=1;
    NVIC_DisableIRQ(TIMER1_IRQn);
    if((NRF_TIMER1->EVENTS_COMPARE[0]==1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
    {
       NRF_TIMER1->EVENTS_COMPARE[0]=0;
      //NRF_TIMER0->TASKS_START=1;
    }
    NRF_GPIOTE->EVENTS_PORT = 0;
    NVIC_EnableIRQ(GPIOTE_IRQn);
  }
  else
  {
    read_ac_value(&x_acc_samples[sample_count],&y_acc_samples[sample_count],&z_acc_samples[sample_count]);
    sample_count += 1;
    if((NRF_TIMER1->EVENTS_COMPARE[0]==1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
    {
      NRF_TIMER1->EVENTS_COMPARE[0]=0;
    //NRF_TIMER0->TASKS_START=1;
    }
  } 
}

//void TIMER0_IRQHandler(void)
//{
//  static uint8_t adc_value;
//  static bool LED_on = 0;
//  adc_value = start_sampling();
//  
//  if(adc_value < 0xCF)//D5) // 3,50 V
//  {
//    NRF_TIMER0->CC[1] = 0x004C4B; // timer is set from 1 hour to 10 s
//    if(!LED_on)
//    {
//      nrf_gpio_pin_set(DEBEUG_PIN);
//      NRF_TIMER0->CC[1] = 0x00C3; // timer is set from 10 s to 100 ms
//      LED_on = 1;
//    }
//    else
//    {
//      nrf_gpio_pin_clear(DEBEUG_PIN);
//      NRF_TIMER0->CC[1] = 0x004C4B; // timer is set from 100 ms to 10 s
//      LED_on = 0;
//    }
//  }
//  else
//  {
//    NRF_TIMER0->CC[1] = 0x006B49D1;   // timer is set to 1 hour
//  }
//  
//   if((NRF_TIMER0->EVENTS_COMPARE[1] == 1) && (NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE1_Msk))
//  {
//    NRF_TIMER0->EVENTS_COMPARE[1] = 0;
//    NRF_TIMER0->TASKS_START = 1;
//  }
//}

//void TIMER2_IRQHandler(void)
//{
//  
// 
//  if((NRF_TIMER2->EVENTS_COMPARE[0]==1) && (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
//  {
//    NRF_TIMER2->EVENTS_COMPARE[0]=0;
//    //NRF_TIMER0->TASKS_START=1;
//  }
//}