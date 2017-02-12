/****************************************************** 
 * File : main.c                                      *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 *  This program contains the algorithm of the main   *
 *  program. When it wakes up from enough vibrations, *
 *  it starts mesuring accelerations from the IMU and * 
 *  sends an average of ten sampling by RF            *
 *  transission.                                      *
 *                                                    *
 ******************************************************/

/************************
*       INCLUDES        *
*************************/

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
#include "uart_debug.h"
 

/************************
*       DEFINES         *
*************************/

#define MAX_LENGTH_SAMPLE       10
#define INC                     4
#define DEC                     1
#define THRESH                  20
#define MAX                     150
#define SIZE_PACKET             15
#define ID_RF                   0x1


/************************
*    GLOBAL VARIABLES   *
*************************/

uint8_t sample_count = 1;
uint8_t read_acc_value = 0;
uint8_t start = 0;
//uint8_t sleep_count=0;

static int16_t x_acc_samples[MAX_LENGTH_SAMPLE]; /* acceleration x samples */
static int16_t y_acc_samples[MAX_LENGTH_SAMPLE]; /* acceleration y samples */
static int16_t z_acc_samples[MAX_LENGTH_SAMPLE]; /* acceleration z samples */

/************************
*      DECLARATIONS     *
*************************/

void GPIOTE_IRQHandler(void);
/** GPIOTE interrupt handler.
* Triggered on state high on pin.
*/
 
void TIMER0_IRQHandler(void);
/** TIMER0 interrupt handler for ADC tasks.
* Triggered when the timer has finished to count.
*/

void TIMER1_IRQHandler(void);
/** TIMER1 interrupt handler for SPI tasks.
* Triggered when the timer has finished to count.
*/

void TIMER2_IRQHandler(void);
/** TIMER2 interrupt handler for Vibration sensor tasks.
* Triggered when the timer has finished to count.
*/

/************************
*         MAIN          *
*************************/

int main(void)
{
  /************************
  *    INITIALIZATIONS    *
  *************************/
  
  gpiot_init();
  timerSPI_init();
  timerVib_init();
  uart_config();
  //timerADC_init();
  init_IMU();
   
  // Disable internal DC/DC converter
  NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Disabled<<POWER_DCDCEN_DCDCEN_Pos;
  NRF_POWER->TASKS_LOWPWR = 1;
    
   
  // Enable GPIOTE interrupt in Nested Vector Interrupt Controller
  NVIC_EnableIRQ(GPIOTE_IRQn);
  uart_putstring("Start\r\n");
    
    
  /************************
  *       MAIN LOOP       *
  *************************/
    
  while (true)
  {
    if(start == 1)
    {
      uart_putstring("start = 1\r\n");
      NRF_TIMER1->TASKS_START = 1;
      NVIC_EnableIRQ(TIMER1_IRQn);
      
      while( read_acc_value == 1) // start == 1) 
      {
        uart_putstring("read_acc_value = 1\r\n");
        NRF_TIMER2->TASKS_START = 1;
        NVIC_EnableIRQ(TIMER2_IRQn);
        
//        if(sample_count == MAX_LENGTH_SAMPLE)
//        {
//          uint16_t x_acc = 0;
//          uint16_t y_acc = 0;
//          uint16_t z_acc = 0;
//          uint8_t data_to_send[SIZE_PACKET];
//          int32_t x_acceleration = 0, y_acceleration = 0, z_acceleration = 0;
//          for (uint8_t i = 0; i < MAX_LENGTH_SAMPLE; i++)
//          {
//            x_acceleration += x_acc_samples[i];
//            y_acceleration += y_acc_samples[i];
//            z_acceleration += z_acc_samples[i];
//          }
//          x_acc = x_acceleration/MAX_LENGTH_SAMPLE;
//          y_acc = y_acceleration/MAX_LENGTH_SAMPLE;
//          z_acc = z_acceleration/MAX_LENGTH_SAMPLE;
//          sample_count = 1;
//                
//          data_to_send[0] = 0x06;                         // Set Length to 6 bytes
//          data_to_send[1] = 0xFF;     // Write 1's to S1, for debug purposes
//          data_to_send[2] = ID_RF;
//          data_to_send[3] = (uint8_t) x_acc;
//          data_to_send[4] = (uint8_t) (x_acc >> 8);
//          data_to_send[5] = (uint8_t) y_acc;
//          data_to_send[6] = (uint8_t) (y_acc >> 8);
//          data_to_send[7] = (uint8_t) z_acc;
//          data_to_send[8] = (uint8_t) (z_acc>>8);
        
//          rf_send(data_to_send);
//          }
          
          //uart_putstring("Start' = 1\r\n");

//          __WFE();
//          __WFE();
          __WFI();
        }
      }
      else 
      {
        uart_putstring("Going to sleep\r\n");
        
        /* SHUTTING DOWN TIMER 1 & 2 */
        NRF_TIMER2->TASKS_STOP = 1;
        NRF_TIMER2->TASKS_SHUTDOWN = 1;
        NVIC_DisableIRQ(TIMER2_IRQn);
        
        NRF_TIMER1->TASKS_STOP = 1;
        NRF_TIMER1->TASKS_SHUTDOWN = 1;
        NVIC_DisableIRQ(TIMER1_IRQn);
        
        /* REACTIVATING PORT EVENT DETECTION */
        NVIC_EnableIRQ(GPIOTE_IRQn);
        
        /* SLEEP */
        __WFI();
//        __WFE();
//        __WFE();
//        NRF_POWER->SYSTEMOFF=POWER_SYSTEMOFF_SYSTEMOFF_Enter<<POWER_SYSTEMOFF_SYSTEMOFF_Pos;
      }
    }
}


/************************
*       FUNCTIONS       *
*************************/


void GPIOTE_IRQHandler(void)
{
  //uart_putstring("Inside GPIOTE_IRQHandler\r\n");
  start = 1;

  // Event causing the interrupt must be cleared
  NRF_GPIOTE->EVENTS_PORT = 0;
  NVIC_DisableIRQ(GPIOTE_IRQn);
}



void TIMER1_IRQHandler(void)
{
  static uint8_t sleep_count=0;
  if ((NRF_GPIO->IN&0x00000040)==(0x00000000))
  {
    //uart_putstring("sleep_count = ");
    //itoac(sleep_count, 0);
    //uart_putstring("\r\n");
    sleep_count ++; 
  }
  else
  {
    sleep_count = 0;
  }
  if(sleep_count > THRESH)
  {
    sleep_count = 0;
    start = 0;
    read_acc_value = 0;
//    NRF_TIMER1->TASKS_START=0;
//    NRF_TIMER1->TASKS_STOP=1;
    
    if((NRF_TIMER1->EVENTS_COMPARE[0]==1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
    {
       NRF_TIMER1->EVENTS_COMPARE[0] = 0;
      //NRF_TIMER0->TASKS_START=1;
    }
    NRF_GPIOTE->EVENTS_PORT = 0;
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NRF_TIMER1->TASKS_STOP = 1;
    NVIC_DisableIRQ(TIMER1_IRQn);
  }
  else
  {
    uart_putstring("launching acc sampling\r\n");
    //read_ac_value(&x_acc_samples[sample_count], &y_acc_samples[sample_count], &z_acc_samples[sample_count]);

    //sample_count += 1;
    read_acc_value = 1;
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

void TIMER2_IRQHandler(void)
{
  uart_putstring("Inside TIMER2 : doing acc sampling\r\n");
  
  read_ac_value(&x_acc_samples[sample_count], &y_acc_samples[sample_count], &z_acc_samples[sample_count]);
  sample_count += 1;
 
  if(sample_count == MAX_LENGTH_SAMPLE)
  {
    uint16_t x_acc = 0;
    uint16_t y_acc = 0;
    uint16_t z_acc = 0;
    uint8_t data_to_send[SIZE_PACKET];
    int32_t x_acceleration = 0, y_acceleration = 0, z_acceleration = 0;
    for (uint8_t i = 0; i < MAX_LENGTH_SAMPLE; i++)
    {
       x_acceleration += x_acc_samples[i];
       y_acceleration += y_acc_samples[i];
       z_acceleration += z_acc_samples[i];
    }
    x_acc = x_acceleration/MAX_LENGTH_SAMPLE;
    y_acc = y_acceleration/MAX_LENGTH_SAMPLE;
    z_acc = z_acceleration/MAX_LENGTH_SAMPLE;
    sample_count = 1;
    
    uart_putstring("sending data\r\n");
    
    data_to_send[0] = 0x06;                         // Set Length to 6 bytes
    data_to_send[1] = 0xFF;     // Write 1's to S1, for debug purposes
    data_to_send[2] = ID_RF;
    data_to_send[3] = (uint8_t) x_acc;
    data_to_send[4] = (uint8_t) (x_acc >> 8);
    data_to_send[5] = (uint8_t) y_acc;
    data_to_send[6] = (uint8_t) (y_acc >> 8);
    data_to_send[7] = (uint8_t) z_acc;
    data_to_send[8] = (uint8_t) (z_acc>>8);
    rf_send(data_to_send);
  }  
  
  
  if((NRF_TIMER2->EVENTS_COMPARE[0]==1) && (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
  {
    NRF_TIMER2->EVENTS_COMPARE[0]=0;
    //NRF_TIMER0->TASKS_START=1;
  }
}