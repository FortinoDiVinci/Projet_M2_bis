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

#define MAX_LENGTH_SAMPLE       5
#define INC                     4
#define DEC                     1
#define THRESH                  20
#define MAX                     150
#define SIZE_PACKET             15
#define ID_RF                   0x1
#undef UART

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
  timerADC_init();
  init_adc();
  init_IMU();
  
  // Disable internal DC/DC converter
  NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Disabled<<POWER_DCDCEN_DCDCEN_Pos;
  NRF_POWER->TASKS_LOWPWR = 1;
   
  // Enable GPIOTE interrupt in Nested Vector Interrupt Controller
  ////////NVIC_EnableIRQ(GPIOTE_IRQn);
    
  /************************
  *       MAIN LOOP       *
  *************************/

//  nrf_gpio_pin_clear(BUCK_ON);
// 
  
  NRF_TIMER2->TASKS_START = 1;
  NVIC_EnableIRQ(TIMER2_IRQn);
  //IMU_ON();

  write_data(0x3F,0X10);
  write_data(0xA4,0x17);
  write_data(0x10,0X58);
  
  while(1)
  {
    __WFI();
  }
  
  
  while (true)
  {
    if (start == 0)
    {
        #ifdef UART
        uart_putstring("Going to sleep\r\n");
        #endif
        /* SHUTTING DOWN TIMER 1 & 2 */
        
        NRF_TIMER2->TASKS_STOP = 1;
        NRF_TIMER2->TASKS_SHUTDOWN = 1;
        NVIC_DisableIRQ(TIMER2_IRQn);
        
        NRF_TIMER1->TASKS_STOP = 1;
        NRF_TIMER1->TASKS_SHUTDOWN = 1;
        NVIC_DisableIRQ(TIMER1_IRQn);
        
          //disable low battery
        NRF_TIMER0->TASKS_STOP = 1;
        NRF_TIMER0->TASKS_SHUTDOWN = 1;
        NVIC_DisableIRQ(TIMER0_IRQn); 
        
//        //disable buck
//        nrf_gpio_pin_set(BUCK_ON);
        
        /* REACTIVATING PORT EVENT DETECTION */
        NVIC_EnableIRQ(GPIOTE_IRQn);
        
        //NRF_POWER->SYSTEMOFF=POWER_SYSTEMOFF_SYSTEMOFF_Enter<<POWER_SYSTEMOFF_SYSTEMOFF_Pos;
        /* SLEEP */
        
        nrf_gpio_pin_set(BUCK_ON);
        __WFI();
        nrf_gpio_pin_clear(BUCK_ON);       
    }
    else
    {
//      uart_putstring("start = 1\r\n");
      
      //enable buck
//      nrf_gpio_pin_clear(BUCK_ON);
//      //nrf_delay_us(700);
      
      /* ACTIVATION OF TIMER 1 & 2 */
      
      NRF_TIMER1->TASKS_START = 1;
      NVIC_EnableIRQ(TIMER1_IRQn);
      
      NRF_TIMER2->TASKS_START = 1;
      NVIC_EnableIRQ(TIMER2_IRQn);
      
      /* DESACTIVATION OF PORT EVENT DETECTION */
      NVIC_DisableIRQ(GPIOTE_IRQn);
      
      while( /* read_acc_value == 1)*/ start == 1) 
      {
        /* SLEEP */
        nrf_gpio_pin_set(BUCK_ON);
        __WFI();     
        nrf_gpio_pin_clear(BUCK_ON);
        #ifdef UART  
        uart_putstring("read_acc_value = 1\r\n");
        #endif
          
      }
     }   
   }
}


/************************
*       FUNCTIONS       *
*************************/


void GPIOTE_IRQHandler(void)
{
  #ifdef UART
  uart_putstring("Inside GPIOTE_IRQHandler\r\n");
  #endif
  start = 1;
  
  nrf_gpio_pin_clear(BUCK_ON);
  
  // enable the IMU to read values
  IMU_ON();

  // Event causing the interrupt must be cleared
  NRF_GPIOTE->EVENTS_PORT = 0;
  
}



void TIMER1_IRQHandler(void)
{
  static uint8_t sleep_count=0;
  
  if ((NRF_GPIO->IN&0x00000040)==(0x00000000))
  {
    #ifdef UART
    uart_putstring("sleep_count = ");
    #endif
//    itoac(sleep_count, 0);
    #ifdef UART
    uart_putstring("\r\n");
    #endif
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
    
    // disable the IMU to save battery
    IMU_OFF();
    
    if((NRF_TIMER1->EVENTS_COMPARE[0]==1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
    {
       NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    }
  }
  else
  {
    if((NRF_TIMER1->EVENTS_COMPARE[0]==1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
    {
      NRF_TIMER1->EVENTS_COMPARE[0]=0;
    }
  } 
}

void TIMER0_IRQHandler(void)
{
  static bool LED_on = 0; 
  if(!LED_on)
  {
    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->CC[1] = 0x00C3; // timer is set from 10 s to 100 ms
    NRF_TIMER0->TASKS_START = 1;
    LED_on = 1;
  }
  else
  {
    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->CC[1] = 0x02FAEE; // timer is set from 100 ms to 10 s
    NRF_TIMER0->TASKS_START = 1;
    LED_on = 0;
  }
  
  if((NRF_TIMER0->EVENTS_COMPARE[1] == 1) && (NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE1_Msk))
  {
    NRF_TIMER0->EVENTS_COMPARE[1] = 0;
    NRF_TIMER0->TASKS_START = 1;
  }
}

void TIMER2_IRQHandler(void)
{
  //enable buck
  nrf_gpio_pin_clear(BUCK_ON);
  nrf_delay_us(700);
  
  read_ac_value(&x_acc_samples[sample_count], &y_acc_samples[sample_count], &z_acc_samples[sample_count]);
  sample_count += 1;
  
  //nrf_gpio_pin_set(BUCK_ON);
 
  if(sample_count == MAX_LENGTH_SAMPLE)
  {
    static uint8_t adc_value;
    static uint16_t adc_count=0;
    adc_count++; 
    if(adc_count==300)
    {
      adc_value = start_sampling();
      if(adc_value <= 0xFF)//D5) // 3,50 V
      {
        NRF_TIMER0->TASKS_START = 1;
        NVIC_EnableIRQ(TIMER0_IRQn);
      }
      else
      {
        NRF_TIMER0->TASKS_SHUTDOWN = 1;
        NVIC_DisableIRQ(TIMER0_IRQn);   
      }
      adc_count=0;
    }
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
    #ifdef UART
    uart_putstring("sending data\r\n");
    #endif
    data_to_send[0] = 0x07;      // Set Length to 6 bytes
    data_to_send[1] = 0xFF;     // Write 1's to S1, for debug purposes
    data_to_send[2] = 0xFF;
    data_to_send[3] = 0xFF;
    data_to_send[4] = ID_RF;
    data_to_send[5] = (uint8_t) x_acc;
    data_to_send[6] = (uint8_t) (x_acc >> 8);
    data_to_send[7] = (uint8_t) y_acc;
    data_to_send[8] = (uint8_t) (y_acc >> 8);
    data_to_send[9] = (uint8_t) z_acc;
    data_to_send[10] = (uint8_t) (z_acc>>8);
    data_to_send[11] = adc_value;
    rf_send(data_to_send);
  }  
    
  if((NRF_TIMER2->EVENTS_COMPARE[0]==1) && (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
  {
    NRF_TIMER2->EVENTS_COMPARE[0]=0;
  }
}