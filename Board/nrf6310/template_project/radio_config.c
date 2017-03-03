/****************************************************** 
 * File : radio_config.c                              *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the RF  *
 * transmission, used to communicate the acceleration *
 * data to the front wheel.                           *
 *                                                    *
 ******************************************************/


/************************
*       INCLUDES        *
*************************/

#include "radio_config.h"

/************************
*       FUNCTIONS       *
*************************/

static uint8_t swap_bits(uint8_t inp)
{
    uint8_t i, retval = 0;
    
    for(i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);     
    }
    
    return retval;    
}

static uint32_t swap_bytes(uint32_t inp)
{
    uint8_t i, inp_byte;
    uint32_t retval = 0;

    for(i = 0; i < 4; i++)
    {
        inp_byte = (inp >> (i * 8));
        inp_byte = swap_bits(inp_byte); 
        retval |= inp_byte << ((3 * 8) - (i * 8));
    }
    return retval;
}


void radio_configure()
{
  // Radio config
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Neg12dBm << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->FREQUENCY = 7UL;           // Frequency bin 7, 2407MHz
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_250Kbit << RADIO_MODE_MODE_Pos);

  // Radio address config
  NRF_RADIO->PREFIX0 = 
      ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
    | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
    | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
    | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format
  
  NRF_RADIO->PREFIX1 = 
      ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
    | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
    | ((uint32_t)swap_bits(0xC5) << 8)  // Prefix byte of address 5 converted to nRF24L series format
    | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

  NRF_RADIO->BASE0 = swap_bytes(0x01234567UL);  // Base address for prefix 0 converted to nRF24L series format
  NRF_RADIO->BASE1 = swap_bytes(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format
  
  NRF_RADIO->TXADDRESS = 0x00UL;      // Set device address 0 to use when transmitting
  NRF_RADIO->RXADDRESSES = 0x01UL;    // Enable device address 0 to use to select which addresses to receive

  // Packet configuration
  NRF_RADIO->PCNF0 = (PACKET0_S1_SIZE << RADIO_PCNF0_S1LEN_Pos) |
                     (PACKET0_S0_SIZE << RADIO_PCNF0_S0LEN_Pos) |
                     (PACKET0_PAYLOAD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

  // Packet configuration
   NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos)    |
                      (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)           |
                      (PACKET1_BASE_ADDRESS_LENGTH << RADIO_PCNF1_BALEN_Pos)       |
                      (PACKET1_STATIC_LENGTH << RADIO_PCNF1_STATLEN_Pos)           |
                      (PACKET1_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

  // CRC Config
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
  if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
  {
    NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value      
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
  }
  else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
  {
    NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
    NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
  }
}

void rf_send(uint8_t *packet)
{
  
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }

  // Set radio configuration parameters
  radio_configure();

  // Set payload pointer
  NRF_RADIO->PACKETPTR = (uint32_t)packet;

  packet[1] = 0xFF;                         // Write 1's to S1, for debug purposes
  packet[0] = 0x06;                         // Set Length to 5 bytes

  NRF_RADIO->EVENTS_READY = 0U;
  
  //enable buck
  nrf_gpio_pin_clear(PIN_BUCK);
  nrf_delay_us(700);
    
  // Enable radio and wait for ready
  NRF_RADIO->TASKS_TXEN = 1;

  while (NRF_RADIO->EVENTS_READY == 0U)
  {
  }
  
  // Start transmission and wait for end of packet.
  NRF_RADIO->TASKS_START = 1U;
  NRF_RADIO->EVENTS_END = 0U;
  
  while(NRF_RADIO->EVENTS_END == 0U) // Wait for end
  {
  }
  
  // Disable radio
  NRF_RADIO->EVENTS_DISABLED = 0U;
  NRF_RADIO->TASKS_DISABLE = 1U;

  while(NRF_RADIO->EVENTS_DISABLED == 0U)
  {
  }
  
   // Diseable buck
  nrf_gpio_pin_set(PIN_BUCK);
}