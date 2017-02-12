/****************************************************** 
 * File : spi_master.c                                *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the SPI *
 * transmission, used initialize the IMU and get the  *
 * acceleratios values                                *
 *                                                    *
 ******************************************************/


/************************
*       INCLUDES        *
*************************/

#include "spi_master.h"


/************************
*       FUNCTIONS       *
*************************/

uint32_t* spi_master_init(SPIModuleNumber module_number, SPIMode mode, bool lsb_first)
{
    /* Declaration */
  
    uint32_t config_mode;

    NRF_SPI_Type *spi_base_address = (SPI0 == module_number)? NRF_SPI0 : (NRF_SPI_Type *)NRF_SPI1;

    if(SPI0 == module_number)
    {
        /* Configure GPIO pins used for pselsck, pselmosi, pselmiso and pselss for SPI0 */
        nrf_gpio_cfg_output(SPI_PSELSCK0);
        nrf_gpio_cfg_output(SPI_PSELMOSI0);
        nrf_gpio_cfg_input(SPI_PSELMISO0, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_output(SPI_PSELSS0);

        /* Configure pins, frequency and mode */
        spi_base_address->PSELSCK  = SPI_PSELSCK0;
        spi_base_address->PSELMOSI = SPI_PSELMOSI0;
        spi_base_address->PSELMISO = SPI_PSELMISO0;
        nrf_gpio_pin_set(SPI_PSELSS0); /* disable Set slave select (inactive high) */
    }
    else
    {
        /* Configure GPIO pins used for pselsck, pselmosi, pselmiso and pselss for SPI1*/
        nrf_gpio_cfg_output(SPI_PSELSCK1);
        nrf_gpio_cfg_output(SPI_PSELMOSI1);
        nrf_gpio_cfg_input(SPI_PSELMISO1, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_output(SPI_PSELSS1);

        /* Configure pins, frequency and mode */
        spi_base_address->PSELSCK  = SPI_PSELSCK1;
        spi_base_address->PSELMOSI = SPI_PSELMOSI1;
        spi_base_address->PSELMISO = SPI_PSELMISO1;
        nrf_gpio_pin_set(SPI_PSELSS1);         /* disable Set slave select (inactive high) */
    }

    spi_base_address->FREQUENCY = (uint32_t) SPI_OPERATING_FREQUENCY;

    switch (mode )
    {
        /*lint -e845 -save // A zero has been given as right argument to operator '!'" */
        case SPI_MODE0:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case SPI_MODE1:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case SPI_MODE2:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        case SPI_MODE3:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        default:
            config_mode = 0;
            break;
        /*lint -restore */
    }
    if (lsb_first)
    {
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        spi_base_address->CONFIG = (config_mode | (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos));
    }
    else
    {
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        spi_base_address->CONFIG = (config_mode | (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos));
    }

    spi_base_address->EVENTS_READY = 0U;

    /* Enable */
    spi_base_address->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

    return (uint32_t *)spi_base_address;
}

bool spi_master_tx_rx(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data)
{
  
    /* Declarations */
  
    uint32_t counter = 0;
    uint16_t number_of_txd_bytes = 0;
    uint32_t SEL_SS_PINOUT;
    
    /*lint -e{826} //Are too small pointer conversion */
    NRF_SPI_Type *spi_base = (NRF_SPI_Type *)spi_base_address;

    if( (uint32_t *)NRF_SPI0 == spi_base_address)
    {
        SEL_SS_PINOUT = SPI_PSELSS0;
    }
    else
    {
        SEL_SS_PINOUT = SPI_PSELSS1;
    }

    /* enable slave (slave select active low) */
    nrf_gpio_pin_clear(SEL_SS_PINOUT);

    while(number_of_txd_bytes < transfer_size)
    {
        spi_base->TXD = (uint32_t)(tx_data[number_of_txd_bytes]);

        /* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
        while ((spi_base->EVENTS_READY == 0U) && (counter < TIMEOUT_COUNTER))
        {
            counter++;
        }

        if (counter == TIMEOUT_COUNTER)
        {
            /* timed out, disable slave (slave select active low) and return with error */
            nrf_gpio_pin_set(SEL_SS_PINOUT);
            return false;
        }
        else
        {   /* clear the event to be ready to receive next messages */
            spi_base->EVENTS_READY = 0U;
        }

        rx_data[number_of_txd_bytes] = (uint8_t)spi_base->RXD;
        number_of_txd_bytes++;
    };

    /* disable slave (slave select active low) */
    nrf_gpio_pin_set(SEL_SS_PINOUT);

    return true;
}

void init_IMU(void)
{
  
  write_data(0x3F,0X10);  // set accelrometre (get mesure : 52 hz; scall:+-16g filter :50hz)
  //write_data(0x33,0x10);     // set accelerometre (get mesure: 52hz scall:+-2g filter :50hz)
  read_data(0x10);        // check value 
  write_data(0x10,0x15);  // disable high-performance mode for accelerometre 
  
}

bool write_data(uint8_t data, uint8_t adress )
{
  /* Declarations */
  
  uint8_t tx_data[2];
  uint8_t rx_data[2];
  
  uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0,0);
  
  if (spi_base_address == 0)
  {
    return false;
  }
  
  tx_data[0] = adress&0x7F; // tranmit adress with MSB set to 0 must be changed for power consumption
  tx_data[1] = data;
  
  if(!spi_master_tx_rx(spi_base_address, 2, (const uint8_t *)tx_data, rx_data) )
    return false;
  
  return true;
}


bool read_data(uint8_t adress)
{
  uint8_t tx_data[2];
  uint8_t rx_data[2];
  uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0,0);
  
  if (spi_base_address == 0)
  {
    return false;
  }
  tx_data[0] = adress|0x80; // tranmit adress with MSB set to 1 must be changed for power consumption
  tx_data[1] = 0;
  
  if(!spi_master_tx_rx(spi_base_address, 2 , (const uint8_t *)tx_data, rx_data) )
    return false;
  return true;
}

bool read_ac_value(int16_t* x_acceleration,int16_t* y_acceleration,int16_t* z_acceleration)
{
  uint8_t tx_data[2];
  uint8_t rx_data[7];
  uint32_t *spi_base_address = spi_master_init(SPI0, SPI_MODE0, 0);
  
  if (spi_base_address == 0)
  {
    return false;
  }
  tx_data[0] = 0xA8;
  tx_data[1] = 0;
  if(!spi_master_tx_rx(spi_base_address, 7 , (const uint8_t *)tx_data, rx_data) )
    return false;
  
  *x_acceleration = (((int16_t)rx_data[2]) << 8) + (int16_t)rx_data[1];
  *y_acceleration = (((int16_t)rx_data[4]) << 8) + (int16_t)rx_data[3];
  *z_acceleration = (((int16_t)rx_data[6]) << 8) + (int16_t)rx_data[5];
  
  return true;
}
