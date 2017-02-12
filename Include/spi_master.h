/****************************************************** 
 * File : spi_master.h                                *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the SPI *
 * transmission, used initialize the IMU and get the  *
 * acceleratios values                                *
 *                                                    *
 ******************************************************/

#ifndef SPI_MASTER_H
#define SPI_MASTER_H

/************************
*       INCLUDES        *
*************************/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "common.h"
#include "spi_master_config.h" 

/************************
*    GLOBAL VARIABLES   *
*************************/


/**
 *  SPI master operating frequency
 */
typedef enum
{
    Freq_125Kbps = 0,        /*!< drive SClk with frequency 125Kbps */
    Freq_250Kbps,            /*!< drive SClk with frequency 250Kbps */
    Freq_500Kbps,            /*!< drive SClk with frequency 500Kbps */
    Freq_1Mbps,              /*!< drive SClk with frequency 1Mbps */
    Freq_2Mbps,              /*!< drive SClk with frequency 2Mbps */
    Freq_4Mbps,              /*!< drive SClk with frequency 4Mbps */
    Freq_8Mbps               /*!< drive SClk with frequency 8Mbps */
      
} SPIFrequency_t;

/**
 *  SPI master module number
 */
typedef enum
{
    SPI0 = 0,               /*!< SPI module 0 */
    SPI1                    /*!< SPI module 1 */
      
} SPIModuleNumber;

/**
 *  SPI mode
 */

typedef enum
{
    //------------------------Clock polarity 0, Clock starts with level 0-------------------------------------------
  
    SPI_MODE0 = 0,          /*!< Sample data at rising edge of clock and shift serial data at falling edge */
    SPI_MODE1,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    
    //------------------------Clock polarity 1, Clock starts with level 1-------------------------------------------
    
    SPI_MODE2,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    SPI_MODE3               /*!< Sample data at rising edge of clock and shift serial data at falling edge */
      
} SPIMode;


/************************
*       FUNCTIONS       *
*************************/

uint32_t* spi_master_init(SPIModuleNumber module_number, SPIMode mode, bool lsb_first);
/**
 * Initializes given SPI master with given configuration.
 *
 * After initializing the given SPI master with given configuration, this function also test if the
 * SPI slave is responding with the configurations by transmitting few test bytes. If the slave did not
 * respond then error is returned and contents of the rx_data are invalid.
 *
 * @param module_number SPI master number (SPIModuleNumber) to initialize.
 * @param mode SPI master mode (mode 0, 1, 2 or 3 from SPIMode)
 * @param lsb_first true if lsb is first bit to shift in/out as serial data on MISO/MOSI pins.
 * @return
 * @retval pointer to direct physical address of the requested SPI module if init was successful
 * @retval 0, if either init failed or slave did not respond to the test transfer
 */

bool spi_master_tx_rx(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data);
/**
 * Transfer/Receive data over SPI bus.
 *
 * If TWI master detects even one NACK from the slave or timeout occurs, STOP condition is issued
 * and the function returns false.
 *
 * @note Make sure at least transfer_size number of bytes is allocated in tx_data/rx_data.
 *
 * @param spi_base_address  register base address of the selected SPI master module
 * @param transfer_size  number of bytes to transmit/receive over SPI master
 * @param tx_data pointer to the data that needs to be transmitted
 * @param rx_data pointer to the data that needs to be received
 * @return
 * @retval true if transmit/reveive of transfer_size were completed.
 * @retval false if transmit/reveive of transfer_size were not complete and tx_data/rx_data points to invalid data.
 */

void init_IMU(void);

/*
 * Initialization of the IMU.
 *
 *
 * @param : None
 * @return : None
 */

bool write_data(uint8_t data, uint8_t adress );
/*
 * Write data in registers of the IMU. (inertial central)
 *
 *
 * @param address :  register base address of the MEMS
 * @param data : data that needs to be transmitted
 
 * @return
 * @retval true if transmit task were completed.
 * @retval false if transmit task were not completed and data points to invalid data.
 */

bool read_data(uint8_t adress);

/*
 * Read data in registers of the MEMs. (inertial central)
 *
 *
 * @param address :  register base address of the MEMS
 * @param data : data that needs to be read
 
 * @return
 * @retval true if read task were completed.
 * @retval false if read task were not completed and data points to invalid data.
 */

bool read_ac_value(int16_t* x_acceleration,int16_t* y_acceleration,int16_t* z_acceleration);

/*
 * Asks the MEMS to get accelerations values on x, y and z directions
 *
 *
 * @param x_acceleration : pointer to data of acceleration on x direction
 * @param y_acceleration : pointer to data of acceleration on y direction
 * @param z_acceleration : pointer to data of acceleration on z direction
 
 * @return
 * @retval true if read task were completed.
 * @retval false if read task were not completed and data points to invalid data.
 */
 
#endif /* SPI_MASTER_H */
