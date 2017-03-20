/****************************************************** 
 * File : radio_config.h                              *
 * Author : Vincent FORTINEAU, Cyril RIOCHE, Fabian   *
 *          LAPOTRE                                   *
 *                                                    *
 * This file contains all the functions about the RF  *
 * transmission, used to communicate the acceleration *
 * data to the front wheel.                           *
 *                                                    *
 ******************************************************/


#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

/************************
*       INCLUDES        *
*************************/

#include "nrf_gpio.h"
#include "nrf_delay.h"


/************************
*       DEFINES         *
*************************/

#define PACKET0_S1_SIZE                  (0UL)  //!< S1 size in bits
#define PACKET0_S0_SIZE                  (0UL)  //!< S0 size in bits
#define PACKET0_PAYLOAD_SIZE             (0UL)  //!< payload size in bits
#define PACKET1_BASE_ADDRESS_LENGTH      (4UL)  //!< base address length in bytes
#define PACKET1_STATIC_LENGTH            (13UL)  //!< static length in bytes
#define PACKET1_PAYLOAD_SIZE             (13UL)  //!< payload size in bits


/************************
*       FUNCTIONS       *
*************************/

static uint8_t swap_bits(uint8_t inp);
/**
 * Swap / mirror bits in a byte.
 *
 * output_bit_7 = input_bit_0
 * output_bit_6 = input_bit_1
 *           :
 * output_bit_0 = input_bit_7
 *
 * @param inp is the input byte to be swapped.
 *
 * @return
 * Returns the swapped / mirrored input byte.
 */

static uint32_t swap_bytes(uint32_t input);
/**
 * Swap bytes in a 32 bit word.
 *
 * The bytes are swapped as follows:
 *
 * output[31:24] = input[7:0] 
 * output[23:16] = input[15:8]
 * output[15:8]  = input[23:16]
 * output[7:0]   = input[31:24]
 *
 * @param input is the input word to be swapped.
 *
 * @return
 * Returns the swapped input byte.
 */

void radio_configure(void);
/*
 * Configure the radio transmission protocol
 *
 * @param : None
 * @return : None
 */

void rf_send(uint8_t *packet);
/*
 * Send the given packet by RF transmission
 *
 * @param packet : pointer to the packet to send
 * @return : None
 */

#endif