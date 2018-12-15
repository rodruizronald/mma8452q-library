//*****************************************************************************
//
//  API for the I2C protocol.
//  File:     i2c.c
//  Version:  1.0v
//  Author:   Ronald Rodriguez Ruiz.
//  Date:     October 18, 2017.
//  ---------------------------------------------------------------------------
//  Specifications:
//  Runs on 8-bit AVR Microcontrollers (ATmega series).
//
//*****************************************************************************

#include <stdint.h>
#include <avr/io.h>
#include "i2c.h"
#include "bitwiseop.h"

//*****************************************************************************
//
//  The following are defines for the transmission status codes.
//
//*****************************************************************************
#define START                                     0x08
#define REPEATED_START                            0x10
#define TRANSMITTER_ADDRESS_ACK                   0x18
#define TRANSMITTER_DATA_ACK                      0x28
#define RECEIVER_ADDRESS_ACK                      0x40
#define RECEIVER_DATA_ACK                         0x50
#define RECEIVER_DATA_NACK                        0x58

//*****************************************************************************
//
//  Prototypes for the private functions.
//
//*****************************************************************************

static void i2c_wait_until_complete(void);
static uint8_t i2c_transmission_status(uint8_t status_code, char* debug);

//*****************************************************************************
//
//  Functions for the API.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the I2C.
//!
//! This function initializes the GPIO pins and configures the bus speed at
//! 100 kHz for the I2C protocol.
//!
//! @return None.
//
//*****************************************************************************
void
I2C_init (void)
{
  //
  //  Set bit rate.
  //  TWBR -> 0x48 -> 72.
  //  Prescaler Value (TWPS1,TWPS0): 1.
  //  16MHz / (16+2*TWBR*(Prescaler Value)) = 100kHz.
  //
  TWBR = 0x48;
  _clear_two_bits(TWSR, TWPS1, TWPS0) ;
}

//*****************************************************************************
//
//! @brief Send start condition.
//!
//! This functions request the I2C bus to start a new transmission.
//!
//! @return status_code Transmission status codes defining the result of the
//!                     operation. If status code is different than 0, an error
//!                     occurred.
//
//*****************************************************************************
uint8_t
I2C_start(void)
{
  //
  //  Set enable and start bits, clear flag.
  //
  _set_three_bits(TWCR, TWINT, TWSTA, TWEN);
  i2c_wait_until_complete();

  return i2c_transmission_status(START);
}

//*****************************************************************************
//
//! @brief Repeat start condition.
//!
//! This functions repeats the start request for a new transmission.
//!
//! @return status_code Transmission status codes defining the result of the
//!                     operation. If status code is different than 0, an error
//!                     occurred.
//
//*****************************************************************************
uint8_t
I2C_repeat_start(void)
{
  //
  //  Set enable and start bits, clear flag.
  //
  _set_three_bits(TWCR, TWINT, TWSTA, TWEN);
  i2c_wait_until_complete();

  return i2c_transmission_status(REPEATED_START);
}

//*****************************************************************************
//
//! @brief Send data.
//!
//! This function send a data byte over the I2C bus to an specific device.
//!
//! @param[in] data    Data byte to be sent over I2C.
//! @param[in] address Address of the receiver.
//!
//! @return status_code Transmission status codes defining the result of the
//!                     operation. If status code is different than 0, an error
//!                     occurred.
//
//*****************************************************************************
uint8_t
I2C_send(uint8_t data, uint8_t address)
{
  uint8_t status_code;

  //
  //  Load data register.
  //
  TWDR = data;

  //
  //  Clear start and stop bits.
  //
  _clear_bit(TWCR, TWSTA);
  //
  //  Set enable bit and clear flag.
  //
  _set_two_bits(TWCR, TWINT, TWEN);

  i2c_wait_until_complete();

  //
  //  Check for any transmission errors
  //
  if(address)
  {
    //
    //  RECEIVER (read):  1xxxxxxx
    //  TRANSMITTER (write): 0xxxxxxx
    //
    if(!(data & 0x01))
    {
      status_code = i2c_transmission_status(TRANSMITTER_ADDRESS_ACK);
    }
    else
    {
      status_code = i2c_transmission_status(RECEIVER_ADDRESS_ACK);
    }
  }
  else
  {
    status_code = i2c_transmission_status(TRANSMITTER_DATA_ACK);
  }

  return status_code;
}

//*****************************************************************************
//
//! @brief Stop condition.
//!
//! This functions releases the I2C bus.
//!
//! @return None.
//
//*****************************************************************************
void
I2C_stop(void)
{
  //
  //  Clear start bit.
  //
  _clear_bit(TWCR, TWSTA);

  //
  //  Set enable and stop bits, clear flag.
  //
  _set_three_bits(TWCR, TWINT, TWEN, TWSTO);

  i2c_wait_until_complete();
}

//*****************************************************************************
//
//! @brief Read ACK.
//!
//! This function reads an acknowledge returned from the requested device.
//!
//! @return TWDR Data byte.
//
//*****************************************************************************
uint8_t
I2C_read_ack(void)
{
  uint8_t status_code;

  //
  //  Set enable bit and clear flag.
  //
  _set_three_bits(TWCR, TWINT, TWEN, TWEA);

  i2c_wait_until_complete();

  status_code = i2c_transmission_status(RECEIVER_DATA_ACK);

  return (TWDR);
}

//*****************************************************************************
//
//! @brief Read NACK.
//!
//! This function reads a no acknowledge returned from the requested device.
//!
//! @return TWDR Data byte.
//
//*****************************************************************************
uint8_t
I2C_read_nack(void)
{
  uint8_t status_code;

  //
  //  Clear start bit.
  //
  _clear_bit(TWCR, TWEA);

  //
  //  Set enable bit and clear flag.
  //
  _set_two_bits(TWCR, TWINT, TWEN);

  i2c_wait_until_complete();

  status_code = i2c_transmission_status(RECEIVER_DATA_NACK);

  return (TWDR);
}

//*****************************************************************************
//
//  Private functions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Wait until transmission is complete.
//!
//! @return None.
//
//*****************************************************************************
static void
i2c_wait_until_complete(void)
{
  //
  //  Wait for transmission complete.
  //
  while( !(TWCR & (1<<TWINT) ))
  {};
}

//*****************************************************************************
//
//! @brief Transmission status.
//!
//! This function reads the status register and checks through an specific
//! status code if the operation was perform succesfully.
//!
//! @param[in] status_code Operation to be checked in the status register.
//!
//! @return error Return the status code if there was an error, if not, then 0.
//
//*****************************************************************************
static uint8_t
i2c_transmission_status(uint8_t status_code)
{
  uint8_t error = 0;

  //
  //  Check TWSR to see for any error.
  //
  if((TWSR & 0xF8) != status_code)
  {
    error = status_code;
  }

  return error;
}
