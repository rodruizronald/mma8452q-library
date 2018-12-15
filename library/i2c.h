//*****************************************************************************
//
//  Prototypes for the I2C protocol.
//  File:     i2c.h
//  Version:  1.0v
//  Author:   Ronald Rodriguez Ruiz.
//  Date:     October 18, 2017.
//  ---------------------------------------------------------------------------
//  Specifications:
//  Runs on 8-bit AVR Microcontrollers (ATmega series).
//
//*****************************************************************************

#ifndef __I2C_H__
#define __I2C_H__

//*****************************************************************************
//
//  The following are defines for I2C commands.
//
//*****************************************************************************

#define I2C_SEND_DATA                       0
#define I2C_SEND_ADDRESS                    1

//*****************************************************************************
//
//  Prototypes for the API.
//
//*****************************************************************************

extern void I2C_init(void);
extern uint8_t I2C_start(void);
extern uint8_t I2C_repeat_start(void);
extern void I2C_stop(void);
extern uint8_t I2C_send(uint8_t data, uint8_t status_code, uint8_t mode);
extern uint8_t I2C_read_ack(void);
extern uint8_t I2C_read_nack(void);

#endif
