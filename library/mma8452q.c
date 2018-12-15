//*****************************************************************************
//
//  API for the MMA8452Q sensor.
//  File:     mma8452q.c
//  Version:  1.0v
//  Author:   Ronald Rodriguez Ruiz.
//  Date:     October 20, 2017.
//  ---------------------------------------------------------------------------
//  Specifications:
//  Runs on 8-bit AVR Microcontrollers (ATmega series).
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include "i2c.h"

//*****************************************************************************
//
//  The following are defines for MMA8452Q address device configuration.
//
//*****************************************************************************

#define MMA8452Q_ADDRESS_R                              0x3B
#define MMA8452Q_ADDRESS_W                              0x3A
#define MMA8452Q_ID                                     0x2A

//*****************************************************************************
//
//  The following are defines for MMA8452Q registers.
//
//*****************************************************************************

#define STATUS_MMA8452Q                                 0x00
#define OUT_X_MSB                                       0x01
#define OUT_X_LSB                                       0x02
#define OUT_Y_MSB                                       0x03
#define OUT_Y_LSB                                       0x04
#define OUT_Z_MSB                                       0x05
#define OUT_Z_LSB                                       0x06
#define SYSMOD                                          0x0B
#define INT_SOURCE                                      0x0C
#define WHO_AM_I                                        0x0D
#define XYZ_DATA_CFG                                    0x0E
#define HP_FILTER_CUTOFF                                0x0F
#define PL_STATUS                                       0x10
#define PL_CFG                                          0x11
#define PL_COUNT                                        0x12
#define PL_BF_ZCOMP                                     0x13
#define P_L_THS_REG                                     0x14
#define FF_MT_CFG                                       0x15
#define FF_MT_SRC                                       0x16
#define FF_MT_THS                                       0x17
#define FF_MT_COUNT                                     0x18
#define TRANSIENT_CFG                                   0x1D
#define TRANSIENT_SRC                                   0x1E
#define TRANSIENT_THS                                   0x1F
#define TRANSIENT_COUNT                                 0x20
#define PULSE_CFG                                       0x21
#define PULSE_SRC                                       0x22
#define PULSE_THSX                                      0x23
#define PULSE_THSY                                      0x24
#define PULSE_THSZ                                      0x25
#define PULSE_TMLT                                      0x26
#define PULSE_LTCY                                      0x27
#define PULSE_WIND                                      0x28
#define ASLP_COUNT                                      0x29
#define CTRL_REG1                                       0x2A
#define CTRL_REG2                                       0x2B
#define CTRL_REG3                                       0x2C
#define CTRL_REG4                                       0x2D
#define CTRL_REG5                                       0x2E
#define OFF_X                                           0x2F
#define OFF_Y                                           0x30
#define OFF_Z                                           0x31

//*****************************************************************************
//
//  Prototypes for the private functions.
//
//*****************************************************************************

static uint8_t mma8452q_read_single_register(uint8_t reg);
static void mma8452q_write_single_register(uint8_t reg, uint8_t data);
static void mma8452q_read_multiple_registers(uint8_t reg, uint8_t *buffer, uint8_t len);
static void mma8452q_write_multiple_registers(uint8_t reg, uint8_t *buffer, uint8_t len);

//*****************************************************************************
//
//  Functions for the API.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize MMA8452Q.
//!
//! This functionconfigures the accelerometer. It sets up the scale, output
//! data rate, portrait/landscape detction and tap detection. It also checks
//! that the sensor is connected to the I2C bus.
//!
//! @param[in] scale Accelerometer scale.
//! @param[in] odr   Output data rate.
//!
//! @return Returns a 0 if communication failed, 1 if successful.
//
//*****************************************************************************
uint8_t
MMA8452Q_init(uint8_t scale, uint8_t odr)
{
  uint8_t id = mma8452q_read_single_register(WHO_AM_I);

  //
  //  WHO_AM_I should always be 0x2A.
  //
  if (id != DEVICE_ID) return 0;

  //
  //  Go to standby mode for configuration.
  //
  mma8452q_standby();
  //
  //  Set up accelerometer scale.
  //
  mma8452q_set_scale(scale);
  //
  //  Set up output data rate.
  //
  mma8452q_set_odr(odr);
  //
  //  Set up portrait/landscape detection.
  //
  mma8452q_setup_pl();
  //
  //  Activate low noise filter.
  //
  mma8452q_active_low_noise(true);

  //
  //  Go to operation mode.
  //
  mma8452q_active();
  delay(100);

  return 1;
}

//*****************************************************************************
//
//! @brief Set standby mode.
//!
//! This function sets the MMA8452 to standby mode. It must be in standby to
//! change most register settings.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_standby(void)
{
  //
  //  Get the current value of CTRL_REG1.
  //
  uint8_t config = mma8452q_read_single_register(CTRL_REG1);
  //
  //  Clear the active bit to go into standby.
  //
  mma8452q_write_single_register(CTRL_REG1, config & ~(0x01));
}

//*****************************************************************************
//
//! @brief Set full-scale range.
//!
//! This function sets the full-scale range of the x, y, and z axis. Possible
//! values for the scale variable are MMA8452Q_SCALE_2G, MMA8452Q_SCALE_4G, or
//! MMA8452Q_SCALE_8G.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_set_scale(uint8_t scale)
{
  uint8_t config = mma8452q_read_single_register(XYZ_DATA_CFG);
  config &= 0xFC;
  config |= (scale >> 2);
  mma8452q_write_single_register(XYZ_DATA_CFG, config);
}

//*****************************************************************************
//
//! @brief Set the output data rate.
//!
//! This function sets the output data rate of the MMA8452Q. Possible values for
//! the odr parameter are: MMA8452Q_ODR_800, MMA8452Q_ODR_400, MMA8452Q_ODR_200,
//! MMA8452Q_ODR_100, MMA8452Q_ODR_50, MMA8452Q_ODR_12, MMA8452Q_ODR_6, or
//! MMA8452Q_ODR_1.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_set_odr(uint8_t odr)
{
  byte config = mma8452q_read_single_register(CTRL_REG1);
  config &= 0xC7;
  config |= (odr << 3);
  mma8452q_write_single_register(CTRL_REG1, config);
}

//*****************************************************************************
//
//! @brief Active high-pass output data.
//!
//! This function actives MMA8452 high-pass filter mode.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_active_hpf(uint8_t state)
{
  uint8_t config = mma8452q_read_single_register(XYZ_DATA_CFG);
  config &= 0xEF;
  config |= (state << 4);
  mma8452q_write_single_register(XYZ_DATA_CFG, config);
}

//*****************************************************************************
//
//! @brief Set high-pass filter cutoff frequency.
//!
//! This function sets MMA8452 high-pass filter cutoff frequency.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_set_cutoff_frequency(uint8_t frequency)
{
  uint8_t config = mma8452q_read_single_register(HP_FILTER_CUTOFF);
  config &= 0xFC;
  config |= (frequency);
  mma8452q_write_single_register(HP_FILTER_CUTOFF, config);
}

//*****************************************************************************
//
//! @brief Set up portrait/landscape detection.
//!
//! This function sets up portrait and landscape detection.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_setup_pl(void)
{
  //
  //  Set PL_EN (enable).
  //
  mma8452q_write_single_register(PL_CFG, 0x40);
  //
  //  Set the debounce rate at 100ms (at 800 Hz).
  //
  mma8452q_write_single_register(PL_COUNT, 0x50);
}

//*****************************************************************************
//
//! @brief Active low noise.
//!
//! This function actives MMA8452 low noise mode.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_active_low_noise(uint8_t state)
{
  byte config = mma8452q_read_single_register(CTRL_REG1);
  config &= 0xFB;
  config |= (state << 2);
  mma8452q_write_single_register(CTRL_REG1, config);
}

//*****************************************************************************
//
//! @brief Set user offset correction.
//!
//! This function sets the MMA8452 offset correction of each axis.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_set_offsets(int8_t *axes)
{
  mma8452q_write_single_register(OFF_X, axes[0]);
  mma8452q_write_single_register(OFF_Y, axes[1]);
  mma8452q_write_single_register(OFF_Z, axes[2]);
}

//*****************************************************************************
//
//! @brief Set active mode.
//!
//! This function sets the MMA8452 to active mode. Needs to be in this mode
//! to output data.
//!
//! @return None.
//
//*****************************************************************************
void
MMA8452Q_active(void)
{
  uint8_t config = mma8452q_read_single_register(CTRL_REG1);
  mma8452q_write_single_register(CTRL_REG1, config | 0x01);
}

//*****************************************************************************
//
//! @brief Check if new data is available.
//!
//! This function checks the status of the MMA8452Q to see if new data is
//! availble returns 0 if no new data is present, or a 1 if new data is
//! available.
//!
//! @return None.
//
//*****************************************************************************
uint8_t
MMA8452Q_available(void)
{
  return (mma8452q_read_single_register(STATUS_MMA8452Q) & 0x08) >> 3;
}

//*****************************************************************************
//
//! @brief Read portrait/landscape status.
//!
//! This function reads the portrait/landscape status register of the MMA8452Q.
//! It will return either MMA8452Q_PORTRAIT_UP, MMA8452Q_PORTRAIT_DOWN,
//! MMA8452Q_LANDSCAPE_RIGHT, MMA8452Q_LANDSCAPE_LEFT, MMA8452Q_BACK,
//! MMA8452Q_FRONT.
//!
//! @return None.
//
//*****************************************************************************
uint8_t
MMA8452Q_read_pl(void)
{
  uint8_t pl_status = mma8452q_read_single_register(PL_STATUS);

  //
  //  Z-tilt lockout -> return BAFRO status.
  //
  if (pl_status & 0x40)
  {
    if(pl_status & 0x01)
      return MMA8452Q_BACK;
    else
      return MMA8452Q_FRONT;
  }
  else
  {
    //
    //  Otherwise return LAPO status.
    //
    return (pl_status & 0x06) >> 1;
  }
}

//*****************************************************************************
//
//  Private Functions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Read a single register.
//!
//! This function reads a single byte from the MMA8452Q register "reg".
//!
//! @param[in] reg Register from which the read operation is performed.
//!
//! @return result Data byte read from register "reg".
//
//*****************************************************************************
uint8_t
mma8452q_read_single_register(uint8_t reg)
{
  // WRITE MODE
  //
  //  Start Condition (take control over the SDA line).
  //
  i2c_start();

  //
  //  Send the address through the I2C bus to select MMA8452Q.
  //
  i2c_send(MMA8452Q_ADDRESS_W, SEND_ADDRESS);

  //
  //  Select the register to be read.
  //
  i2c_send(reg, SEND_DATA);

  // READ MODE
  //
  //  Repeat Start Condition.
  //
  i2c_repeat_start();

  //
  //  Send the address through the line to select MMA8452Q.
  //
  i2c_send(MMA8452Q_ADDRESS_R, SEND_ADDRESS);

  //
  //  Read the incomingbyte.
  //
  uint8_t result = i2c_read_nack();

  //
  //  Stop Condition (release SDA line)
  //
  i2c_stop();

  return result;
}

//*****************************************************************************
//
//! @brief Read multiple registers.
//!
//! This function reads "len" bytes from the MMA8452Q, starting at register "reg"
//! Bytes are stored in "buffer" on exit.
//!
//! @param[in] reg    Register from which the read operation is performed.
//! @param[in] buffer Buffer where the data bytes are stored.
//! @param[in] len    Number of bytes to be stored.
//!
//! @return None.
//
//*****************************************************************************
void
mma8452q_read_multiple_registers(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  // WRITE MODE
  //
  //  Start Condition (take control over the SDA line).
  //
  i2c_start();

  //
  //  Send the address through the I2C bus to select MMA8452Q.
  //
  i2c_send(MMA8452Q_ADDRESS_W, SEND_ADDRESS);

  //
  //  Select the register to be read.
  //
  i2c_send(reg, SEND_DATA);

  // READ MODE
  //
  //  Repeat Start Condition.
  //
  i2c_repeat_start();

  //
  //  Send the address through the line to select MMA8452Q.
  //
  i2c_send(MMA8452Q_ADDRESS_R, SEND_ADDRESS);

  for (uint8_t i = 0; i < len; i++)
  {
    //
    //  Read the incomingbyte.
    //
    if(i < (len -  1))
    {
      buffer[i] = i2c_read_ack();
    }
    else
    {
      buffer[i] = i2c_read_nack();
    }
  }

  //
  //  Stop Condition (release SDA line).
  //
  i2c_stop();
}

//*****************************************************************************
//
//! @brief Write a single register.
//!
//! This function writes a single byte of data to a register in the MMA8452Q.
//!
//! @param[in] reg  Register to which the write operation is performed.
//! @param[in] data Data byte to be written on "reg".
//!
//! @return None.
//
//*****************************************************************************
void
mma8452q_write_single_register(uint8_t reg, uint8_t data)
{
  //
  //  Start Condition (take control over the SDA line).
  //
  i2c_start();

  //
  //  Send the address through the I2C bus to select MMA8452Q.
  //
  i2c_send(MMA8452Q_ADDRESS_W, SEND_ADDRESS);

  //
  //  Select the register to be written.
  //
  i2c_send(reg, SEND_DATA);

  //
  //  Send the data.
  //
  i2c_send(data, SEND_DATA);

  //
  //  Stop Condition (release SDA line).
  //
  i2c_stop();
}

//*****************************************************************************
//
//! @brief Write multiple register.
//!
//! This function writes an array of "len" bytes ("buffer"), starting at
//! register "reg", and auto-incrmenting to the next.
//!
//! @param[in] reg    Register to which the write operation is performed.
//! @param[in] buffer Buffer holding the data bytes to be written.
//! @param[in] len    Number of bytes to be written.
//!
//! @return None.
//
//*****************************************************************************
void
mma8452q_write_multiple_registers(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  //
  //  Start Condition (take control over the SDA line).
  //
  i2c_start();

  //
  //  Send the address through the I2C bus to select MMA8452Q.
  //
  i2c_send(MMA8452Q_ADDRESS_W, SEND_ADDRESS);

  //
  //  Select the register to be written.
  //
  i2c_send(reg, SEND_DATA);

  for (int i = 0; i < len; i++)
  {
    //
    //  Send the data.
    //
    i2c_send(buffer[i], SEND_DATA);
  }

  //
  //  Stop Condition (release SDA line).
  //
  i2c_stop();
}
