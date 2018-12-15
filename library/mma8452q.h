//*****************************************************************************
//
//  Prototypes for the MMA8452Q sensor.
//  File:     MMA8452Q.h
//  Version:  1.0v
//  Author:   Ronald Rodriguez Ruiz.
//  Date:     October 20, 2017.
//  ---------------------------------------------------------------------------
//  Specifications:
//  Runs on 8-bit AVR Microcontrollers (ATmega series).
//
//*****************************************************************************

#ifndef __MMA8452Q_H__
#define __MMA8452Q_H__

//*****************************************************************************
//
//  The following are defines for MMA8452Q sample scale.
//
//*****************************************************************************

#define MMA8452Q_SCALE_2G                 2
#define MMA8452Q_SCALE_4G                 4
#define MMA8452Q_SCALE_8G                 8

//*****************************************************************************
//
//  The following are defines for MMA8452Q output data rate.
//
//*****************************************************************************

#define MMA8452Q_ODR_800                   0  //  Period: 1.25 ms.
#define MMA8452Q_ODR_400                   1  //  Period: 2.5 ms.
#define MMA8452Q_ODR_200                   2  //  Period: 5 ms.
#define MMA8452Q_ODR_100                   3  //  Period: 10 ms.
#define MMA8452Q_ODR_50                    4  //  Period: 20 ms.
#define MMA8452Q_ODR_12                    5  //  Period: 80 ms.
#define MMA8452Q_ODR_6                     6  //  Period: 160 ms.
#define MMA8452Q_ODR_1                     7  //  Period: 640 ms.
};

//*****************************************************************************
//
//  The following are defines for MMA8452Q portrait/landscape settings.
//
//*****************************************************************************

#define MMA8452Q_PORTRAIT_UP              0
#define MMA8452Q_PORTRAIT_DOWN            1
#define MMA8452Q_LANDSCAPE_RIGHT          2
#define MMA8452Q_LANDSCAPE_LEFT           3
#define MMA8452Q_FRONT                    4
#define MMA8452Q_BACK                     5

//*****************************************************************************
//
//  The following are defines for MMA8452Q high-pass filter cutoff frequency.
//
//*****************************************************************************

#define MMA8452Q_HIGH_FREQ                0
#define MMA8452Q_MED_HIGH_FRECQ           1
#define MMA8452Q_MED_LOW_FREQ             2
#define MMA8452Q_LOW_FREQ                 3

//*****************************************************************************
//
//  Prototypes for the API.
//
//*****************************************************************************

extern uint8_t MMA8452Q_init(uint8_t scale, uint8_t odr);
extern uint8_t MMA8452Q_available(void);
extern void MMA8452Q_active(void);
extern void MMA8452Q_standby(void);
extern void MMA8452Q_set_odr(uint8_t odr);
extern void MMA8452Q_setup_pl(void);
extern void MMA8452Q_set_scale(uint8_t scale);
extern uint8_t MMA8452Q_read_pl(void);
extern void MMA8452Q_set_offsets(int8_t *axes);
extern void MMA8452Q_active_hpf(uint8_t state);
extern void MMA8452Q_active_low_noise(uint8_t state);
extern void MMA8452Q_set_cutoff_frequency(uint8_t frequency);

#endif
