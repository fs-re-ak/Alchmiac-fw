/**
  ******************************************************************************
  * @file           : ism330.h
  * @brief          : ISM330DHCX function package
  * @Author			: Fred Simard
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef INC_ISM330_H
#define INC_ISM330_H

// Includes
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "main.h"


// Device I2C address
#define ISM330_I2C_ADD                    0xD7

// Device ID
#define ISM330_ID                           0x6B

// Register Addresses

#define ISM330_FUNC_CFG_ACCESS              0x01
#define ISM330_PIN_CTRL                     0x02
#define ISM330_FIFO_CTRL1                   0x07
#define ISM330_FIFO_CTRL2                   0x08
#define ISM330_FIFO_CTRL3                   0x09
#define ISM330_FIFO_CTRL4                   0x0A
#define ISM330_COUNTER_BDR_REG1             0x0B
#define ISM330_COUNTER_BDR_REG2             0x0C
#define ISM330_INT1_CTRL                    0x0D
#define ISM330_INT2_CTRL                    0x0E
#define ISM330_WHO_AM_I                     0x0F
#define ISM330_CTRL1_XL                     0x10
#define ISM330_CTRL2_G                      0x11
#define ISM330_CTRL3_C                      0x12
#define ISM330_CTRL4_C                      0x13
#define ISM330_CTRL5_C                      0x14
#define ISM330_CTRL6_C                      0x15
#define ISM330_CTRL7_G                      0x16
#define ISM330_CTRL8_XL                     0x17
#define ISM330_CTRL9_XL                     0x18
#define ISM330_CTRL10_C                     0x19
#define ISM330_ALL_INT_SRC                  0x1A
#define ISM330_WAKE_UP_SRC                  0x1B
#define ISM330_TAP_SRC                      0x1C
#define ISM330_D6D_SRC                      0x1D
#define ISM330_STATUS_REG                   0x1E
#define ISM330_STATUS_SPIAUX                0x1E
#define ISM330_OUT_TEMP_L                   0x20
#define ISM330_OUT_TEMP_H                   0x21
#define ISM330_OUTX_L_G                     0x22
#define ISM330_OUTX_H_G                     0x23
#define ISM330_OUTY_L_G                     0x24
#define ISM330_OUTY_H_G                     0x25
#define ISM330_OUTZ_L_G                     0x26
#define ISM330_OUTZ_H_G                     0x27
#define ISM330_OUTX_L_A                     0x28
#define ISM330_OUTX_H_A                     0x29
#define ISM330_OUTY_L_A                     0x2A
#define ISM330_OUTY_H_A                     0x2B
#define ISM330_OUTZ_L_A                     0x2C
#define ISM330_OUTZ_H_A                     0x2D
#define ISM330_EMB_FUNC_STATUS_MAINPAGE     0x35
#define ISM330_FSM_STATUS_A_MAINPAGE        0x36
#define ISM330_FSM_STATUS_B_MAINPAGE        0x37
#define ISM330_MLC_STATUS_MAINPAGE     		0x38
#define ISM330_STATUS_MASTER_MAINPAGE       0x39
#define ISM330_FIFO_STATUS1                 0x3A
#define ISM330_FIFO_STATUS2                 0x3B
#define ISM330_TIMESTAMP0                   0x40
#define ISM330_TIMESTAMP1                   0x41
#define ISM330_TIMESTAMP2                   0x42
#define ISM330_TIMESTAMP3                   0x43
#define ISM330_TAP_CFG0                     0x56

#define FIFO_DATA_OUT_TAG                     0x78

typedef struct {
    int16_t acc[3];   // [X, Y, Z]
    int16_t gyro[3];  // [X, Y, Z]
} IMU_Sample_t;



// ADS1299 Functions
HAL_StatusTypeDef ism330_Init(void);
int32_t ism330_GetID(void);
int16_t* ism330_ReadIMU(void);



#endif /* INC_ISM330_I2C_H_ */
