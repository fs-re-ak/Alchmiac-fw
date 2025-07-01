/**
  ******************************************************************************
  * @file           : lis3mdl.h
  * @brief          : ISM330DHCX function package
  * @Author			: Fred Simard
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef LIS3MDL_H
#define LIS3MDL_H

// Includes
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "main.h"


#define LIS3MDL_I2C_ADD  (0x1E << 1)  // 0x3C 7-bit address shifted left for STM HAL
#define LIS3MDL_WHO_AM_I_REG  0x0F
#define LIS3MDL_WHO_AM_I_VAL  0x3D

#define LIS3MDL_CTRL_REG1     0x20
#define LIS3MDL_CTRL_REG2     0x21
#define LIS3MDL_CTRL_REG3     0x22

#define LIS3MDL_OUTX_L        0x28  // Start address of output data

HAL_StatusTypeDef lis3mdl_Init(void);
int32_t lis3mdl_GetID(void);
int16_t* lis3mdl_ReadMag(void);





#endif
