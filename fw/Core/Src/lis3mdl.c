



#include "stm32wbxx_hal.h"
#include <stdint.h>
#include "lis3mdl.h"


extern I2C_HandleTypeDef hi2c3;

HAL_StatusTypeDef lis3mdl_Init(void)
{
    HAL_StatusTypeDef ret;

    // CTRL_REG1: Temp enable = 0, Ultra-high-performance XY, ODR = 80 Hz
    uint8_t ctrl_reg1 = 0x70;  // 0b01110000
    ret = HAL_I2C_Mem_Write(&hi2c3, LIS3MDL_I2C_ADD, LIS3MDL_CTRL_REG1,
                            I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, 100);
    if (ret != HAL_OK) return ret;

    // CTRL_REG2: FS = ±4 gauss (00)
    uint8_t ctrl_reg2 = 0x00;
    ret = HAL_I2C_Mem_Write(&hi2c3, LIS3MDL_I2C_ADD, LIS3MDL_CTRL_REG2,
                            I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);
    if (ret != HAL_OK) return ret;

    // CTRL_REG3: Continuous-conversion mode
    uint8_t ctrl_reg3 = 0x00;
    ret = HAL_I2C_Mem_Write(&hi2c3, LIS3MDL_I2C_ADD, LIS3MDL_CTRL_REG3,
                            I2C_MEMADD_SIZE_8BIT, &ctrl_reg3, 1, 100);
    if (ret != HAL_OK) return ret;

    return HAL_OK;
}

int32_t lis3mdl_GetID(void)
{
    uint8_t id = 0;
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c3, LIS3MDL_I2C_ADD, LIS3MDL_WHO_AM_I_REG,
                           I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
    if (ret != HAL_OK) {
        return -1;
    }

    return (int32_t)id;  // Should return 0x3D
}

int16_t* lis3mdl_ReadMag(void)
{
    static int16_t mag_data[3];  // [magX, magY, magZ]
    uint8_t raw_data[6];
    HAL_StatusTypeDef ret;

    // Read 6 bytes starting from OUTX_L
    ret = HAL_I2C_Mem_Read(&hi2c3, LIS3MDL_I2C_ADD, LIS3MDL_OUTX_L,
                           I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);
    if (ret != HAL_OK) {
        return NULL;
    }

    mag_data[0] = (int16_t)(raw_data[1] << 8 | raw_data[0]);  // X
    mag_data[1] = (int16_t)(raw_data[3] << 8 | raw_data[2]);  // Y
    mag_data[2] = (int16_t)(raw_data[5] << 8 | raw_data[4]);  // Z

    return mag_data;
}
