



#include "stm32wbxx_hal.h"
#include <stdint.h>
#include "ism330.h"


extern I2C_HandleTypeDef hi2c3;


HAL_StatusTypeDef ism330_Init(void)
{
    HAL_StatusTypeDef ret;

    // CTRL3_C: Enable Block Data Update (BDU) and auto-increment
    uint8_t ctrl3_c = 0x44;  // BDU = 1 (bit6), IF_INC = 1 (bit2)
    ret = HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, ISM330_CTRL3_C,
                            I2C_MEMADD_SIZE_8BIT, &ctrl3_c, 1, 100);
    if (ret != HAL_OK) return ret;

    // CTRL1_XL: Accelerometer ODR = 104 Hz, FS = ±2g
    uint8_t ctrl1_xl = 0x40; // ODR_XL = 104 Hz (0100), FS = ±2g (00)
    ret = HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, ISM330_CTRL1_XL,
                            I2C_MEMADD_SIZE_8BIT, &ctrl1_xl, 1, 100);
    if (ret != HAL_OK) return ret;

    // CTRL2_G: Gyroscope ODR = 104 Hz, FS = ±250 dps
    uint8_t ctrl2_g = 0x40;  // ODR_G = 104 Hz (0100), FS = ±250 dps (00)
    ret = HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, ISM330_CTRL2_G,
                            I2C_MEMADD_SIZE_8BIT, &ctrl2_g, 1, 100);
    if (ret != HAL_OK) return ret;

    return HAL_OK;
}




int32_t ism330_GetID(void){

	int32_t ret;
	uint8_t TXData = 0;
	uint8_t RXData = 0;


	TXData = 0x0F;

	ret = HAL_I2C_Master_Transmit(&hi2c3, ISM330_I2C_ADD, &TXData, 1, 100);
	ret = HAL_I2C_Master_Receive(&hi2c3, ISM330_I2C_ADD, &RXData, 1, 100);


	//NMEA_MSG(2);
	//printf("[RTech]ISM330: I2C W/R Return: %lX, Data = %x \n", ret, RXData);

	return ret;
}


// Buffer to hold raw IMU data
int16_t* ism330_ReadIMU(void)
{
    static int16_t imu_data[6];  // [accX, accY, accZ, gyroX, gyroY, gyroZ]
    uint8_t raw_data[12];        // 6 bytes each for gyro and accel
    HAL_StatusTypeDef ret;

    // Read 12 bytes starting from OUTX_L_G (gyro) to OUTZ_H_A (accel)
    ret = HAL_I2C_Mem_Read(&hi2c3, ISM330_I2C_ADD, ISM330_OUTX_L_G,
                           I2C_MEMADD_SIZE_8BIT, raw_data, 12, 100);
    if (ret != HAL_OK) {
        return NULL;  // Error reading IMU
    }

    // Parse gyro data (first 6 bytes)
    imu_data[3] = (int16_t)(raw_data[1] << 8 | raw_data[0]);  // gyroX
    imu_data[4] = (int16_t)(raw_data[3] << 8 | raw_data[2]);  // gyroY
    imu_data[5] = (int16_t)(raw_data[5] << 8 | raw_data[4]);  // gyroZ

    // Parse accel data (next 6 bytes)
    imu_data[0] = (int16_t)(raw_data[7] << 8 | raw_data[6]);  // accX
    imu_data[1] = (int16_t)(raw_data[9] << 8 | raw_data[8]);  // accY
    imu_data[2] = (int16_t)(raw_data[11] << 8 | raw_data[10]); // accZ

    return imu_data;
}
