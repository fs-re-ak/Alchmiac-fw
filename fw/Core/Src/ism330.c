



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


/*
extern I2C_HandleTypeDef hi2c3;

HAL_StatusTypeDef ism330_Init(void)
{
	// CTRL3_C (0x12): IF_INC=1 (for multi-byte read), BDU=1 (optional)
	uint8_t reg = 0x44;
	HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, 0x12, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);

	// CTRL1_XL (0x10): ODR=0x03 (52Hz), FS=0x00 (2g)
	reg = 0x30; // 0b00110000
	HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, 0x10, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);

	// CTRL2_G (0x11): ODR=0x03 (52Hz), FS=0x00 (250dps)
	reg = 0x30; // 0b00110000
	HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, 0x11, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);

	// FIFO_CTRL1/2: Watermark = 10 (for 5 accel+5 gyro)
	reg = 10;
	HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, 0x07, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);
	reg = 0;
	HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, 0x08, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);

	// FIFO_CTRL3: BDR_GY & BDR_XL = 0x03 (52Hz batch)
	reg = 0x33;
	HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, 0x09, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);

	// FIFO_CTRL4: Stream/Continuous mode (0b110 = 6), bits 2:0
	reg = 0x06;
	HAL_I2C_Mem_Write(&hi2c3, ISM330_I2C_ADD, 0x0A, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);

    // (Optionally: set INT to watermark if you want to wait for INT instead of polling)

    return HAL_OK;
}

#define ISM330_FIFO_DATA_OUT_TAG  0x78  // Base address for FIFO data register
#define ISM330_TAG_GYRO   0x01
#define ISM330_TAG_ACCEL  0x02

#define MAX_FIFO_SAMPLES 32  // Max burst size you expect (5 is typical for 100 ms @ 50Hz)
// Extracts 5 samples (acc+gyro) from FIFO.
// NOTE: Call at 10Hz for 5 samples per poll (~52Hz sample rate).
int ism330_ReadFIFO(IMU_Sample_t *buffer, int max_samples)
{
    HAL_StatusTypeDef ret;
    uint8_t fifo_status[2];
    uint16_t fifo_level;
    int samples_found = 0;

    // Step 1: Read FIFO level (number of words currently held in FIFO)
    ret = HAL_I2C_Mem_Read(&hi2c3, ISM330_I2C_ADD, ISM330_FIFO_STATUS1, I2C_MEMADD_SIZE_8BIT, fifo_status, 2, 100);
    if (ret != HAL_OK) return -1;
    fifo_level = fifo_status[0] | ((fifo_status[1] & 0x03) << 8); // words (each 7 bytes)

    if (fifo_level > max_samples * 2) fifo_level = max_samples * 2; // Only process up to max_samples

    // Use these to hold temporary pairs. For each sample we wait until both an accel and a gyro are received.
    IMU_Sample_t temp = {0};
    uint8_t have_acc = 0, have_gyro = 0;

    for (int n = 0; n < fifo_level && samples_found < max_samples; n++) {
        uint8_t word[7];
        ret = HAL_I2C_Mem_Read(&hi2c3, ISM330_I2C_ADD, ISM330_FIFO_DATA_OUT_TAG, I2C_MEMADD_SIZE_8BIT, word, 7, 100);
        if (ret != HAL_OK)
            return -1;

        uint8_t tag = word[0] & 0x1F;

        switch (tag) {
            case ISM330_TAG_ACCEL: // 0x02
                temp.acc[0] = (int16_t)(word[2] << 8 | word[1]);
                temp.acc[1] = (int16_t)(word[4] << 8 | word[3]);
                temp.acc[2] = (int16_t)(word[6] << 8 | word[5]);
                have_acc = 1;
                break;
            case ISM330_TAG_GYRO: // 0x01
                temp.gyro[0] = (int16_t)(word[2] << 8 | word[1]);
                temp.gyro[1] = (int16_t)(word[4] << 8 | word[3]);
                temp.gyro[2] = (int16_t)(word[6] << 8 | word[5]);
                have_gyro = 1;
                break;
            // (Optionally handle timestamp, temp, SH tags here...)
            default:
                break;
        }
        // When both an accel and a gyro sample are read, store a complete IMU_Sample_t
        if (have_acc && have_gyro) {
            buffer[samples_found++] = temp;
            have_acc = 0;
            have_gyro = 0;
            // Optionally clear temp, e.g. memset(&temp, 0, sizeof(temp));
        }
    }
    return samples_found;
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
*/
