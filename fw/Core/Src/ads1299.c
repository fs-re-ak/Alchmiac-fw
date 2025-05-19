/*
 * ads1299.c
 *
 *  Created on: May 19, 2025
 *      Author: atom2
 */




#include "stm32wbxx_hal.h"
#include "ads1299.h"
#include "gpios.h"




#define POWER_UP_DELAY_MS 200

extern SPI_HandleTypeDef hspi1;

static void setnReset(uint8_t value);
static void setnPWRdown(uint8_t value);
static void setADS1299CS(uint8_t value);
static void setADS1299Start(uint8_t value);

static uint8_t send_ads1299_command(uint8_t command);
static uint8_t write_ads1299_register(uint8_t start_addr, uint8_t num_regs, uint8_t *data);


//static void GPIOsDefaultState();
static void PowerUpSequence();



void ADS1299_Init(){

    //send_ads1299_command(ADS1299_SDATAC);
	PowerUpSequence();

}



static void PowerUpSequence(){

	uint8_t i = 0;
	uint8_t register_value = 0x00;
	uint8_t channset_value[ADS1299_CHANNELS] = {0};

	setnReset(GPIO_PIN_RESET);
	setnPWRdown(GPIO_PIN_RESET);
	setADS1299Start(GPIO_PIN_RESET);
	setADS1299CS(GPIO_PIN_SET);

	// Minimum delay after power on.
	HAL_Delay(POWER_UP_DELAY_MS);

	setnReset(GPIO_PIN_SET);
	setnPWRdown(GPIO_PIN_SET);
	HAL_Delay(1);

	setnReset(GPIO_PIN_RESET);
	HAL_Delay(1);
	setnReset(GPIO_PIN_SET);

	// Minimum delay after power on.
	HAL_Delay(POWER_UP_DELAY_MS);

	send_ads1299_command(ADS1299_SDATAC);

	register_value = 0x96;
	write_ads1299_register(ADS1299_REG_CONFIG1, 1, &register_value);
	register_value = 0xC0;
	write_ads1299_register(ADS1299_REG_CONFIG2, 1, &register_value);
	register_value = 0xE0;
	write_ads1299_register(ADS1299_REG_CONFIG3, 1, &register_value);

	for(i=0;i<ADS1299_CHANNELS;i++){
		channset_value[i] = 0x01;
	}

	write_ads1299_register(ADS1299_REG_CH1SET, ADS1299_CHANNELS, channset_value);

	setADS1299Start(GPIO_PIN_SET);

	// Wait 1ms.
	HAL_Delay(1);

	send_ads1299_command(ADS1299_RDATAC);

	// Wait 1ms.
	HAL_Delay(20);

	send_ads1299_command(ADS1299_SDATAC);
	register_value = 0xD0;
	write_ads1299_register(ADS1299_REG_CONFIG2, 1, &register_value);

	for(i=0;i<ADS1299_CHANNELS;i++){
		channset_value[i] = 0x55;
	}

	write_ads1299_register(ADS1299_REG_CH1SET, ADS1299_CHANNELS, channset_value);

	HAL_Delay(1);

	send_ads1299_command(ADS1299_RDATAC);

}





void EEGRecordingSequence(){

	uint8_t i = 0;
	uint8_t register_value = 0x00;
	uint8_t channset_value[ADS1299_CHANNELS] = {0};

	// Wait 1ms.
	HAL_Delay(20);

	send_ads1299_command(ADS1299_SDATAC);
	register_value = 0xD0;
	write_ads1299_register(ADS1299_REG_CONFIG2, 1, &register_value);
	register_value = 0xEC;
	write_ads1299_register(ADS1299_REG_CONFIG3, 1, &register_value);
	register_value = 0x0F;
	write_ads1299_register(ADS1299_REG_BIAS_SENSP, 1, &register_value);
	register_value = 0x20;
	write_ads1299_register(ADS1299_REG_MISC1, 1, &register_value);

	for(i=0;i<4;i++){
		channset_value[i] = 0x60;
	}

	for(i=4;i<ADS1299_CHANNELS;i++){
		channset_value[i] = 0x01;
	}

	write_ads1299_register(ADS1299_REG_CH1SET, ADS1299_CHANNELS, channset_value);

	HAL_Delay(1);

	send_ads1299_command(ADS1299_RDATAC);

}


/*
void EEGRecordingSequence(){

	uint8_t i = 0;
	uint8_t register_value = 0x00;
	uint8_t channset_value[ADS1299_CHANNELS] = {0};

  // Wait 1ms.
  HAL_Delay(20);

  send_ads1299_command(ADS1299_SDATAC);
  register_value = 0xC0;
  write_ads1299_register(ADS1299_REG_CONFIG2, 1, &register_value);
  register_value = 0xEC;
  write_ads1299_register(ADS1299_REG_CONFIG3, 1, &register_value);
  register_value = 0xFF;
  write_ads1299_register(ADS1299_REG_BIAS_SENSP, 1, &register_value);
  register_value = 0x20;
  write_ads1299_register(ADS1299_REG_MISC1, 1, &register_value);



  for(i=0;i<ADS1299_CHANNELS;i++){
    channset_value[i] = 0x50;
  }

  write_ads1299_register(ADS1299_REG_CH1SET, ADS1299_CHANNELS, channset_value);

  HAL_Delay(1);

  send_ads1299_command(ADS1299_RDATAC);

}*/




static void setnReset(uint8_t value){
	HAL_GPIO_WritePin(ADS1299_nRESET_GPIO_Port, ADS1299_nRESET_Pin, value);
}

static void setnPWRdown(uint8_t value){
	HAL_GPIO_WritePin(ADS1299_nPWDN_GPIO_Port, ADS1299_nPWDN_Pin, value);
}

static void setADS1299Start(uint8_t value){
	HAL_GPIO_WritePin(ADS1299_START_GPIO_Port, ADS1299_START_Pin, value);
}

static void setADS1299CS(uint8_t value){
	HAL_GPIO_WritePin(ADS1299_CS_GPIO_Port, ADS1299_CS_Pin, value);
}

static uint8_t send_ads1299_command(uint8_t command)
{
    // Pull CS (Chip Select) low to begin the SPI communication
	setADS1299CS(GPIO_PIN_RESET);  // Assuming CS is on GPIOA pin 4

    // Transmit the command to ADS1299
    if (HAL_SPI_Transmit(&hspi1, &command, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    	setADS1299CS(GPIO_PIN_SET);
        return HAL_ERROR;  // Return error if transmission fails
    }

    // Pull CS (Chip Select) high to end the SPI communication
    setADS1299CS(GPIO_PIN_SET);

    return HAL_OK;  // Return success
}



uint8_t read_ads1299_register(uint8_t start_addr, uint8_t num_regs, uint8_t *buffer)
{

    // Buffer to hold the command sequence
    uint8_t tx_buffer[2];
    tx_buffer[0] = ADS1299_RREG | start_addr;  // Read command with starting register address
    tx_buffer[1] = num_regs - 1;                       // Number of registers to read (0-indexed)

    setADS1299CS(GPIO_PIN_RESET);

    // Transmit the command to ADS1299
    if (HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY) != HAL_OK)
    {
    	HAL_GPIO_WritePin(ADS1299_CS_GPIO_Port, ADS1299_CS_Pin, GPIO_PIN_SET);
        return HAL_ERROR;  // Return error if transmission fails
    }

    // Receive the data from the ADS1299
    if (HAL_SPI_Receive(&hspi1, buffer, num_regs, HAL_MAX_DELAY) != HAL_OK)
    {
    	  HAL_GPIO_WritePin(ADS1299_CS_GPIO_Port, ADS1299_CS_Pin, GPIO_PIN_SET);
        return HAL_ERROR;  // Return error if reception fails
    }

    setADS1299CS(GPIO_PIN_SET);

    return HAL_OK;
}



static uint8_t write_ads1299_register(uint8_t start_addr, uint8_t num_regs, uint8_t *data)
{
    // Buffer to hold the command sequence
    uint8_t tx_buffer[2];
    tx_buffer[0] = ADS1299_WREG | start_addr;  // Write command with starting register address
    tx_buffer[1] = num_regs - 1;                        // Number of registers to write (0-indexed)

    // Pull CS (Chip Select) low to begin the SPI communication
    setADS1299CS(GPIO_PIN_RESET);  // Assuming CS is on GPIOA pin 4

    // Transmit the write command and register address
    if (HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // End SPI communication
        return HAL_ERROR;  // Return error if transmission fails
    }

    // Transmit the data to be written to the registers
    if (HAL_SPI_Transmit(&hspi1, data, num_regs, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // End SPI communication
        return HAL_ERROR;  // Return error if data transmission fails
    }

    setADS1299CS(GPIO_PIN_SET);

    return HAL_OK;  // Return success
}






/**
 * @brief Reads samples from the ADS1299 via SPI.
 * @param statusBuffer Pointer to a buffer to store the 3 status bytes.
 * @param dataBuffer Pointer to a buffer to store the 24 data bytes (8 channels x 3 bytes each).
 * @retval HAL_StatusTypeDef HAL status indicating success or error.
 */
HAL_StatusTypeDef ADS1299_ReadSamples(uint8_t *statusBuffer, uint8_t *dataBuffer) {

    uint8_t spiRxBuffer[27] = {0}; // Buffer to store all received bytes (3 status + 24 data)
    HAL_StatusTypeDef result;

    // Ensure that statusBuffer and dataBuffer are not NULL
    if (statusBuffer == NULL || dataBuffer == NULL) {
        return HAL_ERROR;
    }

    setADS1299CS(GPIO_PIN_RESET);

    // Perform SPI reception of 27 bytes
    result = HAL_SPI_Receive(&hspi1, spiRxBuffer, 27, HAL_MAX_DELAY);

    setADS1299CS(GPIO_PIN_SET);

    if (result != HAL_OK) {
        return result; // Return if there is an SPI error
    }

    // Copy the first 3 bytes to the status buffer
    for (int i = 0; i < 3; i++) {
        statusBuffer[i] = spiRxBuffer[i];
    }

    // Copy the next 24 bytes to the data buffer
    for (int i = 0; i < 24; i++) {
        dataBuffer[i] = spiRxBuffer[i + 3];
    }

    return HAL_OK;
}









