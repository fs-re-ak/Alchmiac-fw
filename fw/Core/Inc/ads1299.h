

#ifndef ADS1299_H
#define ADS1299_H


#include "stm32wbxx_hal.h"

// GPIOs
#define ADS1299_nDRDY_Pin GPIO_PIN_9
#define ADS1299_nDRDY_GPIO_Port GPIOC
#define ADS1299_nDRDY_EXTI_IRQn EXTI9_5_IRQn
#define ADS1299_nRESET_Pin GPIO_PIN_7
#define ADS1299_nRESET_GPIO_Port GPIOC
#define ADS1299_nPWDN_Pin GPIO_PIN_8
#define ADS1299_nPWDN_GPIO_Port GPIOC
#define ADS1299_CS_Pin GPIO_PIN_4
#define ADS1299_CS_GPIO_Port GPIOA
#define ADS1299_START_Pin GPIO_PIN_6
#define ADS1299_START_GPIO_Port GPIOC


// Generic constants
#define ADS1299_CHANNELS 8


// SPI Command Definitions (Datasheet, 35)
#define ADS1299_WAKEUP  0x02 // Wake-up from standby mode
#define ADS1299_STANDBY 0x04 // Enter Standby mode
#define ADS1299_RESET   0x06 // Reset the device registers to default
#define ADS1299_START   0x08 // Start and restart (synchronize) conversions
#define ADS1299_STOP    0x0A // Stop conversion
#define ADS1299_RDATAC  0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define ADS1299_SDATAC  0x11 // Stop Read Data Continuous mode
#define ADS1299_RDATA   0x12 // Read data by command; supports multiple read back
#define ADS1299_RREG    0x20 // Read Register
#define ADS1299_WREG    0x40 // Write to Register

// Register Addresses
#define ADS1299_REG_ID         0x00
#define ADS1299_REG_CONFIG1    0x01
#define ADS1299_REG_CONFIG2    0x02
#define ADS1299_REG_CONFIG3    0x03
#define ADS1299_REG_LOFF       0x04
#define ADS1299_REG_CH1SET     0x05
#define ADS1299_REG_CH2SET     0x06
#define ADS1299_REG_CH3SET     0x07
#define ADS1299_REG_CH4SET     0x08
#define ADS1299_REG_CH5SET     0x09
#define ADS1299_REG_CH6SET     0x0A
#define ADS1299_REG_CH7SET     0x0B
#define ADS1299_REG_CH8SET     0x0C
#define ADS1299_REG_BIAS_SENSP 0x0D
#define ADS1299_REG_BIAS_SENSN 0x0E
#define ADS1299_REG_LOFF_SENSP 0x0F
#define ADS1299_REG_LOFF_SENSN 0x10
#define ADS1299_REG_LOFF_FLIP  0x11
#define ADS1299_REG_LOFF_STATP 0x12
#define ADS1299_REG_LOFF_STATN 0x13
#define ADS1299_REG_GPIO       0x14
#define ADS1299_REG_MISC1      0x15
#define ADS1299_REG_MISC2      0x16
#define ADS1299_REG_CONFIG4    0x17



void ADS1299_Init();

//void SetupTestMode();
//void SetupEEGMode();
//void SetupImpedanceMode();

HAL_StatusTypeDef ADS1299_ReadSamples(uint8_t *statusBuffer, uint8_t *dataBuffer);
uint8_t read_ads1299_register(uint8_t start_addr, uint8_t num_regs, uint8_t *buffer);


#endif
