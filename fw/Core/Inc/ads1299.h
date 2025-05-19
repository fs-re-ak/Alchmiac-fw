

#ifndef ADS1299_H
#define ADS1299_H


#include "stm32wbxx_hal.h"

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

//void ADS1299_Init();

//void SetupTestMode();
//void SetupEEGMode();
//void SetupImpedanceMode();




#endif
