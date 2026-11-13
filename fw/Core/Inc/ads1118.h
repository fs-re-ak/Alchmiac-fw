/*
 * ads1118.h
 *
 *  Created on: Feb 28, 2025
 *      Author: atom2
 *
 *  ADS1118 (16-bit ADC on the mezzanine, SPI1) used to sample the PPG clip.
 *  The PPG signal comes in on ADS1118_IN0 (mezzanine J16.1) -> AIN0.
 */

#ifndef INC_ADS1118_H_
#define INC_ADS1118_H_


#include "stm32wbxx_hal.h"

#define ADS1118_CS_Pin GPIO_PIN_6
#define ADS1118_CS_GPIO_Port GPIOA


/*
 * Config register (datasheet, section 9.6.1):
 *   SS=1 (start single-shot) | MUX=100 (AIN0 vs GND) | PGA=001 (FSR +/-4.096 V)
 *   MODE=1 (single-shot) | DR=100 (128 SPS, 7.8 ms) | TS_MODE=0 (ADC)
 *   PULL_UP_EN=1 | NOP=01 (write config) | reserved=1
 */
#define ADS1118_CONFIG_PPG          0xC38B
/* NOP=00: the frame is ignored by the ADS1118, used to read the last result. */
#define ADS1118_CONFIG_READ_ONLY    0x0000

/* 1 LSB = FSR / 2^15 = 4.096 V / 32768 = 125 uV */
#define ADS1118_LSB_UV              125


/*
 * PPG sampling paced by the ADS1299 DRDY (250 SPS).
 * One PPG sample every PPG_DECIMATION EEG samples -> 50 Hz,
 * i.e. 2 PPG samples per 10-sample EEG packet.
 */
#define PPG_DECIMATION              5
#define PPG_SAMPLE_SIZE             2   /* int16, big-endian */


/*
 * No init function on purpose: the ADS1118 must not be accessed before the
 * ADS1299 is configured (see ads1118.c). Only call these from the DRDY interrupt.
 */
HAL_StatusTypeDef ADS1118_StartConversion(void);
int16_t ADS1118_ReadConversion(void);

/*
 * To be called once per EEG sample, from the DRDY interrupt, right after the ADS1299 read.
 * eeg_sample_index : index of the EEG sample in the packet (0..N-1)
 * ppg_field        : start of the PPG area of the packet
 * Starts a conversion on index % PPG_DECIMATION == 0 and stores the result
 * on index % PPG_DECIMATION == PPG_DECIMATION-1.
 */
void ADS1118_PPG_OnEegSample(uint8_t eeg_sample_index, uint8_t *ppg_field);


#endif /* INC_ADS1118_H_ */
