/*
 * ads1118.c
 *
 *  ADS1118 driver (SPI1, CS on PA6) used to sample the PPG clip on AIN0.
 *
 *  The ADS1118 shares SPI1 with the ADS1299. The ADS1118 is ONLY accessed from
 *  the ADS1299 DRDY interrupt (ADS1118_PPG_OnEegSample), right after the
 *  ADS1299 read:
 *    - the two devices never use the bus at the same time;
 *    - nothing is sent on SPI1 before the ADS1299 driver has configured the
 *      ADS1299 (an ADS1118 access before ADS1299_Init() corrupted the ADS1299
 *      configuration: wrong data rate, misaligned samples).
 *  There is therefore no init function: every start frame writes the whole
 *  configuration, and every read checks the configuration readback.
 */

#include "stm32wbxx_hal.h"
#include "ads1118.h"


#define ADS1118_SPI_TIMEOUT_MS      2
/* Bits compared on config readback: MUX, PGA, MODE, DR, TS_MODE, PULL_UP_EN */
#define ADS1118_CONFIG_CHECK_MASK   0x7FF8
/* Stored in the packet when no valid PPG sample is available
 * (ADS1118 absent, SPI error). Cannot happen on AIN0 vs GND. */
#define PPG_INVALID_SAMPLE          ((int16_t)0x8000)

extern SPI_HandleTypeDef hspi1;

/* Set when the last start frame was sent without SPI error. */
static uint8_t conversion_started = 0;


static void setADS1118CS(GPIO_PinState value){
	HAL_GPIO_WritePin(ADS1118_CS_GPIO_Port, ADS1118_CS_Pin, value);
}

/*
 * One SPI frame of nb_words 16-bit words, MSB first.
 * The ADS1118 shifts out the last conversion result (then the config register
 * in a 32-bit frame) while it shifts in the config.
 */
static HAL_StatusTypeDef ads1118_transfer(const uint16_t *tx_words, uint16_t *rx_words, uint8_t nb_words){

	uint8_t tx[4];
	uint8_t rx[4] = {0};
	HAL_StatusTypeDef result;

	for(uint8_t i = 0; i < nb_words; i++){
		tx[2*i]     = (uint8_t)(tx_words[i] >> 8);
		tx[2*i + 1] = (uint8_t)(tx_words[i] & 0xFF);
	}

	setADS1118CS(GPIO_PIN_RESET);
	result = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(2 * nb_words), ADS1118_SPI_TIMEOUT_MS);
	setADS1118CS(GPIO_PIN_SET);

	for(uint8_t i = 0; i < nb_words; i++){
		rx_words[i] = (uint16_t)((rx[2*i] << 8) | rx[2*i + 1]);
	}

	return result;
}


/* Writes the full PPG configuration and starts a single-shot conversion (16-bit frame). */
HAL_StatusTypeDef ADS1118_StartConversion(void){

	const uint16_t tx = ADS1118_CONFIG_PPG;
	uint16_t rx = 0;

	return ads1118_transfer(&tx, &rx, 1);
}


/*
 * Reads the last conversion result with a 32-bit frame (result + config readback)
 * that does not modify the configuration (NOP=00).
 * Returns PPG_INVALID_SAMPLE on SPI error or when the readback does not match
 * the PPG configuration (ADS1118 absent or not answering).
 */
int16_t ADS1118_ReadConversion(void){

	const uint16_t tx[2] = {ADS1118_CONFIG_READ_ONLY, ADS1118_CONFIG_READ_ONLY};
	uint16_t rx[2] = {0};

	if(ads1118_transfer(tx, rx, 2) != HAL_OK){
		return PPG_INVALID_SAMPLE;
	}
	if((rx[1] & ADS1118_CONFIG_CHECK_MASK) != (ADS1118_CONFIG_PPG & ADS1118_CONFIG_CHECK_MASK)){
		return PPG_INVALID_SAMPLE;
	}
	return (int16_t)rx[0];
}


void ADS1118_PPG_OnEegSample(uint8_t eeg_sample_index, uint8_t *ppg_field){

	uint8_t phase = eeg_sample_index % PPG_DECIMATION;
	uint8_t slot  = eeg_sample_index / PPG_DECIMATION;
	int16_t value;

	if(phase == 0){
		conversion_started = (ADS1118_StartConversion() == HAL_OK);
	}
	else if(phase == PPG_DECIMATION - 1){
		/* 4 EEG periods (16 ms) after the start: the 7.8 ms conversion is done. */
		value = conversion_started ? ADS1118_ReadConversion() : PPG_INVALID_SAMPLE;
		conversion_started = 0;
		ppg_field[slot * PPG_SAMPLE_SIZE]     = (uint8_t)((uint16_t)value >> 8);
		ppg_field[slot * PPG_SAMPLE_SIZE + 1] = (uint8_t)((uint16_t)value & 0xFF);
	}
}
