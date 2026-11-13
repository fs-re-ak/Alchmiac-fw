/*
 * fakes.c : host (PC) stand-ins for the hardware used by p2p_server_app.c and
 * ads1118.c, including a behavioural model of the ADS1118 on SPI1.
 */
#include <string.h>
#include "fakes.h"

SPI_HandleTypeDef hspi1;
fake_state_t fake;

static const uint16_t ADS1118_DEFAULT_CONFIG = 0x058B; /* datasheet reset value */

void fake_reset(void){
	memset(&fake, 0, sizeof(fake));
	fake.ads1118_cs = GPIO_PIN_SET;
	fake.config = ADS1118_DEFAULT_CONFIG;
	fake.fail_transfer_at = -1;
	fake.last_sent_index = -1;
}

/* ---------------- HAL ---------------- */

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
	if(GPIOx == ADS1118_CS_GPIO_Port && GPIO_Pin == ADS1118_CS_Pin){
		fake.ads1118_cs = PinState;
	}
}

static uint16_t miso_word(uint16_t value){
	switch(fake.miso_mode){
	case MISO_STUCK_HIGH: return 0xFFFF;
	case MISO_STUCK_LOW:  return 0x0000;
	default:              return value;
	}
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout){
	(void)Timeout;
	int index = fake.nb_transfers++;

	if(hspi != &hspi1 || fake.ads1118_cs != GPIO_PIN_RESET || (Size != 2 && Size != 4)){
		fake.protocol_errors++;
		return HAL_ERROR;
	}
	if(index == fake.fail_transfer_at){
		return HAL_ERROR;
	}

	frame_t *f = &fake.frames[fake.nb_frames < MAX_FRAMES ? fake.nb_frames++ : MAX_FRAMES - 1];
	f->nb_words = (uint8_t)(Size / 2);
	f->eeg_index = fake.current_eeg_index;

	for(int w = 0; w < f->nb_words; w++){
		uint16_t tx = (uint16_t)((pTxData[2*w] << 8) | pTxData[2*w + 1]);
		uint16_t rx;
		f->tx[w] = tx;

		/* DOUT: conversion result first, then config readback (SS reads 0). */
		if(w == 0){
			rx = fake.data_register;
		}else{
			rx = (uint16_t)(fake.config & 0x7FFF);
		}
		rx = miso_word(rx);
		pRxData[2*w]     = (uint8_t)(rx >> 8);
		pRxData[2*w + 1] = (uint8_t)(rx & 0xFF);

		/* DIN: NOP=01 -> config written; SS=1 in single-shot mode -> conversion. */
		if(((tx >> 1) & 0x3) == 0x1){
			fake.config = tx;
			if((tx & 0x8000) && (tx & 0x0100)){
				fake.nb_conversions++;
				fake.last_start_eeg_index = fake.current_eeg_index;
				fake.data_register = (uint16_t)fake.next_values[fake.next_value_pos++ % MAX_VALUES];
				f->started_conversion = 1;
			}
		}
	}
	return HAL_OK;
}

/* ---------------- ADS1299 ---------------- */

HAL_StatusTypeDef ADS1299_ReadSamples(uint8_t *statusBuffer, uint8_t *dataBuffer){
	if(fake.ads1118_cs != GPIO_PIN_SET){
		fake.bus_conflicts++;   /* both chips selected on SPI1 */
	}
	memset(statusBuffer, 0xC0, 3);
	for(int i = 0; i < 24; i++){
		dataBuffer[i] = eeg_pattern(fake.nb_eeg_reads, i);
	}
	fake.nb_eeg_reads++;
	return HAL_OK;
}

/* ---------------- BLE / sequencer ---------------- */

uint8_t APP_BLE_Send_EEGData_Notification(uint8_t *payload, uint8_t length){
	if(fake.nb_eeg_reads < fake.ble_busy_until){
		fake.nb_refused_sends++;
		return 0x64;   /* BLE_STATUS_INSUFFICIENT_RESOURCES: TX pool full */
	}
	if(fake.last_sent_index >= 0 && payload[0] != (fake.last_sent_index + 1) % 128){
		fake.index_gaps++;
	}
	fake.last_sent_index = payload[0];
	if(fake.nb_notifications < MAX_NOTIFICATIONS){
		memcpy(fake.notifications[fake.nb_notifications], payload, length);
		fake.notification_length[fake.nb_notifications] = length;
	}
	fake.nb_notifications++;
	return 0;
}

uint8_t APP_BLE_Send_Event_Notification(event_packet_t *payload){ (void)payload; return 0; }
uint8_t APP_BLE_Send_IMU_Notification(uint8_t *accel){
	memcpy(fake.last_imu, accel, sizeof(fake.last_imu));
	fake.nb_imu_notifications++;
	return 0;
}

void UTIL_SEQ_RegTask(UTIL_SEQ_bm_t TaskId_bm, uint32_t Flags, void (*Task)(void)){
	(void)TaskId_bm; (void)Flags; (void)Task;
}

void UTIL_SEQ_SetTask(UTIL_SEQ_bm_t TaskId_bm, uint32_t Task_Prio){
	(void)Task_Prio;
	fake.pending_tasks |= TaskId_bm;
}

/* ---------------- IMU ---------------- */

static int16_t zeros[9];
int16_t *ism330_ReadIMU(void){ return zeros; }
int16_t *lis3mdl_ReadMag(void){ return zeros; }
