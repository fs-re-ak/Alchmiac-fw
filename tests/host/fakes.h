/*
 * fakes.h : host test doubles for the firmware (HAL SPI/GPIO, ADS1118 model,
 * ADS1299 read, BLE notifications, sequencer).
 */
#ifndef TESTS_HOST_FAKES_H_
#define TESTS_HOST_FAKES_H_

#include "stm32wbxx_hal.h"
#include "stm32_seq.h"
#include "app_ble.h"
#include "ads1118.h"

#define MAX_FRAMES          256
#define MAX_VALUES          16
#define MAX_NOTIFICATIONS   16
#define NOTIFICATION_MAX    260

typedef enum { MISO_NORMAL = 0, MISO_STUCK_HIGH, MISO_STUCK_LOW } miso_mode_t;

typedef struct {
	uint8_t  nb_words;
	uint16_t tx[2];
	int      eeg_index;          /* EEG sample index when the frame was sent */
	uint8_t  started_conversion;
} frame_t;

typedef struct {
	/* ADS1118 model */
	GPIO_PinState ads1118_cs;
	uint16_t config;
	uint16_t data_register;
	int16_t  next_values[MAX_VALUES];
	int      next_value_pos;
	int      nb_conversions;
	int      last_start_eeg_index;
	miso_mode_t miso_mode;
	int      fail_transfer_at;   /* index of the SPI transfer that fails, -1 = none */

	/* Observations */
	int      nb_transfers;
	int      nb_frames;
	frame_t  frames[MAX_FRAMES];
	int      protocol_errors;    /* transfer without CS low, wrong size... */
	int      bus_conflicts;      /* ADS1299 read while ADS1118 selected */
	int      current_eeg_index;
	int      nb_eeg_reads;
	int      nb_notifications;
	uint8_t  notifications[MAX_NOTIFICATIONS][NOTIFICATION_MAX];
	uint8_t  notification_length[MAX_NOTIFICATIONS];
	uint32_t pending_tasks;
} fake_state_t;

extern fake_state_t fake;

void fake_reset(void);

/* Deterministic EEG byte produced by the fake ADS1299 for read #n, byte i. */
static inline uint8_t eeg_pattern(int n, int i){ return (uint8_t)((n * 37 + i * 11 + 5) & 0xFF); }

#endif /* TESTS_HOST_FAKES_H_ */
