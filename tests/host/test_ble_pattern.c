/*
 * test_ble_pattern.c : host tests for the BLE transfer test pattern
 * (p2p_server_app.c compiled with BLE_TEST_PATTERN=1).
 *
 * Every EEG and IMU packet produced by the real firmware code is decoded here
 * the way the receiving side must decode it (EEG and PPG big-endian, IMU
 * little-endian, signed) and compared with the expected counters, over a whole
 * 16-bit cycle (wrap-around 0xFFFF -> 0x0000 and sign change 0x7FFF -> 0x8000).
 *
 * Optional dumps of the raw packets, to check a decoder against the real
 * firmware output (see AlchemiacPythonSDK, tests/test_ble_pattern.py):
 *   BLE_PATTERN_EEG_DUMP=<file>  first EEG_DUMP_PACKETS EEG packets, back to back
 *   BLE_PATTERN_IMU_DUMP=<file>  first IMU_DUMP_PACKETS IMU packets, back to back
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fakes.h"
#include "ble.h"
#include "p2p_stm.h"
#include "p2p_server_app.h"
#include "app_conf.h"
#include "hermes_ble.h"

#if !BLE_TEST_PATTERN
#error "test_ble_pattern.c must be compiled with -DBLE_TEST_PATTERN=1"
#endif

void APP_BLE_Manage_ADS1299_event_exec(void);
void get_and_send_motion_samples(void);

#define EEG_SAMPLES_PER_PACKET  10
#define EEG_CHANNELS            8
#define EEG_BYTES               (EEG_SAMPLES_PER_PACKET * 24)
#define PACKET_LEN              (1 + EEG_BYTES + 4)
#define IMU_LEN                 18
#define PATTERN_START           0xFF00u

/* A whole 16-bit cycle of the EEG counter, plus the start offset and some margin. */
#define NB_EEG_PACKETS          6600
#define NB_IMU_PACKETS          66000
#define EEG_DUMP_PACKETS        40      /* covers the first wrap-around (sample 256) */
#define IMU_DUMP_PACKETS        300     /* covers the first wrap-around (packet 256) */

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { checks++; if(!(cond)){ failures++; \
	printf("  ECHEC %s:%d : ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while(0)

static int32_t be24s(const uint8_t *p){
	int32_t v = (p[0] << 16) | (p[1] << 8) | p[2];
	return (v & 0x800000) ? v - 0x1000000 : v;
}
static int16_t be16s(const uint8_t *p){ return (int16_t)((p[0] << 8) | p[1]); }
static int16_t le16s(const uint8_t *p){ return (int16_t)(p[0] | (p[1] << 8)); }

static void enable_notifications(int on){
	P2PS_STM_App_Notification_evt_t evt;
	memset(&evt, 0, sizeof(evt));
	evt.P2P_Evt_Opcode = on ? P2PS_STM__NOTIFY_ENABLED_EVT : P2PS_STM_NOTIFY_DISABLED_EVT;
	P2PS_STM_App_Notification(&evt);
}

/* One packet's worth of DRDY interrupts, running the BLE task when scheduled. */
static void run_one_packet(void){
	for(int k = 0; k < EEG_SAMPLES_PER_PACKET; k++){
		fake.current_eeg_index = fake.nb_eeg_reads % EEG_SAMPLES_PER_PACKET;
		APP_BLE_Manage_ADS1299_event();
		if(fake.pending_tasks & (1u << CFG_TASK_ADS_SAMPLE_ID)){
			fake.pending_tasks &= ~(1u << CFG_TASK_ADS_SAMPLE_ID);
			APP_BLE_Manage_ADS1299_event_exec();
		}
	}
}

static FILE *open_dump(const char *env){
	const char *path = getenv(env);
	return path ? fopen(path, "wb") : NULL;
}

/* ------------------------------------------------------------------------ */

static void test_eeg_pattern(void){
	printf("test_eeg_pattern (%d paquets EEG+PPG, cycle 16 bits complet)\n", NB_EEG_PACKETS);
	fake_reset();
	FILE *dump = open_dump("BLE_PATTERN_EEG_DUMP");

	int bad_index = 0, bad_eeg = 0, bad_ppg = 0, bad_len = 0;
	int seen_wrap = 0, seen_ppg_sign_change = 0;
	int16_t last_ppg = 0;

	enable_notifications(1);
	for(int p = 0; p < NB_EEG_PACKETS; p++){
		fake.nb_notifications = 0;
		run_one_packet();
		if(fake.nb_notifications != 1){
			CHECK(0, "paquet %d : %d notification(s)", p, fake.nb_notifications);
			break;
		}
		const uint8_t *pkt = fake.notifications[0];
		if(dump && p < EEG_DUMP_PACKETS) fwrite(pkt, 1, PACKET_LEN, dump);

		if(fake.notification_length[0] != PACKET_LEN) bad_len++;
		if(pkt[0] != p % 128) bad_index++;

		for(int s = 0; s < EEG_SAMPLES_PER_PACKET; s++){
			uint16_t n = (uint16_t)(PATTERN_START + p * EEG_SAMPLES_PER_PACKET + s);
			if(s > 0 && n == 0) seen_wrap = 1;
			for(int c = 0; c < EEG_CHANNELS; c++){
				int32_t expected = (c < EEG_CHANNELS - 1) ? c * 65536 + n : (int32_t)n - 65536;
				int32_t got = be24s(pkt + 1 + s * 24 + c * 3);
				if(got != expected){
					if(bad_eeg++ < 5) printf("  paquet %d ech %d canal %d : %ld, attendu %ld\n",
					                         p, s, c, (long)got, (long)expected);
				}
			}
		}
		for(int k = 0; k < 2; k++){
			uint16_t n = (uint16_t)(PATTERN_START + p * EEG_SAMPLES_PER_PACKET + 5 * k + 4);
			int16_t got = be16s(pkt + 1 + EEG_BYTES + 2 * k);
			if(got != (int16_t)n){
				if(bad_ppg++ < 5) printf("  paquet %d PPG%d : %d, attendu %d\n", p, k, got, (int16_t)n);
			}
			if(last_ppg > 32000 && got < -32000) seen_ppg_sign_change = 1;
			last_ppg = got;
		}
	}
	enable_notifications(0);
	if(dump) fclose(dump);

	CHECK(bad_len == 0, "%d paquet(s) de mauvaise longueur", bad_len);
	CHECK(bad_index == 0, "%d index de paquet faux (0..127 attendu)", bad_index);
	CHECK(bad_eeg == 0, "%d valeur(s) EEG fausses", bad_eeg);
	CHECK(bad_ppg == 0, "%d valeur(s) PPG fausses", bad_ppg);
	CHECK(seen_wrap, "le compteur 16 bits doit revenir a 0");
	CHECK(seen_ppg_sign_change, "le PPG doit passer de +32767 a -32768");

	/* The real acquisition still runs: same SPI traffic and timing as without the pattern. */
	CHECK(fake.nb_eeg_reads == NB_EEG_PACKETS * EEG_SAMPLES_PER_PACKET, "ADS1299 lu %d fois", fake.nb_eeg_reads);
	CHECK(fake.nb_conversions == NB_EEG_PACKETS * 2, "%d conversions ADS1118, %d attendues",
	      fake.nb_conversions, NB_EEG_PACKETS * 2);
	CHECK(fake.bus_conflicts == 0, "conflits SPI : %d", fake.bus_conflicts);
	CHECK(fake.protocol_errors == 0, "erreurs de protocole : %d", fake.protocol_errors);
}

static void test_imu_pattern(void){
	printf("test_imu_pattern (%d paquets IMU, int16 little-endian)\n", NB_IMU_PACKETS);
	fake_reset();
	FILE *dump = open_dump("BLE_PATTERN_IMU_DUMP");
	int bad = 0, seen_wrap = 0;

	enable_notifications(1);
	for(int p = 0; p < NB_IMU_PACKETS; p++){
		get_and_send_motion_samples();
		if(dump && p < IMU_DUMP_PACKETS) fwrite(fake.last_imu, 1, IMU_LEN, dump);
		for(int i = 0; i < IMU_LEN / 2; i++){
			uint16_t v = (uint16_t)(PATTERN_START + p + i);
			int16_t got = le16s(fake.last_imu + 2 * i);
			if(got != (int16_t)v){
				if(bad++ < 5) printf("  paquet IMU %d champ %d : %d, attendu %d\n", p, i, got, (int16_t)v);
			}
			if(i == 0 && p > 0 && v == 0) seen_wrap = 1;
		}
	}
	enable_notifications(0);
	if(dump) fclose(dump);

	CHECK(fake.nb_imu_notifications == NB_IMU_PACKETS, "%d paquets IMU envoyes", fake.nb_imu_notifications);
	CHECK(bad == 0, "%d valeur(s) IMU fausses", bad);
	CHECK(seen_wrap, "le compteur IMU doit revenir a 0");
}

int main(void){
	test_eeg_pattern();
	test_imu_pattern();

	printf("\n%d verifications, %d echec(s)\n", checks, failures);
	return failures ? 1 : 0;
}
