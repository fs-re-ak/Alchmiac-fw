/*
 * test_ppg.c : host tests for the PPG acquisition (ADS1118) and the EEG+PPG
 * BLE packet built by p2p_server_app.c.
 *
 * The real firmware sources (Core/Src/ads1118.c, STM32_WPAN/App/p2p_server_app.c)
 * are compiled for the PC and linked against fakes.c. See README.md.
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

void APP_BLE_Manage_ADS1299_event_exec(void);

#define EEG_SAMPLES_PER_PACKET  10
#define EEG_BYTES               (EEG_SAMPLES_PER_PACKET * 24)
#define PACKET_LEN              (1 + EEG_BYTES + 4)
#define PPG_INVALID             0x8000

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { checks++; if(!(cond)){ failures++; \
	printf("  ECHEC %s:%d : ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while(0)

static uint16_t be16(const uint8_t *p){ return (uint16_t)((p[0] << 8) | p[1]); }

static void enable_notifications(int on){
	P2PS_STM_App_Notification_evt_t evt;
	memset(&evt, 0, sizeof(evt));
	evt.P2P_Evt_Opcode = on ? P2PS_STM__NOTIFY_ENABLED_EVT : P2PS_STM_NOTIFY_DISABLED_EVT;
	P2PS_STM_App_Notification(&evt);
}

/* Simulates nb_drdy ADS1299 DRDY interrupts, running the BLE task when scheduled. */
static void run_drdy(int nb_drdy){
	for(int k = 0; k < nb_drdy; k++){
		fake.current_eeg_index = fake.nb_eeg_reads % EEG_SAMPLES_PER_PACKET;
		APP_BLE_Manage_ADS1299_event();
		if(fake.pending_tasks & (1u << CFG_TASK_ADS_SAMPLE_ID)){
			fake.pending_tasks &= ~(1u << CFG_TASK_ADS_SAMPLE_ID);
			APP_BLE_Manage_ADS1299_event_exec();
		}
	}
}

/*
 * Same, with a sequencer model: after each DRDY, the pending BLE task runs up to
 * max_runs times (one run per pending bit, as UTIL_SEQ does; several SetTask before
 * a run collapse into one). Every period EEG reads, the BLE stack refuses the
 * notifications for busy_reads EEG periods (TX pool full until a connection event).
 */
static void run_drdy_congested(int nb_drdy, int period, int busy_reads, int max_runs){
	for(int k = 0; k < nb_drdy; k++){
		if(k % period == 3){
			fake.ble_busy_until = fake.nb_eeg_reads + busy_reads;
		}
		fake.current_eeg_index = fake.nb_eeg_reads % EEG_SAMPLES_PER_PACKET;
		APP_BLE_Manage_ADS1299_event();
		for(int r = 0; r < max_runs && (fake.pending_tasks & (1u << CFG_TASK_ADS_SAMPLE_ID)); r++){
			fake.pending_tasks &= ~(1u << CFG_TASK_ADS_SAMPLE_ID);
			APP_BLE_Manage_ADS1299_event_exec();
		}
	}
}

/* ------------------------------------------------------------------------ */

static void test_constants(void){
	printf("test_constants\n");
	CHECK(NEW_NOTIFY_CHAR_VALUE_LENGTH == PACKET_LEN, "longueur caracteristique = %d", NEW_NOTIFY_CHAR_VALUE_LENGTH);
	CHECK(PACKET_LEN + 3 <= CFG_BLE_MAX_ATT_MTU, "paquet %d + 3 > MTU %d", PACKET_LEN, CFG_BLE_MAX_ATT_MTU);
	CHECK(PACKET_LEN <= 255, "la longueur doit tenir dans le uint8_t de APP_BLE_Send_EEGData_Notification");
	/* Start -> read = (PPG_DECIMATION-1) EEG periods of 4 ms; conversion at 128 SPS = 7.8 ms */
	CHECK((PPG_DECIMATION - 1) * 4.0 > 7.8 * 1.2, "pas assez de temps pour la conversion");
	CHECK(250 / PPG_DECIMATION == 50, "frequence PPG = %d Hz", 250 / PPG_DECIMATION);
	CHECK(ADS1118_CONFIG_PPG == 0xC38B, "config 0x%04X", ADS1118_CONFIG_PPG);
}

static void test_absent(void){
	const miso_mode_t modes[] = {MISO_STUCK_HIGH, MISO_STUCK_LOW};
	for(int m = 0; m < 2; m++){
		printf("test_absent (ADS1118 absent, MISO bloque a %d)\n", m == 0);
		fake_reset();
		fake.miso_mode = modes[m];

		uint8_t ppg[4] = {0};
		for(int i = 0; i < EEG_SAMPLES_PER_PACKET; i++){
			fake.current_eeg_index = i;
			ADS1118_PPG_OnEegSample((uint8_t)i, ppg);
		}
		CHECK(be16(ppg) == PPG_INVALID && be16(ppg + 2) == PPG_INVALID,
		      "valeur invalide attendue (readback incorrect) : %04X %04X", be16(ppg), be16(ppg + 2));
		CHECK(fake.ads1118_cs == GPIO_PIN_SET, "CS doit etre relache");
	}
}

static void test_not_connected(void){
	printf("test_not_connected (aucun acces SPI avant le streaming, comportement existant conserve)\n");
	fake_reset();
	enable_notifications(0);
	run_drdy(30);
	CHECK(fake.nb_notifications == 0, "aucun paquet sans abonnement, %d envoyes", fake.nb_notifications);
	CHECK(fake.nb_transfers == 0, "aucun acces ADS1118 sans abonnement, %d", fake.nb_transfers);
	CHECK(fake.nb_eeg_reads == 0, "aucune lecture ADS1299 sans abonnement (comportement existant)");
}

static void test_streaming(void){
	printf("test_streaming (3 paquets EEG+PPG de bout en bout)\n");
	const int16_t values[] = {1234, -1, 32767, 0, 16000, -32767};
	fake_reset();
	memcpy(fake.next_values, values, sizeof(values));
	int first_stream_frame = fake.nb_frames;

	enable_notifications(1);
	run_drdy(3 * EEG_SAMPLES_PER_PACKET);
	enable_notifications(0);

	CHECK(fake.nb_notifications == 3, "3 paquets attendus, %d recus", fake.nb_notifications);
	CHECK(fake.bus_conflicts == 0, "ADS1299 lu pendant que l'ADS1118 est selectionne (%d fois)", fake.bus_conflicts);
	CHECK(fake.protocol_errors == 0, "erreurs de protocole : %d", fake.protocol_errors);
	CHECK(fake.nb_conversions == 6, "6 conversions attendues, %d", fake.nb_conversions);

	for(int p = 0; p < fake.nb_notifications && p < 3; p++){
		const uint8_t *pkt = fake.notifications[p];
		CHECK(fake.notification_length[p] == PACKET_LEN, "paquet %d : %d octets", p, fake.notification_length[p]);
		CHECK(pkt[0] == p, "paquet %d : index %d", p, pkt[0]);

		/* EEG layout unchanged: bytes 1..240 = the 10 ADS1299 reads, in order */
		int eeg_ok = 1;
		for(int s = 0; s < EEG_SAMPLES_PER_PACKET; s++){
			for(int i = 0; i < 24; i++){
				if(pkt[1 + s * 24 + i] != eeg_pattern(p * EEG_SAMPLES_PER_PACKET + s, i)) eeg_ok = 0;
			}
		}
		CHECK(eeg_ok, "paquet %d : donnees EEG modifiees", p);

		uint16_t ppg0 = be16(pkt + 1 + EEG_BYTES);
		uint16_t ppg1 = be16(pkt + 1 + EEG_BYTES + 2);
		CHECK(ppg0 == (uint16_t)values[2 * p], "paquet %d PPG0 = 0x%04X, attendu 0x%04X", p, ppg0, (uint16_t)values[2 * p]);
		CHECK(ppg1 == (uint16_t)values[2 * p + 1], "paquet %d PPG1 = 0x%04X, attendu 0x%04X", p, ppg1, (uint16_t)values[2 * p + 1]);
	}

	/* SPI schedule: start at EEG index 0 and 5, read at 4 and 9, nothing else. */
	int nb_stream_frames = fake.nb_frames - first_stream_frame;
	CHECK(nb_stream_frames == 3 * 4, "4 trames ADS1118 par paquet attendues, %d", nb_stream_frames);
	for(int i = first_stream_frame; i < fake.nb_frames; i++){
		const frame_t *f = &fake.frames[i];
		int idx = f->eeg_index;
		if(idx == 0 || idx == 5){
			CHECK(f->nb_words == 1, "index %d : lancement en trame de 16 bits attendu", idx);
			CHECK(f->tx[0] == ADS1118_CONFIG_PPG && f->started_conversion, "index %d : lancement attendu (0x%04X)", idx, f->tx[0]);
		}else if(idx == 4 || idx == 9){
			CHECK(f->nb_words == 2, "index %d : lecture en trame de 32 bits (donnee + config) attendue", idx);
			CHECK(f->tx[0] == 0x0000 && f->tx[1] == 0x0000 && !f->started_conversion,
			      "index %d : lecture seule attendue (0x%04X 0x%04X)", idx, f->tx[0], f->tx[1]);
			CHECK(i > first_stream_frame && fake.frames[i - 1].started_conversion && fake.frames[i - 1].eeg_index == idx - 4,
			      "index %d : la lecture doit suivre le lancement 4 echantillons plus tot", idx);
		}else{
			CHECK(0, "acces ADS1118 inattendu a l'index EEG %d", idx);
		}
	}
	CHECK((fake.config & 0x7FFF) == 0x438B, "la config ne doit pas etre modifiee par les lectures");

	/* Optional: save the packets to check the SDK decoding against the real firmware output. */
	const char *dump = getenv("PPG_DUMP_FILE");
	if(dump){
		FILE *f = fopen(dump, "wb");
		for(int p = 0; f && p < fake.nb_notifications && p < 3; p++){
			fwrite(fake.notifications[p], 1, fake.notification_length[p], f);
		}
		if(f) fclose(f);
	}
}

static void test_spi_error_during_read(void){
	printf("test_spi_error_during_read\n");
	const int16_t values[] = {111, 222};
	fake_reset();
	memcpy(fake.next_values, values, sizeof(values));
	/* Transfers: #0 start(0), #1 read(4), #2 start(5), #3 read(9) */
	fake.fail_transfer_at = 1;

	enable_notifications(1);
	run_drdy(EEG_SAMPLES_PER_PACKET);
	enable_notifications(0);

	CHECK(fake.nb_notifications == 1, "1 paquet attendu");
	CHECK(be16(fake.notifications[0] + 1 + EEG_BYTES) == PPG_INVALID, "PPG0 invalide attendu apres erreur SPI");
	CHECK(be16(fake.notifications[0] + 1 + EEG_BYTES + 2) == 222, "PPG1 doit rester valide");
	CHECK(fake.ads1118_cs == GPIO_PIN_SET, "CS relache apres l'erreur");
}

static void test_spi_error_during_start(void){
	printf("test_spi_error_during_start (pas de valeur perimee)\n");
	const int16_t values[] = {111, 222};
	fake_reset();
	memcpy(fake.next_values, values, sizeof(values));
	fake.fail_transfer_at = 0;   /* start(0) fails: no conversion */

	enable_notifications(1);
	run_drdy(EEG_SAMPLES_PER_PACKET);
	enable_notifications(0);

	CHECK(fake.nb_notifications == 1, "1 paquet attendu");
	CHECK(be16(fake.notifications[0] + 1 + EEG_BYTES) == PPG_INVALID,
	      "PPG0 invalide attendu quand le lancement a echoue (0x%04X)", be16(fake.notifications[0] + 1 + EEG_BYTES));
	CHECK(be16(fake.notifications[0] + 1 + EEG_BYTES + 2) == 111, "PPG1 = premiere conversion reussie");
	CHECK(fake.ads1118_cs == GPIO_PIN_SET, "CS relache apres l'erreur");
}

static void test_ble_backlog_drained(void){
	printf("test_ble_backlog_drained (envois refuses par la pile BLE : le retard doit etre rattrape)\n");
	const int nb_packets = 1000;   /* 40 s at 250 Hz */
	fake_reset();
	enable_notifications(1);
	/* The BLE stack is busy for 25 EEG periods (100 ms, two connection intervals) every second. */
	run_drdy_congested(nb_packets * EEG_SAMPLES_PER_PACKET, 250, 25, 4);
	enable_notifications(0);

	CHECK(fake.nb_refused_sends > 0, "le scenario doit provoquer des refus d'envoi");
	CHECK(fake.index_gaps == 0, "%d saut(s) d'index : paquets perdus par le firmware", fake.index_gaps);
	CHECK(fake.nb_notifications >= nb_packets - 1, "%d paquets envoyes sur %d produits : retard non rattrape",
	      fake.nb_notifications, nb_packets);
}

int main(void){
	test_constants();
	test_absent();
	test_not_connected();
	test_streaming();
	test_spi_error_during_read();
	test_spi_error_during_start();
	test_ble_backlog_drained();

	printf("\n%d verifications, %d echec(s)\n", checks, failures);
	return failures ? 1 : 0;
}
