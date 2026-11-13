/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : App/p2p_server_app.c
  * Description        : P2P Server Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <p2p_server_app.h>
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "stm32_seq.h"
#include "app_ble.h"
#include "gpios.h"
#include "ads1299.h"
#include "ads1118.h"
#include "hermes_ble.h"
#include "ism330.h"
#include "lis3mdl.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef struct{
    uint8_t             Device_Led_Selection;
    uint8_t             Led1;
 }P2P_LedCharValue_t;

 typedef struct{
    uint8_t             Device_Button_Selection;
    uint8_t             ButtonStatus;
 }P2P_ButtonCharValue_t;


typedef struct
{
  uint8_t               Notification_Status; /* used to check if P2P Server is enabled to Notify */
  P2P_LedCharValue_t    LedControl;
  P2P_ButtonCharValue_t ButtonControl;
  uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_24BIT_VALUE 0xFFFFFF  // Maximum value for 24-bit integer

#define PACKETBUFFER_DEPTH 30
#define NB_SAMPLES_PER_PACKET 10
#define SAMPLE_SIZE 24
#define EEG_PACKET_SIZE (NB_SAMPLES_PER_PACKET * SAMPLE_SIZE + 1)

/* PPG samples (int16 big-endian) appended after the EEG samples */
#define PPG_NB_SAMPLES_PER_PACKET (NB_SAMPLES_PER_PACKET / PPG_DECIMATION)
#define PPG_OFFSET EEG_PACKET_SIZE
#define PACKET_SIZE (EEG_PACKET_SIZE + PPG_NB_SAMPLES_PER_PACKET * PPG_SAMPLE_SIZE)

_Static_assert(NB_SAMPLES_PER_PACKET % PPG_DECIMATION == 0, "EEG packet must hold a whole number of PPG periods");
_Static_assert(PACKET_SIZE <= NEW_NOTIFY_CHAR_VALUE_LENGTH, "EEG characteristic too small for the packet");

#define MOTION_NB_SAMPLES_PER_PACKET 10
#define MOTION_SAMPLE_SIZE sizeof(uint16_t)*3
#define MOTION_PACKET_SIZE (MOTION_NB_SAMPLES_PER_PACKET * MOTION_SAMPLE_SIZE)

/*
 * BLE transfer test pattern. NEVER enable it in a headset firmware.
 * With BLE_TEST_PATTERN=1, the EEG, PPG and IMU values are replaced by 16-bit
 * counters just before being sent, so that the receiving side can check the
 * decoding (byte order, sign, wrap-around) and tell losses from corruption.
 * The ADS1299 and the ADS1118 are still read: timing and SPI traffic are real.
 *
 * Counter n: +1 per EEG sample (250 Hz), starts at TEST_PATTERN_START at boot
 * (first wrap after 1 s, then every 262 s).
 *   EEG channel c (24-bit big-endian) : bytes [c, n >> 8, n & 0xFF] for c = 0..6
 *                                       -> c * 65536 + n
 *                                       bytes [0xFF, n >> 8, n & 0xFF] for c = 7
 *                                       -> n - 65536 (negative: sign extension)
 *   PPG (int16 big-endian)            : (int16_t)n of the EEG sample where the
 *                                       real value is stored (indices 4 and 9)
 * Counter m: +1 per IMU packet (50 Hz), same start value.
 *   IMU field i = 0..8 (int16 little-endian, like the real data) : (int16_t)(m + i)
 * The packet index is unchanged (0..127).
 */
#ifndef BLE_TEST_PATTERN
#define BLE_TEST_PATTERN 0
#endif

#if BLE_TEST_PATTERN
#warning "BLE_TEST_PATTERN enabled: EEG, PPG and IMU data are replaced by a test pattern"
#define TEST_PATTERN_START          0xFF00u
#define TEST_PATTERN_LAST_CH_TAG    0xFF
#define EEG_NB_CHANNELS             (SAMPLE_SIZE / 3)
#endif


/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t buffered_packets_array[PACKETBUFFER_DEPTH][PACKET_SIZE] = {0};
static uint8_t buffer_index = 0;
static uint8_t read_index = 0;
static uint8_t sample_index = 0;

static uint8_t packet_counter = 0;

uint8_t statusBuffer[3];

static event_packet_t current_event_payload;


static uint8_t motion_packet[18] = {0};

#if BLE_TEST_PATTERN
static uint16_t test_pattern_eeg = TEST_PATTERN_START;
static uint16_t test_pattern_imu = TEST_PATTERN_START;

/* Overwrites one EEG sample (and its PPG value, if stored at this index). */
static void test_pattern_fill_eeg(uint8_t *sample, uint8_t *ppg_field, uint8_t eeg_sample_index){
	uint16_t n = test_pattern_eeg++;

	for(uint8_t c = 0; c < EEG_NB_CHANNELS; c++){
		sample[3*c]     = (c == EEG_NB_CHANNELS - 1) ? TEST_PATTERN_LAST_CH_TAG : c;
		sample[3*c + 1] = (uint8_t)(n >> 8);
		sample[3*c + 2] = (uint8_t)(n & 0xFF);
	}
	if(eeg_sample_index % PPG_DECIMATION == PPG_DECIMATION - 1){
		uint8_t *ppg = &ppg_field[(eeg_sample_index / PPG_DECIMATION) * PPG_SAMPLE_SIZE];
		ppg[0] = (uint8_t)(n >> 8);
		ppg[1] = (uint8_t)(n & 0xFF);
	}
}

static void test_pattern_fill_motion(uint8_t *packet){
	uint16_t m = test_pattern_imu++;

	for(uint8_t i = 0; i < sizeof(motion_packet) / sizeof(int16_t); i++){
		uint16_t v = (uint16_t)(m + i);
		packet[2*i]     = (uint8_t)(v & 0xFF);
		packet[2*i + 1] = (uint8_t)(v >> 8);
	}
}
#endif

void SWA_Send_Notification(void);
void SWB_Send_Notification(void);

void SWA_Local_Notification(void);
void SWB_Local_Notification(void);

void APP_BLE_Manage_ADS1299_event_exec(void);

void get_and_send_motion_samples(void);


/**
 * START of Section BLE_APP_CONTEXT
 */

P2P_Server_App_Context_t P2P_Server_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//static void P2PS_Send_Notification(void);
//static void P2PS_APP_LED_BUTTON_context_Init(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */
/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
    case P2PS_STM_BOOT_REQUEST_EVT:
      APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
      APP_DBG_MSG(" \n\r");

      *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
      NVIC_SystemReset();
      break;
#endif
/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 1;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n"); 
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 0;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

#if 0
    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
      if(pNotification->DataTransfered.pPayload[0] == 0x00){ /* ALL Deviceselected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#if(P2P_SERVER1 != 0)  
      if(pNotification->DataTransfered.pPayload[0] == 0x01){ /* end device 1 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#endif
/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */
      
/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
          
/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
//       P2PS_APP_LED_BUTTON_context_Init();
    	P2P_Server_App_Context.Notification_Status = 0;
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */


#ifdef BLE_BUTTON_EVENTS
	UTIL_SEQ_RegTask( 1<< CFG_TASK_SWA_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, SWA_Send_Notification ); //
	UTIL_SEQ_RegTask( 1<< CFG_TASK_SWB_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, SWB_Send_Notification ); //
	UTIL_SEQ_RegTask( 1<< CFG_TASK_IMU_SAMPLE_ID, UTIL_SEQ_RFU, get_and_send_motion_samples ); //
	UTIL_SEQ_RegTask( 1<< CFG_TASK_ADS_SAMPLE_ID, UTIL_SEQ_RFU, APP_BLE_Manage_ADS1299_event_exec ); //


#else
	UTIL_SEQ_RegTask( 1<< CFG_TASK_SWA_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, SWA_Local_Notification ); //
	UTIL_SEQ_RegTask( 1<< CFG_TASK_SWB_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, SWB_Local_Notification ); //
#endif

//(FS) Need to attach whatever user function here

  /**
   * Initialize LedButton Service
   */
  P2P_Server_App_Context.Notification_Status=0; 


  //P2PS_APP_LED_BUTTON_context_Init();
/* USER CODE END P2PS_APP_Init */
  return;
}



/* USER CODE BEGIN FD */

#if 0
void P2PS_APP_LED_BUTTON_context_Init(void){
  
  
#if(P2P_SERVER1 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x01; /* Device1 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x01;/* Device1 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif

}
#endif

void APP_SWA_Button_Action(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SWA_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
  return;
}

void APP_SWB_Button_Action(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SWB_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
  return;
}



void SWA_Local_Notification(void)
{
	// insert local button management here
}


void SWB_Local_Notification(void)
{
	// insert local button management here
}


void SWA_Send_Notification(void)
{
	current_event_payload.event_type = EVENT_TYPE_BUTTON_PRESSED;
	current_event_payload.source_id = BUTTON_ID_A;
	current_event_payload.packet_id = packet_counter;

	APP_BLE_Send_Event_Notification(&current_event_payload);
}


void SWB_Send_Notification(void)
{
	current_event_payload.event_type = EVENT_TYPE_BUTTON_PRESSED;
	current_event_payload.source_id = BUTTON_ID_B;
	current_event_payload.packet_id = packet_counter;

	APP_BLE_Send_Event_Notification(&current_event_payload);
}

void get_and_send_motion_samples(void){

	if(P2P_Server_App_Context.Notification_Status==1){

		int16_t* imu_sample = ism330_ReadIMU();
		int16_t* compass_sample = lis3mdl_ReadMag();

		memcpy(motion_packet,imu_sample,6*sizeof(int16_t));
		memcpy(&motion_packet[sizeof(int16_t)*6],compass_sample,3*sizeof(int16_t));
#if BLE_TEST_PATTERN
		test_pattern_fill_motion(motion_packet);
#endif

		if(APP_BLE_Send_IMU_Notification((uint8_t*)motion_packet)!=0){
			HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);
		}
	}
}





/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/



void APP_BLE_Manage_ADS1299_event(void){

	if(P2P_Server_App_Context.Notification_Status==1){

		if(sample_index==0){
			buffered_packets_array[buffer_index][0] = packet_counter;
		}

		ADS1299_ReadSamples(statusBuffer, &buffered_packets_array[buffer_index][sample_index*SAMPLE_SIZE+1]);
		ADS1118_PPG_OnEegSample(sample_index, &buffered_packets_array[buffer_index][PPG_OFFSET]);
#if BLE_TEST_PATTERN
		test_pattern_fill_eeg(&buffered_packets_array[buffer_index][sample_index*SAMPLE_SIZE+1],
		                      &buffered_packets_array[buffer_index][PPG_OFFSET], sample_index);
#endif
		sample_index++;

		if(sample_index >= NB_SAMPLES_PER_PACKET){


			buffer_index = (buffer_index + 1) % PACKETBUFFER_DEPTH;
			packet_counter = (packet_counter + 1) % 128;
			sample_index = 0;

			// this will call BLE transfer
			UTIL_SEQ_SetTask( 1<<CFG_TASK_ADS_SAMPLE_ID, CFG_SCH_PRIO_0);
		}
	}
}

static uint8_t error_count = 0;

void APP_BLE_Manage_ADS1299_event_exec(void)
{

	if (read_index != buffer_index) {
		if(APP_BLE_Send_EEGData_Notification(buffered_packets_array[read_index], PACKET_SIZE)!=0){
			HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);
			UTIL_SEQ_SetTask( 1<<CFG_TASK_ADS_SAMPLE_ID, CFG_SCH_PRIO_0);
			error_count++;

			if(error_count>2){
				HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);
			}
			return;
		}else{
			error_count = 0;
		}

	    read_index = (read_index + 1) % PACKETBUFFER_DEPTH;
	}


}



uint8_t is_connected(void){
	return P2P_Server_App_Context.Notification_Status == 1;
}



/* USER CODE END FD_LOCAL_FUNCTIONS*/
