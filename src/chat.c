/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name          : chat.c
 * Author             : AMS - VMA RF  Application team
 * Version            : V1.0.0
 * Date               : 30-November-2015
 * Description        : This file handles bytes received from USB and the init
 *                      function.
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "sleep.h"
#include "chat.h"
#include "SDK_EVAL_Config.h"
#include "stdbool.h"
#include "OTA_btl.h"

#include "lsm6dso.h"
#include "sleep.h"
#include "HWAdvanceFeatures.h"

#if ENABLE_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* 0x01 represents printing calculated data, 0x02 represents printing raw data. */
#define XENGINEINPUT_DATA   (0X01)
#define RAW_DATA						(0X02)

#define ENABLE_FREE_FALL	0

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  float mag[3];                  /* Calibrated mag [uT]/50 */
  float acc[3];                  /* Acceleration in [g] */
  float gyro[3];                 /* Angular rate [dps] */
} MFX_CM0P_input_t;

/* Private defines -----------------------------------------------------------*/

#define CMD_BUFF_SIZE 512

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t connInfo[20];
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
AxesRaw_t acc_data, gyro_data, mag_data, mag_offset;
MFX_CM0P_input_t xEngineInput;
//volatile bool connected = false;

volatile uint32_t start_time = 0;
volatile bool sensorTimer_expired = false;
volatile uint8_t button_flag = 0;
static uint8_t sleep_flag = 1;
volatile uint8_t timer_irq_flag = 0;
static volatile uint8_t timer_flag = 0;
volatile uint8_t free_fall_notify = 0;

volatile FeaturePresence xFeaturePresence;
volatile FeatureNotification xFeatureNotification;
volatile HardwareFeaturePresence xHardwareFeaturePresence;


/** 
 * @brief  Handle of TX,RX  Characteristics.
 */
#ifdef CLIENT
uint16_t tx_handle;
uint16_t rx_handle;
#endif 

/* UUIDs */
UUID_t UUID_Tx;
UUID_t UUID_Rx;

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
volatile uint8_t receivedString[20];
volatile uint8_t receivedStringIDX = 0;
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */

static char cmd[CMD_BUFF_SIZE];
static uint16_t cmd_buff_end = 0, cmd_buff_start = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : CHAT_DeviceInit.
 * Description    : Init the Chat device.
 * Input          : none.
 * Return         : Status.
 *******************************************************************************/
uint8_t CHAT_DeviceInit(void) {
	uint8_t ret;
	uint16_t service_handle;
	uint16_t dev_name_char_handle;
	uint16_t appearance_char_handle;
	uint8_t name[] = { 'B', 'l', 'u', 'e', 'N', 'R', 'G', '1' };

#if SERVER
	uint8_t role = GAP_PERIPHERAL_ROLE;
	uint8_t bdaddr[] = { 0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02 };
#else
	uint8_t role = GAP_CENTRAL_ROLE;
	uint8_t bdaddr[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};
#endif 

	/* Configure Public address */
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Setting BD_ADDR failed: 0x%02x\r\n", ret);
		return ret;
	}

	/* Set the TX power to -2 dBm */
	aci_hal_set_tx_power_level(1, 0x07);

	/* GATT Init */
	ret = aci_gatt_init();
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error in aci_gatt_init(): 0x%02x\r\n", ret);
		return ret;
	} else {
		PRINTF("aci_gatt_init() --> SUCCESS\r\n");
	}

	/* GAP Init */
	ret = aci_gap_init(role, 0x00, 0x08, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error in aci_gap_init() 0x%02x\r\n", ret);
		return ret;
	} else {
		PRINTF("aci_gap_init() --> SUCCESS\r\n");
	}

	/* Set the device name */
	ret = aci_gatt_update_char_value_ext(0, service_handle, dev_name_char_handle, 0, sizeof(name), 0, sizeof(name), name);
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error in Gatt Update characteristic value 0x%02x\r\n", ret);
		return ret;
	} else {
		PRINTF("aci_gatt_update_char_value_ext() --> SUCCESS\r\n");
	}

#if  SERVER
	ret = Add_Chat_Service();
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error in Add_Chat_Service 0x%02x\r\n", ret);
		return ret;
	} else {
		PRINTF("Add_Chat_Service() --> SUCCESS\r\n");
	}
	
	ret = Add_Chat_Service_test();
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error in Add_Chat_Service_test 0x%02x\r\n", ret);
		return ret;
	} else {
		PRINTF("Add_Chat_Service_test() --> SUCCESS\r\n");
	}

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT     
	ret = OTA_Add_Btl_Service();
	if(ret == BLE_STATUS_SUCCESS)
	PRINTF("OTA service added successfully.\n");
	else
	PRINTF("Error while adding OTA service.\n");
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 

#endif

	return BLE_STATUS_SUCCESS;
}

void Send_Data_Over_BLE(void) {
	if (!APP_FLAG(SEND_DATA) || APP_FLAG(TX_BUFFER_FULL))
		return;
	
	PRINTF("cmd string is: %s", cmd);

	while (cmd_buff_start < cmd_buff_end) {
		uint32_t len = MIN(20, cmd_buff_end - cmd_buff_start);

#if SERVER
		if (aci_gatt_update_char_value_ext(connection_handle, chatServHandle, AccGyroCharHandle, 1, len, 0, len, (uint8_t *) cmd + cmd_buff_start) == BLE_STATUS_INSUFFICIENT_RESOURCES) {
#elif CLIENT
			if (aci_gatt_write_without_resp(connection_handle, rx_handle + 1, len, (uint8_t *) cmd + cmd_buff_start) == BLE_STATUS_INSUFFICIENT_RESOURCES) {
#else
#error "Define SERVER or CLIENT"
#endif
			APP_FLAG_SET(TX_BUFFER_FULL);
			return;
		}
		cmd_buff_start += len;
	}

	// All data from buffer have been sent.
	APP_FLAG_CLEAR(SEND_DATA);
	cmd_buff_end = 0;
	NVIC_EnableIRQ(UART_IRQn);
}

/*******************************************************************************
 * Function Name  : Process_InputData.
 * Description    : Process a command. It should be called when data are received.
 * Input          : data_buffer: data address.
 *	           Nb_bytes: number of received bytes.
 * Return         : none.
 *******************************************************************************/
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes) {
	uint8_t i;

	for (i = 0; i < Nb_bytes; i++) {
		if (cmd_buff_end >= CMD_BUFF_SIZE - 1) {
			cmd_buff_end = 0;
		}

		cmd[cmd_buff_end] = data_buffer[i];
		SdkEvalComIOSendData(data_buffer[i]);
		cmd_buff_end++;

		if ((cmd[cmd_buff_end - 1] == '\n') || (cmd[cmd_buff_end - 1] == 0x0D)) {
			if (cmd_buff_end != 1) {

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
				if (!strncmp("OTAServiceManager", (char *) (cmd), 17)) {
					OTA_Jump_To_Service_Manager_Application();
				}
#endif

				cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.

				// Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
				APP_FLAG_SET(SEND_DATA);
				NVIC_DisableIRQ(UART_IRQn);

				cmd_buff_start = 0;

			} else {
				cmd_buff_end = 0; // Discard
			}
		}
	}
}

/*******************************************************************************
 * Function Name  : Make_Connection.
 * Description    : If the device is a Client create the connection. Otherwise puts
 *                  the device in discoverable mode.
 * Input          : none.
 * Return         : none.
 *******************************************************************************/
void Make_Connection(void) {
	tBleStatus ret;

#if CLIENT
	tBDAddr bdaddr = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};

	ret = aci_gap_create_connection(0x4000, 0x4000, PUBLIC_ADDR, bdaddr, PUBLIC_ADDR, 40, 40, 0, 60, 2000, 2000);
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error while starting connection: 0x%04x\r\n", ret);
		Clock_Wait(100);
	}

#else

	uint8_t local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'B', 'l', 'u', 'e', 'N', 'R', 'G', '1', '_', 'C', 'h', 'a', 't' };

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	hci_le_set_scan_response_data(18,BTLServiceUUID4Scan);
#else
	hci_le_set_scan_response_data(0, NULL);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 

	ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
	if (ret != BLE_STATUS_SUCCESS)
		PRINTF("Error in aci_gap_set_discoverable(): 0x%02x\r\n", ret);
	else
		PRINTF("aci_gap_set_discoverable() --> SUCCESS\r\n");
#endif
}

/*******************************************************************************
 * Function Name  : Init_Accelerometer_Gyroscope.
 * Description    : Init LSM6DSO accelerometer/gyroscope.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Accelerometer_Gyroscope(void) {

	uint8_t rst;

	lsm6dso_i3c_disable_set(0, LSM6DSO_I3C_DISABLE);

	rst = lsm6dso_reset_set(0, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(0, &rst);
	} while (rst);

	lsm6dso_pin_mode_set(0, LSM6DSO_PUSH_PULL);
	lsm6dso_pin_polarity_set(0, LSM6DSO_ACTIVE_LOW);
	lsm6dso_all_on_int1_set(0, PROPERTY_ENABLE);
	lsm6dso_int_notification_set(0, LSM6DSO_ALL_INT_LATCHED);

	lsm6dso_block_data_update_set(0, PROPERTY_ENABLE);
	lsm6dso_xl_power_mode_set(0, LSM6DSO_LOW_NORMAL_POWER_MD);
	lsm6dso_gy_power_mode_set(0, LSM6DSO_GY_NORMAL);
	lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
	lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz);
	//lsm6dso_xl_full_scale_set(0, LSM6DSO_2g);
	lsm6dso_xl_full_scale_set(0, LSM6DSO_16g);
	lsm6dso_gy_full_scale_set(0, LSM6DSO_2000dps);

	lsm6dso_auto_increment_set(0, PROPERTY_ENABLE);

//	InitHWFeatures();

}

/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : None
 *******************************************************************************/
void Sensor_DeviceInit(void) {
	uint8_t who_am_I_8 = 0x00;
	lsm6dso_device_id_get(0, &who_am_I_8);
	
	if (who_am_I_8 == LSM6DSO_ID) {
		PRINTF("ID: 0x%2x\r\n", who_am_I_8);
		PRINTF("Sensor LSM6DSOW detected successful\r\n");
		
		Init_Accelerometer_Gyroscope();
		
		lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
		lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF);
#if ENABLE_DEBUG_WIRELESS
		SdkEvalLedOn(LED1);
		SdkEvalLedOn(LED2);
		SdkEvalLedOff(LED3);
#endif
		PRINTF("Sensor in low-power: OK\r\n");
	}
	else {
		PRINTF("Sensor LSM6DSO detected failed\r\n");
	}
}

/*******************************************************************************
 * Function Name  : Acc_gyro_data_print.
 * Description    : Print data of acceleration and gyroscope.
 * Input          : none.
 * Return         : none.
 *******************************************************************************/
void acc_gyro_data_print(int8_t  format_flag) {
	if (format_flag == XENGINEINPUT_DATA) {
		// WSU
		xEngineInput.acc[0] = (((float) (acceleration_mg[0])) / 1000);
		xEngineInput.acc[1] = (((float) (acceleration_mg[1])) / 1000);
		xEngineInput.acc[2] = (((float) (acceleration_mg[2])) / 1000);
		xEngineInput.gyro[0] = (((float) (angular_rate_mdps[0])) / 1000);
		xEngineInput.gyro[1] = (((float) (angular_rate_mdps[1])) / 1000);
		xEngineInput.gyro[2] = (((float) (angular_rate_mdps[2])) / 1000);
		
		PRINTF("acceleration x = %f\r\n", xEngineInput.acc[0]);
		PRINTF("acceleration y = %f\r\n", xEngineInput.acc[1]);
		PRINTF("acceleration z = %f\r\n", xEngineInput.acc[2]);
		PRINTF("angular_rate x = %f\r\n", xEngineInput.gyro[0]);
		PRINTF("angular_rate y = %f\r\n", xEngineInput.gyro[1]);
		PRINTF("angular_rate z = %f\r\n", xEngineInput.gyro[2]);
		PRINTF("\r\n");
	}
	else {
		PRINTF("acceleration x = %d\r\n", acc_data.AXIS_X);
		PRINTF("acceleration y = %d\r\n", acc_data.AXIS_Y);
		PRINTF("acceleration z = %d\r\n", acc_data.AXIS_Z);
		PRINTF("angular_rate x = %d\r\n", gyro_data.AXIS_X);
		PRINTF("angular_rate y = %d\r\n", gyro_data.AXIS_Y);
		PRINTF("angular_rate z = %d\r\n", gyro_data.AXIS_Z);
		PRINTF("\r\n");
	}
		
}



/*******************************************************************************
 * Function Name  : APP_Tick.
 * Description    : Tick to run the application state machine.
 * Input          : none.
 * Return         : none.
 *******************************************************************************/
void APP_Tick(void) {
#if CLIENT
	tBleStatus ret;
#endif
	
	if (sleep_flag) {
		sleep_flag = 1;
#if ENABLE_DEBUG_WIRELESS
		SdkEvalLedOff(LED3);
		SdkEvalLedOff(LED1);
		SdkEvalLedOff(LED2);
#endif
		PRINTF("BlueNRG enter sleeping\r\n");
		BlueNRG_Sleep(SLEEPMODE_CPU_HALT, 0, 0);
	}

	if (timer_irq_flag && APP_FLAG(CONNECTED)) {
		lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
		lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz);
		memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(0, data_raw_acceleration.u8bit);
		
		/*Convert the data according to the range*/
//		acceleration_mg[0] = LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[0]);
//		acceleration_mg[1] = LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[1]);
//		acceleration_mg[2] = LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[2]);
		
		acceleration_mg[0] = LSM6DSO_FROM_FS_16g_TO_mg(data_raw_acceleration.i16bit[0]);
		acceleration_mg[1] = LSM6DSO_FROM_FS_16g_TO_mg(data_raw_acceleration.i16bit[1]);
		acceleration_mg[2] = LSM6DSO_FROM_FS_16g_TO_mg(data_raw_acceleration.i16bit[2]);

		memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso_angular_rate_raw_get(0, data_raw_angular_rate.u8bit);
		angular_rate_mdps[0] = LSM6DSO_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
		angular_rate_mdps[1] = LSM6DSO_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
		angular_rate_mdps[2] = LSM6DSO_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[2]);

		// WSU
		acc_data.AXIS_X = (int32_t) acceleration_mg[0];
		acc_data.AXIS_Y = (int32_t) acceleration_mg[1];
		acc_data.AXIS_Z = (int32_t) acceleration_mg[2];
		gyro_data.AXIS_X = (int32_t) angular_rate_mdps[0];
		gyro_data.AXIS_Y = (int32_t) angular_rate_mdps[1];
		gyro_data.AXIS_Z = (int32_t) angular_rate_mdps[2];

		acc_gyro_data_print(RAW_DATA);
		
		AccGyro_Update(&acc_data, &gyro_data);
		Clock_Wait(10);
	}
	
	if (free_fall_notify)
	{
		AccEvent_Notify(ACC_FREE_FALL, 2);
		free_fall_notify = 0;
	}

#if REQUEST_CONN_PARAM_UPDATE    
	if(APP_FLAG(CONNECTED) && !APP_FLAG(L2CAP_PARAM_UPD_SENT) && Timer_Expired(&l2cap_req_timer))
	{
		aci_l2cap_connection_parameter_update_req(connection_handle, 8, 16, 0, 600);
		APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
	}
#endif

#if CLIENT
	/* Start TX handle Characteristic discovery if not yet done */
	if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_TX_CHAR_HANDLE)) {
		if (!APP_FLAG(START_READ_TX_CHAR_HANDLE)) {
			/* Discovery TX characteristic handle by UUID 128 bits */

			const uint8_t charUuid128_TX[16] = {0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe1, 0xf2, 0x73, 0xd9};

			Osal_MemCpy(&UUID_Tx.UUID_16, charUuid128_TX, 16);
			ret = aci_gatt_disc_char_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, &UUID_Tx);
			if (ret != 0)
			PRINTF("Error in aci_gatt_disc_char_by_uuid() for TX characteristic: 0x%04xr\n", ret);
			else
			PRINTF("aci_gatt_disc_char_by_uuid() for TX characteristic --> SUCCESS\r\n");
			APP_FLAG_SET(START_READ_TX_CHAR_HANDLE);
		}
	}
	/* Start RX handle Characteristic discovery if not yet done */
	else if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_RX_CHAR_HANDLE)) {
		/* Discovery RX characteristic handle by UUID 128 bits */
		if (!APP_FLAG(START_READ_RX_CHAR_HANDLE)) {
			/* Discovery TX characteristic handle by UUID 128 bits */

			const uint8_t charUuid128_RX[16] = {0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe2, 0xf2, 0x73, 0xd9};

			Osal_MemCpy(&UUID_Rx.UUID_16, charUuid128_RX, 16);
			ret = aci_gatt_disc_char_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, &UUID_Rx);
			if (ret != 0)
			PRINTF("Error in aci_gatt_disc_char_by_uuid() for RX characteristic: 0x%04xr\n", ret);
			else
			PRINTF("aci_gatt_disc_char_by_uuid() for RX characteristic --> SUCCESS\r\n");

			APP_FLAG_SET(START_READ_RX_CHAR_HANDLE);
		}
	}

	if (APP_FLAG(CONNECTED) && APP_FLAG(END_READ_TX_CHAR_HANDLE) && APP_FLAG(END_READ_RX_CHAR_HANDLE) && !APP_FLAG(NOTIFICATIONS_ENABLED)) {
		uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
		struct timer t;
		Timer_Set(&t, CLOCK_SECOND * 10);

		while (aci_gatt_write_char_desc(connection_handle, tx_handle + 2, 2, client_char_conf_data) == BLE_STATUS_NOT_ALLOWED) { //TX_HANDLE;
			// Radio is busy.
			if (Timer_Expired(&t))
			break;
		}
		APP_FLAG_SET(NOTIFICATIONS_ENABLED);
	}
#endif

}/* end APP_Tick() */

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Role, uint8_t Peer_Address_Type, uint8_t Peer_Address[6], uint16_t Conn_Interval, uint16_t Conn_Latency, uint16_t Supervision_Timeout, uint8_t Master_Clock_Accuracy)
{
	connection_handle = Connection_Handle;

	APP_FLAG_SET(CONNECTED);

	sensorTimer_expired = false;
	HAL_VTimerStart_ms(SENSOR_TIMER, ENV_SENSOR_UPDATE_RATE);
	start_time = HAL_VTimerGetCurrentTime_sysT32();

#if ENABLE_DEBUG_WIRELESS	
	SdkEvalLedOn(LED1);
	SdkEvalLedOn(LED3);
	SdkEvalLedOff(LED2);
#endif
	PRINTF("connection complete!\r\n");
	
#if REQUEST_CONN_PARAM_UPDATE
	APP_FLAG_CLEAR(L2CAP_PARAM_UPD_SENT);
	Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Reason) {
	APP_FLAG_CLEAR(CONNECTED);
	/* Make the device connectable again. */
	APP_FLAG_SET(SET_CONNECTABLE);
	APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
	APP_FLAG_CLEAR(TX_BUFFER_FULL);

	APP_FLAG_CLEAR(START_READ_TX_CHAR_HANDLE);
	APP_FLAG_CLEAR(END_READ_TX_CHAR_HANDLE);
	APP_FLAG_CLEAR(START_READ_RX_CHAR_HANDLE);
	APP_FLAG_CLEAR(END_READ_RX_CHAR_HANDLE);

#if ENABLE_DEBUG_WIRELESS
	SdkEvalLedOn(LED2);
	SdkEvalLedOn(LED3);	
	SdkEvalLedOff(LED1);
#endif
	PRINTF("disconnection complete!\r\n");
 
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	OTA_terminate_connection();
#endif 
	
	Make_Connection();

}/* end hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Offset, uint16_t Attr_Data_Length, uint8_t Attr_Data[]) {
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 

	PRINTF("attribute modified\r\n");
	Attribute_Modified_CB(Attr_Handle, Attr_Data_Length, Attr_Data);
}

#if CLIENT

/*******************************************************************************
 * Function Name  : aci_gatt_notification_event.
 * Description    : This event occurs when a notification is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_notification_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint8_t Attribute_Value_Length, uint8_t Attribute_Value[]) {
#if CLIENT
	uint16_t attr_handle;

	attr_handle = Attribute_Handle;
	if (attr_handle == tx_handle + 1) {
		for (int i = 0; i < Attribute_Value_Length; i++)
		PRINTF("%c", Attribute_Value[i]);
	}
#endif
}

/*******************************************************************************
 * Function Name  : aci_gatt_disc_read_char_by_uuid_resp_event.
 * Description    : This event occurs when a discovery read characteristic by UUID response.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_disc_read_char_by_uuid_resp_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint8_t Attribute_Value_Length, uint8_t Attribute_Value[]) {
	PRINTF("aci_gatt_disc_read_char_by_uuid_resp_event, Connection Handle: 0x%04X\n", Connection_Handle);
	if (APP_FLAG(
					START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE)) {
		tx_handle = Attribute_Handle;
		PRINTF("TX Char Handle 0x%04X\n", tx_handle);
	} else if (APP_FLAG(
					START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE)) {
		rx_handle = Attribute_Handle;
		PRINTF("RX Char Handle 0x%04X\n", rx_handle);
	}
}

/*******************************************************************************
 * Function Name  : aci_gatt_proc_complete_event.
 * Description    : This event occurs when a GATT procedure complete is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_proc_complete_event(uint16_t Connection_Handle, uint8_t Error_Code) {
	if (APP_FLAG(
					START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE)) {
		PRINTF("aci_GATT_PROCEDURE_COMPLETE_Event for START_READ_TX_CHAR_HANDLE \r\n");
		APP_FLAG_SET(END_READ_TX_CHAR_HANDLE);
	} else if (APP_FLAG(
					START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE)) {
		PRINTF("aci_GATT_PROCEDURE_COMPLETE_Event for START_READ_TX_CHAR_HANDLE \r\n");
		APP_FLAG_SET(END_READ_RX_CHAR_HANDLE);
	}
}

#endif /* CLIENT */

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle, uint16_t Available_Buffers) {
	/* It allows to notify when at least 2 GATT TX buffers are available */
	APP_FLAG_CLEAR(TX_BUFFER_FULL);
}

/*******************************************************************************
 * Function Name  : aci_gatt_read_permit_req_event.
 * Description    : This event is given when a read request is received
 *                  by the server from the client.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Offset) {
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	/* Lower/Higher Applications with OTA Service */
	aci_gatt_allow_read(Connection_Handle);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
}

void aci_hal_end_of_radio_activity_event(uint8_t Last_State, uint8_t Next_State, uint32_t Next_State_SysTime) {
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	if (Next_State == 0x02) /* 0x02: Connection event slave */
	{
		OTA_Radio_Activity(Next_State_SysTime);
	}
#endif 
}

void BUTTON_IrqHandler(void) {
	if (GPIO_GetITPendingBit(GPIO_Pin_11)) {
		GPIO_EXTICmd(GPIO_Pin_11, DISABLE);
		GPIO_ClearITPendingBit(GPIO_Pin_11);
		
		sleep_flag = 0x01 & (~sleep_flag);
		
//		button_flag = 0x01 & (~button_flag);

#if !ENABLE_FREE_FALL		
		timer_irq_flag = 0x01 & (~timer_irq_flag);
		Clock_Clear();
#endif
		
		
#if ENABLE_DEBUG_WIRELESS
		SdkEvalLedOn(LED2);
		SdkEvalLedOff(LED1);
		SdkEvalLedOff(LED3); 
#endif

#if ENABLE_FREE_FALL
		if (!xHardwareFeaturePresence.HwFreeFall)
		{
			lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
			ResetHWPedometer();
			
			uint8_t data = 0;
			lsm6dso_read_reg(0, LSM6DSO_TAP_CFG0, &data, 8);
			data &= 0xfe;
			lsm6dso_write_reg(0, LSM6DSO_TAP_CFG0, &data, 8);
			lsm6dso_read_reg(0, LSM6DSO_TAP_CFG0, &data, 8);
			PRINTF("TAP_CFG0.LIR = %d\r\n", (data & 0x01));
			
			GPIO_EXTICmd(GPIO_Pin_13, ENABLE);
			EnableHWFreeFall();
			
			PRINTF("FreeFall enable\r\n");
			xHardwareFeaturePresence.HwFreeFall = true;			
		}
		else 
		{
			DisableHWFreeFall();
			PRINTF("FreeFall disable\r\n");
			xHardwareFeaturePresence.HwFreeFall = false;
		}
#endif

		GPIO_EXTICmd(GPIO_Pin_11, ENABLE);
	}
	
#if ENABLE_FREE_FALL
	if (GPIO_GetITPendingBit(GPIO_Pin_13))
	{
		GPIO_EXTICmd(GPIO_Pin_13, DISABLE);
		
		lsm6dso_all_sources_t all_source;
		lsm6dso_all_sources_get(0, &all_source);
		
		if (all_source.reg.all_int_src.ff_ia)
		{
			free_fall_notify = 1;
			PRINTF("Free Fall event\r\n");
#if ENABLE_DEBUG_WIRELESS
			SdkEvalLedOn(LED3);
			SdkEvalLedOff(LED1);	
			SdkEvalLedOff(LED2);
#endif
		}
		
		GPIO_ClearITPendingBit(GPIO_Pin_13);
		GPIO_EXTICmd(GPIO_Pin_13, ENABLE);
	}
#endif

}

void Timer_IrqHandler(void)
{
	timer_flag = 1;
}

