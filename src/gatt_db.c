#include <stdio.h>
#include <string.h>
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "osal.h"
#include "app_state.h"
#include "SDK_EVAL_Config.h"
#include "chat.h"
#include "gatt_db.h"

#include "clock.h"


#if ENABLE_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

uint16_t chatServHandle, AccGyroCharHandle, RXCharHandle, TXCharHandle;
uint16_t chatServHandle_t, TXCharHandle_t, RXCharHandle_t;

extern volatile uint32_t start_time;
extern uint16_t time_set;
extern uint8_t timer_irq_flag;

uint8_t buff[20];

/* UUIDs */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;

uint32_t getTimestamp(void){
	return HAL_VTimerDiff_ms_sysT32(HAL_VTimerGetCurrentTime_sysT32(), start_time);
}

/*******************************************************************************
 * Function Name  : Add_Chat_Service
 * Description    : Add the Chat service.
 * Input          : None
 * Return         : Status.
 *******************************************************************************/
uint8_t Add_Chat_Service(void) {
	uint8_t ret;

	/*
	 UUIDs:
	 D973F2E0-B19E-11E2-9E96-0800200C9A66
	 D973F2E1-B19E-11E2-9E96-0800200C9A66
	 D973F2E2-B19E-11E2-9E96-0800200C9A66
75a028a0-53c3-11ea-aaef-0800200c9a66
75a028a1-53c3-11ea-aaef-0800200c9a66
75a028a2-53c3-11ea-aaef-0800200c9a66
	 */

	const uint8_t uuid[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe0, 0xf2, 0x73, 0xd9 };
	const uint8_t charUuidTX[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe1, 0xf2, 0x73, 0xd9 };
	const uint8_t charUuidRX[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe2, 0xf2, 0x73, 0xd9 };
	const uint8_t charUuidTX_t[16] = {0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe3, 0xf2, 0x73, 0xd9 };
	
	Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 9, &chatServHandle);
	if (ret != BLE_STATUS_SUCCESS)
		goto fail;

	Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTX, 16);
	ret = aci_gatt_add_char(chatServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0, 16, 1, &AccGyroCharHandle);
	if (ret != BLE_STATUS_SUCCESS)
		goto fail;
	
	Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidRX, 16);
	ret = aci_gatt_add_char(chatServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &RXCharHandle);
	if (ret != BLE_STATUS_SUCCESS) 
		goto fail;

	Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTX_t, 16);
	ret = aci_gatt_add_char(chatServHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0, 16, 1, &TXCharHandle);
	if (ret != BLE_STATUS_SUCCESS)
		goto fail;
	
	PRINTF("Chat Service added.\r\nTX Char Handle %04X, RX Char Handle %04X\r\n", AccGyroCharHandle, RXCharHandle);
	return BLE_STATUS_SUCCESS;

	fail: PRINTF("Error while adding Chat service.\r\n");
	return BLE_STATUS_ERROR;

}


uint8_t Add_Chat_Service_test(void) {
	uint8_t ret;

	/*
	 UUIDs:
75a028a0-53c3-11ea-aaef-0800200c9a66
75a028a1-53c3-11ea-aaef-0800200c9a66
75a028a2-53c3-11ea-aaef-0800200c9a66
	 */

	const uint8_t uuid[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0xef, 0xaa, 0xea, 0x11, 0xc3, 0x53, 0xa0, 0x28, 0xa0, 0x75 };
	const uint8_t charUuidTX_t[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0xef, 0xaa, 0xea, 0x11, 0xc3, 0x53, 0xa1, 0x28, 0xa0, 0x75 };
	const uint8_t charUuidRX_t[16] = { 0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0xef, 0xaa, 0xea, 0x11, 0xc3, 0x53, 0xa2, 0x28, 0xa0, 0x75 };
	
	Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 6, &chatServHandle_t);
	if (ret != BLE_STATUS_SUCCESS)
		goto fail_t;

	Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidTX_t, 16);
	ret = aci_gatt_add_char(chatServHandle_t, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0, 16, 1, &TXCharHandle_t);
	if (ret != BLE_STATUS_SUCCESS)
		goto fail_t;

	Osal_MemCpy(&char_uuid.Char_UUID_128, charUuidRX_t, 16);
	ret = aci_gatt_add_char(chatServHandle_t, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &RXCharHandle_t);
	if (ret != BLE_STATUS_SUCCESS)
		goto fail_t;
	
	PRINTF("Chat Service added.\r\nTX Char Handle %04X, RX Char Handle %04X\r\n", TXCharHandle_t, RXCharHandle_t);
	return BLE_STATUS_SUCCESS;

	fail_t: PRINTF("Error while adding Chat service.\r\n");
	return BLE_STATUS_ERROR;
}

/*******************************************************************************
 * Function Name  : Attribute_Modified_CB
 * Description    : Callback when RX/TX attribute handle is modified.
 * Input          : Handle of the characteristic modified. Handle of the attribute
 *                  modified and data written
 * Return         : None.
 *******************************************************************************/
void Attribute_Modified_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data) {
	PRINTF("call back complete!\r\n");
	if (handle == RXCharHandle + 1) {
//		for (int i = 0; i < data_length; i++)
//			PRINTF("%c", att_data[i]);
		sscanf((const char *)att_data, "%d", (int *)&time_set);
	} else if (handle == AccGyroCharHandle + 2) {
		if (att_data[0] == 0x01)
			APP_FLAG_SET(NOTIFICATIONS_ENABLED);
	} else if (handle == TXCharHandle + 2) {
		if (att_data[0] == 0x01)
			APP_FLAG_SET(NOTIFICATIONS_ENABLED);
	} else if (handle == TXCharHandle_t + 2) {
		if (att_data[0] == 0x01)
			APP_FLAG_SET(NOTIFICATIONS_ENABLED);
	} else if (handle == RXCharHandle_t + 1) {
		timer_irq_flag = 0x01 & (~timer_irq_flag);
		Clock_Clear();
	}
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  SensorAxes_t Acc Structure containing acceleration value in mg
 * @param  SensorAxes_t Gyro Structure containing Gyroscope value
 * @param  SensorAxes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyro_Update(AxesRaw_t *Acc, AxesRaw_t *Gyro) {

	STORE_LE_16(buff, (getTimestamp()));

	STORE_LE_16(buff + 2, Acc->AXIS_X);
	STORE_LE_16(buff + 4, Acc->AXIS_Y);
	STORE_LE_16(buff + 6, Acc->AXIS_Z);

	Gyro->AXIS_X /= 10;
	Gyro->AXIS_Y /= 10;
	Gyro->AXIS_Z /= 10;

	STORE_LE_16(buff + 8, Gyro->AXIS_X);
	STORE_LE_16(buff + 10, Gyro->AXIS_Y);
	STORE_LE_16(buff + 12, Gyro->AXIS_Z);
	STORE_LE_16(buff + 14, '\0');
	
	return  aci_gatt_update_char_value(chatServHandle, AccGyroCharHandle, 0, 2 + 3 * 3 * 2, buff);
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte) {
	tBleStatus ret = 0;
	uint8_t buff_2[2 + 2];
	uint8_t buff_3[2 + 3];

	switch (dimByte) {
	case 2:
		STORE_LE_16(buff_2, (getTimestamp()));
		STORE_LE_16(buff_2 + 2, Command);
		ret = aci_gatt_update_char_value(chatServHandle, TXCharHandle, 0, 2 + 2, buff_2);
		break;
	case 3:
		STORE_LE_16(buff_3, (getTimestamp()));
		buff_3[2] = 0;
		STORE_LE_16(buff_3 + 3, Command);
		ret = aci_gatt_update_char_value(chatServHandle, TXCharHandle, 0, 2 + 3, buff_3);
		break;
	}
	return ret;
}
 
