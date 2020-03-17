

#ifndef _GATT_DB_H_
#define _GATT_DB_H_

/** 
 * @brief Structure containing acceleration value of each axis.
 */
typedef struct {
	int32_t AXIS_X;
	int32_t AXIS_Y;
	int32_t AXIS_Z;
} AxesRaw_t;

tBleStatus Add_Chat_Service(void);
tBleStatus Add_Chat_Service_test(void);
void Attribute_Modified_CB(uint16_t handle, uint16_t data_length, uint8_t *att_data);

tBleStatus AccGyro_Update(AxesRaw_t *Acc, AxesRaw_t *Gyro);
tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte);


extern uint16_t chatServHandle, AccGyroCharHandle, RXCharHandle, TXCharHandle;
extern uint16_t chatServHandle_t, TXCharHandle_t, RXCharHandle_t;

#endif /* _GATT_DB_H_ */
