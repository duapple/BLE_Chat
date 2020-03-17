
#ifndef _CHAT_H_
#define _CHAT_H_

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_TIMER 		0		// Fixed ODR @ 25 Hz
#define ADVERTISING_TIMER 	1		// Fixed @ 20 seconds
#define RESET_TIMER			2		// Fixed 5 seconds

#define ADVERTISING_TIME	20000	// Time to advertise before enter standby [ms]
#define RESET_TIME 			5000	// Time to wait before reset [ms]

#define BATTERY_UPDATE_RATE			1000    // Fixed ODR @  1 Hz
#define ENV_SENSOR_UPDATE_RATE		100     // Fixed ODR @  10 Hz
#define MOTION_SENSOR_UPDATE_RATE 	40		// Fixed ODR @ 25 Hz

typedef struct {
	bool AccelerometerGyroscopePresence;
	bool MagnetometerPresence;
	bool HumidityTemperaturePresence;
	bool PressurePresence;
	bool ProximityLightPresence;
	bool iNemoEngine;
	bool Pedometer;
} FeaturePresence;

typedef struct {
	bool HwFreeFall;
	bool HwPedometer;
	bool HwDoubleTAP;
	bool HwSingleTAP;
	bool HwWakeUp;
	bool HwTilt;
	bool HwOrientation6D;
	bool MultipleEvent;
} HardwareFeaturePresence;

typedef struct {
	bool MotionNotification;
	bool EnvironmentalNotification;
	bool ProximityNotification;
	bool iNemoEngineNotification;
	bool BatteryMonitorNotification;
	bool LedNotification;
	bool eCompassNotification;
	bool AudioNotification;
	bool SyncNotification;
	bool AccEventNotification;
	bool Advertising;
} FeatureNotification;


uint8_t CHAT_DeviceInit(void);
void APP_Tick(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);

void Make_Connection(void);
void Sensor_DeviceInit(void);

void BUTTON_IrqHandler(void);
void Timer_IrqHandler(void);

#endif // _CHAT_H_
