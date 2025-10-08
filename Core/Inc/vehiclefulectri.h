/*
 * vehiclefulectri.h
 *
 *  Created on: Oct 3, 2025
 *      Author: Geek
 */

#ifndef INC_VEHICLEFULECTRI_H_
#define INC_VEHICLEFULECTRI_H_



#include "main.h"

// Structs
typedef struct {
    uint8_t vehicleSpeed;
    uint8_t vehiclespeedstatus;
    uint8_t VehicleTemp;
    uint8_t VehiclePressure;
} vehicleInfo_T;

typedef struct {
    uint8_t EngineeRPM;
    uint8_t EngineeTemp;
    uint8_t EngineePressure;
    uint8_t Engineeweight;
} EngineeInfo_T;

typedef struct {
    uint8_t FuelWeight;
    uint8_t FuelStatus;
    uint8_t FuelPressure;
} FuelStatus_T;
typedef enum {
    MOTION_NORMAL = 0,
    MOTION_WARNING = 1,
    MOTION_CRITICAL = 2,
    MOTION_UNKNOWN = 255
} MotionStatus_T;

extern MotionStatus_T systemMotionState;


// Global variables
extern vehicleInfo_T vehicleData;
extern EngineeInfo_T engineData;

// Functions
void FuelEconomy_Ctrl(void);
void sendFuelEconomyMessage(uint8_t status);
void sendTemperatureHumidityMessage(void);

void handleCANRxMessage(uint32_t id, uint8_t* data, uint8_t dlc);

void processMotionStatusNormal(void);
void handleMotionWarning(void);
void handleMotionCritical(void);
void handleUnknownMotionStatus(uint8_t status);
void triggerSafetyProtocol(void);
void logMotionEvent(const char* message);

int __io_putchar(int ch);
#endif /* INC_VEHICLEFULECTRI_H_ */
