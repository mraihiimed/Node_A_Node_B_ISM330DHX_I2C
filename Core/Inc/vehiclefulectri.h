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

// Global variables
extern vehicleInfo_T vehicleData;
extern EngineeInfo_T engineData;

// Functions
void FuelEconomy_Ctrl(void);
void sendFuelEconomyMessage(uint8_t status);
void handleCANRxMessage(uint32_t id, uint8_t* data, uint8_t dlc);



#endif /* INC_VEHICLEFULECTRI_H_ */
