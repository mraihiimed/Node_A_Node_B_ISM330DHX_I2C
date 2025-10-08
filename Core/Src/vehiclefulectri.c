/*
 * vehiclefulectri.c
 *
 *  Created on: Oct 3, 2025
 *      Author: Geek
 */

#include "vehiclefulectri.h"
#include "main.h"
#include "dht.h"
#include <math.h>
#include <string.h>  // Add this if not already included
#include <stdio.h>
#include <stdlib.h>
// Global data storage
vehicleInfo_T vehicleData;
EngineeInfo_T engineData;
volatile uint8_t motionStatusFromNodeB = 0;
volatile uint8_t motionWarningFlag = 0;
volatile uint8_t motionCriticalFlag = 0;
volatile uint8_t motionFaultCounter = 0;
static uint8_t lastUnknownStatus ;
static uint8_t safetyTriggered;
volatile uint8_t motorControlEnabled = 0;

MotionStatus_T systemMotionState = MOTION_NORMAL;
extern CAN_HandleTypeDef hcan;

//Send Temperature and Humidity via DHT11
void sendTemperatureHumidityMessage(void)
{
   CAN_TxHeaderTypeDef txHeader;
   uint8_t txData[2];
   uint32_t txMailbox;

   txHeader.StdId=0x6FD;//Example ID for DHT11 status
   txHeader.IDE=CAN_ID_STD;
   txHeader.RTR=CAN_RTR_DATA;
   txHeader.DLC=2;

//   txData[0]=(uint8_t)(tCelsius*10); // scaled to 0.1°C
//   txData[1]=(uint8_t)(RH*10);       // scaled to 0.1%

   uint8_t temp = (uint8_t)(fminf(tCelsius * 10, 255));
   uint8_t hum  = (uint8_t)(fminf(RH * 10, 255));
   txData[0] = temp;
   txData[1] = hum;


   HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}


// Send fuel economy status
void sendFuelEconomyMessage(uint8_t status) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    TxHeader.StdId = 0x500;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    txData[0] = status;

    HAL_CAN_AddTxMessage(&hcan, &TxHeader, txData, &txMailbox);
}

// Decision logic
void FuelEconomy_Ctrl(void) {
    uint8_t fuelEconomyStatus = 0;

    uint8_t EngineSpeed = engineData.EngineeRPM;
    uint8_t VehicleSpeed = vehicleData.vehicleSpeed;

    if (EngineSpeed > 100) {
        if (VehicleSpeed > 0 && VehicleSpeed < 30)
            fuelEconomyStatus = 0; //0x14
        else if (VehicleSpeed >= 30 && VehicleSpeed < 60)
            fuelEconomyStatus = 1; //0x32
        else if (VehicleSpeed >= 60 && VehicleSpeed < 90)
            fuelEconomyStatus = 2; //0x50
        else if (VehicleSpeed >= 90 && VehicleSpeed < 120)
           fuelEconomyStatus = 3; //0x64
        else if (VehicleSpeed >= 120 && VehicleSpeed < 160)
            fuelEconomyStatus = 4;//0x82
        else if (VehicleSpeed >= 160 && VehicleSpeed < 200)
            fuelEconomyStatus = 5;//0xB4

        sendFuelEconomyMessage(fuelEconomyStatus);
    }
}
void handleCANRxMessage(uint32_t id, uint8_t* data, uint8_t dlc)
{
   	char msg[64];
    if (id == 0x402 && dlc == 1)
    {
        uint8_t status = data[0];

        switch (status)
        {
            case 0: // Normal
                processMotionStatusNormal();
                break;
            case 1: // Warning
                handleMotionWarning();
                break;
            case 2: // Critical
                handleMotionCritical();
                break;
            default: // Unknown
                handleUnknownMotionStatus(status);
                break;
        }

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Blink on feedback
    }
    else if (id == 0x200 && dlc >= sizeof(vehicleInfo_T))
    {
        memcpy(&vehicleData, data, sizeof(vehicleInfo_T));
        FuelEconomy_Ctrl(); // Trigger update after new vehicle data
    }
    else if (id == 0x300 && dlc >= sizeof(EngineeInfo_T))
    {
        memcpy(&engineData, data, sizeof(EngineeInfo_T));
        FuelEconomy_Ctrl(); // Trigger update after new engine data
    }
    if (id == 0x6FE && dlc == 1)
    {

    	//snprintf(msg, sizeof(msg), "Received CAN ID: 0x%03X, DLC: %d\r\n", id, dlc);
    	snprintf(msg, sizeof(msg), "Received CAN ID: 0x%03lX, DLC: %d\r\n", id, dlc);

    	uart_send_string(msg);

    	uint8_t status = data[0];

    	switch (status)
    	    {
    	        case 0: uart_send_string("System Status: NORMAL\r\n"); break;
    	        case 1: uart_send_string("System Status: WARNING\r\n"); break;
    	        case 2: uart_send_string("System Status: CRITICAL\r\n"); break;
    	        default: uart_send_string("System Status: UNKNOWN\r\n"); break;
    	    }

    	    // Optional: LED feedback
    	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, status == 2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        // 0 = Normal, 1 = Warning, 2 = Critical
       // TBD handleSystemStatus(status);
    }

}

void processMotionStatusNormal(void)
{
    // Example: reset flags or continue normal operation
	motionWarningFlag = 0;
	motionCriticalFlag = 0;
	motionFaultCounter = 0;
	systemMotionState = MOTION_NORMAL;
    uart_send_string("Normal: elevated motion detected\r\n");
	logMotionEvent("Normal: elevated motion detected\r\n");
}
void handleMotionWarning(void)
{
    // Example: reduce throttle, log warning, or trigger soft alert
    // You can set a flag or call a control function
	motionWarningFlag = 1;
	motionCriticalFlag = 0;
	systemMotionState = MOTION_WARNING;
    uart_send_string("Warning: elevated motion detected\r\n");
    logMotionEvent("Warning: elevated motion detected\r\n");
}

void handleMotionCritical(void)
{
    // Example: trigger emergency stop, log fault, or send diagnostic frame
	motionCriticalFlag = 1;
	motionWarningFlag = 0;
	systemMotionState = MOTION_CRITICAL;
    uart_send_string("Critical: unsafe motion detected\r\n");
	logMotionEvent("Critical: unsafe motion detected\r\n");
	triggerSafetyProtocol();
}

void handleUnknownMotionStatus(uint8_t status)
{
    char buffer[64];
    // Log or ignore unknown status codes
    systemMotionState = MOTION_UNKNOWN;
    lastUnknownStatus = status;
    snprintf(buffer, sizeof(buffer), "Unknown motion status received: 0x%02X\r\n", status);
    logMotionEvent(buffer);
}
void triggerSafetyProtocol(void)
{
    // Example: set a flag or disable actuators
    safetyTriggered = 1;

    // Optional: stop motors or enter safe mode
     motorControlEnabled = 0;
}
void logMotionEvent(const char* message)
{
	if(motorControlEnabled)
	   uart_send_string("log Motion Event\r\n");

       printf("Motion Event: %s\r\n", message);
}
