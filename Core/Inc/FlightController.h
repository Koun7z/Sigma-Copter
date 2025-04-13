/*
** FlightController.h
**
**  Created on: Jan 28, 2025
**      Author: pwoli
*/

#ifndef INC_FLIGHTCONTROLLER_H_
#define INC_FLIGHTCONTROLLER_H_

#include "FC_Config.h"
#include "FC_CommonTypes.h"
#include "ProcessControl.h"

#include <stdint.h>


extern FC_MotorThrust FC_GlobalThrust;
extern FC_RC_Data_Instance FC_RC_Data;
extern FC_IMU_Data_Instance FC_IMU_Data;


uint8_t FC_Init(void);

void FC_Update(float dt);

void FC_EmergencyDisarm(void);

void FC_EmergencyRearm(void);

void FC_DataAcquisitionInit(void);

void FC_RC_UpdateAxisChannels(int throttle, int pitch, int roll, int yaw);

void FC_RC_UpdateAuxChannels(int aux1, int aux2, int aux3, int aux4);

void FC_RC_UpdateArmStatus(bool armStatus);

void FC_RC_SetFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

void FC_IMU_UpdateGyro(float pitchRate, float rollRate, float yawRate, float dt);

void FC_IMU_SetGyroFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

void FC_IMU_UpdateAccel(float accelX, float accelY, float accelZ, float dt);

void FC_IMU_SetAccelFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

#endif /* INC_FLIGHTCONTROLLER_H_ */
