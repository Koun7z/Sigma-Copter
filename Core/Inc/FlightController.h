/*
 * FlightController.h
 *
 *  Created on: Jan 28, 2025
 *      Author: pwoli
 */

#ifndef INC_FLIGHTCONTROLLER_H_
#define INC_FLIGHTCONTROLLER_H_

#include "ProcessControl.h"

#include <stdint.h>

// If false, Aux1 channel controls arming
#define FC_EXTERNAL_ARM_STATUS 1

#define FC_CHANNEL_MAX      1812
#define FC_CHANNEL_MIN      172
#define FC_CHANNEL_MIDPOINT ((FC_CHANNEL_MAX + FC_CHANNEL_MIN) / 2)

#define FC_THROTTLE_RESOLUTION 2048
#define FC_IDLE_THROTTLE       50

#define FC_PITCH_LINEAR_RATE 200
#define FC_ROLL_LINEAR_RATE  200
#define FC_YAW_LINEAR_RATE   200

// RC data filtering
#define FC_RC_AXIS_FILTER_ENABLE 1
#define FC_RC_AXIS_FILTER_TYPE   2  // FIR = 1, IIR = 2
#define FC_RC_AXIS_FILTER_ORDER  2

// IMU data filtering
#define FC_IMU_GYRO_FILTER_ENABLE 1
#define FC_IMU_GYRO_FILTER_TYPE   2  // FIR = 1, IIR = 2
#define FC_IMU_GYRO_FILTER_ORDER  2

typedef struct
{
	int Motor1;  // FR
	int Motor2;  // FL
	int Motor3;  // RR
	int Motor4;  // RL
} FC_MotorThrust;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
	float Throttle;

	float Aux1;
	float Aux2;
	float Aux3;
	float Aux4;

	bool Arm;
} FC_RC_Data_Instance;

typedef struct
{
	float GyroX;
	float GyroY;
	float GyroZ;

	float AccelX;
	float AccelY;
	float AccelZ;

	float PitchAngle;
	float RollAngle;
} FC_IMU_Data_Instance;


extern FC_IMU_Data_Instance FC_IMU_Data;
extern FC_RC_Data_Instance FC_RC_Data;

extern FC_MotorThrust FC_GlobalThrust;


size_t FC_Init();

void FC_Update(float dt);

void FC_EmergencyDisarm();
void FC_EmergencyRearm();

void FC_RC_UpdateAxisChannels(int throttle, int pitch, int roll, int yaw);
void FC_RC_UpdateAuxChannels(int aux1, int aux2, int aux3, int aux4);
void FC_RC_UpdateArmStatus(bool armStatus);

void FC_RC_SetFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

void FC_IMU_UpdateGyro(float pitchRate, float rollRate, float yawRate);

void FC_IMU_SetGyroFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

void FC_IMU_UpdateAccel(float pitchRate, float rollRate, float yawRate);

#endif /* INC_FLIGHTCONTROLLER_H_ */
