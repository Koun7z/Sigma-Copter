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

#define FC_CHANNEL_MAX      1984
#define FC_CHANNEL_MIN      0
#define FC_CHANNEL_MIDPOINT ((FC_CHANNEL_MAX - FC_CHANNEL_MIN) / 2)

#define FC_THROTTLE_RESOLUTION 2048
#define FC_IDLE_THROTTLE       50

#define FC_PITCH_LINEAR_RATE 200
#define FC_ROLL_LINEAR_RATE  200
#define FC_YAW_LINEAR_RATE   200

// RC data filtering
#define FC_RC_AXIS_FILTER_ENABLE 0
#define FC_RC_AXIS_FILTER_TYPE   2  // FIR = 1, IIR = 2
#define FC_RC_AXIS_FILTER_ORDER  2

// IMU data filtering
#define FC_IMU_GYRO_FILTER_ENABLE 0
#define FC_IMU_GYRO_FILTER_TYPE   2  // FIR = 1, IIR = 2
#define FC_IMU_GYRO_FILTER_ORDER  2

typedef struct
{
	int Motor1;  // FR
	int Motor2;  // FL
	int Motor3;  // RR
	int Motor4;  // RL
} FC_MotorThrust;

extern PID_Instance_f32 PitchRatePID;
extern PID_Instance_f32 RollRatePID;
extern PID_Instance_f32 YawRatePID;

extern FC_MotorThrust FC_GlobalThrust;

size_t FC_Init();

void FC_Update(float dt);

void FC_EmergencyDisarm();
void FC_EmergencyRearm();

void FC_RC_UpdateAxisChannels(int throttle, int pitch, int roll, int yaw);
void FC_RC_UpdateAuxChannels(int aux1, int aux2, int aux3, int aux4);
void FC_RC_UpdateArmStatus(bool armStatus);

bool FC_RC_SetFilter(size_t filterOrder, float* filterCoeffsNumerator, float* filterCoeffsDenominator);

void FC_IMU_UpdateGyro(float pitchRate, float rollRate, float yawRate);

bool FC_IMU_SetGyroFilter(size_t filterOrder, float* filterCoeffsNumerator, float* filterCoeffsDenominator);

void FC_IMU_UpdateAccel(float pitchRate, float rollRate, float yawRate);

#endif /* INC_FLIGHTCONTROLLER_H_ */
