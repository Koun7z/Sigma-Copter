//
// Created by pwoli on 16.03.2025.
//

#ifndef FC_COMMONTYPES_H
#define FC_COMMONTYPES_H

#include <stdbool.h>

#include "Quaternion.h"

typedef struct
{
	float Motor1;  // FR
	float Motor2;  // FL
	float Motor3;  // RR
	float Motor4;  // RL
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

	float MagX;
	float MagY;
	float MagZ;

	DSP_Quaternion_f32 Attitude;
	DSP_Quaternion_f32 AttitudeTarget;
} FC_IMU_Data_Instance;

typedef enum
{
	FC_Good = 0,
	FC_ManualDisarm = 1,
	FC_BatteryLow = 2,
	FC_ConnectionLost = 3
} FC_StatusTypeDef;

#endif //FC_COMMONTYPES_H
