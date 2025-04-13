//
// Created by pwoli on 16.03.2025.
//

#ifndef FC_COMMONTYPES_H
#define FC_COMMONTYPES_H

#include <stdbool.h>

#include "Quaternion.h"

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

	DSP_Quaternion_f32 Attitude;
} FC_IMU_Data_Instance;

#endif //FC_COMMONTYPES_H
