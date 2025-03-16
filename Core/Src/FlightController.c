/*
 * FlightController.c
 *
 *  Created on: Jan 28, 2025
 *      Author: pwoli
 */

#include "FlightController.h"

#include "ProcessControl.h"

#include <string.h>

PID_Instance_f32 FC_PitchRatePID;
PID_Instance_f32 FC_RollRatePID;
PID_Instance_f32 FC_YawRatePID;

FC_MotorThrust FC_GlobalThrust;
FC_RC_Data_Instance FC_RC_Data;
FC_IMU_Data_Instance FC_IMU_Data;

bool manualDisarm = false;

static int clamp(int num, int min, int max)
{
	const int t = num < min ? min : num;
	return t > max ? max : t;
}

size_t FC_Init()
{
	FC_GlobalThrust.Motor1 = 0;
	FC_GlobalThrust.Motor2 = 0;
	FC_GlobalThrust.Motor3 = 0;
	FC_GlobalThrust.Motor4 = 0;

	PID_Init_f32(&FC_PitchRatePID, 2, 0, 0);

	// FC_PitchRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	// FC_PitchRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);


	PID_Init_f32(&FC_RollRatePID, 2, 0, 0);

	// FC_RollRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	// FC_RollRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);


	PID_Init_f32(&FC_YawRatePID, 2, 0, 0);

	// FC_YawRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	// FC_YawRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);

	FC_FiltersInit();
	return 0;
}


void FC_Update(float dt)
{
	if(manualDisarm || !FC_RC_Data.Arm)
	{
		FC_GlobalThrust.Motor1 = 0;
		FC_GlobalThrust.Motor2 = 0;
		FC_GlobalThrust.Motor3 = 0;
		FC_GlobalThrust.Motor4 = 0;

		return;
	}

	const float pitchCommand    = FC_RC_Data.Pitch;
	const float rollCommand     = FC_RC_Data.Roll;
	const float yawCommand      = FC_RC_Data.Yaw;
	const float throttleCommand = FC_RC_Data.Throttle;

	const float pitchRate = FC_IMU_Data.GyroY;
	const float rollRate  = FC_IMU_Data.GyroX;
	const float yawRate   = FC_IMU_Data.GyroZ;

	const float pitchOffset = PID_Update_f32(&FC_PitchRatePID, pitchCommand - pitchRate, dt) * 0.5f;
	const float rollOffset  = PID_Update_f32(&FC_RollRatePID, rollCommand - rollRate, dt) * 0.5f;
	const float yawOffset   = PID_Update_f32(&FC_YawRatePID, yawCommand - yawRate, dt) * 0.5f;

	int m1 = (int)(throttleCommand - pitchOffset - rollOffset - yawOffset);
	int m2 = (int)(throttleCommand - pitchOffset + rollOffset + yawOffset);
	int m3 = (int)(throttleCommand + pitchOffset - rollOffset + yawOffset);
	int m4 = (int)(throttleCommand + pitchOffset + rollOffset - yawOffset);

	// Find max throttle
	int max = m1 > m2 ? m1 : m2;
	max     = max > m3 ? max : m3;
	max     = max > m3 ? max : m4;

	// Find min throttle
	int min = m1 < m2 ? m1 : m2;
	min     = min < m3 ? min : m3;
	min     = min < m4 ? min : m4;


	// Adjust for min/max throttle values to maintain motor offset
	int r = 0;
	if(max > FC_THROTTLE_RESOLUTION)
	{
		r = FC_THROTTLE_RESOLUTION - max;

		if((min + r) < FC_IDLE_THROTTLE)
		{
			r += (FC_IDLE_THROTTLE - (min + r)) / 2;
		}
	}
	else if(min < FC_IDLE_THROTTLE)
	{
		r = FC_IDLE_THROTTLE - min;

		if((max + r) > FC_THROTTLE_RESOLUTION)
		{
			r += (FC_THROTTLE_RESOLUTION - (max + r)) / 2;
		}
	}

	m1 += r;
	m2 += r;
	m3 += r;
	m4 += r;

	FC_GlobalThrust.Motor1 = clamp(m1, FC_IDLE_THROTTLE, FC_THROTTLE_RESOLUTION);
	FC_GlobalThrust.Motor2 = clamp(m2, FC_IDLE_THROTTLE, FC_THROTTLE_RESOLUTION);
	FC_GlobalThrust.Motor3 = clamp(m3, FC_IDLE_THROTTLE, FC_THROTTLE_RESOLUTION);
	FC_GlobalThrust.Motor4 = clamp(m4, FC_IDLE_THROTTLE, FC_THROTTLE_RESOLUTION);
}

void FC_EmergencyDisarm()
{
	manualDisarm = true;

	FC_GlobalThrust.Motor1 = 0;
	FC_GlobalThrust.Motor2 = 0;
	FC_GlobalThrust.Motor3 = 0;
	FC_GlobalThrust.Motor4 = 0;
}
void FC_EmergencyRearm()
{
	manualDisarm = false;
}
