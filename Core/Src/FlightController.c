/*
 * FlightController.c
 *
 *  Created on: Jan 28, 2025
 *      Author: pwoli
 */

#include "FlightController.h"

#include "ProcessControl.h"
#include <sys\stat.h>

#include <string.h>

PID_Instance_f32 FC_PitchRatePID;
PID_Instance_f32 FC_RollRatePID;
PID_Instance_f32 FC_YawRatePID;

FC_MotorThrust FC_GlobalThrust;
FC_RC_Data_Instance FC_RC_Data;
FC_IMU_Data_Instance FC_IMU_Data;

bool manualDisarm = false;

static float clamp(const float num, const float min, const float max)
{
	const float t = num < min ? min : num;
	return t > max ? max : t;
}

uint8_t FC_Init()
{
	FC_GlobalThrust.Motor1 = 0;
	FC_GlobalThrust.Motor2 = 0;
	FC_GlobalThrust.Motor3 = 0;
	FC_GlobalThrust.Motor4 = 0;

	PID_Init_f32(&FC_PitchRatePID, 1.5f, 0.0f, 0);

	PID_Init_f32(&FC_RollRatePID, 1.5f, 0.0f, 0);

	PID_Init_f32(&FC_YawRatePID, 1.5f, 0.0f, 0);

	FC_DataAcquisitionInit();
	return 0;
}


void FC_Update(const float dt)
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

	float m1 = throttleCommand + pitchOffset - rollOffset - yawOffset;
	float m2 = throttleCommand + pitchOffset + rollOffset + yawOffset;
	float m3 = throttleCommand - pitchOffset - rollOffset + yawOffset;
	float m4 = throttleCommand - pitchOffset + rollOffset - yawOffset;

	// Find max throttle
	float max = m1 > m2 ? m1 : m2;
	max       = max > m3 ? max : m3;
	max       = max > m3 ? max : m4;

	// Find min throttle
	float min = m1 < m2 ? m1 : m2;
	min       = min < m3 ? min : m3;
	min       = min < m4 ? min : m4;

	// Adjust for min/max throttle values to maintain motor offset
	float r = 0;
	if(max > 100.0f)
	{
		r = 100.0f - max;

		if((min + r) < FC_IDLE_THROTTLE)
		{
			r += (FC_IDLE_THROTTLE - (min + r)) / 2;
		}
	}
	else if(min < FC_IDLE_THROTTLE)
	{
		r = FC_IDLE_THROTTLE - min;

		if((max + r) > 100.0f)
		{
			r += (100.0f - (max + r)) / 2;
		}
	}

	m1 += r;
	m2 += r;
	m3 += r;
	m4 += r;

	FC_GlobalThrust.Motor1 = clamp(m1, FC_IDLE_THROTTLE, 100.0f) * (FC_MAX_THROTTLE / 100.0f);
	FC_GlobalThrust.Motor2 = clamp(m2, FC_IDLE_THROTTLE, 100.0f) * (FC_MAX_THROTTLE / 100.0f);
	FC_GlobalThrust.Motor3 = clamp(m3, FC_IDLE_THROTTLE, 100.0f) * (FC_MAX_THROTTLE / 100.0f);
	FC_GlobalThrust.Motor4 = clamp(m4, FC_IDLE_THROTTLE, 100.0f) * (FC_MAX_THROTTLE / 100.0f);

#if FC_SERIAL_DEBUG
	static float debug_timer = 0;
	debug_timer             += dt;
	if(debug_timer > 1.0f / FC_SERIAL_REPORT_RATE)
	{
		debug_timer = 0;
		FC_DEBUG_LOG("FC_Attitude: %f, %f, %f, %f \n\r", FC_IMU_Data.Attitude.r, FC_IMU_Data.Attitude.i,
		             FC_IMU_Data.Attitude.j, FC_IMU_Data.Attitude.k);

		FC_DEBUG_LOG("FC_AXES: %f, %f, %f, %f \n\r", FC_RC_Data.Throttle, FC_RC_Data.Roll, FC_RC_Data.Pitch,
		             FC_RC_Data.Yaw);
	}
#endif
}

void FC_EmergencyDisarm()
{
	manualDisarm = true;

	FC_GlobalThrust.Motor1 = 0.0f;
	FC_GlobalThrust.Motor2 = 0.0f;
	FC_GlobalThrust.Motor3 = 0.0f;
	FC_GlobalThrust.Motor4 = 0.0f;
}

void FC_EmergencyRearm()
{
	manualDisarm = false;
}
