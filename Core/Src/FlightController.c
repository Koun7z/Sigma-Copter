/*
 * FlightController.c
 *
 *  Created on: Jan 28, 2025
 *      Author: pwoli
 */

#include "FlightController.h"

#include "CRSF_Connection.h"
#include "MPU6050.h"
#include "PID.h"

#define THROTTLE CRSF_Channels.Ch3
#define PITCH CRSF_Channels.Ch2
#define ROLL CRSF_Channels.Ch1
#define YAW CRSF_Channels.Ch4
#define ARM CRSF_ArmStatus

#define CHANNEL_MAX 1984
#define CHANNEL_MIN 0
#define CHANNEL_MIDPOINT ((CHANNEL_MAX - CHANNEL_MIN) / 2)

#define PITCH_LINEAR_RATE 200
#define ROLL_LINEAR_RATE 200
#define YAW_LINEAR_RATE 200

#define THROTTLE_RESOLUTION 2048
#define IDLE_THROTTLE 50

struct PID_State PitchRatePID;
struct PID_State RollRatePID;
struct PID_State YawRatePID;

struct FC_MotorThrust FC_GlobalThrust;
struct IMU_Data FC_IMUData;

bool _manualDisarm = false;

static uint16_t _clamp(uint16_t num, uint16_t max, uint16_t min)
{
	const uint16_t t = num < min ? min : num;
	return t > max ? max : t;
}

uint8_t FC_Init()
{
	FC_GlobalThrust.Motor1 = 0;
	FC_GlobalThrust.Motor2 = 0;
	FC_GlobalThrust.Motor3 = 0;
	FC_GlobalThrust.Motor4 = 0;

	if(!PID_Init(&PitchRatePID, 3, 0, 0))
	{
		return 3;
	}

	PitchRatePID.MaxSaturation = THROTTLE_RESOLUTION / 4;
	PitchRatePID.MinSaturation = -(THROTTLE_RESOLUTION / 4);

	PitchRatePID.MaxWindup = THROTTLE_RESOLUTION / 4;
	PitchRatePID.MinWindup = -(THROTTLE_RESOLUTION / 4);

	if(!PID_Init(&RollRatePID, 3, 0, 0))
	{
		return 3;
	}

	RollRatePID.MaxSaturation = THROTTLE_RESOLUTION / 4;
	RollRatePID.MinSaturation = -(THROTTLE_RESOLUTION / 4);

	RollRatePID.MaxWindup = THROTTLE_RESOLUTION / 4;
	RollRatePID.MinWindup = -(THROTTLE_RESOLUTION / 4);

	if(!PID_Init(&YawRatePID, 3, 0, 0))
	{
		return 3;
	}

	YawRatePID.MaxSaturation = THROTTLE_RESOLUTION / 4;
	YawRatePID.MinSaturation = -(THROTTLE_RESOLUTION / 4);

	YawRatePID.MaxWindup = THROTTLE_RESOLUTION / 4;
	YawRatePID.MinWindup = -(THROTTLE_RESOLUTION / 4);

	return 0;
}

void FC_Update(float dt)
{
	if(_manualDisarm || !ARM)
	{
		FC_GlobalThrust.Motor1 = 0;
		FC_GlobalThrust.Motor2 = 0;
		FC_GlobalThrust.Motor3 = 0;
		FC_GlobalThrust.Motor4 = 0;

		return;
	}

	float yaw = (YAW - CHANNEL_MIDPOINT) * YAW_LINEAR_RATE / CHANNEL_MIDPOINT;	    // deg/s
	float pitch = (PITCH - CHANNEL_MIDPOINT)* PITCH_LINEAR_RATE / CHANNEL_MIDPOINT; // deg/s
	float roll = (ROLL - CHANNEL_MIDPOINT) * ROLL_LINEAR_RATE / CHANNEL_MIDPOINT;   // deg/s
	float throttle = THROTTLE * THROTTLE_RESOLUTION / CHANNEL_MAX;

	float pitchRate = FC_IMUData.GyroX;
	float rollRate = FC_IMUData.GyroY;
	float yawRate = FC_IMUData.GyroZ;

	float pitchOffset = PID_Update(&PitchRatePID, pitch - pitchRate, dt) * 0.5;
	float rollOffset = PID_Update(&RollRatePID, roll - rollRate, dt) * 0.5;
	float yawOffset = PID_Update(&YawRatePID, yaw - yawRate, dt) * 0.5;

	uint16_t m1 = throttle - pitchOffset - rollOffset - yawOffset;
	uint16_t m2 = throttle - pitchOffset + rollOffset + yawOffset;
	uint16_t m3 = throttle + pitchOffset - rollOffset + yawOffset;
	uint16_t m4 = throttle + pitchOffset + rollOffset - yawOffset;

	// Find max throttle
	uint16_t max = m1 > m2? m1 : m2;
	max = max > m3? max : m3;
	max = max > m3? max : m4;

	//Find min throttle
	uint16_t min = m1 < m2? m1 : m2;
	min = min < m3? min : m3;
	min = min < m4? min : m4;

	int16_t r = 0;
	if(max > THROTTLE_RESOLUTION)
	{
		r = THROTTLE_RESOLUTION - max;

		if((min + r) < IDLE_THROTTLE)
		{
			r += (IDLE_THROTTLE - (min + r)) / 2;
		}
	}
	else if(min < IDLE_THROTTLE)
	{
		r = IDLE_THROTTLE - min;

		if((max + r) > THROTTLE_RESOLUTION)
		{
			r += (THROTTLE_RESOLUTION - (max + r)) / 2;
		}
	}

	m1 += r;
	m2 += r;
	m3 += r;
	m4 += r;

	FC_GlobalThrust.Motor1 = _clamp(m1, THROTTLE_RESOLUTION, IDLE_THROTTLE);
	FC_GlobalThrust.Motor2 = _clamp(m2, THROTTLE_RESOLUTION, IDLE_THROTTLE);
	FC_GlobalThrust.Motor3 = _clamp(m3, THROTTLE_RESOLUTION, IDLE_THROTTLE);
	FC_GlobalThrust.Motor4 = _clamp(m4, THROTTLE_RESOLUTION, IDLE_THROTTLE);
}

void FC_ManualDisarm()
{
	_manualDisarm = true;

	FC_GlobalThrust.Motor1 = 0;
	FC_GlobalThrust.Motor2 = 0;
	FC_GlobalThrust.Motor3 = 0;
	FC_GlobalThrust.Motor4 = 0;
}

void FC_ManualArm()
{
	_manualDisarm = false;
}
