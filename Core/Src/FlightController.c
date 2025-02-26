/*
 * FlightController.c
 *
 *  Created on: Jan 28, 2025
 *      Author: pwoli
 */

#include "FlightController.h"

#include "ProcessControl.h"
#include "SignalFiltering.h"


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

PID_Instance_f32 PitchRatePID;
PID_Instance_f32 RollRatePID;
PID_Instance_f32 YawRatePID;

FC_MotorThrust FC_GlobalThrust;

FC_IMU_Data_Instance FC_IMU_Data;
FC_RC_Data_Instance FC_RC_Data;

bool manualDisarm = false;


// RC data filters
#if FC_RC_AXIS_FILTER_ENABLE
#  if FC_RC_AXIS_FILTER_TYPE == 1
DSP_FIR_RT_Instance_f32 rcAxisPitchFilter;
DSP_FIR_RT_Instance_f32 rcAxisRollFilter;
DSP_FIR_RT_Instance_f32 rcAxisYawFilter;
DSP_FIR_RT_Instance_f32 rcAxisThrottleFilter;
#  endif

#  if FC_RC_AXIS_FILTER_TYPE == 2
DSP_IIR_RT_Instance_f32 rcAxisPitchFilter;
DSP_IIR_RT_Instance_f32 rcAxisRollFilter;
DSP_IIR_RT_Instance_f32 rcAxisYawFilter;
DSP_IIR_RT_Instance_f32 rcAxisThrottleFilter;

float rcAxisFilterPitchOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterRollOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterYawOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterThrottleOutputBuff[FC_RC_AXIS_FILTER_ORDER];

float rcAxisFilterCoeffsA[FC_RC_AXIS_FILTER_ORDER];
#  endif
float rcAxisFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER + 1];

float rcAxisFilterPitchInputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterRollInputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterYawInputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterThrottleInputBuff[FC_RC_AXIS_FILTER_ORDER];
#endif

// IMU data filters
#if FC_IMU_GYRO_FILTER_ENABLE
#  if FC_IMU_GYRO_FILTER_TYPE == 1
DSP_FIR_RT_Instance_f32 rcAxisPitchFilter;
DSP_FIR_RT_Instance_f32 rcAxisRollFilter;
DSP_FIR_RT_Instance_f32 rcAxisYawFilter;
DSP_FIR_RT_Instance_f32 rcAxisThrottleFilter;
#  endif

#  if FC_IMU_GYRO_FILTER_TYPE == 2
DSP_IIR_RT_Instance_f32 imuGyroXFilter;
DSP_IIR_RT_Instance_f32 imuGyroYFilter;
DSP_IIR_RT_Instance_f32 imuDataZFilter;

float imuGyroXOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroYOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroZOutputBuff[FC_RC_AXIS_FILTER_ORDER];

float imuGyroFilterCoeffsA[FC_RC_AXIS_FILTER_ORDER];
#  endif
float imuGyroFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER + 1];

float imuGyroXInputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroYInputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroZInputBuff[FC_RC_AXIS_FILTER_ORDER];
#endif

static uint32_t clamp(uint32_t num, uint32_t min, uint32_t max)
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

	PID_Init_f32(&PitchRatePID, 3, 0, 0);

	PitchRatePID.MaxOutput = FC_THROTTLE_RESOLUTION / 4;
	PitchRatePID.MinOutput = -(FC_THROTTLE_RESOLUTION / 4);

	PitchRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	PitchRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);


	PID_Init_f32(&RollRatePID, 3, 0, 0);

	RollRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	RollRatePID.MinOutput = -(FC_THROTTLE_RESOLUTION / 4);

	RollRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	RollRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);


	PID_Init_f32(&YawRatePID, 3, 0, 0);

	YawRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	YawRatePID.MinOutput = -(FC_THROTTLE_RESOLUTION / 4);

	YawRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	YawRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);

#if FC_RC_AXIS_FILTER_ENABLE
	for(size_t i = 0; i < FC_RC_AXIS_FILTER_ORDER; i++)
	{
		rcAxisFilterCoeffsB[i] = 0;
	}
	rcAxisFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER] = 1;

#  if FC_RC_AXIS_FILTER_TYPE == 1
	DSP_FIR_RT_Init_f32(&rcAxisPitchFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterPitchInputBuff);
	DSP_FIR_RT_Init_f32(&rcAxisRollFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterRollInputBuff);
	DSP_FIR_RT_Init_f32(&rcAxisYawFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterYawInputBuff);
	DSP_FIR_RT_Init_f32(&rcAxisThrottleFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB,
	                    rcAxisFilterThrottleInputBuff);
#  endif
#  if FC_RC_AXIS_FILTER_TYPE == 2
	DSP_IIR_RT_Init_f32(&rcAxisPitchFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterPitchInputBuff, rcAxisFilterPitchOutputBuff);
	DSP_IIR_RT_Init_f32(&rcAxisRollFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterRollInputBuff, rcAxisFilterRollOutputBuff);
	DSP_IIR_RT_Init_f32(&rcAxisYawFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterYawInputBuff, rcAxisFilterYawOutputBuff);
	DSP_IIR_RT_Init_f32(&rcAxisThrottleFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterThrottleInputBuff, rcAxisFilterThrottleOutputBuff);
#  endif
#endif

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


	const float pitchRate = FC_IMU_Data.GyroX;
	const float rollRate  = FC_IMU_Data.GyroY;
	const float yawRate   = FC_IMU_Data.GyroZ;

	float pitchOffset = PID_Update_f32(&PitchRatePID, pitch - pitchRate, dt) * 0.5f;
	float rollOffset  = PID_Update_f32(&RollRatePID, roll - rollRate, dt) * 0.5f;
	float yawOffset   = PID_Update_f32(&YawRatePID, yaw - yawRate, dt) * 0.5f;

	uint32_t m1 = (uint32_t)(throttle - pitchOffset - rollOffset - yawOffset);
	uint32_t m2 = (uint32_t)(throttle - pitchOffset + rollOffset + yawOffset);
	uint32_t m3 = (uint32_t)(throttle + pitchOffset - rollOffset + yawOffset);
	uint32_t m4 = (uint32_t)(throttle + pitchOffset + rollOffset - yawOffset);

	// Find max throttle
	uint32_t max = m1 > m2 ? m1 : m2;
	max          = max > m3 ? max : m3;
	max          = max > m3 ? max : m4;

	// Find min throttle
	uint32_t min = m1 < m2 ? m1 : m2;
	min          = min < m3 ? min : m3;
	min          = min < m4 ? min : m4;

	uint32_t r = 0;
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

void FC_RC_UpdateAxisChannels(int throttle, int pitch, int roll, int yaw)
{
	FC_RC_Data.Pitch = (float)(pitch - FC_CHANNEL_MIDPOINT) * FC_PITCH_LINEAR_RATE / FC_CHANNEL_MIDPOINT;  // deg/s
	FC_RC_Data.Roll  = (float)(roll - FC_CHANNEL_MIDPOINT) * FC_ROLL_LINEAR_RATE / FC_CHANNEL_MIDPOINT;    // deg/s
	FC_RC_Data.Yaw   = (float)(yaw - FC_CHANNEL_MIDPOINT) * FC_YAW_LINEAR_RATE / FC_CHANNEL_MIDPOINT;      // deg/s
	FC_RC_Data.Throttle =
	  (float)(throttle - FC_CHANNEL_MIN) * FC_THROTTLE_RESOLUTION / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);

#if FC_RC_AXIS_FILTER_ENABLE
#  if FC_RC_AXIS_FILTER_TYPE == 1
	FC_RC_Data.Pitch    = DSP_FIR_RT_Update_f32(&rcAxisPitchFilter, FC_RC_Data.Pitch);
	FC_RC_Data.Roll     = DSP_FIR_RT_Update_f32(&rcAxisRollFilter, FC_RC_Data.Roll);
	FC_RC_Data.Yaw      = DSP_FIR_RT_Update_f32(&rcAxisYawFilter, FC_RC_Data.Yaw);
	FC_RC_Data.Throttle = DSP_FIR_RT_Update_f32(&rcAxisThrottleFilter, FC_RC_Data.Throttle);
#  endif
#  if FC_RC_AXIS_FILTER_TYPE == 2
	FC_RC_Data.Pitch    = DSP_IIR_RT_Update_f32(&rcAxisPitchFilter, FC_RC_Data.Pitch);
	FC_RC_Data.Roll     = DSP_IIR_RT_Update_f32(&rcAxisRollFilter, FC_RC_Data.Roll);
	FC_RC_Data.Yaw      = DSP_IIR_RT_Update_f32(&rcAxisYawFilter, FC_RC_Data.Yaw);
	FC_RC_Data.Throttle = DSP_IIR_RT_Update_f32(&rcAxisThrottleFilter, FC_RC_Data.Throttle);
#  endif
#endif
}

void FC_RC_UpdateAuxChannels(int aux1, int aux2, int aux3, int aux4)
{
	FC_RC_Data.Aux1 = (float)((aux1 - FC_CHANNEL_MIN) * 100) / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux2 = (float)((aux2 - FC_CHANNEL_MIN) * 100) / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux3 = (float)((aux3 - FC_CHANNEL_MIN) * 100) / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux4 = (float)((aux4 - FC_CHANNEL_MIN) * 100) / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);

#if !FC_EXTERNAL_ARM_STATUS
	FC_RC_Data.Arm = (bool)(FC_RC_Data.Aux1 > 90);
#endif
}

void FC_RC_UpdateArmStatus(bool armStatus)
{
#if FC_EXTERNAL_ARM_STATUS
	FC_RC_Data.Arm = armStatus;
#endif
}


void FC_UpdateGyro(float pitchRate, float rollRate, float yawRate) {}
