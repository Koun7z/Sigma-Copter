/*
 * FlightController.c
 *
 *  Created on: Jan 28, 2025
 *      Author: pwoli
 */

#include "FlightController.h"

#include "ProcessControl.h"
#include "SignalFiltering.h"

#include <string.h>

PID_Instance_f32 FC_PitchRatePID;
PID_Instance_f32 FC_RollRatePID;
PID_Instance_f32 FC_YawRatePID;

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
DSP_FIR_RT_Instance_f32 imuGyroXFilter;
DSP_FIR_RT_Instance_f32 imuGyroYFilter;
DSP_FIR_RT_Instance_f32 imuGyroZFilter;

#  endif

#  if FC_IMU_GYRO_FILTER_TYPE == 2
DSP_IIR_RT_Instance_f32 imuGyroXFilter;
DSP_IIR_RT_Instance_f32 imuGyroYFilter;
DSP_IIR_RT_Instance_f32 imuGyroZFilter;

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

	PID_Init_f32(&FC_PitchRatePID, 3, 0, 0);

	FC_PitchRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	FC_PitchRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);


	PID_Init_f32(&FC_RollRatePID, 3, 0, 0);

	FC_RollRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	FC_RollRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);


	PID_Init_f32(&FC_YawRatePID, 3, 0, 0);

	FC_YawRatePID.MaxWindup = FC_THROTTLE_RESOLUTION / 4;
	FC_YawRatePID.MinWindup = -(FC_THROTTLE_RESOLUTION / 4);

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


#if FC_IMU_GYRO_FILTER_ENABLE
	for(size_t i = 0; i < FC_RC_AXIS_FILTER_ORDER; i++)
	{
		imuGyroFilterCoeffsB[i] = 0;
	}
	imuGyroFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER] = 1;

#  if FC_IMU_GYRO_FILTER_TYPE == 1
	DSP_FIR_RT_Init_f32(&imuGyroXFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroXInputBuff);
	DSP_FIR_RT_Init_f32(&imuGyroYFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroYInputBuff);
	DSP_FIR_RT_Init_f32(&imuGyroZFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroZInputBuff);
#  endif

#  if FC_IMU_GYRO_FILTER_TYPE == 2
	DSP_IIR_RT_Init_f32(&imuGyroXFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroFilterCoeffsA,
	                    imuGyroXInputBuff, imuGyroXOutputBuff);
	DSP_IIR_RT_Init_f32(&imuGyroYFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroFilterCoeffsA,
	                    imuGyroYInputBuff, imuGyroYOutputBuff);
	DSP_IIR_RT_Init_f32(&imuGyroZFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroFilterCoeffsA,
	                    imuGyroZInputBuff, imuGyroZOutputBuff);
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


	//Adjust for min/max throttle values to maintain motor offset
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
	FC_RC_Data.Aux1 = (float)(aux1 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux2 = (float)(aux2 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux3 = (float)(aux3 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux4 = (float)(aux4 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);

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
void FC_RC_SetFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator)
{
#if FC_RC_AXIS_FILTER_ENABLE
#  if FC_RC_AXIS_FILTER_TYPE == 1

	memcpy(rcAxisFilterCoeffsB, filterCoeffsNumerator, (FC_RC_AXIS_FILTER_ORDER + 1) * sizeof(float));
	DSP_ReverseArray_f32(rcAxisFilterCoeffsB, FC_RC_AXIS_FILTER_ORDER + 1);

#  endif
#  if FC_RC_AXIS_FILTER_TYPE == 2

	memcpy(rcAxisFilterCoeffsB, filterCoeffsNumerator, (FC_RC_AXIS_FILTER_ORDER + 1) * sizeof(float));
	memcpy(rcAxisFilterCoeffsA, filterCoeffsDenominator, FC_RC_AXIS_FILTER_ORDER * sizeof(float));

	DSP_ReverseArray_f32(rcAxisFilterCoeffsB, FC_RC_AXIS_FILTER_ORDER + 1);
	DSP_ReverseArray_f32(rcAxisFilterCoeffsA, FC_RC_AXIS_FILTER_ORDER);
#  endif
#endif
}


void FC_IMU_UpdateGyro(float pitchRate, float rollRate, float yawRate)
{
#if FC_IMU_GYRO_FILTER_ENABLE
#  if FC_IMU_GYRO_FILTER_TYPE == 1

	FC_IMU_Data.GyroX = DSP_FIR_RT_Update_f32(&imuGyroXFilter, rollRate);
	FC_IMU_Data.GyroY = DSP_FIR_RT_Update_f32(&imuGyroYFilter, pitchRate);
	FC_IMU_Data.GyroZ = DSP_FIR_RT_Update_f32(&imuGyroYFilter, yawRate);

#  endif
#  if FC_IMU_GYRO_FILTER_TYPE == 2

	FC_IMU_Data.GyroX = DSP_IIR_RT_Update_f32(&imuGyroXFilter, rollRate);
	FC_IMU_Data.GyroY = DSP_IIR_RT_Update_f32(&imuGyroYFilter, pitchRate);
	FC_IMU_Data.GyroZ = DSP_IIR_RT_Update_f32(&imuGyroZFilter, yawRate);

#  endif
#else

	FC_IMU_Data.GyroX = rollRate;
	FC_IMU_Data.GyroY = pitchRate;
	FC_IMU_Data.GyroZ = yawRate;

#endif
}
void FC_IMU_SetGyroFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator)
{
#if FC_IMU_GYRO_FILTER_ENABLE
#  if FC_IMU_GYRO_FILTER_TYPE == 1
	memcpy(imuGyroFilterCoeffsB, filterCoeffsNumerator, (FC_RC_AXIS_FILTER_ORDER + 1) * sizeof(float));
	DSP_ReverseArray_f32(imuGyroFilterCoeffsB, FC_RC_AXIS_FILTER_ORDER + 1);
#  endif
#  if FC_IMU_GYRO_FILTER_TYPE == 2
	memcpy(imuGyroFilterCoeffsB, filterCoeffsNumerator, (FC_RC_AXIS_FILTER_ORDER + 1) * sizeof(float));
	memcpy(imuGyroFilterCoeffsA, filterCoeffsDenominator, FC_RC_AXIS_FILTER_ORDER * sizeof(float));

	DSP_ReverseArray_f32(imuGyroFilterCoeffsB, FC_RC_AXIS_FILTER_ORDER + 1);
	DSP_ReverseArray_f32(imuGyroFilterCoeffsA, FC_RC_AXIS_FILTER_ORDER);
#  endif
#endif
}
