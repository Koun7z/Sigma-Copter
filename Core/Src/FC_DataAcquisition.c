//
// Created by pwoli on 14.03.2025.
//

#include "FlightController.h"
//
#include "FC_Config.h"
#include "SignalFiltering.h"

#include <string.h>

/*
** RC data filters
*/

DSP_IIR_RT_Instance_f32 rcAxisPitchFilter;
DSP_IIR_RT_Instance_f32 rcAxisRollFilter;
DSP_IIR_RT_Instance_f32 rcAxisYawFilter;
DSP_IIR_RT_Instance_f32 rcAxisThrottleFilter;

float rcAxisFilterCoeffsA[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER + 1];

float rcAxisFilterPitchInputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterRollInputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterYawInputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterThrottleInputBuff[FC_RC_AXIS_FILTER_ORDER];

float rcAxisFilterPitchOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterRollOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterYawOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float rcAxisFilterThrottleOutputBuff[FC_RC_AXIS_FILTER_ORDER];


/*
** IMU data filters
*/

// Gyroscope filter instances
DSP_IIR_RT_Instance_f32 imuGyroXFilter;
DSP_IIR_RT_Instance_f32 imuGyroYFilter;
DSP_IIR_RT_Instance_f32 imuGyroZFilter;

float imuGyroFilterCoeffsA[FC_RC_AXIS_FILTER_ORDER];
float imuGyroFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER + 1];

float imuGyroXInputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroYInputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroZInputBuff[FC_RC_AXIS_FILTER_ORDER];

float imuGyroXOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroYOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuGyroZOutputBuff[FC_RC_AXIS_FILTER_ORDER];

// Accelerometer filter instances
DSP_IIR_RT_Instance_f32 imuAccelXFilter;
DSP_IIR_RT_Instance_f32 imuAccelYFilter;
DSP_IIR_RT_Instance_f32 imuAccelZFilter;

float imuAccelXInputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuAccelYInputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuAccelZInputBuff[FC_RC_AXIS_FILTER_ORDER];

float imuAccelXOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuAccelYOutputBuff[FC_RC_AXIS_FILTER_ORDER];
float imuAccelZOutputBuff[FC_RC_AXIS_FILTER_ORDER];

float imuAccelFilterCoeffsA[FC_RC_AXIS_FILTER_ORDER];
float imuAccelFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER + 1];

void FC_DataAcquisitionInit()
{
#if FC_RC_AXIS_FILTER_ENABLE
	for(size_t i = 0; i < FC_RC_AXIS_FILTER_ORDER; i++)
	{
		rcAxisFilterCoeffsB[i] = 0;
	}
	rcAxisFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER] = 1;

	DSP_IIR_RT_Init_f32(&rcAxisPitchFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterPitchInputBuff, rcAxisFilterPitchOutputBuff);
	DSP_IIR_RT_Init_f32(&rcAxisRollFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterRollInputBuff, rcAxisFilterRollOutputBuff);
	DSP_IIR_RT_Init_f32(&rcAxisYawFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterYawInputBuff, rcAxisFilterYawOutputBuff);
	DSP_IIR_RT_Init_f32(&rcAxisThrottleFilter, FC_RC_AXIS_FILTER_ORDER, rcAxisFilterCoeffsB, rcAxisFilterCoeffsA,
	                    rcAxisFilterThrottleInputBuff, rcAxisFilterThrottleOutputBuff);
#endif

#if FC_IMU_GYRO_FILTER_ENABLE
	for(size_t i = 0; i < FC_RC_AXIS_FILTER_ORDER; i++)
	{
		imuGyroFilterCoeffsB[i] = 0;
	}
	imuGyroFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER] = 1;

	DSP_IIR_RT_Init_f32(&imuGyroXFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroFilterCoeffsA,
	                    imuGyroXInputBuff, imuGyroXOutputBuff);
	DSP_IIR_RT_Init_f32(&imuGyroYFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroFilterCoeffsA,
	                    imuGyroYInputBuff, imuGyroYOutputBuff);
	DSP_IIR_RT_Init_f32(&imuGyroZFilter, FC_IMU_GYRO_FILTER_ORDER, imuGyroFilterCoeffsB, imuGyroFilterCoeffsA,
	                    imuGyroZInputBuff, imuGyroZOutputBuff);
#endif

#if FC_IMU_ACCEL_FILTER_ENABLE
	for(size_t i = 0; i < FC_RC_AXIS_FILTER_ORDER; i++)
	{
		imuAccelFilterCoeffsB[i] = 0;
	}
	imuAccelFilterCoeffsB[FC_RC_AXIS_FILTER_ORDER] = 1;

	DSP_IIR_RT_Init_f32(&imuAccelXFilter, FC_IMU_ACCEL_FILTER_ORDER, imuAccelFilterCoeffsB, imuAccelFilterCoeffsA,
	                    imuAccelXInputBuff, imuAccelXOutputBuff);
	DSP_IIR_RT_Init_f32(&imuAccelYFilter, FC_IMU_ACCEL_FILTER_ORDER, imuAccelFilterCoeffsB, imuAccelFilterCoeffsA,
	                    imuAccelYInputBuff, imuAccelYOutputBuff);
	DSP_IIR_RT_Init_f32(&imuAccelZFilter, FC_IMU_ACCEL_FILTER_ORDER, imuAccelFilterCoeffsB, imuAccelFilterCoeffsA,
	                    imuAccelZInputBuff, imuAccelZOutputBuff);
#endif

	FC_RC_Data.Arm  = false;
	FC_RC_Data.Aux1 = 0.0f;
	FC_RC_Data.Aux2 = 0.0f;
	FC_RC_Data.Aux3 = 0.0f;
	FC_RC_Data.Aux4 = 0.0f;
	FC_RC_UpdateAxisChannels(0, FC_CHANNEL_MIDPOINT, FC_CHANNEL_MIDPOINT, FC_CHANNEL_MIDPOINT);

	FC_IMU_Data.AccelX   = 0.0f;
	FC_IMU_Data.AccelY   = 0.0f;
	FC_IMU_Data.AccelZ   = 0.0f;
	FC_IMU_Data.GyroX    = 0.0f;
	FC_IMU_Data.GyroY    = 0.0f;
	FC_IMU_Data.GyroZ    = 0.0f;
	FC_IMU_Data.Attitude = (DSP_Quaternion_f32) {1, 0, 0, 0};
}


void FC_RC_UpdateAxisChannels(int throttle, int pitch, int roll, int yaw)
{
	FC_RC_Data.Pitch = (float)(pitch - FC_CHANNEL_MIDPOINT) * FC_PITCH_LINEAR_RATE / FC_CHANNEL_MIDPOINT;  // deg/s
	FC_RC_Data.Roll  = (float)(roll - FC_CHANNEL_MIDPOINT) * FC_ROLL_LINEAR_RATE / FC_CHANNEL_MIDPOINT;    // deg/s
	FC_RC_Data.Yaw   = (float)(yaw - FC_CHANNEL_MIDPOINT) * FC_YAW_LINEAR_RATE / FC_CHANNEL_MIDPOINT;      // deg/s
	FC_RC_Data.Throttle =
	  (float)(throttle - FC_CHANNEL_MIN) * FC_THROTTLE_RESOLUTION / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);

#if FC_RC_AXIS_FILTER_ENABLE
	FC_RC_Data.Pitch    = DSP_IIR_RT_Update_f32(&rcAxisPitchFilter, FC_RC_Data.Pitch);
	FC_RC_Data.Roll     = DSP_IIR_RT_Update_f32(&rcAxisRollFilter, FC_RC_Data.Roll);
	FC_RC_Data.Yaw      = DSP_IIR_RT_Update_f32(&rcAxisYawFilter, FC_RC_Data.Yaw);
	FC_RC_Data.Throttle = DSP_IIR_RT_Update_f32(&rcAxisThrottleFilter, FC_RC_Data.Throttle);
#endif
}

void FC_RC_UpdateAuxChannels(const int aux1, const int aux2, const int aux3, const int aux4)
{
	FC_RC_Data.Aux1 = (float)(aux1 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux2 = (float)(aux2 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux3 = (float)(aux3 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);
	FC_RC_Data.Aux4 = (float)(aux4 - FC_CHANNEL_MIN) * 100 / (FC_CHANNEL_MAX - FC_CHANNEL_MIN);

#if !FC_EXTERNAL_ARM_STATUS
	FC_RC_Data.Arm = (bool)(FC_RC_Data.Aux1 > 90);
#endif
}

void FC_RC_UpdateArmStatus(const bool armStatus)
{
#if FC_EXTERNAL_ARM_STATUS
	FC_RC_Data.Arm = armStatus;
#endif
}

void FC_RC_SetFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator)
{
#if FC_RC_AXIS_FILTER_ENABLE
	memcpy(rcAxisFilterCoeffsB, filterCoeffsNumerator, (FC_RC_AXIS_FILTER_ORDER + 1) * sizeof(float));
	memcpy(rcAxisFilterCoeffsA, filterCoeffsDenominator, FC_RC_AXIS_FILTER_ORDER * sizeof(float));

	DSP_ReverseArray_f32(rcAxisFilterCoeffsB, FC_RC_AXIS_FILTER_ORDER + 1);
	DSP_ReverseArray_f32(rcAxisFilterCoeffsA, FC_RC_AXIS_FILTER_ORDER);
#endif
}


void FC_IMU_UpdateGyro(const float pitchRate, const float rollRate, const float yawRate, const float dt)
{
#if FC_IMU_GYRO_FILTER_ENABLE
	FC_IMU_Data.GyroX = DSP_IIR_RT_Update_f32(&imuGyroXFilter, rollRate);
	FC_IMU_Data.GyroY = DSP_IIR_RT_Update_f32(&imuGyroYFilter, pitchRate);
	FC_IMU_Data.GyroZ = DSP_IIR_RT_Update_f32(&imuGyroZFilter, yawRate);
#else
	FC_IMU_Data.GyroX = rollRate;
	FC_IMU_Data.GyroY = pitchRate;
	FC_IMU_Data.GyroZ = yawRate;
#endif
}

void FC_IMU_SetGyroFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator)
{
#if FC_IMU_GYRO_FILTER_ENABLE
	memcpy(imuGyroFilterCoeffsB, filterCoeffsNumerator, (FC_RC_AXIS_FILTER_ORDER + 1) * sizeof(float));
	memcpy(imuGyroFilterCoeffsA, filterCoeffsDenominator, FC_RC_AXIS_FILTER_ORDER * sizeof(float));

	DSP_ReverseArray_f32(imuGyroFilterCoeffsB, FC_RC_AXIS_FILTER_ORDER + 1);
	DSP_ReverseArray_f32(imuGyroFilterCoeffsA, FC_RC_AXIS_FILTER_ORDER);
#endif
}

void FC_IMU_UpdateAccel(const float accelX, const float accelY, const float accelZ, const float dt)
{
#if FC_IMU_ACCEL_FILTER_ENABLE
	FC_IMU_Data.AccelX = DSP_IIR_RT_Update_f32(&imuAccelXFilter, accelX);
	FC_IMU_Data.AccelY = DSP_IIR_RT_Update_f32(&imuAccelYFilter, accelY);
	FC_IMU_Data.AccelZ = DSP_IIR_RT_Update_f32(&imuAccelZFilter, accelZ);
#else
	FC_IMU_Data.AccelX = accelX;
	FC_IMU_Data.AccelY = accelY;
	FC_IMU_Data.AccelZ = accelZ;
#endif
}

void FC_IMU_SetAccelFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator)
{
#if FC_IMU_ACCEL_FILTER_ENABLE
	memcpy(imuAccelFilterCoeffsB, filterCoeffsNumerator, (FC_RC_AXIS_FILTER_ORDER + 1) * sizeof(float));
	memcpy(imuAccelFilterCoeffsA, filterCoeffsDenominator, FC_RC_AXIS_FILTER_ORDER * sizeof(float));

	DSP_ReverseArray_f32(imuAccelFilterCoeffsB, FC_RC_AXIS_FILTER_ORDER + 1);
	DSP_ReverseArray_f32(imuAccelFilterCoeffsA, FC_RC_AXIS_FILTER_ORDER);
#endif
}
