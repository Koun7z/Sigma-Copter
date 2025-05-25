//
// Created by pwoli on 18.03.2025.
//
#include "FC_NC_Filter.h"

#include <math.h>

uint8_t FC_NC_Init_f32(FC_NC_Instance_f32* instance,
                       const float Gain,
                       const float SLERP_Threshold,
                       const float Threshold1,
                       const float Threshold2)
{
	instance->Gain            = Gain;
	instance->SLERP_Threshold = SLERP_Threshold;
	instance->Threshold1      = Threshold1;
	instance->Threshold2      = Threshold2;

	return 0;
}

void FC_NC_FilterUpdate_f32(const FC_NC_Instance_f32* filter, FC_IMU_Data_Instance* imuData, const float dt)
{
	const float p = imuData->GyroX;
	const float q = imuData->GyroY;
	const float r = imuData->GyroZ;


	float a_i = imuData->AccelX;
	float a_j = imuData->AccelY;
	float a_k = imuData->AccelZ;

	// Integrating local angular velocity
	const DSP_Quaternion_f32 omega_l = {.r = 0.0f, .i = p, .j = q, .k = r};

	DSP_Quaternion_f32 delta_q_gyro;

	DSP_QT_Multiply_f32(&delta_q_gyro, &omega_l, &imuData->Attitude);
	DSP_QT_Scale_f32(&delta_q_gyro, &delta_q_gyro, -0.5f * dt);

	DSP_Quaternion_f32 q_gyro;

	DSP_QT_Add_f32(&q_gyro, &imuData->Attitude, &delta_q_gyro);
	DSP_QT_Normalize_f32(&q_gyro, &q_gyro);


	// Normalizing local acceleration vector
	const float acc_norm = sqrtf(a_i * a_i + a_j * a_j + a_k * a_k);
	a_i                  = a_i / acc_norm;
	a_j                  = a_j / acc_norm;
	a_k                  = a_k / acc_norm;

	// Calculating predicted gravity vector
	float G_pred[3] = {a_i, a_j, a_k};
	DSP_Quaternion_f32 q_gyro_LG;
	DSP_QT_Conjugate_f32(&q_gyro_LG, &q_gyro);

	DSP_QT_RotateVector_f32(G_pred, G_pred, &q_gyro_LG);


	// Calculating accelerometer correction
	DSP_Quaternion_f32 delta_q_acc;
	if (G_pred[2] >= 0.0f)
	{
		const float sq_gk = sqrtf(2.0f * (G_pred[2] + 1.0f));

		delta_q_acc.r = sqrtf((G_pred[2] + 1.0f) / 2.0f);
		delta_q_acc.i = -G_pred[1] / sq_gk;
		delta_q_acc.j = G_pred[0] / sq_gk;
		delta_q_acc.k = 0.0f;
	}
	else
	{
		const float sq_gk = sqrtf(2.0f * (1.0f - G_pred[2]));

		delta_q_acc.r = -G_pred[1] / sq_gk;
		delta_q_acc.i = sqrtf((1.0f - G_pred[2]) / 2.0f);
		delta_q_acc.j = 0.0f;
		delta_q_acc.k = G_pred[0] / sq_gk;
	}


	// Simple adaptive gain
	float gain          = filter->Gain;
	const float acc_err = fabsf(acc_norm - 1.0f);

	if(acc_err > filter->Threshold2)
	{
		gain = 0.0f;
	}
	else if(acc_err > filter->Threshold1)
	{
		gain *= (filter->Threshold2 - acc_err) / filter->Threshold1;
	}

	// Applying gain to accelerometer correction
	if(delta_q_acc.r > filter->SLERP_Threshold)
	{
		DSP_QT_Scale_f32(&delta_q_acc, &delta_q_acc, gain);
		delta_q_acc.r += 1.0f - gain;
	}
	else
	{
		const float omega_slerp = acosf(delta_q_acc.r);
		const float sin_om      = sinf(omega_slerp);

		const float scale = sinf(gain * omega_slerp) / sin_om;
		DSP_QT_Scale_f32(&delta_q_acc, &delta_q_acc, scale);

		delta_q_acc.r += sinf((1.0f - gain) * omega_slerp) / sin_om;
	}

	// Applying correction
	DSP_QT_Multiply_f32(&imuData->Attitude, &q_gyro, &delta_q_acc);
	DSP_QT_Normalize_f32(&imuData->Attitude, &imuData->Attitude);
}
