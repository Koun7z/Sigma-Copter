//
// Created by pwoli on 18.03.2025.
//
#include "FC_NC_Filter.h"

#include "DSP_Constants.h"

#include <math.h>

uint8_t FC_NC_Init_f32(FC_NC_Instance_f32* instance,
                       const float Gain,
                       const float Threshold1,
                       const float Threshold2)
{

	instance->Gain = Gain;
	instance->Threshold1 = Threshold1;
	instance->Threshold2 = Threshold2;

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

	const float acc_norm = sqrtf(a_i*a_i + a_j*a_j + a_k*a_k);
	a_i = a_i / acc_norm;
	a_j = a_j / acc_norm;
	a_k = a_k / acc_norm;

	DSP_Quaternion_f32 q_acc;
	if(a_k >= 0)
	{
		const float sq_ak = sqrtf(2.0f * (a_k + 1.0f));

		q_acc.r = sqrtf((a_k + 1.0f) / 2.0f);
		q_acc.i = -a_j / sq_ak;
		q_acc.j = a_i / sq_ak;
		q_acc.k = 0.0f;
	}
	else
	{
		const float sq_ak = sqrtf(2.0f * (1.0f - a_k));

		q_acc.r = -a_j / sq_ak;
		q_acc.i = sqrtf((1.0f - a_k) / 2.0f);
		q_acc.j = 0.0f;
		q_acc.k = a_i / sq_ak;
	}

	const DSP_Quaternion_f32 omega_l = {.r = 0, .i = p, .j = q, .k = r};

	DSP_Quaternion_f32 delta_q_gyro;

	DSP_QT_Multiply_f32(&delta_q_gyro, &omega_l, &imuData->Attitude);
	DSP_QT_Scale_f32(&delta_q_gyro, &delta_q_gyro, -0.5f * dt);

	DSP_Quaternion_f32 q_gyro;

	DSP_QT_Add_f32(&q_gyro, &imuData->Attitude, &delta_q_gyro);
	DSP_QT_Normalize_f32(&q_gyro, &q_gyro);

    float G_pred[3] = {a_i, a_j, a_k};
	DSP_Quaternion_f32 q_gyro_LG;
	DSP_QT_Conjugate_f32(&q_gyro_LG, &q_gyro);

	DSP_QT_RotateVector_f32(G_pred, G_pred, &q_gyro_LG);

	DSP_Quaternion_f32 delta_q_acc;
	const float sq_gz = sqrtf(2.0f * (G_pred[2] + 1.0f));
	delta_q_acc.r = sqrtf((G_pred[2] + 1.0f) / 2.0f);
	delta_q_acc.i = -G_pred[1] / sq_gz;
	delta_q_acc.j = G_pred[0] / sq_gz;
	delta_q_acc.k = 0.0f;

	DSP_QT_Scale_f32(&delta_q_acc, &delta_q_gyro, filter->Gain);
	delta_q_acc.r += (1 - filter->Gain);

	DSP_QT_Normalize_f32(&delta_q_acc, &delta_q_acc);

	DSP_QT_Multiply_f32(&imuData->Attitude, &q_gyro, &delta_q_acc);
}
