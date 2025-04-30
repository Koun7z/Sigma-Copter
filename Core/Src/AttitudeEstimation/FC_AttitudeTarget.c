//
// Created by pwoli on 30.04.2025.
//


#include "FC_AttitudeTarget.h"

#include <math.h>

void FC_IntegrateTargetAttitude(FC_IMU_Data_Instance* imuData, const FC_RC_Data_Instance* rcData, const float dt)
{
	const float p = rcData->Roll;
	const float q = rcData->Pitch;
	const float r = rcData->Yaw;

	// Integrating target angular velocity
	DSP_Quaternion_f32 q_omega = {.r = 0.0f, .i = p, .j = q, .k = r};

	DSP_QT_Scale_f32(&q_omega, &q_omega, -0.5f * dt);
	DSP_QT_Multiply_f32(&q_omega, &q_omega, &imuData->AttitudeTarget);


	DSP_QT_Add_f32(&imuData->AttitudeTarget, &imuData->AttitudeTarget, &q_omega);
	DSP_QT_Normalize_f32(&imuData->AttitudeTarget, &imuData->AttitudeTarget);
}

void FC_SetTargetAttitude(FC_IMU_Data_Instance* imuData, const FC_RC_Data_Instance* rcData, const float dt)
{

}

void FC_GetTargetVelocity(const FC_IMU_Data_Instance* imuData, float* targetVelocity)
{
	const DSP_Quaternion_f32* q_t = &imuData->AttitudeTarget;
	const DSP_Quaternion_f32* q_c = &imuData->Attitude;

	targetVelocity[0] = 2.0f * (q_c->r * q_t->i - q_c->i * q_t->r - q_c->j * q_t->k - q_c->k * q_t->j);
	targetVelocity[1] = 2.0f * (q_c->r * q_t->j - q_c->i * q_t->k - q_c->j * q_t->r - q_c->k * q_t->i);
	targetVelocity[2] = 2.0f * (q_c->r * q_t->k - q_c->i * q_t->j - q_c->j * q_t->i - q_c->k * q_t->r);
}