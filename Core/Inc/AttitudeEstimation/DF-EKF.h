//
// Created by pwoli on 08.03.2025.
//

#ifndef DF_EKF_H
#define DF_EKF_H

#include "FC_CommonTypes.h"

typedef struct
{
	float P[4][4];
	float R_g[3];
	float R_a[2];
	float Q[4];

	float mu;
	float m;
} DF_EKF_Instance_f32;

void DF_EKF_Init_f32(DF_EKF_Instance_f32* ekf, float Q[4], float R_g[3], float R_a[2], float mu, float m);

void DF_EKF_UpdatePrediction_f32(DF_EKF_Instance_f32* ekf, FC_IMU_Data_Instance* imuData, float dt);

void DF_EKF_UpdateCorrection_f32(DF_EKF_Instance_f32* ekf, FC_IMU_Data_Instance* imuData);

#endif  // DF_EKF_H
