//
// Created by pwoli on 08.03.2025.
//

#include "DF-EKF.h"

#include "DSP_Constants.h"
#include "math.h"

const float g = 9.80665f;

void DF_EKF_UpdatePrediction_f32(DF_EKF_Instance_f32* ekf, FC_IMU_Data_Instance* imuData, float dt)
{
	const float p = imuData->GyroX;
	const float q = imuData->GyroY;
	const float r = imuData->GyroZ;

	const float u = imuData->I_Velocity;
	const float v = imuData->J_Velocity;

	float phi   = DEG_TO_RAD_F32 * imuData->RollAngle;
	float theta = DEG_TO_RAD_F32 * imuData->PitchAngle;

	const float mu_m = ekf->mu / ekf->m;

	/*
	** Calculate common trigonometric terms
	*/
	float cos_phi = cosf(phi);
	float sin_phi = sinf(phi);

	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);
	float tan_theta = sin_theta / cos_theta;

	/*
	** Propagate the state in time based on gyroscope measurements
    */
	imuData->RollAngle  += (p + q * sin_phi * tan_theta + r * cos_phi * tan_theta) * dt;
	imuData->PitchAngle += (q * cos_phi + r * sin_phi) * dt;

	imuData->I_Velocity += (-g * sin_theta - mu_m * u) * dt;
	imuData->J_Velocity += (g * cos_theta * sin_phi - mu_m * v) * dt;

	phi   = DEG_TO_RAD_F32 * imuData->RollAngle;
	theta = DEG_TO_RAD_F32 * imuData->PitchAngle;

	/*
	** Recalculate common trigonometric terms using new state estimation
	*/
	cos_phi = cosf(phi);
	sin_phi = sinf(phi);

	cos_theta = cosf(theta);
	sin_theta = sinf(theta);
	tan_theta = sin_theta / cos_theta;

	/*
	** Computing Jacobian A <==> jacobian(f(x,u), x)
	** Omitted elements = 0
	*/
	float A[4][4];
	A[0][0] = sin_theta * (q * cos_phi - r * sin_phi) / tan_theta;
	A[0][1] = (r * cos_phi + q * sin_phi) / (cos_theta * cos_theta);
	A[1][0] = r * cos_phi - q * sin_phi;
	A[2][1] = -g * cos_theta;
	A[2][2] = -mu_m;
	A[3][0] = -g * cos_phi * cos_theta;
	A[3][1] = -g * sin_phi * sin_theta;
	A[3][3] = -mu_m;

	/*
	** Computing Jacobian B <=> jacobian(f(x,u), u)
	** Omitted elements = 0
	*/
	float B[2][3];
	B[0][1] = sin_phi * tan_theta;
	B[0][2] = cos_phi * tan_theta;
	B[1][1] = cos_phi;
	B[1][2] = sin_phi;

	/*
	** Propagating uncertainty matrix P
	*/
	float dP[4][4];
	dP[0][0] = (ekf->R_g[1] * B[0][1] * B[0][1] + ekf->R_g[2] * B[0][2] * B[0][2] + ekf->Q[0] + ekf->R_g[0]
	            + 2 * A[0][0] * ekf->P[0][0] + A[0][1] * ekf->P[0][1] + A[0][1] * ekf->P[1][0])
	         * dt;

	dP[0][1] = (A[0][0] * ekf->P[0][1] + A[1][0] * ekf->P[0][0] + A[0][1] * ekf->P[1][1]
	            + B[0][1] * B[1][1] * ekf->R_g[1] + B[0][2] * B[1][2] * ekf->R_g[2])
	         * dt;

	dP[0][2] = (A[0][0] * ekf->P[0][2] + A[0][1] * ekf->P[1][2] + A[2][1] * ekf->P[0][1] + A[2][2] * ekf->P[0][2]) * dt;

	dP[0][3] = (A[0][0] * ekf->P[0][3] + A[0][1] * ekf->P[1][3] + A[3][0] * ekf->P[0][0] + A[3][1] * ekf->P[0][1]
	            + A[3][3] * ekf->P[0][3])
	         * dt;

	dP[1][0] = (A[0][0] * ekf->P[1][0] + A[1][0] * ekf->P[0][0] + A[0][1] * ekf->P[1][1]
	            + B[0][1] * B[1][1] * ekf->R_g[1] + B[0][2] * B[1][2] * ekf->R_g[2])
	         * dt;

	dP[1][1] = (ekf->R_g[1] * B[1][1] * B[1][1] + ekf->R_g[2] * B[1][2] * B[1][2] + ekf->Q[1] + A[1][0] * ekf->P[0][1]
	            + A[1][0] * ekf->P[1][0])
	         * dt;

	dP[1][2] = (A[1][0] * ekf->P[0][2] + A[2][1] * ekf->P[1][1] + A[2][2] * ekf->P[1][2]) * dt;

	dP[1][3] = (A[1][0] * ekf->P[0][3] + A[3][0] * ekf->P[1][0] + A[3][1] * ekf->P[1][1] + A[3][3] * ekf->P[1][3]) * dt;

	dP[2][0] = (A[0][0] * ekf->P[2][0] + A[0][1] * ekf->P[2][1] + A[2][1] * ekf->P[1][0] + A[2][2] * ekf->P[2][0]) * dt;

	dP[2][1] = (A[1][0] * ekf->P[2][0] + A[2][1] * ekf->P[1][1] + A[2][2] * ekf->P[2][1]) * dt;

	dP[2][2] = (ekf->Q[2] + A[2][1] * ekf->P[1][2] + A[2][1] * ekf->P[2][1] + 2 * A[2][2] * ekf->P[2][2]) * dt;

	dP[2][3] = (A[2][1] * ekf->P[1][3] + A[2][2] * ekf->P[2][3] + A[3][0] * ekf->P[2][0] + A[3][1] * ekf->P[2][1]
	            + A[3][3] * ekf->P[2][3])
	         * dt;

	dP[3][0] = (A[0][0] * ekf->P[3][0] + A[3][0] * ekf->P[0][0] + A[0][1] * ekf->P[3][1] + A[3][1] * ekf->P[1][0]
	            + A[3][3] * ekf->P[3][0])
	         * dt;

	dP[3][1] = (A[3][0] * ekf->P[0][1] + A[1][0] * ekf->P[3][0] + A[3][1] * ekf->P[1][1] + A[3][3] * ekf->P[3][1]) * dt;

	dP[3][2] = (A[3][0] * ekf->P[0][2] + A[3][1] * ekf->P[1][2] + A[2][1] * ekf->P[3][1] + A[2][2] * ekf->P[3][2]
	            + A[3][3] * ekf->P[3][2])
	         * dt;

	dP[3][3] = (ekf->Q[3] + A[3][0] * ekf->P[0][3] + A[3][1] * ekf->P[1][3] + A[3][0] * ekf->P[3][0]
	            + A[3][1] * ekf->P[3][1] + 2 * A[3][3] * ekf->P[3][3])
	         * dt;

	ekf->P[0][0] += dP[0][0];
	ekf->P[0][1] += dP[0][1];
	ekf->P[0][2] += dP[0][2];
	ekf->P[0][3] += dP[0][3];

	ekf->P[1][0] += dP[1][0];
	ekf->P[1][1] += dP[1][1];
	ekf->P[1][2] += dP[1][2];
	ekf->P[1][3] += dP[1][3];

	ekf->P[2][0] += dP[2][0];
	ekf->P[2][1] += dP[2][1];
	ekf->P[2][2] += dP[2][2];
	ekf->P[2][3] += dP[2][3];

	ekf->P[3][0] += dP[3][0];
	ekf->P[3][1] += dP[3][1];
	ekf->P[3][2] += dP[3][2];
	ekf->P[3][3] += dP[3][3];
}


void DF_EKF_UpdateCorrection_f32(DF_EKF_Instance_f32* ekf, FC_IMU_Data_Instance* imuData)
{
	float L[4][2];
	float P_p[4][4];

	const float C  = -ekf->mu / ekf->m;
	const float C2 = C * C;
	const float C3 = C * C2;
	const float C4 = C * C3;

	/*
	** Compute common therms for L gain
	*/
	const float sigma1 = ekf->R_a[0] * ekf->R_a[1] + C4 * ekf->P[2][2] * ekf->P[3][3] - C4 * ekf->P[2][3] * ekf->P[3][2]
	                   + C2 * ekf->P[2][2] * ekf->R_a[1] + C2 * ekf->P[3][3] * ekf->R_a[0];

	const float sigma2 = C2 * ekf->P[2][3] * ekf->P[3][2];

	const float sigma3 = C2 * ekf->P[2][2] * ekf->P[3][3];

	/*
	** Compute the correction gain matrix L
	*/
	L[0][0] =
	  C * (ekf->P[0][2] * ekf->R_a[1] + C2 * ekf->P[0][2] * ekf->P[3][3] - C2 * ekf->P[0][3] * ekf->P[3][2]) / sigma1;

	L[0][1] =
	  C * (ekf->P[0][3] * ekf->R_a[0] - C2 * ekf->P[0][2] * ekf->P[2][3] + C2 * ekf->P[0][3] * ekf->P[2][2]) / sigma1;

	L[1][0] =
	  C * (ekf->P[1][2] * ekf->R_a[1] + C2 * ekf->P[1][2] * ekf->P[3][3] - C2 * ekf->P[1][3] * ekf->P[3][2]) / sigma1;

	L[1][1] =
	  C * (ekf->P[1][3] * ekf->R_a[0] - C2 * ekf->P[1][2] * ekf->P[2][3] + C2 * ekf->P[1][3] * ekf->P[2][2]) / sigma1;

	L[2][0] = C * (ekf->P[2][2] * ekf->R_a[1] + sigma3 - sigma2) / sigma1;

	L[2][1] = C * ekf->P[2][3] * ekf->R_a[0] / sigma1;

	L[3][0] = C * ekf->P[3][2] * ekf->R_a[1] / sigma1;

	L[3][1] = C * (ekf->P[3][3] * ekf->R_a[0] + sigma3 - sigma2) / sigma1;


	/*
	** Compute updated matrix P
	*/
	P_p[0][0] = ekf->P[0][0] - C * L[0][0] * ekf->P[2][0] - C * L[0][1] * ekf->P[3][0];
	P_p[0][1] = ekf->P[0][1] - C * L[0][0] * ekf->P[2][1] - C * L[0][1] * ekf->P[3][1];
	P_p[0][2] = ekf->P[0][2] - C * L[0][0] * ekf->P[2][2] - C * L[0][1] * ekf->P[3][2];
	P_p[0][3] = ekf->P[0][3] - C * L[0][0] * ekf->P[2][3] - C * L[0][1] * ekf->P[3][3];

	P_p[1][0] = ekf->P[1][0] - C * L[1][0] * ekf->P[2][0] - C * L[1][1] * ekf->P[3][0];
	P_p[1][1] = ekf->P[1][1] - C * L[1][0] * ekf->P[2][1] - C * L[1][1] * ekf->P[3][1];
	P_p[1][2] = ekf->P[1][2] - C * L[1][0] * ekf->P[2][2] - C * L[1][1] * ekf->P[3][2];
	P_p[1][3] = ekf->P[1][3] - C * L[1][0] * ekf->P[2][3] - C * L[1][1] * ekf->P[3][3];

	P_p[2][0] = -ekf->P[2][0] * (C * L[2][0] - 1) - C * L[2][1] * ekf->P[3][0];
	P_p[2][1] = -ekf->P[2][1] * (C * L[2][0] - 1) - C * L[2][1] * ekf->P[3][1];
	P_p[2][2] = -ekf->P[2][2] * (C * L[2][0] - 1) - C * L[2][1] * ekf->P[3][2];
	P_p[2][3] = -ekf->P[2][3] * (C * L[2][0] - 1) - C * L[2][1] * ekf->P[3][3];

	P_p[3][0] = -ekf->P[3][0] * (C * L[3][1] - 1) - C * L[3][0] * ekf->P[2][0];
	P_p[3][1] = -ekf->P[3][1] * (C * L[3][1] - 1) - C * L[3][0] * ekf->P[2][1];
	P_p[3][2] = -ekf->P[3][2] * (C * L[3][1] - 1) - C * L[3][0] * ekf->P[2][2];
	P_p[3][3] = -ekf->P[3][3] * (C * L[3][1] - 1) - C * L[3][0] * ekf->P[2][3];

	ekf->P[0][0] = P_p[0][0];
	ekf->P[0][1] = P_p[0][1];
	ekf->P[0][2] = P_p[0][2];
	ekf->P[0][3] = P_p[0][3];

	ekf->P[1][0] = P_p[1][0];
	ekf->P[1][1] = P_p[1][1];
	ekf->P[1][2] = P_p[1][2];
	ekf->P[1][3] = P_p[1][3];

	ekf->P[2][0] = P_p[2][0];
	ekf->P[2][1] = P_p[2][1];
	ekf->P[2][2] = P_p[2][2];
	ekf->P[2][3] = P_p[2][3];

	ekf->P[3][0] = P_p[3][0];
	ekf->P[3][1] = P_p[3][1];
	ekf->P[3][2] = P_p[3][2];
	ekf->P[3][3] = P_p[3][3];


	const float u   = imuData->I_Velocity;
	const float v   = imuData->J_Velocity;
	const float a_i = imuData->AccelX;
	const float a_j = imuData->AccelY;

	/*
	** Updated system state variables based on computed correction gain and accel measurements
	*/
	imuData->RollAngle  = L[0][0] * (a_i - C * u) + L[0][1] * (a_j - C * v);
	imuData->PitchAngle = L[1][0] * (a_i - C * u) + L[1][1] * (a_j - C * v);
	imuData->I_Velocity = L[2][0] * (a_i - C * u) + L[2][1] * (a_j - C * v);
	imuData->J_Velocity = L[3][0] * (a_i - C * u) + L[3][1] * (a_j - C * v);
}
