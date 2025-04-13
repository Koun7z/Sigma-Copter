//
// Created by pwoli on 14.03.2025.
//

#ifndef CONFIG_H
#define CONFIG_H

// If false, Aux1 channel controls arming
#define FC_EXTERNAL_ARM_STATUS 1

#define FC_CHANNEL_MAX      1812
#define FC_CHANNEL_MIN      172
#define FC_CHANNEL_MIDPOINT ((FC_CHANNEL_MAX + FC_CHANNEL_MIN) / 2.0f)

#define FC_THROTTLE_RESOLUTION 1400
#define FC_IDLE_THROTTLE       60

#define FC_PITCH_LINEAR_RATE 200
#define FC_ROLL_LINEAR_RATE  200
#define FC_YAW_LINEAR_RATE   200

/*
** RC data filtering
*/
#define FC_RC_AXIS_FILTER_ENABLE 1
#define FC_RC_AXIS_FILTER_ORDER  2

/*
** IMU data filtering
*/

// Gyroscope data filter
#define FC_IMU_GYRO_FILTER_ENABLE 1
#define FC_IMU_GYRO_FILTER_ORDER  2

// Acceleromer data filter
#define FC_IMU_ACCEL_FILTER_ENABLE 1
#define FC_IMU_ACCEL_FILTER_ORDER  2

// DF-EKF Config
#define DF_EKF_Q_COVARIANCE {0.01f, 0.01f, 0.01f, 0.01f}
#define DF_EKF_R_A_COVARIANCE {0.00004f, 0.00004f}
#define DF_EKF_R_G_COVARIANCE {0.025f, 0.025f, 0.025f}
#define DF_EKF_M 0.65f
#define DF_EKF_MU 0.6f


// NC Filter Config
#define FC_NC_ALPHA 0.05f

#endif //CONFIG_H
