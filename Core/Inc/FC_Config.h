//
// Created by pwoli on 14.03.2025.
//

#ifndef CONFIG_H
#define CONFIG_H

/*
** Serial port debug
*/
#define FC_SERIAL_DEBUG 1
#define FC_SERIAL_REPORT_RATE 30

#if FC_SERIAL_DEBUG
#  include <stdio.h>
#  define FC_DEBUG_LOG(...) printf(__VA_ARGS__)
#else
#  define FC_DEBUG_LOG(...)
#endif

/*
** RC data input
*/

// If false, Aux1 channel controls arming
#define FC_EXTERNAL_ARM_STATUS 1

#define FC_ROLL_DIRECTION 1
#define FC_PITCH_DIRECTION -1
#define FC_YAW_DIRECTION 1
#define FC_THROTTLE_DIRECTION 1

#define FC_CHANNEL_MAX      1812
#define FC_CHANNEL_MIN      172
#define FC_CHANNEL_MIDPOINT ((FC_CHANNEL_MAX + FC_CHANNEL_MIN) / 2.0f)

#define FC_IDLE_THROTTLE     3  // [%]
#define FC_MAX_THROTTLE      80 // 0 - 100 [%]

#define FC_PITCH_LINEAR_RATE 200 // [deg/s]
#define FC_ROLL_LINEAR_RATE  200 // [deg/s]
#define FC_YAW_LINEAR_RATE   200 // [deg/s]

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

/*
** Attitude estimators config
*/

// Nonlinear complementary filter config
#define FC_NC_GAIN 0.05f
#define FC_NC_SLERP_THRESHOLD 0.90f
#define FC_NC_THRESHOLD1 0.1f
#define FC_NC_THRESHOLD2 0.2f

#endif //CONFIG_H
