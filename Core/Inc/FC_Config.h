//
// Created by pwoli on 14.03.2025.
//

#ifndef CONFIG_H
#define CONFIG_H

#include "DSP_Constants.h"

/*
** Serial port debug
*/
#define FC_SERIAL_DEBUG       1
#define FC_SERIAL_REPORT_RATE 30

/*
** RC data input
*/

// If false, Aux1 channel controls arming
#define FC_EXTERNAL_ARM_STATUS 1

#define FC_ROLL_DIRECTION     1
#define FC_PITCH_DIRECTION    -1
#define FC_YAW_DIRECTION      1
#define FC_THROTTLE_DIRECTION 1

#define FC_RC_AXES_DEAD_ZONE     1.0f  // [Linear rate %]
#define FC_RC_THROTTLE_DEAD_ZONE 1.0f  // [Throttle %]

#define FC_PITCH_LINEAR_RATE 200       // [deg/s]
#define FC_PITCH_MAX_RATE    200       // [deg/s]  <- Not implemented
#define FC_PITCH_EXPO        0         //	       <- Not implemented

#define FC_ROLL_LINEAR_RATE 200        // [deg/s]
#define FC_ROLL_MAX_RATE    200        // [deg/s]  <- Not implemented
#define FC_PITCH_EXPO       0          //	       <- Not implemented

#define FC_YAW_LINEAR_RATE 200         // [deg/s]
#define FC_YAW_MAX_RATE    200         // [deg/s]  <- Not implemented
#define FC_YAW_EXPO        0           //	       <- Not implemented


#define FC_IDLE_THROTTLE 5   // [%]
#define FC_MAX_THROTTLE  80  // 0 - 100 [%]

#define FC_CHANNEL_MAX      1812
#define FC_CHANNEL_MIN      172
#define FC_CHANNEL_MIDPOINT ((FC_CHANNEL_MAX + FC_CHANNEL_MIN) / 2.0f)

#define FC_ROLL_DEAD_ZONE  (FC_RC_AXES_DEAD_ZONE / 100.0f * FC_PITCH_LINEAR_RATE * DEG_TO_RAD_F32)
#define FC_PITCH_DEAD_ZONE (FC_RC_AXES_DEAD_ZONE / 100.0f * FC_ROLL_LINEAR_RATE * DEG_TO_RAD_F32)
#define FC_YAW_DEAD_ZONE   (FC_RC_AXES_DEAD_ZONE / 100.0f * FC_YAW_LINEAR_RATE * DEG_TO_RAD_F32)


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
#define FC_NC_ACC_GAIN        0.05f
#define FC_NC_MAG_GAIN        1.0f
#define FC_NC_SLERP_THRESHOLD 0.90f
#define FC_NC_THRESHOLD1      0.1f
#define FC_NC_THRESHOLD2      0.2f


/*
** Common PID parameres
*/

#define FC_PID_Ts             0.005f  // Update should be called every specified interval
#define FC_THROTTLE_GAIN_COMP 5.0f    // Compensation for thrust to throttle nonlinearity

/*
** Position PID parameres
*/

#define FC_POSITION_PID_SATURATION_Tt 2.0f

#define FC_ROLL_POSITION_Kp 1.0f
#define FC_ROLL_POSITION_Ti 0.5f
#define FC_ROLL_POSITION_Td 0.2f
#define FC_ROLL_POSITION_N  10.0f

#define FC_PITCH_POSITION_Kp 1.0f
#define FC_PITCH_POSITION_Ti 0.5f
#define FC_PITCH_POSITION_Td 0.2f
#define FC_PITCH_POSITION_N  10.0f

#define FC_YAW_POSITION_Kp 1.0f
#define FC_YAW_POSITION_Ti 0.5f
#define FC_YAW_POSITION_Td 0.2f
#define FC_YAW_POSITION_N  10.0f

/*
** Rate PID parameres
*/

#define FC_RATE_PID_SATURATION 30.0f
#define FC_RATE_PID_DeadZone   0.5f

#define FC_ROLL_RATE_Kp 8.0f
#define FC_ROLL_RATE_Ti 0.2f
#define FC_ROLL_RATE_Td 0.03f
#define FC_ROLL_RATE_Tt FC_ROLL_RATE_Ti
#define FC_ROLL_RATE_N  10.0f

#define FC_PITCH_RATE_Kp 8.0f
#define FC_PITCH_RATE_Ti 0.2f
#define FC_PITCH_RATE_Td 0.03f
#define FC_PITCH_RATE_Tt FC_PITCH_RATE_Ti
#define FC_PITCH_RATE_N  10.0f

#define FC_YAW_RATE_Kp 8.0f
#define FC_YAW_RATE_Ti 0.2f
#define FC_YAW_RATE_Td 0.0f
#define FC_YAW_RATE_Tt FC_YAW_RATE_Ti
#define FC_YAW_RATE_N  10.0f


#endif  // CONFIG_H
