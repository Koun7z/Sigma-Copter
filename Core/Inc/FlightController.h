/*
** FlightController.h
**
**  Created on: Jan 28, 2025
**      Author: pwoli
*/

#ifndef INC_FLIGHTCONTROLLER_H_
#define INC_FLIGHTCONTROLLER_H_

#include "FC_Config.h"
#include "FC_CommonTypes.h"
#include "ProcessControl.h"

#include <stdint.h>
#include <stddef.h>
/**
 * @Brief Structure holding calculated thrust of all motors as a float value between 0 - 100
 */
extern FC_MotorThrust FC_GlobalThrust;

/**
 * @Brief Structure holding filtered RC channels data
 */
extern FC_RC_Data_Instance FC_RC_Data;

/**
 * @Brief Structure holding filtered IMU data
 */
extern FC_IMU_Data_Instance FC_IMU_Data;

extern bool FC_EmergencyDisarmStatus;

/**
 * @brief      Debug message callback
 * @param[in]  msg  Debug message string
 * @param[in]  len  N-characters
 */
void FC_OnDebugLog(const char* msg, size_t len);

/**
 * @brief   Initializes flight controller
 * @retval  uint8_t:
 *			- 0 - Initialization success
 */
uint8_t FC_Init(void);

/**
 * @brief      Main update loop of the controller
 * @param[in]  dt Fixed refresh rate of the update loop [s]
 *
 * @warning  Update function runs as fast as you call it,
 *			 it is the user that has to ensure that it runs at a specific, fixed refresh rate.
 */
void FC_Update(float dt);

/**
 * @brief  Emergency disarm function, it prevents main loop from executing
 *         and sets all motor power to 0 until rearmed with corresponding function
 */
void FC_EmergencyDisarm(void);

/**
 * @brief  Disables emergency disarm status
 */
void FC_EmergencyRearm(void);

/**
 * @brief  Called internally by FC_Init() function.
 *		   Initialize data acquisition parameters.
 */
void FC_DataAcquisitionInit(void);

/**
 * @brief      Used to update control inputs from RC handset.
 *			   Should be called at least once before every FC_Update().
 *			   The function performs data conversion and filtering based on settings in FC_Config.h file.
 * @param[in]  throttle  Raw throttle value from handset
 * @param[in]  roll      Raw roll value
 * @param[in]  pitch	 Raw pitch value
 * @param[in]  yaw       Raw yaw value
 */
void FC_RC_UpdateAxesChannels(int throttle, int roll, int pitch, int yaw);

/**
 * @brief	   Used to update auxiliary inputs from RC handset.
 *			   If external arm status is disabled, uses Aux1 channel as Arm status.
 *			   No filtering is performed on the Aux channels.
 * @param[in]  aux1  Raw Aux1 data
 * @param[in]  aux2  Raw Aux2 data
 * @param[in]  aux3  Raw Aux3 data
 * @param[in]  aux4  Raw Aux4 data
 */
void FC_RC_UpdateAuxChannels(int aux1, int aux2, int aux3, int aux4);

/**
 * @brief     Used to manually set the arm status.
 * @param[in] armStatus:
              - true  - Armed
              - false - Disarmed
 */
void FC_RC_UpdateArmStatus(bool armStatus);

/**
 * @brief	   Sets the internal RC IIR filter coefficients.
 *			   The coefficients are copied into an internal array, and should be passed in standard order.
 *			   Number of coefficients to pass depends on the filter order set in FC_Config file.
 * @param[in] *filterCoeffsNumerator    Array of IIR numerator coefficients
 * @param[in] *filterCoeffsDenominator  Array of IIR denominatior coefficients
 */
void FC_RC_SetFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

/**
 * @brief       Used to update gyroscope measurements.
 *			   Should be called at least once before every FC_Update().
 *			   The function performs data filtering based on settings in FC_Config.h file.
 *			   Function should be called before accelerometer update as it also performs EKF prediction step.
 * @param[in]  rollRate   Roll rate [rad/s]
 * @param[in]  pitchRate  Pitch rate [rad/s]
 * @param[in]  yawRate    Yaw rate [rad/s]
 * @param[in]  dt         Fixed sampling time [s]
 */
void FC_IMU_UpdateGyro(float rollRate, float pitchRate, float yawRate, float dt);

/**
 * @brief	   Sets the internal gyroscope IIR filter coefficients.
 *			   The coefficients are copied into an internal array, and should be passed in standard order.
 *			   Number of coefficients to pass depends on the filter order set in FC_Config file.
 * @param[in] *filterCoeffsNumerator    Array of IIR numerator coefficients
 * @param[in] *filterCoeffsDenominator  Array of IIR denominatior coefficients
 */
void FC_IMU_SetGyroFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

/**
* @brief       Used to update accelerometer measurements.
 *			   Should be called at least once before every FC_Update().
 *			   The function performs data filtering based on settings in FC_Config.h file.
 *			   Function should be called after gyroscope update as it also performs attitude estimation (EKF correction)
 * @param[in]  rollRate   Roll rate [g]
 * @param[in]  pitchRate  Pitch rate [g]
 * @param[in]  yawRate    Yaw rate [g]
 * @param[in]  dt         Fixed sampling time [s]
 */
void FC_IMU_UpdateAccel(float accelX, float accelY, float accelZ, float dt);

/**
 * @brief	   Sets the internal accelerometer IIR filter coefficients.
 *			   The coefficients are copied into an internal array, and should be passed in standard order.
 *			   Number of coefficients to pass depends on the filter order set in FC_Config file.
 * @param[in] *filterCoeffsNumerator    Array of IIR numerator coefficients
 * @param[in] *filterCoeffsDenominator  Array of IIR denominatior coefficients
 */
void FC_IMU_SetAccelFilter(const float* filterCoeffsNumerator, const float* filterCoeffsDenominator);

#endif /* INC_FLIGHTCONTROLLER_H_ */
