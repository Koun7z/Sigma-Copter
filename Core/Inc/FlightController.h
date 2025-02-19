/*
 * FlightController.h
 *
 *  Created on: Jan 28, 2025
 *      Author: pwoli
 */

#ifndef INC_FLIGHTCONTROLLER_H_
#define INC_FLIGHTCONTROLLER_H_

#include <stdint.h>

struct FC_MotorThrust
{
	uint16_t Motor1; // FR
	uint16_t Motor2; // FL
	uint16_t Motor3; // RR
	uint16_t Motor4; // RL
};

extern struct PID_State PitchRatePID;
extern struct PID_State RollRatePID;
extern struct PID_State YawRatePID;

extern struct FC_MotorThrust FC_GlobalThrust;
extern struct IMU_Data FC_IMUData;

uint8_t FC_Init();

void FC_Update(float dt);

void FC_ManualArm();
void FC_ManualDisarm();

#endif /* INC_FLIGHTCONTROLLER_H_ */
