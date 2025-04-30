//
// Created by pwoli on 30.04.2025.
//

#ifndef FC_ATTITUDETARGET_H
#define FC_ATTITUDETARGET_H

#include "FC_CommonTypes.h"

void FC_IntegrateTargetAttitude(FC_IMU_Data_Instance* imuData, const FC_RC_Data_Instance* rcData, float dt);

void FC_SetTargetAttitude(FC_IMU_Data_Instance* imuData, const FC_RC_Data_Instance* rcData, float dt);

void FC_GetTargetVelocity(const FC_IMU_Data_Instance* imuData, float* targetVelocity);

#endif //FC_ATTITUDETARGET_H
