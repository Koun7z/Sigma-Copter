//
// Created by pwoli on 18.03.2025.
//

#ifndef FC_NC_FILTER_H
#define FC_NC_FILTER_H

#include "FC_CommonTypes.h"

#include <stdint.h>

typedef struct
{
	float Gain;
	float Threshold1;
	float Threshold2;
}FC_NC_Instance_f32;

uint8_t FC_NC_Init_f32(FC_NC_Instance_f32* instance, float Gain, float Threshold1, float Threshold2);

void FC_NC_FilterUpdate_f32(const FC_NC_Instance_f32* filter, FC_IMU_Data_Instance* imuData, float dt);

#endif //FC_NC_FILTER_H
