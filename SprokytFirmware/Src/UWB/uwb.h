#ifndef _UWB_H_
#define _UWB_H_

#include <stdio.h>
#include "dwm_api.h"

int UWB_Init();
void UWB_Send(uint8_t address, uint8_t data);
void UWB_Update();
bool UWB_IsReady();
void UWB_GetPosition(float* out_x, float* out_y, float* out_z);
bool UWB_GetModuleData(float* moduleData, uint8_t buffSz, uint8_t* out_size);

#endif // _UWB_H_