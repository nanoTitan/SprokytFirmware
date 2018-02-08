#ifndef _UWB_H_
#define _UWB_H_

#include <stdio.h>
#include "dwm_api.h"

int UWB_Init();
void UWB_Send(uint8_t address, uint8_t data);
void UWB_Update();
bool UWB_HasPosition();
void UWB_GetPosition(float* out_x, float* out_y, float* out_z);

#endif // _UWB_H_