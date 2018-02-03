#ifndef _UWB_H_
#define _UWB_H_

#include <stdio.h>

int UWB_Init();
void UWB_Send(uint8_t address, uint8_t data);
void UWB_Update();

#endif // _UWB_H_