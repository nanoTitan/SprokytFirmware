#pragma once

#include <stdint.h>
#include "constants.h"

void Stepper_Init();
void Stepper_Update();
void Stepper_RegisterAngularPosCallback(AngularPositionCallback callback);
void Stepper_SetSpeedAndDirection(float speed, direction_t direction);
void Stepper_SetSpeed(float speed);
void Stepper_SetDirection(direction_t direction);
void Stepper_MotorTest();