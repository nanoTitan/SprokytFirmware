#pragma once

#include <stdint.h>
#include "constants.h"

void MotorController_init();
void MotorController_setMotor(uint8_t motorIndxMask, float power, direction_t direction);
int MotorController_isArmed();
void MotorController_callibrateESCs();