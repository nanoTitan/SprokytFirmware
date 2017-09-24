#pragma once

#include "constants.h"
#include <stdint.h>

struct PIDInfo;

void StepperCameraControl_init();
void StepperCameraControl_update();
void StepperCameraControl_parseInstruction(uint8_t data_length, uint8_t *att_data);
void StepperCameraControl_setMotor(uint8_t motorIndex, float value, direction_t direction);
