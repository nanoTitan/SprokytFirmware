#pragma once

#include "constants.h"
#include <stdint.h>

struct PIDInfo;

void CameraControl_init();
void CameraControl_update();
void CameraControl_parseInstruction(uint8_t data_length, uint8_t *att_data);
void CameraControl_setMotor(uint8_t motorIndex, float value, direction_t direction);
