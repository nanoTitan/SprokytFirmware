#pragma once

#include "constants.h"
#include <stdint.h>

struct PIDInfo;

void ServoCameraControl_init();
void ServoCameraControl_update();
void ServoCameraControl_parseInstruction(uint8_t data_length, uint8_t *att_data);
void ServoCameraControl_setMotor(uint8_t motorIndex, float value, direction_t direction);
