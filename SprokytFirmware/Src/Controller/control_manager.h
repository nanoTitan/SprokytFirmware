#pragma once

#include <stdint.h>
#include "constants.h"

void ControlMgr_init();
void ControlMgr_setState(int state);
int ControlMgr_getState();
void ControlMgr_setType(int ctrlType);
void ControlMgr_update();
void ControlMgr_setMotor(uint8_t motorIndex, uint8_t value, direction_t dir);
void ControlMgr_parseInstruction(uint8_t data_length, uint8_t *att_data);
uint32_t ControlMgr_Get_Battery();