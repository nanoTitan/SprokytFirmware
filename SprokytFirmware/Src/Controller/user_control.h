#ifndef _USER_CONTROL_H_
#define _USER_CONTROL_H_

#include "constants.h"
#include <stdint.h>

struct PIDInfo;

void UserControl_init();
void UserControl_update();
void UserControl_parseInstruction(uint8_t data_length, uint8_t *att_data);
void UserControl_setMotor(uint8_t motorIndex, float value, direction_t direction);

#endif // _USER_CONTROL_H_