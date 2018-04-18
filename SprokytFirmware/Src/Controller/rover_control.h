#ifndef _ROVER_CONTROL_H_
#define _ROVER_CONTROL_H_

#include "constants.h"
#include <stdint.h>

struct PIDInfo;

void RoverControl_init();
void RoverControl_update();
void RoverControl_parseInstruction(uint8_t data_length, uint8_t *att_data);
void RoverControl_setMotor(uint8_t motorIndex, float value, direction_t direction);

void RoverControl_testEkf();

#endif // _ROVER_CONTROL_H_