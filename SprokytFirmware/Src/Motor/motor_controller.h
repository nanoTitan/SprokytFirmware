#pragma once

#include <stdint.h>
#include "constants.h"
#include "Servo.h"

void MotorController_init();
void MotorController_setMotor(uint8_t motorIndxMask, float power, direction_t direction);
void MotorController_setServo(SERVO_CHANNEL servo, float dutyCycle);
int MotorController_isArmed();
void MotorController_callibrateESCs();

void MotorController_UpdateMotorTest();