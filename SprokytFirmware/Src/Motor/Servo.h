#pragma once

#include <stdint.h>


void Servo_Init();
void Servo_SetDutyCycle(int channel, float dutyCycle);					/*The signal you would use to tell a traditional servo to go to its middle position, 1.5ms pulse signal (position "90" when used with the Arduino Servo library), will cause the FS90R to stop, a 2ms pulse signal (position "180") will cause the FS90R to rotate full speed counterclockwise, and a 1ms pulse signal (position "0") will cause the FS90R to rotate clockwise at full speed.*/ 