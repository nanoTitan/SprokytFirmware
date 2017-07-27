#pragma once

#include <stdint.h>

typedef enum
{ 
	SERVO_CHANNEL_1 = 0,
	SERVO_CHANNEL_2	= 1,
	//SERVO_CHANNEL_3 = 2,
	//SERVO_CHANNEL_4 = 3,
	
	SERVO_CHANNEL_COUNT
} SERVO_CHANNEL;

void Servo_Init();
void Servo_SetPwm(int iMotorChannel, uint16_t fFrequency, float fPulsewidth);
void Servo_SetPwmFrequency();
void Servo_SetPwmPulsewidth(SERVO_CHANNEL tb_channel, float fPulsewidth);					/*The signal you would use to tell a traditional servo to go to its middle position, 1.5ms pulse signal (position "90" when used with the Arduino Servo library), will cause the FS90R to stop, a 2ms pulse signal (position "180") will cause the FS90R to rotate full speed counterclockwise, and a 1ms pulse signal (position "0") will cause the FS90R to rotate clockwise at full speed.*/ 