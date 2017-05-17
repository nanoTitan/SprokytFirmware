#include "motor_controller.h"
#include "math_ext.h"
#include "TB6612FNG.h"
#include "stm32f4xx_hal.h"
#include "debug.h"

/* Private Variables ------------------------------------------------------------------*/
//Timeout _motorArmTimeout;
//int _motorsArmed = 0;

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotorsCallback();
static void MotorController_armESCs();

void MotorController_init()
{
	// 20000 us = 50 Hz, 2040.8us = 490 Hz, 83.3 us = 12 Khz	
	float frequency = 50000;      // 50KHz
	
	TB_Init();
	TB_SetPwmPulsewidth(TB_CHANNEL_A1, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_B1, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_A2, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_B2, 0);
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_STOP);
	
	// MotorController_setMotor(MOTOR_ALL, 0, BWD);
}

void MotorController_UpdateMotorTest()
{
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_CW);
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_CW);
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_CW);
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_CW);
	
	static float pwm = 0;
	int dir = 1;
	
	for (int i = 1; i < 10; ++i)
	{
		TB_SetPwmPulsewidth(TB_CHANNEL_A1, pwm);
		TB_SetPwmPulsewidth(TB_CHANNEL_B1, pwm);
		TB_SetPwmPulsewidth(TB_CHANNEL_A2, pwm);
		TB_SetPwmPulsewidth(TB_CHANNEL_B2, pwm);
		HAL_Delay(200);
		pwm += 0.1f;
	}
	
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_STOP);
	HAL_Delay(2000);
	
	pwm = 1;
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_CCW);
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_CCW);
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_CCW);
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_CCW);
	for (int i = 1; i < 10; ++i)
	{
		TB_SetPwmPulsewidth(TB_CHANNEL_A1, pwm);
		TB_SetPwmPulsewidth(TB_CHANNEL_B1, pwm);
		TB_SetPwmPulsewidth(TB_CHANNEL_A2, pwm);
		TB_SetPwmPulsewidth(TB_CHANNEL_B2, pwm);
		HAL_Delay(200);
		pwm -= 0.1f;
	}
	
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_STOP);
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_STOP);
	TB_SetPwmPulsewidth(TB_CHANNEL_A1, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_B1, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_A2, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_B2, 0);
	HAL_Delay(2000);
}

/*
int MotorController_isArmed()
{
	return _motorsArmed;
}

void MotorController_setMotor(uint8_t motorIndxMask, float power, direction_t dir)
{
	MotorController_setMotors_TB6612(motorIndxMask, power, dir);
}

void MotorController_setMotors_TB6612(uint8_t motorIndxMask, float power, uint8_t direction)
{
	if (motorIndxMask & MOTOR_A)
	{
		if (power == 0)
			motorDriver1.motorA_stop();
		else
		{
			motorDriver1.setPwmApulsewidth(power);
			if (direction == FWD)
				motorDriver1.motorA_cw();
			else
				motorDriver1.motorA_ccw();
		}		
	}
	
	if (motorIndxMask & MOTOR_B)
	{
		if (power == 0)
			motorDriver1.motorB_stop();
		else
		{
			motorDriver1.setPwmBpulsewidth(power);	
			if (direction == FWD)
				motorDriver1.motorB_cw();
			else
				motorDriver1.motorB_ccw();
		}
	}
	
	if (motorIndxMask & MOTOR_C)
	{
		if (power == 0)
			motorDriver2.motorA_stop();
		else
		{
			motorDriver2.setPwmApulsewidth(power);	
			if (direction == FWD)
				motorDriver2.motorA_cw();
			else
				motorDriver2.motorA_ccw();
		}
	}
	
	if (motorIndxMask & MOTOR_D)
	{
		if (power == 0)
			motorDriver2.motorB_stop();
		else
		{
			motorDriver2.setPwmBpulsewidth(power);	
			if (direction == FWD)
				motorDriver2.motorB_cw();
			else
				motorDriver2.motorB_ccw();
		}
	}
}
*/

#if 0
void MotorController_setServos(uint8_t motorIndxMask, float power, direction_t direction)
{
	// 1000us: CCW 100%, 1500us: Stop, 2000us CW 100%
	if (power < 0)
		power = 0;
	
	float pwm = mapf(power, 0, 1, 0, 500);
	if (direction == FWD)
		pwm += 1500;
	else
		pwm = 1500 - pwm;
	
//	if (motorIndxMask & MOTOR_A)
//		m_bldcArray[0].pulsewidth_us(pwm);	
//	if (motorIndxMask & MOTOR_B)
//		m_bldcArray[1].pulsewidth_us(pwm);
//	if (motorIndxMask & MOTOR_C)
//		m_bldcArray[2].pulsewidth_us(pwm);
//	if (motorIndxMask & MOTOR_D)
//		m_bldcArray[3].pulsewidth_us(pwm);
}
#endif