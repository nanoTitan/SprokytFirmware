#include "motor_controller.h"
#include "math_ext.h"
#include "TB6612FNG.h"
#include "stm32f4xx_hal.h"
#include "debug.h"

/* Private Variables ------------------------------------------------------------------*/
//Timeout _motorArmTimeout;
//int _motorsArmed = 0;


//TB6612FNG motorDriver1(PB_1, PA_1, PA_0, PB_0, PA_5, PA_6, PA_4);
//TB6612FNG motorDriver2(PB_6, PA_15, PA_8, PC_7, PC_3, PC_4, PC_2);
//PwmOut m_bldcArray[] = { PA_11, PB_10, PC_6 };
//TB6612FNG motorDriver1(PB_1, PB_2, PC_5, PB_0, PB_12, PB_13, PA_4);
//TB6612FNG motorDriver2(PA_6, PB_15, PB_14, PA_8, PC_3, PC_4, PC_2);
//PwmOut m_bldcArray[] = { PA_0, PB_10, PB_7, PB_6 };
//PwmOut m_bldcArray[] = { PB_6, PB_6, PB_6, PB_6 };

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotorsCallback();
static void MotorController_armESCs();



#ifdef MOTOR_STSPIN
void STSpinInit();
void MotorController_setMotors_STSPIN(uint8_t motorIndxMask, float power, direction_t dir);
void STSPIN_setMotor(uint8_t indx, float pwm, uint8_t direction);
#elif defined(MOTOR_ESC)

#elif defined(MOTOR_TOSHIBA)
void MotorController_setMotors_TB6612(uint8_t motorIndxMask, float power, uint8_t direction);
//void MotorController_setServos(uint8_t motorIndxMask, float power, direction_t direction);
#endif

void MotorController_init()
{
	// 20000 us = 50 Hz, 2040.8us = 490 Hz, 83.3 us = 12 Khz	
	float frequency = 50000;      // 50KHz
	
	TB_Init();
	TB_SetPwmPulsewidth(TB_CHANNEL_A1, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_B1, 0);
	//TB_SetPwmPulsewidth(TB_CHANNEL_A2, 0);
	//TB_SetPwmPulsewidth(TB_CHANNEL_B2, 0);
	TB_SetWorkMode(TB_CHANNEL_A1, TB_CM_STOP);
	TB_SetWorkMode(TB_CHANNEL_B1, TB_CM_STOP);
	//TB_SetWorkMode(TB_CHANNEL_A2, TB_CM_STOP);
	//TB_SetWorkMode(TB_CHANNEL_B2, TB_CM_STOP);
	
	// MotorController_setMotor(MOTOR_ALL, 0, BWD);
}

void MotorController_UpdateMotorTest()
{
	TB_SetWorkMode(TB_CHANNEL_A1, TB_CM_CW);
	TB_SetWorkMode(TB_CHANNEL_B1, TB_CM_CW);
	
	static float pwm = 0;
	int dir = 1;
	HAL_Delay(1);
	if (pwm < 1)
		pwm += 0.01;
	
	PRINTF("pwm: %f\r\n", pwm);
	
	TB_SetPwmPulsewidth(TB_CHANNEL_A1, pwm);
	TB_SetPwmPulsewidth(TB_CHANNEL_B1, pwm);
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