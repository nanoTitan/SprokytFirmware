#include "motor_controller.h"
#include "math_ext.h"
#include "TB6612FNG.h"
#include "Servo.h"
#include "stm32f4xx_hal.h"
#include "debug.h"

/* Private Variables ------------------------------------------------------------------*/
int _motorsArmed = 0;

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotorsCallback();
static void MotorController_setMotors_TB6612(uint8_t motorIndxMask, float power, direction_t direction);
static void MotorController_setServos(uint8_t motorIndxMask, float power, direction_t dir);

void MotorController_init()
{
#if defined(MOTOR_TOSHIBA)
	TB_Init();
	TB_SetPwmPulsewidth(TB_CHANNEL_A1, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_B1, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_A2, 0);
	TB_SetPwmPulsewidth(TB_CHANNEL_B2, 0);
	
	MotorController_setMotor(MOTOR_ALL, 0, FWD);
#elif defined(MOTOR_SERVO)
	Servo_Init();
	Servo_SetPwmPulsewidth(SERVO_CHANNEL_1, 0);
	//Servo_SetPwmPulsewidth(SERVO_CHANNEL_2, 0)
#endif // MOTOR_TOSHIBA
}

int MotorController_isArmed()
{
	return _motorsArmed;
}

void MotorController_setMotor(uint8_t motorIndxMask, float power, direction_t dir)
{
#if defined(MOTOR_TOSHIBA)
	MotorController_setMotors_TB6612(motorIndxMask, power, dir);
#elif defined(MOTOR_SERVO)
	MotorController_setServos(motorIndxMask, power, dir);
#endif // MOTOR_TOSHIBA
}

void MotorController_setMotors_TB6612(uint8_t motorIndxMask, float power, direction_t direction)
{
	if (motorIndxMask & MOTOR_A)
	{
		if (power == 0)
			TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_STOP);
		else
		{
			TB_SetPwmPulsewidth(TB_CHANNEL_A1, power);
			if (direction == FWD)
				TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_CW);
			else
				TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_CCW);
		}		
	}
	
	if (motorIndxMask & MOTOR_B)
	{
		if (power == 0)
			TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_STOP);
		else
		{
			TB_SetPwmPulsewidth(TB_CHANNEL_B1, power);
			if (direction == FWD)
				TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_CW);
			else
				TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_CCW);
		}
	}
	
	if (motorIndxMask & MOTOR_C)
	{
		if (power == 0)
			TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_STOP);
		else
		{
			TB_SetPwmPulsewidth(TB_CHANNEL_A2, power);
			if (direction == FWD)
				TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_CW);
			else
				TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_CCW);
		}
	}
	
	if (motorIndxMask & MOTOR_D)
	{
		if (power == 0)
			TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_STOP);
		else
		{
			TB_SetPwmPulsewidth(TB_CHANNEL_B2, power);
			if (direction == FWD)
				TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_CW);
			else
				TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_CCW);
		}
	}
}

void MotorController_setServos(uint8_t motorIndxMask, float power, direction_t dir)
{
	
}

void MotorController_UpdateMotorTest()
{
#if defined(MOTOR_TOSHIBA)
	
#define TEST_CHN_A1	// constant spin, PWM doesn't change
#define TEST_CHN_B1	// good
#define TEST_CHN_A2	// good
#define TEST_CHN_B2	// good
	
#if defined(TEST_CHN_A1)
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_CW);
#endif
#if defined(TEST_CHN_B1)
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_CW);
#endif
#if defined(TEST_CHN_A2)
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_CW);
#endif
#if defined(TEST_CHN_B2)
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_CW);
#endif
	
	static float pwm = 0;
	int dir = 1;
	
	for (int i = 1; i < 10; ++i)
	{
#if defined(TEST_CHN_A1)
		TB_SetPwmPulsewidth(TB_CHANNEL_A1, pwm);
#endif
#if defined(TEST_CHN_B1)
		TB_SetPwmPulsewidth(TB_CHANNEL_B1, pwm);
#endif
#if defined(TEST_CHN_A2)
		TB_SetPwmPulsewidth(TB_CHANNEL_A2, pwm);
#endif
#if defined(TEST_CHN_B2)
		TB_SetPwmPulsewidth(TB_CHANNEL_B2, pwm);
#endif
		HAL_Delay(200);
		pwm += 0.1f;
	}
	
#if defined(TEST_CHN_A1)
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_STOP);
#endif
#if defined(TEST_CHN_B1)
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_STOP);
#endif
#if defined(TEST_CHN_A2)
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_STOP);
#endif
#if defined(TEST_CHN_B2)
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_STOP);
#endif
	HAL_Delay(2000);
	
	pwm = 1;
#if defined(TEST_CHN_A1)
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_CCW);
#endif
#if defined(TEST_CHN_B1)
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_CCW);
#endif
#if defined(TEST_CHN_A2)
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_CCW);
#endif
#if defined(TEST_CHN_B2)
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_CCW);
#endif
	
	for (int i = 1; i < 10; ++i)
	{
#if defined(TEST_CHN_A1)
		TB_SetPwmPulsewidth(TB_CHANNEL_A1, pwm);
#endif
#if defined(TEST_CHN_B1)
		TB_SetPwmPulsewidth(TB_CHANNEL_B1, pwm);
#endif
#if defined(TEST_CHN_A2)
		TB_SetPwmPulsewidth(TB_CHANNEL_A2, pwm);
#endif
#if defined(TEST_CHN_B2)
		TB_SetPwmPulsewidth(TB_CHANNEL_B2, pwm);
#endif
		HAL_Delay(200);
		pwm -= 0.1f;
	}
	
#if defined(TEST_CHN_A1)
	TB_SetWorkMode(TB_CHANNEL_A1, TB_ControlMode_STOP);
	TB_SetPwmPulsewidth(TB_CHANNEL_A1, 0);
#endif
#if defined(TEST_CHN_B1)
	TB_SetWorkMode(TB_CHANNEL_B1, TB_ControlMode_STOP);
	TB_SetPwmPulsewidth(TB_CHANNEL_B1, 0);
#endif
#if defined(TEST_CHN_A2)
	TB_SetWorkMode(TB_CHANNEL_A2, TB_ControlMode_STOP);
	TB_SetPwmPulsewidth(TB_CHANNEL_A2, 0);
#endif
#if defined(TEST_CHN_B2)
	TB_SetWorkMode(TB_CHANNEL_B2, TB_ControlMode_STOP);
	TB_SetPwmPulsewidth(TB_CHANNEL_B2, 0);
#endif
	
#elif defined(MOTOR_SERVO)
	
	float pwm = 0;
	int dir = 1;
	for (int i = 0; i < 11; ++i)
	{
		Servo_SetPwmPulsewidth(SERVO_CHANNEL_1, pwm);
		HAL_Delay(500);
		pwm += 0.1f;
	}
	
	Servo_SetPwmPulsewidth(SERVO_CHANNEL_1, 0.5f);
#endif // MOTOR_TOSHIBA
	
	HAL_Delay(2000);
}