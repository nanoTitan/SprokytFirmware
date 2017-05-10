#include "motor_controller.h"
#include "mbed.h"
#include "math_ext.h"

#if defined(MOTOR_STSPIN)
#include "STSpin.h"
#elif defined(MOTOR_TOSHIBA)
#include "TB6612FNG.h"
#endif


/* Private Variables ------------------------------------------------------------------*/
Timeout _motorArmTimeout;
bool _motorsArmed = false;

#if defined(MOTOR_ESC)

#elif defined(MOTOR_TOSHIBA)
#ifdef FIRMWARE_VERSION_0_2_2
TB6612FNG motorDriver1(PB_1, PA_1, PA_0, PB_0, PA_5, PA_6, PA_4);
TB6612FNG motorDriver2(PB_6, PA_15, PA_8, PC_7, PC_3, PC_4, PC_2);
//PwmOut m_bldcArray[] = { PA_11, PB_10, PC_6 };
#elseTB6612FNG motorDriver1(PB_1, PB_2, PC_5, PB_0, PB_12, PB_13, PA_4);
TB6612FNG motorDriver2(PA_6, PB_15, PB_14, PA_8, PC_3, PC_4, PC_2);
//PwmOut m_bldcArray[] = { PA_0, PB_10, PB_7, PB_6 };
//PwmOut m_bldcArray[] = { PB_6, PB_6, PB_6, PB_6 };
#endif
#elif defined(MOTOR_STSPIN)
#endif

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
#ifdef MOTOR_STSPIN	
	STSpinInit();
#elif defined(MOTOR_ESC)
	
#elif defined(MOTOR_TOSHIBA)
	float fPwmPeriod = 0.00002f;      // 50KHz
	motorDriver1.setPwmAperiod(fPwmPeriod);
	motorDriver1.setPwmBperiod(fPwmPeriod);
	motorDriver2.setPwmAperiod(fPwmPeriod);
	motorDriver2.setPwmBperiod(fPwmPeriod);
	
//	m_bldcArray[0].period_us(20000);				// 20000 us = 50 Hz, 2040.8us = 490 Hz, 83.3 us = 12 Khz	
//	m_bldcArray[1].period_us(20000);
//	m_bldcArray[2].period_us(20000);
//	m_bldcArray[3].period_us(20000);
#endif
	
	MotorController_setMotor(MOTOR_ALL, 0, BWD);
}

bool MotorController_isArmed()
{
	return _motorsArmed;
}

void MotorController_armESCs()
{
	for (int i = 1050; i < 2000; i += 50)
	{		
		MotorController_setMotor(MOTOR_ALL, i, FWD);
		wait_ms(10);
	}
	
	for (int i = 1950; i >= 1000; i -= 50)
	{
		MotorController_setMotor(MOTOR_ALL, i, FWD);
		wait_ms(10);
	}
	
	wait_ms(4000);
}

void MotorController_callibrateESCs()
{
	for (int i = 1050; i <= 2000; i+=50)
	{		
		MotorController_setMotor(MOTOR_D, i, FWD);
		wait_ms(10);
	}
	
	MotorController_setMotor(MOTOR_D, 2000, FWD);
	wait_ms(4000);
	
	for (int i = 1950; i >= 1000; i-=50)
	{
		MotorController_setMotor(MOTOR_D, i, FWD);
		wait_ms(10);
	}
	
	wait_ms(4000);
}

void MotorController_setMotor(uint8_t motorIndxMask, float power, direction_t dir)
{		
#ifdef MOTOR_STSPIN	
	MotorController_setMotors_STSPIN(motorIndxMask, power, dir);
#elif defined(MOTOR_ESC)
	
#elif defined(MOTOR_TOSHIBA)
	MotorController_setMotors_TB6612(motorIndxMask, power, dir);	
	//MotorController_setServos(motorIndxMask, power, dir);
#endif
}

#ifdef MOTOR_STSPIN
void MotorController_setMotors_STSPIN(uint8_t motorIndxMask, float power, direction_t dir)
{
	if (motorIndxMask & MOTOR_A)
		STSpinSetMotor(0, 0, power, dir);
	
	if (motorIndxMask & MOTOR_B)
		STSpinSetMotor(0, 1, power, dir);
	
	if (motorIndxMask & MOTOR_C)
		STSpinSetMotor(1, 0, power, dir);
	
	if (motorIndxMask & MOTOR_D)
		STSpinSetMotor(1, 1, power, dir);
}

#elif defined(MOTOR_ESC)

#elif defined(MOTOR_TOSHIBA)
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