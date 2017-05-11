#include "TB6612FNG.h"
#include "hw_init.h"
#include "stm32f4xx_hal_conf.h"

#define SIGNAL_HIGH     (1)
#define SIGNAL_LOW      (0)

#define TB6612FNG_PWM_PERIOD_DEFAULT      (0.00002)   // 50KHz
#define TB6612FNG_PWM_PULSEWIDTH_DEFAULT  (0.50)      // 50% duty cycle

#define TB_TIMER TIMER_3

// Private variables
uint16_t maxDutyCycle = 256;
uint16_t pwmFrequency = 10000;

// Private function declarations


void TB_Init()
{
//	PinName pinPwmA,
//	PinName pinAin1,
//	PinName pinAin2,
//	PinName pinPwmB,
//	PinName pinBin1,
//	PinName pinBin2,
//	PinName pinNStby)
//	: pwmA(pinPwmA)
//	, Ain1(pinAin1)
//	, Ain2(pinAin2)
//	, pwmB(pinPwmB)
//	, Bin1(pinBin1)
//	, Bin2(pinBin2)
//	, nStby(pinNStby)
//		
//		
//	Ain1 = SIGNAL_LOW;
//	Ain2 = SIGNAL_LOW;
//	Bin1 = SIGNAL_LOW;
//	Bin2 = SIGNAL_LOW;
//	pwmA.period(TB6612FNG_PWM_PERIOD_DEFAULT);
//	pwmA = TB6612FNG_PWM_PULSEWIDTH_DEFAULT;
//	pwmB.period(TB6612FNG_PWM_PERIOD_DEFAULT);
//	pwmB = TB6612FNG_PWM_PULSEWIDTH_DEFAULT;
//	nStby = SIGNAL_LOW;
}

void TB_SetPwm(int iMotorChannel, uint16_t fFrequency, float fPulsewidth)
{
	TB_SetPwmFrequency(fFrequency);
	TB_SetPwmPulsewidth(iMotorChannel, fPulsewidth);
}

void TB_SetPwmFrequency(uint16_t fFrequency)
{
	pwmFrequency = fFrequency;
	HW_PWMSetFrequency(TB_TIMER, pwmFrequency);
}

void TB_SetPwmPulsewidth(int iMotorChannel, float fPulsewidth)
{
	if (fPulsewidth < 0)
		fPulsewidth = 0;	
	else if (fPulsewidth > 1)
		fPulsewidth = 1;
	
	int compare = pwmFrequency * maxDutyCycle * fPulsewidth;
	int channel = TIM_CHANNEL_1;
	
	switch (iMotorChannel)
	{
	case MOTOR_CHANNEL_A1:
		channel = TIM_CHANNEL_1;
		break;
		
	case MOTOR_CHANNEL_B1:
		channel = TIM_CHANNEL_2;
		break;
		
	case MOTOR_CHANNEL_A2:
		channel = TIM_CHANNEL_3;
		break;
		
	case MOTOR_CHANNEL_B2:
		channel = TIM_CHANNEL_4;
		break;
		
	default:
		return;
	}
	
	HW_PWMSetCompare(TB_TIMER, channel, compare);
}

void TB_Standby(int iMotorChannel)
{
	
}

void TB_MotorStop(int iMotorChannel)
{
	
}

void TB_MotorCCW(int iMotorChannel)
{
	
}

void TB_MotorCW(int iMotorChannel)
{
	
}

 
 