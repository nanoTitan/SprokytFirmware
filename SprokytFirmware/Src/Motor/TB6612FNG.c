#include "TB6612FNG.h"

#define SIGNAL_HIGH     (1)
#define SIGNAL_LOW      (0)

#define TB6612FNG_PWM_PERIOD_DEFAULT      (0.00002)   // 50KHz
#define TB6612FNG_PWM_PULSEWIDTH_DEFAULT  (0.50)      // 50% duty cycle

// Private variables
PwmOut pwmA;
DigitalOut Ain1;
DigitalOut Ain2;
PwmOut pwmB;
DigitalOut Bin1;
DigitalOut Bin2;
DigitalOut nStby;

// Private function declarations




void TB6612FNG_Init()
{
	PinName pinPwmA,
	PinName pinAin1,
	PinName pinAin2,
	PinName pinPwmB,
	PinName pinBin1,
	PinName pinBin2,
	PinName pinNStby)
	: pwmA(pinPwmA)
	, Ain1(pinAin1)
	, Ain2(pinAin2)
	, pwmB(pinPwmB)
	, Bin1(pinBin1)
	, Bin2(pinBin2)
	, nStby(pinNStby)
		
		
	Ain1 = SIGNAL_LOW;
	Ain2 = SIGNAL_LOW;
	Bin1 = SIGNAL_LOW;
	Bin2 = SIGNAL_LOW;
	pwmA.period(TB6612FNG_PWM_PERIOD_DEFAULT);
	pwmA = TB6612FNG_PWM_PULSEWIDTH_DEFAULT;
	pwmB.period(TB6612FNG_PWM_PERIOD_DEFAULT);
	pwmB = TB6612FNG_PWM_PULSEWIDTH_DEFAULT;
	nStby = SIGNAL_LOW;
}

void setPwmA(float fPeriod, float fPulsewidth)
{
	pwmA.period(fPeriod);
	pwmA = fPulsewidth;
}
 
void setPwmAperiod(float fPeriod)
{
	pwmA.period(fPeriod);
}
 
void setPwmApulsewidth(float fPulsewidth)
{
	pwmA = fPulsewidth;
}
 
void setPwmB(float fPeriod, float fPulsewidth)
{
	pwmB.period(fPeriod);
	pwmB = fPulsewidth;
}
 
void setPwmBperiod(float fPeriod)
{
	pwmB.period(fPeriod);
}
 
void setPwmBpulsewidth(float fPulsewidth)
{
	pwmB = fPulsewidth;
}
 
void standby(void)
{
	nStby = SIGNAL_LOW;
}
 
void motorA_stop(void)
{
	Ain1 = SIGNAL_LOW;
	Ain2 = SIGNAL_LOW;
}
 
void motorA_ccw(void)
{
	Ain1 = SIGNAL_LOW;
	Ain2 = SIGNAL_HIGH;
	nStby = SIGNAL_HIGH;
}
 
void motorA_cw(void)
{
	Ain1 = SIGNAL_HIGH;
	Ain2 = SIGNAL_LOW;
	nStby = SIGNAL_HIGH;
}
 
void motorB_stop(void)
{
	Bin1 = SIGNAL_LOW;
	Bin2 = SIGNAL_LOW;
}
 
void motorB_ccw(void)
{
	Bin1 = SIGNAL_LOW;
	Bin2 = SIGNAL_HIGH;
	nStby = SIGNAL_HIGH;
}
 
void motorB_cw(void)
{
	Bin1 = SIGNAL_HIGH;
	Bin2 = SIGNAL_LOW;
	nStby = SIGNAL_HIGH;
}
 
 