#include "Servo.h"
#include "stm32f4xx_hal_conf.h"
#include "constants.h"
#include "error.h"


static TIM_HandleTypeDef servoTimHandle;
float pwmLow = 700;
float pwmHigh = 2300;

static void ServoTimerInit();

void Servo_Init()
{
	memset(&servoTimHandle, 0, sizeof(TIM_HandleTypeDef));
	
	ServoTimerInit();
	Servo_SetDutyCycle(SERVO_CHANNEL_1, 0.5f);	// Turn motor(s) off by default
}

void ServoTimerInit()
{	
	/*
	Timer Freqency = Period x PWM Frequency
	               = 20,000 x 50 Hz = 1 MHz
	
	Prescaler = Clock Speed / Timer Frequency 
			  = 100 MHz / 1 MHz
			  = 100
	*/
	
	float period = 0.02f;			// frequency of 50 Hz gives a period of 1 / 50 Hz = 0.02
	int us = period * 1000000.0f;	// convert seconds to microseconds
	
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t scc = SystemCoreClock;
	
	GPIO_InitStruct.Pin = SERVO1_PWM_Pin | SERVO2_PWM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(SERVO_GPIO_Port, &GPIO_InitStruct);
	
	servoTimHandle.Instance = TIM_SERVO;
	servoTimHandle.Init.Prescaler = 100;
	servoTimHandle.Init.Period = (us - 1);
	servoTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	servoTimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
	
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&servoTimHandle, &sClockSourceConfig);
	
	//__HAL_TIM_DISABLE(&servoTimHandle);

	if (HAL_TIM_PWM_Init(&servoTimHandle) != HAL_OK) {
		Error_Handler();
	}
	
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&servoTimHandle, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;	
	if (HAL_TIM_PWM_ConfigChannel(&servoTimHandle, &sConfigOC, SERVO_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	
	HAL_TIM_PWM_Start(&servoTimHandle, SERVO_CHANNEL_1);
//	HAL_TIM_PWM_Start(&servoTimHandle, TIM_CHANNEL_4);		// mbed: HAL_TIMEx_PWMN_Start(&servoTimHandle, TIM_CHANNEL_4);
	
	//__HAL_TIM_ENABLE(&servoTimHandle);
}

/*
Servo_SetPwmPulsewidth
channel - The channel of the timer to set
dutyCycle - A value between 0 and 1 where 0 is full CW, 1 is full CCW and 0.5 is stop
*/
void Servo_SetDutyCycle(int channel, float dutyCycle)
{
	if (dutyCycle < 0)
		dutyCycle = 0;	
	else if (dutyCycle > 1)
		dutyCycle = 1;
	
	switch (channel)
	{
		case SERVO_CHANNEL_1:
		//case SERVO_CHANNEL_2:
			break;
		
		default:
			PRINTF("Invalid servo channel: %d\r\n", channel);
			return;
	}
	
	float range = pwmHigh - pwmLow;
	float pwm = pwmLow + (range * dutyCycle);
	__HAL_TIM_SET_COMPARE(&servoTimHandle, channel, pwm);
}