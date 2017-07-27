#include "Servo.h"
#include "stm32f4xx_hal_conf.h"
#include "constants.h"
#include "error.h"


static TIM_HandleTypeDef htim3;
int servo_period = 0;
uint32_t servo_prescaler = 1;
float pwmLow = 700;
float pwmHigh = 2300;

static void Servo_Init_GPIO();
static void Servo_INIT_PWM();
static void Servo_PWM_OutWrite();

void Servo_Init()
{
	Servo_Init_GPIO();
	Servo_INIT_PWM();
}

void Servo_Init_GPIO()
{	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	// PWM GPIO Pins
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = SERVO1_PWM_Pin | SERVO2_PWM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(SERVO_PWM_GPIO_Port, &GPIO_InitStruct);
	
//	GPIO_InitStruct.Pin = MD1_PWMB_Pin | MD1_PWMA_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	
//	// AIN, BIN, and STANDBY pins
//	
//	/*Configure GPIO pins : MD2_STBY_Pin MD2_BIN1_Pin MD2_BIN2_Pin MD1_AIN2_Pin */
//	GPIO_InitStruct.Pin = MD2_STBY_Pin | MD2_BIN1_Pin | MD2_BIN2_Pin | MD1_AIN2_Pin;	// ESP_RESET_Pin
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//	
//	/*Configure GPIO pins : MD1_STBY_Pin */
//	GPIO_InitStruct.Pin = MD1_STBY_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//	/*Configure GPIO pins : MD1_AIN1_Pin MD1_BIN1_Pin MD1_BIN2_Pin MD2_AIN2_Pin MD2_AIN1_Pin */
//	GPIO_InitStruct.Pin = MD1_AIN1_Pin | MD1_BIN1_Pin | MD1_BIN2_Pin | MD2_AIN2_Pin | MD2_AIN1_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Servo_INIT_PWM()
{
	//__HAL_RCC_TIM1_CLK_ENABLE();						// mbed
	//__HAL_RCC_TIM3_CLK_ENABLE();
	
	Servo_SetPwmFrequency();      // 50Hz default
}

void Servo_SetPwmFrequency()
{		
	float period = 0.02f;			// frequency of 50 Hz gives a period of 1 / 50 Hz = 0.02
	int us = period * 1000000.0f;	// convert seconds to microseconds
	
	/*
	Timer Freqency = Period x PWM Frequency
	               = 20,000 x 50 Hz = 1 MHz
	
	Prescaler = Clock Speed / Timer Frequency 
			  = 100 MHz / 1 MHz
			  = 100
	*/
	
	htim3.Instance = TIM3;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	uint32_t PclkFreq = 0;
	uint32_t APBxCLKDivider = RCC_HCLK_DIV1;
	float dc = 0;
	uint8_t i = 0;

	__HAL_TIM_DISABLE(&htim3);

	// Get clock configuration
	// Note: PclkFreq contains here the Latency (not used after)
	//HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);

	//PclkFreq = HAL_RCC_GetPCLK2Freq();
	//APBxCLKDivider = RCC_ClkInitStruct.APB2CLKDivider;

	htim3.Init.Prescaler = 100;
	htim3.Init.Period = (us - 1);
	htim3.Init.ClockDivision = 0;
	htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	// Save for future use
	servo_period = us;
	
	// Set duty cycle again
	Servo_PWM_OutWrite();

	__HAL_TIM_ENABLE(&htim3);
	
	//uint32_t autoReload = (SystemCoreClock + pwmFrequency / 2) / pwmFrequency - 1;
	//__HAL_TIM_SET_AUTORELOAD(&htim3, autoReload);
}

void Servo_PWM_OutWrite()
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

//	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//	{
//		Error_Handler();
//	}
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);		// mbed: HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);		// mbed: HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_4);
}

void Servo_SetPwm(int iMotorChannel, uint16_t fFrequency, float fPulsewidth)
{
	Servo_SetPwmFrequency(fFrequency);
	Servo_SetPwmPulsewidth(iMotorChannel, fPulsewidth);
}

/*
Servo_SetPwmPulsewidth
tb_channel - The channel of the timer to set
dutyCycle - A value between 0 and 1 where 0 is full CW, 1 is full CCW and 0.5 is stop
*/
void Servo_SetPwmPulsewidth(SERVO_CHANNEL channel, float dutyCycle)
{
	if (dutyCycle < 0)
		dutyCycle = 0;	
	else if (dutyCycle > 1)
		dutyCycle = 1;
	
	float range = pwmHigh - pwmLow;
	float pwm = pwmLow + (range * dutyCycle);
	__HAL_TIM_SET_COMPARE(&htim3, channel, pwm);
}