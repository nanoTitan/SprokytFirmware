#include "TB6612FNG.h"
#include "stm32f4xx_hal_conf.h"
#include "constants.h"
#include "error.h"

#define SIGNAL_HIGH     (1)
#define SIGNAL_LOW      (0)

#define TB6612FNG_PWM_PERIOD_DEFAULT      (0.00002)   // 50KHz
#define TB6612FNG_PWM_PULSEWIDTH_DEFAULT  (0.50)      // 50% duty cycle

// Private variables
TIM_HandleTypeDef htim3;
uint16_t maxDutyCycle = 256;
uint16_t pwmFrequency = 10000;

// Private function declarations
static void TB_Init_GPIO();
static void TB_INIT_PWM();


void TB_Init()
{
	TB_INIT_PWM();
	TB_Init_GPIO();
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);		// mbed: HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);		// mbed: HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_4);
}

void TB_Init_GPIO()
{	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	// PWM GPIO Pins
	
	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.Pin = MD2_PWMA_Pin | MD2_PWMB_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//	HAL_GPIO_Init(MD2_PWMA_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MD1_PWMB_Pin | MD1_PWMA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// AIN, BIN, and STANDBY pins
	
	/*Configure GPIO pins : MD2_STBY_Pin MD2_BIN1_Pin MD2_BIN2_Pin MD1_AIN2_Pin */
	GPIO_InitStruct.Pin = /*MD2_STBY_Pin | MD2_BIN1_Pin | MD2_BIN2_Pin | */ MD1_AIN2_Pin;	// ESP_RESET_Pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/*Configure GPIO pins : MD1_STBY_Pin */
	GPIO_InitStruct.Pin = MD1_STBY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : MD1_AIN1_Pin MD1_BIN1_Pin MD1_BIN2_Pin MD2_AIN2_Pin MD2_AIN1_Pin */
	GPIO_InitStruct.Pin = MD1_AIN1_Pin | MD1_BIN1_Pin | MD1_BIN2_Pin /* | MD2_AIN2_Pin | MD2_AIN1_Pin */;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void TB_INIT_PWM()
{
	__HAL_RCC_TIM1_CLK_ENABLE();						// mbed
	__HAL_RCC_TIM3_CLK_ENABLE();
	
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	pwmFrequency = 10000;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;															// mbed: 95
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = (SystemCoreClock + pwmFrequency / 2) / pwmFrequency - 1;		// mbed: 19  (20 - 1)
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;															// 10 (50% duty cycle of 20us)
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;	
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	
	__HAL_TIM_ENABLE(&htim3);	
}

void TB_SetPwm(int iMotorChannel, uint16_t fFrequency, float fPulsewidth)
{
	TB_SetPwmFrequency(fFrequency);
	TB_SetPwmPulsewidth(iMotorChannel, fPulsewidth);
}

void TB_SetPwmFrequency(uint16_t frequency)
{		
	pwmFrequency = frequency;
	if (pwmFrequency < 1000)
		pwmFrequency  = 1000;
	else if (pwmFrequency > 50000)
		pwmFrequency = 50000;
	
	uint32_t autoReload = (SystemCoreClock + pwmFrequency / 2) / pwmFrequency - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim3, autoReload);
}

void TB_SetPwmPulsewidth(int tb_channel, float fPulsewidth)
{
	if (fPulsewidth < 0)
		fPulsewidth = 0;	
	else if (fPulsewidth > 1)
		fPulsewidth = 1;
	
	int compare = pwmFrequency * maxDutyCycle * fPulsewidth;
	int channel = TIM_CHANNEL_1;
	
	switch (tb_channel)
	{
	case TB_CHANNEL_A1:
		channel = TIM_CHANNEL_4;
		break;
		
	case TB_CHANNEL_B1:
		channel = TIM_CHANNEL_3;
		break;
		
	case TB_CHANNEL_A2:
		channel = TIM_CHANNEL_1;
		break;
		
	case TB_CHANNEL_B2:
		channel = TIM_CHANNEL_2;
		break;
		
	default:
		return;
	}

	__HAL_TIM_SET_COMPARE(&htim3, channel, compare);
}

void TB_1A_Stop()
{
	HAL_GPIO_WritePin(GPIOB, MD1_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MD1_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_1A_MotorCW()
{
	HAL_GPIO_WritePin(GPIOB, MD1_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, MD1_AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_1A_MotorCCW()
{
	HAL_GPIO_WritePin(GPIOB, MD1_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MD1_AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_1B_Stop()
{
	HAL_GPIO_WritePin(GPIOB, MD1_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, MD1_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_1B_MotorCW()
{
	HAL_GPIO_WritePin(GPIOB, MD1_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, MD1_BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_1B_MotorCCW()
{
	HAL_GPIO_WritePin(GPIOB, MD1_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, MD1_BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_2A_Stop()
{
	HAL_GPIO_WritePin(GPIOB, MD2_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, MD2_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_2A_MotorCW()
{
	HAL_GPIO_WritePin(GPIOB, MD2_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, MD2_AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MD2_STBY_Pin, GPIO_PIN_SET);
}					  

void TB_2A_MotorCCW()
{
	HAL_GPIO_WritePin(GPIOB, MD2_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, MD2_AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, MD2_STBY_Pin, GPIO_PIN_SET);
}				  

void TB_2B_Stop()
{
	HAL_GPIO_WritePin(GPIOC, MD2_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MD2_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_2B_MotorCW()
{
	HAL_GPIO_WritePin(GPIOC, MD2_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, MD2_BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MD2_STBY_Pin, GPIO_PIN_SET);
}

void TB_2B_MotorCCW()
{
	HAL_GPIO_WritePin(GPIOC, MD2_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MD2_BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, MD2_STBY_Pin, GPIO_PIN_SET);
}

void TB_SetWorkMode(int tb_channel, int tb_control_mode)
{
	static void( *TB_ControlModeFunctionArray[][3] )(void) = 
	{
		{ TB_1A_MotorCW, TB_1A_MotorCCW, TB_1A_Stop },
		{ TB_1B_MotorCCW, TB_1B_MotorCW, TB_1B_Stop },
		{ TB_2A_MotorCW, TB_2A_MotorCCW, TB_2A_Stop },
		{ TB_2B_MotorCCW, TB_2B_MotorCW, TB_2B_Stop },
	};
	
	if (tb_channel < 0 || tb_channel >= TB_CHANNEL_COUNT)
		return;
	
	if (tb_control_mode < 0 || tb_control_mode >= TB_CONTROL_MODE_COUNT)
		return;
	
	TB_ControlModeFunctionArray[tb_channel][tb_control_mode]();
}
