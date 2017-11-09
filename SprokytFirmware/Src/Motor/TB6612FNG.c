#include "TB6612FNG.h"
#include "stm32f4xx_hal_conf.h"
#include "constants.h"
#include "error.h"

#define SIGNAL_HIGH     (1)
#define SIGNAL_LOW      (0)

#define TB6612FNG_PWM_PERIOD_DEFAULT      (0.00002)   // 50KHz
#define TB6612FNG_PWM_PULSEWIDTH_DEFAULT  (0.50)      // 50% duty cycle

// Private variables
static TIM_HandleTypeDef hMdTim1;
static TIM_HandleTypeDef htim3;
uint16_t maxDutyCycle = 256;
uint16_t pwmFrequency = 10000;
int period = 0;
uint32_t prescaler = 1;

// Private function declarations
static void TB_Init_GPIO();
static void TB_INIT_PWM();
static void TB_PWM_OutWrite();


void TB_Init()
{
	TB_Init_GPIO();
	TB_INIT_PWM();
}

void TB_Init_GPIO()
{	
	GPIO_InitTypeDef GPIO_InitStruct;
	
#ifdef MOTOR_1_ENABLED
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*Configure GPIO pins : PWMA, PWMB */
	GPIO_InitStruct.Pin = MD1_PWMA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = MD1_PWMA_AF;
	HAL_GPIO_Init(MD1_PWMA_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MD1_PWMB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = MD1_PWMB_AF;
	HAL_GPIO_Init(MD1_PWMB_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : AIN1, BIN1, AIN2 BIN2 */
	GPIO_InitStruct.Pin = MD1_AIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MD1_AIN1_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MD1_AIN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MD1_AIN2_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MD1_BIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MD1_BIN1_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = MD1_BIN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MD1_BIN2_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pins : STBY */
	GPIO_InitStruct.Pin = MD1_STBY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MD1_STBY_GPIO_Port, &GPIO_InitStruct);
	
#endif // MOTOR_1_ENABLED
	
#ifdef MOTOR_2_ENABLED
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	/*Configure GPIO pins : PWMA, PWMB */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pins : AIN1, BIN1, AIN2 BIN2 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pins : STBY */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#endif // MOTOR_2_ENABLED
}

void TB_INIT_PWM()
{
#ifdef MOTOR_1_ENABLED
	MD1_RCC_CL_ENABLE();
#endif
	
#ifdef MOTOR_2_ENABLED
	MD2_RCC_CL_ENABLE();
#endif
	
	pwmFrequency = 10000;
	TB_SetPwmFrequency(0.00002f);      // 50KHz default
}

void TB_PWM_OutWrite()
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	
#ifdef MOTOR_1_ENABLED
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&hMdTim1, &sMasterConfig) != HAL_OK)
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
	
	if (HAL_TIM_PWM_ConfigChannel(&hMdTim1, &sConfigOC, MD1_CHANNEL_A) != HAL_OK)
	{
		Error_Handler();
	}
	
	if (HAL_TIM_PWM_ConfigChannel(&hMdTim1, &sConfigOC, MD1_CHANNEL_B) != HAL_OK)
	{
		Error_Handler();
	}
	
	HAL_TIM_PWM_Start(&hMdTim1, MD1_CHANNEL_A);
	HAL_TIM_PWM_Start(&hMdTim1, MD1_CHANNEL_B);
	
#endif	// MOTOR_1_ENABLED
	
#ifdef MOTOR_2_ENABLED
	TODO
#endif	// MOTOR_1_ENABLED
}

void TB_SetPwm(int iMotorChannel, uint16_t fFrequency, float fPulsewidth)
{
	TB_SetPwmFrequency(fFrequency);
	TB_SetPwmPulsewidth(iMotorChannel, fPulsewidth);
}

void TB_SetPwmFrequency(float seconds)
{		
#ifdef MOTOR_1_ENABLED
	int us = seconds * 1000000.0f;
	
	hMdTim1.Instance = MD1_TIM;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	uint32_t PclkFreq = 0;
	uint32_t APBxCLKDivider = RCC_HCLK_DIV1;
	float dc = 0;
	uint8_t i = 0;

	__HAL_TIM_DISABLE(&hMdTim1);

	// Get clock configuration
	// Note: PclkFreq contains here the Latency (not used after)
	//HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);

	//PclkFreq = HAL_RCC_GetPCLK2Freq();
	//APBxCLKDivider = RCC_ClkInitStruct.APB2CLKDivider;

	hMdTim1.Init.Prescaler = (((SystemCoreClock) / 1000000)) - 1; // 1 us tick    // PclkFreq
	hMdTim1.Init.Period = (us - 1);
	hMdTim1.Init.ClockDivision = 0;
	hMdTim1.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if (HAL_TIM_PWM_Init(&hMdTim1) != HAL_OK) {
		Error_Handler();
	}

	// Save for future use
	period = us;
	
	// Set duty cycle again
	TB_PWM_OutWrite();

	__HAL_TIM_ENABLE(&hMdTim1);
	
	//uint32_t autoReload = (SystemCoreClock + pwmFrequency / 2) / pwmFrequency - 1;
	//__HAL_TIM_SET_AUTORELOAD(&htim3, autoReload);
	
#endif // MOTOR_1_ENABLED
	
#ifdef MOTOR_2_ENABLED
	TODO
#endif	// MOTOR_1_ENABLED
}

void TB_SetPwmPulsewidth(int tb_channel, float fPulsewidth)
{
	if (fPulsewidth < 0)
		fPulsewidth = 0;	
	else if (fPulsewidth > 1)
		fPulsewidth = 1;
	
	int compare = (period * fPulsewidth) / prescaler;
	
#ifdef MOTOR_1_ENABLED
	if (tb_channel == MD1_CHANNEL_A || tb_channel == MD1_CHANNEL_B)
		__HAL_TIM_SET_COMPARE(&hMdTim1, tb_channel, compare);
#endif // MOTOR_1_ENABLED
	
#ifdef MOTOR_2_ENABLED
	if (tb_channel == MD2_CHANNEL_A || tb_channel == MD2_CHANNEL_B)
		__HAL_TIM_SET_COMPARE(&hMdTim2, tb_channel, compare);
#endif // MOTOR_2_ENABLED
}

void TB_1_Standy()
{
	HAL_GPIO_WritePin(MD1_STBY_GPIO_Port, MD1_STBY_Pin, GPIO_PIN_RESET);
}

void TB_1A_Stop()
{
	HAL_GPIO_WritePin(MD1_AIN1_GPIO_Port, MD1_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_AIN2_GPIO_Port, MD1_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_1A_MotorCW()
{
	HAL_GPIO_WritePin(MD1_AIN1_GPIO_Port, MD1_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_AIN2_GPIO_Port, MD1_AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_STBY_GPIO_Port, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_1A_MotorCCW()
{
	HAL_GPIO_WritePin(MD1_AIN1_GPIO_Port, MD1_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_AIN2_GPIO_Port, MD1_AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_STBY_GPIO_Port, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_1B_Stop()
{
	HAL_GPIO_WritePin(MD1_BIN1_GPIO_Port, MD1_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_BIN2_GPIO_Port, MD1_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_1B_MotorCW()
{
	HAL_GPIO_WritePin(MD1_BIN1_GPIO_Port, MD1_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_BIN2_GPIO_Port, MD1_BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_STBY_GPIO_Port, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_1B_MotorCCW()
{
	HAL_GPIO_WritePin(MD1_BIN1_GPIO_Port, MD1_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_BIN2_GPIO_Port, MD1_BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_STBY_GPIO_Port, MD1_STBY_Pin, GPIO_PIN_SET);
}

void TB_2_Standy()
{
	HAL_GPIO_WritePin(MD2_STBY_GPIO_Port, MD2_STBY_Pin, GPIO_PIN_RESET);
}

void TB_2A_Stop()
{
	HAL_GPIO_WritePin(MD2_AIN1_GPIO_Port, MD2_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_AIN2_GPIO_Port, MD2_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_2A_MotorCW()
{
	HAL_GPIO_WritePin(MD2_AIN1_GPIO_Port, MD2_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_AIN2_GPIO_Port, MD2_AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_STBY_GPIO_Port, MD2_STBY_Pin, GPIO_PIN_SET);
}					  

void TB_2A_MotorCCW()
{
	HAL_GPIO_WritePin(MD2_AIN1_GPIO_Port, MD2_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_AIN2_GPIO_Port, MD2_AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_STBY_GPIO_Port, MD2_STBY_Pin, GPIO_PIN_SET);
}				  

void TB_2B_Stop()
{
	HAL_GPIO_WritePin(MD2_BIN1_GPIO_Port, MD2_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_BIN2_GPIO_Port, MD2_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_2B_MotorCW()
{
	HAL_GPIO_WritePin(MD2_BIN1_GPIO_Port, MD2_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_BIN2_GPIO_Port, MD2_BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_STBY_GPIO_Port, MD2_STBY_Pin, GPIO_PIN_SET);
}

void TB_2B_MotorCCW()
{
	HAL_GPIO_WritePin(MD2_BIN1_GPIO_Port, MD2_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_BIN2_GPIO_Port, MD2_BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_STBY_GPIO_Port, MD2_STBY_Pin, GPIO_PIN_SET);
}

void TB_SetWorkMode(int tb_channel, int tb_control_mode)
{
	static void( *TB_ControlModeFunctionArray[][4] )(void) = 
	{
		{ TB_1_Standy, TB_1A_MotorCW, TB_1A_MotorCCW, TB_1A_Stop },
		{ TB_1_Standy, TB_1B_MotorCCW, TB_1B_MotorCW, TB_1B_Stop },
		{ TB_2_Standy, TB_2A_MotorCW, TB_2A_MotorCCW, TB_2A_Stop },
		{ TB_2_Standy, TB_2B_MotorCCW, TB_2B_MotorCW, TB_2B_Stop },
	};
	
	if (tb_channel < 0 || tb_channel >= TB_CHANNEL_COUNT)
		return;
	
	if (tb_control_mode < 0 || tb_control_mode >= TB_CONTROL_MODE_COUNT)
		return;
	
	TB_ControlModeFunctionArray[tb_channel][tb_control_mode]();
}
