
/* Includes ------------------------------------------------------------------*/
#include "hw_init.h"
#include "stm32f4xx_hal_conf.h"

#include "constants.h"
#include "error.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


void Hardware_Init()
{
	MX_TIM3_Init();
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	GPIO_InitTypeDef GPIO_InitStruct;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0;
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
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
	
	HAL_TIM_MspPostInit(&htim3);
}

void HW_PWMSetCompare(int timer, int channel, int compare)
{
	TIM_HandleTypeDef* pTim = NULL;
	switch (timer)
	{
		case TIMER_3:
			pTim = &htim3;
			break;
		
		default:
			return;
	}
	
	if (channel < TIM_CHANNEL_1 || channel > TIM_CHANNEL_4)
		return;

	__HAL_TIM_SET_COMPARE(pTim, channel, compare);
}

void HW_PWMSetFrequency(int timer, uint16_t frequency)
{
	TIM_HandleTypeDef* pTim = NULL;
	switch (timer)
	{
	case TIMER_3:
		pTim = &htim3;
		break;
		
	default:
		return;
	}
	
	uint32_t autoReload = (SystemCoreClock + frequency / 2) / frequency - 1;
	__HAL_TIM_SET_AUTORELOAD(pTim, autoReload);
}