#include "stm32f4xx_hal_conf.h"
/*
 * HAL_TIM_Encoder_MspInit()
 * Overrides the __weak function stub in stm32f4xx_hal_tim.h
 *
 * Edit the below for your preferred pin wiring & pullup/down
 * I have encoder common at 3V3, using GPIO_PULLDOWN on inputs.
 * Encoder A&B outputs connected directly to GPIOs.
 *
 * www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00102166.pdf
 * www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00141306.pdf
 *
 * TIM1_CH1: AF1 @ PA_8, PE_9 
 * TIM1_CH2: AF1 @ PA_9, PE_11 
 *
 * TIM2_CH1: AF1 @ PA_0, PA_5, PA_15, PB_8*     *F446 only
 * TIM2_CH2: AF1 @ PA_1, PB_3, PB_9*            *F446 only
 *
 * TIM3_CH1: AF2 @ PA_6, PB_4, PC_6
 * TIM3_CH2: AF2 @ PA_7, PB_5, PC_7
 *
 * TIM4_CH1: AF2 @ PB_6, PD_12
 * TIM4_CH2: AF2 @ PB_7, PD_13
 *
 * TIM5_CH1: AF2 @ PA_0*    *TIM5 used by mbed system ticker so unavailable
 * TIM5_CH2: AF2 @ PA_1*
 *
 */

#if defined(STM32F411xE) || defined(STM32F401xE)
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (htim->Instance == TIM1) { //PA8 PA9 = Nucleo D7 D8
        __TIM1_CLK_ENABLE();
        __GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (htim->Instance == TIM2) { //PA0 PA1 = Nucleo A0 A1
        __TIM2_CLK_ENABLE();
        __GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;				// Orig: GPIO_PULLDOWN
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	    
	    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
	    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if (htim->Instance == TIM3) { //PB4 PB5 = Nucleo D5 D4
        __TIM3_CLK_ENABLE();
	    __GPIOC_CLK_ENABLE();							// Orig: __GPIOB_CLK_ENABLE			Alt: __GPIOC_CLK_ENABLE
	    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;	// Orig: GPIO_PIN_4 | GPIO_PIN_5	Alt: GPIO_PIN_6 | GPIO_PIN_7
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;				// Orig: GPIO_PULLDOWN
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);			// Orig: GPIOB						Alt: GPIOC
	    
	    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
	    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
    else if (htim->Instance == TIM4) { // PB6 PB7 = Nucleo D10 MORPHO_PB7
        __TIM4_CLK_ENABLE();
        __GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	    
	    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
	    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
	else if (htim->Instance == TIM5) { // PA1 PA0
		__TIM5_CLK_ENABLE();
		__GPIOA_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		HAL_NVIC_SetPriority(TIM5_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(TIM5_IRQn);
	}
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
	if (htim_encoder->Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_DISABLE();
	
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
	}
	else if (htim_encoder->Instance == TIM3)
	{
		__HAL_RCC_TIM3_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
	}
}
#endif