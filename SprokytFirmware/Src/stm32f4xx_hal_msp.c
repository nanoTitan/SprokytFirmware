/**
  ******************************************************************************
  * @file    stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    06-May-2016
  * @brief   This file contains the HAL System and Peripheral (PPP) MSP initialization
  *          and de-initialization functions.
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_nucleo_bluenrg.h"
#include "constants.h"

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup SensorDemo
 *  @{
 */
 
/** @defgroup STM32F4XX_HAL_MSP
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32F4XX_HAL_MSP_Private_Functions
  * @{
  */

void HAL_MspInit(void)
{
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	/* BusFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	/* UsageFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

/**
 * @brief  This function is used for low level initialization of the SPI 
 *         communication with the BlueNRG Expansion Board.
 * @param  hspi: SPI handle.
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(hspi->Instance==BNRG_SPI_INSTANCE)
	{	
		/* Enable peripherals clock */

		/* Enable GPIO Ports Clock */  
		BNRG_SPI_RESET_CLK_ENABLE();
		BNRG_SPI_SCLK_CLK_ENABLE();
		BNRG_SPI_MISO_CLK_ENABLE();
		BNRG_SPI_MOSI_CLK_ENABLE();
		BNRG_SPI_CS_CLK_ENABLE();
		BNRG_SPI_IRQ_CLK_ENABLE();

		/* Enable SPI clock */
		BNRG_SPI_CLK_ENABLE();

		/* Reset */
		GPIO_InitStruct.Pin = BNRG_SPI_RESET_PIN;
		GPIO_InitStruct.Mode = BNRG_SPI_RESET_MODE;
		GPIO_InitStruct.Pull = BNRG_SPI_RESET_PULL;
		GPIO_InitStruct.Speed = BNRG_SPI_RESET_SPEED;
		GPIO_InitStruct.Alternate = BNRG_SPI_RESET_ALTERNATE;
		HAL_GPIO_Init(BNRG_SPI_RESET_PORT, &GPIO_InitStruct);	
		HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET);	/*Added to avoid spurious interrupt from the BlueNRG */

		/* SCLK */
		GPIO_InitStruct.Pin = BNRG_SPI_SCLK_PIN;
		GPIO_InitStruct.Mode = BNRG_SPI_SCLK_MODE;
		GPIO_InitStruct.Pull = BNRG_SPI_SCLK_PULL;
		GPIO_InitStruct.Speed = BNRG_SPI_SCLK_SPEED;
		GPIO_InitStruct.Alternate = BNRG_SPI_SCLK_ALTERNATE;
		HAL_GPIO_Init(BNRG_SPI_SCLK_PORT, &GPIO_InitStruct); 

		/* MISO */
		GPIO_InitStruct.Pin = BNRG_SPI_MISO_PIN;
		GPIO_InitStruct.Mode = BNRG_SPI_MISO_MODE;
		GPIO_InitStruct.Pull = BNRG_SPI_MISO_PULL;
		GPIO_InitStruct.Speed = BNRG_SPI_MISO_SPEED;
		GPIO_InitStruct.Alternate = BNRG_SPI_MISO_ALTERNATE;
		HAL_GPIO_Init(BNRG_SPI_MISO_PORT, &GPIO_InitStruct);

		/* MOSI */
		GPIO_InitStruct.Pin = BNRG_SPI_MOSI_PIN;
		GPIO_InitStruct.Mode = BNRG_SPI_MOSI_MODE;
		GPIO_InitStruct.Pull = BNRG_SPI_MOSI_PULL;
		GPIO_InitStruct.Speed = BNRG_SPI_MOSI_SPEED;
		GPIO_InitStruct.Alternate = BNRG_SPI_MOSI_ALTERNATE;
		HAL_GPIO_Init(BNRG_SPI_MOSI_PORT, &GPIO_InitStruct);

		/* NSS/CSN/CS */
		GPIO_InitStruct.Pin = BNRG_SPI_CS_PIN;
		GPIO_InitStruct.Mode = BNRG_SPI_CS_MODE;
		GPIO_InitStruct.Pull = BNRG_SPI_CS_PULL;
		GPIO_InitStruct.Speed = BNRG_SPI_CS_SPEED;
		GPIO_InitStruct.Alternate = BNRG_SPI_CS_ALTERNATE;
		HAL_GPIO_Init(BNRG_SPI_CS_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);

		/* IRQ -- INPUT */
		GPIO_InitStruct.Pin = BNRG_SPI_IRQ_PIN;
		GPIO_InitStruct.Mode = BNRG_SPI_IRQ_MODE;
		GPIO_InitStruct.Pull = BNRG_SPI_IRQ_PULL;
		GPIO_InitStruct.Speed = BNRG_SPI_IRQ_SPEED;
		GPIO_InitStruct.Alternate = BNRG_SPI_IRQ_ALTERNATE;
		HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStruct);

		/* Configure the NVIC for SPI */  
		HAL_NVIC_SetPriority(BNRG_SPI_EXTI_IRQn, 3, 0);    
		HAL_NVIC_EnableIRQ(BNRG_SPI_EXTI_IRQn);
	}
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

	if (htim_pwm->Instance == TIM1)
	{
	/* USER CODE BEGIN TIM1_MspInit 0 */

	  /* USER CODE END TIM1_MspInit 0 */
	    /* Peripheral clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();
	  /* USER CODE BEGIN TIM1_MspInit 1 */

	    /* USER CODE END TIM1_MspInit 1 */
	}
	else if (htim_pwm->Instance == TIM2)
	{
	/* USER CODE BEGIN TIM2_MspInit 0 */

	  /* USER CODE END TIM2_MspInit 0 */
	    /* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();
	  /* USER CODE BEGIN TIM2_MspInit 1 */

	    /* USER CODE END TIM2_MspInit 1 */
	}
	else if (htim_pwm->Instance == TIM3)
	{
		/* USER CODE BEGIN TIM3_MspInit 0 */

		/* USER CODE END TIM3_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
		
		/* USER CODE BEGIN TIM3_MspInit 1 */

		/* USER CODE END TIM3_MspInit 1 */
	}
	else if (htim_pwm->Instance == TIM4)
	{
	/* USER CODE BEGIN TIM4_MspInit 0 */

	  /* USER CODE END TIM4_MspInit 0 */
	    /* Peripheral clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();
	  /* USER CODE BEGIN TIM4_MspInit 1 */

	    /* USER CODE END TIM4_MspInit 1 */
	}

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (htim->Instance == TIM1)
	{
	/* USER CODE BEGIN TIM1_MspPostInit 0 */

	  /* USER CODE END TIM1_MspPostInit 0 */
	    /**TIM1 GPIO Configuration    
	    PA8     ------> TIM1_CH1 
	    */
		GPIO_InitStruct.Pin = MD2_PWMB_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(MD2_PWMB_GPIO_Port, &GPIO_InitStruct);

		  /* USER CODE BEGIN TIM1_MspPostInit 1 */

		    /* USER CODE END TIM1_MspPostInit 1 */
	}
	else if (htim->Instance == TIM2)
	{
	/* USER CODE BEGIN TIM2_MspPostInit 0 */

	  /* USER CODE END TIM2_MspPostInit 0 */
  
	    /**TIM2 GPIO Configuration    
	    PA0-WKUP     ------> TIM2_CH1
	    PB10     ------> TIM2_CH3 
	    */
		GPIO_InitStruct.Pin = SERVO_PWM1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(SERVO_PWM1_GPIO_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = SERVO_PWM2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(SERVO_PWM2_GPIO_Port, &GPIO_InitStruct);

		  /* USER CODE BEGIN TIM2_MspPostInit 1 */

		    /* USER CODE END TIM2_MspPostInit 1 */
	}
	else if (htim->Instance == TIM3)
	{
		/* USER CODE BEGIN TIM3_MspPostInit 0 */

		/* USER CODE END TIM3_MspPostInit 0 */
  
	    /**TIM3 GPIO Configuration    
	    PA6     ------> TIM3_CH1
	    PB0     ------> TIM3_CH3
	    PB1     ------> TIM3_CH4
	    PB5     ------> TIM3_CH2 
	    */
		GPIO_InitStruct.Pin = MD2_PWMA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(MD2_PWMA_GPIO_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = MD1_PWMB_Pin | MD1_PWMA_Pin | IMU_PWM_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM3_MspPostInit 1 */

		/* USER CODE END TIM3_MspPostInit 1 */
	}
	else if (htim->Instance == TIM4)
	{
	/* USER CODE BEGIN TIM4_MspPostInit 0 */

	  /* USER CODE END TIM4_MspPostInit 0 */
  
	    /**TIM4 GPIO Configuration    
	    PB6     ------> TIM4_CH1
	    PB7     ------> TIM4_CH2 
	    */
		GPIO_InitStruct.Pin = SERVO_PWM4_Pin | SERVO_PWM3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		  /* USER CODE BEGIN TIM4_MspPostInit 1 */

		    /* USER CODE END TIM4_MspPostInit 1 */
	}

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

	if (htim_pwm->Instance == TIM1)
	{
	/* USER CODE BEGIN TIM1_MspDeInit 0 */

	  /* USER CODE END TIM1_MspDeInit 0 */
	    /* Peripheral clock disable */
		__HAL_RCC_TIM1_CLK_DISABLE();
	  /* USER CODE BEGIN TIM1_MspDeInit 1 */

	    /* USER CODE END TIM1_MspDeInit 1 */
	}
	else if (htim_pwm->Instance == TIM2)
	{
	/* USER CODE BEGIN TIM2_MspDeInit 0 */

	  /* USER CODE END TIM2_MspDeInit 0 */
	    /* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();
	  /* USER CODE BEGIN TIM2_MspDeInit 1 */

	    /* USER CODE END TIM2_MspDeInit 1 */
	}
	else if (htim_pwm->Instance == TIM3)
	{
	/* USER CODE BEGIN TIM3_MspDeInit 0 */

	  /* USER CODE END TIM3_MspDeInit 0 */
	    /* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();
	  /* USER CODE BEGIN TIM3_MspDeInit 1 */

	    /* USER CODE END TIM3_MspDeInit 1 */
	}
	else if (htim_pwm->Instance == TIM4)
	{
	/* USER CODE BEGIN TIM4_MspDeInit 0 */

	  /* USER CODE END TIM4_MspDeInit 0 */
	    /* Peripheral clock disable */
		__HAL_RCC_TIM4_CLK_DISABLE();
	  /* USER CODE BEGIN TIM4_MspDeInit 1 */

	    /* USER CODE END TIM4_MspDeInit 1 */
	}

}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
