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
#include "stm32f4xx_nucleo_ihm06a1.h"
#include "main.h"
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
extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId);
static void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32F4XX_HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
#if defined(IMU_ENABLED)
	if (htim_base->Instance == TIM_IMU)
	{
		// IMU Sensor Fusion clock enable
		TIM_IMU_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM_SF_IRQn, 0x0F, 0);	/* Set the TIMx priority */
		HAL_NVIC_EnableIRQ(TIM_SF_IRQn);			/* Enable the TIMx global Interrupt */
	}
#endif // IMU_ENABLED
}

/**
  * @brief TIM MSP DeInitialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
#if defined(IMU_ENABLED)
	if (htim_base->Instance == TIM_IMU)
	{
		// IMU Sensor Fusion clock disable
		TIM_IMU_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(TIM_SF_IRQn);
	}
#endif // IMU_ENABLED
}

void HAL_MspInit(void)
{		
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	
#if defined(SERVO_ENABLED)
	// Motor IRQ enable
	TIM_SERVO_CLK_ENABLE();
#endif // SERVO_ENABLED
	
	//HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

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

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
#if defined(SERIAL_PRINT)
	GPIO_InitTypeDef GPIO_InitStruct;
	if (huart->Instance == USART2)
	{
		/* USER CODE BEGIN USART6_MspInit 0 */

		/* USER CODE END USART6_MspInit 0 */
	    /* Peripheral clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();
  
	    /**USART6 GPIO Configuration    
	    PC6     ------> USART6_TX
	    PC7     ------> USART6_RX 
	    */
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN USART6_MspInit 1 */

		/* USER CODE END USART6_MspInit 1 */
	}
#endif // SERIAL_PRINT
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
#if defined(SERIAL_PRINT)
	if (huart->Instance == USART2)
	{
		__HAL_RCC_USART2_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
	}
#endif // SERIAL_PRINT
}

/**
 * @brief  This function is used for low level initialization of the SPI 
 *         communication with the BlueNRG Expansion Board.
 * @param  hspi: SPI handle.
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
#if defined(BLE_ENABLED)
	if(hspi->Instance==BNRG_SPI_INSTANCE)
	{	
		GPIO_InitTypeDef GPIO_InitStruct;
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
		HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET);		/*Added to avoid spurious interrupt from the BlueNRG */

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
#endif // BLE_ENABLED
	
#if defined(UWB_ENABLED)
	if (hspi->Instance == UWB_SPIx)
	{
		GPIO_InitTypeDef GPIO_InitStruct;

		/* Enable GPIO Ports Clock */  
		UWB_SPIx_SCK_GPIO_CLK_ENABLE();
		UWB_SPIx_MISO_GPIO_CLK_ENABLE();
		UWB_SPIx_MOSI_GPIO_CLK_ENABLE();
		UWB_SPIx_NSS_GPIO_CLK_ENABLE();

		/* Enable SPI clock */
		UWB_SPIx_CLK_ENABLE();
  
	    /**SPI3 GPIO Configuration    
	    PA15     ------> SPI3_NSS
	    PC10     ------> SPI3_SCK
	    PC11     ------> SPI3_MISO
	    PC12     ------> SPI3_MOSI 
	    */
		
		GPIO_InitStruct.Pin = UWB_SPIx_NSS_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = UWB_SPIx_NSS_AF;
		HAL_GPIO_Init(UWB_SPIx_NSS_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = UWB_SPIx_SCK_PIN | UWB_SPIx_MISO_PIN | UWB_SPIx_MOSI_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = UWB_SPIx_SCK_AF;
		HAL_GPIO_Init(UWB_SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);
	}
#endif // UWB_ENABLED
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
#if defined(UWB_ENABLED)
	if (hspi->Instance == UWB_SPIx)
	{
		UWB_SPIx_CLK_DISABLE();
  
	    /**SPI3 GPIO Configuration    
	    PA15     ------> SPI3_NSS
	    PC10     ------> SPI3_SCK
	    PC11     ------> SPI3_MISO
	    PC12     ------> SPI3_MOSI 
	    */
		HAL_GPIO_DeInit(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN);
		HAL_GPIO_DeInit(UWB_SPIx_SCK_GPIO_PORT, UWB_SPIx_SCK_PIN | UWB_SPIx_MISO_PIN | UWB_SPIx_MOSI_PIN);
	}
#endif // UWB_ENABLED
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	if (hi2c->Instance == MPU9250_I2C)
	{		
		/* Enable I2C GPIO clocks */
		MPU9250_CLK_ENABLE();

		/* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
		GPIO_InitStruct.Pin			= MPU9250_SDA_PIN | MPU9250_SCL_PIN;
		GPIO_InitStruct.Mode		= GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed		= GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull		= GPIO_NOPULL;
		GPIO_InitStruct.Alternate	= MPU9250_SCL_SDA_AF;
		HAL_GPIO_Init(MPU9250_PORT, &GPIO_InitStruct);

		/* Enable the I2C_EXPBD peripheral clock */
		MPU9250_I2C_CLK_ENABLE();

		/* Force the I2C peripheral clock reset */
		MPU9250_I2C_FORCE_RESET();

		/* Release the I2C peripheral clock reset */
		MPU9250_I2C_RELEASE_RESET();
		
		/* Enable and set I2C_EXPBD Interrupt to the highest priority */
		HAL_NVIC_SetPriority(MPU9250_I2C_EV_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(MPU9250_I2C_EV_IRQn);

			/* Enable and set I2C_EXPBD Interrupt to the highest priority */
		HAL_NVIC_SetPriority(MPU9250_I2C_ER_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(MPU9250_I2C_ER_IRQn);
	}
}

void HAL_I2C_MspDeinit(I2C_HandleTypeDef* hi2c)
{
	if (hi2c->Instance == MPU9250_I2C)
	{
		MPU9250_I2C_CLK_DISABLE();
		HAL_GPIO_DeInit(MPU9250_PORT, MPU9250_SDA_PIN | MPU9250_SCL_PIN);	
	}
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{	
	if (htim_pwm->Instance == TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
	}
	else if (htim_pwm->Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();
	}
#if defined(STEPPER_ENABLED)
	else if (htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_PWM_REF)
	{

		GPIO_InitTypeDef GPIO_InitStruct;
		
		/* Peripheral clock enable */
		__BSP_MOTOR_CONTROL_BOARD_CLCK_ENABLE_PWM_REF();
		//__HAL_RCC_TIM2_CLK_ENABLE();

		/* GPIO clock enable -----------------------------------------------------*/
		/* already done in STSPIN220 GPIO initialization function */
    
		/* Configure STSPIN220 - PWM for REF pin ---------------------------------*/
		GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_PWM_REF;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AF_PWM_REF;
		HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_PWM_REF, &GPIO_InitStruct);
	}
#endif // STEPPER_ENABLED
	else if (htim_pwm->Instance == TIM4)
	{
		__HAL_RCC_TIM4_CLK_ENABLE();
	}
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
	if (htim_pwm->Instance == TIM1)
	{
		__HAL_RCC_TIM1_CLK_DISABLE();
	}
	else if (htim_pwm->Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_DISABLE();
	}
	else if (htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_PWM_REF)
	{
#if defined(STEPPER_ENABLED)
	  /* Peripheral clock disable */
		__BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_PWM_REF();
    
		/* GPIO Deconfiguration */
		HAL_GPIO_DeInit(
			BSP_MOTOR_CONTROL_BOARD_PORT_PWM_REF,
			BSP_MOTOR_CONTROL_BOARD_PIN_PWM_REF);
#endif // STEPPER_ENABLED
	}
	else if (htim_pwm->Instance == TIM4)
	{
		__HAL_RCC_TIM4_CLK_DISABLE();
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if (htim->Instance == TIM3)
	{
		/* USER CODE BEGIN TIM3_MspPostInit 0 */

		/* USER CODE END TIM3_MspPostInit 0 */

		/* USER CODE BEGIN TIM3_MspPostInit 1 */

		/* USER CODE END TIM3_MspPostInit 1 */
	}
}

/**
* @ brief TIM MSP Initialization
* @ param[in] htim TIM handle pointer
* @ retval None
*/
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
#if defined(STEPPER_ENABLED) 
	if (htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIM_STCK)
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Peripheral clock enable -----------------------------------------------*/
		__BSP_MOTOR_CONTROL_BOARD_CLCK_ENABLE_TIM_STCK();
    
		/* GPIO clock enable -----------------------------------------------------*/
		/* already done in STSPIN220 GPIO initialization function */
  
	    /* Configure STSPIN220 - TIMER for STCK pin ------------------------------*/
		GPIO_InitStruct.Pin = BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK_MODE3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
		GPIO_InitStruct.Alternate = BSP_MOTOR_CONTROL_BOARD_AF_TIM_STCK;
		HAL_GPIO_Init(BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK_MODE3, &GPIO_InitStruct);
      
		/* Enable the timer interrupt & set priority -----------------------------*/
		HAL_NVIC_SetPriority(
			BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK,
			BSP_MOTOR_CONTROL_BOARD_PRIORITY_TIM_STCK,
			0);
		
		HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK);
	}
#endif // STEPPER_ENABLED
}

/**
  * @brief TIM MSP DeInitialization
  * @param[in] htim TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
#if defined(STEPPER_ENABLED)
	if (htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIM_STCK)
	{
		/* Peripheral clock disable ----------------------------------------------*/
		__BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_TIM_STCK();
    
		/* GPIO clock disable ----------------------------------------------------*/
		/* do not disable as the clock is likely used for other HW resources */
  
	    /* STCK pin GPIO deinitialization ----------------------------------------*/
		HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK_MODE3, BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK_MODE3);
      
		  /* Disable the timer interrupt -------------------------------------------*/
		HAL_NVIC_DisableIRQ(BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK);
	}
#endif // STEPPER_ENABLED
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ( (htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIM_STCK) && (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIM_STCK) )
	{
		BSP_MotorControl_StepClockHandler(0);
	}
}