/**
  ******************************************************************************
  * @file    stm32xx_it.c 
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "stm32xx_it.h"
#include "debug.h"
#include "hci.h"
#include "constants.h"
#include "stm32f4xx_nucleo_ihm06a1.h"

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup SensorDemo
 *  @{
 */
 
/** @defgroup INTERRUPT_HANDLER 
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t button_event = 0;
/* SPI handler declared in "main.c" file */

extern SPI_HandleTypeDef SpiHandle;
extern uint8_t magcal_request;
extern TIM_HandleTypeDef hTimerStepClock;
/* Private function prototypes -----------------------------------------------*/
extern void BSP_MotorControl_FlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/
void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress);

/******************************************************************************/
/*            Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  NMI_Handler This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  HardFault_Handler This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{	
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  SVC_Handler This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  DebugMon_Handler This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  PendSV_Handler This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  SysTick_Handler This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();	// <--- Do I need this??? It was generated from CubeMX USART project
}


/******************************************************************************/
/*                 Peripherals Interrupt Handlers                   */
/******************************************************************************/

/**
  * @brief  This function handles External line interrupt request for BlueNRG.
  * @param  None
  * @retval None
  */
void BNRG_SPI_EXTI_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BNRG_SPI_EXTI_PIN);
}


/**
  * @brief  This function handles the Push Button interrupt request.
  * @param  None
  * @retval None
  */
void PUSH_BUTTON_EXTI_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
	HAL_GPIO_EXTI_IRQHandler(BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT);
  
  button_event = 1;
}

/**
* @brief  This function handles TIM interrupt request for sensor fusion
* @param  None
* @retval None
*/
void TIM_IMU_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&ImuTimHandle);
}

/**
* @brief  This function handles TIM interrupt request for StSpin220 Stepper
* @param  None
* @retval None
*/
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&hTimerStepClock);
}

/**
  * @brief  EXTI line detection callbacks
  * @param  GPIO_Pin the pin connected to EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BSP_MOTOR_CONTROL_BOARD_PIN_EN_AND_FAULT)
	{
		BSP_MotorControl_FlagInterruptHandler();
	}
	if (GPIO_Pin == KEY_BUTTON_PIN)
	{
		// Tell IMU to begin magnetometer calibration request
		if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
		{
			magcal_request = 1;
		}
	}
	
	
	// UPdate BlueNRG ISR. This normally happens in bluenrg_interface.c. 
	// We put it here since multiple objects need to know about EXTI_Callback
	HCI_Isr();
}

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*
void PPP_IRQHandler(void)
{
}
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
