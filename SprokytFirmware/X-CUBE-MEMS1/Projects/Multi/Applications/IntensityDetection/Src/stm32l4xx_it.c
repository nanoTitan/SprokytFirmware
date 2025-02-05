/**
  ******************************************************************************
  * @file        stm32l4xx_it.c
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Interrupt Service Routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "stm32l4xx_it.h"
#include "MotionID_Manager.h"

/** @addtogroup MOTION_ID_Applications
  * @{
  */

/** @addtogroup INTENSITY_DETECTION
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
extern volatile uint8_t flash_erase_request;
extern flash_state_t FlashState;
extern uint8_t DataLoggerActive;

/* Private function prototypes -----------------------------------------------*/
void TIM_ID_IRQHandler(void);

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception
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
  * @brief  This function handles Debug Monitor exception
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM_ID global interrupt.
  * @param  None
  * @retval None
  */
void TIM_ID_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&ID_TimHandle);
}


/**
  * @brief  This function handles External lines 10-15 interrupt requests
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(KEY_BUTTON_PIN) != RESET)
  {
    /* Button locked when active GUI datastream */
    if (DataLoggerActive)
    {
      __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
      return;
    }

    if (FlashState == FLASH_FULL)
    {
      HAL_Delay(3500);

      if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
      {
        flash_erase_request = 1;
      }

      __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
      return;
    }

    HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
