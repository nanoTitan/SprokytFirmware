/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32l0xx_hal_msp.c
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    March 15th, 2017
  * @brief   HAL MSP module.    
  ******************************************************************************
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
#include "main.h"

/** @defgroup MSP_module
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId); 
extern void BSP_MotorControl_FlagInterruptHandler(void);
///ButtonHandler defined in main.c
extern void ButtonHandler(void);
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief PWM MSP Initialization 
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_PWM_REF)
  {
    /* Peripheral clock enable */
    __BSP_MOTOR_CONTROL_BOARD_CLCK_ENABLE_PWM_REF();
  
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
}

/**
  * @brief PWM MSP De-Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance == BSP_MOTOR_CONTROL_BOARD_PWM_REF)
  {
    /* Peripheral clock disable */
    __BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_PWM_REF();
    
    /* GPIO Deconfiguration */
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PORT_PWM_REF,\
      BSP_MOTOR_CONTROL_BOARD_PIN_PWM_REF);
  }
}

/**
  * @brief TIM MSP Initialization
  * @param[in] htim TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
  {
  GPIO_InitTypeDef GPIO_InitStruct;
  
  if(htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIM_STCK)
  {
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
    HAL_NVIC_SetPriority(BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK,\
      BSP_MOTOR_CONTROL_BOARD_PRIORITY_TIM_STCK,\
      0);
    HAL_NVIC_EnableIRQ(BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK);
  }
}

/**
  * @brief TIM MSP DeInitialization
  * @param[in] htim TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIM_STCK)
  {
    /* Peripheral clock disable ----------------------------------------------*/
    __BSP_MOTOR_CONTROL_BOARD_CLCK_DISABLE_TIM_STCK();
    
    /* GPIO clock disable ----------------------------------------------------*/
    /* do not disable as the clock is likely used for other HW resources */
  
    /* STCK pin GPIO deinitialization ----------------------------------------*/
    HAL_GPIO_DeInit(BSP_MOTOR_CONTROL_BOARD_PORT_TIM_STCK_MODE3,\
      BSP_MOTOR_CONTROL_BOARD_PIN_TIM_STCK_MODE3);
      
    /* Disable the timer interrupt -------------------------------------------*/
    HAL_NVIC_DisableIRQ(BSP_MOTOR_CONTROL_BOARD_IRQn_TIM_STCK);
  }
}


/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if((htim->Instance == BSP_MOTOR_CONTROL_BOARD_TIM_STCK)&&\
     (htim->Channel == BSP_MOTOR_CONTROL_BOARD_HAL_ACT_CHAN_TIM_STCK))
    {
      BSP_MotorControl_StepClockHandler(0);
    }
}

/**
  * @brief External Line Callback 
  * @param[in] GPIO_Pin pin number
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
    ButtonHandler();
  }
 }
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
