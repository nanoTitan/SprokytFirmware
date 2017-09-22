/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM06A1_ExampleFor1Motor/Src/main.c 
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    March 15th, 2017
  * @brief   This example shows how to use 1 IHM06A1 expansion board
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

/** @defgroup IHM06A1_Example_for_1_motor_device
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STEP_ANGLE            (18) // decidegree
#define FULL_STEPS_PER_TURN   (3600/(STEP_ANGLE))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t gLastError;

/* Initialization parameters. */
Stspin220_Init_t initDeviceParameters =
{
  480,             //Acceleration rate in pulse/s2 (must be greater than 0)
  480,             //Deceleration rate in pulse/s2 (must be greater than 0)
  1600,            //Running speed in pulse/s (8 pulse/s < Maximum speed <= 10 000 pulse/s )
  400,             //Minimum speed in pulse/s (8 pulse/s <= Minimum speed < 10 000 pulse/s)
  20,              //Acceleration current torque in % (from 0 to 100)
  15,              //Deceleration current torque in % (from 0 to 100)
  10,              //Running current torque in % (from 0 to 100)
  25,              //Holding current torque in % (from 0 to 100)
  TRUE,            //Torque boost speed enable
  200,             //Torque boost speed threshold in fullstep/s
  STEP_MODE_1_32,  //Step mode via enum motorStepMode_t  
  HOLD_MODE,       //Automatic HIZ STOP
  100000           //REF frequency (Hz)
};

/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);
void ButtonHandler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  int32_t pos;
  uint16_t mySpeed;
  uint16_t myDec;
  motorStepMode_t myStepMode;
  uint32_t myFreqPwm;
  uint8_t myTorque;

  /* STM32xx HAL library initialization */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
    
//----- Init of the Motor control library
  /* Set the STSPIN220 library to use 1 device */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, 1);
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the STSPIN220 parameters are set with the predefined values from file    */
  /* stspin220_target_config.h, otherwise the parameters are set using the    */
  /* initDeviceParameters structure values.                                   */
  //BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, &initDeviceParameters);
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, NULL);
  
  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function MyErrorHandler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(MyErrorHandler);

//----- Configure Button for user interaction

   /* Set Systick Interrupt priority highest to ensure no lock by using HAL_Delay */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0); 
  
  /* Configure KEY Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

//----- Get the PWM frequency used for the VREFA and VREFB voltage generation
  
  myFreqPwm = BSP_MotorControl_GetBridgeInputPwmFreq(0);

//----- Set the PWM frequency used for the VREFA and VREFB voltage generation
  
  BSP_MotorControl_SetBridgeInputPwmFreq(0, myFreqPwm>>1);

//----- Get the predefined step mode
  
  myStepMode = BSP_MotorControl_GetStepMode(0);

//----- Move device 2 turns in the FORWARD direction using predefined microsteps

  /* Move 2*FULL_STEPS_PER_TURN<<myStepMode microsteps in the FORWARD direction */
  BSP_MotorControl_Move(0, FORWARD, 2*FULL_STEPS_PER_TURN<<myStepMode);

  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 1 second */
  HAL_Delay(1000);
  
//----- Disable the power bridges
  
  /* Disable the power bridges */
  BSP_MotorControl_CmdDisable(0);
  
  /* Wait for 1 second */
  HAL_Delay(1000);  

//----- Stop with power bridges disabled

  /* Turn off power bridges when motor is stopped */
  BSP_MotorControl_SetStopMode(0, HIZ_MODE);
  
//----- Move device 2 turns in the BACKWARD direction using predefined microsteps

  /* Move 2*FULL_STEPS_PER_TURN<<myStepMode microsteps in the BACKWARD direction */
  BSP_MotorControl_Move(0, BACKWARD, 2*FULL_STEPS_PER_TURN<<myStepMode);

  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Set the current position to be the Home position */
  BSP_MotorControl_SetHome(0);
  
  /* Wait for 1 second */
  HAL_Delay(1000);

//----- Stop with power bridges enabled

  /* Keep power bridges on when motor is stopped */
  BSP_MotorControl_SetStopMode(0, HOLD_MODE);   
  
//----- Go to position -6400

  /* Request to go to position -6400 */
  BSP_MotorControl_GoTo(0, -6400);  
  
  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position */
  pos = BSP_MotorControl_GetPosition(0);

  if (pos != -6400) 
  {
    MyErrorHandler(STSPIN220_ERROR_POSITION);
  }
  
  /* Set the current position to be the Mark position */
  BSP_MotorControl_SetMark(0);

  /* Wait for 1 second */
  HAL_Delay(1000);
  
//----- Go Home

  /* Request to go to Home */
  BSP_MotorControl_GoHome(0);  
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position */
  pos = BSP_MotorControl_GetPosition(0);
  
  /* Wait for 1 second */
  HAL_Delay(1000);

//----- Go to position 6400

  /* Request to go to position 6400 */
  BSP_MotorControl_CmdGoToDir(0, FORWARD, 6400);
  
  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position */
  pos = BSP_MotorControl_GetPosition(0);

  /* Wait for 1 second */
  HAL_Delay(1000);
  
//----- Go Mark which was set previously after go to -6400

  /* Request to go to Mark position */
  BSP_MotorControl_GoMark(0);  
  
  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position */
  pos = BSP_MotorControl_GetPosition(0);

  /* Wait for 1 second */
  HAL_Delay(1000);

//----- Run the motor BACKWARD

  /* Request to run BACKWARD */
   BSP_MotorControl_Run(0, BACKWARD);       
   HAL_Delay(5000);

   /* Get current speed */
   mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

//----- Increase the speed while running

  /* Increase speed to 2400 microstep/s */
  BSP_MotorControl_SetMaxSpeed(0, 2400);
  HAL_Delay(5000);

   /* Get current speed */
   mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

//----- Decrease the speed while running

  /* Decrease speed to 1200 microstep/s */
  BSP_MotorControl_SetMaxSpeed(0, 1200);
  HAL_Delay(5000);

  /* Get current speed */
  mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

//----- Increase acceleration while running

  /* Increase acceleration to 2000 microstep/s^2 */
  BSP_MotorControl_SetAcceleration(0, 2000);
  HAL_Delay(5000);

  /* Increase speed to 2400 microstep/s */
  BSP_MotorControl_SetMaxSpeed(0, 2400);
  HAL_Delay(5000);

  /* Get current speed */
  mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

  if (mySpeed != 2400)
  {
    MyErrorHandler(STSPIN220_ERROR_SPEED);
  }
//----- Get current deceleration

  myDec = BSP_MotorControl_GetDeceleration(0);
  
//----- Increase deceleration while running

  /* Increase deceleration to 2000 microstep/s^2 */
  BSP_MotorControl_SetDeceleration(0, 2000);
  HAL_Delay(5000);

  /* Decrease speed to 1200 microstep/s */
  BSP_MotorControl_SetMaxSpeed(0, 1200);
  HAL_Delay(5000);

  /* Get current speed */
  mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

//----- Retore previous deceleration
  
  BSP_MotorControl_SetDeceleration(0, myDec);

//----- Soft stopped required while running

  /* Request soft stop */
  BSP_MotorControl_SoftStop(0);

  /* Wait for the motor ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);
  
//----- Run stopped by hardstop

  /* Request to run in FORWARD direction */
  BSP_MotorControl_Run(0, FORWARD);       
  HAL_Delay(5000);
  
  /* Request to immediatly stop */
  BSP_MotorControl_HardStop(0);
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- GOTO stopped by softstop

 /* Request to go to position 20000  */
  BSP_MotorControl_GoTo(0, 20000);  
  HAL_Delay(5000);

  /* Request to perform a soft stop */
  BSP_MotorControl_SoftStop(0);
  BSP_MotorControl_WaitWhileActive(0);

  /* Wait for 2 seconds */
  HAL_Delay(2000);  

//----- Change current torque while running (RUN_TORQUE value is kept unchanged)
  
  /* Get the running torque */
  myTorque = BSP_MotorControl_GetTorque(0, RUN_TORQUE);
  
  /* Request to run in FORWARD direction */
  BSP_MotorControl_Run(0, FORWARD);
  
  /* When the motor is running steady for 100ms, decrease the current torque */
  while (BSP_MotorControl_GetDeviceState(0)!=STEADY);
  HAL_Delay(100);
  BSP_MotorControl_SetTorque(0, CURRENT_TORQUE, myTorque>>1);

  /* Wait for 1 second */
  HAL_Delay(1000);
  
  /* Request soft stop */
  BSP_MotorControl_SoftStop(0);

  /* Wait for the motor ends moving */  
  BSP_MotorControl_WaitWhileActive(0);
  
  /* Get the running torque and check it has not change */
  if (BSP_MotorControl_GetTorque(0, RUN_TORQUE) != myTorque) 
  {
    MyErrorHandler(STSPIN220_ERROR_POSITION);
  }
  
  /* Wait for 1 second */
  HAL_Delay(1000); 
  
//----- Change the holding torque (HOLD TORQUE value is changed)
  
  BSP_MotorControl_SetTorque(0, HOLD_TORQUE, 5);
  
  /* Wait for 1 second */
  HAL_Delay(1000); 

//----- Change step mode to full step mode

  /* Select full step mode */
  BSP_MotorControl_SelectStepMode(0, STEP_MODE_FULL);

  /* Set speed, acceleration and deceleration to scale with full step mode */
  if (BSP_MotorControl_SetMinSpeed(0, BSP_MotorControl_GetMinSpeed(0)>>myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_MIN_SPEED);
  }
  if (BSP_MotorControl_SetMaxSpeed(0, BSP_MotorControl_GetMaxSpeed(0)>>myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_MAX_SPEED);
  }
  if (BSP_MotorControl_SetAcceleration(0, BSP_MotorControl_GetAcceleration(0)>>myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_ACCELERATION);
  }
  if (BSP_MotorControl_SetDeceleration(0, BSP_MotorControl_GetDeceleration(0)>>myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_DECELERATION);
  }
  
  /* Request to go position 200 (full steps) */
  BSP_MotorControl_GoTo(0, 200);

  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position */
  pos =  BSP_MotorControl_GetPosition(0);

  if (pos != 200) 
  {
    MyErrorHandler(STSPIN220_ERROR_POSITION);
  }
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Restore step mode to predefined step mode
  /* Select predefined step mode */
  BSP_MotorControl_SelectStepMode(0, myStepMode);

  /* Set speed, acceleration and deceleration to scale with microstep mode */
  if (BSP_MotorControl_SetMaxSpeed(0, BSP_MotorControl_GetMaxSpeed(0)<<myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_MAX_SPEED);
  }
  if (BSP_MotorControl_SetMinSpeed(0, BSP_MotorControl_GetMinSpeed(0)<<myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_MIN_SPEED);
  }
  if (BSP_MotorControl_SetAcceleration(0, BSP_MotorControl_GetAcceleration(0)<<myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_ACCELERATION);
  }
  if (BSP_MotorControl_SetDeceleration(0, BSP_MotorControl_GetDeceleration(0)<<myStepMode) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_DECELERATION);
  }
  
  /* Request to go to position 200*2^myStepMode */
  pos = 200<<myStepMode;
  BSP_MotorControl_GoTo(0, pos);

  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);

  /* Check current position */
  if (BSP_MotorControl_GetPosition(0) != pos) 
  {
    MyErrorHandler(STSPIN220_ERROR_POSITION);
  }
  else
  {
    /* Set the current position to be the Home position */
    BSP_MotorControl_SetHome(0);
  }
  
  /* Wait for 2 seconds */
  HAL_Delay(2000);

//----- Automatic Full-step switch demonstration
  /* Increase max speed beyond torque boost speed threshold */
  mySpeed = (STSPIN220_CONF_PARAM_TORQUE_BOOST_TH<<(myStepMode+1));
  if (BSP_MotorControl_SetMaxSpeed(0, mySpeed) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_MAX_SPEED);
  }
  /* increase acceleration */
  if (BSP_MotorControl_SetAcceleration(0, BSP_MotorControl_GetAcceleration(0)<<3) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_ACCELERATION);
  }
  /* increase deceleration */
  if (BSP_MotorControl_SetDeceleration(0, BSP_MotorControl_GetDeceleration(0)<<3) == FALSE)
  {
    MyErrorHandler(STSPIN220_ERROR_SET_DECELERATION);
  }
  /* Request to run */
  BSP_MotorControl_Run(0, BACKWARD);
  HAL_Delay(5000);
  
  /* Request soft stop */
  BSP_MotorControl_SoftStop(0);
  HAL_Delay(5000);

//----- Slowest move

  /* Set the current position to be the Home position */
  BSP_MotorControl_SetHome(0);
  
  /* Set speed and acceleration at lowest values */
  BSP_MotorControl_SetMinSpeed(0, 8);
  BSP_MotorControl_SetMaxSpeed(0, 8);
  BSP_MotorControl_SetAcceleration(0, 8);
  BSP_MotorControl_SetDeceleration(0, 8);

  /* Move forward device number of microsteps in a full step + 1 microstep */
  pos = (1<<myStepMode)+1;
  BSP_MotorControl_Move(0, FORWARD, pos);
  
  /* Wait for the motor ends moving */
  BSP_MotorControl_WaitWhileActive(0);
  
  /* Check current position */
  if (BSP_MotorControl_GetPosition(0) != pos) 
  {
    MyErrorHandler(STSPIN220_ERROR_POSITION);
  }
  else
  {
    /* Set the current position to be the Home position */
    BSP_MotorControl_SetHome(0);
  }
  
  /* Wait for 40 milliseconds */
  HAL_Delay(40);
 
//----- Stop with Standby mode
  
  /* Enter standby mode when motor is stopped */
  BSP_MotorControl_SetStopMode(0, STANDBY_MODE);

  /* Move forward 1 microstep */
  BSP_MotorControl_Move(0, FORWARD, 1);
  BSP_MotorControl_WaitWhileActive(0);
  BSP_MotorControl_SetHome(0);

  /* Wait for 40 milliseconds */
  HAL_Delay(40);
  
  /* Move forward 1 microstep */
  BSP_MotorControl_Move(0, FORWARD, 1);
  BSP_MotorControl_WaitWhileActive(0);
  BSP_MotorControl_SetHome(0);
  
  /* Wait for 40 milliseconds */
  HAL_Delay(40);
  
  /* Move forward 1 microstep */
  BSP_MotorControl_Move(0, FORWARD, 1);
  BSP_MotorControl_WaitWhileActive(0);
  BSP_MotorControl_SetHome(0);

  /* Wait for 40 milliseconds */
  HAL_Delay(40);
  
  /* Move forward 1 microstep */
  BSP_MotorControl_Move(0, FORWARD, 1);
  BSP_MotorControl_WaitWhileActive(0);
  BSP_MotorControl_SetHome(0);  

//----- Stop with power bridges enabled

  /* Keep power bridges on when motor is stopped */
  BSP_MotorControl_SetStopMode(0, HOLD_MODE);
  
//----- Set speed, acceleration and deceleration from stspin220_target_config.h 
  
  /* Set speed and acceleration from stspin220_target_config.h */
  BSP_MotorControl_SetMaxSpeed(0, STSPIN220_CONF_PARAM_RUNNING_SPEED);
  BSP_MotorControl_SetMinSpeed(0, STSPIN220_CONF_PARAM_MIN_SPEED);
  BSP_MotorControl_SetAcceleration(0, STSPIN220_CONF_PARAM_ACC);
  BSP_MotorControl_SetDeceleration(0, STSPIN220_CONF_PARAM_DEC);
 
  /* Wait for 2 seconds */
  HAL_Delay(2000);
  
//----- Infinite loop with Run and Soft Stop
  while(1)
  { 
    /* Request to run */
    BSP_MotorControl_Run(0, BSP_MotorControl_GetDirection(0));
    HAL_Delay(4000);
  
    /* Request soft stop */
    BSP_MotorControl_SoftStop(0);
    HAL_Delay(5000);
  }
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  //When EN pin is forced low by a failure, configure the GPIO as an ouput low
  BSP_MotorControl_CmdDisable(0);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void MyErrorHandler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Infinite loop */
  while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of button press and it changes the
  * current direction of the motor
  * @param  None
  * @retval None
  */
void ButtonHandler(void)
{
  if (BSP_MotorControl_GetDirection(0) != BACKWARD)
  {
    BSP_MotorControl_SetDirection(0, BACKWARD);
  }
  else
  {
    BSP_MotorControl_SetDirection(0, FORWARD);
  }
  /* Let 200 ms before clearing the IT for key debouncing */
  HAL_Delay(200);
  __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
  HAL_NVIC_ClearPendingIRQ(KEY_BUTTON_EXTI_IRQn);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
