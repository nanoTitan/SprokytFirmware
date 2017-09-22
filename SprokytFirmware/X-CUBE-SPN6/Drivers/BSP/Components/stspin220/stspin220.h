/******************************************************//**
  * @file    stspin220.h 
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    March 15th, 2017
  * @brief   Header for STSPIN220 driver (fully integrated microstepping motor driver)
  * @note    (C) COPYRIGHT 2016 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STSPIN220_H
#define __STSPIN220_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stspin220_target_config.h"
#include "motor.h"
   
/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STSPIN220
  * @{
  */   
   
/* Exported Constants --------------------------------------------------------*/

/** @defgroup Stspin220_Exported_Constants Stspin220 Exported Constants
  * @{
  */   
/// Current FW major version
#define STSPIN220_FW_MAJOR_VERSION (uint8_t)(1)
/// Current FW minor version
#define STSPIN220_FW_MINOR_VERSION (uint8_t)(1)
/// Current FW patch version
#define STSPIN220_FW_PATCH_VERSION (uint8_t)(0)
/// Current FW version
#define STSPIN220_FW_VERSION       (uint32_t)((STSPIN220_FW_MAJOR_VERSION<<16)|\
                                              (STSPIN220_FW_MINOR_VERSION<<8)|\
                                              (STSPIN220_FW_PATCH_VERSION))

/// Max position
#define STSPIN220_MAX_POSITION           (0x7FFFFFFF)

/// Min position
#define STSPIN220_MIN_POSITION           (0x80000000)

/// Position range
#define STSPIN220_POSITION_RANGE         ((uint32_t)(STSPIN220_MAX_POSITION -\
                                                        STSPIN220_MIN_POSITION))
/// STSPIN220 error base number
#define STSPIN220_ERROR_BASE             (0xA000)

/// run bit mask
#define STSPIN220_RUN_BIT_MASK           (0x01)

/// move bit mask
#define STSPIN220_MOVE_BIT_MASK          (0x02)

/// soft stop bit mask
#define STSPIN220_SOFT_STOP_BIT_MASK     (0x04)
   
/// direction change bit mask
#define STSPIN220_DIR_CHANGE_BIT_MASK    (0x08)

/// Maximum frequency of the step clock frequency in Hz
#define STSPIN220_MAX_STCK_FREQ          (10000)

/// Minimum frequency of the step clock frequency in Hz
#define STSPIN220_MIN_STCK_FREQ          (8)

/// Minimum duration of standby 
#define STANDBY_MIN_DURATION             (1)
    
/// Dead time after standby exit
#define AFTER_STANDBY_EXIT_DEAD_TIME     (1)

/// Reset delay to select step mode
#define SELECT_STEP_MODE_DELAY           (1)

/// PWM REF and bridges disable delay
#define DISABLE_DELAY                    (1)

/// Microstepping sequencer maximum value
#define SEQUENCER_MAX_VALUE              (uint16_t)(0x3FF)

/**
  * @}
  */

/* Exported Variables --------------------------------------------------------*/

/** @addtogroup Stspin220_Exported_Variables
  * @{
  */
extern motorDrv_t   stspin220Drv;
/**
  * @}
  */
     
/* Exported Types  -------------------------------------------------------*/

/** @defgroup Stspin220_Exported_Types Stspin220 Exported Types
  * @{
  */
    
/** @defgroup Error_Types Error Types
  * @{
  */
/// Errors
typedef enum {
  STSPIN220_ERROR_SET_HOME         = STSPIN220_ERROR_BASE,      /// Error while setting home position
  STSPIN220_ERROR_SET_MAX_SPEED    = STSPIN220_ERROR_BASE + 1,  /// Error while setting max speed
  STSPIN220_ERROR_SET_MIN_SPEED    = STSPIN220_ERROR_BASE + 2,  /// Error while setting min speed
  STSPIN220_ERROR_SET_ACCELERATION = STSPIN220_ERROR_BASE + 3,  /// Error while setting acceleration
  STSPIN220_ERROR_SET_DECELERATION = STSPIN220_ERROR_BASE + 4,  /// Error while setting decelaration
  STSPIN220_ERROR_MCU_OSC_CONFIG   = STSPIN220_ERROR_BASE + 5,  /// Error while configuring mcu oscillator
  STSPIN220_ERROR_MCU_CLOCK_CONFIG = STSPIN220_ERROR_BASE + 6,  /// Error while configuring mcu clock
  STSPIN220_ERROR_POSITION         = STSPIN220_ERROR_BASE + 7,  /// Unexpected current position (wrong number of steps)
  STSPIN220_ERROR_SPEED            = STSPIN220_ERROR_BASE + 8,  /// Unexpected current speed
  STSPIN220_ERROR_INIT             = STSPIN220_ERROR_BASE + 9,  /// Unexpected number of devices or unexpected value for predefined parameter
  STSPIN220_ERROR_SET_DIRECTION    = STSPIN220_ERROR_BASE + 10, /// Error while setting direction
  STSPIN220_ERROR_SET_STEP_MODE    = STSPIN220_ERROR_BASE + 11, /// Attempt to set an unsupported step mode
  STSPIN220_ERROR_APPLY_SPEED      = STSPIN220_ERROR_BASE + 12, /// Error while applying speed
  STSPIN220_ERROR_SET_TORQUE       = STSPIN220_ERROR_BASE + 13, /// Error while setting torque
  STSPIN220_ERROR_STEP_CLOCK       = STSPIN220_ERROR_BASE + 14  /// Error related to step clock
}errorTypes_t;
/**
  * @}
  */

/** @defgroup Device_Commands Device Commands
  * @{
  */
/// Device commands 
typedef enum {
  NO_CMD              = 0x00, 
  RUN_CMD             = (STSPIN220_RUN_BIT_MASK),
  MOVE_CMD            = (STSPIN220_MOVE_BIT_MASK),
} deviceCommand_t;
/**
  * @}
  */


/** @defgroup Device_Parameters Device Parameters
  * @{
  */

/// Device Parameters Structure Type
typedef struct {
    /// accumulator used to store speed increase smaller than 1 pps
    volatile uint32_t accu;           
    /// Position in microstep according to current step mode
    volatile int32_t currentPosition;
    /// Position of sequencer
    volatile int16_t sequencerPosition;
    /// mark position in microstep (motor position control mode)
    volatile int32_t markPosition;
    /// position in microstep at the end of the accelerating phase
    volatile uint32_t endAccPos;      
    /// nb of in microstep performed from the beggining of the goto or the move command 
    volatile uint32_t relativePos;    
    /// position in microstep step at the start of the decelerating phase
    volatile uint32_t startDecPos;    
    /// nb of microstep steps to perform for the goto or move commands
    uint32_t stepsToTake;
    
    /// constant speed phase torque value (%)
    volatile uint8_t runTorque;
    /// acceleration phase torque value (%)
    volatile uint8_t accelTorque;
    /// deceleration phase torque value (%)
    volatile uint8_t decelTorque;
    /// holding phase torque value (%)
    volatile uint8_t holdTorque;
    /// current selected torque value
    volatile uint8_t currentTorque;
    /// torque update
    volatile bool updateTorque;
    /// PWM frequency used to generate REF voltage
    volatile uint32_t refPwmFreq;
    /// torque boost enable
    volatile bool torqueBoostEnable;
    /// torque boost speed threshold
    volatile uint16_t torqueBoostSpeedThreshold;
    
    /// acceleration in pps^2 
    volatile uint16_t acceleration;
    /// deceleration in pps^2
    volatile uint16_t deceleration;
    /// max speed in pps (speed use for goto or move command)
    volatile uint16_t maxSpeed;
    /// min speed in pps
    volatile uint16_t minSpeed;
    /// current speed in pps
    volatile uint16_t speed;
    
    /// command under execution
    volatile deviceCommand_t commandExecuted; 
    /// FORWARD or BACKWARD direction
    volatile motorDir_t direction;                 
    /// current state of the device
    volatile motorState_t motionState;
    /// current step mode
    volatile motorStepMode_t stepMode;
    /// latched step mode
    motorStepMode_t stepModeLatched;
    /// current stop mode
    motorStopMode_t stopMode;
    
}deviceParams_t; 

/// Motor driver initialization structure definition  
typedef struct
{
  /// acceleration in pps^2
  uint16_t acceleration;
  /// deceleration in pps^2
  uint16_t deceleration;
  /// max speed in pps (speed use for goto or move command)
  uint16_t maxSpeed;
  /// min speed in pps
  uint16_t minSpeed;
  /// acceleration phase torque value (%)
  uint8_t accelTorque;
  /// deceleration phase torque value (%)
  uint8_t decelTorque;
  /// constant speed phase torque value (%)
  uint8_t runTorque;
  /// holding phase torque value (%)
  uint8_t holdTorque;
  /// torque boost enable
  bool torqueBoostEnable;
  /// torque boost speed threshold
  uint16_t torqueBoostSpeedThreshold;
  /// step mode
  motorStepMode_t stepMode;
  /// stop mode
  motorStopMode_t stopMode;
  /// PWM frequency used to generate REF voltage
  uint32_t vrefPwmFreq;
} Stspin220_Init_t;
    
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup Stspin220_Exported_Functions Stspin220 Exported Functions
  * @{
  */   
motorDrv_t* Stspin220_GetMotorHandle(void);                      //Return handle of the motor driver handle
void Stspin220_Init(void* pInit);                                 //Start the STSPIN220 library
uint16_t Stspin220_ReadId(void);                                 //Read Id to get driver instance
void Stspin220_AttachErrorHandler(void (*callback)(uint16_t));   //Attach a user callback to the error handler
void Stspin220_AttachFlagInterrupt(void (*callback)(void));      //Attach a user callback to the flag Interrupt
uint8_t Stspin220_CheckStatusHw(void);                           //Check if STSPIN220 has a fault by reading EN pin position
void Stspin220_Disable(uint8_t deviceId);                        //Disable the power stage of the specified device
void Stspin220_Enable(uint8_t deviceId);                         //Enable the power stage of the specified device
void Stspin220_ErrorHandler(uint16_t error);                     //Error handler which calls the user callback
void Stspin220_ExitDeviceFromStandby(uint8_t deviceId);          //Exit STSPIN220 device from standby 
uint16_t Stspin220_GetAcceleration(uint8_t deviceId);            //Return the acceleration in pps^2
uint16_t Stspin220_GetCurrentSpeed(uint8_t deviceId);            //Return the current speed in pps
uint16_t Stspin220_GetDeceleration(uint8_t deviceId);            //Return the deceleration in pps^2
motorState_t Stspin220_GetDeviceState(uint8_t deviceId);         //Return the device state
motorDir_t Stspin220_GetDirection(uint8_t deviceId);             //Get the motor current direction
uint32_t Stspin220_GetFwVersion(void);                           //Return the FW version
int32_t Stspin220_GetMark(uint8_t deviceId);                     //Return the mark position 
uint16_t Stspin220_GetMaxSpeed(uint8_t deviceId);                //Return the max speed in pps
uint16_t Stspin220_GetMinSpeed(uint8_t deviceId);                //Return the min speed in pps
uint8_t Stspin220_GetNbDevices(void);                            //Return the nupber of devices
int32_t Stspin220_GetPosition(uint8_t deviceId);                 //Return the ABS_POSITION (32b signed)
motorStepMode_t Stspin220_GetStepMode(uint8_t deviceId);         //Get the motor step mode
motorStopMode_t Stspin220_GetStopMode(uint8_t deviceId);         //Get the selected mode to stop the motor
uint8_t Stspin220_GetTorque(uint8_t deviceId, motorTorqueMode_t torqueMode);
bool Stspin220_GetTorqueBoostEnable(uint8_t deviceId);           //Get the torque boost feature status
uint16_t Stspin220_GetTorqueBoostThreshold(uint8_t deviceId);    //Get the torque boost threshold
void Stspin220_GoHome(uint8_t deviceId);                         //Move to the home position
void Stspin220_GoMark(uint8_t deviceId);                         //Move to the Mark position
void Stspin220_GoTo(uint8_t deviceId, int32_t targetPosition);   //Go to the specified position
void Stspin220_GoToDir(uint8_t deviceId,\
  motorDir_t direction,\
  int32_t targetPosition);                                       //Go to the specified position using the specified direction
void Stspin220_HardStop(uint8_t deviceId);                       //Stop the motor and keep holding torque
void Stspin220_HardHiZ(uint8_t deviceId);                        //Stop the motor and disable the power bridge
void Stspin220_Move(uint8_t deviceId,                            //Move the motor of the specified number of steps
                motorDir_t direction,
                uint32_t stepCount);
void Stspin220_PutDeviceInStandby(uint8_t deviceId);              //Put STSPIN220 device in standby (low power consumption)
void Stspin220_Run(uint8_t deviceId, motorDir_t direction);       //Run the motor 
bool Stspin220_SetAcceleration(uint8_t deviceId,uint16_t newAcc); //Set the acceleration in pps^2
bool Stspin220_SetDeceleration(uint8_t deviceId,uint16_t newDec); //Set the deceleration in pps^2
void Stspin220_SetDirection(uint8_t deviceId,                     //Set the STSPIN220 direction pin
                        motorDir_t direction);
void Stspin220_SetHome(uint8_t deviceId);                         //Set current position to be the home position
void Stspin220_SetMark(uint8_t deviceId);                         //Set current position to be the Markposition
bool Stspin220_SetMaxSpeed(uint8_t deviceId,uint16_t newMaxSpeed);//Set the max speed in pps
bool Stspin220_SetMinSpeed(uint8_t deviceId,uint16_t newMinSpeed);//Set the min speed in pps
bool Stspin220_SetNbDevices(uint8_t nbDevices);
bool Stspin220_SetStepMode(uint8_t deviceId,                      
  motorStepMode_t stepMode);                                      // Step mode selection
void Stspin220_SetStopMode(uint8_t deviceId,\
  motorStopMode_t stopMode);                                      //Select the mode to stop the motor
void Stspin220_SetTorque(uint8_t deviceId,\
  motorTorqueMode_t torqueMode,\
  uint8_t torqueValue);
void Stspin220_SetTorqueBoostEnable(uint8_t deviceId, bool enable); // Enable or disable the torque boost feature
void Stspin220_SetTorqueBoostThreshold(uint8_t deviceId, uint16_t speedThreshold); //Set the torque boost threshold
bool Stspin220_SoftStop(uint8_t deviceId);                        //Progressively stop the motor by using the device deceleration and set deceleration torque
uint32_t Stspin220_VrefPwmGetFreq(uint8_t deviceId);              //Get the frequency of REF PWM of the specified device
void Stspin220_VrefPwmSetFreq(uint8_t deviceId, uint32_t newFreq);//Set the frequency of REF PWM of the specified device
void Stspin220_WaitWhileActive(uint8_t deviceId);                 //Wait for the device state becomes Inactive
/**
  * @}
  */

/** @defgroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */
///Delay of the requested number of milliseconds
extern void Stspin220_Board_Delay(uint32_t delay);
///Disable the power bridges (leave the output bridges HiZ)    
extern void Stspin220_Board_Disable(void);
///Disable Irq
extern void Stspin220_Board_DisableIrq(void);
//Get the EN FAULT pin state
extern uint32_t Stspin220_Board_EN_AND_FAULT_PIN_GetState(void);
///Enable the power bridges (leave the output bridges HiZ)
extern void Stspin220_Board_Enable(void); 
///Enable Irq
extern void Stspin220_Board_EnableIrq(void);
///Initialise GPIOs used for STSPIN220
extern void Stspin220_Board_GpioInit(void);
///Init the reference voltage pwm
extern void Stspin220_Board_PwmRefInit(void); 
///Set the frequency and duty cycle of PWM used for the reference voltage generation
extern void Stspin220_Board_PwmRefSetFreqAndDutyCycle(uint32_t newFreq, uint8_t dutyCycle);
///Start the reference voltage pwm
extern void Stspin220_Board_PwmRefStart(void);
///Stop the reference voltage pwm
extern void Stspin220_Board_PwmRefStop(void);
///Reset the STSPIN220 reset pin
extern void Stspin220_Board_ReleaseReset(void);
///Set the STSPIN220 reset pin 
extern void Stspin220_Board_Reset(void);
///Set direction GPIO
extern void Stspin220_Board_SetDirectionGpio(uint8_t gpioState);
///Select Full Step mode
extern void Stspin220_Board_SetFullStep(void);
///Select the STSPIN220 mode1, mode2, mode3 and mode4 pins levels
extern bool Stspin220_Board_SetModePins(uint8_t modePin1Level,\
  uint8_t modePin2Level,\
  uint8_t modePin3Level,\
  uint8_t modePin4Level);
///Step clock compare value initialization
extern void Stspin220_Board_TimStckCompareInit(void);
///DeInit the timer
extern void Stspin220_Board_TimStckDeInit(void);
///Init the timer
extern void Stspin220_Board_TimStckInit(void);
///Set step clock frequency
extern void Stspin220_Board_TimStckSetFreq(uint16_t newFreq);
///Start step clock
extern void Stspin220_Board_TimStckStart(void);
///Stop the timer
extern uint8_t Stspin220_Board_TimStckStop(volatile uint8_t *pToggleOdd);
///Unselect Full Step mode
extern void Stspin220_Board_UnsetFullStep(void);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* #ifndef __STSPIN220_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
