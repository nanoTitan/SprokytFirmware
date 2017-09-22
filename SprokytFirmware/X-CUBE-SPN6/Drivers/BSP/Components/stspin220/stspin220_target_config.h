/**************************************************************************//**
  * @file    stspin220_target_config.h
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    March 15th, 2017
  * @brief   Predefines values for the STSPIN220 registers
  * and for the devices parameters
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
#ifndef __STSPIN220_TARGET_CONFIG_H
#define __STSPIN220_TARGET_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup BSP
  * @{
  */   
   
/** @addtogroup STSPIN220
  * @{
  */   

/** @addtogroup Stspin220_Exported_Constants
  * @{
  */   
   
/** @defgroup Predefined_Stspin220_Registers_Values Predefined Stspin220 Registers Values
  * @{
  */   
   
/// The maximum number of devices
#define MAX_NUMBER_OF_DEVICES                                 (1)

/************************ Speed Profile  *******************************/

/// Acceleration rate in pulse/s2 (must be greater than 0)
#define STSPIN220_CONF_PARAM_ACC                               (480)
   
/// Deceleration rate in pulse/s2 (must be greater than 0)
#define STSPIN220_CONF_PARAM_DEC                               (480)
   
/// Running speed in pulse/s (8 pulse/s < Maximum speed <= 10 000 pulse/s )
#define STSPIN220_CONF_PARAM_RUNNING_SPEED                     (1600)
   
/// Minimum speed in pulse/s (8 pulse/s <= Minimum speed < 10 000 pulse/s)
#define STSPIN220_CONF_PARAM_MIN_SPEED                         (400)

/************************ Torque  *******************************/

/// Acceleration torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_ACC_TORQUE                        (20)

/// Deceleration torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_DEC_TORQUE                        (15)

/// Running torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_RUNNING_TORQUE                    (10)

/// Holding torque in % (from 0 to 100)
#define STSPIN220_CONF_PARAM_HOLDING_TORQUE                    (25)

/// Torque boost speed enable
#define STSPIN220_CONF_PARAM_TORQUE_BOOST_EN                   (TRUE)

/// Torque boost speed threshold in fullstep/s
#define STSPIN220_CONF_PARAM_TORQUE_BOOST_TH                   (200)
    
/******************************* Others ***************************************/

/// Step mode selection settings
#define STSPIN220_CONF_PARAM_STEP_MODE                         (STEP_MODE_1_32)
   
/// Automatic HIZ STOP
#define STSPIN220_CONF_PARAM_AUTO_HIZ_STOP                     (HOLD_MODE)

/// REF PWM frequency (Hz)
#define STSPIN220_CONF_PARAM_REF_PWM_FREQUENCY                 (100000)

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

#ifdef __cplusplus
  }
#endif

#endif /* __STSPIN220_TARGET_CONFIG_H */
