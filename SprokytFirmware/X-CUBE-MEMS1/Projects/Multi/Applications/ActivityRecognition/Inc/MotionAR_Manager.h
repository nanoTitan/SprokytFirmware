/**
  ******************************************************************************
  * @file        MotionAR_Manager.h
  * @author      MEMS Application Team
  * @version     V2.0.0
  * @date        01-May-2017
  * @brief       Header for MotionAR_Manager.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTIONAR_MANAGER_H_
#define _MOTIONAR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "motion_ar.h"
#include "main.h"


/** @addtogroup MOTION_AR_Applications
  * @{
  */

/** @addtogroup DATALOGACTIVITY
  * @{
  */

/* Exported Macros -----------------------------------------------------------*/
/**
* @brief  Scale factor. It is used to scale acceleration from mg to g.
*/
#define FROM_MG_TO_G    0.001

/* Exported Types ------------------------------------------------------------*/
/* Exported Variables --------------------------------------------------------*/
extern AxesF_TypeDef        ACC_Value_f;
extern MAR_output_t         ActivityCode;


/* Exported Functions Prototypes ---------------------------------------------*/
void MotionAR_manager_init(void);
void MotionAR_manager_run(void);
void MotionAR_manager_get_version(char *version, int *length);


/**
 * @}
 */  /* end of group DATALOGACTIVITY */

/**
 * @}
 */  /* end of group MOTION_AR_Applications */

#ifdef __cplusplus
}
#endif

#endif //_MOTIONAR_MANAGER_H_

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

