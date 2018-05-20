// Serial Communication using USART

#ifndef _DIFFERENTIAL_DRIVE_H_
#define _DIFFERENTIAL_DRIVE_H_

/* Includes ------------------------------------------------------------------*/
#include "math_ext.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef void(*DiffDriveCallback)(const Transform_t*);

/* Exported macro ------------------------------------------------------------*/
// *** All measurements are in SI units (meters, seconds, etc) *** 
#define DD_WHEEL_BASE_LENGTH	0.08574f
#define DD_WHEEL_RADIUS			.045105f
#define MAX_MOTOR_VEL_COUNT		25	// max counts per second

/* Exported constants --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void DiffDrive_Init();
void DiffDrive_Update();
void DiffDrive_SetPidAuto(bool isAuto);
bool DiffDrive_GetPidAuto();
void DiffDrive_SetAngularPosDegree(float angle);
void DiffDrive_SetPos(float x, float z);
void DiffDrive_ParseTranslate(uint8_t _x, uint8_t _y);
const Transform_t* DiffDrive_GetTransform(); 
void DiffDrive_RegisterCallback(DiffDriveCallback callback);

#endif /* _DIFFERENTIAL_DRIVE_H_ */
