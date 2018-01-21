// Serial Communication using USART

#ifndef _DIFFERENTIAL_DRIVE_H_
#define _DIFFERENTIAL_DRIVE_H_

/* Includes ------------------------------------------------------------------*/
#include "math_ext.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
// *** All measurements are in SI units *** 
#define DD_WHEEL_BASE_LENGTH	0.1524f
#define DD_WHEEL_RADIUS			.055f

/* Exported constants --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void DiffDrive_Init();
void DiffDrive_Update();
void DiffDrive_SetAngularPosDegree(float angle);
void DiffDrive_ParseTranslate(uint8_t _x, uint8_t _y);
const Transform_t* DiffDrive_GetTransform(); 

#endif /* _DIFFERENTIAL_DRIVE_H_ */
