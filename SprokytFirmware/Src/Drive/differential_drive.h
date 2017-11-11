// Serial Communication using USART

#ifndef _DIFFERENTIAL_DRIVE_H_
#define _DIFFERENTIAL_DRIVE_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
// *** All measurements are in SI units *** 
#define DD_WHEEL_BASE_DIST	0.1f
#define DD_WHEEL_RADIUS 0.03f

/* Exported constants --------------------------------------------------------*/
const float DD_Wheel_Half_Dist = DD_WHEEL_BASE_DIST * 0.5f;

/* Exported functions ------------------------------------------------------- */
void DiffDrive_Init();
void DiffDrive_Update();
void DiffDrive_SetVehicleRotation(float rot);

#endif /* _DIFFERENTIAL_DRIVE_H_ */
