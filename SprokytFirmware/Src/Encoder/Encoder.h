#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
// *** All measurements are in SI units *** 
#define ENCODER_COUNT_PER_REV				3575			// 12 counts per revolution for quadrature encoding * 298:1 gear ratio - 1 for 0 index = 3575
#define ENCODER_ONE_OVER_COUNT_PER_REV		0.00027972f		// 1 / 3575
#define ENCODER_COUNT_MAX_WRAP_CHECK		3475.0f			// A max threshold to see if the count wrapped around
#define ENCODER_COUNT_MIN_WRAP_CHECK		100.0f			// A min threshold to see if the count wrapped around

/* Exported constants --------------------------------------------------------*/

void Encoder_Init();
void Encoder_Update();
float Encoder_GetAngle1();
float Encoder_GetAngle2();
float Encoder_GetAngVel1();
float Encoder_GetAngVel2();
float Encoder_GetAngVelPulseTiming1();
float Encoder_GetAngVelPulseTiming2();
int8_t Encoder_GetDir1();
int8_t Encoder_GetDir2();
void Encoder_ClearCycleCounts();
