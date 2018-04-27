#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
// *** All measurements are in SI units *** 
#define ENCODER_COUNT_PER_REV				1799			// 3 tooth encoder * 150:1 gear ratio * 4 counts for quadrature encoding - 1 for 0 index = 1799
#define ENCODER_COUNT_MAX_WRAP_CHECK		1650.0f			// A max threshold to see if the count wrapped around
#define ENCODER_COUNT_MIN_WRAP_CHECK		150.0f			// A min threshold to see if the count wrapped around
#define ENCODER_ONE_OVER_QUAD_COUNT_PER_REV	0.0005556f		// 1 / (150:1 * 3 tooth * Quadrature Count) = 1 / (150 * 3 * 4) = 1 / 1800

/* Exported constants --------------------------------------------------------*/

void Encoder_Init();
void Encoder_Update();
float Encoder_GetAngle1();
float Encoder_GetAngle2();
float Encoder_GetAngVel1();
float Encoder_GetAngVel2();
int8_t Encoder_GetDir1();
int8_t Encoder_GetDir2();
