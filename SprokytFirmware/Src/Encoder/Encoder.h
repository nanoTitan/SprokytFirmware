#ifndef ENCODER_H
#define ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
// *** All measurements are in SI units *** 
#define ENCODER_COUNT_PER_REV				1800			// 3 tooth encoder * 150:1 gear ratio * 4 counts for quadrature encoding = 1800
#define ENCODER_ONE_OVER_QUAD_COUNT_PER_REV	0.0005556f		// 1 / (450 * 4) = 1 / 1800

/* Exported constants --------------------------------------------------------*/


void Encoder_Init();
void Encoder_Update();
float Encoder_GetRot1();
float Encoder_GetRot2();
float Encoder_GetDeltaRad1();
float Encoder_GetDeltaRad2();
float Encoder_GetAngVel1();
float Encoder_GetAngVel2();
int8_t Encoder_GetDir1();
int8_t Encoder_GetDir2();

#endif