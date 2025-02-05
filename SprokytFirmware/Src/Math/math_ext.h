#ifndef _MATH_EXT_H_
#define _MATH_EXT_H_

#include <math.h>
#include <stdint.h>

#define M_1_255  0.003921568627451
#define M_180_Over_Pi 57.29577951308233
#define M_Pi_Over_180 0.01745329251
#define M_2PI 6.28318530718
#define ONE_OVER_SQRT2 0.7071067811865

typedef struct 
{
	float x;
	float y;
	float z;
} EulerAngle_t;

typedef struct 
{
	float w;
	float x;
	float y;
	float z;
} Quaternion_t;

typedef struct
{
	float x;
	float y;
} Vector2_t;

typedef struct Transform
{
	float x;
	float y;
	float z;
	float yaw;
	float pitch;
	float roll;
} Transform_t;

static float clampf(float x, float min, float max)
{
	if (x < min)	return min;
	if (x > max)	return max;
	return x;
}

static int clamp(int x, int min, int max)
{
	if (x < min)	return min;
	if (x > max)	return max;
	return x;
}

static float RadiansToDeg(float radians)
{
	return radians * M_180_Over_Pi;
}

static float DegToRadians(float degrees)
{
	return degrees * M_Pi_Over_180;
}

static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	if (x <= in_min)
		return out_min;
	
	if (x >= in_max)
		return out_max;
	
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint32_t mapi(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
	if (x <= in_min)
		return out_min;
	
	if (x >= in_max)
		return out_max;
	
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	if (x <= in_min)
		return out_min;
	
	if (x >= in_max)
		return out_max;
	
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float wrap_180(float x)
{
	if (x < -180)
		return x + 360;
	if (x > 180)
		return x - 360;
	
	return x;
}

static float vector2_length(float x1, float y1, float x2, float y2)
{
	float x = x1 - x2;
	float y = y1 - y2;
	return sqrtf(x*x + y*y);
}

static float vector2_length_sqr(float x1, float y1, float x2, float y2)
{
	float x = x1 - x2;
	float y = y1 - y2;
	return x*x + y*y;
}

#endif // _MATH_EXT_H_
