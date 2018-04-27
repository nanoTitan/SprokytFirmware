#pragma once

#include <stdint.h>
#include <stdbool.h>

enum PID_direction {
	PID_DIRECT,
	PID_REVERSE,
};

struct PID {	
	float input;
	float output;
	float setpoint;
	float Kp;
	float Ki;
	float Kd;
	float outMin;
	float outMax;
	float iterm;
	float lastInput;
	uint32_t sampletime;
	uint32_t lastTime;
	bool isAutoMode;
	enum PID_direction direction;
};

typedef struct PID* PID_t;

void PID_Create(PID_t pid, float input, float output, float setpoint, float min, float max);
bool PID_CanCompute(PID_t _pid);
bool PID_Compute(PID_t _pid);
void PID_SetTunings(PID_t _pid, float Kp, float Ki, float Kd);
void PID_SetSampleTime(PID_t _pid, uint32_t sample);
void PID_SetOutputLimits(PID_t _pid, float min, float max);
void PID_SetMode(PID_t _pid, bool isAuto);
void PID_SetDirection(PID_t _pid, enum PID_direction direction);
void PID_Setpoint(PID_t pid, float setpoint);