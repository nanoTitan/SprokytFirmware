
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "error.h"

static void PID_Initialize(PID_t pid);

void PID_Create(PID_t pid, float input, float output, float setpoint, float min, float max)
{	
	pid->input = input;
	pid->output = output;
	pid->setpoint = setpoint;
	PID_SetOutputLimits(pid, min, max);
	pid->isAutoMode = false;
	pid->sampletime = 100;
	PID_SetDirection(pid, PID_DIRECT);
	pid->lastTime = HAL_GetTick();
	
	PID_Initialize(pid);
}

void PID_Initialize(PID_t pid)
{
	pid->lastInput = pid->input;
	pid->iterm = pid->output;
	if (pid->iterm > pid->outMax) pid->iterm = pid->outMax;
	else if (pid->iterm < pid->outMin) pid->iterm = pid->outMin;
}

bool PID_CanCompute(PID_t pid)
{
	uint32_t currTime = HAL_GetTick();
	uint32_t dT = currTime - pid->lastTime;
	if (dT < pid->sampletime)
		return false;
	
	return true;
}

bool PID_Compute(PID_t pid)
{
	// Check if control is enabled
	if (!pid->isAutoMode)
		return false;
	
	// Integral
	float error = pid->setpoint - pid->input;
	
	pid->iterm += pid->Ki * error;
	
	if (pid->iterm > pid->outMax) pid->iterm = pid->outMax;
	else if (pid->iterm < pid->outMin) pid->iterm = pid->outMin;
	
	// Compute PID output
	float deltaInput = pid->input - pid->lastInput;
	pid->output = pid->Kp * error + pid->iterm - pid->Kd * deltaInput;
	
	//PRINTF("err/pIn/pOut/in/out: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", error, pid->setpoint, pid->input, pid->lastInput, deltaInput, pid->output);
	
	if (pid->output > pid->outMax) pid->output = pid->outMax;
	else if (pid->output < pid->outMin) pid->output = pid->outMin;
	
	pid->lastInput = pid->input;
	pid->lastTime = HAL_GetTick(); 
	
	return true;
}

void PID_SetTunings(PID_t pid, float Kp, float Ki, float Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0) return;
 
	float sampleSecs = ((float)pid->sampletime) / 1000;
	pid->Kp = Kp;
	pid->Ki = Ki * sampleSecs;
	pid->Kd = Kd / sampleSecs;
 
	if (pid->direction == PID_REVERSE)
	{
		pid->Kp = -pid->Kp;
		pid->Ki = -pid->Ki;
		pid->Kd = -pid->Kd;
	}
}

void PID_SetSampleTime(PID_t pid, uint32_t newSampleTime)
{
	if (newSampleTime > 0)
	{
		float ratio = (float)newSampleTime / (float)pid->sampletime;
		pid->Ki *= ratio;
		pid->Kd /= ratio;
		pid->sampletime = (uint32_t)newSampleTime;
	}
}

void PID_SetOutputLimits(PID_t pid, float min, float max)
{
	if (min > max)
		return;
	
	pid->outMin = min;
	pid->outMax = max;
	
	if (pid->output > pid->outMax) pid->output = pid->outMax;
	else if (pid->output < pid->outMin) pid->output = pid->outMin;
 	
	if (pid->iterm > pid->outMax) pid->iterm = pid->outMax;
	else if (pid->iterm < pid->outMin) pid->iterm = pid->outMin;
}

void PID_SetMode(PID_t pid, bool isAuto)
{
	if (!pid->isAutoMode && isAuto)
	{
		PID_Initialize(pid);
	}
	
	pid->isAutoMode = isAuto;
}

void PID_SetDirection(PID_t pid, enum PID_direction direction)
{
	pid->direction = direction;
}

void PID_Setpoint(PID_t pid, float setpoint)
{
	pid->setpoint = setpoint;
}