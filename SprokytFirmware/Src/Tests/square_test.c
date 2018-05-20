
/* Includes ------------------------------------------------------------------*/
#include "square_test.h"
#include "differential_drive.h"
#include "rover_control.h"
#include "stm32f4xx_hal.h"
#include "debug.h"
#include <stdbool.h>
#include <math.h>

/* Private Functions ------------------------------------------------------------------*/
static void SquareTest_nextStep(const Transform_t* pTrans);
static void SquareTest_startStep(const Transform_t* pTrans);
static void SquareTest_end();
static void SquareTest_startStepPause();
static void SquareTest_updateStepPause();
static void SquareTest_setMotors(uint8_t x, uint8_t y);

/* Private variables ------------------------------------------------------------------*/
static enum SquareTestStep m_currStep = ST_Side1;
static enum SquareTestStepDir m_currDir = ST_CW;
static float m_length = 0;
static float m_lengthSqr = 0;
static float m_currLengthSqr = 0;
static float m_lastRot = 0;
static uint32_t m_lastTime = 0;
static uint32_t m_stepPauseTime = 0;
static uint32_t m_currStepPauseTime = 0;
static float m_startX = 0, m_startZ = 0;
static bool m_done = true;
static bool m_doStepPause = false;
static bool m_wasPidAuto = false;
static float m_maxVel = 5;
static float m_rotVel = 5;

/* Constants variables ------------------------------------------------------------------*/
static const uint32_t kStUpdateTime = 10;

void SquareTest_start(enum SquareTestStepDir dir, float length, uint32_t stepPauseTime)
{
	m_currDir = dir;
	m_currStep = ST_Side1;
	m_length = length > 0 ? length : 0;
	m_lengthSqr = m_length*m_length;
	m_currLengthSqr = 0;
	m_lastRot = 0;
	m_lastTime = 0;
	m_stepPauseTime = stepPauseTime;
	m_currStepPauseTime = 0;
	m_done = false;
	m_wasPidAuto = DiffDrive_GetPidAuto();
		
	const Transform_t* pTrans = RoverControl_getTransform();
	SquareTest_startStep(pTrans);
}

void SquareTest_update()
{
	if (m_done)
		return;
	
	uint32_t currTime = HAL_GetTick();
	if (currTime - m_lastTime < kStUpdateTime)
		return;
	
	if (m_doStepPause)
	{
		SquareTest_updateStepPause();
		return;
	}
	
	const Transform_t* pTrans = RoverControl_getTransform();
	
	switch (m_currStep)
	{
	case ST_Side1:
	case ST_Side2:
	case ST_Side3:
	case ST_Side4:
		m_currLengthSqr = vector2_length_sqr(pTrans->x, pTrans->z, m_startX, m_startZ);
		if (m_currLengthSqr >= m_lengthSqr)
		{
			SquareTest_nextStep(pTrans);
			//PRINTF("x: %f, z: %f, yaw: %f\n", pTrans->x, pTrans->z, pTrans->yaw);
		}
		break;
		
	case ST_Turn1:
	case ST_Turn2:
	case ST_Turn3:
	case ST_Turn4:
		{
			float currYaw = 0;
			if (m_currDir == ST_CW)
			{
				if (pTrans->yaw < m_lastRot)
					currYaw = pTrans->yaw + 360 - m_lastRot;
				else
					currYaw = pTrans->yaw - m_lastRot;
			}
			else
			{
				if (pTrans->yaw > m_lastRot)
					currYaw = m_lastRot + 360 - pTrans->yaw;
				else
					currYaw = m_lastRot - pTrans->yaw;
			}
			
			
			if (currYaw > 360)
				currYaw -= 360;
			else if(currYaw < 0)
				currYaw += 360;
			
			if (currYaw >= 90)
			{
				SquareTest_nextStep(pTrans);
				PRINTF("currYaw: %f\n", pTrans->yaw);
			}
			break;
		}
		break;
		
	default:
		break;
	}
	
	m_lastTime = currTime;
}

void SquareTest_startStepPause()
{
	m_doStepPause = true;
	m_currStepPauseTime = HAL_GetTick();
}

void SquareTest_updateStepPause()
{
	uint32_t currTime = HAL_GetTick();
	if (currTime - m_currStepPauseTime >= m_stepPauseTime)
	{
		const Transform_t* pTrans = RoverControl_getTransform();
		SquareTest_startStep(pTrans);
		m_doStepPause = false;
	}
}

void SquareTest_nextStep(const Transform_t* pTrans)
{
	// Stop the motors
	SquareTest_setMotors(127, 127);
	
	++m_currStep;
	if (m_currStep > ST_Turn4)
	{
		SquareTest_end();
		return;
	}
	
	// Pause time between steps if set
	if (m_stepPauseTime > 0)
	{
		SquareTest_startStepPause();
	}
	else
	{
		SquareTest_startStep(pTrans);
	}
}

void SquareTest_startStep(const Transform_t* pTrans)
{
	m_startX = pTrans->x;
	m_startZ = pTrans->z;
	m_lastRot = pTrans->yaw;
	
	PRINTF("currYaw: %f\n", m_lastRot);
	
	switch (m_currStep)
	{
	case ST_Side1:
	case ST_Side2:
	case ST_Side3:
	case ST_Side4:
		{
			// Set the forward throttle to a reasonable amount so it doesn't slip
			SquareTest_setMotors(127, 127-m_maxVel);
			break;
		}
		
	case ST_Turn1:
	case ST_Turn2:
	case ST_Turn3:
	case ST_Turn4:
		{
			// Set the turn throttle to a reasonable amount so it doesn't slip
			float x = m_currDir == ST_CW ? 127 - m_rotVel : 127 + m_rotVel;			
			SquareTest_setMotors(x, 127);
			break;
		}
	}
}

void SquareTest_end()
{
	m_done = true;
	m_currStep = ST_Side1;
	DiffDrive_SetPidAuto(m_wasPidAuto);
}

void SquareTest_setMotors(uint8_t x, uint8_t y)
{
	if (x == 127 && y == 127)
	{
		DiffDrive_SetPidAuto(false);
	}
	else
	{
		DiffDrive_SetPidAuto(true);
	}
	
	uint8_t data[3] = { INSTRUCTION_TRANSLATE, x, y };
	RoverControl_parseInstruction(3, data);
}