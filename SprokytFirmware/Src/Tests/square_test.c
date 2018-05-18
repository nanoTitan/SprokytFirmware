
/* Includes ------------------------------------------------------------------*/
#include "square_test.h"
#include "differential_drive.h"
#include "rover_control.h"
#include <stdbool.h>
#include <math.h>

/* Private Functions ------------------------------------------------------------------*/
static void SquareTest_nextStep(const Transform_t* pTrans);
static void SquareTest_startStep(const Transform_t* pTrans);

enum SquareTestStep
{
	ST_Side1 = 0,
	ST_Turn1,
	ST_Side2,
	ST_Turn2,
	ST_Side3,
	ST_Turn3,
	ST_Side4,
	ST_Turn4
};

enum SquareTestStepDir
{
	ST_CW = 0,
	ST_CCW,
};

static enum SquareTestStep m_currStep = ST_Side1;
static enum SquareTestStepDir m_currDir = ST_CW;
static float m_length = 0;
static float m_lengthSqr = 0;
static float m_currLengthSqr = 0;
static float m_lastRot = 0;
static const uint32_t kStUpdateTime = 100;
static uint32_t m_lastTime = 0;
static float m_startX = 0, m_startY = 0;
static bool m_done = true;

void SquareTest_start(enum SquareTestStepDir dir, float length)
{
	m_currDir = dir;
	m_currStep = ST_Side1;
	m_length = length > 0 ? length : 0;
	m_lengthSqr = m_length*m_length;
	m_currLengthSqr = 0;
	m_lastRot = 0;
	m_lastTime = 0;
	m_done = false;
	
	const Transform_t* pTrans = RoverControl_getTransform();
	m_startX = pTrans->x;
	m_startY = pTrans->y;
}

void SquareTest_update()
{
	if (m_done)
		return;
	
	uint32_t currTime = HAL_GetTick();
	if (currTime - m_lastTime < kStUpdateTime)
	{
		return;
	}
	
	const Transform_t* pTrans = RoverControl_getTransform();
	
	switch (m_currStep)
	{
	case ST_Side1:
	case ST_Side2:
	case ST_Side3:
	case ST_Side4:
		m_currLengthSqr = vector2_length_sqr(pTrans->x, pTrans->z, m_startX, m_startY);
		if (m_currLengthSqr >= m_lengthSqr)
		{
			SquareTest_nextStep(pTrans);
		}
		break;
		
	case ST_Turn1:
	case ST_Turn2:
	case ST_Turn3:
	case ST_Turn4:
		{
			float currYaw = fabs(pTrans->yaw - m_lastRot);
			if (currYaw > 360)
				currYaw -= 360;
			
			if (currYaw >= 90)
			{
				SquareTest_nextStep(pTrans);
			}
			break;
		}
		break;
		
	default:
		break;
	}
	
	m_lastTime = currTime;
}

void SquareTest_nextStep(const Transform_t* pTrans)
{
	m_startX = pTrans->x;
	m_startY = pTrans->y;
	m_lastRot = pTrans->yaw;
	m_currStep += 1;
	
	if (m_currStep > ST_Turn4)
	{
		m_done = true;
		m_currStep = ST_Side1;
		return;
	}
	
	SquareTest_startStep(pTrans);
}

void SquareTest_startStep(const Transform_t* pTrans)
{
	switch (m_currStep)
	{
	case ST_Side1:
	case ST_Side2:
	case ST_Side3:
	case ST_Side4:
		{
			// Set the forward throttle to a reasonable amount so it doesn't slip
			uint8_t data[3] = {INSTRUCTION_TRANSLATE, 0, 200 };
			RoverControl_parseInstruction(3, data);
			break;
		}
		
	case ST_Turn1:
	case ST_Turn2:
	case ST_Turn3:
	case ST_Turn4:
		{
			// Set the turn throttle to a reasonable amount so it doesn't slip
			float x = m_currDir == ST_CW ? 200 : 54;
			
			uint8_t data[3] = { INSTRUCTION_TRANSLATE, x, 0 };
			RoverControl_parseInstruction(3, data);
			break;
		}
	}
}
