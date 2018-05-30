
/* Includes ------------------------------------------------------------------*/
#include "square_test.h"
#include "differential_drive.h"
#include "rover_control.h"
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "PID/pid.h"
#include "motor_controller.h"
#include <stdbool.h>
#include <math.h>

/* Private Functions ------------------------------------------------------------------*/
static void SquareTest_nextStep(const Transform_t* pTrans);
static void SquareTest_startStep(const Transform_t* pTrans);
static void SquareTest_end();
static void SquareTest_startStepPause();
static void SquareTest_updateStepPause();
static void SquareTest_setMotors(uint8_t x, uint8_t y);
static void UpdatePIDControllers(const Transform_t* pTrans);

/* Private variables ------------------------------------------------------------------*/
static enum SquareTestStep m_currStep = ST_Side1;
static enum SquareTestStepDir m_currDir = ST_CW;
static enum SquareTestStepDir m_mainDir = ST_CW;
static float m_length = 0;
static float m_lengthSqr = 0;
static float m_currLengthSqr = 0;
static uint32_t m_lastTime = 0;
static uint32_t m_stepPauseTime = 0;
static uint32_t m_currStepPauseTime = 0;
static float m_startX = 0, m_startZ = 0;
static bool m_done = true;
static bool m_doStepPause = false;
static bool m_wasPidAuto = false;
static bool m_hasPastSetpoint = false;
static float m_lastYaw = 0;
static float m_maxVel = 10;
static float m_maxRotVel = 10;
static float m_minRotVel = 0.04f;
static float m_posPID = 0;
static struct PID m_pidAngleController;

/* Constants variables ------------------------------------------------------------------*/
static const uint32_t kStUpdateTime = 10;
static const float m_kDeltaYawTolerance = 0.3f;
static const float m_kP = 0.001f;
static const float m_kI = 0.000f;
static const float m_kD = 0.0000f;

void SquareTest_start(enum SquareTestStepDir dir, float length, uint32_t stepPauseTime)
{
	m_currDir = dir;
	m_mainDir = dir;
	m_currStep = ST_Side1;
	m_length = length > 0 ? length : 0;
	m_lengthSqr = m_length*m_length;
	m_currLengthSqr = 0;
	m_lastTime = 0;
	m_stepPauseTime = stepPauseTime;
	m_currStepPauseTime = 0;
	m_done = false;
	m_wasPidAuto = DiffDrive_GetPidAuto();
	DiffDrive_SetPidAuto(true);
	
	// PIDs
	PID_Create(&m_pidAngleController, 0, 0, 0, -MAX_MOTOR_VEL_COUNT, MAX_MOTOR_VEL_COUNT);
	PID_SetTunings(&m_pidAngleController, m_kP, m_kI, m_kD);
	PID_SetMode(&m_pidAngleController, true);
	
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
	
	// Translate forward
	if (m_currStep == ST_Side1 || m_currStep == ST_Side2 || m_currStep == ST_Side3 || m_currStep == ST_Side4)
	{
		//PRINTF("ST Side x: %f, z: %f, yaw: %f\n", pTrans->x, pTrans->z, pTrans->yaw);
		
		m_currLengthSqr = vector2_length_sqr(pTrans->x, pTrans->z, m_startX, m_startZ);
		if (m_currLengthSqr >= m_lengthSqr)
		{
			SquareTest_nextStep(pTrans);
		}
	}	// Turn
	else if (m_currStep == ST_Turn1 || m_currStep == ST_Turn2 || m_currStep == ST_Turn3 || m_currStep == ST_Turn4)
	{
		// Have we rotated past the setpoint from the starting point
		if (m_currDir == ST_CW && m_pidAngleController.setpoint > pTrans->yaw && m_pidAngleController.setpoint < m_lastYaw)
		{
			m_hasPastSetpoint = true;
			m_currDir = ST_CCW;
		}
		else if (m_currDir == ST_CCW && m_pidAngleController.setpoint < pTrans->yaw && m_pidAngleController.setpoint > m_lastYaw)
		{
			m_hasPastSetpoint = true;
			m_currDir = ST_CW;
		}
		
		UpdatePIDControllers(pTrans);
		
		float deltaYaw = fabsf(pTrans->yaw - m_pidAngleController.setpoint);
		if (deltaYaw > 180)
			deltaYaw = fabsf(deltaYaw - 360);
		
		if (deltaYaw < m_kDeltaYawTolerance)
		{
			SquareTest_nextStep(pTrans);
		}
	}
	
	m_lastYaw = pTrans->yaw;
	m_lastTime = currTime;	
}

static void UpdatePIDControllers(const Transform_t* pTrans)
{
	// Update the PID controllers
	if (!m_pidAngleController.isAutoMode)
		return;
	
	if (PID_CanCompute(&m_pidAngleController))
	{
		if (m_currDir == ST_CW)
		{
			if (m_pidAngleController.setpoint > pTrans->yaw)
				m_pidAngleController.input = pTrans->yaw + 360;
			else
				m_pidAngleController.input = pTrans->yaw;
		}
		else
		{
			if (m_pidAngleController.setpoint < pTrans->yaw)
				m_pidAngleController.input = pTrans->yaw - 360;
			else
				m_pidAngleController.input = pTrans->yaw;	
		}
		
		PID_Compute(&m_pidAngleController);
		
		if (m_pidAngleController.output > 0 && m_pidAngleController.output < m_minRotVel)
			m_pidAngleController.output = m_minRotVel;
		else if (m_pidAngleController.output < 0 && m_pidAngleController.output > -m_minRotVel)
			m_pidAngleController.output = -m_minRotVel;
		
		float vel = m_pidAngleController.output / 0.36f * m_maxRotVel;
		float x = 127 + vel;
		SquareTest_setMotors(x, 127);
		
		//PRINTF("ST Turn: sp: %.3f, in: %.3f, out: %.3f\n", m_pidAngleController.setpoint, m_pidAngleController.input, m_pidAngleController.output);
	}
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
	PRINTF("Next: %d *******************\n", m_currStep);
	
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
	m_currDir = m_mainDir;
	m_hasPastSetpoint = false;
	m_startX = pTrans->x;
	m_startZ = pTrans->z;
	m_posPID = pTrans->yaw;
	m_lastYaw = pTrans->yaw;
	
	PRINTF("Start: %d *******************\n", m_currStep);
	
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
			if (m_currDir == ST_CW)
				m_pidAngleController.setpoint = pTrans->yaw - 90;
			else
				m_pidAngleController.setpoint = pTrans->yaw + 90;
			
			if (m_pidAngleController.setpoint > 360)
				m_pidAngleController.setpoint -= 360;
			else if (m_pidAngleController.setpoint < 0)
				m_pidAngleController.setpoint += 360;
			
			// Set the turn throttle to a reasonable amount so it doesn't slip
			//float x = m_currDir == ST_CW ? 127 - m_maxRotVel : 127 + m_maxRotVel;			
			//SquareTest_setMotors(x, 127);
			break;
		}
	}
}

void SquareTest_end()
{
	m_done = true;
	m_currStep = ST_Side1;
	SquareTest_setMotors(127, 127);
}

void SquareTest_setMotors(uint8_t x, uint8_t y)
{
	if (x == 127 && y == 127)
	{
		DiffDrive_SetPidAuto(false);
		MotorController_setMotor(MOTOR_A, 0, FWD);	
		MotorController_setMotor(MOTOR_B, 0, FWD);
		return;
	}
	
	DiffDrive_SetPidAuto(true);
	uint8_t data[3] = { INSTRUCTION_TRANSLATE, x, y };
	RoverControl_parseInstruction(3, data);
}