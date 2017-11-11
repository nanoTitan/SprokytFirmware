#include "differential_drive.h"
#include "Encoder.h"
#include "error.h"
#include "debug.h"
#include "stm32f4xx_hal_conf.h"
#include "math_ext.h"

/* Private variables ---------------------------------------------------------*/
static float m_leftAngVel = 0;
static float m_rightAngVel = 0;
static float m_vehicleRotation = 0;
static float m_vehicleVelocity = 0;
static uint32_t m_lastTime = 0;
static Vector2_t m_icc = {0, 0};	// Instantaneous Center of Curvature (ICC). The point which the robot rotates about

/* Private function prototypes -----------------------------------------------*/


void DiffDrive_Init()
{
	Encoder_Init();
}

/*
*******************************************************************************
DiffDrive_Update

w (R + l/2) = Vr
w (R - l/2) = Vl

R = 1/2 * ((Vl + Vr) / (Vr - Vl))
w = (Vr - Vl) /  l

w - rate of rotation
l - distance between the centers of the two wheels
Vr, Vl - the right and left wheel velocities along the ground
R - the signed distance from the ICC to the midpoint between the wheels
*******************************************************************************
*/
void DiffDrive_Update()
{
	// Update the encoders
	Encoder_Update();
	
	float currTime = HAL_GetTick() * 0.001f;
	
	// Update wheel velocities
	
	
	m_lastTime = currTime;
}

void DiffDrive_SetVehicleRotation(float rot)
{
	m_vehicleRotation = rot;
	
	// Normalize rotation between 0 and 360 degrees
	while (m_vehicleRotation < 0)
	{
		m_vehicleRotation += 360;
	}
	
	while (m_vehicleRotation > 360)
	{
		m_vehicleRotation -= 360;
	}
}