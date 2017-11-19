#include "differential_drive.h"
#include "Encoder.h"
#include "error.h"
#include "debug.h"
#include "math_ext.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "constants.h"
#include "motor_controller.h"
#include "imu.h"

#define PRINT_DIFF_DRIVE

/* Private variables ---------------------------------------------------------*/
static float m_leftAngVel = 0;
static float m_rightAngVel = 0;
static float m_vehicleRotation = 0;
static float m_vehicleVelocity = 0;
static float m_angVelocity = 0;
static float m_angPosition = 0;
static float m_lastTime = 0;
static Vector2_t m_icc = {0, 0};	// Instantaneous Center of Curvature (ICC). The point which the robot rotates about
const float DD_Half_Wheel_Base_Length = DD_WHEEL_BASE_LENGTH * 0.5f;
const float DD_One_Over_Wheel_Base_Length = 1.0f / DD_WHEEL_BASE_LENGTH;

/* Private function prototypes -----------------------------------------------*/


void DiffDrive_Init()
{
#if defined(ENCODER_ENABLED)
	Encoder_Init();
#endif // ENCODER_ENABLED
}

/*
*******************************************************************************
DiffDrive_Update

w (R + l/2) = Vr
w (R - l/2) = Vl

R = l/2 * ((Vl + Vr) / (Vr - Vl))
w = (Vr - Vl) /  l

w - rate of rotation of the vehicle (angular velocity)
l - distance between the centers of the two wheels
Vr, Vl - the right and left wheel translational velocities along the ground
R - the signed distance from the Instantaneous Center of Curvature (ICC) to the midpoint between the wheels
**********************************************************************************************************************
*/
void DiffDrive_Update()
{
#if defined(ENCODER_ENABLED)
	
	float currTime = HAL_GetTick() * 0.001f;
	float deltaTime = currTime - m_lastTime;
	
	// Update once every 100ms
	if (deltaTime < 0.01f)
	{
		return;
	}
	
	// Update the encoders
	Encoder_Update();
	
	// Update wheel velocities
	// V = wR  translational velocity of wheel center is rotational velocity * wheel radius
	float Vl = Encoder_GetAngVel1() * DD_WHEEL_RADIUS;
	float Vr = Encoder_GetAngVel2() * DD_WHEEL_RADIUS;
	
	float R = 0;
	float VrMinusVl = Vr - Vl;
	
	// Prevent divide by zero, and also know if R is exact center
	if (VrMinusVl != 0)
		R = DD_Half_Wheel_Base_Length * ((Vl + Vr) / VrMinusVl);
	
	m_angVelocity = VrMinusVl * DD_One_Over_Wheel_Base_Length;
	m_angPosition = m_angPosition + m_angVelocity * deltaTime;
	
	// Normalize the angular position between 0-2pi (0° - 360°)
	if (m_angPosition > M_2PI)
		m_angPosition -= M_2PI;
	else if(m_angPosition < 0)
		m_angPosition += M_2PI;
	
	m_lastTime = currTime;
#endif // ENCODER_ENABLED
	
#ifdef PRINT_DIFF_DRIVE
	// Print IMU values for testing
	static int printCnt = 0;
	++printCnt;
	if (printCnt > 0)
	{
#if defined(IMU_ENABLED)
		float yaw, pitch, roll;
		IMU_get_yawPitchRoll(&yaw, &pitch, &roll);
		PRINTF("%3.1f, %3.1f, %3.1f\n", yaw, pitch, roll);		
#endif // IMU_ENABLED
		
		PRINTF("%3.3f, %3.3f\n", m_angVelocity, m_angPosition);	
		
		printCnt = 0;
	}
#endif // PRINT_DIFF_DRIVE
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

void DiffDrive_ParseTranslate(uint8_t _x, uint8_t _y)
{
	// A (left) - B (right)
	
	// TODO: Save the speeds so we can adjust set velocity against measured velocity to compensate for error
	
	direction_t dir = FWD;
	float x = mapf(_x, 0, 255, -1, 1);
	float y = mapf(_y, 0, 255, -1, 1);
	
	float e = x;
	if (fabs(y) > fabs(x))
		e = y;
		
	if (e < 0)
		e = -e;
		
	if (x > 0)
	{
		if (y > 0)
		{		
			float d = y - x;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, d, dir);
			MotorController_setMotor(MOTOR_B, e, FWD);
		}
		else
		{
			float d = x + y;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, e, BWD);
			MotorController_setMotor(MOTOR_B, d, dir);
		}
	}
	else
	{
		if (y > 0)
		{
			float d = x + y;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, e, FWD);
			MotorController_setMotor(MOTOR_B, d, dir);
		}
		else
		{
			float d = y - x;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, d, dir);
			MotorController_setMotor(MOTOR_B, e, BWD);
		}
	}
}