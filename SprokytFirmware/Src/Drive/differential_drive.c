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
static Vector2_t m_vehiclePosition = { 0, 0 };
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
theta = dT / b * (Vr - Vl)

w - rate of rotation of the vehicle (angular velocity)
theta - Instantaneous rotation of vehicle
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
	
	// Update once every 10ms
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
	
	// Calculate the instantaneous rotation of the vehicle
	float theta = m_angVelocity * deltaTime;
	
	// Add the instantaneous rotation to our total to update the current angle
	m_angPosition = m_angPosition + theta;
	
	// Normalize the angular position between 0-2pi (0° - 360°)
	if (m_angPosition > M_2PI)
		m_angPosition -= M_2PI;
	else if(m_angPosition < 0)
		m_angPosition += M_2PI;
	
	/*
	Compute new position
	
	px = cos(theta/2) * (2Rsin(theta/2))
	py = sin(theta/2) * (2Rsin(theta/2))
	*/
	
	float thetaOver2 = theta * 0.5f;
	float cosThetaOver2 = cosf(thetaOver2);
	float sinThetaOver2 = sinf(thetaOver2);
	float twoRsinThetaOver2 = 2 * R * sinThetaOver2;
	
	m_vehiclePosition.x = m_vehiclePosition.x + (cosThetaOver2 * twoRsinThetaOver2);
	m_vehiclePosition.y = m_vehiclePosition.y + (sinThetaOver2 * twoRsinThetaOver2);
	
	m_lastTime = currTime;
#endif // ENCODER_ENABLED
	
#ifdef PRINT_DIFF_DRIVE
	// Print IMU values for testing
	static int printCnt = 0;
	++printCnt;
	if (printCnt > 5)
	{
#if defined(IMU_ENABLED)
		float yaw, pitch, roll;
		IMU_get_yawPitchRoll(&yaw, &pitch, &roll);
		PRINTF("%3.1f, %3.1f, %3.1f\n", yaw, pitch, roll);		
#endif // IMU_ENABLED
		
		//PRINTF("%1.2f, %1.2f\n", Vl, Vr);	
		//PRINTF("%1.2f, %1.2f\n", m_angVelocity, m_angPosition);	
		//PRINTF("%.2f, %.2f\n", m_vehiclePosition.x, m_vehiclePosition.y);	
		
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
	// A needs to turn CCW to move forward, and B should turn clockwise
	
	direction_t dir = FWD;
	float x = mapf(_x, 0, 255, -1, 1);
	float y = mapf(_y, 0, 255, -1, 1);
	
	PRINTF("%1.2f, %1.2f\n", x, y);	
	
	float left = 0;
	float right = 0;
		
	if (x > 0)
	{
		if (y > 0)
		{		
			right = y - x;
			if (right < 0)
			{
				dir = BWD;
				right = -right;
			}	
			
			left = fmaxf(x, y);
			
			MotorController_setMotor(MOTOR_A, left, BWD);	// Spin opposite way to account for reversed motor
			MotorController_setMotor(MOTOR_B, right, dir);
		}
		else
		{
			dir = BWD;
			left = x + y;
			if (left < 0)
			{
				dir = FWD;
				left = -left;
			}
			
			right = fmaxf(x, -y);
				
			MotorController_setMotor(MOTOR_A, left, dir);	// Spin opposite way to account for reversed motor
			MotorController_setMotor(MOTOR_B, right, BWD);
		}
	}
	else
	{
		if (y > 0)
		{
			dir = BWD;
			left = x + y;
			if (left < 0)
			{
				dir = FWD;
				left = -left;
			}	
			
			right = fmaxf(-x, y);
			
			MotorController_setMotor(MOTOR_A, left, dir);	// Spin opposite way to account for reversed motor
			MotorController_setMotor(MOTOR_B, right, FWD);
		}
		else
		{
			right = y - x;
			if (right < 0)
			{
				dir = BWD;
				right = -right;
			}	
			
			left = fmaxf(-x, -y);
				
			MotorController_setMotor(MOTOR_A, left, FWD);	// Spin opposite way to account for reversed motor
			MotorController_setMotor(MOTOR_B, right, dir);
		}
	}
}