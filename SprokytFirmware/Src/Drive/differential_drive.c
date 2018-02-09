#include "differential_drive.h"
#include "Encoder.h"
#include "error.h"
#include "debug.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "constants.h"
#include "motor_controller.h"
#include "imu.h"
#include "BLE.h"

#define PRINT_DIFF_DRIVE

/* Private variables ---------------------------------------------------------*/
static float m_leftAngVel = 0;
static float m_rightAngVel = 0;
static float m_vehicleVelocity = 0;
static float m_angVelocity = 0;
static float m_angPosition = 0;
static float m_lastTime = 0;
static Transform_t m_transform = { 0 };
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
#if !defined(ENCODER_ENABLED)
	return;
#endif // ENCODER_ENABLED
	
	float currTime = HAL_GetTick() * 0.001f;
	float deltaTime = currTime - m_lastTime;
	
	// Update once every 25ms
	if (deltaTime < 0.025f)
	{
		return;
	}
	
	// Update the encoders frequently so the counts and velocities are accurate
	Encoder_Update();
	
	// Update wheel velocities
	// V = wR  translational velocity of wheel center is rotational velocity * wheel radius
	// Vl is negated to account for reversed motor direction (wheel on left spins CCW to go forward)
	float Vl = -Encoder_GetAngVel1() * DD_WHEEL_RADIUS;
	float Vr = Encoder_GetAngVel2() * DD_WHEEL_RADIUS;
	
	float R = 0;
	float VrMinusVl = Vr - Vl;
	
	// Prevent divide by zero, and also know if R is exact center
	if (VrMinusVl != 0)
		R = DD_Half_Wheel_Base_Length * ((Vl + Vr) / VrMinusVl);
	
	float angVel = VrMinusVl * DD_One_Over_Wheel_Base_Length;
	
	// Calculate the instantaneous rotation of the vehicle
	float theta = angVel * deltaTime;
	
	/*
	Compute new position
	
	px = cos(theta/2) * (2Rsin(theta/2))
	py = sin(theta/2) * (2Rsin(theta/2))
	*/
	
	float cosTheta = cosf(theta);
	float sinTheta = sinf(theta);
	float RsinAng = R * sinf(m_angPosition);
	float RcosAng = R * cosf(m_angPosition);
	
	m_transform.x = m_transform.x + cosTheta * RsinAng + sinTheta * RcosAng - RsinAng;
	m_transform.y = 0;
	m_transform.z = m_transform.z + sinTheta * RsinAng - cosTheta * RcosAng + RcosAng;
	
	/*
	Compute new angular position
	Add the instantaneous rotation to our total to update the current angle and normalize
	Use negative so that CW is positive angular velocity
	*/
	m_angPosition = m_angPosition + theta;
	m_angVelocity = -angVel;
	
	if (m_angPosition > M_2PI)
		m_angPosition -= M_2PI;
	else if (m_angPosition < 0)
		m_angPosition += M_2PI;
	
	/*
	We need to report CW as positive angle. So subtract our current angle from 360 to do that
	Also, 0 degrees should point towards the positive X-axis. Add 90 degrees to account for 
	the listener interpreting 0 degrees as positive Z-axis
	*/
	m_transform.yaw = RadiansToDeg(m_angPosition);
	m_transform.pitch = 3.3f;
	m_transform.roll = 4.4f;
	
	// Update the time
	m_lastTime = currTime;

	static int printCnt = 0;
	++printCnt;
	if (printCnt > 10)
	{		
#ifdef PRINT_DIFF_DRIVE
		//PRINTF("%.3f, %.3f\n", Vl, Vr);	
		//PRINTF("%.2f, %.2f\n", m_angVelocity, m_angPosition);	
		//PRINTF("%.2f, %.2f, %.2f\n", m_transform.yaw, m_transform.pitch, m_transform.roll);		
		//PRINTF("%.2f, %.2f, %.2f\n", m_transform.x, m_transform.z, m_transform.yaw);
		//PRINTF("%.2f %.2f\n", m_angPosition, m_transform.yaw);	
#endif // PRINT_DIFF_DRIVE
		
		printCnt = 0;
	}
}

void DiffDrive_SetAngularPosDegree(float angle)
{
	m_angPosition = DegToRadians(angle);
	m_transform.yaw = angle;
	
	// Normalize rotation between 0 and 360 degrees
	while (m_angPosition < 0)
		m_angPosition += 360;
	
	while (m_angPosition > 360)
		m_angPosition -= 360;
}

void DiffDrive_SetPos(float x, float z)
{
	m_transform.x = x;
	m_transform.z = z;
}

void DiffDrive_ParseTranslate(uint8_t _x, uint8_t _y)
{
	// A (left) - B (right)
	// A needs to turn CCW to move forward, and B should turn clockwise
	
	direction_t dir = FWD;
	float x = mapf(_x, 0, 255, -1, 1);
	float y = mapf(_y, 0, 255, -1, 1);
	
	//PRINTF("%1.2f, %1.2f\n", x, y);	
	
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

const Transform_t* DiffDrive_GetTransform()
{
	return &m_transform;
}