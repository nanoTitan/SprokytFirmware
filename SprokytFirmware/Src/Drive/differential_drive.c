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
#include "PID/pid.h"
#include "stm32f4xx_hal_conf.h"


/* Private variables ---------------------------------------------------------*/
static float m_leftVel = 0;
static float m_rightVel = 0;
static direction_t m_leftDir = FWD;
static direction_t m_rightDir = FWD;
static float m_leftX = 0;
static float m_leftAngVel = 0;
static float m_rightAngVel = 0;
static float m_vehicleVelocity = 0;
static float m_angVelocity = 0;
static float m_angPosition = 0;
static float m_velLeft = 0;
static float m_velRight = 0;
static float m_lastTime = 0;
static Transform_t m_transform = { 0 };
static Vector2_t m_vehiclePosition = { 0, 0 };
static Vector2_t m_icc = {0, 0};	// Instantaneous Center of Curvature (ICC). The point which the robot rotates about
static DiffDriveCallback m_ddFuncCallback = NULL;
const float DD_Half_Wheel_Base_Length = DD_WHEEL_BASE_LENGTH * 0.5f;
const float DD_One_Over_Wheel_Base_Length = 1.0f / DD_WHEEL_BASE_LENGTH;

static struct PID m_pidLeft;
static struct PID m_pidRight;
static bool m_pidAuto = true;

static const float m_kP = 0.001f;
static const float m_kI = 0.0000f;
static const float m_kD = 0.0000f;

/* Private function prototypes -----------------------------------------------*/
static void UpdatePIDControllers();

void DiffDrive_Init()
{
#if !defined(ENCODER_ENABLED)
	return;
#endif // ENCODER_ENABLED
	
	Encoder_Init();
	
	// PIDs
	PID_Create(&m_pidLeft, 0, 0, 0, -MAX_MOTOR_VEL_COUNT, MAX_MOTOR_VEL_COUNT);
	PID_SetTunings(&m_pidLeft, m_kP, m_kI, m_kD);
	PID_SetMode(&m_pidLeft, m_pidAuto);
	
	PID_Create(&m_pidRight, 0, 0, 0, -MAX_MOTOR_VEL_COUNT, MAX_MOTOR_VEL_COUNT);
	PID_SetTunings(&m_pidRight, m_kP, m_kI, m_kD);
	PID_SetMode(&m_pidRight, m_pidAuto);
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
	
	// Update the encoders frequently so the counts and velocities are accurate
	Encoder_Update();	
	UpdatePIDControllers();
	
	float currTime = HAL_GetTick() * 0.001f;
	float deltaTime = currTime - m_lastTime;
	
	// Update once every 25ms
	if (deltaTime < 0.025f)
	{
		return;
	}
	
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
	
	// Update the registered callback
	if (m_ddFuncCallback)
		m_ddFuncCallback(&m_transform);
	
	// Update the time
	m_lastTime = currTime;

#ifdef PRINT_DIFF_DRIVE
	static float lastPrintTime = 0;
	if (currTime - lastPrintTime > 0.1f)
	{		
		//PRINTF("%.3f, %.3f\n", Vl, Vr);	
		//PRINTF("%.2f, %.2f\n", m_angVelocity, m_angPosition);	
		//PRINTF("%.2f, %.2f, %.2f\n", m_transform.yaw, m_transform.pitch, m_transform.roll);		
		PRINTF("%.2f, %.2f, %.2f\n", m_transform.x, m_transform.z, m_transform.yaw);
		//PRINTF("%.2f %.2f\n", m_angPosition, m_transform.yaw);	
		//PRINTF("R: %.3f, %.3f, %.3f\n", m_pidRight.input, m_pidRight.output, m_pidRight.setpoint);
		
		lastPrintTime = currTime;
	}
#endif // PRINT_DIFF_DRIVE
}

void DiffDrive_SetPidAuto(bool isAuto)
{
	m_pidAuto = isAuto;
}

bool DiffDrive_GetPidAuto()
{
	return m_pidAuto;
}

static void UpdatePIDControllers()
{
	// Update the PID controllers
	if (!m_pidAuto)
		return;
	
	if (PID_CanCompute(&m_pidRight))
	{
		m_pidLeft.input = Encoder_GetAngVel1();
		m_pidRight.input = Encoder_GetAngVel2();
		
		// PID inputs should be positive
		if (m_pidLeft.input < 0)
			m_pidLeft.input = -m_pidLeft.input;
		
		if (m_pidRight.input < 0)
			m_pidRight.input = -m_pidRight.input;
		
//		m_pidLeft.setpoint = 10;
//		m_pidRight.setpoint = 10;
//		m_pidLeft.input = 1;
//		m_pidRight.input = 1;
//		m_rightDir = BWD;
		
		PID_Compute(&m_pidLeft);
		PID_Compute(&m_pidRight);
		
		m_velLeft += m_pidLeft.output * MAX_MOTOR_VEL_COUNT;
		m_velRight += m_pidRight.output * MAX_MOTOR_VEL_COUNT;			// <------------ TODO: This should probably be accumulating, because as error goes down, so does output
		
		if (m_velLeft < 0) m_velLeft = 0;
		else if (m_velLeft > MAX_MOTOR_VEL_COUNT) m_velLeft = MAX_MOTOR_VEL_COUNT;
		
		if (m_velRight < 0) m_velRight = 0;
		else if (m_velRight > MAX_MOTOR_VEL_COUNT) m_velRight = MAX_MOTOR_VEL_COUNT;
		
		MotorController_setMotor(MOTOR_A, m_velLeft, m_leftDir);
		MotorController_setMotor(MOTOR_B, m_velRight, m_rightDir);
		
		//PRINTF("RV: %.3f, %.3f, %i\n", m_pidRight.input, m_velRight, m_rightDir);
		//PRINTF("RV: %.3f, %.3f, %.3f\n", m_pidRight.input, m_pidRight.output, m_pidRight.setpoint);
	}
}

void DiffDrive_RegisterCallback(DiffDriveCallback callback)
{
	m_ddFuncCallback = callback;
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
	
	float x = mapf(_x, 0, 254, -1, 1);		// In max at 254 (instead of 255) so that halfway of 127 will translate to 0 in mapf function
	float y = mapf(_y, 0, 254, -1, 1);
	
	m_leftVel = 0;
	m_rightVel = 0;
	
	if (x > 0)
	{
		if (y > 0)
		{		
			m_rightVel = y - x;
			if (m_rightVel < 0)
			{
				m_rightDir = BWD;			// Spin opposite way to account for reversed motor
				m_rightVel = -m_rightVel;
			}
			else if (m_rightVel > 0)
			{
				m_rightDir = FWD;			// Only change direction if we have to
			}
			
			m_leftVel = fmaxf(x, y);
			m_leftDir = BWD;
		}
		else
		{
			m_leftVel = x + y;
			if (m_leftVel < 0)
			{
				m_leftDir = FWD;		// Spin opposite way to account for reversed motor
				m_leftVel = -m_leftVel;
			}
			else if (m_leftVel > 0)
			{
				m_leftDir = BWD;			// Only change direction if we have to
			}
			
			m_rightVel = fmaxf(x, -y);
			m_rightDir = BWD;
		}
	}
	else
	{
		if (y > 0)
		{
			m_leftVel = x + y;
			if (m_leftVel < 0)
			{
				m_leftDir = FWD;			// Spin opposite way to account for reversed motor
				m_leftVel = -m_leftVel;
			}
			else if (m_leftVel > 0)
			{
				m_leftDir = BWD;			// Only change direction if we have to
			}
			
			m_rightVel = fmaxf(-x, y);
			m_rightDir = FWD;
		}
		else
		{
			m_rightVel = y - x;
			if (m_rightVel < 0)
			{
				m_rightDir = BWD;			// Spin opposite way to account for reversed motor
				m_rightVel = -m_rightVel;
			}
			else if (m_rightVel > 0)
			{
				m_rightDir = FWD;			// Only change direction if we have to
			}
			
			m_leftVel = fmaxf(-x, -y);
			
			if (m_leftVel > 0)
			{
				m_leftDir = FWD;
			}	
			else if (m_leftVel < 0)
			{
				m_leftDir = BWD;			// Only change direction if we have to
			}	
		}
	}
	
	if (m_pidAuto)
	{
		PID_Setpoint(&m_pidLeft, m_leftVel * MAX_MOTOR_VEL_COUNT);
		PID_Setpoint(&m_pidRight, m_rightVel * MAX_MOTOR_VEL_COUNT);
	}
	else
	{
		MotorController_setMotor(MOTOR_A, m_leftVel, m_leftDir);	
		MotorController_setMotor(MOTOR_B, m_rightVel, m_rightDir);
	}
}

const Transform_t* DiffDrive_GetTransform()
{
	return &m_transform;
}