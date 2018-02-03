#include "rover_control.h"
#include "control_manager.h"
#include "motor_controller.h"
#include "BLE.h"
#include "differential_drive.h"
#include "TinyEKF.h"
//#include "Wifi.h"
#include "math_ext.h"
#include "debug.h"
#include "imu.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Definitions ---------------------------------------------------------*/
#define YAW_DIFF_MAX	10

/* Private variables ---------------------------------------------------------*/
static TinyEKF m_ekf;
static Transform_t m_trans;
static Transform_t m_lastTrans;
static const Transform_t* m_ddTrans;
static float m_currImuYaw = 0;
static uint8_t m_x = 0;
static uint8_t m_y = 0;
static BOOL m_doUpdate = FALSE;
static BOOL m_hasSetInitialStates = FALSE;

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnected();
static void UpdateDisconnected();
static void InitSensorFusion();
static void UpdateSensorFusion();
static void UpdateOrientationRounding(float* out_imuYaw, float* out_ddYaw);
static uint8_t UpdateTrackingError();
static void Disarm();
static void RunMotorTest();
static void PrintIMU();
static void ParseTranslateQuadDrive(uint8_t _x, uint8_t _y);

/* Private functions ---------------------------------------------------------*/
void RoverControl_init()
{	
	memset(&m_trans, 0, sizeof(Transform_t));
	memset(&m_lastTrans, 0, sizeof(Transform_t));
	
	DiffDrive_Init();
	m_ddTrans = DiffDrive_GetTransform();
	
	InitSensorFusion();
}

void InitSensorFusion()
{
	TinyEKF_init(&m_ekf);
	//TinyEKF_print(&m_ekf);
}

void RoverControl_update()
{	
	CONTROL_STATE state = ControlMgr_getState();
	
// Debug flight control without begin connected
#if defined(DEBUG_FLIGHT_CONTROL_NO_CONNECT)
	state = CONTROL_STATE_CONNECTED;
#endif 
	
	switch (state)
	{
	case CONTROL_STATE_IDLE:
		break;
		
	case CONTROL_STATE_CONNECTED:
		UpdateConnected();
		break;
		
	case CONTROL_STATE_DISCONNECTED:
		UpdateDisconnected();
		break;
		
	default:
		break;
	}
	
	DiffDrive_Update();
	
#if defined(IMU_ENABLED)
	uint8_t isStable = IMU_get_sensorFusionStable();
	
	// Update the differential drive angular position with the IMU on start
	if (!m_hasSetInitialStates && isStable)
	{
		m_currImuYaw = IMU_get_yaw();
		
		DiffDrive_SetAngularPosDegree(m_currImuYaw);
		
		TinyEKF_setX(&m_ekf, 0, m_ddTrans->x);			// Initial state x
		TinyEKF_setX(&m_ekf, 1, m_ddTrans->z);			// Initial state y
		TinyEKF_setX(&m_ekf, 2, m_currImuYaw);			// Initial state yaw
		
		m_hasSetInitialStates = TRUE;
	}
	
	// Don't update sensor fusion if we're not stable or states haven't been set
	if (!m_hasSetInitialStates || !isStable)	
		return;
#endif // IMU_ENABLED
	
	UpdateSensorFusion();
	UpdateTrackingError();
	
	// Send Transform to BLE
	static int i = 0;
	++i;
	if ( i > 10 && 
		(m_trans.x != m_lastTrans.x ||
		m_trans.z != m_lastTrans.z ||
		m_trans.yaw != m_lastTrans.yaw) )
	{
		BLE_PositionUpdate(&m_trans);	
		memcpy(&m_lastTrans, &m_trans, sizeof(Transform_t));
		
		i = 0;
	}
}

void UpdateConnected()
{
	//RunMotorTest();
	//PrintIMU();
	
	if (m_doUpdate)
	{
		DiffDrive_ParseTranslate(m_x, m_y);
		m_doUpdate = FALSE;

		//PRINTF("x: %u, y: %u\r\n", m_x, m_y);
	}
}

void UpdateDisconnected()
{
	// TODO: Show flashing LEDs if connection is lost
	
	Disarm();
}

void UpdateSensorFusion()
{
	double uwb_x = 10;
	double uwb_z = 10;
	float imuYaw, ddYaw;
	float posScale = 10.0f;
	
#if defined(IMU_ENABLED)
	m_currImuYaw = IMU_get_yaw();
#endif // IMU_ENABLED
	
	static uint32_t lastTime = 0;
	uint32_t currTime = HAL_GetTick();
	if (currTime - lastTime > 5)
	{
		UpdateOrientationRounding(&imuYaw, &ddYaw);
		
		double z[6] = { uwb_x, uwb_z, m_ddTrans->x, m_ddTrans->z, imuYaw, ddYaw };
		TinyEKF_step(&m_ekf, z);
		
		// Switch and negate x,z coordinates so they show up correctly in app coordinates
#if defined(UWB_ENABLED)
		m_trans.x = -TinyEKF_getX(&m_ekf, 1) * posScale;
		m_trans.z = -TinyEKF_getX(&m_ekf, 0) * posScale;
#else
		m_trans.x = -m_ddTrans->z * posScale;
		m_trans.z = -m_ddTrans->x * posScale;
#endif	// UWB_ENABLED
		
#if defined(IMU_ENABLED)
		m_trans.yaw = TinyEKF_getX(&m_ekf, 2);
		//m_trans.yaw = imuYaw * 0.1f + ddYaw * 0.9f;
#else
		m_trans.yaw = m_ddTrans->yaw;
#endif	// IMU_ENABLED
		
		static uint32_t printTime = 0;
		printTime += currTime - lastTime;
		if (printTime > 50)
		{
#if defined(UWB_ENABLED)
			//PRINTF("SF ux %.2f, uz %.2f, ddx %.2f, ddz: %.2f, sf_x %.1f, sf_z %.1f\r\n", uwb_x, uwb_z, m_ddTrans->x, m_ddTrans->z, m_trans.x, m_trans.z);
#endif
			//PRINTF("SF imu: %.1f dd: %.1f sf: %.1f x: %.1f z: %.1f\r\n", m_currImuYaw, m_ddTrans->yaw, m_trans.yaw, m_ddTrans->x, m_ddTrans->z);
			//PRINTF("imu: %.1f dd: %.1f curr: %.1f\n", imuYaw, ddYaw, m_trans.yaw);
			PRINTF("x: %.1f y: %.1f z: %.1f yaw: %.1f\n", m_trans.x, m_trans.y, m_trans.z, m_trans.yaw);
			printTime = 0;
		}
		
		lastTime = currTime;
	}
}

void UpdateOrientationRounding(float* out_imuYaw, float* out_ddYaw)
{
	float currYaw = TinyEKF_getX(&m_ekf, 2);
	
	if (m_currImuYaw > -1 && m_currImuYaw < 90 && m_ddTrans->yaw < 361 && m_ddTrans->yaw > 270)
	{
		*out_imuYaw = m_currImuYaw + 360;
		*out_ddYaw = m_ddTrans->yaw;
		
		if (currYaw < 90)
			TinyEKF_setX(&m_ekf, 2, currYaw + 360);
	}
	else if (m_ddTrans->yaw > -1 && m_ddTrans->yaw < 90 && m_currImuYaw < 361 && m_currImuYaw > 270)
	{
		*out_ddYaw = m_ddTrans->yaw + 360;
		*out_imuYaw = m_currImuYaw;
		
		if (currYaw < 90)
			TinyEKF_setX(&m_ekf, 2, currYaw + 360);
	}
	else
	{
		*out_imuYaw = m_currImuYaw;
		*out_ddYaw = m_ddTrans->yaw;
		
		if (currYaw > 360)
			TinyEKF_setX(&m_ekf, 2, currYaw - 360);
	}
}

uint8_t UpdateTrackingError()
{
//	float yawDiff = m_currImuYaw - m_ddTrans->yaw;
//	float absYaw = fabsf(yawDiff);
//	if (absYaw > YAW_DIFF_MAX)
//	{
//		DiffDrive_SetAngularPosDegree(m_currImuYaw);
//	}
}

void Disarm()
{	
	// Turn motors off
	MotorController_setMotor(MOTOR_ALL, MIN_THROTTLE, FWD);
	ControlMgr_setState(CONTROL_STATE_IDLE);
}

void RoverControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
{
	 if (data_length == 0)
		return;
	
	uint8_t instruction = att_data[0];
	if (instruction == INSTRUCTION_TRANSLATE)
	{
		m_x = att_data[1];
		m_y = att_data[2];
		
		m_doUpdate = TRUE;
	}
}

void ParseTranslateQuadDrive(uint8_t _x, uint8_t _y)
{
	/*
	A - B
	|   |
	D - C
	*/
	
	direction_t dir = FWD;
	float x = mapf(_x, 0, 255, -1, 1);
	float y = mapf(_y, 0, 255, -1, 1);
	
	// ***********************
	// TODO: Fix bug where using more than 75% of power causes wifi to loose connection. 
	// This seems to be a power regulation issue
/*
	float max = 0.75f;
	if (x < -max)
		x = -max;
	else if (x > max)
		x = max;
	
	if (y < -max)
		y = -max;
	else if (y > max)
		y = max;
*/
	// ***********************
		
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
			MotorController_setMotor(MOTOR_C, d, dir);
			MotorController_setMotor(MOTOR_B, e, FWD);
			MotorController_setMotor(MOTOR_D, e, FWD);
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
			MotorController_setMotor(MOTOR_C, e, BWD);
			MotorController_setMotor(MOTOR_B, d, dir);
			MotorController_setMotor(MOTOR_D, d, dir);
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
			MotorController_setMotor(MOTOR_C, e, FWD);
			MotorController_setMotor(MOTOR_B, d, dir);
			MotorController_setMotor(MOTOR_D, d, dir);
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
			MotorController_setMotor(MOTOR_C, d, dir);
			MotorController_setMotor(MOTOR_B, e, BWD);
			MotorController_setMotor(MOTOR_D, e, BWD);
		}
	}
}

void RunMotorTest()
{
		// ********* Test code for servos *********
	static float speed = 0;	
	static BOOL hasRun = FALSE;
	static int motor = MOTOR_ALL;
	
	if (hasRun)
		return;
	
	// up
	//wait_ms(5);
	speed = 0.5;
	MotorController_setMotor(motor, speed, FWD);	// TEST Servos
	HAL_Delay(3000);
	
	// Back
	MotorController_setMotor(motor, speed, BWD);	// TEST Servos
	HAL_Delay(3000);
	
	// Left
	MotorController_setMotor(MOTOR_A, speed, BWD);
	MotorController_setMotor(MOTOR_B, speed, FWD);
	MotorController_setMotor(MOTOR_C, speed, FWD);
	MotorController_setMotor(MOTOR_D, speed, BWD);
	HAL_Delay(3000);
	
	// Right
	MotorController_setMotor(MOTOR_A, speed, FWD);
	MotorController_setMotor(MOTOR_B, speed, BWD);
	MotorController_setMotor(MOTOR_C, speed, BWD);
	MotorController_setMotor(MOTOR_D, speed, FWD);
	HAL_Delay(3000);
	
	// Rotate Left
	MotorController_setMotor(MOTOR_A, speed, BWD);
	MotorController_setMotor(MOTOR_B, speed, FWD);
	MotorController_setMotor(MOTOR_C, speed, BWD);
	MotorController_setMotor(MOTOR_D, speed, FWD);
	HAL_Delay(3000);
	
	// Rotate Right
	MotorController_setMotor(MOTOR_A, speed, FWD);
	MotorController_setMotor(MOTOR_B, speed, BWD);
	MotorController_setMotor(MOTOR_C, speed, FWD);
	MotorController_setMotor(MOTOR_D, speed, BWD);
	HAL_Delay(3000);
	
	MotorController_setMotor(motor, 0, FWD);	// TEST Servos
	
	hasRun = TRUE;
	
	// ********* End Test code for servos ********* 
}

//void PrintIMU()
//{
//	float yaw =  IMU_get_sf_yaw();
//	float pitch = IMU_get_sf_pitch();
//	float roll = IMU_get_sf_roll();
//	
//	PRINTF("yaw: %.2f, pitch: %.2f, roll: %.2f\r\n", yaw, pitch, roll);			// yaw, pitch, roll
//}