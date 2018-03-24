#include "rover_control.h"
#include "control_manager.h"
#include "motor_controller.h"
#include "BLE.h"
#include "differential_drive.h"
#include "TinyEKF.h"
//#include "Wifi.h"
#include "uwb.h"
#include "math_ext.h"
#include "debug.h"
#include "MPU9250.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Definitions ---------------------------------------------------------*/
#define YAW_DIFF_MAX	10

/* Private variables ---------------------------------------------------------*/
static TinyEKF m_ekf;
static Transform_t m_trans;
static Transform_t m_lastTrans;
static Transform_t m_ddTrans;
static float m_currImuYaw = 0;
static uint8_t m_x = 0;
static uint8_t m_y = 0;
static BOOL m_updateInstructions = FALSE;
static bool m_hasSetInitialStates = false;
static bool m_hasImuUpdate = false;
static bool m_hasDiffDriveUpdate = false;
static bool m_hasSetTagInfo = false;

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnected();
static void UpdateDisconnected();
static bool InitSensorFusion();
static void UpdateSensorFusion();
static void UpdateTransform();
static void UpdateOrientationRounding(float* out_imuYaw, float* out_ddYaw);
static bool UpdateTrackingError();
static void Disarm();
static void RunMotorTest();
static void PrintIMU();
static void ParseTranslateQuadDrive(uint8_t _x, uint8_t _y);
static void IMU_Callback(float yaw);
static void DiffDrive_Callback(const Transform_t* transform);

static void PrintTransform( );

/* Private functions ---------------------------------------------------------*/
void RoverControl_init()
{	
	memset(&m_trans, 0, sizeof(Transform_t));
	memset(&m_lastTrans, 0, sizeof(Transform_t));
	
	DiffDrive_Init();
	DiffDrive_RegisterCallback(DiffDrive_Callback);
	
#if defined(IMU_ENABLED)
	MPU9250_RegisterAngularPosCallback(IMU_Callback);
#endif // IMU_ENABLED
	
#if defined(SENSOR_FUSION_ENABLED)
	TinyEKF_init(&m_ekf);
	//TinyEKF_print(&m_ekf);
#endif // SENSOR_FUSION_ENABLED
}

bool InitSensorFusion()
{
#if defined(SENSOR_FUSION_ENABLED)	
	bool imuStable = true;
	bool uwbIsReady = true;
	float x = 0, y = 0, z = 0;
	
#if defined(IMU_ENABLED)
	imuStable = MPU9250_get_sensorFusionStable();
	if (!imuStable ||  !m_hasImuUpdate)
		return false;
#endif //IMU_ENABLED
	
#if defined(UWB_ENABLED)	
	
	UWB_GetPosition(&x, &y, &z);
	uwbIsReady = UWB_IsReady();
	if (!uwbIsReady)
		return false;
#endif //UWB_ENABLED
	
	// Don't update sensor fusion if we're not stable or states haven't been set
	if (!m_hasDiffDriveUpdate)	
		return false;
	
	// Update the differential drive angular position with the IMU on start
	DiffDrive_SetAngularPosDegree(m_currImuYaw);
	DiffDrive_SetPos(x, z);
	m_ddTrans.x = x;
	m_ddTrans.y = y;
	m_ddTrans.z = z;
	m_ddTrans.yaw = m_currImuYaw;
//	m_ddTrans.pitch = m_currImuPitch;
//	m_ddTrans.roll = m_currImuRoll;
	
	TinyEKF_setX(&m_ekf, 0, x);						// Initial state x
	TinyEKF_setX(&m_ekf, 1, z);						// Initial state y
	TinyEKF_setX(&m_ekf, 2, m_currImuYaw);		// Initial state yaw	
#endif // SENSOR_FUSION_ENABLED
	
	return true;
}

static bool SetTagInfo()
{
#if !defined(BLE_ENABLED)
	return true;
#endif // BLE_ENABLED
	
#if defined(UWB_ENABLED)
	float tagInfo[15];
	uint8_t size = 0;
	
	if (!BLE_IsConnected())
		return false;
	
	if (!UWB_GetModuleData(tagInfo, 15*sizeof(float), &size))
		return false;
	
	tBleStatus result = BLE_SetTagInfo(tagInfo, size);
	if (result != BLE_STATUS_SUCCESS)
	{
		return false;
	}
#endif // UWB_ENABLED
	
	return true;
}

void RoverControl_update()
{	
	CONTROL_STATE state = ControlMgr_getState();
	
// Debug flight control without begin connected
#if defined(DEBUG_CONTROLLER_NO_CONNECT)
	state = CONTROL_STATE_CONNECTED;
#endif 
	
	switch (state)
	{
	case CONTROL_STATE_IDLE:
	case CONTROL_STATE_CONNECTED:
		UpdateConnected();
		break;
		
	case CONTROL_STATE_DISCONNECTED:
		UpdateDisconnected();
		break;
		
	default:
		break;
	}
}

void UpdateConnected()
{	
	if (m_updateInstructions)
	{
		DiffDrive_ParseTranslate(m_x, m_y);
		m_updateInstructions = FALSE;
		//PRINTF("x: %u, y: %u\r\n", m_x, m_y);
	}
	
	DiffDrive_Update();
	
	if (!m_hasSetTagInfo)
	{
		m_hasSetTagInfo = SetTagInfo();
	}
	
	if (!m_hasSetInitialStates)
	{
		m_hasSetInitialStates = InitSensorFusion();
		return;
	}
	
#if defined(SENSOR_FUSION_ENABLED)
	UpdateSensorFusion();
#else
	UpdateTransform();
#endif // SENSOR_FUSION_ENABLED
	
	// Send Transform to BLE
	static int i = 0;
	++i;
	if (i > 10 && 
		(m_trans.x != m_lastTrans.x ||
		m_trans.y != m_lastTrans.y ||
		m_trans.z != m_lastTrans.z ||
		m_trans.yaw != m_lastTrans.yaw))
	{
#if defined(BLE_ENABLED)
		BLE_PositionUpdate(&m_trans);
#endif // BLE_ENABLED
		
		memcpy(&m_lastTrans, &m_trans, sizeof(Transform_t));
		i = 0;
	}
}

void UpdateDisconnected()
{
	// TODO: Show flashing LEDs if connection is lost
	Disarm();
	m_hasSetTagInfo = false;
	ControlMgr_setState(CONTROL_STATE_IDLE);
}

void UpdateSensorFusion()
{
	float uwb_x, uwb_y, uwb_z;
	float imuYaw, ddYaw;
		
	static uint32_t lastTime = 0;
	uint32_t currTime = HAL_GetTick();
	uint32_t deltaTime = currTime - lastTime;
	if (deltaTime < 100)
	{
		return;
	}
	
	// Only update SensorFusion when new states ready
	// TODO: Is this right? We may need to update more frequently
	if (!m_hasDiffDriveUpdate)
		return;
	
#if defined(IMU_ENABLED)
	if (!m_hasImuUpdate)
		return;
	
	m_hasImuUpdate = false;
#endif	// IMU_ENABLED
	
	m_hasDiffDriveUpdate = false;
	
#if defined(UWB_ENABLED)
	UWB_GetPosition(&uwb_x, &uwb_y, &uwb_z);
#else
	uwb_x = m_ddTrans.x;
	uwb_y = 0;
	uwb_z = m_ddTrans.z;
#endif
	
#if defined(IMU_ENABLED)
	UpdateOrientationRounding(&imuYaw, &ddYaw);
#else
	imuYaw = ddYaw = m_ddTrans.yaw;
#endif // IMU_ENABLED
	
	// EKF Step
	double z[6] = { uwb_x, m_ddTrans.x, uwb_z, m_ddTrans.z, imuYaw, ddYaw };
	TinyEKF_step(&m_ekf, z);
	
	// Switch, negate, and scale x,z coordinates so they show up correctly in app coordinates	
#if defined(UWB_ENABLED)
	m_trans.x = TinyEKF_getX(&m_ekf, 0);
	m_trans.y = uwb_y;
	m_trans.z = TinyEKF_getX(&m_ekf, 1);
#else
	m_trans.x = m_ddTrans.x;
	m_trans.y = uwb_y;
	m_trans.z = m_ddTrans.z;
#endif	// UWB_ENABLED
	
#if defined(IMU_ENABLED)
	//m_trans.yaw = TinyEKF_getX(&m_ekf, 2);
	m_trans.yaw = imuYaw * 0.1f + ddYaw * 0.9f;
#else
	m_trans.yaw = m_ddTrans.yaw;
#endif	// IMU_ENABLED
	
#if defined(BLE_ENABLED) && defined(SENSOR_FUSION_DEBUG_ENABLED)
	static uint32_t lastDebugTime = 0;
	
	// EKF Debug twice a second
	if (currTime - lastDebugTime >= 500)
	{
		float ekfDebug[7] = { currTime, uwb_x, m_ddTrans.x, uwb_z, m_ddTrans.z, imuYaw, ddYaw };
		BLE_LogEkfDebug(ekfDebug, 28);		
		lastDebugTime = currTime;
	}
#endif // BLE_ENABLED && SENSOR_FUSION_DEBUG_ENABLED
	
	PrintTransform();
	
	lastTime = currTime;
}

void UpdateTransform()
{	
	m_trans.x = m_ddTrans.x;
	m_trans.y = 0;
	m_trans.z = m_ddTrans.z;
	m_trans.yaw = m_ddTrans.yaw;
	
	PrintTransform();
}

void UpdateOrientationRounding(float* out_imuYaw, float* out_ddYaw)
{
	float currYaw = TinyEKF_getX(&m_ekf, 2);
	
	if (m_currImuYaw > -1 && m_currImuYaw < 90 && m_ddTrans.yaw < 361 && m_ddTrans.yaw > 270)
	{
		*out_imuYaw = m_currImuYaw + 360;
		*out_ddYaw = m_ddTrans.yaw;
		
		if (currYaw < 90)
			TinyEKF_setX(&m_ekf, 2, currYaw + 360);
	}
	else if (m_ddTrans.yaw > -1 && m_ddTrans.yaw < 90 && m_currImuYaw < 361 && m_currImuYaw > 270)
	{
		*out_ddYaw = m_ddTrans.yaw + 360;
		*out_imuYaw = m_currImuYaw;
		
		if (currYaw < 90)
			TinyEKF_setX(&m_ekf, 2, currYaw + 360);
	}
	else
	{
		*out_imuYaw = m_currImuYaw;
		*out_ddYaw = m_ddTrans.yaw;
		
		if (currYaw > 360)
			TinyEKF_setX(&m_ekf, 2, currYaw - 360);
	}
}

bool UpdateTrackingError()
{
#if defined(IMU_ENABLED)
	float yawDiff = m_currImuYaw - m_ddTrans.yaw;
	float absYaw = fabsf(yawDiff);
	if (absYaw > YAW_DIFF_MAX)
	{
		DiffDrive_SetAngularPosDegree(m_currImuYaw);
	}
	return true;
#endif // IMU_ENABLED
	
	return false;
}

void IMU_Callback(float yaw)
{
	if (m_trans.yaw == yaw)
		return;
	
	m_currImuYaw = yaw;
	m_hasImuUpdate = true;
}

void DiffDrive_Callback(const Transform_t* transform)
{	
	memcpy((void*)&m_ddTrans, transform, sizeof(Transform_t));
	m_hasDiffDriveUpdate = true;
}

void Disarm()
{	
	// Turn motors off
	MotorController_setMotor(MOTOR_ALL, 0, FWD);
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
		
		m_updateInstructions = TRUE;
	}
}

void PrintTransform()
{
#if defined(PRINT_ROVER_CONTROL)
	static uint32_t lastPrintTime = 0;
	uint32_t currTime = HAL_GetTick();
	if (currTime - lastPrintTime >= 500)
	{
		PRINTF("x: %.1f y: %.1f z: %.1f yaw: %.1f\n", m_trans.x, m_trans.y, m_trans.z, m_trans.yaw);
		lastPrintTime = currTime;
	}
#endif // PRINT_ROVER_CONTROL
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