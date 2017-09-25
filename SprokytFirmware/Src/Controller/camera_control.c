#include "camera_control.h"
#include "control_manager.h"
#include "motor_controller.h"
#include "BLE.h"
#include "imu.h"
#include "Servo.h"
#include "math_ext.h"
#include "debug.h"
#include <math.h>
#include <assert.h>

/* Private variables ---------------------------------------------------------*/
static BOOL m_doUpdate = FALSE;
static uint8_t m_x = 0;
static uint8_t m_y = 0;
static float m_yaw = -1;
static float m_pitch = -1;

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnected();
static void UpdateDisconnected();
static void AngularYawUpdate(float yaw);
static void AngularPitchUpdate(float pitch);
static void Disarm();
static void PrintIMU();
static void ParseTranslate(uint8_t _x, uint8_t _y);

/* Private functions ---------------------------------------------------------*/
void CameraControl_init()
{	
#if defined(IMU_ENABLED)
	IMU_RegisterAngularPosCallback(AngularYawUpdate);
#endif // IMU_ENABLED
	
#if defined(STEPPER_ENABLED)
	Stepper_RegisterAngularPosCallback(AngularYawUpdate);
#endif // STEPPER_ENABLED
}

void CameraControl_update()
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
}

void UpdateConnected()
{
	//PrintIMU();
	
	if (m_doUpdate)
	{
		ParseTranslate(m_x, m_y);		
		m_doUpdate = FALSE;
		//PRINTF("x: %u, y: %u\r\n", m_x, m_y);
	}
}

void UpdateDisconnected()
{
	// TODO: Show flashing LEDs if connection is lost
	
	Disarm();
}

void AngularYawUpdate(float yaw)
{
	if (m_yaw == yaw)
		return;
	
	if (!BLE_IsConnected())
		return;
	
	m_yaw = yaw;
	BLE_AngularPosUpdate(m_yaw, m_pitch);
}

void AngularPitchUpdate(float pitch)
{
	if (m_pitch == pitch)
		return;
	
	if (!BLE_IsConnected())
		return;
	
	m_pitch = pitch;
	BLE_AngularPosUpdate(m_yaw, m_pitch);
}

void Disarm()
{	
	// Turn motors off
#if defined(STEPPER_ENABLED)
	MotorController_setMotor(STEPPER_MOTOR_1, 0, FWD);
#endif // STEPPER_ENABLED
	
#if defined(SERVO_ENABLED)
	MotorController_setMotor(MOTOR_ALL, MIN_THROTTLE, FWD);
#endif // SERVO_ENABLED
	
	ControlMgr_setState(CONTROL_STATE_IDLE);
}

void CameraControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
{
	 if (data_length == 0)
		return;
	
	uint8_t instruction = att_data[0];
	if (instruction == INSTRUCTION_TRANSLATE)
	{
		m_x = att_data[1];
		m_y = att_data[2];
		//PRINTF("x: %u, y: %u\r\n", m_x, m_y);
		m_doUpdate = TRUE;		
	}
}

void ParseTranslate(uint8_t _x, uint8_t _y)
{	
	direction_t dir = _x > 127? FWD : BWD;
	float x = mapf(_x, 0, 255, 0, 1);
	float y = mapf(_y, 0, 255, 0, 1);
	
		// Turn motors off
#if defined(STEPPER_ENABLED)
	MotorController_setMotor(STEPPER_MOTOR_1, x, dir);
#endif // STEPPER_ENABLED
	
#if defined(SERVO_ENABLED)
	MotorController_setMotor(SERVO_CHANNEL_1, x, dir);
#endif // SERVO_ENABLED
	
}

//void PrintIMU()
//{
//	float yaw =  IMU_get_sf_yaw();
//	float pitch = IMU_get_sf_pitch();
//	float roll = IMU_get_sf_roll();
//	
//	PRINTF("yaw: %.2f, pitch: %.2f, roll: %.2f\r\n", yaw, pitch, roll);			// yaw, pitch, roll
//}