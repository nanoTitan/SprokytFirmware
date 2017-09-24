#include "steppercamera_control.h"
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

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnected();
static void UpdateDisconnected();
static void StepperIMUUpdate();
static void Disarm();
static void PrintIMU();
static void ParseTranslate(uint8_t _x, uint8_t _y);

/* Private functions ---------------------------------------------------------*/
void StepperCameraControl_init()
{	
	RegisterImuCallback(StepperIMUUpdate);
}

void StepperCameraControl_update()
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

void StepperIMUUpdate(float data[], int size)
{
	assert(size > 2);
	if (size < 3)
		return;
	
	if (!BLE_IsConnected())
		return;
	
	BLE_Imu_Update(data, size);
}

void Disarm()
{	
	// Turn motors off
	MotorController_setMotor(STEPPER_MOTOR_1, 0, FWD);
	ControlMgr_setState(CONTROL_STATE_IDLE);
}

void StepperCameraControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
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
	direction_t dir = FWD;
	float x = mapf(_x, 0, 255, 0, 1);
	float y = mapf(_y, 0, 255, 0, 1);
	
	MotorController_setMotor(STEPPER_MOTOR_1, x, dir);
}

//void PrintIMU()
//{
//	float yaw =  IMU_get_sf_yaw();
//	float pitch = IMU_get_sf_pitch();
//	float roll = IMU_get_sf_roll();
//	
//	PRINTF("yaw: %.2f, pitch: %.2f, roll: %.2f\r\n", yaw, pitch, roll);			// yaw, pitch, roll
//}