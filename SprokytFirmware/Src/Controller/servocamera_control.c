#include "servocamera_control.h"
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
static void ServoIMUUpdate();
static void Disarm();
static void RunMotorTest();
static void PrintIMU();
static void ParseTranslate(uint8_t _x, uint8_t _y);

/* Private functions ---------------------------------------------------------*/
void ServoCameraControl_init()
{	
	RegisterImuCallback(ServoIMUUpdate);
}

void ServoCameraControl_update()
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
	//RunMotorTest();
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

void ServoIMUUpdate(float data[], int size)
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
	MotorController_setMotor(MOTOR_ALL, MIN_THROTTLE, FWD);
	ControlMgr_setState(CONTROL_STATE_IDLE);
}

void ServoCameraControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
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
	/*
	A - B
	|   |
	D - C
	*/
	
	direction_t dir = FWD;
	float x = mapf(_x, 0, 255, 0, 1);
	float y = mapf(_y, 0, 255, 0, 1);
	
	MotorController_setMotor(SERVO_CHANNEL_1, x, dir);
	//MotorController_setMotor(SERVO_CHANNEL_2, y, dir);
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