#include "rover_control.h"
#include "control_manager.h"
#include "motor_controller.h"
#include "BLE.h"
#include "differential_drive.h"

//#include "Wifi.h"
#include "math_ext.h"
#include "debug.h"
//#include "imu.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static BOOL m_doUpdate = FALSE;
static uint8_t m_x = 0;
static uint8_t m_y = 0;

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnected();
static void UpdateDisconnected();
static void Disarm();
static void RunMotorTest();
static void PrintIMU();
static void ParseTranslateQuadDrive(uint8_t _x, uint8_t _y);

/* Private functions ---------------------------------------------------------*/
void RoverControl_init()
{	
	DiffDrive_Init();
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
		PRINTF("x: %u, y: %u\n", m_x, m_y);
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