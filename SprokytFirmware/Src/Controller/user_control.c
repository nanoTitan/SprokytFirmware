#include "user_control.h"
#include "control_manager.h"
#include "motor_controller.h"
#include "PID.h"
//#include "BLE.h"
#include "Wifi.h"
#include "math_ext.h"
#include "debug.h"
#include <mbed.h>

extern "C" 
{
#include "imu.h"
}


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void UpdateUserControl();
static void UpdateDisconnected();
static void Disarm();
static void RunMotorTest();
static void PrintIMU();

/* Private functions ---------------------------------------------------------*/
void UserControl_init()
{	
}

void UserControl_update()
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
		UpdateUserControl();
		break;
		
	case CONTROL_STATE_DISCONNECTED:
		UpdateDisconnected();
		break;
		
	default:
		break;
	}
}

void UpdateUserControl()
{
	//RunMotorTest();
	PrintIMU();
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

void UserControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
{
	if (data_length == 3)
	{
		uint8_t instruction = att_data[0];
		direction_t dir = FWD;
		if( instruction == INSTRUCTION_FORWARD )
			dir = BWD;
		
		uint8_t motorIndex = att_data[1];
		uint8_t value = att_data[2];
		
		float newValue = mapf(value, 0, 255, 0, 1);
		UserControl_setMotor(motorIndex, newValue, dir);
		
		//PRINTF("%u, %.2f, %d\r\n", motorIndex, newValue, dir);
	}
}

void UserControl_setMotor(uint8_t motorIndex, float value, direction_t dir)
{
	MotorController_setMotor(motorIndex, value, dir);
}

void RunMotorTest()
{
		// ********* Test code for servos *********
	static float speed = 0;	
	static bool up = true;
	static int motor = MOTOR_ALL;
	
	if (up)
	{
		wait_ms(50);
		UserControl_setMotor(motor, speed, FWD);	// TEST Servos
		speed += 0.01;
		
		if (speed >= 1)
		{
			up = false;
		}	
	}
	else
	{
		wait_ms(50);
		UserControl_setMotor(motor, speed, BWD);	// TEST Servos
		speed -= 0.01;
		
		if (speed <= 0 && motor < MOTOR_D)
		{
			UserControl_setMotor(motor, 0, FWD);	// Stop
			motor <<= 1;
			speed = 0;
			up = true;
		}
	}
	// ********* End Test code for servos ********* 
}

void PrintIMU()
{
	float yaw =  IMU_get_sf_yaw();
	float pitch = IMU_get_sf_pitch();
	float roll = IMU_get_sf_roll();
	
	PRINTF("yaw: %.2f, pitch: %.2f, roll: %.2f\r\n", yaw, pitch, roll);			// yaw, pitch, roll
}