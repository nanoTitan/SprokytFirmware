#include "StSpin220Stepper.h"
#include "motorcontrol.h"
#include "stspin220.h"
#include "stm32f4xx_nucleo_ihm06a1.h"
#include "stm32f4xx_hal_conf.h"
#include "constants.h"
#include "error.h"
#include "debug.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STEP_ANGLE            (18) // decidegree
#define FULL_STEPS_PER_TURN   (3600/(STEP_ANGLE))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t gLastError;
static int32_t m_pos;
static uint16_t m_Speed;
static uint16_t m_Dec;
static motorStepMode_t m_StepMode;
static float m_newSpeed;
static direction_t m_newDir = NONE;
static uint32_t m_FreqPwm;
static uint32_t m_lastPosSendTime;
static uint8_t m_Torque;
static uint8_t m_updateSpeed = 0;
static uint8_t m_updateDir = 0;
static uint32_t m_numStepsPerTurn = 0;
static float m_oneOverNumStepsPerTurn = 0;
static AngularPositionCallback m_angularFunc = NULL;

/* Private function prototypes -----------------------------------------------*/
static void UpdateAngularPosition();
static void UpdateSpeed();
static void UpdateDirection();
static void MyFlagInterruptHandler(void);
void ButtonHandler(void);

/* Initialization parameters. */
Stspin220_Init_t initDeviceParameters =
{
	1200,						//Acceleration rate in pulse/s2 (must be greater than 0)
	1200,						//Deceleration rate in pulse/s2 (must be greater than 0)
	STEPPER_MAX_CAMERA_SPEED,   //Running speed in pulse/s (8 pulse/s < Maximum speed <= 10 000 pulse/s )
	STEPPER_MIN_CAMERA_SPEED,   //Minimum speed in pulse/s (8 pulse/s <= Minimum speed < 10 000 pulse/s)
	20,							//Acceleration current torque in % (from 0 to 100)
	15,							//Deceleration current torque in % (from 0 to 100)
	10,							//Running current torque in % (from 0 to 100)
	25,							//Holding current torque in % (from 0 to 100)
	TRUE,						//Torque boost speed enable
	200,						//Torque boost speed threshold in fullstep/s
	STEP_MODE_1_32,				//Step mode via enum motorStepMode_t  
	HOLD_MODE,					//Automatic HIZ STOP
	100000						//REF frequency (Hz)
};

void Stepper_Init()
{
	/* Set the STSPIN220 library to use 1 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, 1);
	
	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the STSPIN220 parameters are set with the predefined values from file    */
	/* stspin220_target_config.h, otherwise the parameters are set using the    */
	/* initDeviceParameters structure values.                                   */
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, &initDeviceParameters);
	//BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, NULL);
	
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
	BSP_MotorControl_AttachErrorHandler(CError_Handler_1);
	
	m_FreqPwm = BSP_MotorControl_GetBridgeInputPwmFreq(0);	// Get the PWM frequency used for the VREFA and VREFB voltage generation
	BSP_MotorControl_SetBridgeInputPwmFreq(0, m_FreqPwm >> 1);	// Set the PWM frequency used for the VREFA and VREFB voltage generation
	
	m_StepMode = BSP_MotorControl_GetStepMode(0);	// Get the predefined step mode
	m_numStepsPerTurn = FULL_STEPS_PER_TURN * (1 << BSP_MotorControl_GetStepMode(0));
	m_oneOverNumStepsPerTurn = 1.0f / (float)m_numStepsPerTurn;
	
	BSP_MotorControl_SetStopMode(0, HOLD_MODE);   	
}

void Stepper_RegisterAngularPosCallback(AngularPositionCallback callback)
{
	if (callback != NULL)
	{
		m_angularFunc = callback;
	}
}

void Stepper_Update()
{
//	static float nspeed = 1.0f;
//	static uint32_t count = 0;
	static uint8_t toggle = 0;
//	++count;
//	if (count == 100000)
//	{
//		if (toggle % 4 == 3)
//		{
//			m_newDir = BWD;
//			nspeed = 1;
//		}
//		else
//		{
//			m_newDir = FWD;
//			nspeed -= 0.2f;
//		}
//		
//		m_updateSpeed = 1;
//		m_updateDir = 1;
//		count = 0;
//		m_newSpeed = nspeed;
//		++toggle;
//	}
	
	UpdateSpeed();
	UpdateDirection();
	UpdateAngularPosition();
}

void UpdateAngularPosition()
{
	if (m_angularFunc)
	{
		uint32_t tickstart = HAL_GetTick();
		if (tickstart - m_lastPosSendTime > 100)
		{
			int32_t pos = BSP_MotorControl_GetPosition(STEPPER_MOTOR_1);
			if (pos > m_numStepsPerTurn-1 || pos < 0)
			{
				pos = pos % m_numStepsPerTurn;
			}
			
			if (pos < 0)
			{
				pos += m_numStepsPerTurn;
			}
			
			float angle = pos * m_oneOverNumStepsPerTurn * 360.0f;
			m_angularFunc(angle);
			m_lastPosSendTime = tickstart;
		}
	}
}

void UpdateSpeed()
{	
	if (m_updateSpeed == 0)
		return;
	
	if (m_newSpeed == 0)
	{
		BSP_MotorControl_HardStop(STEPPER_MOTOR_1);
	}
	else
	{
		uint16_t newMax = (STEPPER_MAX_CAMERA_SPEED - STEPPER_MIN_CAMERA_SPEED) * m_newSpeed + STEPPER_MIN_CAMERA_SPEED;
		if (newMax < STEPPER_MIN_CAMERA_SPEED)
			newMax = STEPPER_MIN_CAMERA_SPEED;
		
		BSP_MotorControl_SetMaxSpeed(STEPPER_MOTOR_1, newMax);
	}
	
	m_updateSpeed = 0;
}

void UpdateDirection()
{	
	if (m_updateDir == 0)
		return;
	
	motorDir_t dir = m_newDir == FWD ? FORWARD : BACKWARD;
	motorState_t state = BSP_MotorControl_GetDeviceState(STEPPER_MOTOR_1);
	if (dir != BSP_MotorControl_GetDirection(STEPPER_MOTOR_1) || state == INACTIVE || state == STANDBY || state == STANDBYTOINACTIVE)
	{
		BSP_MotorControl_HardStop(STEPPER_MOTOR_1);
		BSP_MotorControl_WaitWhileActive(0);
		BSP_MotorControl_Run(STEPPER_MOTOR_1, dir);
	}
	
	m_updateDir = 0;
}

void Stepper_SetSpeedAndDirection(float speed, direction_t direction)
{
	Stepper_SetSpeed(speed);
	Stepper_SetDirection(direction);
}

void Stepper_SetSpeed(float speed)
{
	m_newSpeed = speed;
	if (m_newSpeed < 0)
		m_newSpeed = 0;
	else if (m_newSpeed > 1)
		m_newSpeed = 1;
	
	m_updateSpeed = 1;
}

void Stepper_SetDirection(direction_t direction)
{
	m_newDir = direction;
	m_updateDir = 1;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  //When EN pin is forced low by a failure, configure the GPIO as an ouput low
	BSP_MotorControl_CmdDisable(0);
}

/**
  * @brief  This function is executed in case of button press and it changes the
  * current direction of the motor
  * @param  None
  * @retval None
  */
void ButtonHandler(void)
{
	if (BSP_MotorControl_GetDirection(0) != BACKWARD)
	{
		BSP_MotorControl_SetDirection(0, BACKWARD);
	}
	else
	{
		BSP_MotorControl_SetDirection(0, FORWARD);
	}
	/* Let 200 ms before clearing the IT for key debouncing */
	HAL_Delay(200);
	__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
	HAL_NVIC_ClearPendingIRQ(KEY_BUTTON_EXTI_IRQn);
}

void Stepper_MotorTest()
{
	//BSP_MotorControl_SetMinSpeed(0, 20);
	//BSP_MotorControl_SetAcceleration(0, 1200);
	//BSP_MotorControl_SetDeceleration(0, 1200);
	
	/* Keep power bridges on when motor is stopped */
	BSP_MotorControl_SetStopMode(0, HOLD_MODE);   
	
	/* Request to run BACKWARD */
	BSP_MotorControl_SetMaxSpeed(0, 1200);
	BSP_MotorControl_Run(0, BACKWARD);       
	HAL_Delay(5000);
	
	BSP_MotorControl_SetMaxSpeed(0, 2400);
	HAL_Delay(5000);

	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);
	if (m_Speed != 2400)
	{
		CError_Handler_1(1);
	}
	
	BSP_MotorControl_SetMaxSpeed(0, 4800);
	HAL_Delay(5000);

	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);
	if (m_Speed != 4800)
	{
		CError_Handler_1(1);
	}

	//----- Decrease the speed while running

	BSP_MotorControl_SetMaxSpeed(0, 20);	
	BSP_MotorControl_Run(0, FORWARD);  
	HAL_Delay(5000);
	
	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);
	if (m_Speed != 20)
	{
		CError_Handler_1(1);
	}

	BSP_MotorControl_SoftStop(0);
	BSP_MotorControl_WaitWhileActive(0);
}

void Stepper_MotorTest2()
{
	/* Move 2*FULL_STEPS_PER_TURN<<m_StepMode microsteps in the FORWARD direction */
	BSP_MotorControl_Move(0, FORWARD, 2*FULL_STEPS_PER_TURN << m_StepMode);

	  /* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 1 second */
	HAL_Delay(1000);
  
	//----- Disable the power bridges
  
	  /* Disable the power bridges */
	BSP_MotorControl_CmdDisable(0);
  
	/* Wait for 1 second */
	HAL_Delay(1000);  

	//----- Stop with power bridges disabled

	  /* Turn off power bridges when motor is stopped */
	BSP_MotorControl_SetStopMode(0, HIZ_MODE);
  
	//----- Move device 2 turns in the BACKWARD direction using predefined microsteps

	  /* Move 2*FULL_STEPS_PER_TURN<<m_StepMode microsteps in the BACKWARD direction */
	BSP_MotorControl_Move(0, BACKWARD, 2*FULL_STEPS_PER_TURN << m_StepMode);

	  /* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);

	  /* Set the current position to be the Home position */
	BSP_MotorControl_SetHome(0);
  
	/* Wait for 1 second */
	HAL_Delay(1000);

	//----- Stop with power bridges enabled

	  /* Keep power bridges on when motor is stopped */
	BSP_MotorControl_SetStopMode(0, HOLD_MODE);   
  
	//----- Go to position -6400

	  /* Request to go to position -6400 */
	BSP_MotorControl_GoTo(0, -6400);  
  
	/* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position */
	m_pos = BSP_MotorControl_GetPosition(0);

	if (m_pos != -6400) 
	{
		CError_Handler_1(STSPIN220_ERROR_POSITION);
	}
  
	/* Set the current position to be the Mark position */
	BSP_MotorControl_SetMark(0);

	  /* Wait for 1 second */
	HAL_Delay(1000);
  
	//----- Go Home

	  /* Request to go to Home */
	BSP_MotorControl_GoHome(0);  
	BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position */
	m_pos = BSP_MotorControl_GetPosition(0);
  
	/* Wait for 1 second */
	HAL_Delay(1000);

	//----- Go to position 6400

	  /* Request to go to position 6400 */
	BSP_MotorControl_CmdGoToDir(0, FORWARD, 6400);
  
	/* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position */
	m_pos = BSP_MotorControl_GetPosition(0);

	  /* Wait for 1 second */
	HAL_Delay(1000);
  
	//----- Go Mark which was set previously after go to -6400

	  /* Request to go to Mark position */
	BSP_MotorControl_GoMark(0);  
  
	/* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position */
	m_pos = BSP_MotorControl_GetPosition(0);

	  /* Wait for 1 second */
	HAL_Delay(1000);

	//----- Run the motor BACKWARD

	  /* Request to run BACKWARD */
	BSP_MotorControl_Run(0, BACKWARD);       
	HAL_Delay(5000);

	   /* Get current speed */
	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Increase the speed while running

	  /* Increase speed to 2400 microstep/s */
	BSP_MotorControl_SetMaxSpeed(0, 2400);
	HAL_Delay(5000);

	   /* Get current speed */
	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Decrease the speed while running

	  /* Decrease speed to 1200 microstep/s */
	BSP_MotorControl_SetMaxSpeed(0, 1200);
	HAL_Delay(5000);

	  /* Get current speed */
	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Increase acceleration while running

	  /* Increase acceleration to 2000 microstep/s^2 */
	BSP_MotorControl_SetAcceleration(0, 2000);
	HAL_Delay(5000);

	  /* Increase speed to 2400 microstep/s */
	BSP_MotorControl_SetMaxSpeed(0, 2400);
	HAL_Delay(5000);

	  /* Get current speed */
	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);

	if (m_Speed != 2400)
	{
		CError_Handler_1(STSPIN220_ERROR_SPEED);
	}
	//----- Get current deceleration

	m_Dec = BSP_MotorControl_GetDeceleration(0);
  
	//----- Increase deceleration while running

	  /* Increase deceleration to 2000 microstep/s^2 */
	BSP_MotorControl_SetDeceleration(0, 2000);
	HAL_Delay(5000);

	  /* Decrease speed to 1200 microstep/s */
	BSP_MotorControl_SetMaxSpeed(0, 1200);
	HAL_Delay(5000);

	  /* Get current speed */
	m_Speed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Retore previous deceleration
  
	BSP_MotorControl_SetDeceleration(0, m_Dec);

	//----- Soft stopped required while running

	  /* Request soft stop */
	BSP_MotorControl_SoftStop(0);

	  /* Wait for the motor ends moving */  
	BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 2 seconds */
	HAL_Delay(2000);
  
	//----- Run stopped by hardstop

	  /* Request to run in FORWARD direction */
	BSP_MotorControl_Run(0, FORWARD);       
	HAL_Delay(5000);
  
	/* Request to immediatly stop */
	BSP_MotorControl_HardStop(0);
	BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 2 seconds */
	HAL_Delay(2000);

	//----- GOTO stopped by softstop

	 /* Request to go to position 20000  */
	BSP_MotorControl_GoTo(0, 20000);  
	HAL_Delay(5000);

	  /* Request to perform a soft stop */
	BSP_MotorControl_SoftStop(0);
	BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 2 seconds */
	HAL_Delay(2000);  

	//----- Change current torque while running (RUN_TORQUE value is kept unchanged)
  
	  /* Get the running torque */
	m_Torque = BSP_MotorControl_GetTorque(0, RUN_TORQUE);
  
	/* Request to run in FORWARD direction */
	BSP_MotorControl_Run(0, FORWARD);
  
	/* When the motor is running steady for 100ms, decrease the current torque */
	while (BSP_MotorControl_GetDeviceState(0) != STEADY)
		;
	HAL_Delay(100);
	BSP_MotorControl_SetTorque(0, CURRENT_TORQUE, m_Torque >> 1);

	  /* Wait for 1 second */
	HAL_Delay(1000);
  
	/* Request soft stop */
	BSP_MotorControl_SoftStop(0);

	  /* Wait for the motor ends moving */  
	BSP_MotorControl_WaitWhileActive(0);
  
	/* Get the running torque and check it has not change */
	if (BSP_MotorControl_GetTorque(0, RUN_TORQUE) != m_Torque) 
	{
		CError_Handler_1(STSPIN220_ERROR_POSITION);
	}
  
	/* Wait for 1 second */
	HAL_Delay(1000); 
  
	//----- Change the holding torque (HOLD TORQUE value is changed)
  
	BSP_MotorControl_SetTorque(0, HOLD_TORQUE, 5);
  
	/* Wait for 1 second */
	HAL_Delay(1000); 

	//----- Change step mode to full step mode

	  /* Select full step mode */
	BSP_MotorControl_SelectStepMode(0, STEP_MODE_FULL);

	  /* Set speed, acceleration and deceleration to scale with full step mode */
	if (BSP_MotorControl_SetMinSpeed(0, BSP_MotorControl_GetMinSpeed(0) >> m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_MIN_SPEED);
	}
	if (BSP_MotorControl_SetMaxSpeed(0, BSP_MotorControl_GetMaxSpeed(0) >> m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_MAX_SPEED);
	}
	if (BSP_MotorControl_SetAcceleration(0, BSP_MotorControl_GetAcceleration(0) >> m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_ACCELERATION);
	}
	if (BSP_MotorControl_SetDeceleration(0, BSP_MotorControl_GetDeceleration(0) >> m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_DECELERATION);
	}
  
	/* Request to go position 200 (full steps) */
	BSP_MotorControl_GoTo(0, 200);

	  /* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position */
	m_pos =  BSP_MotorControl_GetPosition(0);

	if (m_pos != 200) 
	{
		CError_Handler_1(STSPIN220_ERROR_POSITION);
	}
  
	/* Wait for 2 seconds */
	HAL_Delay(2000);

	//----- Restore step mode to predefined step mode
	  /* Select predefined step mode */
	BSP_MotorControl_SelectStepMode(0, m_StepMode);

	  /* Set speed, acceleration and deceleration to scale with microstep mode */
	if (BSP_MotorControl_SetMaxSpeed(0, BSP_MotorControl_GetMaxSpeed(0) << m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_MAX_SPEED);
	}
	if (BSP_MotorControl_SetMinSpeed(0, BSP_MotorControl_GetMinSpeed(0) << m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_MIN_SPEED);
	}
	if (BSP_MotorControl_SetAcceleration(0, BSP_MotorControl_GetAcceleration(0) << m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_ACCELERATION);
	}
	if (BSP_MotorControl_SetDeceleration(0, BSP_MotorControl_GetDeceleration(0) << m_StepMode) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_DECELERATION);
	}
  
	/* Request to go to position 200*2^m_StepMode */
	m_pos = 200 << m_StepMode;
	BSP_MotorControl_GoTo(0, m_pos);

	  /* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);

	  /* Check current position */
	if (BSP_MotorControl_GetPosition(0) != m_pos) 
	{
		CError_Handler_1(STSPIN220_ERROR_POSITION);
	}
	else
	{
	  /* Set the current position to be the Home position */
		BSP_MotorControl_SetHome(0);
	}
  
	/* Wait for 2 seconds */
	HAL_Delay(2000);

	//----- Automatic Full-step switch demonstration
	  /* Increase max speed beyond torque boost speed threshold */
	m_Speed = (STSPIN220_CONF_PARAM_TORQUE_BOOST_TH << (m_StepMode + 1));
	if (BSP_MotorControl_SetMaxSpeed(0, m_Speed) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_MAX_SPEED);
	}
	/* increase acceleration */
	if (BSP_MotorControl_SetAcceleration(0, BSP_MotorControl_GetAcceleration(0) << 3) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_ACCELERATION);
	}
	/* increase deceleration */
	if (BSP_MotorControl_SetDeceleration(0, BSP_MotorControl_GetDeceleration(0) << 3) == FALSE)
	{
		CError_Handler_1(STSPIN220_ERROR_SET_DECELERATION);
	}
	/* Request to run */
	BSP_MotorControl_Run(0, BACKWARD);
	HAL_Delay(5000);
  
	/* Request soft stop */
	BSP_MotorControl_SoftStop(0);
	HAL_Delay(5000);

	//----- Slowest move

	  /* Set the current position to be the Home position */
	BSP_MotorControl_SetHome(0);
  
	/* Set speed and acceleration at lowest values */
	BSP_MotorControl_SetMinSpeed(0, 8);
	BSP_MotorControl_SetMaxSpeed(0, 8);
	BSP_MotorControl_SetAcceleration(0, 8);
	BSP_MotorControl_SetDeceleration(0, 8);

	  /* Move forward device number of microsteps in a full step + 1 microstep */
	m_pos = (1 << m_StepMode) + 1;
	BSP_MotorControl_Move(0, FORWARD, m_pos);
  
	/* Wait for the motor ends moving */
	BSP_MotorControl_WaitWhileActive(0);
  
	/* Check current position */
	if (BSP_MotorControl_GetPosition(0) != m_pos) 
	{
		CError_Handler_1(STSPIN220_ERROR_POSITION);
	}
	else
	{
	  /* Set the current position to be the Home position */
		BSP_MotorControl_SetHome(0);
	}
  
	/* Wait for 40 milliseconds */
	HAL_Delay(40);
 
   //----- Stop with Standby mode
  
     /* Enter standby mode when motor is stopped */
	BSP_MotorControl_SetStopMode(0, STANDBY_MODE);

	  /* Move forward 1 microstep */
	BSP_MotorControl_Move(0, FORWARD, 1);
	BSP_MotorControl_WaitWhileActive(0);
	BSP_MotorControl_SetHome(0);

	  /* Wait for 40 milliseconds */
	HAL_Delay(40);
  
	/* Move forward 1 microstep */
	BSP_MotorControl_Move(0, FORWARD, 1);
	BSP_MotorControl_WaitWhileActive(0);
	BSP_MotorControl_SetHome(0);
  
	/* Wait for 40 milliseconds */
	HAL_Delay(40);
  
	/* Move forward 1 microstep */
	BSP_MotorControl_Move(0, FORWARD, 1);
	BSP_MotorControl_WaitWhileActive(0);
	BSP_MotorControl_SetHome(0);

	  /* Wait for 40 milliseconds */
	HAL_Delay(40);
  
	/* Move forward 1 microstep */
	BSP_MotorControl_Move(0, FORWARD, 1);
	BSP_MotorControl_WaitWhileActive(0);
	BSP_MotorControl_SetHome(0);  

	//----- Stop with power bridges enabled

	  /* Keep power bridges on when motor is stopped */
	BSP_MotorControl_SetStopMode(0, HOLD_MODE);
  
	//----- Set speed, acceleration and deceleration from stspin220_target_config.h 
  
	  /* Set speed and acceleration from stspin220_target_config.h */
	BSP_MotorControl_SetMaxSpeed(0, STSPIN220_CONF_PARAM_RUNNING_SPEED);
	BSP_MotorControl_SetMinSpeed(0, STSPIN220_CONF_PARAM_MIN_SPEED);
	BSP_MotorControl_SetAcceleration(0, STSPIN220_CONF_PARAM_ACC);
	BSP_MotorControl_SetDeceleration(0, STSPIN220_CONF_PARAM_DEC);
 
    /* Wait for 2 seconds */
	HAL_Delay(2000);
}