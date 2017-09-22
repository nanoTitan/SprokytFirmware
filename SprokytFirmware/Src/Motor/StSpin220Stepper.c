#include "StSpin220Stepper.h"
#include "motorcontrol.h"
#include "stspin220.h"
#include "stm32f4xx_nucleo_ihm06a1.h"
#include "stm32f4xx_hal_conf.h"
#include "constants.h"
#include "error.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STEP_ANGLE            (18) // decidegree
#define FULL_STEPS_PER_TURN   (3600/(STEP_ANGLE))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t gLastError;
static int32_t pos;
static uint16_t mySpeed;
static uint16_t myDec;
static motorStepMode_t myStepMode;
static uint32_t myFreqPwm;
static uint8_t myTorque;

/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);
void ButtonHandler(void);

/* Initialization parameters. */
Stspin220_Init_t initDeviceParameters =
{
	480,             //Acceleration rate in pulse/s2 (must be greater than 0)
	480,             //Deceleration rate in pulse/s2 (must be greater than 0)
	1600,            //Running speed in pulse/s (8 pulse/s < Maximum speed <= 10 000 pulse/s )
	400,             //Minimum speed in pulse/s (8 pulse/s <= Minimum speed < 10 000 pulse/s)
	20,              //Acceleration current torque in % (from 0 to 100)
	15,              //Deceleration current torque in % (from 0 to 100)
	10,              //Running current torque in % (from 0 to 100)
	25,              //Holding current torque in % (from 0 to 100)
	TRUE,            //Torque boost speed enable
	200,             //Torque boost speed threshold in fullstep/s
	STEP_MODE_1_32,  //Step mode via enum motorStepMode_t  
	HOLD_MODE,       //Automatic HIZ STOP
	100000           //REF frequency (Hz)
};

void Stepper_Init()
{
	/* Set the STSPIN220 library to use 1 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, 1);
	
	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the STSPIN220 parameters are set with the predefined values from file    */
	/* stspin220_target_config.h, otherwise the parameters are set using the    */
	/* initDeviceParameters structure values.                                   */
	//BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, &initDeviceParameters);
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN220, NULL);
	
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
	BSP_MotorControl_AttachErrorHandler(CError_Handler_1);
	
	myFreqPwm = BSP_MotorControl_GetBridgeInputPwmFreq(0);	// Get the PWM frequency used for the VREFA and VREFB voltage generation
	BSP_MotorControl_SetBridgeInputPwmFreq(0, myFreqPwm >> 1);	// Set the PWM frequency used for the VREFA and VREFB voltage generation
	
	myStepMode = BSP_MotorControl_GetStepMode(0);	// Get the predefined step mode
}
