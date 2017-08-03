/*
Ruka Firmware
Copyright Sprokyt LLC 2016
All Rights Reserved
*/

#include "constants.h"
#include "control_manager.h"
#include "BLE.h"
#include "motor_controller.h"
#include "imu.h"
//#include "LED/LEDManager.h"
#include "error.h"
#include "debug.h"
#include "cube_hal.h"

void blink();

int main()
{	
	PRINTF("***************************\r\n");
	PRINTF("Ruka Firmware Version %s\r\n", FIRMWARE_VERSION_STR);
	PRINTF("Copyright Sprokyt LLC 2016\r\n");
	PRINTF("All Rights Reserved\r\n");
	PRINTF("***************************\r\n\r\n");
	
	if (HAL_Init() != HAL_OK)
		Error_Handler(); 
	
	// Configure the system clock
	SystemClock_Config();
	
	//LEDMgr_Init();
	
	// Initialize Button
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);	// BUTTON_MODE_GPIO
	
	__CRC_CLK_ENABLE();		// Enable HAL clock for IMU
	
	blink();
	
	// Motor Controller
	MotorController_init();	
	
	// IMU and Sensors
	IMU_init();
	
	// Control Manager
	ControlMgr_init();
	ControlMgr_setType(CONTROLLER_SERVO_CAMERA);	//  CONTROLLER_ESC_PROGRAMMER CONTROLLER_FLIGHT CONTROLLER_SERVO_CAMERA
	
	// Communication
	//SWPF01SA::Instance()->InitWifi();		// ST Wifi
	//Wifi::Instance()->Init();				// ESP Wifi
	if (InitBLE() != BLE_STATUS_SUCCESS)
		Error_Handler();
	
	PRINTF("Initialization finished. Running program...\r\n");
	
	while (1)
	{		
		// Communication
		//Wifi::Instance()->Update();
		UpdateBLE();
		//SWPF01SA::Instance()->Update();
		
		// IMU and Sensors
		IMU_update();
		
		//MotorController_UpdateMotorTest();
		
		ControlMgr_update();
	}
}

void blink()
{
	__GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_5;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	for (int i = 0; i < 3; ++i)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
}