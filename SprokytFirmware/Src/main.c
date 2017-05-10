/*
Ruka Firmware
Copyright Sprokyt LLC 2016
All Rights Reserved
*/

#include "constants.h"
#include "hw_init.h"
#include "control_manager.h"
#include "BLE.h"
//#include "motor_controller.h"
//#include "imu.h"
//#include "LED/LEDManager.h"
#include "error.h"
#include "debug.h"
#include "cube_hal.h"


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
	
	Hardware_Init();
	
	//LEDMgr_Init();
	
	// Initialize Button
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
	
	// Motor Controller
	//MotorController_init();	
	
	// Control Manager
	//ControlMgr_init();
	//ControlMgr_setType(CONTROLLER_ROVER);	//  CONTROLLER_ESC_PROGRAMMER CONTROLLER_FLIGHT
	
	// IMU and Sensors
	//IMU_init();
	
	// Communication
	if (InitBLE() != BLE_STATUS_SUCCESS)
		Error_Handler(); 
	
	//SWPF01SA::Instance()->InitWifi();		// ST Wifi
	//Wifi::Instance()->Init();				// ESP Wifi
	
	while (1)
	{
		//HAL_Delay(500);
		
		// Communication
		//Wifi::Instance()->Update();
		UpdateBLE();
		//SWPF01SA::Instance()->Update();
		
		// IMU and Sensors
		//IMU_update();
		
		//ControlMgr_update();		
	}
}
