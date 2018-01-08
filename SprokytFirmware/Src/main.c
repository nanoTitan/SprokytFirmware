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
#include "sonar.h"
#include "error.h"
#include "debug.h"
#include "cube_hal.h"
#include "Encoder.h"
#include "serial_print.h"
#include "rover_control.h"

/* Private variables ---------------------------------------------------------*/

static void InitUSART();
void blink();

int main()
{	
	if (HAL_Init() != HAL_OK)
		Error_Handler(); 
	
	// Configure the system clock
	SystemClock_Config();
	
	/* Set Systick Interrupt priority highest to ensure no lock by using HAL_Delay with StSpin220 */
	//HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
	/* Configure the SysTick IRQ priority - set the second lowest priority */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0x0E, 0);
	
#ifdef SERIAL_PRINT
	SerialPrint_Init();
#endif // SERIAL_PRINT
	
	PRINTF("\n***************************\n");
	PRINTF("Sprokyt Firmware Version %s\n", FIRMWARE_VERSION_STR);
	PRINTF("Copyright Sprokyt LLC 2017\n");
	PRINTF("All Rights Reserved\n");
	PRINTF("***************************\n\n");
	
	PRINTF("Initialization start...\n\n");
	
	//LEDMgr_Init();
	
	// Initialize Button and LED
	BSP_LED_Init(LED2);
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);	// BUTTON_MODE_GPIO
	
	__CRC_CLK_ENABLE();		// Enable HAL clock for IMU
	
	PRINTF("System Clock set to: %u \n", (unsigned int)SystemCoreClock);
	
	// Motor Controller
	MotorController_init();
	
	// IMU and Sensors
#if defined(IMU_ENABLED)
	IMU_init();
#endif // IMU_ENABLED
	
	// Communication
#if defined(WIFI_ENABLED)
	Wifi::Instance()->Init();				// ESP Wifi
#endif // WIFI_ENABLED
	
#if defined(BLE_ENABLED)
	if (InitBLE() != BLE_STATUS_SUCCESS)
		Error_Handler();
#endif // BLE_ENABLED
	
#if defined(SONAR_ENABLED)
	if (Sonar_init() != SONAR_STATUS_SUCCESS)
		Error_Handler();
#endif // SONAR_ENABLED
	
	// Control Manager
	RoverControl_init();
	ControlMgr_setType(CONTROLLER_ROVER);
	
	
	
	PRINTF("Initialization done. Running program...\n\n");
	
	while (1)
	{	
		// IMU and Sensors
#if defined(IMU_ENABLED)
		IMU_update();
#endif // IMU_ENABLED
		
#if defined(SONAR_ENABLED)
		Sonar_update();
#endif // SONAR_ENABLED
		
		// Communication
#if defined(WIFI_ENABLED)
		//Wifi::Instance()->Update();
#endif // WIFI_ENABLED
		
#if defined(BLE_ENABLED)
		BLE_Update();
#endif // BLE_ENABLED
		
		MotorController_update();
		ControlMgr_update();
		
		//MotorController_UpdateMotorTest();
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