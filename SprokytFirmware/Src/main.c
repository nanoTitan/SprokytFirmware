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

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

static void InitUSART();
void blink();

int main()
{	
	PRINTF("***************************\n");
	PRINTF("Ruka Firmware Version %s\n", FIRMWARE_VERSION_STR);
	PRINTF("Copyright Sprokyt LLC 2016\n");
	PRINTF("All Rights Reserved\n");
	PRINTF("***************************\n");
	
	if (HAL_Init() != HAL_OK)
		Error_Handler(); 
	
	// Configure the system clock
	SystemClock_Config();
	
	//InitUSART();
	
	//LEDMgr_Init();
	
	// Initialize Button
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);	// BUTTON_MODE_GPIO
	
	__CRC_CLK_ENABLE();		// Enable HAL clock for IMU
	
	PRINTF("System Clock set to: %u \n", (unsigned int)SystemCoreClock);
	
	//blink();
	
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
	
	// Control Manager
	ControlMgr_init();
	ControlMgr_setType(CONTROLLER_STEPPER_CAMERA);
	
	/* Set Systick Interrupt priority highest to ensure no lock by using HAL_Delay with StSpin220 */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
	PRINTF("Initialization finished. Running program...\n");
	
	//uint8_t buff[4] = {'a', 'b', 'c', 'd'};
	
	while (1)
	{		
		//HAL_UART_Transmit(&huart1, buff, 4, 100);
		
		// IMU and Sensors
#if defined(IMU_ENABLED)
		IMU_update();
#endif // IMU_ENABLED
		
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

void InitUSART()
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
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