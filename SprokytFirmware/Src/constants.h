
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <stdint.h>

#define FIRMWARE_VERSION_0_2_2
//#define FIRMWARE_VERSION_0_2_3

#define FIRMWARE_VERSION_STR "0.2.2"

/*********************************************************/
// Component enable macros
/*********************************************************/
#define SERIAL_PRINT
#define BLE_ENABLED
#define UWB_ENABLED
//#define WIFI_ENABLED
//#define IMU_ENABLED
//#define STEPPER_ENABLED
//#define SERVO_ENABLED
//#define SONAR_ENABLED
#define ENCODER_ENABLED
#define SENSOR_FUSION_ENABLED

//#define MOTOR_STEPPER	
//#define MOTOR_SERVO
#define MOTOR_TOSHIBA
//#define MOTOR_STSPIN

#define MOTOR_1_ENABLED
//#define MOTOR_2_ENABLED


/*********************************************************/
// Timings
/*********************************************************/
#define UWB_UPDATE_TIME			300		// in milliseconds

// Definitions for TOSHIBA motor drivers
#define MD1_TIM					TIM1
#define MD2_TIM					TIM2

#define MD1_PWMA_Pin			GPIO_PIN_9
#define MD1_PWMA_GPIO_Port		GPIOA
#define MD1_PWMA_AF				GPIO_AF1_TIM1
#define MD1_CHANNEL_A			TIM_CHANNEL_2

#define MD1_PWMB_Pin			GPIO_PIN_10
#define MD1_PWMB_GPIO_Port		GPIOA
#define MD1_PWMB_AF				GPIO_AF1_TIM1
#define MD1_CHANNEL_B			TIM_CHANNEL_3

#define MD1_AIN1_Pin			GPIO_PIN_12
#define MD1_AIN1_GPIO_Port		GPIOB
#define MD1_AIN2_Pin			GPIO_PIN_13
#define MD1_AIN2_GPIO_Port		GPIOB
#define MD1_BIN1_Pin			GPIO_PIN_14
#define MD1_BIN1_GPIO_Port		GPIOB
#define MD1_BIN2_Pin			GPIO_PIN_15
#define MD1_BIN2_GPIO_Port		GPIOB

#define MD1_STBY_Pin			GPIO_PIN_2
#define MD1_STBY_GPIO_Port		GPIOB

#define MD2_PWMA_Pin			GPIO_PIN_6
#define MD2_PWMA_GPIO_Port		GPIOC
#define MD2_PWMA_AF				GPIO_AF1_TIM2
#define MD2_CHANNEL_A			TIM_CHANNEL_1

#define MD2_PWMB_Pin			GPIO_PIN_7
#define MD2_PWMB_GPIO_Port		GPIOC
#define MD2_PWMB_AF				GPIO_AF1_TIM2
#define MD2_CHANNEL_B			TIM_CHANNEL_2

#define MD2_AIN1_Pin			GPIO_PIN_14
#define MD2_AIN1_GPIO_Port		GPIOB
#define MD2_AIN2_Pin			GPIO_PIN_15
#define MD2_AIN2_GPIO_Port		GPIOB
#define MD2_BIN1_Pin			GPIO_PIN_3
#define MD2_BIN1_GPIO_Port		GPIOC
#define MD2_BIN2_Pin			GPIO_PIN_4
#define MD2_BIN2_GPIO_Port		GPIOC

#define MD2_STBY_Pin			GPIO_PIN_2
#define MD2_STBY_GPIO_Port		GPIOC

#define MD2_RCC_CL_ENABLE		__HAL_RCC_TIM3_CLK_ENABLE
#define MD1_RCC_CL_ENABLE		__HAL_RCC_TIM1_CLK_ENABLE

// Definitions for IMU
#define TIM_IMU			        TIM4						// TIM3 TIM4
#define TIM_IMU_CLK_ENABLE		__TIM4_CLK_ENABLE			// __TIM3_CLK_ENABLE __TIM4_CLK_ENABLE
#define TIM_IMU_CLK_DISABLE     __TIM4_CLK_DISABLE			// __TIM3_CLK_DISABLE __TIM4_CLK_DISABLE
#define TIM_SF_IRQn				TIM4_IRQn					// TIM3_IRQn TIM4_IRQn
#define TIM_IMU_IRQHandler		TIM4_IRQHandler				// TIM3_IRQHandler TIM4_IRQHandler

/* Definitions for Servo clock resources */
#define TIM_SERVO		                     TIM4
#define TIM_SERVO_CLK_ENABLE                 __TIM4_CLK_ENABLE
#define TIM_SERVO_CLK_DISABLE                __TIM4_CLK_DISABLE
#define SERVO_CHANNEL_1						TIM_CHANNEL_1
#define SERVO_CHANNEL_2						TIM_CHANNEL_2
#define SERVO1_PWM_PIN						GPIO_PIN_6
#define SERVO2_PWM_PIN						GPIO_PIN_7
#define SERVO_GPIO_Port						GPIOB

/* Definitions for UWB resources */
#define UWB_SPIx                             SPI3
#define UWB_SPIx_CLK_ENABLE                  __SPI3_CLK_ENABLE
#define UWB_SPIx_CLK_DISABLE                 __SPI3_CLK_DISABLE
#define UWB_SPIx_SCK_GPIO_CLK_ENABLE	     __GPIOC_CLK_ENABLE
#define UWB_SPIx_MISO_GPIO_CLK_ENABLE        __GPIOC_CLK_ENABLE
#define UWB_SPIx_MOSI_GPIO_CLK_ENABLE		 __GPIOC_CLK_ENABLE
#define UWB_SPIx_NSS_GPIO_CLK_ENABLE		 __GPIOA_CLK_ENABLE
#define UWB_SPIx_FORCE_RESET                 __SPI3_FORCE_RESET
#define UWB_SPIx_RELEASE_RESET               __SPI3_RELEASE_RESET
#define UWB_SPIx_SCK_PIN                     GPIO_PIN_10
#define UWB_SPIx_SCK_GPIO_PORT               GPIOC
#define UWB_SPIx_NSS_PORT	                 GPIOA
#define UWB_SPIx_NSS_PIN                     GPIO_PIN_15
#define UWB_SPIx_NSS_AF                      GPIO_AF6_SPI3
#define UWB_SPIx_SCK_AF                      GPIO_AF6_SPI3
#define UWB_SPIx_MISO_PIN                    GPIO_PIN_11
#define UWB_SPIx_MISO_GPIO_PORT              GPIOC
#define UWB_SPIx_MISO_AF                     GPIO_AF6_SPI3
#define UWB_SPIx_MOSI_PIN                    GPIO_PIN_12
#define UWB_SPIx_MOSI_GPIO_PORT              GPIOC
#define UWB_SPIx_MOSI_AF                     GPIO_AF6_SPI3  

/* Definition for Encoder resources */
#define TIM_ENCODER1	                    TIM2
#define TIM_ENCODER1_CLK_ENABLE             __TIM2_CLK_ENABLE
#define TIM_ENCODER1_CLK_DISABLE            __TIM2_CLK_DISABLE
#define ENCODER_1A_Pin						GPIO_PIN_0
#define ENCODER_1B_Pin						GPIO_PIN_1
#define ENCODER_1_Alternate					GPIO_AF1_TIM2
#define ENCODER_1_Port						GPIOA

#define TIM_ENCODER2	                    TIM3
#define TIM_ENCODER2_CLK_ENABLE             __TIM3_CLK_ENABLE
#define TIM_ENCODER2_CLK_DISABLE            __TIM3_CLK_DISABLE
#define ENCODER_2A_Pin						GPIO_PIN_6
#define ENCODER_2B_Pin						GPIO_PIN_7
#define ENCODER_2_Alternate					GPIO_AF2_TIM3
#define ENCODER_2_Port						GPIOC

/* Definition for Stepper Motors resources */
#define STEPPER_MOTOR_1						0
#define STEPPER_MAX_CAMERA_SPEED			4800
#define STEPPER_MIN_CAMERA_SPEED			20

/* Definition for Sonar sensor */
#define SONAR_GPIO_Port						GPIOB
#define SONAR_IN_Pin						GPIO_PIN_15
#define SONAR_VOLTAGE						3.3f
#define SONAR_UPDATE_TIME					500
#define SONAR_STATUS_SUCCESS				0
#define SONAR_STATUS_ERROR					1

#define WIFI_PING_TIMEOUT					3600

#define MAG_DEFAULT_OFFSET_X				58
#define MAG_DEFAULT_OFFSET_Y				-330
#define MAG_DEFAULT_OFFSET_Z				592

#define MIN_THROTTLE						1000
#define MAX_THROTTLE						2000
#define MIN_FLIGHT_THROTTLE					1100
#define MAX_FLIGHT_THROTTLE					1950
#define HOVER_FLIGHT_THROTTLE				1500
#define DESCENT_FLIGHT_THROTTLE				1400

#define INSTRUCTION_PING					0
#define INSTRUCTION_CONTROL_TYPE			1
#define	INSTRUCTION_FORWARD					2
#define	INSTRUCTION_BACKWARD				3
#define	INSTRUCTION_LEFT					4
#define	INSTRUCTION_RIGHT					5
#define	INSTRUCTION_TRANSLATE				6
#define	INSTRUCTION_ROTATE					7
#define INSTRUCTION_THROTTLE				100
#define INSTRUCTION_YAW						101
#define INSTRUCTION_PITCH					102
#define INSTRUCTION_ROLL					103
#define INSTRUCTION_TRIM_THROTTLE			104
#define INSTRUCTION_TRIM_YAW				105
#define INSTRUCTION_TRIM_PITCH				106
#define INSTRUCTION_TRIM_ROLL				107
#define INSTRUCTION_YAW_PID					108
#define INSTRUCTION_PITCH_PID				109
#define INSTRUCTION_ROLL_PID				110

// 250 Quad H Setup
#define MOTOR_A 0x01
#define MOTOR_B 0x02
#define MOTOR_C 0x04
#define MOTOR_D 0x08
#define MOTOR_ALL 0xFF

#define CTRL_UPDATE_TIME 1000

//  Battery and Voltage defines
#ifndef ADC_SUPPLY_VOLTAGE
#define ADC_SUPPLY_VOLTAGE		3300
#define VBAT_UPDATE_TIME		1000
#endif

// Multipliers for VBAT measurement
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx)
#define ADC_VBAT_MULTI			2
#else
#define ADC_VBAT_MULTI			4
#endif


// 150 Quad X Setup
//#define MOTOR_D 0x04
//#define MOTOR_A 0x08
//#define MOTOR_B 0x01
//#define MOTOR_C 0x02
//#define MOTOR_ALL 0xFF

#define Nsta 3		// States - PosX, PosY, Yaw
#define Mobs 6		// Observables - UWB_x, UWB_y, DD_x, DD_y, IMU_yaw, DD_yaw


// Debugging
#define DEBUG_FLIGHT_CONTROL_NO_CONNECT

/**
     * @brief Rotation modes.
     */
typedef enum
{
	BWD = 0, /* Backward. */
	FWD = 1,  /* Forward. */
	NONE = 2
} direction_t;

typedef enum
{
	CONTROL_STATE_IDLE         = 0,
	CONTROL_STATE_CONNECTED    = 1,
	CONTROL_STATE_DISCONNECTED = 2,
} CONTROL_STATE;

typedef enum
{
	CONTROLLER_ROVER			= 0,
	CONTROLLER_STEPPER_CAMERA	= 1,
	CONTROLLER_SERVO_CAMERA		= 2,
	CONTROLLER_FLIGHT			= 3,
	CONTROLLER_BALANCE			= 4,
	CONTROLLER_PROGRAMMER		= 5,	
	CONTROLLER_USER				= 6,
	
	CONTROLLER_COUNT			= 7
} CONTROLLER_TYPE;

typedef void(*AngularPositionCallback)(float);
typedef void(*DistanceCallback)(uint32_t);

#endif /* _CONSTANTS_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
