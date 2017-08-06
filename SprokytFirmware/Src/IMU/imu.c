
/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include "main.h"
#include "cube_hal.h"
#include "control_manager.h"
#include "math_ext.h"
#include "sensor.h"
#include <string.h> // strlen
#include <stdio.h>  // sprintf
#include "debug.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_nucleo.h"
#include "MotionFX_Manager.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "error.h"


/* Extern variables ----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SAMPLE_FREQ                     ((uint8_t)100)  /* [Hz] */
#define SAMPLE_PERIOD                   ((uint8_t)10)   /* [ms] */
#define MOTIONFX_ENGINE_DELTATIME       0.01f

// Enable sensor masks
#define ACCELEROMETER_SENSOR            ((uint32_t)0x00000010)
#define GYROSCOPE_SENSOR                ((uint32_t)0x00000020)
#define MAGNETIC_SENSOR                 ((uint32_t)0x00000040)

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f

/* Public variables ---------------------------------------------------------*/
MFX_output_t mfx_output;
uint8_t magcal_request = 0;
TIM_HandleTypeDef ImuTimHandle;

/* Private variables ---------------------------------------------------------*/
static SensorAxes_t ACC_Value;
static SensorAxes_t GYR_Value;
static SensorAxes_t MAG_Value;
static SensorAxes_t MAG_Offset;
static ImuFunctionCallback imuFunc = NULL;
static void *ACCELERO_handle = NULL;
static void *GYRO_handle = NULL;
static void *MAGNETO_handle = NULL;
static void *PRESSURE_handle = NULL;
static uint32_t Sensors_Enabled = 0;
static uint8_t SF_6X_Enabled = 1;
static uint8_t calibIndex = 0;         // run calibration @ 25Hz
static uint8_t mag_cal_status = 0;
static uint8_t sensor_fusion_active = 0;
volatile static uint8_t sensor_read_request     = 0;

/* Private function prototypes -----------------------------------------------*/
static void ImuTimerInit();
static void CalibrateSensorFusion();
static void InitializeSensors();
static void EnableSensors();
static void Imu_Data_Handler();
static void Accelero_Sensor_Handler();
static void Gyro_Sensor_Handler();
static void Magneto_Sensor_Handler();

/* Private functions ---------------------------------------------------------*/
void IMU_init()
{
	char lib_version[35];
	int lib_version_len;
	
	memset(&mfx_output, 0, sizeof(MFX_output_t));
	memset(&ACC_Value, 0, sizeof(SensorAxes_t));
	memset(&GYR_Value, 0, sizeof(SensorAxes_t));
	memset(&MAG_Value, 0, sizeof(SensorAxes_t));
	memset(&MAG_Offset, 0, sizeof(SensorAxes_t));
	
	memset(&ImuTimHandle, 0, sizeof(TIM_HandleTypeDef));
	memset(&mfx_output, 0, sizeof(MFX_output_t));
	
	InitializeSensors();
	EnableSensors();
	
	ImuTimerInit();
  
	// Initialize Sensor Fusion
	MotionFX_manager_init(GYRO_handle);
	
	// Optional: Get library version
	MotionFX_manager_get_version(lib_version, &lib_version_len);
  
	if (SF_6X_Enabled)
		MotionFX_manager_start_6X();
	else
		MotionFX_manager_start_9X();
	
	// Check if the calibration is already available in memory
	unsigned char calibLoaded = MotionFX_LoadMagCalFromNVM(sizeof(SensorAxes_t), (unsigned int*)&MAG_Offset) ;
	
	if (calibLoaded == 1)
		PRINTF("IMU calibration successfully loaded\r\n");
	else
	{
		/* Enable magnetometer calibration */
		MotionFX_manager_MagCal_start(SAMPLE_PERIOD);

			  /* Test if calibration data are available */
		MFX_MagCal_output_t mag_cal_test;
		MotionFX_MagCal_getParams(&mag_cal_test);

		/* If calibration data are available, load HI coeficients */
		if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
		{
			MAG_Offset.AXIS_X = (int32_t)(mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
			MAG_Offset.AXIS_Y = (int32_t)(mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
			MAG_Offset.AXIS_Z = (int32_t)(mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);

			mag_cal_status = 1;
			BSP_LED_On(LED2);
		}
		else
		{
			PRINTF("IMU calibration not loaded. Run calibration to set magnetometer\r\n");
		}
		
		MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);
	}
}

/**
 * @brief  Initialize Sensor Fusion timer
 * @param  None
 * @retval None
 */
static void ImuTimerInit()
{
#define PERIOD_100HZ  ((uint8_t)20)

#if (defined (USE_STM32F4XX_NUCLEO))
//#define PRESCALER_100HZ  ((uint16_t)41999)	/* 84 MHZ CPU clock */
#define PRESCALER_100HZ  ((uint16_t)49999)		/* 100 MHZ CPU clock which is: (100 MHz / 2) - 1 */

#elif (defined (USE_STM32L4XX_NUCLEO))  /* 80 MHZ CPU clock */
#define PRESCALER_100HZ  ((uint16_t)39999)

#else
#error Not supported platform
#endif

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	ImuTimHandle.Instance = TIM_IMU;
	ImuTimHandle.Init.Prescaler = PRESCALER_100HZ;
	ImuTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	ImuTimHandle.Init.Period = PERIOD_100HZ;
	ImuTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&ImuTimHandle);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&ImuTimHandle, &sClockSourceConfig);
	
	HAL_TIM_OC_Init(&ImuTimHandle);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&ImuTimHandle, &sMasterConfig);
	
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_OC_ConfigChannel(&ImuTimHandle, &sConfigOC, TIM_IMU_CHANNEL);
	
	HAL_TIM_Base_Start_IT(&ImuTimHandle);
}

void RegisterImuCallback(ImuFunctionCallback callback)
{
	if (callback != NULL)
	{
		imuFunc = callback;
	}
}

/**
  * @brief  Period elapsed callback
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *              the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM_IMU)
	{
		sensor_read_request = 1;
	}
}

/**
 * @brief  Handles the Sensor Fusion
 * @param  Msg Sensor Fusion part of the stream
 * @retval None
 */
static void Imu_Data_Handler()
{
	MFX_input_t data_in;
	MFX_input_t *pdata_in = &data_in;
	MFX_output_t data_out;
	MFX_output_t *pdata_out = &data_out;

	if ( (Sensors_Enabled & ACCELEROMETER_SENSOR) && (Sensors_Enabled & GYROSCOPE_SENSOR) && (SF_6X_Enabled || (Sensors_Enabled & MAGNETIC_SENSOR)) )
	{
		data_in.gyro[0] = GYR_Value.AXIS_X  * FROM_MDPS_TO_DPS;
		data_in.gyro[1] = GYR_Value.AXIS_Y  * FROM_MDPS_TO_DPS;
		data_in.gyro[2] = GYR_Value.AXIS_Z  * FROM_MDPS_TO_DPS;

		data_in.acc[0] = ACC_Value.AXIS_X * FROM_MG_TO_G;
		data_in.acc[1] = ACC_Value.AXIS_Y * FROM_MG_TO_G;
		data_in.acc[2] = ACC_Value.AXIS_Z * FROM_MG_TO_G;

		data_in.mag[0] = MAG_Value.AXIS_X * FROM_MGAUSS_TO_UT50;
		data_in.mag[1] = MAG_Value.AXIS_Y * FROM_MGAUSS_TO_UT50;
		data_in.mag[2] = MAG_Value.AXIS_Z * FROM_MGAUSS_TO_UT50;

		/* Run Sensor Fusion algorithm */
		MotionFX_manager_run(pdata_in, pdata_out, MOTIONFX_ENGINE_DELTATIME);

		if (SF_6X_Enabled == 1)
		{
			memcpy(&mfx_output.quaternion_6X, (void *)pdata_out->quaternion_6X, MFX_QNUM_AXES * sizeof(float));
			memcpy(&mfx_output.rotation_6X, (void *)pdata_out->rotation_6X, MFX_NUM_AXES * sizeof(float));
			memcpy(&mfx_output.gravity_6X, (void *)pdata_out->gravity_6X, MFX_NUM_AXES * sizeof(float));
			memcpy(&mfx_output.linear_acceleration_6X, (void *)pdata_out->linear_acceleration_6X, MFX_NUM_AXES * sizeof(float));
			memcpy(&mfx_output.heading_6X, (void *)&(pdata_out->heading_6X), sizeof(float));
			
			// IMU Callback 6X
			if (imuFunc)
				imuFunc(mfx_output.rotation_6X, MFX_NUM_AXES);
		}
		else
		{
			memcpy(&mfx_output.quaternion_9X, (void *)pdata_out->quaternion_9X, MFX_QNUM_AXES * sizeof(float));
			memcpy(&mfx_output.rotation_9X, (void *)pdata_out->rotation_9X, MFX_NUM_AXES * sizeof(float));
			memcpy(&mfx_output.gravity_9X, (void *)pdata_out->gravity_9X, MFX_NUM_AXES * sizeof(float));
			memcpy(&mfx_output.linear_acceleration_9X, (void *)pdata_out->linear_acceleration_9X, MFX_NUM_AXES * sizeof(float));
			memcpy(&mfx_output.heading_9X, (void *)&(pdata_out->heading_9X), sizeof(float));
			
			// IMU Callback 9X
			if (imuFunc)
				imuFunc(mfx_output.rotation_9X, MFX_NUM_AXES);
		}
	}
}

void IMU_update(void)
{	
	/* Check if user button was pressed only when Sensor Fusion is active */
	if (magcal_request)
	{
		/* Reset the Compass Calibration */
		magcal_request = 0;
		mag_cal_status = 0;
		sensor_fusion_active = 1;
      
		MAG_Offset.AXIS_X = 0;
		MAG_Offset.AXIS_Y = 0;
		MAG_Offset.AXIS_Z = 0;
		
#if (defined (MOTION_FX_STORE_CALIB_FLASH))
		/* Reset values in memory */
		ResetCalibrationInMemory();
#endif
		
		/* Enable magnetometer calibration */
		MotionFX_manager_MagCal_start(SAMPLE_PERIOD);
      
		/* Switch off the LED */
		BSP_LED_Off(LED2);
	}
    
	if (sensor_read_request)
	{
		sensor_read_request = 0;
		
		Accelero_Sensor_Handler();
		Gyro_Sensor_Handler();
		
		if (!SF_6X_Enabled)
			Magneto_Sensor_Handler();
		
		// Sensor Fusion
		Imu_Data_Handler();
	}
}

/**
 * @brief  Initialize all sensors
 * @param  None
 */
void InitializeSensors(void)
{	
	PRINTF("Initializing IMU sensors\r\n");
	
	DrvStatusTypeDef result = 0;
		
	/* Try to use LSM6DS3 DIL24 if present, otherwise use LSM6DS0 on board */
	result = BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);		// ACCELERO_SENSORS_AUTO, LSM6DS3_X_0, LSM6DS0_X_0
	if (result != COMPONENT_OK)
		Error_Handler();
	else	
		Sensors_Enabled |= ACCELEROMETER_SENSOR;
	
	/* Try to use LSM6DS3 if present, otherwise use LSM6DS0 */
	result = BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
	if (result != COMPONENT_OK)
		Error_Handler();
	else
		Sensors_Enabled |= GYROSCOPE_SENSOR;
	
	/* Force to use LIS3MDL */
	if (!SF_6X_Enabled)
	{
		result = BSP_MAGNETO_Init(LIS3MDL_0, &MAGNETO_handle);
		if (result != COMPONENT_OK)
			Error_Handler();
		else
			Sensors_Enabled |= MAGNETIC_SENSOR;
	}
	
	PRINTF("IMU sensors initialized.\r\n");
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void EnableSensors(void)
{
	BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
	BSP_GYRO_Sensor_Enable(GYRO_handle);
	
	if (!SF_6X_Enabled)
		BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
	
	PRINTF("IMU sensors enabled.\r\n");
}

/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg ACCELERO part of the stream
 * @retval None
 */
void Accelero_Sensor_Handler()
{
	uint8_t status = 0;
  
	if (Sensors_Enabled & ACCELEROMETER_SENSOR)
	{
		if (BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
		{
			BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
		}
	}
}

/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg GYRO part of the stream
 * @retval None
 */
void Gyro_Sensor_Handler()
{
	uint8_t status = 0;
  
	if (Sensors_Enabled & GYROSCOPE_SENSOR)
	{
		if (BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
		{
			BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
		}
	}
}

/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg MAGNETO part of the stream
 * @retval None
 */
void Magneto_Sensor_Handler()
{
	uint8_t status = 0;
	MFX_MagCal_input_t mag_data_in;
	MFX_MagCal_output_t mag_data_out;
  
	if (Sensors_Enabled & MAGNETIC_SENSOR)
	{
		if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
		{
			BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
			
			if (mag_cal_status == 0)
			{
				MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

				if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
				{
					mag_cal_status = 1;

					MAG_Offset.AXIS_X = (int32_t)(mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
					MAG_Offset.AXIS_Y = (int32_t)(mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
					MAG_Offset.AXIS_Z = (int32_t)(mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);

					/* Disable magnetometer calibration */
					MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);

					/* Switch on the LED */
					BSP_LED_On(LED2);
				}
			}

			MAG_Value.AXIS_X = (int32_t)(MAG_Value.AXIS_X - MAG_Offset.AXIS_X);
			MAG_Value.AXIS_Y = (int32_t)(MAG_Value.AXIS_Y - MAG_Offset.AXIS_Y);
			MAG_Value.AXIS_Z = (int32_t)(MAG_Value.AXIS_Z - MAG_Offset.AXIS_Z);
		}
	}
}

float IMU_get_yaw()
{
	if (sensor_fusion_active)
	{
		if (SF_6X_Enabled)
			return mfx_output.rotation_6X[0];
		
		return mfx_output.rotation_9X[0];
	}
		
	return GYR_Value.AXIS_Z;
}

float IMU_get_pitch()
{
	if (sensor_fusion_active)
	{
		if (SF_6X_Enabled)
			return mfx_output.rotation_6X[1];
		
		return mfx_output.rotation_9X[1];
	}
	
	return GYR_Value.AXIS_Y;
}

float IMU_get_roll()
{
	if (sensor_fusion_active)
	{
		if (SF_6X_Enabled)
			return mfx_output.rotation_6X[2];
		
		return mfx_output.rotation_9X[2];
	}
	
	return GYR_Value.AXIS_X;
}
