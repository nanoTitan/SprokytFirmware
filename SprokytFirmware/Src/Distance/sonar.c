#include "sonar.h"
#include "BLE.h"
#include "math_ext.h"
#include "debug.h"
#include <math.h>
#include <assert.h>

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef adcHandle;
static ADC_ChannelConfTypeDef sConfig;
static DistanceCallback m_distanceFunc = NULL;
static uint32_t lastTick = 0;
static float oneOverVoltageScale = 0;

/* Private function prototypes -----------------------------------------------*/
static void ADCx_Init();
static void ADCx_DeInit();
static void ADCx_MspInit(ADC_HandleTypeDef *hadc);
static void ADCx_MspDeInit(ADC_HandleTypeDef *hadc);
static void Sonar_calculateVoltageScaling();

void SendDistance();

/* Private functions ---------------------------------------------------------*/
uint8_t Sonar_init()
{	
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	ADCx_Init();

	ADC_ChannelConfTypeDef adcChannel;
	adcChannel.Channel = NUCLEO_ADCx_CHANNEL;
	adcChannel.SamplingTime = ADC_SAMPLETIME_144CYCLES;	// ADC_SAMPLETIME_3CYCLES;
	adcChannel.Rank = 1;
	adcChannel.Offset = 0;
	
	uint8_t status = HAL_ERROR;
	status = HAL_ADC_ConfigChannel(&adcHandle, &sConfig);
	if (status == HAL_ERROR)
	{
		PRINTF("Error: HAL ADC configuration failed during sonar initialization\n");
		return SONAR_STATUS_ERROR;
	}
 
	if (HAL_ADC_ConfigChannel(&adcHandle, &adcChannel) != HAL_OK)
	{
		asm("bkpt 255");
	}
	
	Sonar_calculateVoltageScaling();
	
	lastTick = HAL_GetTick();
	return SONAR_STATUS_SUCCESS;
}

static void Sonar_calculateVoltageScaling()
{
	/*
	(Vcc/512) = Vi
	Vcc = Supplied Voltage
	Vi = Volts per inch (Scaling)
	
	Example 1: Say you have an input voltage of +5.0V the formula would read:
	(5.0V/512) = 0.009766V per inch = 9.766mV per inch
	*/
	
	float voltageScale = SONAR_VOLTAGE / 512.0f;
	oneOverVoltageScale = 1.0f / voltageScale;
}

uint8_t Sonar_deinit()
{
	ADCx_DeInit();
}

void Sonar_update()
{
	if (m_distanceFunc == NULL)
		return;
	
	uint32_t deltaTick = HAL_GetTick() - lastTick;
	if (deltaTick > SONAR_UPDATE_TIME)
	{
		//SendDistance();
		lastTick += deltaTick;
	}
}

void Sonar_RegisterDistanceCallback(DistanceCallback callback)
{
	if (callback != NULL)
	{
		m_distanceFunc = callback;
	}
}

void SendDistance()
{
	HAL_ADC_Start(&adcHandle);
	if (HAL_ADC_PollForConversion(&adcHandle, 10) == HAL_ERROR)
	{
		return;
	}
	
	if (((HAL_ADC_GetState(&adcHandle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG))
	{
		/* Get the converted value of regular channel in mV */
		float convertedvalue = (float)HAL_ADC_GetValue(&adcHandle);
		convertedvalue = (convertedvalue * 0.80586f);		// 3300 / 4095 = 0.80586
		
		uint32_t dist = (uint32_t)(oneOverVoltageScale * convertedvalue);
		
		//PRINTF("%u", (unsigned int)dist);
		m_distanceFunc(dist);
	}
}

/**
  * @brief  Initializes ADC HAL.
  */
static void ADCx_Init(void)
{
	if (HAL_ADC_GetState(&adcHandle) == HAL_ADC_STATE_RESET)
	{
	  /* ADC Config */
		adcHandle.Instance                   = NUCLEO_ADCx;
		adcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4; /* (must not exceed 36MHz) */
		adcHandle.Init.Resolution            = ADC_RESOLUTION12b;
		adcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		adcHandle.Init.ContinuousConvMode    = DISABLE;
		adcHandle.Init.DiscontinuousConvMode = DISABLE;
		adcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
		adcHandle.Init.EOCSelection          = EOC_SINGLE_CONV;
		adcHandle.Init.NbrOfConversion       = 1;
		adcHandle.Init.DMAContinuousRequests = DISABLE; 
    
		ADCx_MspInit(&adcHandle);
		HAL_ADC_Init(&adcHandle);
	}
}

/**
  * @brief  Initializes ADC MSP.
  */
static void ADCx_MspInit(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
  
	/*** Configure the GPIOs ***/  
	/* Enable GPIO clock */
	NUCLEO_ADCx_GPIO_CLK_ENABLE();
  
	/* Configure the selected ADC Channel as analog input */
	GPIO_InitStruct.Pin = NUCLEO_ADCx_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NUCLEO_ADCx_GPIO_PORT, &GPIO_InitStruct);
  
	/*** Configure the ADC peripheral ***/ 
	/* Enable ADC clock */
	NUCLEO_ADCx_CLK_ENABLE(); 
}

/**
  * @brief  Initializes ADC HAL.
  */
static void ADCx_DeInit(void)
{
	adcHandle.Instance   = NUCLEO_ADCx;
    
	HAL_ADC_DeInit(&adcHandle);
	ADCx_MspDeInit(&adcHandle);
}

/**
  * @brief  DeInitializes ADC MSP.
  * @note ADC DeInit does not disable the GPIO clock
  */
static void ADCx_MspDeInit(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*** DeInit the ADC peripheral ***/ 
	/* Disable ADC clock */
	NUCLEO_ADCx_CLK_DISABLE(); 

	/* Configure the selected ADC Channel as analog input */
	GPIO_InitStruct.Pin = NUCLEO_ADCx_GPIO_PIN;
	HAL_GPIO_DeInit(NUCLEO_ADCx_GPIO_PORT, GPIO_InitStruct.Pin);

	/* Disable GPIO clock has to be done by the application*/
	/* NUCLEO_ADCx_GPIO_CLK_DISABLE(); */
}

