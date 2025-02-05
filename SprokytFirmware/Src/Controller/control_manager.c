#include "control_manager.h"
#include "BLE.h"
//#include "PID.h"
//#include "user_control.h"
#include "rover_control.h"
//#include "camera_control.h"
//#include "flight_control.h"
//#include "esc_programmer.h"
#include "debug.h"
#include "stm32f4xx_hal_conf_template.h"

/* Private variables ---------------------------------------------------------*/
int m_controlState = CONTROL_STATE_IDLE;
int m_currControllerType = CONTROLLER_USER;
static uint32_t lastTick = 0;
uint32_t m_lastPing = 0;
uint32_t m_vbat = 0;
ADC_HandleTypeDef m_hADC;
DMA_HandleTypeDef  m_hDMA;

/* Private function prototypes -----------------------------------------------*/
static void ControlMgr_setInstruction(uint8_t instruction, uint8_t value);
static void ResetPing();
static void ControlMgr_initADC();
static void UpdateVBat();

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	uint32_t adcVal = HAL_ADC_GetValue(AdcHandle);
	m_vbat = (adcVal * ADC_VBAT_MULTI) * ADC_SUPPLY_VOLTAGE / 0xFFF;
}

void ADC_IRQHandler()
{
	HAL_ADC_IRQHandler(&m_hADC);
}

int ControlMgr_getState()
{
	return m_controlState;
}

void ControlMgr_setState(int state)
{
	m_controlState = state;
}

void ControlMgr_setType(int ctrlType)
{
	m_currControllerType = ctrlType;
	PRINTF("Setting Control State: %d\r\n", ctrlType);
}

void ControlMgr_init()
{
#if defined(VBAT_ENABLED)
	ControlMgr_initADC();
#endif // VBAT_ENABLED
	
	RoverControl_init();
	//CameraControl_init();
	//ServoCameraControl_init();
	//FlightControl_init();
	//UserControl_init();
}

void ControlMgr_deinit()
{
	__HAL_RCC_ADC1_CLK_DISABLE();
}

uint32_t ControlMgr_Get_BatteryVoltage()
{
	return m_vbat;
}

void ControlMgr_update()
{
#if defined(VBAT_ENABLED)
	UpdateVBat();
#endif // VBAT_ENABLED
	
	if (ControlMgr_getState() == CONTROL_STATE_CONNECTED)
	{
#if defined(WIFI_ENABLED)
		uint32_t delta = HAL_GetTick() - m_lastPing;
		if (delta > WIFI_PING_TIMEOUT)							// Ping check
		{
			ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
		}
#endif // WIFI_ENABLED
		
#if defined(BLE_ENABLED)
		if (!BLE_IsConnected() && ControlMgr_getState() != CONTROL_STATE_DISCONNECTED)
		{
			ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
		}	
#endif // BLE_ENABLED
	}
	
	RoverControl_update();
}

static void ControlMgr_initADC()
{	
	__ADC1_CLK_ENABLE();
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	
	ADC_ChannelConfTypeDef sConfig = { 0 };

	m_hADC.Instance = ADC1;
	
	m_hADC.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; /* (must not exceed 36MHz) */
	m_hADC.Init.Resolution = ADC_RESOLUTION_12B;
	m_hADC.Init.ScanConvMode = DISABLE;
	m_hADC.Init.ContinuousConvMode = DISABLE;
	m_hADC.Init.DiscontinuousConvMode = DISABLE;
	m_hADC.Init.NbrOfDiscConversion = 0;
	m_hADC.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	m_hADC.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	m_hADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	m_hADC.Init.NbrOfConversion = 1;
	m_hADC.Init.DMAContinuousRequests = DISABLE;
	m_hADC.Init.EOCSelection = DISABLE;
	
	__HAL_RCC_ADC1_CLK_ENABLE();
	HAL_ADC_Init(&m_hADC);
	
    // Configure ADC channel
	sConfig.Channel      = ADC_CHANNEL_VBAT;
	sConfig.Rank         = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset       = 0;

	HAL_ADC_ConfigChannel(&m_hADC, &sConfig);
	// HAL_ADC_Start_IT(&m_hADC);
	
	lastTick = HAL_GetTick();
}

void ControlMgr_ConfigureDMA()
{
	__DMA2_CLK_ENABLE(); 
	m_hDMA.Instance = DMA2_Stream4;
  
	m_hDMA.Init.Channel  = DMA_CHANNEL_0;
	m_hDMA.Init.Direction = DMA_PERIPH_TO_MEMORY;
	m_hDMA.Init.PeriphInc = DMA_PINC_DISABLE;
	m_hDMA.Init.MemInc = DMA_MINC_ENABLE;
	m_hDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	m_hDMA.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	m_hDMA.Init.Mode = DMA_CIRCULAR;
	m_hDMA.Init.Priority = DMA_PRIORITY_HIGH;
	m_hDMA.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
	m_hDMA.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	m_hDMA.Init.MemBurst = DMA_MBURST_SINGLE;
	m_hDMA.Init.PeriphBurst = DMA_PBURST_SINGLE; 
    
	HAL_DMA_Init(&m_hDMA);
    
	__HAL_LINKDMA(&m_hADC, DMA_Handle, m_hDMA);
 
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);   
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

static void UpdateVBat()
{
	uint32_t deltaTick = HAL_GetTick() - lastTick;
	if (deltaTick < VBAT_UPDATE_TIME)
		return;
		
	HAL_ADC_Start(&m_hADC);
	if (HAL_ADC_PollForConversion(&m_hADC, 10) == HAL_ERROR)
	{
		return;
	}
	
	if (((HAL_ADC_GetState(&m_hADC) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG))
	{
		/* Get the converted value of regular channel in mV */
		float convertedvalue = (float)HAL_ADC_GetValue(&m_hADC);
		convertedvalue = (convertedvalue * 0.80586f);		// 3300 / 4095 = 0.80586
		m_vbat = (uint32_t)convertedvalue * ADC_VBAT_MULTI * ADC_SUPPLY_VOLTAGE / 0xFFF;
		
		//uint32_t dist = (uint32_t)(oneOverVoltageScale * convertedvalue);
		
		PRINTF("voltage: %u\n", (unsigned int)convertedvalue);
		//m_distanceFunc(dist);
	}
	
	lastTick += deltaTick;
}

void ControlMgr_setMotor(uint8_t motorIndex, uint8_t value, direction_t dir)
{	
	// Jump table to store set motor functions
	static void(* const pf[])(uint8_t, uint8_t, direction_t) = { /*NULL, FlightControl_setMotor, NULL, NULL, EscProgrammer_setMotor*/ };
	
	// Update current controller
	if (m_currControllerType < sizeof(pf) / sizeof(*pf))
	{
		if (pf[m_currControllerType] != NULL)
			pf[m_currControllerType](motorIndex, value, dir);
	}
}

void ControlMgr_parseInstruction(uint8_t data_length, uint8_t *att_data)
{	
	if (data_length == 0)
		return;
	
	// Check for ping
	uint8_t instruction = att_data[0];
	if (instruction == INSTRUCTION_PING)
	{
		ResetPing();		
		return;
	}
	
	// Check for controller type change
	if (instruction == INSTRUCTION_CONTROL_TYPE)
	{
		if (data_length != 2)
		{
			PRINTF("Error: Instruction length is not 2\r\n");	
			return;
		}	
		
		uint8_t type = att_data[1];
		if (type < CONTROLLER_COUNT)
		{
			ControlMgr_setType((CONTROLLER_TYPE)type);			
		}	
		
		ResetPing();
		return;
	}
	
	RoverControl_parseInstruction(data_length, att_data);
	ResetPing();
}

void ResetPing()
{
	ControlMgr_setState(CONTROL_STATE_CONNECTED);
	//PRINTF("dT: %lu\r\n", HAL_GetTick() - m_lastPing);	
	m_lastPing = HAL_GetTick();
}

void ControlMgr_setInstruction(uint8_t instruction, uint8_t value)
{	
	// Jump table to store functions for parsing instructions
	static void(* const pf[])(uint8_t, uint8_t) = { /*NULL, FlightControl_setInstruction, NULL, NULL, EscProgrammer_setInstruction*/};
	
	// Update current controller
	if (m_currControllerType < sizeof(pf) / sizeof(*pf))
	{
		if (pf[m_currControllerType] != NULL)
			pf[m_currControllerType](instruction, value);
	}
}
