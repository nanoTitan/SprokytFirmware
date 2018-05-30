#include "encoder.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "error.h"
#include "constants.h"
#include <math_ext.h>

/* Public variables ----------------------------------------------------------*/
TIM_HandleTypeDef  encoder_timer1, encoder_timer2;
uint32_t encoder_elapsed_cyclecount1 = 0;
uint32_t encoder_elapsed_cyclecount2 = 0;
uint32_t encoder_last_cycletime = 0;
uint32_t encoder_test_count = 0;

/* Private variables ---------------------------------------------------------*/

static float m_lastCount1 = 0, m_lastCount2 = 0;
static uint32_t m_lastTime = 0;
static float m_lastAngle1 = 0, m_lastAngle2 = 0;
static float m_angVel1 = 0, m_angVel2 = 0;			// Rotational speed in radian per second
static int8_t m_dir1 = 0, m_dir2 = 0;				// Rotation direction. 0 for CW and 1 for CCW
static float m_encoderRadFreqPerCount = 0;

/* Constants ---------------------------------------------------------*/
const float kEncoderRadPerCount = M_2PI * ENCODER_ONE_OVER_COUNT_PER_REV;

const uint32_t kEncoderUpdateTime = 10;

static void EncoderInit(TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode);

void Encoder_Init()
{
	EncoderInit(&encoder_timer1, TIM_ENCODER1, ENCODER_COUNT_PER_REV, TIM_ENCODERMODE_TI12);
	PRINTF("Encoder for TIM2 initialized\n");

	EncoderInit(&encoder_timer2, TIM_ENCODER2, ENCODER_COUNT_PER_REV, TIM_ENCODERMODE_TI12);
	PRINTF("Encoder for TIM3 initialized\n");
	
	m_encoderRadFreqPerCount = M_2PI * SystemCoreClock / ((ENCODER_COUNT_PER_REV+1) / 4) ;	// Divide by four since interrupt is called once every count
}

void Encoder_Update()
{
	uint32_t currTime = HAL_GetTick();
	uint32_t deltaTime = currTime - m_lastTime;
	
	// If the cycle count isn't being updated, make sure to reset the counters so
	// that it doesn't report a false velocity
	if (currTime - encoder_last_cycletime > 200)
		Encoder_ClearCycleCounts();
	
	if (deltaTime < kEncoderUpdateTime)
	{
		return;
	}
	
	float oneOverDeltaTime = 1.0f / (deltaTime * 0.001f);
	
	float currCount1 = (float)__HAL_TIM_GET_COUNTER(&encoder_timer1);
	m_dir1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&encoder_timer1);
	
	float currCount2 = (float)__HAL_TIM_GET_COUNTER(&encoder_timer2);
	m_dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&encoder_timer2);
	
	// Check if our count values are wrapping
	uint8_t didCntWrap1 = 0;
	uint8_t didCntWrap2 = 0;
	if ((currCount1 > ENCODER_COUNT_MAX_WRAP_CHECK && m_lastCount1 < ENCODER_COUNT_MIN_WRAP_CHECK) ||
		(m_lastCount1 > ENCODER_COUNT_MAX_WRAP_CHECK && currCount1 < ENCODER_COUNT_MIN_WRAP_CHECK))
		didCntWrap1 = 1;
	
	if ((currCount2 > ENCODER_COUNT_MAX_WRAP_CHECK && m_lastCount2 < ENCODER_COUNT_MIN_WRAP_CHECK) ||
		(m_lastCount2 > ENCODER_COUNT_MAX_WRAP_CHECK && currCount2 < ENCODER_COUNT_MIN_WRAP_CHECK))
		didCntWrap2 = 1;
	
	float deltaCnt1 = currCount1 - m_lastCount1;
	float deltaCnt2 = currCount2 - m_lastCount2;
	
	if (didCntWrap1)
	{
		if(currCount1 < m_lastCount1)		// (m_dir1 == 0)	// Forward
			deltaCnt1 = currCount1 + ENCODER_COUNT_PER_REV - m_lastCount1;
		else
			deltaCnt1 = -(m_lastCount1 + ENCODER_COUNT_PER_REV - currCount1);
	}
	
	if (didCntWrap2)
	{
		if(currCount2 < m_lastCount2) // (m_dir2 == 0)	// Forward
			deltaCnt2 = currCount2 + ENCODER_COUNT_PER_REV - m_lastCount2;
		else
			deltaCnt2 = -(m_lastCount2 + ENCODER_COUNT_PER_REV - currCount2);
	}
	
	// Calculate dTheta
	float deltaAngle1 = deltaCnt1 * kEncoderRadPerCount;
	float deltaAngle2 = deltaCnt2 * kEncoderRadPerCount;
	
	// w = dTheta / dTime
	m_angVel1 = deltaAngle1 * oneOverDeltaTime;		
	m_angVel2 = deltaAngle2 * oneOverDeltaTime;
	
	// Save the new counts and angles
	m_lastAngle1 = currCount1 * kEncoderRadPerCount;
	m_lastAngle2 = currCount2 * kEncoderRadPerCount;
	m_lastCount1 = currCount1;
	m_lastCount2 = currCount2;
	m_lastTime = currTime;
	
#ifdef PRINT_ENCODER
	static uint32_t lastPrintTime = 0;
	if (currTime - lastPrintTime > 100)
	{
		//PRINTF("%.3f, %.3f\n", deltaTime, m_lastTime);
		//PRINTF("%u, %u, %u, %u\n", (unsigned int)m_lastCount1, (unsigned int)m_lastCount2, m_dir1, m_dir2);	
		//PRINTF("%2.3f, %2.3f\n", deltaTime, oneOverDeltaTime);	
		//PRINTF("%.3f, %.3f\n", currCount1, currCount2);	
		//PRINTF("%2.3f, %2.3f\n", deltaCnt1, deltaCnt2);	
		//PRINTF("%2.3f, %2.3f\n", deltaAngle1, deltaAngle2);	
		//PRINTF("%2.3f, %2.3f\n", m_lastAngle1, m_lastAngle2);	
		//PRINTF("PC: %.4f, %.4f\n", m_angVel1, m_angVel2);	
		//PRINTF("%u, %u\n", (unsigned int)encoder_elapsed_time1, (unsigned int)encoder_elapsed_time2);	
		//PRINTF("PT: %.4f, %.4f\n", Encoder_GetAngVelPulseTiming1(), Encoder_GetAngVelPulseTiming2());
		PRINTF("%u\n", (unsigned int)encoder_test_count);
		
		lastPrintTime = currTime;
	}
#endif // PRINT_ENCODER
}

float Encoder_GetAngle1()
{
	return m_lastAngle1;
}

float Encoder_GetAngle2()
{
	return m_lastAngle2;
}

float Encoder_GetAngVel1()
{
	return m_angVel1;
}

float Encoder_GetAngVel2()
{
	return m_angVel2;
}

float Encoder_GetAngVelPulseTiming1()
{
	// w = 2pi * f / Nm
	// f = clock frequency (Hz)
	// m = number of clock cycles
	// N = pulses per rotation
	if (encoder_elapsed_cyclecount1 == 0)	// Prevent divide by zero
		return 0;
	
	float w = m_encoderRadFreqPerCount / encoder_elapsed_cyclecount1;
	if (m_dir1 == 0)
		return -w;
	
	return w;
}

float Encoder_GetAngVelPulseTiming2()
{
	if (encoder_elapsed_cyclecount2 == 0)	// Prevent divide by zero
		return 0;
	
	float w = m_encoderRadFreqPerCount / encoder_elapsed_cyclecount2;
	if (m_dir2 == 0)
		return -w;
	
	return w;
}

int8_t Encoder_GetDir1()
{
	return m_dir1;
}

int8_t Encoder_GetDir2()
{
	return m_dir2;
}

void EncoderInit(TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode)
{
	static TIM_Encoder_InitTypeDef encoder;
	TIM_MasterConfigTypeDef sMasterConfig;
	
    timer->Instance = TIMx;
    timer->Init.Period = maxcount;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Prescaler = 0;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	encoder.EncoderMode = encmode;
	encoder.IC1Filter = 0x00;									// Orig: 0x0F							Alt: 0x00
	encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
	encoder.IC1Prescaler = TIM_ICPSC_DIV1;						// Orig: TIM_ICPSC_DIV4					Alt: TIM_ICPSC_DIV1
	encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	
	encoder.IC2Filter = 0x00;									// Orig: 0x0F							Alt: 0x00
	encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;		// Orig: TIM_INPUTCHANNELPOLARITY_FALLING	Alt: TIM_INPUTCHANNELPOLARITY_BOTHEDGE
	encoder.IC2Prescaler = TIM_ICPSC_DIV1;						// Orig: TIM_ICPSC_DIV4					Alt: TIM_ICPSC_DIV1
	encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    if (HAL_TIM_Encoder_Init(timer, &encoder) != HAL_OK) 
    {
        PRINTF("Error: Encoder failed to initialize\r\n");
	    Error_Handler();
    }

	// Start encoder in normal mode
//	if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_1) != HAL_OK) 
//	{
//		PRINTF("Error: Encoder failed to start\r\n");
//		Error_Handler();
//    }
	
	// Start encoder in interrupt mode
	if (HAL_TIM_Encoder_Start_IT(timer, TIM_CHANNEL_1) != HAL_OK) 
	{
		PRINTF("Error: Encoder failed to start\r\n");
		Error_Handler();
    }
	
	TIMx->EGR = 1;           // Generate an update event
	TIMx->CR1 = 1;           // Enable the counter
}

void Encoder_ClearCycleCounts()
{
	encoder_elapsed_cyclecount1 = 0;
	encoder_elapsed_cyclecount2 = 0;
}