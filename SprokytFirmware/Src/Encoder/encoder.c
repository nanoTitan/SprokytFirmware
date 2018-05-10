#include "encoder.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "error.h"
#include "constants.h"
#include <math_ext.h>


static TIM_HandleTypeDef  timer1, timer2;
static float m_lastCount1 = 0, m_lastCount2 = 0;
static uint32_t m_lastTime = 0;
static float m_lastAngle1 = 0, m_lastAngle2 = 0;
static float m_angVel1 = 0, m_angVel2 = 0;			// Rotational speed in radian per second
static int8_t m_dir1 = 0, m_dir2 = 0;				// Rotation direction. 0 for CW and 1 for CCW
const float Encoder_Rad_Per_Count = M_2PI * ENCODER_ONE_OVER_QUAD_COUNT_PER_REV;

static void EncoderInit(TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode);

void Encoder_Init()
{
	EncoderInit(&timer1, TIM_ENCODER1, ENCODER_COUNT_PER_REV, TIM_ENCODERMODE_TI12);
	PRINTF("Encoder for TIM2 initialized\n");

	EncoderInit(&timer2, TIM_ENCODER2, ENCODER_COUNT_PER_REV, TIM_ENCODERMODE_TI12);
	PRINTF("Encoder for TIM3 initialized\n");
}

void Encoder_Update()
{
	uint32_t currTime = HAL_GetTick();
	uint32_t deltaTime = currTime - m_lastTime;
	
	if (deltaTime < 10)
	{
		return;
	}
	
	float oneOverDeltaTime = 1.0f / (deltaTime * 0.001f);
	
	float currCount1 = (float)__HAL_TIM_GET_COUNTER(&timer1);
	m_dir1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer1);
	
	float currCount2 = (float)__HAL_TIM_GET_COUNTER(&timer2);
	m_dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer2);
	
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
	float deltaAngle1 = deltaCnt1 * Encoder_Rad_Per_Count;
	float deltaAngle2 = deltaCnt2 * Encoder_Rad_Per_Count;
	
	// w = dTheta / dTime
	m_angVel1 = deltaAngle1 * oneOverDeltaTime;		
	m_angVel2 = deltaAngle2 * oneOverDeltaTime;
	
	// Save the new counts and angles
	m_lastAngle1 = currCount1 * Encoder_Rad_Per_Count;
	m_lastAngle2 = currCount2 * Encoder_Rad_Per_Count;
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
		PRINTF("%2.3f, %2.3f\n", deltaAngle1, deltaAngle2);	
		//PRINTF("%2.3f, %2.3f\n", m_lastAngle1, m_lastAngle2);	
		//PRINTF("%.4f, %.4f\n", m_angVel1, m_angVel2);	
		
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
	
//	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	if (HAL_TIMEx_MasterConfigSynchronization(timer, &sMasterConfig) != HAL_OK)
//	{
//		PRINTF("Error: Encoder could not syncronize with master config\r\n");
//	}

	if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_1) != HAL_OK) 
	{
		PRINTF("Error: Encoder failed to start\r\n");
		Error_Handler();
    }
	
	TIMx->EGR = 1;           // Generate an update event
	TIMx->CR1 = 1;           // Enable the counter
}
