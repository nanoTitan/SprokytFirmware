#include "encoder.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "error.h"
#include "constants.h"
#include <math.h>

static TIM_HandleTypeDef  timer1, timer2;
static uint32_t m_count1 = 0, m_count2 = 0;
static uint32_t m_lastTime = 0;
static float m_rotation1 = 0, m_rotation2 = 0;
static float m_deltaRad1 = 0, m_deltaRad2 = 0;		// The difference in radians since the last update
static float m_angVel1 = 0, m_angVel2 = 0;			// Rotational speed in radian per second
static int8_t m_dir1 = 0, m_dir2 = 0;				// Rotation direction. 0 for CW and 1 for CCW
const float Encoder_Rad_Per_Count = 2 * ENCODER_ONE_OVER_QUAD_COUNT_PER_REV;

static void EncoderInit(TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode);

void Encoder_Init()
{
	EncoderInit(&timer1, TIM_ENCODER1, ENCODER_COUNT_PER_REV+6, TIM_ENCODERMODE_TI12);
	PRINTF("Encoder for TIM2 initialized\n");

	EncoderInit(&timer2, TIM_ENCODER2, ENCODER_COUNT_PER_REV+6, TIM_ENCODERMODE_TI12);
	PRINTF("Encoder for TIM3 initialized\n");
}

void Encoder_Update()
{
	float currTime = HAL_GetTick() * 0.001f;
	float oneOverDeltaTime = 1.0f / (currTime - m_lastTime);
	
	uint32_t currCount1 = __HAL_TIM_GET_COUNTER(&timer1);
	m_dir1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer1);
	
	uint32_t currCount2 = __HAL_TIM_GET_COUNTER(&timer2);
	m_dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer2);
	
	// Calculate the velocities
	uint32_t deltaCnt1 = currCount1 - m_count1;
	uint32_t deltaCnt2 = currCount2 - m_count2;
	
	float currRot1 = currCount1 * Encoder_Rad_Per_Count;
	float currRot2 = currCount2 * Encoder_Rad_Per_Count;
	
	m_angVel1 = fabs(currRot1 - m_rotation1) * oneOverDeltaTime;
	m_angVel2 = fabs(currRot2 - m_rotation2) * oneOverDeltaTime;
	
	// Save the new counts and rotations
	m_count1 = currCount1;
	m_count2 = currCount2;
	
	m_rotation1 = currRot1;
	m_rotation2 = currRot2;
	
	m_lastTime = currTime;
	
	static int printCnt = 0;
	++printCnt;
	if (printCnt > 100000)
	{
		//PRINTF("%u, %u, %u, %u\n", (unsigned int)m_count1, (unsigned int)m_count2, m_dir1, m_dir2);	
		PRINTF("%2.3f, %2.3f\n", m_rotation1, m_rotation2);	
		
		printCnt = 0;
	}
}

float Encoder_GetRot1()
{
	return m_rotation1;
}

float Encoder_GetRot2()
{
	return m_rotation2;
}

float Encoder_GetDeltaRad1()
{
	return m_deltaRad1;
}

float Encoder_GetDeltaRad2()
{
	return m_deltaRad2;
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
