#include "Encoder.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "error.h"
#include "constants.h"
#include "math_ext.h"

static TIM_Encoder_InitTypeDef encoder1, encoder2;
static TIM_HandleTypeDef  timer1, timer2;
static uint32_t count1 = 0, count2 = 0;
static uint32_t m_lastTime = 0;
static float m_rotation1 = 0, m_rotation2 = 0;
static float m_deltaRad1 = 0, m_deltaRad2 = 0;		// The difference in radians since the last update
static float m_angVel1 = 0, m_angVel2 = 0;			// Rotational speed in radian per second
static int8_t m_dir1 = 0, m_dir2 = 0;				// Rotation direction. 0 for CW and 1 for CCW
const float Encoder_Rad_Per_Count = 2 * ENCODER_ONE_OVER_QUAD_COUNT_PER_REV;

static void EncoderInit(TIM_Encoder_InitTypeDef * encoder, TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode);

void Encoder_Init()
{
	// counting on both A&B inputs, 4 ticks per cycle, full 32-bit count
	EncoderInit(&encoder1, &timer1, TIM_ENCODER1, 0xffffffff, TIM_ENCODERMODE_TI12);

	// counting on both A&B inputs, 4 ticks per cycle, full 32-bit count
	EncoderInit(&encoder2, &timer2, TIM_ENCODER2, 0xffffffff, TIM_ENCODERMODE_TI12);
}

void Encoder_Update()
{
	float currTime = HAL_GetTick() * 0.001f;
	float oneOverDeltaTime = 1.0f / (currTime - m_lastTime);
	
	uint32_t currCount1 = __HAL_TIM_GET_COUNTER(&timer1);
	uint32_t currDir1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer1);
	
	uint32_t currCount2 = __HAL_TIM_GET_COUNTER(&timer2);
	uint32_t currDir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer2);
	
	// Calculate the velocities
	uint32_t deltaCnt1 = currCount1 - count1;
	uint32_t deltaCnt2 = currCount2 - count2;
	
	float currRot1 = currCount1 * Encoder_Rad_Per_Count;
	float currRot2 = currCount2 * Encoder_Rad_Per_Count;
	
	m_angVel1 = abs(currRot1 - m_rotation1) * oneOverDeltaTime;
	m_angVel2 = abs(currRot2 - m_rotation2) * oneOverDeltaTime;
	
	// Save the new counts and rotations
	count1 = currCount1;
	count2 = currCount2;
	m_dir1 = currDir1;
	m_dir2 = currDir2;
	
	while (count1 > ENCODER_COUNT_PER_REV)
		count1 -= ENCODER_COUNT_PER_REV;
	
	while (count2 > ENCODER_COUNT_PER_REV)
		count2 -= ENCODER_COUNT_PER_REV;
	
	m_rotation1 = currRot1;
	m_rotation2 = currRot2;
	
	m_lastTime = currTime;
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

void EncoderInit(TIM_Encoder_InitTypeDef * encoder, TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode)
{
    timer->Instance = TIMx;
    timer->Init.Period = maxcount;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Prescaler = 0;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    encoder->EncoderMode = encmode;

    encoder->IC1Filter = 0x0F;
    encoder->IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
    encoder->IC1Prescaler = TIM_ICPSC_DIV4;
    encoder->IC1Selection = TIM_ICSELECTION_DIRECTTI;

    encoder->IC2Filter = 0x0F;
    encoder->IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    encoder->IC2Prescaler = TIM_ICPSC_DIV4;
    encoder->IC2Selection = TIM_ICSELECTION_DIRECTTI;

    if (HAL_TIM_Encoder_Init(timer, encoder) != HAL_OK) 
    {
        printf("Couldn't Init Encoder\r\n");
	    Error_Handler();
    }

	if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_1) != HAL_OK) 
	{
        printf("Couldn't Start Encoder\r\n");
		Error_Handler();
    }
}

