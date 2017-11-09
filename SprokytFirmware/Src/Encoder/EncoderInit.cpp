#include "stm32f4xx_hal_conf.h"
#include "error.h"
#include "constants.h"

static TIM_Encoder_InitTypeDef encoder1, encoder2;
static TIM_HandleTypeDef  timer1, timer2;
static uint32_t count1, count2;
static int8_t dir1, dir2;

static void EncoderInit(TIM_Encoder_InitTypeDef * encoder, TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode);
static void Encoder_UpdateCounts();
static void Encoder_CalculateRotation();

void Encoder_Init()
{
	// counting on both A&B inputs, 4 ticks per cycle, full 32-bit count
	EncoderInit(&encoder1, &timer1, TIM_ENCODER1, 0xffffffff, TIM_ENCODERMODE_TI12);

	// counting on both A&B inputs, 4 ticks per cycle, full 32-bit count
	EncoderInit(&encoder2, &timer2, TIM_ENCODER2, 0xffffffff, TIM_ENCODERMODE_TI12);
	
	Encoder_UpdateCounts();
}

void Encoder_Update()
{
	Encoder_UpdateCounts();
	Encoder_CalculateRotation();
}

void Encoder_UpdateCounts()
{
	count1 = __HAL_TIM_GET_COUNTER(&timer1);
	dir1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer1);
	
	count2 = __HAL_TIM_GET_COUNTER(&timer2);
	dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer2);	
}

void Encoder_CalculateRotation()
{
	float rot1 = count1 * ENCODER_ONE_OVER_QUAD_COUNT_PER_REV;
	float rot2 = count2 * ENCODER_ONE_OVER_QUAD_COUNT_PER_REV;
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

