#include "TB6612FNG.h"
#include "hw_init.h"
#include "stm32f4xx_hal_conf.h"
#include "constants.h"

#define SIGNAL_HIGH     (1)
#define SIGNAL_LOW      (0)

#define TB6612FNG_PWM_PERIOD_DEFAULT      (0.00002)   // 50KHz
#define TB6612FNG_PWM_PULSEWIDTH_DEFAULT  (0.50)      // 50% duty cycle

// Private variables
uint16_t maxDutyCycle = 256;
uint16_t pwmFrequency = 10000;

// Private function declarations


void TB_Init()
{
//	PinName pinPwmA,
//	PinName pinAin1,
//	PinName pinAin2,
//	PinName pinPwmB,
//	PinName pinBin1,
//	PinName pinBin2,
//	PinName pinNStby)
//	: pwmA(pinPwmA)
//	, Ain1(pinAin1)
//	, Ain2(pinAin2)
//	, pwmB(pinPwmB)
//	, Bin1(pinBin1)
//	, Bin2(pinBin2)
//	, nStby(pinNStby)
//		
//		
//	Ain1 = SIGNAL_LOW;
//	Ain2 = SIGNAL_LOW;
//	Bin1 = SIGNAL_LOW;
//	Bin2 = SIGNAL_LOW;
//	pwmA.period(TB6612FNG_PWM_PERIOD_DEFAULT);
//	pwmA = TB6612FNG_PWM_PULSEWIDTH_DEFAULT;
//	pwmB.period(TB6612FNG_PWM_PERIOD_DEFAULT);
//	pwmB = TB6612FNG_PWM_PULSEWIDTH_DEFAULT;
//	nStby = SIGNAL_LOW;
	
}

void TB_SetPwm(int iMotorChannel, uint16_t fFrequency, float fPulsewidth)
{
	TB_SetPwmFrequency(fFrequency);
	TB_SetPwmPulsewidth(iMotorChannel, fPulsewidth);
}

void TB_SetPwmFrequency(uint16_t fFrequency)
{
	pwmFrequency = fFrequency;
	HW_PWMSetFrequency(TIMER_1, pwmFrequency);
	HW_PWMSetFrequency(TIMER_3, pwmFrequency);
}

void TB_SetPwmPulsewidth(int tb_channel, float fPulsewidth)
{
	if (fPulsewidth < 0)
		fPulsewidth = 0;	
	else if (fPulsewidth > 1)
		fPulsewidth = 1;
	
	int compare = pwmFrequency * maxDutyCycle * fPulsewidth;
	int timer = TIMER_3;
	int channel = TIM_CHANNEL_1;
	
	switch (tb_channel)
	{
	case TB_CHANNEL_A1:
		timer = TIMER_3;
		channel = TIM_CHANNEL_4;
		break;
		
	case TB_CHANNEL_B1:
		timer = TIMER_3;
		channel = TIM_CHANNEL_3;
		break;
		
	case TB_CHANNEL_A2:
		timer = TIMER_3;
		channel = TIM_CHANNEL_1;
		break;
		
	case TB_CHANNEL_B2:
		channel = TIM_CHANNEL_2;
		break;
		
	default:
		return;
	}
	
	HW_PwmTIM3SetCompare(channel, compare);
}

void TB_1A_Standby()
{
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN2_Pin, GPIO_PIN_SET);
}

void TB_1A_Stop()
{
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_1A_MotorCW()
{
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_1A_MotorCCW()
{
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_PWMA_GPIO_Port, MD1_AIN2_Pin, GPIO_PIN_SET);
}

void TB_1B_Standby()
{
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN2_Pin, GPIO_PIN_SET);
}

void TB_1B_Stop()
{
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_1B_MotorCW()
{
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_1B_MotorCCW()
{
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD1_PWMB_GPIO_Port, MD1_BIN2_Pin, GPIO_PIN_SET);
}

void TB_2A_Standby()
{
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN2_Pin, GPIO_PIN_SET);
}

void TB_2A_Stop()
{
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_2A_MotorCW()
{
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN2_Pin, GPIO_PIN_RESET);
}

void TB_2A_MotorCCW()
{
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_PWMA_GPIO_Port, MD2_AIN2_Pin, GPIO_PIN_SET);
}

void TB_2B_Standby()
{
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN2_Pin, GPIO_PIN_SET);
}

void TB_2B_Stop()
{
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_2B_MotorCW()
{
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN2_Pin, GPIO_PIN_RESET);
}

void TB_2B_MotorCCW()
{
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MD2_PWMB_GPIO_Port, MD2_BIN2_Pin, GPIO_PIN_SET);
}

void TB_SetWorkMode(int tb_channel, int tb_control_mode)
{
	static void( *TB_ControlModeFunctionArray[][4] )(void) = 
	{
		{ TB_1A_Standby, TB_1A_MotorCW, TB_1A_MotorCCW, TB_1A_Stop },
		{ TB_1B_Standby, TB_1B_MotorCCW, TB_1B_MotorCW, TB_1B_Stop },
		{ TB_2A_Standby, TB_2A_MotorCW, TB_2A_MotorCCW, TB_2A_Stop },
		{ TB_2B_Standby, TB_2B_MotorCCW, TB_2B_MotorCW, TB_2B_Stop },
	};
	
	if (tb_channel < 0 || tb_channel >= TB_CHANNEL_COUNT)
		return;
	
	if (tb_control_mode < 0 || tb_control_mode >= TB_CONTROL_MODE_COUNT)
		return;
	
	TB_ControlModeFunctionArray[tb_channel][tb_control_mode]();
}
