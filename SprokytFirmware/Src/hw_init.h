
#ifndef _HW_INIT_H_
#define _HW_INIT_H_

#include <stdint.h>

typedef enum
{ 
	TIMER_1 = 0,
	TIMER_2 = 1,
	TIMER_3 = 2,
	TIMER_4 = 3
} MCU_TIMER;

void Hardware_Init();

/**
  * @brief  Sets the PWM period and pulse width for a given channel
  * @param  int: MCU_TIMER selected
  * @param  channel: TIM Channel to be configured.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  int: Period of the PWM signal
  * @param  int: pulse width percent of the PWM signal. This should be between 0 and 1.
  * @retval None
  */
void HW_PWMSetCompare(int timer, int channel, int compare);
void HW_PWMSetFrequency(int timer, uint16_t frequency);

#endif // _HW_INIT_H_