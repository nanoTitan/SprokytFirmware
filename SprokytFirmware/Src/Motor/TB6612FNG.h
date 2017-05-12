/* File: TB6612FNG.h
 * Author: Robert Abad      Copyright (c) 2013
 *
 * Desc: driver for Toshiiba TB6612FNG Motor Driver.  Though this motor driver
 *       can be used to drive two motors (A and B), it can be used to drive
 *       just one. Member functions can be used to set period and pulsewidth
 *       but are not necessary as the constructor does set default values).
 *       The datasheet for this device can be found here:
 *       http://www.toshiba.com/taec/components2/Datasheet_Sync/261/27197.pdf
 *
 *       Below is some sample code:
 *
 *       #include "mbed.h"
 *       #include "TB6612FNG.h"
 *
 *       #define TB6612FNG_PIN_PWMA      (p22)
 *       #define TB6612FNG_PIN_AIN1      (p17)
 *       #define TB6612FNG_PIN_AIN2      (p16)
 *       #define TB6612FNG_PIN_PWMB      (p21)
 *       #define TB6612FNG_PIN_BIN1      (p19)
 *       #define TB6612FNG_PIN_BIN2      (p20)
 *       #define TB6612FNG_PIN_NSTBY     (p18)
 *       TB6612FNG motorDriver( TB6612FNG_PIN_PWMA, TB6612FNG_PIN_AIN1, TB6612FNG_PIN_AIN2,
 *                              TB6612FNG_PIN_PWMB, TB6612FNG_PIN_BIN1, TB6612FNG_PIN_BIN2,
 *                              TB6612FNG_PIN_NSTBY );
 *       float fPwmPeriod;
 *       float fPwmPulsewidth;
 *
 *       int main()
 *       {
 *           fPwmPeriod = 0.00002f;      // 50KHz
 *           fPwmPulsewidth = 0.50;      // 50% duty cycle
 *           motorDriver.setPwmAperiod(fPwmPeriod);
 *           motorDriver.setPwmBperiod(fPwmPeriod);
 *           motorDriver.setPwmApulsewidth(fPwmPulsewidth);
 *           motorDriver.setPwmBpulsewidth(fPwmPulsewidth);
 *   
 *           while(1)
 *           {
 *               motorDriver.motorA_ccw();
 *               wait(2);
 *               motorDriver.motorA_cw();
 *               wait(2);
 *               motorDriver.motorA_stop();
 *               wait(2);
 *               motorDriver.motorB_ccw();
 *               wait(2);
 *               motorDriver.motorB_cw();
 *               wait(2);
 *               motorDriver.motorB_stop();
 *               wait(2);
 *           }
 *       }
 */

#include <stdint.h>

typedef enum
{ 
	TB_CHANNEL_A1 = 0,
	TB_CHANNEL_B1 = 1,
	TB_CHANNEL_A2 = 2,
	TB_CHANNEL_B2 = 3,
	
	TB_CHANNEL_COUNT
} TB_CHANNEL;

typedef enum
{
	TB_CM_SHORT_BREAK  = 0,
	TB_CM_CW,
	TB_CM_CCW,
	TB_CM_STOP,
	
	TB_CONTROL_MODE_COUNT
} TB_CONTROL_MODE;

#ifndef _TB6612FNG_H_
#define _TB6612FNG_H_

void TB_Init();
void TB_SetPwm(int iMotorChannel, uint16_t fFrequency, float fPulsewidth);
void TB_SetPwmFrequency(uint16_t fFrequency);
void TB_SetPwmPulsewidth(int tb_channel, float fPulsewidth);
void TB_SetWorkMode(int tb_channel, int tb_control_mode);

#endif // _TB6612FNG_H_