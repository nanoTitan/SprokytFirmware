/**
  @page Motion_Control_IHM06A1_ExampleFor1Motor  
  Example of one motor driving with an STSPIN220 (used via expansion board IHM06A1)
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    Multi/Examples/MotionControl/IHM06A1_ExampleFor1Motor/readme.txt  
  * @author  IPC Rennes
  * @version V1.2.0
  * @date    March 15th, 2017
  * @brief   Description of the example of one motor driving with STSPIN220
  ******************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

This example describes how to use the STSPIN220 FW library to drive one motor.


@par Directory contents 

  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32f3xx_hal_conf.h    HAL configuration file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32f0xx_hal_conf.h    HAL configuration file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32l0xx_hal_conf.h    HAL configuration file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32f3xx_it.h          Interrupt handlers header file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32f0xx_it.h          Interrupt handlers header file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/stm32l0xx_it.h          Interrupt handlers header file
  - MotionControl/IHM06A1_ExampleFor1Motor/Inc/main.h                  Header for main.c module
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32f4xx_it.c          Interrupt handlers
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32f3xx_it.c          Interrupt handlers
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32f0xx_it.c          Interrupt handlers
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32l0xx_it.c          Interrupt handlers    
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/main.c                  Main program
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/system_stm32f4xx.c      System source file
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/system_stm32f3xx.c      System source file
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/system_stm32f0xx.c      System source file
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/system_stm32l0xx.c      System source file
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32f4xx_hal_msp.c     HAL MSP module
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32f3xx_hal_msp.c     HAL MSP module
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32f0xx_hal_msp.c     HAL MSP module
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/stm32l0xx_hal_msp.c     HAL MSP module
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/clock_f4.c              Clock configuration
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/clock_f3.c              Clock configuration
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/clock_f0.c              Clock configuration
  - MotionControl/IHM06A1_ExampleFor1Motor/Src/clock_l0.c              Clock configuration


@par Hardware and Software environment

  This example requires :
    - Either a NUCLEO-F401RE, NUCLEO-F334R8, NUCLEO-F030R8 or NUCLEO-L053R8 board : a STM32 Nucleo development board for STM32 either F4, F3, F0 or L0 series
    - an X-NUCLEO-IHM06A1 board: a stepper motor driver expansion board based on the STSPIN220
    - one bipolar stepper motor connected to the X-NUCLEO-IHM06A1 board   

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
