/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh_uartrx.c
 * @brief   low-level module handshake (LMH) utilities to handle DWM1001 SPI
 *          rx functionalities with DataReady pin enabled (DRDY). 
 *
 *          In Makefile, interface configuration needs to be defined as:
 *          INTERFACE_NUMBER = 2
 *
 *          This file describes the RX setup example. 
 
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lmh_spirx_drdy.h"
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "dwm_constants.h"
#include "lmh.h"

#define LMH_SPIRX_DRDY_HEADER_LENGTH           1
#define LMH_SPIRX_DRDY_SIZE_OFFSET             0
#if LMH_SPIRX_DRDY_HEADER_LENGTH == 2
#define LMH_SPIRX_DRDY_NUM_OFFSET              1
#endif   

#define LMH_SPIRX_DRDY_TIMEOUT_DEFAULT       1000

static int  LMH_SPIRX_DRDY_IntCfg(SPI_HandleTypeDef* spiHandle, uint8_t value);
void LMH_SPIRX_DRDY_DrdyCb(void);

static bool lmh_spirx_drdy_initialized[2] = { false, false };
static int  lmh_spirx_drdy_timeout = LMH_SPIRX_DRDY_TIMEOUT_DEFAULT;
static int  lmh_spirx_drdy_wait = HAL_SPI_WAIT_PERIOD;
static bool lmh_spirx_drdy_drdy_flag = false;
static int  lmh_spirx_wait = HAL_SPI_WAIT_PERIOD;
static int  lmh_spirx_timeout = LMH_SPIRX_TIMEOUT_DEFAULT;
static uint16_t tx_len = 0;

/**
 * @brief : initialises the UARTRX functions. 
 */
void LMH_SPIRX_DRDY_Init(SPI_HandleTypeDef* spiHandle)
{  
	LMH_SPIRX_Init(spiHandle);
   
	// TODO: Fix this so it only checks once (not a while loop)
	// if (LMH_SPIRX_DRDY_IntCfg(spiHandle, DWM1001_INTR_SPI_DATA_READY) == LMH_ERR)
	while (LMH_SPIRX_DRDY_IntCfg(spiHandle, DWM1001_INTR_SPI_DATA_READY) == LMH_ERR)
	{
		PRINTF("\n\t DW: LMH_SPIRX_DRDY_IntCfg() failed. \t >>>>>> Error <<<<<<\n");
		HAL_Delay(100);
		//return;
	}
   
	LMH_SPIRX_DRDY_SetTimeout(LMH_SPIRX_DRDY_TIMEOUT_DEFAULT);
	LMH_SPIRX_DRDY_SetWait(HAL_SPI_WAIT_PERIOD);     
	LMH_SPIRX_DRDY_PinCheck();    
	LMH_SPIRX_DRDY_SetToIdle(spiHandle);
}

/**
 * @brief : de-initialises the UARTRX functions. 
 */
void LMH_SPIRX_DRDY_DeInit(void)
{
}

/**
 * @brief : send the command to configure DWM1001 to enable Data Ready pin
 *
 * @return Error code
 */
static int LMH_SPIRX_DRDY_IntCfg(SPI_HandleTypeDef* spiHandle, uint8_t value)
{        
	uint8_t tx_data[DWM1001_TLV_MAX_SIZE];
	uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
	uint16_t rx_len;   
	tx_len = 0;
	tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_INT_CFG;
	tx_data[tx_len++] = 1;
	tx_data[tx_len++] = (uint8_t)value;    
	
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
	HAL_StatusTypeDef txResult = HAL_SPI_Transmit(spiHandle, tx_data, tx_len, DWM_SPI_TIMEOUT);
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
	
	if (txResult != HAL_OK)
	{
		PRINTF("\tDW: ERROR: module failed to set config\n"); 
		return LMH_ERR;
	}
	
	return LMH_SPIRX_WaitForRx(spiHandle, rx_data, &rx_len, 3);
}

/**
 * @brief : sets the DWM1001 module SPIRX functions into idle mode. 
 */
int LMH_SPIRX_DRDY_SetToIdle(SPI_HandleTypeDef* spiHandle)
{
	uint8_t dummy = 0xff;
	uint16_t length;
	int i = 3;
	uint8_t rxBuff[16] = { 0 };
	
	// Sending three 0xFF dummy bytes, each in a single transmission, sets the state to IDLE
	PRINTF("\tDW: Reseting DWM1001 to SPI:IDLE \n"); 
	
	while (i-- > 0)
	{
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
		HAL_StatusTypeDef txResult = HAL_SPI_Transmit(spiHandle, &dummy, 1, lmh_spirx_timeout);
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
		HAL_Delay(lmh_spirx_wait);
		if (txResult != HAL_OK)
		{
			PRINTF("\tDW: ERROR: module failed to be set to idle\n"); 
			return LMH_ERR;
		}
	}
	
	// The response data will become all dummy bytes of value 0xFF to indicate it is in IDLE
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
	HAL_StatusTypeDef rxResult = HAL_SPI_Receive(spiHandle, rxBuff, 16, lmh_spirx_timeout);
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
	
	if (rxBuff[0] != 0xFF)
	{
		PRINTF("\tDW: ERROR: module failed to be set to idle\n"); 
		return LMH_ERR;
	}
	
	return LMH_OK;
}

/**
 * @brief : on drdy pin going high/low, set/reset the lmh_spirx_drdy_drdy_flag 
 *
 * @return none
 */
void LMH_SPIRX_DRDY_DrdyCb(void)
{
	LMH_SPIRX_DRDY_PinCheck();
	//PRINTF("\tDW: DRDY pin rising edge detected.  %s \n", LMH_SPIRX_DRDY_PinGet() ? "+++" : "---"); 
}

/**
 * @brief : update lmh_spirx_drdy_drdy_flag status according to HAL_GPIO_DRDY status
 *
 * @return none
 */
void LMH_SPIRX_DRDY_PinCheck(void)
{   
	GPIO_PinState pinState = HAL_GPIO_ReadPin(UWB_INT_PORT, UWB_INT_PIN);
	lmh_spirx_drdy_drdy_flag = pinState == GPIO_PIN_SET ? true : false;
}

/**
 * @brief : get lmh_spirx_drdy_drdy_flag 
 *
 * @return lmh_spirx_drdy_drdy_flag
 */
bool LMH_SPIRX_DRDY_PinGet(void)
{   
   return lmh_spirx_drdy_drdy_flag;
}

/**
 * @brief : Set the SPIRX_DRDY time out period. 
 *
 * @param [in] timeout, SPIRX_DRDY time out period in ms
 */
void LMH_SPIRX_DRDY_SetTimeout(int timeout)
{
   lmh_spirx_drdy_timeout = timeout;
}

/**
 * @brief : Set the SPIRX_DRDY wait period between each poll of SIZE. 
 *
 * @param [in] wait, SPIRX_DRDY wait period in ms
 */
void LMH_SPIRX_DRDY_SetWait(int wait)
{
   lmh_spirx_drdy_wait = wait;
}

/**
 * @brief : wait length=exp_length for max time=lmh_spirx_drdy_wait
 *          needs LMH_SPIRX_DRDY_Init() at initialization 
 *
 * @param [out] data,       pointer to received data 
 * @param [out] length,     pointer to received data length 
 * @param [in] exp_length,  expected data length
 *
 * @return Error code
 */
int LMH_SPIRX_DRDY_WaitForRx(SPI_HandleTypeDef* spiHandle, uint8_t* data, uint16_t* length, uint16_t exp_length)
{   
	uint8_t len_header, sizenum[LMH_SPIRX_DRDY_HEADER_LENGTH];
	int timeout = lmh_spirx_drdy_timeout;
	int ret_val = LMH_OK;
	int wait_temp;
	
	// check invalid cases
	if(exp_length < DWM1001_TLV_RET_VAL_MIN_SIZE)
	{
		PRINTF("\tSPI: exp_length must be >= 3 >>>>>> Error <<<<<<\n");    
		return LMH_ERR;
	}
	
	// wait for SIZE to be ready 
	//PRINTF("\tDW: Rx step 1: \n");
	memset(sizenum, 0, LMH_SPIRX_DRDY_HEADER_LENGTH);
	//PRINTF("\tDW: Start wait for spirx_drdy ...\n");
	wait_temp = 0;
	while((!LMH_SPIRX_DRDY_PinGet()) && (timeout>0))
	{
		HAL_Delay(lmh_spirx_drdy_wait);
		wait_temp += lmh_spirx_drdy_wait;
		timeout-=lmh_spirx_drdy_wait;
		LMH_SPIRX_DRDY_PinCheck();    
	}   
	//PRINTF("\tDW: Waited %d ms for spirx_drdy ...\n", wait_temp); 
      
	//PRINTF("\tDW: Wait %d ms...\n", lmh_spirx_drdy_wait); 
	HAL_Delay(lmh_spirx_drdy_wait);
	timeout-=lmh_spirx_drdy_wait;
   
	if(timeout <= 0)
	{
		PRINTF("\tDW: Read SIZE timed out after %d ms... >>>>>> TIMED OUT <<<<<< \n", lmh_spirx_drdy_timeout);        
		LMH_SPIRX_DRDY_PinCheck();   
		ret_val = LMH_ERR;
	}
    
	// read SIZE & NUM
	len_header = LMH_SPIRX_DRDY_HEADER_LENGTH;
	
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
	HAL_StatusTypeDef result = HAL_SPI_Receive(spiHandle, sizenum, len_header, lmh_spirx_drdy_timeout);
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
      
	// wait for DATA to be ready 
	//PRINTF("\tDW: Rx step 2: \n");
	//PRINTF("\tDW: Start wait for spirx_drdy ...\n");
	wait_temp = 0;
	while((!LMH_SPIRX_DRDY_PinGet()) && (timeout>0))
	{
		HAL_Delay(lmh_spirx_drdy_wait);
		wait_temp += lmh_spirx_drdy_wait;
		timeout-=lmh_spirx_drdy_wait;
		LMH_SPIRX_DRDY_PinCheck();        
	}
	
	//PRINTF("\tDW: Waited %d ms for spirx_drdy ...\n", timeout); 
	
	if (timeout < 0)
	{
		PRINTF("\tDW: Read SIZE timed out after %d ms... >>>>>> TIMED OUT <<<<<< \n", lmh_spirx_timeout);  
		return LMH_ERR;
	}
	else if (sizenum[LMH_SPIRX_DRDY_SIZE_OFFSET] == 0 || sizenum[LMH_SPIRX_DRDY_SIZE_OFFSET] == 0Xff)
	{
		PRINTF("\tDW: Error: Invalid read SIZE %d\n", sizenum[LMH_SPIRX_DRDY_SIZE_OFFSET]);  
		return LMH_ERR;
	}
	
	// read DATA 
	*length = 0;
#if LMH_SPIRX_DRDY_HEADER_LENGTH == 2
	uint8_t i;
	for(i = 0; i < sizenum[LMH_SPIRX_DRDY_NUM_OFFSET]; i++)
#endif
	{     
		HAL_Delay(lmh_spirx_drdy_wait);
		timeout-=lmh_spirx_drdy_wait;
		//PRINTF("\tDW: Wait %d ms...\n", lmh_spirx_drdy_wait); 
		
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
		result = HAL_SPI_Receive(spiHandle, data, sizenum[LMH_SPIRX_DRDY_SIZE_OFFSET], lmh_spirx_drdy_timeout);
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
		
		if (result == HAL_TIMEOUT)
		{
			PRINTF("\tDW: Read SIZE timed out after %d ms... >>>>>> TIMED OUT <<<<<< \n", lmh_spirx_timeout);  
			return LMH_ERR;
		}
		else if(result != HAL_OK)
		{
			PRINTF("\tDW: ERROR - Read failed\n");  
			return LMH_ERR;
		}
		
		*length += sizenum[LMH_SPIRX_DRDY_SIZE_OFFSET];
	}
	
	HAL_Delay(lmh_spirx_drdy_wait);
	//PRINTF("\tDW: Wait %d ms...\n", lmh_spirx_drdy_wait); 
	LMH_SPIRX_DRDY_PinCheck();   
   
	if(LMH_CheckRetVal(data) != LMH_OK)
	{
		return LMH_ERR;
	}
   
	if((*length != exp_length) && (exp_length != DWM1001_TLV_MAX_SIZE))
	{
		PRINTF("\tDW: Error: Expecting %d bytes, received %d bytes, in %d ms\n", exp_length, *length, lmh_spirx_drdy_timeout-timeout);
		return LMH_ERR;   
	}
   
	//PRINTF("\tDW: Received %d bytes, in %d ms \t OK\n", *length, lmh_spirx_drdy_timeout-timeout);
	return ret_val;
}











