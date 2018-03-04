/*! ------------------------------------------------------------------------------------------------------------------
 * @file    lmh_spirx.c
 * @brief   low-level module handshake (LMH) utilities to handle DWM1001 data 
 *          transmission and receiving over SPI interface
 *          Use LMH_SPIRX_Init() before using to initialize the utilities. 
 *          Use LMH_SPIRX_WaitForRx() to wait for response message
 *
 *          In Makefile, interface configuration needs to be defined as:
 *          INTERFACE_NUMBER = 1
 *
 *          This file describes the RX setup example. 
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "dwm_constants.h"
#include "lmh.h"

#define LMH_SPIRX_HEADER_LENGTH           1
#define LMH_SPIRX_SIZE_OFFSET             0
#if LMH_SPIRX_HEADER_LENGTH == 2
#define LMH_SPIRX_NUM_OFFSET              1
#endif   

static bool lmh_spirx_initialized[2]={false, false};
static int  lmh_spirx_timeout = LMH_SPIRX_TIMEOUT_DEFAULT;
static int  lmh_spirx_wait = HAL_SPI_WAIT_PERIOD;

static int LMH_SPIRX_IntCfg(SPI_HandleTypeDef* spiHandle, uint8_t value);
void LMH_SPIRX_DRDY_Cb();

/**
 * @brief : initialises the SPIRX functions. 
 */
void LMH_SPIRX_Init(SPI_HandleTypeDef* spiHandle)
{   
	// TODO: Fix this so it only checks once (not a while loop)
	// if (LMH_SPIRX_IntCfg(spiHandle, DWM1001_INTR_SPI_DATA_READY) == LMH_ERR)
	while (LMH_SPIRX_IntCfg(spiHandle, DWM1001_INTR_LOC_READY) == LMH_ERR)
	{
		PRINT_UWB("\n\t DW: LMH_SPIRX_IntCfg() failed. \t >>>>>> Error <<<<<<\n");
		HAL_Delay(100);
		//return;
	}
	
	LMH_SPIRX_SetTimeout(LMH_SPIRX_TIMEOUT_DEFAULT);
	LMH_SPIRX_SetWait(HAL_SPI_WAIT_PERIOD);
	LMH_SPIRX_SetToIdle(spiHandle);
	
	PRINT_UWB("\n\tLMH: LMH_SPIRX_Init done.\n");     
}

/**
 * @brief : de-initialises the SPIRX functions. 
 */
void LMH_SPIRX_DeInit(void)
{
}

/**
 * @brief : send the command to configure DWM1001 to enable Data Ready pin
 *
 * @return Error code
 */
static int LMH_SPIRX_IntCfg(SPI_HandleTypeDef* spiHandle, uint8_t value)
{        
	uint8_t tx_data[DWM1001_TLV_MAX_SIZE];
	uint8_t rx_data[DWM1001_TLV_MAX_SIZE];
	uint16_t rx_len;   
	uint16_t tx_len = 0;
	tx_data[tx_len++] = DWM1001_TLV_TYPE_CMD_INT_CFG;
	tx_data[tx_len++] = 1;
	tx_data[tx_len++] = (uint8_t)value;    
	
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
	HAL_StatusTypeDef txResult = HAL_SPI_Transmit(spiHandle, tx_data, tx_len, DWM_SPI_TIMEOUT);
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
	
	if (txResult != HAL_OK)
	{
		PRINT_UWB("\tDW: ERROR: module failed to set config\n"); 
		return LMH_ERR;
	}
	
	return LMH_SPIRX_WaitForRx(spiHandle, rx_data, &rx_len, 3);
}

/**
 * @brief : sets the DWM1001 module SPIRX functions into idle mode. 
 */
int LMH_SPIRX_SetToIdle(SPI_HandleTypeDef* spiHandle)
{
	uint8_t dummy = 0xff;
	uint16_t length;
	int i = 3;
	uint8_t rxBuff[16] = { 0 };
	
	// Sending three 0xFF dummy bytes, each in a single transmission, sets the state to IDLE
	PRINT_UWB("\tDW: Reseting DWM1001 to SPI:IDLE \n"); 
	
	while (i-- > 0)
	{
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
		HAL_StatusTypeDef txResult = HAL_SPI_Transmit(spiHandle, &dummy, 1, lmh_spirx_timeout);
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
		HAL_Delay(lmh_spirx_wait);
		if (txResult != HAL_OK)
		{
			PRINT_UWB("\tDW: ERROR: module failed to be set to idle\n"); 
			return LMH_ERR;
		}
	}
	
	// The response data will become all dummy bytes of value 0xFF to indicate it is in IDLE
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
	HAL_StatusTypeDef rxResult = HAL_SPI_Receive(spiHandle, rxBuff, 16, lmh_spirx_timeout);
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
	
	if (rxBuff[0] != 0xFF)
	{
		PRINT_UWB("\tDW: ERROR: module failed to be set to idle\n"); 
		return LMH_ERR;
	}
	
//	int rxResult = LMH_SPIRX_WaitForRx(spiHandle, rxBuff, &length, 3);
//	if (rxResult != LMH_OK ||
//		(rxBuff[0] != 3 || rxBuff[1] != 1 || rxBuff[2] != 0xFF || rxBuff[3] != 0xFF || rxBuff[4] != 0xFF))
//	{
//		PRINT_UWB("\tDW: ERROR: module failed to be set to idle\n"); 
//		return LMH_ERR;
//	}
	
	return LMH_OK;
}

/**
 * @brief : Set the SPIRX time out period. 
 *
 * @param [in] timeout, SPIRX time out period in ms
 */
void LMH_SPIRX_SetTimeout(int timeout)
{
	lmh_spirx_timeout = timeout;
}

/**
 * @brief : Set the SPIRX wait period between each poll of SIZE. 
 *
 * @param [in] wait, SPIRX wait period in ms
 */
void LMH_SPIRX_SetWait(int wait)
{
	lmh_spirx_wait = wait;
}

/**
 * @brief : wait length=exp_length for max time=lmh_spirx_wait
 *          needs LMH_SPIRX_Init() at initialization 
 *
 * @param [out] data,       pointer to received data 
 * @param [out] length,     pointer to received data length 
 * @param [in] exp_length,  expected data length
 *
 * @return Error code
 */
int LMH_SPIRX_WaitForRx(SPI_HandleTypeDef* spiHandle, uint8_t* data, uint16_t* length, uint16_t exp_length)
{
	uint16_t len_header = LMH_SPIRX_HEADER_LENGTH;
	uint16_t rxLen = 0;
	uint8_t sizenum[LMH_SPIRX_HEADER_LENGTH];
	uint8_t dummy[LMH_SPIRX_HEADER_LENGTH] = { 0 };
	int timeout = lmh_spirx_timeout;

	if(exp_length < DWM1001_TLV_RET_VAL_MIN_SIZE)
	{
		PRINT_UWB("\tDW >>>Error<<<: exp_length must be >= 3\n");    
		return LMH_ERR;
	}
    
	//PRINT_UWB("\tDW: Rx reading header: \n");
	HAL_StatusTypeDef result = HAL_OK;
	memset(sizenum, 0, LMH_SPIRX_HEADER_LENGTH);
	
	while (result == HAL_OK && sizenum[LMH_SPIRX_SIZE_OFFSET] == 0 && timeout >= 0)
	{
		HAL_Delay(lmh_spirx_wait);
		timeout -= lmh_spirx_wait;
		
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
		result = HAL_SPI_Receive(spiHandle, sizenum, len_header, lmh_spirx_timeout);
		//result = HAL_SPI_TransmitReceive(spiHandle, dummy, sizenum, len_header, lmh_spirx_timeout);
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
	}
    
	if (timeout < 0)
	{
		PRINT_UWB("\tDW: Read SIZE timed out after %d ms... >>>>>> TIMED OUT <<<<<< \n", lmh_spirx_timeout);  
		return LMH_ERR;
	}
	else if (sizenum[LMH_SPIRX_SIZE_OFFSET] == 0 || sizenum[LMH_SPIRX_SIZE_OFFSET] == 0Xff)
	{
		PRINT_UWB("\tDW: Error: Invalid read SIZE %d\n", sizenum[LMH_SPIRX_SIZE_OFFSET]);  
		return LMH_ERR;
	}
	
	//PRINT_UWB("\tDW: Receive TLV message: \n");
	*length = 0;
   
#if LMH_SPIRX_HEADER_LENGTH == 2
	uint8_t i;
	for(i = 0; i < sizenum[LMH_SPIRX_NUM_OFFSET]; i++)
#endif
	{
		HAL_Delay(lmh_spirx_wait);
		timeout -= lmh_spirx_wait;      
		
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_RESET); /**< Put chip select line low */
		result = HAL_SPI_Receive(spiHandle, data, sizenum[LMH_SPIRX_SIZE_OFFSET], lmh_spirx_timeout);
		//result = HAL_SPI_TransmitReceive(spiHandle, dummy, data, sizenum[LMH_SPIRX_SIZE_OFFSET], lmh_spirx_timeout);
		HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line high */
		
		if (result == HAL_TIMEOUT)
		{
			PRINT_UWB("\tDW: Read SIZE timed out after %d ms... >>>>>> TIMED OUT <<<<<< \n", lmh_spirx_timeout);  
			return LMH_ERR;
		}
		
		*length += sizenum[LMH_SPIRX_SIZE_OFFSET];
	}
	
	HAL_Delay(lmh_spirx_wait);
	//PRINT_UWB("\tDW: Wait %d ms...\n", lmh_spirx_wait); 
   
	if(LMH_CheckRetVal(data) != LMH_OK)
	{
		return LMH_ERR;
	}
	
	if((*length != exp_length) && (exp_length != DWM1001_TLV_MAX_SIZE))
	{
		PRINT_UWB("\tDW >>>ERROR<<<: Expecting %d bytes, received %d bytes, in %d ms\n", exp_length, *length, lmh_spirx_timeout-timeout);
		return LMH_ERR;
	}
   
	PRINT_UWB("\tDW: Received %d bytes, in %d ms \t OK\n", *length, lmh_spirx_timeout - timeout);
	return LMH_OK;
}

void PRINT_UWB(const char* format, ...)
{
#if defined(DEBUG_PRINT) && defined(LMH_DEBUG_PRINT)
	va_list argptr;
	va_start(argptr, format);
	vfprintf(stderr, format, argptr);
	va_end(argptr);
#endif // DEBUG_PRINT
}










