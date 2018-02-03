#ifndef _DWM_CONSTANTS_H_
#define _DWM_CONSTANTS_H_

#define UWB_STATUS_SUCCESS	0
#define UWB_STATUS_ERROR	1

#define HAL_SPI_MAX_LENGTH           255
#define HAL_SPI_MAX_PRINT_LENGTH     HAL_SPI_MAX_LENGTH*3
#define HAL_SPI_WAIT_PERIOD          200

#define HAL_SPI_DEV0 0
#define HAL_SPI_DEV1 1

#define DWM_SPI_TIMEOUT	5000

#define INTERFACE_NUMBER					1		// DW Interface mode: 0 - USART, 1 - SPI, 2 - SPI Interrupt Ready
#if INTERFACE_NUMBER == 0
#include "hal_uart.h"
#define HAL_IF_Tx          HAL_UART_Tx
#define HAL_IF_Rx          HAL_UART_Rx
#define HAL_IF_STR         "HAL_UART"
#else
#define HAL_IF_Tx          HAL_SPI_Tx
#define HAL_IF_Rx          HAL_SPI_Rx
#if INTERFACE_NUMBER == 1
#define HAL_IF_STR         "HAL_SPI"
#endif   
#if INTERFACE_NUMBER == 2
#define HAL_IF_STR         "HAL_SPI_DRDY"
#endif   
#endif 


#endif // _DWM_CONSTANTS_H_