// error.h

#ifndef _ERROR_H_
#define _ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "debug.h"

static volatile uint16_t gLastError;

static void Error_Handler(void)
{
	PRINTF("Error thrown in CError_Handler\r\n");
	while (1)
	{
	}
}

static void CError_Handler_1(uint16_t error)
{
/* Backup error number */
	gLastError = error;
  
	/* Infinite loop */
	while (1)
	{
	}
}
	
#ifdef __cplusplus
}
#endif

#endif // _ERROR_H_