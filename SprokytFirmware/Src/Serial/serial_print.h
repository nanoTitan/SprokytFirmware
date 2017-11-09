// Serial Communication using USART

#ifndef _SERIAL_PRINT_H_
#define _SERIAL_PRINT_H_
#include <stdint.h>

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

void SerialPrint_Init();
void SerialPrint_Print(char* pBuff, uint16_t size);


/* Exported functions ------------------------------------------------------- */

#endif /* _SERIAL_PRINT_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
