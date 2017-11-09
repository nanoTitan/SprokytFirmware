#include "serial_print.h"
#include "error.h"
#include "debug.h"
#include "stm32f4xx_hal_conf.h"
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef huart6;


/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);


void SerialPrint_Init()
{
	MX_GPIO_Init();
	MX_USART6_UART_Init();
	
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
}

void SerialPrint_Print(char* pBuff, uint16_t size)
{
	HAL_UART_Transmit(&huart6, (uint8_t*)pBuff, size, 1000);
	//printf(pBuff);
}

/* USART1 init function */
static void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
}

int _write(int file, char *data, int len)
{
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
	{
		errno = EBADF;
		return -1;
	}

	// arbitrary timeout 1000
	HAL_StatusTypeDef status =
	HAL_UART_Transmit(&huart6, (uint8_t*)data, len, 1000);

	// return # of bytes written - as best we can tell
	return (status == HAL_OK ? len : 0);
}