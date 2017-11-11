#include "serial_print.h"
#include "error.h"
#include "debug.h"
#include "stm32f4xx_hal_conf.h"
#include <errno.h>
#include  <sys/stat.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef huart2;
uint8_t bufftx[10] = "Hello!\r\n";

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


void SerialPrint_Init()
{
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
}

void SerialPrint_Print(char* pBuff, uint16_t size)
{
	HAL_UART_Transmit(&huart2, bufftx, 10, 100);
	//printf(pBuff);
}

/* USART1 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
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
	HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);

	// return # of bytes written - as best we can tell
	return (status == HAL_OK ? len : 0);
}

int _close(int file)
{
	return -1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	if ((file == STDOUT_FILENO) ||
	    (file == STDIN_FILENO) ||
	    (file == STDERR_FILENO))
	{
		return 1;
	}

	errno = EBADF;
	return 0;
}