#ifndef _IMU_CALIBRATION_H_
#define _IMU_CALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
	
/* Public types --------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#if (defined (USE_STM32F4XX_NUCLEO))
#define MOTION_FX_FLASH_SECTOR        FLASH_SECTOR_6
#define MOTION_FX_FLASH_SECTOR_SIZE   (SIZE_FLASH_SECTOR_6)
#define SIZE_FLASH_SECTOR_6          ((uint32_t)0x00020000)
#endif

#if (defined (USE_STM32L4XX_NUCLEO))
#define MOTION_FX_FLASH_SECTOR_SIZE   ((uint32_t)0x00020000)
#endif
#define MOTION_FX_FLASH_ITEM_SIZE     8

/* Exported defines ----------------------------------------------------------*/
#if (defined (USE_STM32F4XX_NUCLEO))
#define MOTION_FX_FLASH_ADD                  ((uint32_t)0x08040000)
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
#define MOTION_FX_FLASH_ADD                  ((uint32_t)0x080DF800) /* page 447 */
#endif

/* Imported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
	void SaveCalibrationToMemory(uint16_t dataSize, uint32_t *data);
	void RecallCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
	void ResetCalibrationInMemory(void);

#ifdef __cplusplus
}
#endif

#endif /* _IMU_CALIBRATION_H_ */

