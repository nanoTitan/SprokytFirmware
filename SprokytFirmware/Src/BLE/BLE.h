#pragma once

#include "stm32_bluenrg_ble.h"
#include "hci.h"
#include <stdint.h>
	 
#define IDB04A1 0
#define IDB05A1 1
#define BDADDR_SIZE 6
#define JOYSTICK_COUNT 4

struct Transform;
typedef struct Transform Transform_t;


/** 
* @brief Structure containing acceleration value (in mg) of each axis.
*/
typedef struct 
{
	uint8_t x;
	uint8_t y;
} Joystick_t;


int InitBLE();
void BLE_Update();
BOOL BLE_IsConnected();
void setConnectable(void);
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
void GAP_DisconnectionComplete_CB(void);
void HCI_Event_CB(void *pckt);
tBleStatus BLE_AngularPosUpdate(float yaw, float pitch);
tBleStatus BLE_PositionUpdate(Transform_t* pTrans);

/** @addtogroup SPROKYT_BLE_Exported_Functions
*  @{
*/

