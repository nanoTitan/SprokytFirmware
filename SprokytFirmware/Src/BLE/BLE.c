#include "BLE.h"
#include "cube_hal.h"
#include "hal_types.h"
#include "string.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "sm.h"
#include "debug.h"
#include "math_ext.h"
#include "constants.h"
#include "control_manager.h"
#include "hci_const.h"
#include "hci_le.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_aci_const.h"  
#include "bluenrg_hal_aci.h"
#include "bluenrg_utils.h"
#include <stdlib.h>


#define COPY_UUID_128_V2(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
} while(0)

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define STORE_LE_32(buf, val)    ( (buf[0] =  (int)val & 0xFF    ) , \
                                   (buf[1] =  (int)val>>8 & 0xFF ) , \
								   (buf[2] =  (int)val>>16 & 0xFF ) , \
								   (buf[3] =  (int)val>>24 & 0xFF ) )


// Control Service
#define COPY_CONTROL_SERVICE_UUID(uuid_struct)		COPY_UUID_128_V2(uuid_struct,0x0d,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xd5,0x2b)
#define COPY_IMU_SERVICE_UUID(uuid_struct)			COPY_UUID_128_V2(uuid_struct,0x0b,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1c)
//#define COPY_DISTANCE_SERVICE_UUID(uuid_struct)		COPY_UUID_128_V2(uuid_struct,0x0b,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x2d)

// Characteristics
#define COPY_CONTROL_CHAR_UUID(uuid_struct)			COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xd5,0x2b)
#define COPY_INSTRUCTION_CHAR_UUID(uuid_struct)     COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xe5,0x2b)
#define COPY_IMU_CHAR_UUID(uuid_struct)				COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_POSITION_CHAR_UUID(uuid_struct)		COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x2c)
#define COPY_TAG_CHAR_UUID(uuid_struct)				COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x3e)

/* Private variables ---------------------------------------------------------*/
static uint8_t SERVER_BDADDR[] = { 0x12, 0x34, 0x00, 0xE1, 0x80, 0x03 };	
static uint8_t bdaddr[BDADDR_SIZE];
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */	
volatile uint8_t do_set_connectable = TRUE;
volatile uint16_t service_connection_handle = 0;
volatile uint8_t is_notification_enabled = FALSE;
volatile uint8_t connected = 0;
uint16_t controlServHandle = 0;
uint16_t imuServHandle = 0;
//uint16_t distServHandle = 0;
uint16_t controlButtonCharHandle = 0;
uint16_t instructionButtonCharHandle = 0;
uint16_t imuCharHandle = 0;
uint16_t posCharHandle = 0;
uint16_t tagCharHandle = 0;

/* Private function prototypes -----------------------------------------------*/
static tBleStatus AddControlService(void);
static tBleStatus AddInstructionService(void);
static void User_Process();
static void setBLEConnectable(void);
uint8_t GetExpansionBoard();
void Read_Request_CB(uint16_t handle);
void GAPDisconnectionCompleteCB(void);
void GAPConnectionCompleteCB(uint8_t addr[6], uint16_t handle);
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data);

int InitBLE()
{
	uint8_t  hwVersion = 0;
	uint16_t fwVersion = 0;
	uint16_t service_handle = 0;
	uint16_t dev_name_char_handle = 0;
	uint16_t appearance_char_handle = 0;
	const char *name = "BlueNRG";
	
	/* Initialize the BlueNRG SPI driver */
	BNRG_SPI_Init();
	
	/* Initialize the BlueNRG HCI */
	HCI_Init();
	
	HAL_Delay(10);
	
	/* Reset BlueNRG hardware */
	BlueNRG_RST();
	
	HAL_Delay(10);
    
	/* get the BlueNRG HW and FW versions */
	uint8_t status = getBlueNRGVersion(&hwVersion, &fwVersion);
	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("Failed to get the BLE version.\n");
		return status;
	}
	
	/* 
	* Reset BlueNRG again otherwise we won't
	* be able to change its MAC address.
	* aci_hal_write_config_data() must be the first
	* command after reset otherwise it will fail.
	*/
	BlueNRG_RST();
  
	PRINTF("HWver %d, FWver %d", hwVersion, fwVersion);
  
	if (hwVersion > 0x30)
	{ /* X-NUCLEO-IDB05A1 expansion board is used */
		bnrg_expansion_board = IDB05A1; 
		/*
		 * Change the MAC address to avoid issues with Android cache:
		 * if different boards have the same MAC address, Android
		 * applications unless you restart Bluetooth on tablet/phone
		 */
		SERVER_BDADDR[5] = 0x02;
	}

	/* The Nucleo board must be configured as SERVER */	
	Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
  
	status = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
		CONFIG_DATA_PUBADDR_LEN,
		bdaddr);
	
	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("Setting BD_ADDR failed.\n");
		return status;
	}
  
	status = aci_gatt_init();    
	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("GATT_Init failed.\n");
		return status;
	}

	if (bnrg_expansion_board == IDB05A1)
	{
		status = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}
	else
	{
		status = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}

	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("GAP_Init failed.\n");
		return status;
	}

	status = aci_gatt_update_char_value(
		service_handle,
		dev_name_char_handle,
		0,
		strlen(name),
		(uint8_t *)name);

	if (status != BLE_STATUS_SUCCESS) 
	{
		PRINTF("aci_gatt_update_char_value failed.\n");            
		return status;
	}
  
	status = aci_gap_set_auth_requirement(
		MITM_PROTECTION_REQUIRED,
		OOB_AUTH_DATA_ABSENT,
		NULL,
		7,
		16,
		USE_FIXED_PIN_FOR_PAIRING,
		123456,
		BONDING);
	
	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("aci_gap_set_auth_requirement failed.\n");            
		return status;
	}
  
	PRINTF("SERVER: BLE Stack Initialized\n");
	
	status = AddControlService();
	if (status != BLE_STATUS_SUCCESS)
		return status;

	/* Set output power level */
	status = aci_hal_set_tx_power_level(1, 4);
	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("Error setting BLE power level.\n");
	}
	
	return BLE_STATUS_SUCCESS;
}

void BLE_Update()
{
	HCI_Process();
	User_Process();
}

BOOL BLE_IsConnected()
{
	return connected;
}

void setConnectable(void)
{  
	tBleStatus ret;
  
	const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};
  
	/* disable scan response */
	hci_le_set_scan_resp_data(0,NULL);
	PRINTF("General Discoverable Mode.\n");
  
	ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
									sizeof(local_name), local_name, 0, NULL, 0, 0);
	
	if (ret != BLE_STATUS_SUCCESS) 
	PRINTF("Error while setting discoverable mode (%d)\n", ret);    
}

/*
 * @brief  Add Input button service using a vendor specific profile.
 * @param  None
 * @retval Status
 */
tBleStatus AddControlService(void)
{
	tBleStatus ret;
	uint8_t uuid[16];
  
	/* copy "Input service UUID" defined above to 'uuid' local variable */
	COPY_CONTROL_SERVICE_UUID(uuid);
	
	ret = aci_gatt_add_serv(
		UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		7,
		&controlServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
	/* copy "INPUT button characteristic UUID" defined above to 'uuid' local variable */  
	COPY_CONTROL_CHAR_UUID(uuid);
	
	ret =  aci_gatt_add_char(
		controlServHandle,
		UUID_TYPE_128,
		uuid,
		4,
		CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&controlButtonCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;  
	
	PRINTF("Control characteristic added\n");	
	
	/* copy "Instructionn characteristic UUID" defined above to 'uuid' local variable */  
	COPY_INSTRUCTION_CHAR_UUID(uuid);
	
	ret =  aci_gatt_add_char(
		controlServHandle,
		UUID_TYPE_128,
		uuid,
		4,
		CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&instructionButtonCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail; 
	
	PRINTF("Instruction characteristic added\n");
	
	
	// IMU service
	/********************************************************************************************/
	COPY_IMU_SERVICE_UUID(uuid);
	
	ret = aci_gatt_add_serv(
		UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		7,
		&imuServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;
	
	// Tag - report the tag and each anchor's position
	/********************************************************************************************/	
	COPY_TAG_CHAR_UUID(uuid);
	
	ret =  aci_gatt_add_char(
		imuServHandle,
		UUID_TYPE_128,
		uuid,
		128,
		CHAR_PROP_NOTIFY | CHAR_PROP_READ | ATTR_PERMISSION_NONE,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&tagCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;
	
	// Position - report the position and orientation information
	/********************************************************************************************/	
	COPY_POSITION_CHAR_UUID(uuid);
	
	ret =  aci_gatt_add_char(
		imuServHandle,
		UUID_TYPE_128,
		uuid,
		32,
		CHAR_PROP_NOTIFY | CHAR_PROP_READ | ATTR_PERMISSION_NONE,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&posCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;  
	
	PRINTF("Distance characteristic added\n");	
	
	//********************************************************************************************/  
	
	PRINTF("Distance characteristic added\n");	
	
	// Done adding services
	PRINTF("BLE services added\n");
	return BLE_STATUS_SUCCESS; 
	
fail:
	PRINTF("Error while adding BLE services: %x.\n", ret);
	return BLE_STATUS_ERROR;
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  AxesRaw_t* p_axes
 * @retval None
 */
void User_Process()
{
	if (do_set_connectable) {
		setBLEConnectable();
		do_set_connectable = FALSE;
	}
}

/**
 * @brief  Puts the device in connectable mode.
 *         If you want to specify a UUID list in the advertising data, those data can
 *         be specified as a parameter in aci_gap_set_discoverable().
 *         For manufacture data, aci_gap_update_adv_data must be called.
 * @param  None 
 * @retval None
 */
/* Ex.:
 *
 *  tBleStatus ret;    
 *  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};    
 *  const uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID,0x34,0x12};    
 *  const uint8_t manuf_data[] = {4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x05, 0x02, 0x01};
 *  
 *  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
 *                                 8, local_name, 3, serviceUUIDList, 0, 0);    
 *  ret = aci_gap_update_adv_data(5, manuf_data);
 *
 */
void setBLEConnectable(void)
{  
	tBleStatus ret;
  
	const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'B', 'l', 'u', 'e', 'N', 'R', 'G' };
  
	/* disable scan response */
	hci_le_set_scan_resp_data(0, NULL);
	PRINTF("General Discoverable Mode.\n");
  
	ret = aci_gap_set_discoverable(
		ADV_IND,
		0,
		0,
		PUBLIC_ADDR,
		NO_WHITE_LIST_USE,
		sizeof(local_name),
		local_name,
		0,
		NULL,
		0,
		0);
	
	if (ret != BLE_STATUS_SUCCESS) 
		PRINTF("Error while setting discoverable mode (%d)\n", ret);    
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void GAPConnectionCompleteCB(uint8_t addr[6], uint16_t handle)
{  
	connected = TRUE;
	service_connection_handle = handle;
  
	PRINTF("Connected to device:");
	for (int i = 5; i > 0; i--) 
	{
		PRINTF("%02X-", addr[i]);
	}
	PRINTF("%02X\n", addr[0]);
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None 
 * @retval None
 */
void GAPDisconnectionCompleteCB(void)
{
	connected = FALSE;
	PRINTF("Disconnected\n");
	/* Make the device connectable again. */
	do_set_connectable = TRUE;
	is_notification_enabled = FALSE;
	
	ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
}

uint8_t GetExpansionBoard()
{
	return bnrg_expansion_board; 
}

/**
 * @brief  Read request callback.
 * @param  uint16_t Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{  
//	if (handle == imuCharHandle + 1) {		
//	} 
  
	//EXIT:
	if (service_connection_handle != 0)
		aci_gatt_allow_read(service_connection_handle);
}

/**
 * @brief  This function is called when an attribute characteristic is modified.
 * @param  Handle of the attribute
 * @param  Size of the modified attribute data
 * @param  Pointer to the modified attribute data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
	if (handle == controlButtonCharHandle + 1)
	{
		if (data_length < 3)
			return;
		
		uint8_t motorIndex = att_data[0];
		uint8_t value = att_data[1];
		uint8_t direction = att_data[2];
		
		//PRINTF("indx: %u, value: %u, dir: %u\n", motorIndex, value, direction);
		ControlMgr_setMotor(motorIndex, value, (direction_t)direction);
	}
	else if (handle == instructionButtonCharHandle + 1)
	{
		ControlMgr_parseInstruction(data_length, att_data);		
	}
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
	hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
	/* obtain event packet */
	hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
	if (hci_pckt->type != HCI_EVENT_PKT)
		return;
  
	switch (event_pckt->evt) {
    
	case EVT_DISCONN_COMPLETE:
		{
			GAPDisconnectionCompleteCB();
		}
		break;
    
	case EVT_LE_META_EVENT:
		{
			evt_le_meta_event *evt = (evt_le_meta_event*)event_pckt->data;
      
			switch (evt->subevent) {
			case EVT_LE_CONN_COMPLETE:
				{
					evt_le_connection_complete *cc = (evt_le_connection_complete*)evt->data;
					GAPConnectionCompleteCB(cc->peer_bdaddr, cc->handle);
				}
				break;
			}
		}
		break;
    
	case EVT_VENDOR:
		{
			evt_blue_aci *blue_evt = (evt_blue_aci*)event_pckt->data;
			switch (blue_evt->ecode) {

			// GATT Attribute modification
			case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:         
				{
					if (GetExpansionBoard() == IDB05A1) {
						evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
						Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data); 
					}
					else {
						evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
						Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data); 
					}                       
				}
				break;

			case EVT_BLUE_GATT_READ_PERMIT_REQ:
				{
					evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req*)blue_evt->data;                    
					Read_Request_CB(pr->attr_handle);                    
				}
				break;
			}
		}
		break;
	}    
}

/**
 * @brief  Update acceleration characteristic value.
 *
 * @param  Structure containing acceleration value in mg
 * @retval Status
 */
tBleStatus BLE_AngularPosUpdate(float yaw, float pitch)
{
	//static uint8_t buff[4] = {0};
	unsigned char const * const buff = (unsigned char const *)&yaw;
	
	if (!connected)
		return BLE_STATUS_ERROR;
    
//	STORE_LE_32(buff, yaw);
//	STORE_LE_32((buff + 4), pitch);
	
//	buff[3] =  (int)yaw & 0xFF;
//	buff[2] =  (int)yaw >> 8 & 0xFF;
//	buff[1] =  (int)yaw >> 16 & 0xFF;
//	buff[0] =  (int)yaw >> 24 & 0xFF;
	
//	buff[4] =  (int)pitch & 0xFF;
//	buff[5] =  (int)pitch >> 8 & 0xFF;
//	buff[6] =  (int)pitch >> 16 & 0xFF;
//	buff[7] =  (int)pitch >> 24 & 0xFF;
	
	tBleStatus status = aci_gatt_update_char_value(
		imuServHandle,
		imuCharHandle,
		0,
		sizeof(yaw),
		buff);
	
	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("Error while updating IMU characteristic: %x\n", status);
		return BLE_STATUS_ERROR ;
	}
	
	return BLE_STATUS_SUCCESS;	
}

tBleStatus BLE_PositionUpdate(const Transform_t* pTrans)
{
	unsigned char const * const buff = (unsigned char const *)pTrans;
	
	if (!connected)
		return BLE_STATUS_ERROR;
	
	tBleStatus status = aci_gatt_update_char_value(
		imuServHandle,
		posCharHandle,
		0,
		32,
		buff);
	
	if (status != BLE_STATUS_SUCCESS)
	{
		//PRINTF("BLE Error: 0x%x\n", status);
		return BLE_STATUS_ERROR;
	}
	
	return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Set the anchor and tag info from a 16 float array
 *
 * @param  tagInfo. An array of x,y,z positions for the tag, anchor1, anchor2, anchor3, and anchor4 respectively
 * @retval Status
 */
tBleStatus BLE_SetTagInfo(float* tagInfo, uint8_t size)
{
	unsigned char const * const buff = (unsigned char const *)tagInfo;
	
	if (!connected)
		return BLE_STATUS_ERROR;
	
	tBleStatus status = aci_gatt_update_char_value(
		imuServHandle,
		tagCharHandle,
		0,
		size,
		buff);
	
	if (status != BLE_STATUS_SUCCESS)
	{
		//PRINTF("BLE Error: 0x%x\n", status);
		return BLE_STATUS_ERROR;
	}
	
	return BLE_STATUS_SUCCESS;	
}