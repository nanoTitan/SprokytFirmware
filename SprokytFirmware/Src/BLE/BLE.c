#include "BLE.h"
#include "cube_hal.h"
#include "hal_types.h"
#include "string.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "sm.h"
#include "debug.h"
#include "math.h"
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
#include "sensor_service.h"
#include <stdlib.h>


#define COPY_UUID_128_V2(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

// LED service
#define COPY_LED_SERVICE_UUID(uuid_struct)  COPY_UUID_128_V2(uuid_struct,0x0b,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LED_UUID(uuid_struct)          COPY_UUID_128_V2(uuid_struct,0x0c,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	
// Input Service
#define COPY_INPUT_SERVICE_UUID(uuid_struct)		COPY_UUID_128_V2(uuid_struct,0x0d,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xd5,0x2b)
#define COPY_INPUT_CHAR_UUID(uuid_struct)			COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xd5,0x2b)
#define COPY_INSTRUCTION_CHAR_UUID(uuid_struct)     COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xe5,0x2b)

/* Private variables ---------------------------------------------------------*/
static tBleStatus AddLEDService(void);
static tBleStatus AddInputService(void);
static tBleStatus AddInstructionService(void);
static void User_Process();
static void setBLEConnectable(void);
static uint8_t SERVER_BDADDR[] = { 0x12, 0x34, 0x00, 0xE1, 0x80, 0x03 };	
static uint8_t bdaddr[BDADDR_SIZE];
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */	
volatile uint8_t do_set_connectable = TRUE;
volatile uint16_t service_connection_handle = 0;
volatile uint8_t is_notification_enabled = FALSE;
volatile uint8_t connected = 0;
uint16_t ledServHandle = 0;
uint16_t ledButtonCharHandle = 0;
uint16_t inputServHandle = 0;
uint16_t inputButtonCharHandle = 0;
uint16_t instructionButtonCharHandle = 0;

/* Private function prototypes -----------------------------------------------*/
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
  
	status = Add_Acc_Service();
  
	if (status == BLE_STATUS_SUCCESS)
		PRINTF("Acc service added successfully.\n");
	else
		PRINTF("Error while adding Acc service.\n");
  
	status = Add_Environmental_Sensor_Service();
  
	if (status == BLE_STATUS_SUCCESS)
		PRINTF("Environmental Sensor service added successfully.\n");
	else
		PRINTF("Error while adding Environmental Sensor service.\n");
	
	status = AddLEDService();
	if (status == BLE_STATUS_SUCCESS)
		PRINTF("LED service added successfully.\n");
	else
		PRINTF("Error while adding LED service.\n");
	
	status = AddInputService();
	if (status == BLE_STATUS_SUCCESS)
		PRINTF("Input service added successfully.\n");
	else
		PRINTF("Error while adding Input service.\n");

		  /* Set output power level */
	status = aci_hal_set_tx_power_level(1, 4);
	if (status != BLE_STATUS_SUCCESS)
	{
		PRINTF("Error setting BLE power level.\n");
	}
	
	return BLE_STATUS_SUCCESS;
}

void UpdateBLE()
{
	HCI_Process();
	User_Process();
}

BOOL IsBleConnected()
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
 * @brief  Add LED button service using a vendor specific profile.
 * @param  None
 * @retval Status
 */
tBleStatus AddLEDService(void)
{
	tBleStatus ret;
	uint8_t uuid[16];
  
	/* copy "LED service UUID" defined above to 'uuid' local variable */
	COPY_LED_SERVICE_UUID(uuid);
	
	ret = aci_gatt_add_serv(UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		7,
		&ledServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
	/* copy "LED button characteristic UUID" defined above to 'uuid' local variable */  
	COPY_LED_UUID(uuid);
	
	ret =  aci_gatt_add_char(ledServHandle,
		UUID_TYPE_128,
		uuid,
		4,
		CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&ledButtonCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;  
  
	PRINTF("Service LED BUTTON added. Handle 0x%04X, LED button Charac handle: 0x%04X\n", ledServHandle, ledButtonCharHandle);	
	return BLE_STATUS_SUCCESS; 
  
fail:
	PRINTF("Error while adding LED service.\n");
	return BLE_STATUS_ERROR;
}

/*
 * @brief  Add Input button service using a vendor specific profile.
 * @param  None
 * @retval Status
 */
tBleStatus AddInputService(void)
{
	tBleStatus ret;
	uint8_t uuid[16];
  
	/* copy "Input service UUID" defined above to 'uuid' local variable */
	COPY_INPUT_SERVICE_UUID(uuid);
	
	ret = aci_gatt_add_serv(UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		7,
		&inputServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
	/* copy "INPUT button characteristic UUID" defined above to 'uuid' local variable */  
	COPY_INPUT_CHAR_UUID(uuid);
	
	ret =  aci_gatt_add_char(inputServHandle,
		UUID_TYPE_128,
		uuid,
		4,
		CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&inputButtonCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;  
  
	PRINTF("Service Input BUTTON added. Handle 0x%04X, Input button Charac handle: 0x%04X\n", inputServHandle, inputButtonCharHandle);	
	
	/* copy "Instructionn characteristic UUID" defined above to 'uuid' local variable */  
	COPY_INSTRUCTION_CHAR_UUID(uuid);
	
	ret =  aci_gatt_add_char(inputServHandle,
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
  
	PRINTF("Service Instruction added. Handle 0x%04X, Input button Charac handle: 0x%04X\n", inputServHandle, instructionButtonCharHandle);
	return BLE_STATUS_SUCCESS; 
  
fail:
	PRINTF("Error while adding INPUT service.\n");
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
	if (handle == GetAccServHandle() + 1) {
		Acc_Update();
	}  
	else if (handle == GetTempCharHandle() + 1) {
		int16_t data;
		data = 270 + ((uint64_t)rand() * 15) / RAND_MAX; //sensor emulation        
		Acc_Update(); //FIXME: to overcome issue on Android App
		                                    // If the user button is not pressed within
		                                    // a short time after the connection,
		                                    // a pop-up reports a "No valid characteristics found" error.
		Temp_Update(data);
	}
	else if(handle == GetPressCharHandle() + 1){
		int32_t data;
		struct timer t;  
		Timer_Set(&t, CLOCK_SECOND/10);
		data = 100000 + ((uint64_t)rand()*1000)/RAND_MAX;
		Press_Update(data);
	}
	else if(handle == GetHumidtyCharHandle() + 1){
		uint16_t data;
    
		data = 450 + ((uint64_t)rand()*100)/RAND_MAX;
    
		Humidity_Update(data);
	}
  
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
	if (handle == inputButtonCharHandle + 1)
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