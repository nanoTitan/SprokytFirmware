#include "uwb.h"
#include "dwm_constants.h"
#include "constants.h"
#include "lmh.h"
#include "stm32f4xx_hal.h"
#include "error.h"
#include "control_manager.h"

static SPI_HandleTypeDef m_uwbSpi;
static dwm_cfg_tag_t m_cfg_tag;
static dwm_cfg_t m_cfg_node;
static dwm_loc_data_t m_loc;
static dwm_status_t m_status;
static dwm_pos_t m_pos;
static uint32_t m_lastTime = 0;
static bool m_isReadingPos = false;
static bool m_locationDataReady = false;

void UWB_Position_Ready_Callback();

int UWB_Init()
{
	m_isReadingPos = false;
		
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	m_uwbSpi.Instance = UWB_SPIx;
	m_uwbSpi.Init.Mode = SPI_MODE_MASTER;
	m_uwbSpi.Init.Direction = SPI_DIRECTION_2LINES;
	m_uwbSpi.Init.DataSize = SPI_DATASIZE_8BIT;
	m_uwbSpi.Init.CLKPolarity = SPI_POLARITY_LOW;
	m_uwbSpi.Init.CLKPhase = SPI_PHASE_1EDGE;
	m_uwbSpi.Init.NSS = SPI_NSS_SOFT;
	m_uwbSpi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;	// SPI_BAUDRATEPRESCALER_256;	//SPI_BAUDRATEPRESCALER_32
	m_uwbSpi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	m_uwbSpi.Init.TIMode = SPI_TIMODE_DISABLE;
	m_uwbSpi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	if (HAL_SPI_Init(&m_uwbSpi) != HAL_OK)
	{
		Error_Handler();
	}
	
	HAL_GPIO_WritePin(UWB_SPIx_NSS_PORT, UWB_SPIx_NSS_PIN, GPIO_PIN_SET); /**< Put chip select line High */
	
	// Wait 2 seconds for node to reset
	HAL_Delay(1000);
	LMH_Init(&m_uwbSpi);
	
	PRINTF("\tDW: Setting module to tag.\n");
	m_cfg_tag.low_power_en = 0; 
	m_cfg_tag.meas_mode = DWM_MEAS_MODE_TWR;
	m_cfg_tag.loc_engine_en = 1;
	m_cfg_tag.common.led_en = 1;
	m_cfg_tag.common.ble_en = 1;
	m_cfg_tag.common.uwb_mode = DWM_UWB_MODE_ACTIVE;
	m_cfg_tag.common.fw_update_en = 0;
	
	dwm_cfg_tag_set(&m_uwbSpi, &m_cfg_tag);
    
	// Wait 2 seconds for node to reset
	HAL_Delay(2000);  
	dwm_cfg_get(&m_uwbSpi, &m_cfg_node);
	
	if ((m_cfg_tag.low_power_en      != m_cfg_node.low_power_en) 
	   || (m_cfg_tag.meas_mode           != m_cfg_node.meas_mode) 
	   || (m_cfg_tag.loc_engine_en       != m_cfg_node.loc_engine_en) 
	   || (m_cfg_tag.common.led_en       != m_cfg_node.common.led_en) 
	   || (m_cfg_tag.common.ble_en       != m_cfg_node.common.ble_en) 
	   || (m_cfg_tag.common.uwb_mode     != m_cfg_node.common.uwb_mode) 
	   || (m_cfg_tag.common.fw_update_en != m_cfg_node.common.fw_update_en))
	{
		PRINTF("low_power_en        cfg_tag=%d : cfg_node=%d\n", m_cfg_tag.low_power_en, m_cfg_node.low_power_en); 
		PRINTF("meas_mode           cfg_tag=%d : cfg_node=%d\n", m_cfg_tag.meas_mode, m_cfg_node.meas_mode); 
		PRINTF("loc_engine_en       cfg_tag=%d : cfg_node=%d\n", m_cfg_tag.loc_engine_en, m_cfg_node.loc_engine_en); 
		PRINTF("common.led_en       cfg_tag=%d : cfg_node=%d\n", m_cfg_tag.common.led_en, m_cfg_node.common.led_en); 
		PRINTF("common.ble_en       cfg_tag=%d : cfg_node=%d\n", m_cfg_tag.common.ble_en, m_cfg_node.common.ble_en); 
		PRINTF("common.uwb_mode     cfg_tag=%d : cfg_node=%d\n", m_cfg_tag.common.uwb_mode, m_cfg_node.common.uwb_mode); 
		PRINTF("common.fw_update_en cfg_tag=%d : cfg_node=%d\n", m_cfg_tag.common.fw_update_en, m_cfg_node.common.fw_update_en);  
		PRINTF("\n\tDW: Configuration failed.\n\n");
	}
	else
	{
		PRINTF("\n\tDW: Configuration succeeded.\n\n");
	}
	
	m_loc.p_pos = &m_pos;
	
	return UWB_STATUS_SUCCESS;
}

void UWB_Update()
{
	int result = RV_OK;
#if INTERFACE_NUMBER == 3
	if (!m_locationDataReady)
		return;
	
	m_status.uwbmac_joined = false;
	m_status.loc_data = false;
	
	result = dwm_status_get(&m_uwbSpi, &m_status);
	if (result != RV_OK)
		return;
	
	if (!m_status.loc_data)
		return;
	
	result = dwm_loc_get(&m_uwbSpi, &m_loc);
#else
	uint32_t currTime = HAL_GetTick();
	if (currTime - m_lastTime < UWB_UPDATE_TIME)
		return;
	
	m_lastTime = currTime;	// Reset retry time
	result = dwm_loc_get(&m_uwbSpi, &m_loc);
#endif	// INTERFACE_NUMBER
	
	if (result != RV_OK)
		return;
	
	m_isReadingPos = true;
		
	// Print tag position
	//PRINTF("\t[%d,%d,%d,%u]\n", m_loc.p_pos->x, m_loc.p_pos->y, m_loc.p_pos->z, m_loc.p_pos->qf);

	/*
	// Print anchor positions
	for (i = 0; i < m_loc.anchors.dist.cnt; ++i) 
	{
		PRINTF("\t%u)", i);
		PRINTF("0x%llx", m_loc.anchors.dist.addr[i]);
		if (i < m_loc.anchors.an_pos.cnt) 
		{
			PRINTF("[%d,%d,%d,%u]", m_loc.anchors.an_pos.pos[i].x, m_loc.anchors.an_pos.pos[i].y, m_loc.anchors.an_pos.pos[i].z, m_loc.anchors.an_pos.pos[i].qf);
		}
			
		PRINTF("=%u,%u\n", (uint32_t)m_loc.anchors.dist.dist[i], m_loc.anchors.dist.qf[i]);
	}
	*/
}

void UWB_Position_Ready_Callback()
{
	m_locationDataReady = true;
}

bool UWB_IsReady()
{
	return m_isReadingPos;
}

void UWB_GetPosition(float* out_x, float* out_y, float* out_z)
{
	*out_x = m_loc.p_pos->x;
	*out_y = m_loc.p_pos->y;
	*out_z = m_loc.p_pos->z;
}

bool UWB_GetModuleData(float* moduleData, uint8_t buffSz, uint8_t* out_size)
{
	int indx = 0;
	if (!m_isReadingPos)
		return false;
	
	*out_size = (1 + m_loc.anchors.dist.cnt) * 3 * 4;
	if (buffSz < *out_size)
		return false;
	
	moduleData[indx++] = m_loc.p_pos->x;
	moduleData[indx++] = m_loc.p_pos->y;
	moduleData[indx++] = m_loc.p_pos->z;
	
	for (int i = 0; i < m_loc.anchors.dist.cnt; ++i) 
	{
		if (i < m_loc.anchors.an_pos.cnt) 
		{
			moduleData[indx++] = m_loc.anchors.an_pos.pos[i].x;
			moduleData[indx++] = m_loc.anchors.an_pos.pos[i].y;
			moduleData[indx++] = m_loc.anchors.an_pos.pos[i].z;
		}
	}
	
	return true;
}