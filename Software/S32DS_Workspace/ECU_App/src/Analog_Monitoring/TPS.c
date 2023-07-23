/*
 * TPS.c
 *
 *	Throttle Position Monitoring.
 *
 *  Created on: 2023年3月29日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "TPS.h"
#include "bsp_adc.h"

/*******************************************************************************
* Defines                                                                
*******************************************************************************/

/*******************************************************************************
* Macros                                                                
*******************************************************************************/

/*******************************************************************************
* Global Constant definition                         
*******************************************************************************/

/*******************************************************************************
* Local Constant definition                         
*******************************************************************************/

/*******************************************************************************
* Global Variables definition                         
*******************************************************************************/
uint16_t throttle_position_adc = 0;
/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
static uint8_t tps_counter = 0;
static uint16_t tps_data_buffer[TPS_BUFFER_SIZE];
static uint8_t tps_buffer_counter = 0;
static uint8_t tps_collection_rate_counter = 0;
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void tps_adc_filter(void);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void tps_init(void)
{
	for(tps_counter = 0; tps_counter < TPS_BUFFER_SIZE; tps_counter++)
	{
		tps_data_buffer[tps_counter] = 0;
	}
}

/**
  * @brief 获取滤波后的节气门位置传感器的ADC值
  * @param void
  * @retval	void
  * @note
  */
uint16_t tps_adc_get(void)
{
	return throttle_position_adc;
}

/**
  * @brief
  * @param void
  * @retval	void
  * @note
  */
void tps_monitoring(void)
{
	if(tps_collection_rate_counter >= TPS_DATA_COLLECTION_RATE)
	{
		tps_data_buffer[tps_buffer_counter] = adc_read_channel(TPS_ADC_CHANNEL);

		tps_collection_rate_counter = 0;
		tps_buffer_counter++;
		if(tps_buffer_counter >= TPS_BUFFER_SIZE)
		{
			tps_adc_filter();
			tps_buffer_counter = 0;
		}

	}
	else
	{
		tps_collection_rate_counter++;
	}
}

/**
  * @brief 节气门位置传感器滤波
  * @param void
  * @retval	void
  * @note
  */
static void tps_adc_filter(void)
{
	uint32_t average = 0;
	for(tps_counter = 0; tps_counter < TPS_BUFFER_SIZE; tps_counter++)
	{
		average += tps_data_buffer[tps_counter];
	}

	average = average >> 4;		//average/16

	throttle_position_adc = average;
}
