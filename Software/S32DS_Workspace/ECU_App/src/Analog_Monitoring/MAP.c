/*
 * MAP.c
 *
 * Manifold Absolute Pressure Monitoring
 *
 *  Created on: 2023年3月18日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "Application_Define.h"
#include "MAP.h"
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
extern uint8_t teeth_counter;

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
static uint8_t map_counter = 0;
uint16_t map_data_buffer[MAP_BUFFER_SIZE];
static uint8_t map_buffer_counter = 0;

/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
		 
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void map_init(void)
{
	for(map_counter = 0; map_counter < MAP_BUFFER_SIZE; map_counter++)
	{
		map_data_buffer[map_counter] = 0;
	}
}


/**
  * @brief
  * @param void
  * @retval	void
  * @note
  */
void map_monitoring(void)
{
	if((teeth_counter >= MAP_TOOTH_START) && (teeth_counter <= (MAP_TOOTH_END)))
	{
		if(map_buffer_counter == (teeth_counter - MAP_TOOTH_START))
		{
			map_data_buffer[map_buffer_counter] = adc_read_channel_mv(MAP_ADC_CHANNEL);

			map_buffer_counter++;
		}

		if(teeth_counter == MAP_TOOTH_END)
		{
			map_buffer_counter = 0;
		}
	}
}


