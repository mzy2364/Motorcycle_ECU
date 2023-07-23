/*
 * ioa.c
 *
 *  Created on: 2023年6月19日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "ioa.h"
#include "bsp_gpio.h"

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
uint8_t di_ign_on = 0;
uint8_t di_hs_01 = 0;
uint8_t di_ls_01 = 0;
uint8_t di_ls_02 = 0;
/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
uint8_t local_di_filter[DI_CH_NUM] = {0};
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void ioa_di_read(void);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void ioa_task(void)
{
	ioa_di_read();
}

/**
  * @brief read Digital IO
  * @param void
  * @retval	void
  * @note
  */
static void ioa_di_read(void)
{
	if(gpio_read(DI_IGN))
	{
		if(local_di_filter[0] < DI_FILTER_TIME)
		{
			local_di_filter[0]++;
		}
		else
		{
			di_ign_on = 1;
		}
	}
	else
	{
		if(local_di_filter[0] > 0)
		{
			local_di_filter[0]--;
		}
		else
		{
			di_ign_on = 0;
		}
	}

	if(gpio_read(DI_HS_01))
	{
		if(local_di_filter[1] < DI_FILTER_TIME)
		{
			local_di_filter[1]++;
		}
		else
		{
			di_hs_01 = 1;
		}
	}
	else
	{
		if(local_di_filter[1] > 0)
		{
			local_di_filter[1]--;
		}
		else
		{
			di_hs_01 = 0;
		}
	}

	if(gpio_read(DI_LS_01) == 0)
	{
		if(local_di_filter[2] < DI_FILTER_TIME)
		{
			local_di_filter[2]++;
		}
		else
		{
			di_ls_01 = 1;
		}
	}
	else
	{
		if(local_di_filter[2] > 0)
		{
			local_di_filter[2]--;
		}
		else
		{
			di_ls_01 = 0;
		}
	}

	if(gpio_read(DI_LS_02) == 0)
	{
		if(local_di_filter[3] < DI_FILTER_TIME)
		{
			local_di_filter[3]++;
		}
		else
		{
			di_ls_02 = 1;
		}
	}
	else
	{
		if(local_di_filter[3] > 0)
		{
			local_di_filter[3]--;
		}
		else
		{
			di_ls_02 = 0;
		}
	}
}

