/*
 * gpio.c
 *
 *  Created on: 2023年3月2日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "bsp_gpio.h"
#include "gpio.h"

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

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/

/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
		 
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief GPIO初始化
  * @param void
  * @retval	void
  * @note
  */
void gpio_init(void)
{
	CONFIG_PIN_AS_GPIO(VALVE_PORT,VALVE_PIN,OUTPUT);
	OUTPUT_CLEAR(VALVE_PORT,VALVE_PIN);

	CONFIG_PIN_AS_GPIO(DI_HS_IGN_ON_PORT,DI_HS_IGN_ON_PIN,INPUT);
	CONFIG_PIN_AS_GPIO(DI_HS_02_PORT,DI_HS_02_PIN,INPUT);
	CONFIG_PIN_AS_GPIO(DI_LS_03_PORT,DI_LS_03_PIN,INPUT);
	CONFIG_PIN_AS_GPIO(DI_LS_04_PORT,DI_LS_04_PIN,INPUT);

	ENABLE_INPUT(DI_HS_IGN_ON_PORT,DI_HS_IGN_ON_PIN);
	ENABLE_INPUT(DI_HS_02_PORT,DI_HS_02_PIN);
	ENABLE_INPUT(DI_LS_03_PORT,DI_LS_03_PIN);
	ENABLE_INPUT(DI_LS_04_PORT,DI_LS_04_PIN);
}

uint8_t gpio_read(uint8_t ch)
{
	switch(ch)
	{
	case 0:
		if(READ_INPUT(DI_HS_IGN_ON_PORT,DI_HS_IGN_ON_PIN)) return 1;
		break;
	case 1:
		if(READ_INPUT(DI_HS_02_PORT,DI_HS_02_PIN)) return 1;
		break;
	case 2:
		if(READ_INPUT(DI_LS_03_PORT,DI_LS_03_PIN)) return 1;
		break;
	case 3:
		if(READ_INPUT(DI_LS_04_PORT,DI_LS_04_PIN)) return 1;
		break;
	default:
		return 0;
		break;
	}
	return 0;
}

