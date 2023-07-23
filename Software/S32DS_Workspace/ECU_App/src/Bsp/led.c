/*
 * led.c
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "led.h"

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
  * @brief LED初始化
  * @param void
  * @retval	void
  * @note
  */
void led_init(void)
{
	CONFIG_PIN_AS_GPIO(LEDR_PORT,LEDR_PIN,OUTPUT);
	CONFIG_PIN_AS_GPIO(LEDG_PORT,LEDG_PIN,OUTPUT);
	CONFIG_PIN_AS_GPIO(LEDB_PORT,LEDB_PIN,OUTPUT);

	/* close all led */
	OUTPUT_SET(LEDR_PORT,LEDR_PIN);
	OUTPUT_SET(LEDG_PORT,LEDG_PIN);
	OUTPUT_SET(LEDB_PORT,LEDB_PIN);
}

/**
  * @brief LED打开
  * @param void
  * @retval	void
  * @note
  */
void led_on(led_ch_t led)
{
	switch(led)
	{
	case LED_R:
		OUTPUT_CLEAR(LEDR_PORT,LEDR_PIN);
		break;
	case LED_G:
		OUTPUT_CLEAR(LEDG_PORT,LEDG_PIN);
		break;
	case LED_B:
		OUTPUT_CLEAR(LEDB_PORT,LEDB_PIN);
		break;
	default:
		break;
	}
}

/**
  * @brief LED关闭
  * @param void
  * @retval	void
  * @note
  */
void led_off(led_ch_t led)
{
	switch(led)
	{
	case LED_R:
		OUTPUT_SET(LEDR_PORT,LEDR_PIN);
		break;
	case LED_G:
		OUTPUT_SET(LEDG_PORT,LEDG_PIN);
		break;
	case LED_B:
		OUTPUT_SET(LEDB_PORT,LEDB_PIN);
		break;
	default:
		break;
	}
}

/**
  * @brief LED翻转
  * @param void
  * @retval	void
  * @note
  */
void led_toggle(led_ch_t led)
{
	switch(led)
	{
	case LED_R:
		OUTPUT_TOGGLE(LEDR_PORT,LEDR_PIN);
		break;
	case LED_G:
		OUTPUT_TOGGLE(LEDG_PORT,LEDG_PIN);
		break;
	case LED_B:
		OUTPUT_TOGGLE(LEDB_PORT,LEDB_PIN);
		break;
	default:
		break;
	}
}

