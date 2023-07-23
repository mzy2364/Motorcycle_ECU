/*
 * User_Management.c
 *
 *  Created on: 2023年3月31日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "User_Management.h"
#include "Engine_Management.h"
#include "TLE8080EM.h"
#include "bsp_gpio.h"
#include "typedef.h"
#include "Fuel_Control.h"
#include "Spark_Control.h"
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
/* ECU上电时候的油泵打开超时时间 */
uint16_t fuel_pump_timeout = 0;
/* 发动机启动过程的超时时间 */
uint16_t run_startup_timeout = 0;

uint8_t cal_fuel_pump_oe = 0;
uint8_t cal_fuel_pump_ov = 0;
uint8_t cal_mil_oe = 0;
uint8_t cal_mil_ov = 0;
uint8_t cal_inj_oe = 0;
uint8_t cal_inj_ov = 0;
uint8_t cal_ignition_oe = 0;
uint8_t cal_ignition_ov = 0;
uint16_t cal_idle_speed = 1500;
uint16_t cal_startup_fuel_add = 2500;

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
app_state_t app_state = INIT;
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void fuel_pump_controller_init(void);
static void fuel_pump_controller(void);
static void mil_controller(uint8_t en);
static void overwrite(void);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void user_management_init(void)
{
	fuel_pump_controller_init();
}

/**
  * @brief
  * @param void
  * @retval	void
  * @note
  */
void user_management(void)
{
	overwrite();

	switch(app_state)
	{
	case INIT:
		app_state = STOP;
		break;
	case STOP:
		mil_controller(ON);
		fuel_pump_controller();

		if(engine_speed < Stall_Speed)
		{
			app_state = START;
		}
		else if(engine_speed > Over_Speed)
		{
			app_state = OVERRUN;
		}
		else
		{
			app_state = RUN;
		}
		break;
	case START:
		mil_controller(ON);
		valve_ctrl(OFF);
		fuel_pump_controller();
		run_startup_timeout = ENGUNE_STARTUP_TOUT;
		startup_fuel_add = cal_startup_fuel_add;
		if(engine_speed < Stall_Speed)
		{
			app_state = START;
		}
		else if(engine_speed > Over_Speed)
		{
			app_state = OVERRUN;
		}
		else
		{
			app_state = RUN_STARTUP;
		}
		break;
	case RUN_STARTUP:
		mil_controller(ON);
		fuel_pump_controller();
		if(engine_speed < Stall_Speed)
		{
			app_state = START;
		}
		else if(engine_speed > Over_Speed)
		{
			app_state = OVERRUN;
		}
		else
		{
			if(run_startup_timeout > 0)
			{
				app_state = RUN_STARTUP;
				run_startup_timeout--;
				if(startup_fuel_add > 10)
				{
					startup_fuel_add -= 10;
				}
			}
			else
			{
				app_state = RUN;
				startup_fuel_add = 0;
			}
		}
		break;
	case RUN:
		mil_controller(OFF);
		fuel_pump_controller();

		if(load == 0)
		{
			valve_ctrl(ON);
		}
		else
		{
			valve_ctrl(OFF);
		}

		if(engine_speed < Stall_Speed)
		{
			app_state = START;
		}
		else if(engine_speed > Over_Speed)
		{
			app_state = OVERRUN;
		}
		else
		{
			app_state = RUN;
		}
		break;
	case OVERRUN:
		mil_controller(OFF);
		fuel_pump_controller();
		if(engine_speed < Over_Speed)
		{
			app_state = RUN;
		}
		break;
	default:
		app_state = STOP;
		break;
	}
}



/**
  * @brief 燃油泵控制初始化
  * @param void
  * @retval	void
  * @note
  */
static void fuel_pump_controller_init(void)
{
	FUEL_PUMP_CTRL(ON);
	fuel_pump_timeout = FUEL_PUMP_TOUT;
}

/**
  * @brief 喷油嘴OV
  * @param void
  * @retval	void
  * @note
  */
static void overwrite(void)
{
//	if(cal_inj_oe)
//	{
//		if(cal_inj_ov)
//			SET_INJ_ON();
//		else
//			SET_INJ_OFF();
//	}
//
//	if(cal_ignition_oe)
//	{
//		if(cal_ignition_ov)
//			SET_SPARK_ON();
//		else
//			SET_SPARK_OFF();
//
//	}
}

/**
  * @brief MIL灯控制
  * @param void
  * @retval	void
  * @note
  */
static void mil_controller(uint8_t en)
{
	if(cal_mil_oe)
		mil_ctrl(cal_mil_ov);
	else
		mil_ctrl(en);
}

/**
  * @brief 燃油泵控制
  * @param void
  * @retval	void
  * @note
  */
static void fuel_pump_controller(void)
{
	uint8_t fuel_pump_ctrl = 0;
	if((app_state == RUN) || (app_state == RUN_STARTUP))
	{
		fuel_pump_timeout = FUEL_PUMP_TOUT;
		fuel_pump_ctrl = ON;
	}
	else if((app_state == START)||(app_state == OVERRUN))
	{
		if(fuel_pump_timeout > 0)
		{
			fuel_pump_ctrl = ON;
			fuel_pump_timeout--;
		}
		else
		{
			fuel_pump_ctrl = OFF;
		}
	}
	else
	{
		if(fuel_pump_timeout > 0)
		{
			fuel_pump_ctrl = ON;
			fuel_pump_timeout--;
		}
		else
		{
			fuel_pump_ctrl = OFF;
		}
	}

	if(cal_fuel_pump_oe)
		fuel_pump_ctrl = cal_fuel_pump_ov;
	FUEL_PUMP_CTRL(fuel_pump_ctrl);
}
