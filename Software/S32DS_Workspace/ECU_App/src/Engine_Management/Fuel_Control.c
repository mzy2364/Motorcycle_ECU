/*
 * Fuel_Control.c
 *
 *  Created on: 2023年3月2日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include <stdlib.h>
#include "Fuel_Control.h"
#include "bsp_gpio.h"
#include "ftm.h"
#include "Engine_Management.h"
#include "Crank_Sensing.h"
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
extern uint16_t current_tooth_time;
extern uint16_t Fuel_Load_Map_Values[FUEL_LOAD_POINTS];
extern uint16_t Fuel_RPM_Map_Values[FUEL_RPM_POINTS];
extern const uint32_t Fuel_Pulse_Width_Map[FUEL_RPM_POINTS][FUEL_LOAD_POINTS];
extern const uint16_t Fuel_Pulse_Angle_Map[FUEL_RPM_POINTS][FUEL_LOAD_POINTS];

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
fuel_state_t fuel_state = FUEL_CTRL_INACTIVE;

uint16_t current_fuel_start = 0;
uint16_t current_fuel_start_tooth = 0;
uint16_t current_fuel_start_offset = 0;
uint16_t current_fuel_pulse = 0;

extern uint16_t next_fuel_start;
extern uint16_t next_fuel_start_tooth;
extern uint16_t next_fuel_start_offset;
extern uint16_t next_fuel_start_pulse;

/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void update_current_fuel_params(void);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 喷油控制初始化
  * @param void
  * @retval	void
  * @note
  */
void fuel_control_init(void)
{
	fuel_state = FUEL_CTRL_READY;
}

/**
  * @brief 喷油控制任务
  * @param void
  * @retval	void
  * @note
  */
void fuel_control_task(void)
{
	uint32_t fuel_start = 0;
	if(fuel_state == FUEL_CTRL_READY)
	{
		update_current_fuel_params();
		if(teeth_counter == current_fuel_start_tooth)
		{
			current_fuel_start_offset = current_fuel_start_offset * angle_clock_rate;

			fuel_start = current_tooth_time + current_fuel_start_offset;
			if(fuel_start > 0XFFFF)
			{
				fuel_start -= 0XFFFF;
			}

			/* Set output pin on next compare event */
			SET_FUEL_ON_NEXT_STATE();

			FTM_SetChannelValue(FTM2, FUEL_FTM_CHANNEL,(uint16_t)fuel_start);
			FTM_ClrChannelFlag(FTM2, FUEL_FTM_CHANNEL);
			FTM_EnableChannelInt(FTM2, FUEL_FTM_CHANNEL);

			fuel_state = FUEL_PULSE_SCHEDULED;
		}
	}
}



/**
  * @brief 喷油角度读取
  * @param rpm_input:转速输入值 单位 RPM
  * @param load_input:负载需求值 单位 %
  * @retval	返回查找到的最优喷油角度
  * @note
  */
uint16_t fuel_pulse_angle_map_read(uint16_t rpm_input,uint16_t load_input)
{
	uint16_t load = 0;
	uint16_t rpm = 0;

	/* Obtain index of best match for input data within map values */
	load = map_bin_search(load_input, Fuel_Load_Map_Values, FUEL_LOAD_POINTS);

	rpm = map_bin_search(rpm_input, Fuel_RPM_Map_Values, FUEL_RPM_POINTS);

	return (Fuel_Pulse_Angle_Map[rpm][load]);
}

/**
  * @brief 喷油脉宽读取
  * @param rpm_input:转速输入值 单位 RPM
  * @param load_input:负载需求值 单位 %
  * @retval	返回查找到的最优喷油角度
  * @note
  */
uint32_t fuel_pulse_width_map_read(uint16_t rpm_input,uint16_t load_input)
{
	uint16_t load = 0;
	uint16_t rpm = 0;

	/* Obtain index of best match for input data within map values */
	load = map_bin_search(load_input, Fuel_Load_Map_Values, FUEL_LOAD_POINTS);

	rpm = map_bin_search(rpm_input, Fuel_RPM_Map_Values, FUEL_RPM_POINTS);

	return (Fuel_Pulse_Width_Map[rpm][load]);
}

/**
  * @brief 在map中查找输入值最接近的数组下标
  * @param target:需要查找的值
  * @param array:数组指针
  * @param array_length:数组长度
  * @retval	返回查找到的最接近的值的数组下标
  * @note 调用前必须确保数组长度大于1,数组需要先升序排序
  */
uint16_t map_bin_search(uint16_t target, uint16_t *array, uint16_t array_length)
{
	uint16_t low = 0;
	uint16_t high = array_length - 1;
	uint16_t mid = 0;
	int result = -1;
	int left_diff = 0,right_diff = 0;

	while(low <= high)
	{
		mid = (low + high) / 2;
		if(target < array[mid])			//如果如果target < mid 则新区间为[low,mid-1]
		{
			if(mid == 0)
				break;
			high = mid-1;
		}
		else if(target > array[mid])	//如果target > mid 则新区间为[mid+1,high]
		{
			if(mid >= array_length)
				break;
			low = mid+1;
		}
		else
		{
			result = mid;
			break;
		}
	}

	if(result == -1)
	{
		if (target > array[mid])
		{
			if(mid >= array_length)
			{
				result = result - 1;
			}
			else
			{
				left_diff = abs(target - array[mid]);
				right_diff = abs(array[mid+1] - target);
				if(left_diff > right_diff)
					result = mid + 1;
				else
					result = mid;
			}
		}
		else
		{
			if(mid == 0)
			{
				result = mid;
			}
			else
			{
				left_diff = abs(target - array[mid-1]);
				right_diff = abs(array[mid] - target);
				if(left_diff > right_diff)
					result = mid;
				else
					result = mid - 1;
			}
		}
	}

	return result;
}

/**
  * @brief 更新喷油任务相关参数
  * @param void
  * @retval	void
  * @note
  */
static void update_current_fuel_params(void)
{
	current_fuel_start = next_fuel_start;
	current_fuel_pulse = next_fuel_start_pulse;
    current_fuel_start_tooth = next_fuel_start_tooth;
    current_fuel_start_offset = next_fuel_start_offset;
}

/**
  * @brief 喷油控制任务中断
  * @param void
  * @retval	void
  * @note
  */
void fuel_control_isr(void)
{
	uint32_t fuel_width = 0;
	if(fuel_state == FUEL_PULSE_SCHEDULED)
	{
		/* Signal INJ is ON at this moment */

		fuel_width = FTM_GetCountValue(FTM2) + current_fuel_pulse;
		if(fuel_width > 0XFFFF)
		{
			fuel_width -= 0XFFFF;
		}

		CLR_FUEL_ON_NEXT_STATE();

		FTM_SetChannelValue(FTM2, FUEL_FTM_CHANNEL,(uint16_t)fuel_width);
		FTM_EnableChannelInt(FTM2, FUEL_FTM_CHANNEL);

		fuel_state = FUEL_PULSE_ON;
	}
	else if(fuel_state == FUEL_PULSE_ON)
	{
		/* Signal INJ is OFF at this moment */

		FTM_DisableChannelInt(FTM2, FUEL_FTM_CHANNEL);
		fuel_state = FUEL_CTRL_READY;
	}
	else
	{
		FTM_DisableChannelInt(FTM2, FUEL_FTM_CHANNEL);
	}
}


