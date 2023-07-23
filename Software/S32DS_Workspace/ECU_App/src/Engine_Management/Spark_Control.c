/*
 * Spark_Control.c
 *
 *  Created on: 2023年3月8日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "Spark_Control.h"
#include "Application_Define.h"
#include "Fuel_Control.h"
#include "bsp_gpio.h"
#include "ftm.h"
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
extern uint16_t next_spark_start;
extern uint8_t next_spark_dwell_tooth;
extern uint16_t next_spark_dwell_offset;

extern uint16_t Spark_Load_Map_Values[SPARK_LOAD_POINTS];
extern uint16_t Spark_RPM_Map_Values[SPARK_RPM_POINTS];
extern const uint16_t Spark_Timing_Map[SPARK_RPM_POINTS][SPARK_LOAD_POINTS];

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
spark_state_t spark_state = SPARK_CTRL_INACTIVE;

uint16_t current_spark_start = 0;
uint8_t current_spark_dwell_tooth = 0;
uint16_t current_spark_dwell_offset = 0;

/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void update_current_spark_params(void);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 点火线圈控制初始化
  * @param void
  * @retval	void
  * @note
  */
void spark_control_init(void)
{
	spark_state = READY_TO_SCHEDULE_SPARK;
}

/**
  * @brief 点火线圈控制任务
  * @param void
  * @retval	void
  * @note
  */
void spark_control_task(void)
{
	uint32_t spark_start = 0;
	if(spark_state == READY_TO_SCHEDULE_SPARK)
	{
		update_current_spark_params();
		if(teeth_counter == current_spark_dwell_tooth)
		{
			current_spark_dwell_offset = current_spark_dwell_offset * angle_clock_rate;

			spark_start = current_tooth_time + current_spark_dwell_offset;
			if(spark_start > 0XFFFF)
			{
				spark_start -= 0XFFFF;
			}

			/* Set output pin on next compare event */
			SET_SPARK_ON_NEXT_STATE();

			FTM_SetChannelValue(FTM2, SPARK_FTM_CHANNEL,(uint16_t)spark_start);
			FTM_ClrChannelFlag(FTM2, SPARK_FTM_CHANNEL);
			FTM_EnableChannelInt(FTM2, SPARK_FTM_CHANNEL);

			spark_state = WAITING_STARTING_CURRENT_FLOW;
		}
	}
}

/**
  * @brief 点火角度读取
  * @param rpm_input:转速输入值 单位 RPM
  * @param load_input:负载需求值 单位 %
  * @retval	返回查找到的最优喷油角度
  * @note
  */
uint16_t spark_map_read(uint16_t rpm_input,uint16_t load_input)
{
	uint16_t load = 0;
	uint16_t rpm = 0;

	/* Obtain index of best match for input data within map values */
	load = map_bin_search(load_input, Spark_Load_Map_Values, SPARK_LOAD_POINTS);

	rpm = map_bin_search(rpm_input, Spark_RPM_Map_Values, SPARK_RPM_POINTS);

	return (Spark_Timing_Map[rpm][load]);
}

/**
  * @brief 更新喷油任务相关参数
  * @param void
  * @retval	void
  * @note
  */
static void update_current_spark_params(void)
{
    current_spark_start = next_spark_start;
    current_spark_dwell_tooth = next_spark_dwell_tooth;
    current_spark_dwell_offset = next_spark_dwell_offset;
}

/**
  * @brief 点火线圈控制任务中断
  * @param void
  * @retval	void
  * @note
  */
void spark_control_isr(void)
{
	uint32_t dwell_count = 0;
	if(spark_state == WAITING_STARTING_CURRENT_FLOW)
	{
		/* Ingition signal IGN is ON at this moment */
		dwell_count = FTM_GetCountValue(FTM2) + DWELL_TIME;
		if(dwell_count > 0XFFFF)
		{
			dwell_count -= 0XFFFF;
		}

		CLR_SPARK_ON_NEXT_STATE();

		FTM_SetChannelValue(FTM2, SPARK_FTM_CHANNEL,(uint16_t)dwell_count);
		FTM_EnableChannelInt(FTM2, SPARK_FTM_CHANNEL);

		spark_state = WAITING_IGNITION_TIME;
	}
	else if(spark_state == WAITING_IGNITION_TIME)
	{
		/* Ingition signal IGN is OFF at this moment */
		FTM_DisableChannelInt(FTM2, SPARK_FTM_CHANNEL);
		spark_state = READY_TO_SCHEDULE_SPARK;
	}
	else
	{
		FTM_DisableChannelInt(FTM2, SPARK_FTM_CHANNEL);
	}
}
