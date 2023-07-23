/*
 * Engine_Management.c
 *
 *  Created on: 2023年3月6日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "Engine_Management.h"
#include "Crank_Sensing.h"
#include "Fuel_Control.h"
#include "MAP.h"
#include "TPS.h"
#include "ETEMP.h"
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
/** Engine speed (RPM) */
uint32_t engine_speed = 0;
/** Load */
uint32_t load = 0;
uint8_t load_valid = Invalid;
/* mil */
uint8_t mil_status = 0;

extern uint16_t prior_periods[];

/* Fuel injection pulse variables */
uint16_t next_fuel_start = 0;
uint16_t next_fuel_start_tooth = 0;
uint16_t next_fuel_start_offset = 0;
uint16_t next_fuel_start_pulse = 0;

/* Ignition spark pulse variables */
uint16_t next_spark_start = 0;
uint8_t next_spark_dwell_tooth = 0;
uint16_t next_spark_dwell_offset = 0;

/* Fuel pulse auxiliary variables */
uint16_t fuel_start_angle = 0;
uint16_t fuel_start_tooth = 0;
uint16_t fuel_pulse_width = 0;
uint16_t fuel_angle_offset = 0;

/* Ignition spark pulse variables */
uint16_t spark_start_angle = 0;
uint16_t spark_dwell_angle = 0;
uint8_t spark_dwell_tooth = 0;
uint16_t spark_dwell_offset = 0;

///* calibration */
//uint16_t cal_tps_min = 185;		//节气门开度0%的时候的ADC值
//uint16_t cal_tps_max = 2775;	//节气门开度100%的时候ADC值
uint16_t cal_tps_min = 185;		//节气门开度0%的时候的ADC值
uint16_t cal_tps_max = 2705;	//节气门开度100%的时候ADC值
uint16_t cal_fuel_correction = FUEL_CORRECTION_INIT;	//喷油脉宽修正 偏移
uint8_t cal_spark_angle_correction = SPARK_CORRECTION_INIT;
uint16_t cal_spark_angle = 110;

uint16_t startup_fuel_add = 0;
uint16_t fuel_add = 0;
uint16_t fuel_cut = 0;
uint16_t spark_advance = 0;
uint16_t spark_retard = 0;

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
  * @brief 发动机管理模块初始化
  * @param void
  * @retval	void
  * @note
  */
void engine_management_init(void)
{
	map_init();
	tps_init();
	etemp_init();
}

/**
  * @brief 发动机管理
  * @param void
  * @retval	void
  * @note
  */
void engine_management(void)
{
	tps_monitoring();

	fuel_pulse(engine_speed,load);
	fuel_start(engine_speed,load);

	spark_start(engine_speed,load);
	spark_dwell();
}

/**
  * @brief 发动机管理10ms任务
  * @param void
  * @retval	void
  * @note
  */
void engine_management_10ms(void)
{
	engine_speed_calculate();
	load_calculate();
	etemp_monitoring();
}

/**
  * @brief 喷油起始角度计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @retval	void
  * @note
  */
void fuel_start(uint16_t rpm,uint16_t load)
{
	/* Calculate angle to start next fuel pulse */
	fuel_start_cal(rpm, load, FUEL1_REF_ANGLE);

	/* Updates start angle and tooth before it */
	next_fuel_start = fuel_start_angle;
	next_fuel_start_tooth = fuel_start_tooth;
	next_fuel_start_offset = fuel_angle_offset;
}

/**
  * @brief 喷油起始角度计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @param ref_angle:参考角度
  * @retval	void
  * @note
  */
void fuel_start_calculate(uint16_t rpm,uint16_t load,uint16_t ref_angle)
{
	fuel_start_angle = fuel_pulse_angle_map_read(rpm, load);

	if(crank_state == SYNCHRONIZED_4C)
	{
		//Compensate for missing tooth not TDC.
		fuel_start_angle += ref_angle;

		if(fuel_start_angle >= 720)
		{
			//Keep the angle bgetween 0 and 720 as the process is two revolutions.
			fuel_start_angle -= 720;
		} 

	}
	else
	{
		//Compensate for missing tooth not TDC.
		fuel_start_angle += ref_angle;

		if(fuel_start_angle >= 360)
		{
			//Keep the angle between 0 and 360 as the process is one revolution.
			fuel_start_angle -= 360;
		}
	}

	/* Calculate tooth before fuel pulse event */
	fuel_start_tooth = fuel_start_angle / DEGREES_PER_TOOTH;

	/* Calculate angle error to properly estimate the pulse start */
	fuel_angle_offset = fuel_start_angle % DEGREES_PER_TOOTH;

	if(!fuel_start_tooth)
	{
		if(crank_state == SYNCHRONIZED_4C)
		{
			/* Adjust if calculated tooth is zero- synchronized to 4 stroke */
			fuel_start_tooth = NUMBER_OF_TEETH_4C;
		}
		else
		{
			/* Adjust if calculated tooth is zero- not synchronized to 4 stroke */
			fuel_start_tooth = NUMBER_OF_TEETH;
		}
	}

	if(!fuel_angle_offset)
	{
		/* If there is not angle error, go back another tooth */
		fuel_start_tooth --; 

		if(!fuel_start_tooth)
		{

			if(crank_state == SYNCHRONIZED_4C)
			{
				/* Adjust if calculated tooth is zero- synchronized to 4 stroke */
				fuel_start_tooth = NUMBER_OF_TEETH_4C;
			}
			else
			{
				/* Adjust if calculated tooth is zero- not synchronized to 4 stroke */
				fuel_start_tooth = NUMBER_OF_TEETH;
			}
		}
		/* Time to start pulse is 1 period */
		fuel_angle_offset = DEGREES_PER_TOOTH;
	}

	/* Decrement to match teeth numbering scheme (0 to NUMBER_OF:TEETH -1)*/
	fuel_start_tooth --;
}

/**
  * @brief 喷油起始角度计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @param ref_angle:参考角度
  * @retval	void
  * @note
  */
void fuel_start_cal(uint16_t rpm,uint16_t load,uint16_t ref_angle)
{
	fuel_start_angle = next_fuel_start_pulse / angle_clock_rate;

	if(fuel_start_angle >= ref_angle)
	{
		if(crank_state == SYNCHRONIZED_4C)
		{
			fuel_start_angle = 720 - (fuel_start_angle - ref_angle);
		}
		else
		{
			fuel_start_angle = 360 - (fuel_start_angle - ref_angle);
		}
	}
	else
	{
		fuel_start_angle = ref_angle - fuel_start_angle;
	}

	fuel_start_tooth = fuel_start_angle / DEGREES_PER_TOOTH;

	fuel_angle_offset = fuel_start_angle % DEGREES_PER_TOOTH;

	if(!fuel_start_tooth)
	{
		if(crank_state == SYNCHRONIZED_4C)
		{
			fuel_start_tooth = NUMBER_OF_TEETH_4C;
		}
		else
		{
			fuel_start_tooth = NUMBER_OF_TEETH;
		}
	}

	if(!fuel_angle_offset)
	{
		fuel_start_tooth--;

		if(!fuel_start_tooth)
		{
			if(crank_state == SYNCHRONIZED_4C)
			{
				fuel_start_tooth = NUMBER_OF_TEETH_4C;
			}
			else
			{
				fuel_start_tooth = NUMBER_OF_TEETH;
			}
		}

		fuel_angle_offset = DEGREES_PER_TOOTH;
	}

	fuel_start_tooth--;
}

/**
  * @brief 喷油脉宽计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @retval	void
  * @note
  */
void fuel_pulse(uint16_t rpm,uint16_t load)
{
	next_fuel_start_pulse = fuel_pulse_width_calculate(rpm, load);
}

/**
  * @brief 喷油脉宽计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @retval	void
  * @note
  */
uint32_t fuel_pulse_width_calculate(uint16_t rpm,uint16_t load)
{
	fuel_pulse_width = fuel_pulse_width_map_read(rpm, load);

	fuel_pulse_width = fuel_pulse_width + startup_fuel_add + fuel_add - fuel_cut;

	if(cal_fuel_correction != FUEL_CORRECTION_INIT)
	{
		if(cal_fuel_correction > FUEL_CORRECTION_INIT)
		{
			fuel_pulse_width += (cal_fuel_correction - FUEL_CORRECTION_INIT);
		}
		else
		{
			if(fuel_pulse_width > (FUEL_CORRECTION_INIT - cal_fuel_correction))
				fuel_pulse_width -= (FUEL_CORRECTION_INIT - cal_fuel_correction);
		}
	}

	if(crank_state != SYNCHRONIZED_4C)
	{
		/* 四冲程未同步 以二冲程模式运行 */
		fuel_pulse_width >>= 1;
	}

	return fuel_pulse_width;
}

/**
  * @brief 点火起始角度计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @retval	void
  * @note
  */
void spark_start(uint16_t rpm,uint16_t load)
{
	/* Calculate spark event angle */
	next_spark_start = spark_start_calculate(rpm, load, cal_spark_angle);
}

/**
  * @brief 点火起始角度计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @param ref_angle:参考角度
  * @retval	void
  * @note
  */
uint16_t spark_start_calculate(uint16_t rpm,uint16_t load,uint16_t ref_angle)
{
	spark_start_angle = spark_map_read(rpm,load);

	if(crank_state == SYNCHRONIZED_4C)
	{
		if(ref_angle > spark_start_angle)
		{
			spark_start_angle = ref_angle + 360 - spark_start_angle;
		}
		else
		{
			spark_start_angle = 720 - spark_start_angle - ref_angle;
		}
	}
	else
	{
		if(ref_angle > spark_start_angle)
		{
			spark_start_angle = ref_angle - spark_start_angle;
		}
		else
		{
			spark_start_angle = 360 - spark_start_angle - ref_angle;
		}
	}
	spark_start_angle = spark_start_angle + spark_advance - spark_retard;

	return spark_start_angle;
}

/**
  * @brief 点火起始位置计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @retval	void
  * @note
  */
void spark_dwell(void)
{
	spark_dwell_calculate(next_spark_start);

	next_spark_dwell_tooth = spark_dwell_tooth;
	next_spark_dwell_offset = spark_dwell_offset;
}

/**
  * @brief 点火起始位置计算
  * @param rpm:发动机转速 单位 RPM
  * @param load:需求负载 单位 %
  * @param ref_angle:参考角度
  * @retval	void
  * @note
  */
void spark_dwell_calculate(uint16_t start_angle)
{
	spark_dwell_angle = (DWELL_TIME) / angle_clock_rate;

	if(spark_dwell_angle >= start_angle)
	{
		start_angle = 360 - (spark_dwell_angle - start_angle);
	}
	else
	{
		start_angle = start_angle - spark_dwell_angle;
	}

	spark_dwell_tooth = start_angle / DEGREES_PER_TOOTH;

	spark_dwell_offset = start_angle % DEGREES_PER_TOOTH;

	if(!spark_dwell_tooth)
	{
		if(crank_state == SYNCHRONIZED_4C)
		{
			spark_dwell_tooth = NUMBER_OF_TEETH_4C;
		}
		else
		{
			spark_dwell_tooth = NUMBER_OF_TEETH;
		}
	}

	if(!spark_dwell_offset)
	{
		spark_dwell_tooth--;

		if(!spark_dwell_tooth)
		{
			if(crank_state == SYNCHRONIZED_4C)
			{
				spark_dwell_tooth = NUMBER_OF_TEETH_4C;
			}
			else
			{
				spark_dwell_tooth = NUMBER_OF_TEETH;
			}
		}

		spark_dwell_offset = DEGREES_PER_TOOTH;
	}

	spark_dwell_tooth--;
}

/**
  * @brief 发动机转速计算
  * @param void
  * @retval	void
  * @note
  */
void engine_speed_calculate(void)
{
//	static uint8_t eng_speed_invalid_count = 0;
	uint32_t periods_sum = 0;
	uint32_t loop_count = 0;
	uint32_t data_count = 0;
	uint32_t period = 0;

	loop_count = 2*NUMBER_OF_TEETH_4C-1;

	while(loop_count > 0)
	{
		if(prior_periods[loop_count] != 0)
		{
			periods_sum += prior_periods[loop_count - 1];
			data_count++;
		}
		loop_count--;
	}

	if(data_count)
	{
		period = periods_sum / data_count;
		engine_speed = PERIOD_TO_RPM(period);
	}
	else
	{
		engine_speed = 0;
//		eng_speed_invalid_count++;
//		if(eng_speed_invalid_count >= 10)
//		{
//			eng_speed_invalid_count = 0;
//			engine_speed = 0;
//		}
	}

}

/**
  * @brief 节气门位置计算
  * @param void
  * @retval	void
  * @note
  */
void load_calculate(void)
{
	uint16_t gas_pedal = tps_adc_get();
	if(cal_tps_min <= cal_tps_max)
	{
		if((gas_pedal < cal_tps_min) || (gas_pedal > cal_tps_max))
		{
			load_valid = Invalid;
			load = 0;
		}
		else
		{
			load_valid = Valid;
			load = ((float)(gas_pedal - cal_tps_min) / (cal_tps_max - cal_tps_min)) * 100.0f;
		}
	}
	else
	{
		if((gas_pedal < cal_tps_max) || (gas_pedal > cal_tps_min))
		{
			load_valid = Invalid;
			load = 0;
		}
		else
		{
			load_valid = Valid;
			load = ((float)(cal_tps_min - gas_pedal) / (cal_tps_min - cal_tps_max)) * 100.0f;
		}
	}
}

