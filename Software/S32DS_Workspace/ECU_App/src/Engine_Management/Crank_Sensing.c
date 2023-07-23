/*
 * Crank_Sensing.c
 *
 *  Created on: 2023年3月4日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "Crank_Sensing.h"
#include "Spark_Control.h"
#include "Fuel_Control.h"
#include "ftm.h"
#include "led.h"
#include "Engine_Speed_Output.h"
#include "MAP.h"
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
uint16_t previous_period = 0;
uint16_t current_period = 0;
uint16_t current_tooth_time = 0;
uint16_t previous_tooth_time = 0;

uint16_t last_timeout = 0;

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
/* Initial State */
crank_state_t crank_state = TOOTH_INIT;

uint16_t prior_periods[2 * NUMBER_OF_TEETH_4C];

/** Index to store period data */
static uint8_t period_counter = 0;

uint8_t teeth_counter = 0;

uint8_t Eng_Rotation_Sync_cont = 0;

/* 缺齿有效标志 在发生缺齿中断的时候置一 */
uint8_t missing_tooth_valid = 0;

//uint32_t missing_tooth_int_cnt = 0;
//uint32_t missing_tooth_int_tick = 0;
//uint32_t missing_tooth_int_en_times = 0;
//uint32_t missing_tooth_int_times = 0;
//uint32_t tooth_timeout_tick = 0;

/* 一个角度对应多少个定时器的counter */
uint32_t angle_clock_rate = 0;

//Flag for indicating a positive test for intake pressure signature.
uint8_t signature_valid = 0;

//Index for looking at the MAP data buffer.
uint8_t buffer_counter = 0;

/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void ftm2_isr(void);
static uint8_t calculate_tooth_period(void);
static uint8_t missing_tooth_gap(void);
static uint8_t tooth_after_gap(void);
static uint8_t valid_period(void);
static void increment_teeth_counter(void);
static void missing_tooth_interupt_enable(uint8_t en);
static void period_timeout_set(void);

/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 曲轴信号检测初始化
  * @param void
  * @retval	void
  * @note
  */
void crank_sensing_init(void)
{
	FTM_ConfigType FTM2_Config = {0};
	FTM_ChParamsType FTM2CH0_Config = {0};
	FTM_ChParamsType FTM2CH1_Config = {0};
	FTM_ChParamsType FTM2CH2_Config = {0};
	FTM_ChParamsType FTM2CH3_Config = {0};
	FTM_ChParamsType FTM2CH5_Config = {0};

	SIM_PINSEL1 |= SIM_PINSEL1_FTM2PS3(1);	//FTM2 CH3 映射到 PTD1 上	曲轴采集信号
	SIM_PINSEL1 |= SIM_PINSEL1_FTM2PS2(1);	//FTM2 CH2 映射到 PTD0 上	缺齿中断通道
	SIM_PINSEL1 |= SIM_PINSEL1_FTM2PS1(2);	//FTM2 CH1 映射到 PTF1 上	点火驱动
	SIM_PINSEL1 |= SIM_PINSEL1_FTM2PS0(2);	//FTM2 CH0 映射到 PTF0 上	喷油信号
	SIM_PINSEL1 |= SIM_PINSEL1_FTM2PS5(0);	//FTM2 CH5 映射到 PTB5 上	齿信号缺失超时中断

	FTM2_Config.modulo = 0XFFFF;
	FTM2_Config.clk_source = FTM_CLOCK_SYSTEMCLOCK;
	FTM2_Config.cntin = 0;
	FTM2_Config.prescaler = FTM_CLOCK_PS_DIV16;
	FTM2_Config.toie = 0;
	FTM2_Config.filter = 0X2000;

	FTM2CH3_Config.ctrl.bits.bMode = FTM_INPUT_CAPTURE;
	FTM2CH3_Config.ctrl.bits.bEdge = FTM_INPUTCAPTURE_RISINGEDGE;
	FTM2CH3_Config.ctrl.bits.bCHIE = 1;

	FTM2CH2_Config.ctrl.bits.bMode = FTM_OUTPUT_COMPARE;
	FTM2CH2_Config.ctrl.bits.bCHIE = 0;
	FTM2CH2_Config.ctrl.bits.bOutCmp = FTM_OUTPUT_TOGGLE;
	FTM2CH2_Config.u16CnV = 0;

	FTM2CH1_Config.ctrl.bits.bMode = FTM_OUTPUT_COMPARE;
	FTM2CH1_Config.ctrl.bits.bCHIE = 0;
	FTM2CH1_Config.ctrl.bits.bOutCmp = FTM_OUTPUT_CLEAR;
	FTM2CH1_Config.u16CnV = 0;

	FTM2CH0_Config.ctrl.bits.bMode = FTM_OUTPUT_COMPARE;
	FTM2CH0_Config.ctrl.bits.bCHIE = 0;
	FTM2CH0_Config.ctrl.bits.bOutCmp = FTM_OUTPUT_CLEAR;
	FTM2CH0_Config.u16CnV = 0;

	FTM2CH5_Config.ctrl.bits.bMode = FTM_OUTPUT_COMPARE;
	FTM2CH5_Config.ctrl.bits.bCHIE = 0;
	FTM2CH5_Config.ctrl.bits.bOutCmp = FTM_OUTPUT_TOGGLE;
	FTM2CH5_Config.u16CnV = 0;


	FTM_SetCallback(FTM2, ftm2_isr);
	FTM_ChannelInit(FTM2,0,FTM2CH0_Config);
	FTM_ChannelInit(FTM2,1,FTM2CH1_Config);
	FTM_ChannelInit(FTM2,2,FTM2CH2_Config);
	FTM_ChannelInit(FTM2,3,FTM2CH3_Config);
	FTM_ChannelInit(FTM2,5,FTM2CH5_Config);
	FTM_Init(FTM2,&FTM2_Config);

	crank_goto_first_edge();
}

/**
  * @brief 曲轴状态切换到第一个边沿
  * @param void
  * @retval	void
  * @note
  */
void crank_goto_first_edge(void)
{
	crank_state = FIRST_EDGE;

	for(period_counter = 0; period_counter < 2*NUMBER_OF_TEETH_4C; period_counter++)
	{
		prior_periods[period_counter] = 0X0000;
	}

	period_counter = 0;
}

/**
  * @brief 齿周期存储
  * @param void
  * @retval	void
  * @note
  */
void tooth_period_store(void)
{
	/* Tooth period data is stored for two engine revolutions */
	if(period_counter >= 2*NUMBER_OF_TEETH_4C)
	{
		period_counter = 0;
	}
	prior_periods[period_counter] = current_period;
	period_counter++;
}

/**
  * @brief 计算齿周期是否在合理范围内
  * @param void
  * @retval	1-齿周期有效，在合理范围内
  *			0-齿周期超限
  * @note
  */
static uint8_t calculate_tooth_period(void)
{
	current_tooth_time = FTM_GetCountValue(FTM2);

	if(previous_tooth_time > current_tooth_time)
	{
		current_period = (0XFFFF - previous_tooth_time) + current_tooth_time;
	}
	else
	{
		current_period = current_tooth_time - previous_tooth_time;
	}


	/* Validate tooth period against valid limits */
	//This will have limitations when in any synchronized state.
	//The problem is that when you get to a missing tooth, the tooth period
	//will change by the gap ratio (double for typical application).
	//When this happens, the test below uses limits for non-missing tooth
	//periods.  This is only an issue at low RPM.  Function can easily be
	//modified to remove this test or add special case.
	if((current_period < MAX_PERIOD_LIMIT) && (current_period > MIN_PERIOD_LIMIT))
	{
		/* Tooth period is within range */
		return 1;
	}
	else
	{
		return 0;
	}
}

static uint8_t valid_period(void)
{
	//Return variable
	uint8_t period_valid = 0;

	uint32_t period0 = 0, period1 = 0;

	/* Calculate previous period plus tolerance */

	period0 = previous_period;
	period0 = period0 + (period0 * POS_PERIOD_PERCENTAGE)/100;

	/* Calculate previous period minus tolerance */
	period1 = previous_period;
	period1 = period1 - (period1 * NEG_PERIOD_PERCENTAGE)/100;

	/* Compare current period against previous period +/- tolerance */
	//Compare against negative tolerance
	if(current_period < period1){
		//Tooth came in faster than mechanical limitation defined in the
		//application header file.
		//Reject this tooth as noise by restoring previous tooth time
		//current_period = 0;
		period_valid = 0;
	}
	else 
	{
		//Test against a slowing engine for positive tolerance
		if(current_period > period0)
		{
			//Engine is slowing and out of synchronization tolerance.
			period_valid = 0;

		}
		else
		{
			//Tooth is in tolerance.
			//Period is valid and ready to be processed.
			period_valid = 1;

		}

	}

	return(period_valid);
}

/**
  * @brief 寻找缺齿
  * @param void
  * @retval	1-找到缺齿
  *			0-没有找到缺齿
  * @note
  */
static uint8_t missing_tooth_gap(void)
{
	uint32_t period = 0;

	/* period1 is "multiplied" by 16 to allow better resolution */
	period  = ((uint32_t)current_period << 4) / previous_period;

	if((period > MIN_GAP_RATIO) && (period < MAX_GAP_RATIO))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
  * @brief 验证缺齿后面的一个齿
  * @param void
  * @retval	1-验证通过
  *			0-验证不通过
  * @note
  */
static uint8_t tooth_after_gap(void)
{
	uint32_t period = 0;

	/* period1 is "multiplied" by 16 to allow better resolution */
	period  = (previous_period << 4) / current_period;

	if((period > MIN_GAP_RATIO) && (period < MAX_GAP_RATIO))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
  * @brief 验证缺齿中断使能
  * @param en 1-使能  0-不使能
  * @retval	None
  * @note
  */
static void missing_tooth_interupt_enable(uint8_t en)
{
	if(en)
	{
		FTM_SetChannelValue(FTM2, FTM_CHANNEL_CHANNEL2,current_period + current_tooth_time);
		FTM_ClrChannelFlag(FTM2, FTM_CHANNEL_CHANNEL2);
		FTM_EnableChannelInt(FTM2, FTM_CHANNEL_CHANNEL2);
	}
	else
	{
		FTM_DisableChannelInt(FTM2, FTM_CHANNEL_CHANNEL2);
	}
}

/**
  * @brief 设置周期的超时时间和中断
  * @param None
  * @retval	None
  * @note
  */
static void period_timeout_set(void)
{
	uint32_t period = current_period;

	period = (period * CRANKSHAFT_TIMEOUT_PERCENTAGE) / 100;

	last_timeout = current_period + (uint16_t)period;

	previous_period = current_period;
	previous_tooth_time = current_tooth_time;

	//period = FTM_GetCountValue(FTM2) + (last_timeout << 2);
	period = current_tooth_time + (current_period << 2);

	if(period > 0XFFFF)
	{
		period -= 0XFFFF;
	}

	FTM_SetChannelValue(FTM2, FTM_CHANNEL_CHANNEL5,(uint16_t)period);
	FTM_ClrChannelFlag(FTM2, FTM_CHANNEL_CHANNEL5);
	FTM_EnableChannelInt(FTM2, FTM_CHANNEL_CHANNEL5);

}

static void increment_teeth_counter(void)
{
	//Increment the tooth counter based on if the application is synchronized to the valves.
	if(crank_state == SYNCHRONIZED_4C)
	{
		//Increment based on two rotations of the engine for the four cycle process.
		if(teeth_counter == NUMBER_OF_TEETH_4C - 1)
		{
			//This was the last tooth.  Reset to starting tooth.  
			teeth_counter = 0;
		}
		else 
		{  
			teeth_counter++;
		}
	}
	else
	{
		//Engine is not sychronized to the valve and continue in two cyle mode.
		if(teeth_counter == NUMBER_OF_TEETH - 1)
		{
			//This was the last tooth.  Reset to starting tooth.  
			teeth_counter = 0;
		}
		else 
		{
			teeth_counter++;
		}
	}
}

/**
  * @brief 曲轴信号定时器中断函数
  * @param void
  * @retval	void
  * @note
  */
static void ftm2_isr(void)
{
	if(FTM_GetChannelFlag(FTM2, FTM_CHANNEL_CHANNEL2) && (FTM2_C2SC & FTM_CnSC_CHIE_MASK))
	{
		FTM_ClrChannelFlag(FTM2, FTM_CHANNEL_CHANNEL2);
		FTM_DisableChannelInt(FTM2, FTM_CHANNEL_CHANNEL5);
		missing_tooth_interupt_enable(0);
		teeth_counter++;

		current_tooth_time = previous_tooth_time + previous_period;
		if(previous_tooth_time > current_tooth_time)
		{
			current_period = (0XFFFF - previous_tooth_time) + current_tooth_time;
		}
		else
		{
			current_period = current_tooth_time - previous_tooth_time;
		}
		period_timeout_set();

		angle_clock_rate = current_period / DEGREES_PER_TOOTH;
		spark_control_task();
		fuel_control_task();
		engine_speed_output_pulse_end();
	}
	if(FTM_GetChannelFlag(FTM2, FTM_CHANNEL_CHANNEL3) && (FTM2_C3SC & FTM_CnSC_CHIE_MASK))
	{
#if(TEETH_LED_DEBUG == 1)
		led_on(LED_B);
#endif
		FTM_ClrChannelFlag(FTM2, FTM_CHANNEL_CHANNEL3);
		FTM_DisableChannelInt(FTM2, FTM_CHANNEL_CHANNEL5);
		switch(crank_state)
		{
			case FIRST_EDGE:
			{
				previous_tooth_time = FTM_GetCountValue(FTM2);
				FTM2->MOD = previous_tooth_time;
				FTM_EnableOverflowInt(FTM2);
				crank_state = FIRST_PERIOD;   /* Go to next state */
			}
			break;
			case FIRST_PERIOD:
			{
				if(calculate_tooth_period())
				{
					tooth_period_store();

					period_timeout_set();

					crank_state = TESTING_FOR_GAP;   /* Go to next state */
				}
				else
				{
					crank_goto_first_edge();
				}
			}
			break;
			case TESTING_FOR_GAP:
			{
				if((calculate_tooth_period() && valid_period()) || missing_tooth_gap())
				{
					tooth_period_store();

					if(missing_tooth_gap())
					{
						teeth_counter = 0;

						if(Eng_Rotation_Sync_cont < 0XFF)
						{
							Eng_Rotation_Sync_cont = 0;
						}

						crank_state = VALIDATING_GAP;   /* Go to next state */
					}
//					else
//					{
//						crank_goto_first_edge();
//					}

					period_timeout_set();
				}
				else
				{
					crank_goto_first_edge();
				}
			}
			break;
			case VALIDATING_GAP:
			{
				if(calculate_tooth_period() && tooth_after_gap())
				{
					tooth_period_store();

					teeth_counter = 1;

					crank_state = SYNCHRONIZED;		/* Go to synchronization state */

					period_timeout_set();
				}
				else
				{
					crank_goto_first_edge();
				}
			}
			break;
			case SYNCHRONIZED:
			{
				if((calculate_tooth_period() && valid_period()) || (missing_tooth_valid == 1))
				{
					tooth_period_store();

					increment_teeth_counter();

#ifdef MAP_TOOTH_BASED
					map_monitoring();
					if(teeth_counter==MAP_TOOTH_END+1)
					{
						signature_valid = 0;
						buffer_counter=0;

						while(buffer_counter<MAP_BUFFER_SIZE)
						{

							if((map_data_buffer[buffer_counter]-map_data_buffer[buffer_counter+1])>MAP_SIGNATURE_DROP_MIN)
							{
								signature_valid=1;
								buffer_counter++;
							}
							else
							{
								buffer_counter=MAP_BUFFER_SIZE;
								signature_valid=0;
							}

						}
						if(signature_valid==1)
						{
							crank_state = SYNCHRONIZED_4C;
						}
						else
						{
							crank_state = SYNCHRONIZED;
						}
					}
#endif

					if(missing_tooth_valid == 1)
					{
						missing_tooth_valid = 0;
					}
					if(teeth_counter == NUMBER_OF_TEETH - 2)
					{
						missing_tooth_interupt_enable(1);
						engine_speed_output_pulse_start();
					}
					if(teeth_counter == 0)
					{
						missing_tooth_interupt_enable(0);
					}

					angle_clock_rate = current_period / DEGREES_PER_TOOTH;
					spark_control_task();
					fuel_control_task();

					period_timeout_set();

				}
				else
				{
#if(TEETH_LED_DEBUG == 1)
					led_toggle(LED_G);
#endif
					crank_goto_first_edge();
				}
			}
			break;
			case SYNCHRONIZED_4C:
			{
				if((calculate_tooth_period() && valid_period()) || (missing_tooth_valid == 1))
				{
					tooth_period_store();

					increment_teeth_counter();

					if(missing_tooth_valid == 1)
					{
						missing_tooth_valid = 0;
					}

					if(teeth_counter == NUMBER_OF_TEETH - 2)
					{
						missing_tooth_interupt_enable(1);
						engine_speed_output_pulse_start();
					}
					else if(teeth_counter == NUMBER_OF_TEETH_4C - 2)
					{
						missing_tooth_interupt_enable(1);
						engine_speed_output_pulse_start();
					}
					if(teeth_counter == 0)
					{
						missing_tooth_interupt_enable(0);
					}

					angle_clock_rate = current_period / DEGREES_PER_TOOTH;
					spark_control_task();
					fuel_control_task();

					period_timeout_set();
				}
				else
				{
#if(TEETH_LED_DEBUG == 1)
					led_toggle(LED_G);
#endif
					crank_goto_first_edge();
				}
			}
			break;
			default:
				break;
		}
	}
	if(FTM_GetChannelFlag(FTM2, FTM_CHANNEL_CHANNEL5) && (FTM2_C5SC & FTM_CnSC_CHIE_MASK))
	{
#if(TEETH_LED_DEBUG == 1)
		led_off(LED_B);
#endif
		FTM_ClrChannelFlag(FTM2, FTM_CHANNEL_CHANNEL5);
		crank_goto_first_edge();
	}
	if(FTM_GetChannelFlag(FTM2, FTM_CHANNEL_CHANNEL0) && (FTM2_C0SC & FTM_CnSC_CHIE_MASK))
	{
		FTM_ClrChannelFlag(FTM2, FTM_CHANNEL_CHANNEL0);
		fuel_control_isr();
	}
	if(FTM_GetChannelFlag(FTM2, FTM_CHANNEL_CHANNEL1) && (FTM2_C1SC & FTM_CnSC_CHIE_MASK))
	{
		FTM_ClrChannelFlag(FTM2, FTM_CHANNEL_CHANNEL1);
		spark_control_isr();
	}
	if(FTM_GetOverFlowFlag(FTM2) && (FTM2->SC & FTM_SC_TOIE_MASK))
	{
//#if(TEETH_LED_DEBUG == 1)
//		led_off(LED_B);
//#endif
		FTM_ClrOverFlowFlag(FTM2);
//		crank_goto_first_edge();
	}

}
