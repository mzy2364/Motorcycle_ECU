/*
 * task.c
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

/*******************************************************************************
* include files
*******************************************************************************/
#include "task.h"
#include "systick.h"
#include "clocks.h"
#include "led.h"
#include "bsp_adc.h"
#include "bsp_ftm.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "can.h"
#include "ioa.h"

#include "TLE8080EM.h"
#include "VBAT.h"
#include "Crank_Sensing.h"
#include "Spark_Control.h"
#include "Fuel_Control.h"
#include "Crankshaft_Simulator.h"
#include "Engine_Management.h"
#include "Idle_Speed_Motor.h"
#include "can_if.h"
#include "User_Management.h"
#include "Engine_Speed_Output.h"
/*******************************************************************************
* Defines
*******************************************************************************/

/*******************************************************************************
* Macros
*******************************************************************************/
#define TICK_PRD_1MS 			1
#define TICK_PRD_5MS 			5
#define TICK_PRD_10MS 			10
#define TICK_PRD_100MS 			100
#define TICK_PRD_500MS 			500
#define TICK_PRD_1S 			1000

#define CalcPeriod(curr_tick, lastTick)	(curr_tick >= lastTick) ? \
										(curr_tick - lastTick) : \
									   (0xFFFFFFFF - lastTick + curr_tick)

#define IsPeriodReached(prd) 			(delta >= prd)

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
static uint32_t tick1ms = 0;
static uint32_t tick5ms = 0;
static uint32_t tick10ms = 0;
static uint32_t tick100ms = 0;
static uint32_t tick500ms = 0;
static uint32_t tick1s = 0;

/*******************************************************************************
* Local Functions prototypes
*******************************************************************************/
static void sch_task1ms(void);
static void sch_task5ms(void);
static void sch_task10ms(void);
static void sch_task100ms(void);
static void sch_task500ms(void);
static void sch_task1s(void);
static void sch_idle(void);

/*******************************************************************************
*  Global Functions Body
*******************************************************************************/

/**
  * @brief TASK初始化
  * @param void
  * @retval	void
  * @note
  */
void task_init(void)
{
	init_clks_FEE_40MHz();
	SystemCoreClockUpdate();
	systick_init();
	led_init();
	adc_init();
	gpio_init();
	//ftm_pwm_init();

	can_if_init();
	tle8080em_init();
	crank_sensing_init();
	spark_control_init();
	fuel_control_init();
	idle_speed_motor_init();

	user_management_init();
	engine_management_init();

	engine_speed_output_init();
}

/**
  * @brief 1ms任务
  * @param void
  * @retval	void
  * @note
  */
static void sch_task1ms(void)
{
	motor_task();
}

/**
  * @brief 5ms任务
  * @param void
  * @retval	void
  * @note
  */
static void sch_task5ms(void)
{

}

/**
  * @brief 10ms任务
  * @param void
  * @retval	void
  * @note
  */
static void sch_task10ms(void)
{
	ioa_task();
	engine_management();
	engine_management_10ms();
	vbat_monitoring();
	tle8080em_task();
	can_if_task();
	user_management();
}

/**
  * @brief 100ms任务
  * @param void
  * @retval	void
  * @note
  */
static void sch_task100ms(void)
{

}

/**
  * @brief 500ms任务
  * @param void
  * @retval	void
  * @note
  */
static void sch_task500ms(void)
{
	led_toggle(LED_R);
}

/**
  * @brief 1S任务
  * @param void
  * @retval	void
  * @note
  */
static void sch_task1s(void)
{

}

/**
  * @brief 1S任务
  * @param void
  * @retval	void
  * @note
  */
static void sch_idle(void)
{
	can_main_function_read();
	can_main_function_write();
}

/**
  * @brief TASK任务调度
  * @param void
  * @retval	void
  * @note
  */
void task_scheduler(void)
{
	uint8_t isIdle;
	uint32_t curr_tick, delta;

	isIdle = 1;
	curr_tick = systick_get_current_tick();

	delta = CalcPeriod(curr_tick, tick1ms);
	if (IsPeriodReached(TICK_PRD_1MS))
	{
		/* Reload Period Tick */
		tick1ms = curr_tick;

		sch_task1ms();

		isIdle = 0;
	}

	delta = CalcPeriod(curr_tick, tick5ms);
	if (IsPeriodReached(TICK_PRD_5MS))
	{
		/* Reload Period Tick */
		tick5ms = curr_tick;

		sch_task5ms();

		isIdle = 0;
	}

	delta = CalcPeriod(curr_tick, tick10ms);
	if (IsPeriodReached(TICK_PRD_10MS))
	{
		/* Reload Period Tick */
		tick10ms = curr_tick;

		sch_task10ms();

		isIdle = 0;
	}

	delta    = CalcPeriod(curr_tick, tick100ms);
	if (IsPeriodReached(TICK_PRD_100MS))
	{
		/* Reload Period Tick */
		tick100ms = curr_tick;

		sch_task100ms();

		isIdle = 0;
	}

	delta    = CalcPeriod(curr_tick, tick500ms);
	if (IsPeriodReached(TICK_PRD_500MS))
	{
		/* Reload Period Tick */
		tick500ms = curr_tick;

		sch_task500ms();

		isIdle = 0;
	}

	delta    = CalcPeriod(curr_tick, tick1s);
	if (IsPeriodReached(TICK_PRD_1S))
	{
		/* Reload Period Tick */
		tick1s = curr_tick;

		sch_task1s();

		isIdle = 0;
	}

	if (isIdle)
	{
		sch_idle();
	}
}
