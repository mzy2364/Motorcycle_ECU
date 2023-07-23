/*
 * systick.c
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

/*******************************************************************************
* include files
*******************************************************************************/
#include "systick.h"

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
static volatile uint32_t tick_cnt = 0u;

/*******************************************************************************
* Local Functions prototypes
*******************************************************************************/


/*******************************************************************************
*  Global Functions Body
*******************************************************************************/

/**
  * @brief 滴答定时器初始化
  * @param void
  * @retval	void
  * @note
  */
void systick_init(void)
{
	SysTick_Config(40000000/1000);
}

/**
  * @brief 滴答定时器反初始化
  * @param void
  * @retval	void
  * @note
  */
void systick_deinit(void)
{
	NVIC_DisableIRQ(SysTick_IRQn);
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief 获取当前时间
  * @param void
  * @retval	返回当前时间ֵ
  * @note
  */
uint32_t systick_get_current_tick(void)
{
	return tick_cnt;
}

/**
  * @brief 滴答定时器低精度阻塞延时函数
  * @param ms-需要延时的ms数
  * @retval	Noneֵ
  * @note 精度1ms
  */
void systick_delay(uint32_t ms)
{
	uint32_t last_tick = systick_get_current_tick();
	uint32_t current_tick = systick_get_current_tick();
	uint32_t delta = 0;
	for(;;)
	{
		current_tick = systick_get_current_tick();
		delta = (current_tick >= last_tick)?(current_tick - last_tick):(0xFFFFFFFF - last_tick + current_tick);
		if(delta >= ms)
			return;
	}
}

/**
  * @brief 返回系统时钟跟输入参数的差值
  * @param tick:需要比较的时基数
  * @retval 返回差值
  * @note
  */
uint32_t systick_time_diff(uint32_t tick)
{
	uint32_t delta = 0;
	uint32_t current_tick = systick_get_current_tick();
	delta = (current_tick >= tick)?(current_tick - tick):(0xFFFFFFFF - tick + current_tick);
	return delta;

}

/**
* @brief 滴答定时器中断服务函数
*/
void SysTick_Handler(void)
{
	tick_cnt++;
}
