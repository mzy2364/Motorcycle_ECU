/*
 * TLE8080EM.c
 *
 *  Created on: 2023年3月19日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "TLE8080EM.h"
#include "bsp_spi.h"
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

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
static uint16_t cmd_reg_data = 0X8000;
static uint16_t diag_reg_data = 0;
uint8_t threshold_register = 0X03;
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
		 
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief TLE8080EM初始化
  * @param void
  * @retval	void
  * @note
  */
void tle8080em_init(void)
{
	spi0_init();
}

/**
  * @brief TLE8080EM周期调度任务
  * @param void
  * @retval	void
  * @note
  */
void tle8080em_task(void)
{
	uint16_t data = 0;
	uint16_t diag_reg_read = 0X2000;
	cmd_reg_data &= ~(0X03 << 11);
	cmd_reg_data |= threshold_register << 11;
	spi0_transmit_receive_halfword(diag_reg_read,&data);
	spi0_transmit_receive_halfword(cmd_reg_data,&data);
	diag_reg_data = data;
}

/**
  * @brief 获取TLE8080EM诊断寄存器的值
  * @param void
  * @retval	诊断寄存器的原始值
  * @note
  */
uint16_t tle8080em_get_diag_reg(void)
{
	return diag_reg_data;
}

/**
  * @brief 电磁阀控制
  * @param void
  * @retval	void
  * @note
  */
void valve_ctrl(uint8_t en)
{
	if(en)
		OUTPUT_SET(VALVE_PORT,VALVE_PIN);
	else
		OUTPUT_CLEAR(VALVE_PORT,VALVE_PIN);
}

/**
  * @brief MIL灯控制
  * @param void
  * @retval	void
  * @note
  */
void mil_ctrl(uint8_t en)
{
	if(en)
		cmd_reg_data |= (1<<0);
	else
		cmd_reg_data &= ~(1<<0);
}

/**
  * @brief 继电器1控制
  * @param void
  * @retval	void
  * @note
  */
void relay1_ctrl(uint8_t en)
{
	if(en)
		cmd_reg_data |= (1<<1);
	else
		cmd_reg_data &= ~(1<<1);
}

/**
  * @brief 继电器2控制
  * @param void
  * @retval	void
  * @note
  */
void relay2_ctrl(uint8_t en)
{
	if(en)
		cmd_reg_data |= (1<<2);
	else
		cmd_reg_data &= ~(1<<2);
}
