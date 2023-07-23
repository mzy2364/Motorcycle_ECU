/*
 * adc.c
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "bsp_adc.h"
#include "adc.h"

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
static uint16_t adc_buf[ADC_CH_NUM];

/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void adc_isr(void);

/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief ADC初始化
  * @param void
  * @retval	void
  * @note
  */
void adc_init(void)
{
	ADC_ConfigType  ADC_Config={{0}};

	/* Initialization of ADC module */
	ADC_Config.u8ClockDiv = ADC_ADIV_DIVIDE_4;
	ADC_Config.u8ClockSource = CLOCK_SOURCE_BUS_CLOCK;
	ADC_Config.u8Mode = ADC_MODE_12BIT;
	ADC_Config.sSetting.bIntEn = 1;
	ADC_Config.sSetting.bContinuousEn = 1;
//	ADC_Config.sSetting.bFiFoScanModeEn = 1;
	ADC_Config.u8FiFoLevel = ADC_FIFO_LEVEL7;
	ADC_Config.u16PinControl= PIN_CONTROL_NUM;

	ADC_SetCallBack(adc_isr);
	ADC_Init(ADC, &ADC_Config);

	ADC_SetChannel(ADC,ADC_CHANNEL_AD1);
	ADC_SetChannel(ADC,ADC_CHANNEL_AD2);
	ADC_SetChannel(ADC,ADC_CHANNEL_AD3);
	ADC_SetChannel(ADC,ADC_CHANNEL_AD4);
	ADC_SetChannel(ADC,ADC_CHANNEL_AD5);
	ADC_SetChannel(ADC,ADC_CHANNEL_AD12);
	ADC_SetChannel(ADC,ADC_CHANNEL_AD13);
}

/**
  * @brief ADC反初始化
  * @param void
  * @retval	void
  * @note
  */
void adc_deinit(void)
{
	ADC_DeInit(ADC);
}

/**
  * @brief ADC读取对应通道的mv值
  * @param void
  * @retval	void
  * @note
  */
uint32_t adc_read_channel_mv(uint8_t ch)
{
	if(ch < ADC_CH_NUM)
	{
		uint32_t ad_mv = 0;
		ad_mv = IOA_CONV_ADC_TO_VOLT((uint16_t)adc_buf[ch]);
		return (uint32_t)ad_mv;
	}
	return 0;
}

/**
  * @brief ADC读取对应通道的转换值
  * @param void
  * @retval	void
  * @note
  */
uint16_t adc_read_channel(uint8_t ch)
{
	if(ch < ADC_CH_NUM)
	{
		return adc_buf[ch];
	}
	return 0;
}

/**
  * @brief ADC中断回调函数
  * @param void
  * @retval	void
  * @note
  */
static void adc_isr(void)
{
	adc_buf[0] = ADC_ReadResultReg(ADC);
	adc_buf[1] = ADC_ReadResultReg(ADC);
	adc_buf[2] = ADC_ReadResultReg(ADC);
	adc_buf[3] = ADC_ReadResultReg(ADC);
	adc_buf[4] = ADC_ReadResultReg(ADC);
	adc_buf[5] = ADC_ReadResultReg(ADC);
	adc_buf[6] = ADC_ReadResultReg(ADC);
}

