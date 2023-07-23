/*
 * adc.h
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

#ifndef BSP_ADC_H_
#define BSP_ADC_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"

#define ADC_VREFH       				5000U
#define ADC_VREFL       				0U
#define IOA_CONV_ADC_TO_VOLT(adc)		((uint32_t)adc)*(ADC_VREFH - ADC_VREFL)/(uint16_t)(1 << 12)

#define ADC_CH_NUM  7

#define ADC_CH0    0x01
#define ADC_CH1    0x02
#define ADC_CH2    0x04
#define ADC_CH3    0x08
#define ADC_CH4    0x10
#define ADC_CH5    0x20
#define ADC_CH6    0x40
#define ADC_CH7    0x80
#define ADC_CH8    0x100
#define ADC_CH9    0x200
#define ADC_CH10   0x400
#define ADC_CH11   0x800
#define ADC_CH12   0x1000
#define ADC_CH13   0x2000
#define ADC_CH14   0x4000
#define ADC_CH15   0x8000

#define PIN_CONTROL_NUM (ADC_CH1 | ADC_CH2 | ADC_CH3 | ADC_CH4 | ADC_CH5 | ADC_CH12 | ADC_CH13)


typedef enum{
	VBAT_ADC_CHANNEL = 0,
	MAP_ADC_CHANNEL,
	O2IN_ADC_CHANNEL,
	MAF_ADC_CHANNEL,
	ATEMP_ADC_CHANNEL,
	TPS_ADC_CHANNEL,
	ETEMP_ADC_CHANNEL,
}adc_channel_t;

void adc_init(void);
void adc_deinit(void);
uint32_t adc_read_channel_mv(uint8_t ch);
uint16_t adc_read_channel(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H_ */
