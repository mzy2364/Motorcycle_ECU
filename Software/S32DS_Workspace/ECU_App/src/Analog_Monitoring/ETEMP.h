/*
 * ETEMP.h
 *
 *  Created on: 2023年3月29日
 *      Author: mzy2364
 */

#ifndef _ETEMP_H_
#define _ETEMP_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "Application_Define.h"

void etemp_init(void);
void etemp_monitoring(void);
uint16_t etemp_adc_get(void);
int16_t etemp_get_temperature(void);
uint8_t etemp_get_temp_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_MONITORING_TPS_H_ */
