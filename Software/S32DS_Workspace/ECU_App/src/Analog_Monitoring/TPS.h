/*
 * TPS.h
 *
 *  Created on: 2023年3月29日
 *      Author: mzy2364
 */

#ifndef _TPS_H_
#define _TPS_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "Application_Define.h"

void tps_init(void);
void tps_monitoring(void);
uint16_t tps_adc_get(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_MONITORING_TPS_H_ */
