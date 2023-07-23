/*
 * VBAT.h
 *
 *  Created on: 2023年3月2日
 *      Author: mzy2364
 */

#ifndef ANALOG_MONITORING_VBAT_H_
#define ANALOG_MONITORING_VBAT_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

void vbat_monitoring(void);
uint16_t vbat_get_mv(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_MONITORING_VBAT_H_ */
