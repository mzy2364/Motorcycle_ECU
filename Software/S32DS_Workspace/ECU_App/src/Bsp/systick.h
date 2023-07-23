/*
 * systick.h
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"

void systick_init(void);
void systick_deinit(void);
uint32_t systick_get_current_tick(void);
void systick_delay(uint32_t ms);
uint32_t systick_time_diff(uint32_t tick);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTICK_H_ */
