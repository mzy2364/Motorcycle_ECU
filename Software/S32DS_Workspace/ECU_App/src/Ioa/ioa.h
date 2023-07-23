/*
 * ioa.h
 *
 *  Created on: 2023年6月19日
 *      Author: mzy2364
 */

#ifndef IOA_IOA_H_
#define IOA_IOA_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"

#define DI_CH_NUM		4
#define DI_FILTER_TIME   5

extern uint8_t di_ign_on;
extern uint8_t di_hs_01;
extern uint8_t di_ls_01;
extern uint8_t di_ls_02;

void ioa_task(void);

#ifdef __cplusplus
}
#endif

#endif /* IOA_IOA_H_ */
