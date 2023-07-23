/*
 * TLE8080EM.h
 *
 *  Created on: 2023年3月19日
 *      Author: mzy2364
 */

#ifndef _TLE8080EM_H_
#define _TLE8080EM_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "Application_Define.h"

void tle8080em_init(void);
void tle8080em_task(void);
void valve_ctrl(uint8_t en);
void mil_ctrl(uint8_t en);
void relay1_ctrl(uint8_t en);
void relay2_ctrl(uint8_t en);
uint16_t tle8080em_get_diag_reg(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_TLE8080EM_H_ */
