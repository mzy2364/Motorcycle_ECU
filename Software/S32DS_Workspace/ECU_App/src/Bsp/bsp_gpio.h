/*
 * gpio.h
 *
 *  Created on: 2023年3月2日
 *      Author: mzy2364
 */

#ifndef BSP_GPIO_H_
#define BSP_GPIO_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "gpio.h"

#define VALVE_PORT	PTA
#define VALVE_PIN	PTA0

#define DI_HS_IGN_ON_PORT	PTF
#define DI_HS_IGN_ON_PIN	PTF7

#define DI_HS_02_PORT	PTF
#define DI_HS_02_PIN	PTF6

#define DI_LS_03_PORT	PTC
#define DI_LS_03_PIN	PTC0

#define DI_LS_04_PORT	PTD
#define DI_LS_04_PIN	PTD5

enum{
	DI_IGN = 0,
	DI_HS_01,
	DI_LS_01,
	DI_LS_02
};

void gpio_init(void);
uint8_t gpio_read(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H_ */
