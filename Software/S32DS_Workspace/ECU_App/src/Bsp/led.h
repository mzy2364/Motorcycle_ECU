/*
 * led.h
 *
 *  Created on: 2023年2月26日
 *      Author: mzy2364
 */

#ifndef _LED_H_
#define _LED_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "gpio.h"

#define LEDR_PORT	PTD
#define LEDG_PORT	PTD
#define LEDB_PORT	PTC

#define LEDR_PIN	PTD6
#define LEDG_PIN	PTD7
#define LEDB_PIN	PTC2

typedef enum{
	LED_R = 0,
	LED_G,
	LED_B,
}led_ch_t;

void led_init(void);
void led_on(led_ch_t led);
void led_off(led_ch_t led);
void led_toggle(led_ch_t led);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_H_ */
