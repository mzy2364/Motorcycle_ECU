/*
 * Engine_Speed_Output.h
 *
 *  Created on: 2023年4月4日
 *      Author: mzy2364
 */

#ifndef ENGINE_MANAGEMENT_ENGINE_SPEED_OUTPUT_H_
#define ENGINE_MANAGEMENT_ENGINE_SPEED_OUTPUT_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"

#define ENG_SPD_OUT_PORT	PTE
#define ENG_SPD_OUT_PIN		PTE7

void engine_speed_output_init(void);
void engine_speed_output_pulse_start(void);
void engine_speed_output_pulse_end(void);

#ifdef __cplusplus
}
#endif

#endif /* ENGINE_MANAGEMENT_ENGINE_SPEED_OUTPUT_H_ */
