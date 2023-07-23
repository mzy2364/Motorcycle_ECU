/*
 * Fuel_Control.h
 *
 *  Created on: 2023年3月2日
 *      Author: mzy2364
 */

#ifndef _FUEL_CONTROL_H_
#define _FUEL_CONTROL_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "Application_Define.h"

#define FUEL_FTM_CHANNEL	FTM_CHANNEL_CHANNEL0
#define SET_FUEL_ON_NEXT_STATE()	FTM2->CONTROLS[FUEL_FTM_CHANNEL].CnSC = (FTM_CnSC_MSA_MASK | (FTM_OUTPUT_SET << 2));
#define CLR_FUEL_ON_NEXT_STATE()	FTM2->CONTROLS[FUEL_FTM_CHANNEL].CnSC = (FTM_CnSC_MSA_MASK | (FTM_OUTPUT_CLEAR << 2));

/* Definition of fuel controller states */
typedef enum
{
	FUEL_CTRL_INACTIVE,
	FUEL_CTRL_READY,
	FUEL_PULSE_SCHEDULED,
	FUEL_PULSE_ON
}fuel_state_t;

extern uint16_t current_fuel_pulse;

void fuel_control_init(void);
void fuel_control_task(void);
void fuel_control_isr(void);
uint16_t fuel_pulse_angle_map_read(uint16_t rpm_input,uint16_t load_input);
uint32_t fuel_pulse_width_map_read(uint16_t rpm_input,uint16_t load_input);

uint16_t map_bin_search(uint16_t target, uint16_t *array, uint16_t array_length);

#ifdef __cplusplus
}
#endif

#endif /* FUEL_CONTROL_FUEL_CONTROL_H_ */
