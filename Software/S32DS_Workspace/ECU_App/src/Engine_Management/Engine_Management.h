/*
 * Engine_Management.h
 *
 *  Created on: 2023年3月6日
 *      Author: mzy2364
 */

#ifndef _ENGINE_MANAGEMENT_H_
#define _ENGINE_MANAGEMENT_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include "Application_Define.h"

#define FUEL_CORRECTION_INIT		32767
#define SPARK_CORRECTION_INIT		24

#define SPARK1_REF_ANGLE  TDC_ANGLE

typedef enum{
	Invalid = 0,
	Valid,
}valid_t;

extern uint32_t engine_speed;
extern uint32_t load;
extern uint8_t mil_status;
extern uint8_t load_valid;
extern uint16_t cal_fuel_correction;
extern uint8_t cal_spark_angle_correction;
extern uint16_t startup_fuel_add;
extern uint16_t fuel_add;
extern uint16_t fuel_cut;
extern uint16_t spark_advance;
extern uint16_t spark_retard;
extern uint16_t cal_spark_angle;

void engine_management_init(void);
void engine_management(void);
void engine_management_10ms(void);
void engine_speed_calculate(void);
void load_calculate(void);

void fuel_start(uint16_t rpm,uint16_t load);
void fuel_start_calculate(uint16_t rpm,uint16_t load,uint16_t ref_angle);
void fuel_start_cal(uint16_t rpm,uint16_t load,uint16_t ref_angle);
void fuel_pulse(uint16_t rpm,uint16_t load);
uint32_t fuel_pulse_width_calculate(uint16_t rpm,uint16_t load);

void spark_start(uint16_t rpm,uint16_t load);
uint16_t spark_start_calculate(uint16_t rpm,uint16_t load,uint16_t ref_angle);
void spark_dwell(void);
void spark_dwell_calculate(uint16_t start_angle);

#ifdef __cplusplus
}
#endif

#endif /* ENGINE_MANAGEMENT_ENGINE_MANAGEMENT_H_ */
