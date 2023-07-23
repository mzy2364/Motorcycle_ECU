/*
 * Spark_Control.h
 *
 *  Created on: 2023年3月8日
 *      Author: mzy2364
 */

#ifndef _SPARK_CONTROL_H_
#define _SPARK_CONTROL_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "Application_Define.h"

#define SPARK_FTM_CHANNEL	FTM_CHANNEL_CHANNEL1
#define SET_SPARK_ON_NEXT_STATE()	FTM2->CONTROLS[SPARK_FTM_CHANNEL].CnSC = (FTM_CnSC_MSA_MASK | (FTM_OUTPUT_SET << 2));
#define CLR_SPARK_ON_NEXT_STATE()	FTM2->CONTROLS[SPARK_FTM_CHANNEL].CnSC = (FTM_CnSC_MSA_MASK | (FTM_OUTPUT_CLEAR << 2));

/* Select dwell to be angle or fixed time */
//#define DWELL_ANGLE        130      /* Dwell angle in degrees */
#define DWELL_TIME_MS       6      /* Dwell time in miliseconds */
#define SOLENOID_DELAY_MS   0      /* Solenoid delay in ms */
#ifdef  DWELL_TIME_MS
#define DWELL_TIME      DWELL_TIME_MS * 1250 /* Dwell time converted to timer cycles */
#endif

/* Definition of spark controller states */
typedef enum
{
	SPARK_CTRL_INACTIVE,
	READY_TO_SCHEDULE_SPARK,
	WAITING_STARTING_CURRENT_FLOW,
	WAITING_IGNITION_TIME
}spark_state_t;

extern uint8_t current_spark_dwell_tooth;

void spark_control_init(void);
void spark_control_task(void);
void spark_control_isr(void);
uint16_t spark_map_read(uint16_t rpm_input,uint16_t load_input);

#ifdef __cplusplus
}
#endif

#endif /* SPARK_CONTROL_SPARK_CONTROL_H_ */
