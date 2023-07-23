/*
 * Crank_Sensing.h
 *
 *  Created on: 2023年3月4日
 *      Author: mzy2364
 */

#ifndef _CRANK_SENSING_H_
#define _CRANK_SENSING_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "Application_Define.h"

#define TEETH_LED_DEBUG		1

/* 定时器频率  20M/16 */
#define TIMER_FREQ		1250000

/* 定义发动机转速最大值和最小值的齿间距对应的定时器cnt值 */
#define MAX_PERIOD_LIMIT    (uint16_t)(60 * (TIMER_FREQ / RPM_MIN) / NUMBER_OF_TEETH)
#define MIN_PERIOD_LIMIT    (uint16_t)(60 * (TIMER_FREQ / RPM_MAX) / NUMBER_OF_TEETH)

#define PERIOD_TO_RPM(period)		(uint16_t)(TIMER_FREQ / ((period * NUMBER_OF_TEETH) / 60))


/* Minimum and maximum values of the gap ratio
   are based on tooth tolerance specified */
//间隙比的最小值和最大值基于指定的齿公差
#define MIN_GAP_RATIO  16 * GAP_RATIO - (16 * NEG_PERIOD_PERCENTAGE) / 100
#define MAX_GAP_RATIO  16 * GAP_RATIO + (16 * POS_PERIOD_PERCENTAGE) / 100

/* States definition */
typedef enum
{
	TOOTH_INIT = 0,
	FIRST_EDGE,
	FIRST_PERIOD,
	TESTING_FOR_GAP,
	VALIDATING_GAP,
	SYNCHRONIZED,
	SYNCHRONIZED_4C
}crank_state_t;

extern crank_state_t crank_state;
extern uint32_t angle_clock_rate;

void crank_sensing_init(void);
void crank_goto_first_edge(void);

#ifdef __cplusplus
}
#endif

#endif /* CRANK_SENSING_CRANK_SENSING_H_ */
