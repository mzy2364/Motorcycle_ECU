/*
 * Crankshaft_Simulator.h
 *
 *  Created on: 2023年3月17日
 *      Author: mzy2364
 */

#ifndef CRANKSHAFT_SIMULATOR_CRANKSHAFT_SIMULATOR_H_
#define CRANKSHAFT_SIMULATOR_CRANKSHAFT_SIMULATOR_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"

#define TOOTH_NUM 48

#define MAX_PIT_VAL	160000
#define MIN_PIT_VAL	6249

void crankshaft_simulator_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CRANKSHAFT_SIMULATOR_CRANKSHAFT_SIMULATOR_H_ */
