/*
 * MAP.h
 *
 *  Created on: 2023年3月18日
 *      Author: mzy2364
 */

#ifndef ANALOG_MONITORING_MAP_H_
#define ANALOG_MONITORING_MAP_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

extern uint16_t map_data_buffer[MAP_BUFFER_SIZE];

void map_init(void);
void map_monitoring(void);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_MONITORING_MAP_H_ */
