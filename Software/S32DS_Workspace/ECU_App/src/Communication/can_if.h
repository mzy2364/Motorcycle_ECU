/*
 * can_if.h
 *
 *  Created on: 2023年3月29日
 *      Author: mzy2364
 */

#ifndef COMMUNICATION_CAN_IF_H_
#define COMMUNICATION_CAN_IF_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "can.h"

void can_if_init(void);
void can_if_task(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_CAN_IF_H_ */
