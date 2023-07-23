/*
 * User_Management.h
 *
 *  Created on: 2023年3月31日
 *      Author: mzy2364
 */

#ifndef USER_MANAGEMENT_H_
#define USER_MANAGEMENT_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "Application_Define.h"

//Timeout for fuel pump to shut off if engine does not start.
//Units are executions of User Management Task which is 10ms.
//Timeout is 3 seconds.
#define FUEL_PUMP_TOUT  300
#define ENGUNE_STARTUP_TOUT	200

#define FUEL_PUMP_CTRL(x)	relay1_ctrl(x)

typedef enum{
	INIT = 0,
	STOP,
	START,
	RUN_STARTUP,
	RUN,
	OVERRUN
}app_state_t;

extern uint8_t cal_fuel_pump_oe;
extern uint8_t cal_fuel_pump_ov;
extern uint8_t cal_mil_oe;
extern uint8_t cal_mil_ov;
extern uint8_t cal_inj_oe;
extern uint8_t cal_inj_ov;
extern uint8_t cal_ignition_oe;
extern uint8_t cal_ignition_ov;
extern app_state_t app_state;


void user_management_init(void);
void user_management(void);


#ifdef __cplusplus
}
#endif

#endif /* USER_MANAGEMENT_H_ */
