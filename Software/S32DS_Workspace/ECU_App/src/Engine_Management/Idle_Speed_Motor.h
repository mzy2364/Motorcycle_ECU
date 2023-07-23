/*
 * Idle_Speed_Motor.h
 *
 *  Created on: 2023年3月21日
 *      Author: mzy2364
 */

#ifndef _IDLE_SPEED_MOTOR_H_
#define _IDLE_SPEED_MOTOR_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"
#include "Application_Define.h"

#define MOTOR_IN1_PORT	PTB
#define MOTOR_IN1_PIN	PTB2

#define MOTOR_IN2_PORT	PTE
#define MOTOR_IN2_PIN	PTE5

#define MOTOR_IN3_PORT	PTB
#define MOTOR_IN3_PIN	PTB3

#define MOTOR_IN4_PORT	PTB
#define MOTOR_IN4_PIN	PTB4

#define MOTOR_INH_PORT	PTC
#define MOTOR_INH_PIN	PTC3

#define MOTOR_EF1_PORT	PTB
#define MOTOR_EF1_PIN	PTB5

#define MOTOR_EF2_PORT	PTE
#define MOTOR_EF2_PIN	PTE6

//States of the idle speed motor
typedef enum{
	MOTOR_STOPPED = 0,
	MOTOR_CLOSING,
	MOTOR_OPENING,
}motor_state_t;

void idle_speed_motor_init(void);
void motor_task(void);
void idle_speed_motor_backward(void);
void idle_speed_motor_forward(void);

#ifdef __cplusplus
}
#endif

#endif /* ENGINE_MANAGEMENT_IDLE_SPEED_MOTOR_H_ */
