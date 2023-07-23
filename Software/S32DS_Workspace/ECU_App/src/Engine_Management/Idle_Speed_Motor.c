/*
 * Idle_Speed_Motor.c
 *
 *  Created on: 2023年3月21日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "Idle_Speed_Motor.h"
#include "ftm.h"
#include "gpio.h"
/*******************************************************************************
* Defines                                                                
*******************************************************************************/

/*******************************************************************************
* Macros                                                                
*******************************************************************************/

/*******************************************************************************
* Global Constant definition                         
*******************************************************************************/

/*******************************************************************************
* Local Constant definition                         
*******************************************************************************/

/*******************************************************************************
* Global Variables definition                         
*******************************************************************************/

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
static uint8_t motor_state = 0;
motor_state_t ism_state = MOTOR_STOPPED;
uint32_t motor_angle = 0;
uint32_t idle_motor_position = 0;
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void ftm0_isr(void);

/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void idle_speed_motor_init(void)
{
//	FTM_ConfigType FTM0_Config = {0};
//	FTM_ChParamsType FTM0CH1_Config = {0};
//	FTM_ChParamsType FTM0CH0_Config = {0};
//
//	SIM->PINSEL |= SIM_PINSEL_FTM0PS1_MASK;	//FTM0_CH1 通道映射到 PTB3 上
//	SIM->PINSEL |= SIM_PINSEL_FTM0PS0_MASK;	//FTM0_CH0 通道映射到 PTB2 上
//
//	FTM0_Config.modulo = 9999;
//	FTM0_Config.combine = FTM_COMBINE_COMBINE0_MASK|FTM_COMBINE_SYNCEN0_MASK;
//	FTM0_Config.clk_source = FTM_CLOCK_SYSTEMCLOCK;
//	FTM0_Config.prescaler = FTM_CLOCK_PS_DIV1;
//	//FTM0_Config.mode = 1;
//	FTM0_Config.toie = 1;
//
//
//	FTM0CH0_Config.ctrl.bits.bMode = FTM_PWMMODE_COMBINE;
//	FTM0CH0_Config.ctrl.bits.bPWMPol = FTM_PWM_HIGHTRUEPULSE;
//	FTM0CH0_Config.u16CnV = 5000;
//
//
//	FTM0CH1_Config.ctrl.bits.bMode = FTM_PWMMODE_COMBINE;
//	FTM0CH1_Config.ctrl.bits.bPWMPol = FTM_PWM_HIGHTRUEPULSE;
//	FTM0CH1_Config.u16CnV = 5000;
//
//	FTM_SetCallback(FTM0, ftm0_isr);
//	FTM_ChannelInit(FTM0,0,FTM0CH0_Config);
//	FTM_ChannelInit(FTM0,1,FTM0CH1_Config);
//	FTM_Init(FTM0,&FTM0_Config);   /* Generate PWM signal */

	SIM_SOPT0 &= ~SIM_SOPT0_NMIE_MASK;
	CONFIG_PIN_AS_GPIO(MOTOR_IN1_PORT,MOTOR_IN1_PIN,OUTPUT);
	CONFIG_PIN_AS_GPIO(MOTOR_IN2_PORT,MOTOR_IN2_PIN,OUTPUT);
	CONFIG_PIN_AS_GPIO(MOTOR_IN3_PORT,MOTOR_IN3_PIN,OUTPUT);
	CONFIG_PIN_AS_GPIO(MOTOR_IN4_PORT,MOTOR_IN4_PIN,OUTPUT);

//	CONFIG_PIN_AS_GPIO(MOTOR_EF1_PORT,MOTOR_EF1_PIN,INPUT);
//	CONFIG_PIN_AS_GPIO(MOTOR_EF2_PORT,MOTOR_EF2_PIN,INPUT);

	CONFIG_PIN_AS_GPIO(MOTOR_INH_PORT,MOTOR_INH_PIN,OUTPUT);
	OUTPUT_SET(MOTOR_INH_PORT,MOTOR_INH_PIN);
}

void A_negative_current(void)
{
	OUTPUT_CLEAR(MOTOR_IN2_PORT,MOTOR_IN2_PIN);
	OUTPUT_CLEAR(MOTOR_IN1_PORT,MOTOR_IN1_PIN);
}

void A_positive_current(void)
{
	OUTPUT_SET(MOTOR_IN2_PORT,MOTOR_IN2_PIN);
	OUTPUT_SET(MOTOR_IN1_PORT,MOTOR_IN1_PIN);
}

void A_no_current(void)
{
	OUTPUT_SET(MOTOR_IN2_PORT,MOTOR_IN2_PIN);
	OUTPUT_CLEAR(MOTOR_IN1_PORT,MOTOR_IN1_PIN);
}

void B_negative_current(void)
{
	OUTPUT_CLEAR(MOTOR_IN4_PORT,MOTOR_IN4_PIN);
	OUTPUT_CLEAR(MOTOR_IN3_PORT,MOTOR_IN3_PIN);
}

void B_positive_current(void)
{
	OUTPUT_SET(MOTOR_IN4_PORT,MOTOR_IN4_PIN);
	OUTPUT_SET(MOTOR_IN3_PORT,MOTOR_IN3_PIN);
}

void B_no_current(void)
{
	OUTPUT_SET(MOTOR_IN4_PORT,MOTOR_IN4_PIN);
	OUTPUT_CLEAR(MOTOR_IN3_PORT,MOTOR_IN3_PIN);
}

/**
  * @brief 步进电机向前运动
  * @param void
  * @retval	void
  * @note
  */
void idle_speed_motor_forward(void)
{
	switch(motor_state)
	{
	case 0:
		B_positive_current();
		A_no_current();
		motor_state++;
		break;
	case 1:
		B_positive_current();
		A_positive_current();
		motor_state++;
		break;
	case 2:
		B_no_current();
		A_positive_current();
		motor_state++;
		break;
	case 3:
		B_negative_current();
		A_positive_current();
		motor_state++;
		break;
	case 4:
		B_negative_current();
		A_no_current();
		motor_state++;
		break;
	case 5:
		B_negative_current();
		A_negative_current();
		motor_state++;
		break;
	case 6:
		B_no_current();
		A_negative_current();
		motor_state++;
		break;
	case 7:
		B_positive_current();
		A_negative_current();
		motor_state = 0;
		idle_motor_position++;
		break;
	default:
		break;
	}
}

/**
  * @brief 步进电机向后运动
  * @param void
  * @retval	void
  * @note
  */
void idle_speed_motor_backward(void)
{
	switch(motor_state)
	{
	case 0:
		A_positive_current();
		B_no_current();
		motor_state++;
		break;
	case 1:
		A_positive_current();
		B_positive_current();
		motor_state++;
		break;
	case 2:
		A_no_current();
		B_positive_current();
		motor_state++;
		break;
	case 3:
		A_negative_current();
		B_positive_current();
		motor_state++;
		break;
	case 4:
		A_negative_current();
		B_no_current();
		motor_state++;
		break;
	case 5:
		A_negative_current();
		B_negative_current();
		motor_state++;
		break;
	case 6:
		A_no_current();
		B_negative_current();
		motor_state++;
		break;
	case 7:
		A_positive_current();
		B_negative_current();
		motor_state = 0;
		idle_motor_position--;
		break;
	default:
		break;
	}
}



void motor_task(void)
{
	//if(idle_motor_position < 10)
	{
		idle_speed_motor_forward();
	}
//	if(ism_state != MOTOR_STOPPED)
//	{
//		if(ism_state == MOTOR_OPENING)
//		{
//			idle_speed_motor_forward();
//			if(idle_motor_position == 0)
//			{
//				ism_state = MOTOR_STOPPED;
//			}
//		}
//		else if(ism_state == MOTOR_CLOSING)
//		{
//			idle_speed_motor_backward();
//			if(idle_motor_position >= 10)
//			{
//				ism_state = MOTOR_STOPPED;
//			}
//		}
//	}
}

/**
  * @brief FTM0中断回调函数
  * @param void
  * @retval	void
  * @note
  */
static void ftm0_isr(void)
{
	if(FTM_GetOverFlowFlag(FTM0) && (FTM0->SC & FTM_SC_TOIE_MASK))
	{
		FTM_ClrOverFlowFlag(FTM0);
	}
}
