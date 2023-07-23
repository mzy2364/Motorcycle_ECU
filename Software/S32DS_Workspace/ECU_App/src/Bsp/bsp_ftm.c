/*
 * bsp_ftm.c
 *
 *  Created on: 2023年3月3日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "bsp_ftm.h"
#include "ftm.h"

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

/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void ftm2_isr(void);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void ftm_pwm_init(void)
{
	FTM_ConfigType FTM2_Config={0};
	FTM_ChParamsType FTM2CH1_Config={0};

	FTM2_Config.modulo=9999;
	FTM2_Config.clk_source=FTM_CLOCK_SYSTEMCLOCK;
	FTM2_Config.prescaler=FTM_CLOCK_PS_DIV1;
	FTM2_Config.mode=1;
	FTM2_Config.toie=1;

	FTM2CH1_Config.ctrl.bits.bMode=FTM_PWMMODE_EDGEALLIGNED;
	FTM2CH1_Config.ctrl.bits.bPWMPol=FTM_PWM_HIGHTRUEPULSE;
	FTM2CH1_Config.u16CnV=1000;

	FTM_SetCallback(FTM2, ftm2_isr);
	FTM_ChannelInit(FTM2,1,FTM2CH1_Config);
	FTM_Init(FTM2,&FTM2_Config);
}


static void ftm2_isr(void)
{

}