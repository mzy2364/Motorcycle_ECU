/*
 * Crankshaft_Simulator.c
 *
 *  Created on: 2023年3月17日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "Crankshaft_Simulator.h"
#include "gpio.h"
#include "pit.h"

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
//一圈24个齿
//转速300RPM，5转每秒，24*5齿每秒，每个齿1/120秒=8.3ms
//定时器20M 时钟周期 1/20M 周期8ms 定时器加载值160000
//转速6000RPM 100转每秒   24*100齿每秒，每个齿1/2400秒=416us
//定时器加载值 8333
//转速8000RPM 133转每秒   24*133齿每秒，每个齿1/3192秒=313us
//定时器加载值 6249
uint8_t gpio0_val_tab[TOOTH_NUM];
uint32_t pit1_flag_counter = 0;



/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
uint32_t pit_load_val = MAX_PIT_VAL;
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void pit_ch1_isr(void);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void crankshaft_simulator_init(void)
{
	uint32_t i = 0;

	PIT_ConfigType config = {0};
	config.bTimerEn = 1;
	config.bInterruptEn = 1;
	config.u32LoadValue = pit_load_val;

	PIT_Init(PIT_CHANNEL1, &config);
	PIT_SetCallback(PIT_CHANNEL1, pit_ch1_isr);

	CONFIG_PIN_AS_GPIO(PTB,PTB5,OUTPUT);

	for(i = 0; i < TOOTH_NUM; i++)
	{
		if((i%2) == 0)
		{
			gpio0_val_tab[i] = 0;
		}
		else
		{
			gpio0_val_tab[i] = 1;
		}
	}

	gpio0_val_tab[45] = 1;
	gpio0_val_tab[46] = 1;
	gpio0_val_tab[47] = 1;

}



void pit_ch1_isr(void) {
	if(pit1_flag_counter >= TOOTH_NUM)
		pit1_flag_counter = 0;

	if(gpio0_val_tab[pit1_flag_counter])
	{
		OUTPUT_CLEAR(PTB,PTB5);
//		OUTPUT_SET(PTB,PTB5);
	}
	else
	{
//		OUTPUT_CLEAR(PTB,PTB5);
		OUTPUT_SET(PTB,PTB5);
	}
	pit1_flag_counter++;
	PIT_ChannelClrFlags(PIT_CHANNEL1);
}
