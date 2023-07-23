/*
 * bsp_spi.c
 *
 *  Created on: 2023年3月3日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "bsp_spi.h"
#include "spi.h"
#include "led.h"
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
		 
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 
  * @param void
  * @retval	void
  * @note
  */
void spi0_init(void)
{
	SPI_ConfigType spi0_config = {{0},0,0};

	SIM_PINSEL0 |= SIM_PINSEL_SPI0PS_MASK;  // SPI on PTE

	CONFIG_PIN_AS_GPIO(PTE,PTE3,OUTPUT);

	spi0_config.sSettings.bModuleEn = 1;
	spi0_config.sSettings.bMasterMode = 1;
	spi0_config.sSettings.bClkPolarityLow = 0;
	spi0_config.sSettings.bClkPhase1 = 1;
	spi0_config.sSettings.bMasterAutoDriveSS = 0;

	spi0_config.u32BitRate = 2000000;
	spi0_config.u32BusClkHz = 20000000;

	SPI_Init(SPI0, &spi0_config);
}

/**
  * @brief SPI发送并且读取半字数据
  * @param 	data:待发送数据
  * 		recv:读取到的数据存放缓冲区
  * @retval	0:读取正常   1:SPI读取失败
  * @note
  */
uint8_t spi0_transmit_receive_halfword(uint16_t data,uint16_t *recv)
{
	uint8_t ret = 1;
	uint8_t send_buf[2] = {data>>8,data};
	uint8_t read_buf[2] = {0};

	OUTPUT_CLEAR(PTE,PTE3);
    ret = SPI_TransferWait(SPI0, read_buf, send_buf,2);
    OUTPUT_SET(PTE,PTE3);

    *recv = read_buf[1];
    *recv |= (uint32_t)read_buf[0] << 8;

    return ret;
}

