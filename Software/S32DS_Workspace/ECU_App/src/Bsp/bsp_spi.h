/*
 * bsp_spi.h
 *
 *  Created on: 2023年3月3日
 *      Author: mzy2364
 */

#ifndef BSP_BSP_SPI_H_
#define BSP_BSP_SPI_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "derivative.h"


void spi0_init(void);
uint8_t spi0_transmit_receive_halfword(uint16_t data,uint16_t *recv);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BSP_SPI_H_ */
