/**
  ******************************************************************************
  * @file    ram_fifo.h
  * @author  1Tianxia Embedded Software Team
  * @version V0.1.0
  * @date    28-07-2014
  * @brief   Header file of  module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 1Tianxia</center></h2>
  * <h2><center>&copy; All Rights Reserved</center></h2>
  *
  ******************************************************************************
  */

#ifndef __RAM_FIFO_H__
#define __RAM_FIFO_H__

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "macro_utility.h"

/* Exported types ------------------------------------------------------------*/

/* Exported macro-------------------------------------------------------------*/

/* Exported constants---------------------------------------------------------*/

/* Exported structures--------------------------------------------------------*/
struct _ramf_ctl {
  uint8_t *buffer;
  uint32_t page_size;
  uint32_t total_page_mask;
  volatile uint32_t rd;        //读
  volatile uint32_t wr;        //写
};

/* Exported variables---------------------------------------------------------*/
bool ramf_init(struct _ramf_ctl *ramf, uint8_t *buffer, uint32_t page_size, uint32_t total_page);
bool ramf_read(struct _ramf_ctl *ramf, uint8_t *dst);
bool ramf_write(struct _ramf_ctl *ramf, uint8_t *src);
bool ramf_write_lengh(struct _ramf_ctl *ramf, uint8_t *src, uint32_t lengh);
bool ramf_read_fresh(struct _ramf_ctl *ramf, uint8_t *dst);
bool ramf_peek_fresh(struct _ramf_ctl *ramf, uint8_t *dst);
bool ramf_peek_offset(struct _ramf_ctl *ramf, uint8_t *dst, uint32_t offset);
uint32_t ramf_nread(struct _ramf_ctl *ramf, uint8_t *dst, uint32_t num);
void ramf_trim(struct _ramf_ctl *ramf, uint32_t count);
void ramf_flush(struct _ramf_ctl *ramf);
uint32_t ramf_data_page_cnt(struct _ramf_ctl *ramf);
uint32_t ramf_empty_page_cnt(struct _ramf_ctl *ramf);

/* Exported functions --------------------------------------------------------*/

#endif


