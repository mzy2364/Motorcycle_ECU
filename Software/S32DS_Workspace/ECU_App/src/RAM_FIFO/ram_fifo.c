/**
  ******************************************************************************
  * @file    mems_ramf.c
  * @author  1Tianxia Embedded Software Team
  * @version V0.1.0
  * @date    28-07-2014
  * @brief   This file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 1Tianxia</center></h2>
  * <h2><center>&copy; All Rights Reserved</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ram_fifo.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  获取未读取的数据页数
 *
 *END**************************************************************************/
uint32_t ramf_data_page_cnt(struct _ramf_ctl *ramf)
{
  return (ramf->wr - ramf->rd);
}


/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  获取当前FIFO中空页数
 *
 *END**************************************************************************/
uint32_t ramf_empty_page_cnt(struct _ramf_ctl *ramf)
{
  return (ramf->total_page_mask + 1 - (ramf->wr - ramf->rd));
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  FIFO初始化
 *
 *END**************************************************************************/
bool ramf_init(struct _ramf_ctl *ramf, uint8_t *buffer, uint32_t page_size, uint32_t total_page)
{
  if (NULL == ramf)
    return false;
  
  if (!IS_POWER_OF_TWO(total_page))
    return false;

  ramf->page_size = page_size;
  ramf->total_page_mask = total_page - 1;
  
  ramf->buffer = buffer;
  ramf->rd = ramf->wr = 0;

  return true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  从FIFO中，读出一页数据到指定缓冲区
 *
 *END**************************************************************************/
bool ramf_read(struct _ramf_ctl *ramf, uint8_t *dst)
{
  uint32_t cnt;

  /*参数合法性检测*/
  if ((NULL == ramf) || (NULL == dst))
    return false;

  /*确保当前有数据页可读*/
  cnt = ramf_data_page_cnt(ramf);
  if (0 == cnt)
    return false;

  memcpy(dst, ramf->buffer + (ramf->rd & ramf->total_page_mask) * ramf->page_size, ramf->page_size);
  ramf->rd++;

  return true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  在FIFO中，写入一页数据
 *
 *END**************************************************************************/
bool ramf_write(struct _ramf_ctl *ramf, uint8_t *src)
{
  if (NULL == ramf)
    return false;

  /* delete the old data */
  if (0 == ramf_empty_page_cnt(ramf))
    ramf->rd++;

  memcpy(ramf->buffer + (ramf->wr & ramf->total_page_mask) * ramf->page_size, src, ramf->page_size);

  ramf->wr++;

  return true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  20190421 自己改写，在想fifo中写入数据时，可以指定写入的字节个数，这种情况用在每次写入的数据长度不固定，但是fifo中的页在分配时，必须一样，只是在拷贝时，提高效率
 *
 *END**************************************************************************/
bool ramf_write_lengh(struct _ramf_ctl *ramf, uint8_t *src, uint32_t lengh)
{
  if (NULL == ramf)
    return false;

  /* delete the old data */
  if (0 == ramf_empty_page_cnt(ramf))
    ramf->rd++;

  memcpy(ramf->buffer + (ramf->wr & ramf->total_page_mask) * ramf->page_size, src, lengh);

  ramf->wr++;

  return true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :
 *
 *END**************************************************************************/
bool ramf_read_fresh(struct _ramf_ctl *ramf, uint8_t *dst)
{
  if ((NULL == ramf) || (NULL == dst))
    return false;

  if (ramf->rd == ramf->wr)
    return false;

  memcpy(dst, ramf->buffer + ((--ramf->wr) & ramf->total_page_mask) * ramf->page_size, ramf->page_size);

  return true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :
 *
 *END**************************************************************************/
bool ramf_peek_fresh(struct _ramf_ctl *ramf, uint8_t *dst)
{

  if ((NULL == ramf) || (NULL == dst))
    return false;

  if (ramf->rd == ramf->wr)
    return false;

  memcpy(dst, ramf->buffer + ((--ramf->wr) & ramf->total_page_mask) * ramf->page_size, ramf->page_size);

  ramf->wr++;
  
  return true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  获取未读取的数据页数
 *
 *END**************************************************************************/
bool ramf_peek_offset(struct _ramf_ctl *ramf, uint8_t *dst, uint32_t offset)
{
  uint32_t cnt;

  if ((NULL == ramf) || (NULL == dst))
    return false;

  cnt = ramf_data_page_cnt(ramf);
  if (cnt < offset)
    return false;

  memcpy(dst, ramf->buffer + ((ramf->wr - offset) & ramf->total_page_mask) * ramf->page_size, ramf->page_size);

  return true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  一次性，读取N页数据到指定缓冲区
 *
 *END**************************************************************************/
uint32_t ramf_nread(struct _ramf_ctl *ramf, uint8_t *dst, uint32_t num)
{
  uint8_t i;
  uint32_t cnt;
  
  if ((NULL == ramf) || (NULL == dst))
    return 0;

  cnt = ramf_data_page_cnt(ramf);
  if (0 == cnt)
    return 0;

  if (cnt > num)
    cnt = num;
  
  for (i = 0; i < cnt; i++) {
    memcpy(dst + i * ramf->page_size, ramf->buffer + (ramf->rd & ramf->total_page_mask) * ramf->page_size, ramf->page_size);
    ramf->rd++;
  }
  
  return (cnt * ramf->page_size);
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :  调整FIFO的页数，裁剪页数
 *
 *END**************************************************************************/
void ramf_trim(struct _ramf_ctl *ramf, uint32_t dst_cnt)
{
  uint32_t cnt;

  cnt = ramf_data_page_cnt(ramf);
  
  if (cnt <= dst_cnt)
    return;

  ramf->rd += (cnt - dst_cnt);
}

/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   : 强值销毁FIFO中的数据
 *
 *END**************************************************************************/
void ramf_flush(struct _ramf_ctl *ramf)
{
  ramf->rd = ramf->wr;
}


