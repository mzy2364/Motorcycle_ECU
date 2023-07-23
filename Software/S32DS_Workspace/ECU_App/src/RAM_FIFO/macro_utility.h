/**
  ******************************************************************************
  * @file    macro_utility.h
  * @author  ssddssdd
  * @version V0.1.0
  * @date    01-09-2016
  * @brief   Header file of  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 ssddssdd</center></h2>
  * <h2><center>&copy; All Rights Reserved</center></h2>
  *
  ******************************************************************************
  */

#ifndef __MACRO_UTILITY_H__
#define __MACRO_UTILITY_H__

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported macro-------------------------------------------------------------*/
/**@brief Perform rounded integer division (as opposed to truncating the result).
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Rounded (integer) result of dividing A by B. 四舍五入法则计算除法，例如：3/2 = 1  而现在3/2 = 2
 */
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))



/**@brief Check if the integer provided is a power of two.  检查提供的整数是否时2的幂
 *
 * @param[in]   A   Number to be tested.
 *
 * @return      true if value is power of two.   如果时2的幂，返回真
 * @return      false if value not power of two.  否则，返回假
 */
#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )



/**@brief Perform integer division, making sure the result is rounded up.  执行整数除法，确保结果向上舍入
 *
 * @details One typical use for this is to compute the number of objects with size B is needed to
 *          hold A number of bytes.
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Integer result of dividing A by B, rounded up.
 */
#define CEIL_DIV(A, B)      \
    (((A) + (B) - 1) / (B))



/**@brief Function for creating a buffer aligned to 4 bytes.  创建以4字节对齐的缓冲区
 *
 * @param[in]   NAME        Name of the buffor.
 * @param[in]   MIN_SIZE    Size of this buffor (it will be rounded up to multiples of 4 bytes).
 */
#define WORD_ALIGNED_MEM_BUFF(NAME, MIN_SIZE) static uint32_t NAME[CEIL_DIV(MIN_SIZE, sizeof(uint32_t))]



/**@brief Function for checking if a pointer value is aligned to a 4 byte boundary.   检查地址是否是4字节边界对齐
 *
 * @param[in]   p   Pointer value to be checked.
 *
 * @return      TRUE if pointer is aligned to a 4 byte boundary, FALSE otherwise.    如果4字节对齐，返回真
 */
static inline bool is_word_aligned(void const* p)
{
    return (((uintptr_t)p & 0x03) == 0);
}

/* Exported constants---------------------------------------------------------*/

/* Exported structures--------------------------------------------------------*/

/* Exported variables---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

#endif


