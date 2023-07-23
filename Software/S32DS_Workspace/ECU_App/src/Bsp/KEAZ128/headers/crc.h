/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2013 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file     crc.h
*
* @author   Freescale

* @brief    Cyclic redundancy check (CRC) header file. 
*
******************************************************************************/
#ifndef CRC_H_
#define CRC_H_
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* Includes
******************************************************************************/
#include "derivative.h"

#if defined(MCU_SKEAZ1284) || defined(MCU_SKEAZN642)
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros
******************************************************************************/


/******************************************************************************
* CRC control bit definition
*
*//*! @addtogroup crc_controlbit
* @{
*******************************************************************************/

/*!
 * @brief CRC control register bit definition.
 *
 */

#define CRC_WIDTH_16BIT                 0	/* Select CRC16 protocol */
#define CRC_WIDTH_32BIT                 1	/* Select CRC32 protocol */
#define CRC_DATA_SEED                   1	/* Write CRC Data Register are seed */
#define CRC_DATA_DATA                   0	/* Write CRC Data Register are data */
#define CRC_READ_COMPLETE               1	/* Invert or complement read CRC Data register */
#define CRC_READ_NONE                   0	/* No XOR on reading */
#define CRC_READ_TRANSPOSE_NONE         0	/* No transposition in read */
#define CRC_READ_TRANSPOSE_BIT          1	/* Only bits in bytes are transposed in read */
#define CRC_READ_TRANSPOSE_ALL          2	/* Both bits in bytes and bytes are transposed in read */
#define CRC_READ_TRANSPOSE_BYTE         3	/* Only bytes are transposed in read */
#define CRC_WRITE_TRANSPOSE_NONE        0	/* No transposition write */
#define CRC_WRITE_TRANSPOSE_BIT         1	/* Only bits in bytes are transposed in write */
#define CRC_WRITE_TRANSPOSE_ALL         2	/* Both bits in bytes and bytes are transposed in write */
#define CRC_WRITE_TRANSPOSE_BYTE        3	/* Only bytes are transposed in write */

/*! @} End of crc_controlbit                                                */

     
/******************************************************************************
* Types
******************************************************************************/
/* CRC configuration structure 
 */  
/******************************************************************************
* CRC Configuration Structure type.
*
*//*! @addtogroup crc_config_type
* @{
*******************************************************************************/ 
/*!
 * @brief CRC Configuration Structure.
 *
 */

typedef struct
{          
    uint8_t bWidth                  : 1;    /* 1: 32-bit CRC protocol , 0: 16-bit CRC protocol */
    uint8_t bDataType               : 1;    /* 1: write seed , 0: write data */
    uint8_t bFinalXOR               : 1;    /* 1: Invert or complement read , 0: No XOR on reading */
    uint8_t bRESERVED               : 1;    /* Reserved bit */
    uint8_t bTransposeReadType      : 2;    /* Type of transpose For read, see reference manual */
    uint8_t bTransposeWriteType     : 2;    /* Type of transpose For write, see reference manual */
    uint32_t u32PolyData               ;    /* 32bit or 16-biy poly data */
} CRC_ConfigType, *CRC_ConfigPtr  ; 
/*! @} End of crc_config_type                                                */

           
/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* CRC API list
*
*//*! @addtogroup crc_api_list
* @{
*******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
void        CRC_Init(CRC_ConfigType *pConfig);
uint32_t    CRC_Cal16(uint32_t u32Seed, uint8_t *msg, uint32_t u32SizeBytes);
uint32_t    CRC_Cal32(uint32_t u32Seed, uint8_t *msg, uint32_t u32SizeBytes);
void        CRC_DeInit(void);
/*! @} End of crc_api_list                                                   */

#ifdef __cplusplus
}
#endif
#else
/* If your device has additional CRC functionality, add it here. */
#endif
#endif /* CRC_H_ */



