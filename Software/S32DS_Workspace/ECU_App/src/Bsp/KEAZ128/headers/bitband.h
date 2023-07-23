
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
* @file bitband.h
*
* @author Freescale
*
* @brief Provide Bit-band utilities.
******************************************************************************/

#ifndef __BIT_BAND_H
#define __BIT_BAND_H
#ifdef __cplusplus
extern "C" {
#endif

#if defined(MCU_SKEAZ1284)
/******************************************************************************
*
*
*//*! @addtogroup BIT_BandType
* @{
*******************************************************************************/
/*!
 * @brief bit band type.
 *
 */
typedef struct
{
		uint32_t bBit0;					/* Alias to 0th bit */
		uint32_t bBit1;					/* Alias to 1th bit */
		uint32_t bBit2;					/* Alias to 2th bit */
		uint32_t bBit3;					/* Alias to 3th bit */
		uint32_t bBit4;					/* Alias to 4th bit */
		uint32_t bBit5;					/* Alias to 5th bit */
		uint32_t bBit6;					/* Alias to 6th bit */
		uint32_t bBit7;					/* Alias to 7th bit */
		uint32_t bBit8;					/* Alias to 8th bit */
		uint32_t bBit9;					/* Alias to 9th bit */
		uint32_t bBit10;				/* Alias to 10th bit */
		uint32_t bBit11;				/* Alias to 11th bit */
		uint32_t bBit12;				/* Alias to 12th bit */
		uint32_t bBit13;				/* Alias to 13th bit */
		uint32_t bBit14;				/* Alias to 14th bit */
		uint32_t bBit15;				/* Alias to 15th bit */
		uint32_t bBit16;				/* Alias to 16th bit */
		uint32_t bBit17;				/* Alias to 17th bit */
		uint32_t bBit18;				/* Alias to 18th bit */
		uint32_t bBit19;				/* Alias to 19th bit */
		uint32_t bBit20;				/* Alias to 20th bit */
		uint32_t bBit21;				/* Alias to 21th bit */
		uint32_t bBit22;				/* Alias to 22th bit */
		uint32_t bBit23;				/* Alias to 23th bit */
		uint32_t bBit24;				/* Alias to 24th bit */
		uint32_t bBit25;				/* Alias to 25th bit */
		uint32_t bBit26;				/* Alias to 26th bit */
		uint32_t bBit27;				/* Alias to 27th bit */
		uint32_t bBit28;				/* Alias to 28th bit */
		uint32_t bBit29;				/* Alias to 29th bit */
		uint32_t bBit30;				/* Alias to 30th bit */
		uint32_t bBit31;				/* Alias to 31th bit */
}BIT_BandType,*BIT_BandPtr;
/*! @} End of BIT_BandType                                               */

/******************************************************************************
* define  API list
*
*//*! @addtogroup bit_band_api_list
* @{
*******************************************************************************/
/*****************************************************************************//*!
   *
   * @brief bit-band initialize pointer, so that invoke the pointer to access alisaed bitband.
   *
   * @param[in]  pVariableAddress - point to variable.
   * @param[in]  pBitbandPtr - point to alisaed bitband address.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
__STATIC_INLINE uint8_t BIT_BandVariableInit( uint32_t *pVariableAddress,BIT_BandPtr *pBitbandPtr )
{

	if(!(((uint32_t)pVariableAddress >= 0x20000000)&&((uint32_t)pVariableAddress <= (0x20000000+12*1024))))
		{
			return 0;
		}

    *pBitbandPtr = (BIT_BandPtr)(((uint32_t)pVariableAddress-0x20000000)*32+0x22000000);

    return 1;
}
/*! @} End of bit_band_api_list                                                    					*/
#ifdef __cplusplus
}
#endif
#elif defined(MCU_SKEAZN642)
/* No aliased bit-band for KEAZ642. */
#else
/* Write definitions for your device. */
#endif
#endif /* __BIT_BAND_H */


