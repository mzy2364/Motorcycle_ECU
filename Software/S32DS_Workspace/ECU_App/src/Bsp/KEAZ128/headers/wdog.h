
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
* @file wdog.h
*
* @author Freescale
*
*
* @brief provide commond watch dog utilities. 
*
*******************************************************************************
*
* provide APIs for accessing watch dog
******************************************************************************/

#ifndef __WDOG_H__
#define __WDOG_H__

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
* Includes
******************************************************************************/

#include "derivative.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros
******************************************************************************/
/* wdog_unlock sequence must be performed within 16 bus clock cycles without
 * any interrupt 
 */
   
/* WDOG clock sources option */
/******************************************************************************
* define watchdog clock source selection
*
*//*! @addtogroup wdog_clock_sources
* @{
*******************************************************************************/

#define WDOG_CLK_BUS                    0   /*!< clock source is bus clock */
#define WDOG_CLK_INTERNAL_32KHZ         2   /*!< clock source is internal oscillator 32 kHz (ICSIRCLK) */
#define WDOG_CLK_INTERNAL_1KHZ	        1   /*!< clock source is internal LPO 1 KHz */
#define WDOG_CLK_EXTERNAL               3   /*!< clock source is external clock */
/*! @} End of wdog_clock_sources                                                    					*/

/* WDOG clock source selection */
#define WDOG_CLK  (WDOG_CLK_INTERNAL_1KHZ)
    
/* WDOG default values */
#define WDOG_CS1_DEFAULT_VALUE      0x80
#define WDOG_CS2_DEFAULT_VALUE      0x01
#define WDOG_TOVAL_DEFAULT_VALUE    0x0400
#define WDOG_WIN_DEFAULT_VALUE      0x0000
     
/* WDOG utilities */  

/******************************************************************************
* define watchdog API list
*
*//*! @addtogroup wdog_api_list
* @{
*******************************************************************************/
/*! 
 * @brief watchdog unlock routine.
 */
#define WDOG_Unlock()        WDOG->CNT = 0x20C5; WDOG->CNT = 0x28D9        
/*! @} End of wdog_api_list                                                    					*/

/******************************************************************************
* Types
******************************************************************************/
/******************************************************************************
* WDOG callback function declaration
*
*//*! @addtogroup wdog_callback
* @{
*******************************************************************************/

/*!
 * @brief WDOG Callback type.
 *
 */

typedef void (*WDOG_CallbackType)(void);

/*! @} End of wdog_callback  */  

/******************************************************************************
* define watchdog configuration structure
*
*! @addtogroup wdog_config_type
* @{
*******************************************************************************/

/*!
 * @brief watchdog configuration structure.
 *
 */
typedef struct {	
    struct {
	uint16_t    bIntEnable      : 1;    /*!< watchdog interrupt enable */
	uint16_t    bDisable        : 1;    /*!< disable watchdog */
	uint16_t    bWaitEnable     : 1;    /*!< enable watchdog in wait mode */
	uint16_t    bStopEnable     : 1;    /*!< enable watchdog in stop mode */
	uint16_t    bDbgEnable      : 1;    /*!< enable watchdog in debug mode */
	uint16_t    bWinEnable      : 1;    /*!< enable watchdog window mode */
	uint16_t    bUpdateEnable   : 1;    /*!< enable update of watchdog control */
	uint16_t    bClkSrc         : 2;    /*!< watchdog clock source selection */  
	uint16_t    bPrescaler      : 1;    /*!< prescaler */  
    }sBits;                             /*!< bitfield structure  */
    uint16_t    u16TimeOut;             /*!< watchdog timeout value */
    uint16_t    u16WinTime;             /*!< watchdog window value */
} WDOG_ConfigType, *WDOG_ConfigPtr;  /*!< watchdog configuration structure type */
/*! @} End of wdog_config_type                                                    					*/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* define watchdog API list
*
*//*! @addtogroup wdog_api_list
* @{
*******************************************************************************/


/*****************************************************************************//*!
*
* @brief set time out value for WDOG.
*        
* @param[in]   u16TimeOut  timeout value to TOVAL register.
*
* @return none
*
* @ Pass/ Fail criteria: none
*
*****************************************************************************/

static inline void WDOG_SetTimeOut(uint16_t u16TimeOut)
{
    WDOG->CNT = 0x20C5; 
    WDOG->CNT = 0x28D9;
    WDOG->TOVAL8B.TOVALL  = u16TimeOut;
    WDOG->TOVAL8B.TOVALH  = u16TimeOut >> 8;
}



/*****************************************************************************//*!
*
* @brief set window value for WDOG.
*        
* @param[in]   u16WinTime  window value to WIN register.
*
* @return none
*
* @ Pass/ Fail criteria: none
*
*****************************************************************************/

static inline void WDOG_SetWindow(uint16_t u16WinTime)
{
    WDOG->CNT = 0x20C5; 
    WDOG->CNT = 0x28D9;
    WDOG->WIN8B.WINL  = u16WinTime;
    WDOG->WIN8B.WINH  = u16WinTime >> 8;
}

/*****************************************************************************//*!
*
* @brief check if watchdog reset occurs.
*        
* @param    none.
*
* @return   TRUE if watchdog reset occurs, FALSE otherwise.
*
* @ Pass/ Fail criteria: none
*****************************************************************************/

static inline uint8_t WDOG_IsReset(void)
{	uint32_t    u32Status;
	u32Status = SIM->SRSID & SIM_SRSID_WDOG_MASK;
    if(u32Status)
    {
        return (1);
    }
    return (0);
}

/*! @} End of wdog_api_list                                                    					*/



void WDOG_Init(WDOG_ConfigPtr pConfig);
void WDOG_DeInit(void);
void WDOG_Disable(void);
void WDOG_DisableWDOGEnableUpdate(void);
void WDOG_Enable(void);
void WDOG_Feed(void);
void WDOG_SetTimeOut(uint16_t u16TimeOut);
void WDOG_SetWindow(uint16_t u16WinTime);
void WDOG_EnableUpdate(void);
void WDOG_DisableUpdate(void);
uint8_t WDOG_IsReset(void);
void WDOG_SetCallback(WDOG_CallbackType pfnCallback);

#ifdef __cplusplus
}
#endif

/********************************************************************/
#endif /* __WDOG_H__ */

