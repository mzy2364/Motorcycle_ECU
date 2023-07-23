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
* @file     pit.h
*
* @author   Freescale
*
*
* @brief    Periodic Interrupt Timer (PIT) driver head file. 
*
******************************************************************************/
#ifndef PIT_H_
#define PIT_H_
#ifdef __cplusplus
extern "C" {
#endif

//#define PIT PIT_BASE_PTR
/******************************************************************************
* Includes
******************************************************************************/
#include "derivative.h"
/******************************************************************************
* Constants
******************************************************************************/
#define FALSE 0
#define TRUE 1
/******************************************************************************
* PIT channel number list
*
*//*! @addtogroup pit_channelnumber
* @{
*******************************************************************************/
enum
{
    PIT_CHANNEL0 = 0,       /*!< PIT channel 0 */
    PIT_CHANNEL1            /*!< PIT channel 1 */
};

/*! @} End of pit_channelnumber                                               */

/******************************************************************************
* Macros
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/

/*
 * Callback type
 */

/******************************************************************************
* PIT callback function declaration
*
*//*! @addtogroup pit_callback
* @{
*******************************************************************************/
typedef void (*PIT_CallbackType)(void);             /*!< PIT callback type */

/*! @} End of pit_callback                                                    */

/* PIT configuration structure 
 */  
/*!
 * @brief PIT configuration type.
 *
 */
typedef struct
{   
    uint8_t bFreeze         : 1;    /*!< 1: stop in debug mode, 0: run in debug mode */
    uint8_t bModuleDis      : 1;    /*!< 1: PIT module is disable, 0: PIT module is enable */    
    uint8_t bReserved0      : 1;    /*!< reserved bit */ 
    uint8_t bReserved1      : 5;    /*!< reserved bit */ 
    uint8_t bTimerEn        : 1;    /*!< 1: channel timer is enable, 0: channel timer is disable */ 
    uint8_t bInterruptEn    : 1;    /*!< 1: channel timer interrupt is enable, 0: channel timer interrupt is disable */ 
    uint8_t bChainMode      : 1;    /*!< 1: chain mode is enable, 0: chain mode is disable */ 
    uint8_t bReserved2      : 5;    /*!< reserved bit */ 
    uint8_t bFlag           : 1;    /*!< 1: flag is set,and write 1 to clear flag, 0: no flag is set */
    uint8_t bReserved3      : 7;    /*!< reserved bit */   
    uint32_t u32LoadValue      ;    /*!< 32-bit channel load value */  
} PIT_ConfigType, *PIT_ConfigPtr; 


/******************************************************************************
* Global variables
******************************************************************************/


/*!
 * inline functions
 */

/******************************************************************************
* PIT API list
*
*//*! @addtogroup pit_api_list
* @{
*******************************************************************************/

/*****************************************************************************//*!
*
* @brief enable pit module.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_Enable(void)
{

    PIT->MCR &= ~PIT_MCR_MDIS_MASK; 
    
}


/*****************************************************************************//*!
*
* @brief disable pit module.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_Disable(void)
{

    PIT->MCR |= PIT_MCR_MDIS_MASK; 
}


/*****************************************************************************//*!
*
* @brief pit timers are stopped in debug mode.
*        
* @param  none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_SetDebugFreeze(void)
{

    PIT->MCR |= PIT_MCR_FRZ_MASK;
}


/*****************************************************************************//*!
*
* @brief pit timers are running in debug mode.
*        
* @param  none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_SetDebugOn(void)
{

    PIT->MCR &= ~PIT_MCR_FRZ_MASK;
}


/*****************************************************************************//*!
*
* @brief enable pit channel timer.
*        
* @param[in]   u8Channel.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_ChannelEnable(uint8_t u8Channel)

{

    PIT->CHANNEL[u8Channel].TCTRL |= PIT_TCTRL_TEN_MASK; 
}


/*****************************************************************************//*!
*
* @brief disable pit channel timer.
*        
* @param[in]   u8Channel.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_ChannelDisable(uint8_t u8Channel)
{

    PIT->CHANNEL[u8Channel].TCTRL &= ~PIT_TCTRL_TEN_MASK; 
}


/*****************************************************************************//*!
*
* @brief enable pit channel timer interrupt.
*        
* @param[in]  u8Channel.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_ChannelEnableInt(uint8_t u8Channel)

{

    PIT->CHANNEL[u8Channel].TCTRL |= PIT_TCTRL_TIE_MASK; 
}


/*****************************************************************************//*!
*
* @brief disable pit channel timer interrupt .
*        
* @param[in]   u8Channel.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_ChannelDisableInt(uint8_t u8Channel)

{

    PIT->CHANNEL[u8Channel].TCTRL &= ~PIT_TCTRL_TIE_MASK; 
}


/*****************************************************************************//*!
*
* @brief  enable pit channel timer chain mode.
*        
* @param[in]   u8Channel.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_ChannelEnableChain(uint8_t u8Channel)
{
    PIT->CHANNEL[u8Channel].TCTRL |= PIT_TCTRL_CHN_MASK;
}


/*****************************************************************************//*!
*
* @brief  disable pit channel timer chain mode.
*        
* @param[in]   u8Channel.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_ChannelDisableChain(uint8_t u8Channel)

{
    PIT->CHANNEL[u8Channel].TCTRL &= ~PIT_TCTRL_CHN_MASK;
}


/*****************************************************************************//*!
*
* @brief get pit channel timer interrrupt flag.
*        
* @param[in]   u8Channel.
*
* @return bflag.
*
* @ Pass/ Fail criteria: none

*****************************************************************************/
static inline uint8_t PIT_ChannelGetFlags(uint8_t u8Channel)

{
    uint8_t bflag;  
    
    bflag = (PIT->CHANNEL[u8Channel].TFLG & PIT_TFLG_TIF_MASK);    
    
    return bflag;
    
}


/*****************************************************************************//*!
*
* @brief clear pit channel timer interrrupt flag.
*        
* @param[in] u8Channel
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PIT_ChannelClrFlags(uint8_t u8Channel)
{
   PIT->CHANNEL[u8Channel].TFLG |= PIT_TFLG_TIF_MASK;      

}


/******************************************************************************
* Global functions
******************************************************************************/
void PIT_Init(uint8_t u8Channel_No, PIT_ConfigType *pConfig);
void PIT_SetLoadVal(uint8_t u8Channel, uint32_t u32loadvalue);
void PIT_SetCallback(uint8_t u8Channel_No, PIT_CallbackType pfnCallback);
void PIT_DeInit(void);
/*! @} End of pit_api_list                                                   */

#ifdef __cplusplus
}
#endif
#endif /* PIT_H_ */
