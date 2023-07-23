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
* @file     rtc.c
*
* @author   Freescale
*
* @brief    Real-time counter (RTC) driver source code.  
*
******************************************************************************/
#include "rtc.h"

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Constants and macros
******************************************************************************/

/******************************************************************************
* Local types
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Local variables
******************************************************************************/
/*!
 * @brief global variable to store RTC callbacks.
 *
 */
RTC_CallbackType RTC_Callback[1] = {(RTC_CallbackType)(0)};    /*!< RTC initial callback */

/******************************************************************************
* Local functions
******************************************************************************/
void RTC_IRQHandler(void);

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* define RTC APIs
*
*//*! @addtogroup rtc_api_list
* @{
*******************************************************************************/

/*****************************************************************************//*!
*
* @brief inital RTC module
*        
* @param[in] pConfig point to configuration  
*
* @return none 
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void RTC_Init(RTC_ConfigType *pConfig)
{
    uint16_t    u16Clocksource, u16Prescler;
    uint16_t    u16ModVal;
    
    u16Clocksource =0;  
    u16Prescler    =0;
    u16ModVal      =0;

    SIM->SCGC     |= SIM_SCGC_RTC_MASK; 
    
    u16ModVal      = pConfig->u16ModuloValue;
    RTC_SetModulo(u16ModVal); 
    
    if (pConfig->bRTCOut)
    {
    
        RTC->SC= RTC_SC_RTCO_MASK;       
    } 
    
    if (pConfig->bInterruptEn)
    {
         Enable_Interrupt(RTC_IRQn);
         RTC_EnableInt(); 
    }
    else      
    {   
    
        Disable_Interrupt(RTC_IRQn);
    }
    
    if (pConfig->bFlag)
    {   
        RTC_ClrFlags();         
    }

    u16Clocksource = pConfig->bClockSource; 
    u16Prescler    = pConfig->bClockPrescaler;
     
    RTC_SetClock(u16Clocksource,u16Prescler );
}



/*****************************************************************************//*!
*
* @brief set call back function for rtc module 
*        
* @param[in] pfnCallback point to call back function
*
* @return none 
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void RTC_SetCallback(RTC_CallbackType pfnCallback)
{
  RTC_Callback[0] = pfnCallback;
}


/*****************************************************************************//*!
*
* @brief de-initialize rtc module , reset rtc register
*        
* @param none  
*
* @return none 
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void RTC_DeInit(void)
{
    Disable_Interrupt(RTC_IRQn);
    RTC->MOD = 0;
    while(RTC->MOD);
    
    if(RTC_GetFlags())
    {
        RTC_ClrFlags();
    }
    
    RTC->SC = 0;
    while(RTC->SC);
    SIM->SCGC &= ~SIM_SCGC_RTC_MASK;
}

/*! @} End of rtc_api_list                                                   */

/*****************************************************************************//*!
*
* @brief RTC module interrupt service routine
*        
* @param none  
*
* @return none 
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void RTC_IRQHandler(void) 
{
    RTC_ClrFlags();	       
    if (RTC_Callback[0])
    {    
        RTC_Callback[0]();     
    }
}


