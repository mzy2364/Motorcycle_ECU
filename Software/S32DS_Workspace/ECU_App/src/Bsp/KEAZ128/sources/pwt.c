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
* @file     pwt.c
*
* @author   Freescale
*
* @brief    Pulse Width Timer (PWT) source code. 
*
******************************************************************************/

#include "pwt.h"
#include "nvic.h"
#if defined(MCU_SKEAZ1284)
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
 * @brief global variable to store PWT callbacks.
 *
 */
PWT_CallbackType PWT_Callback[1] = {(PWT_CallbackType)(0)}; /*!< PWT initial callback */

/******************************************************************************
* Local functions
******************************************************************************/


/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* define PWT APIs
*
*//*! @addtogroup pwt_api_list
* @{
*******************************************************************************/

/*****************************************************************************//*!
*
* @brief initialize pwt module.
*        
*
* @param[in]   pConfig point to configuration  
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void PWT_Init(PWT_ConfigType *pConfig)
{  
    uint32_t u32PWTR1Value;
    u32PWTR1Value = 0;
    SIM->SCGC |= SIM_SCGC_PWT_MASK;     /*!< enable clock to PWT */
      
    if(PWT_GetOverflowFlag())
    {
        PWT_ClrOverflowFlag(); 
    }
    
    if(PWT_GetReadyFlag())
    {
        PWT_ClrReadyFlag(); 
    }

    if (pConfig->bPWTIntEn)            
    {                                     
        Enable_Interrupt(PWT_IRQn);
        u32PWTR1Value |= PWT_R1_PWTIE_MASK;
    }                   
    else                                     
    {                       
        Disable_Interrupt(PWT_IRQn);
    }
 
    if (pConfig->bOverflowIntEn)            
    {                                        
        u32PWTR1Value |= PWT_R1_POVIE_MASK;
    }

    if (pConfig->bReadyIntEn)            
    {                                        
        u32PWTR1Value |= PWT_R1_PRDYIE_MASK;
    } 
    
    u32PWTR1Value |= PWT_R1_PRE(pConfig->bClockPrescaler); 
    u32PWTR1Value |= PWT_R1_EDGE(pConfig->bEdgeSensitivity);     
    u32PWTR1Value |= PWT_R1_PINSEL(pConfig->bPinSelect);     
            
    if (pConfig->bClockSource)            
    {                                        
        u32PWTR1Value |= PWT_R1_PCLKS_MASK;
    } 
    
    if (pConfig->bPWTEn)            
    {                                        
        u32PWTR1Value |= PWT_R1_PWTEN_MASK;  
    } 
  
    PWT->R1 = u32PWTR1Value; 
}


/*****************************************************************************//*!
*
* @brief pwt module set call back.
*        
* @param[in] u8Channel_No channel number.   
* @param[in] pfnCallback point to call back.   
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void PWT_SetCallback(PWT_CallbackType pfnCallback)
{
  PWT_Callback[0] = pfnCallback;
}


/*****************************************************************************//*!
*
* @brief pwt module de-initialize, reset pwt register
*        
* @param none  
*
* @return none 
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void PWT_DeInit(void)
{
    
    Disable_Interrupt(PWT_IRQn);
    PWT_DisableInt();
    PWT_DisableReadyInt();
    PWT_DisableOverFlowInt();
    PWT_SetClockPresacler(0);        
    PWT_SetEdgeSensitivity(0 ); 
    PWT_SetSoftReset();
    PWT_Disable();    
    
    while((PWT->R1 & PWT_R1_PWTEN_MASK));
    SIM->SCGC &= ~SIM_SCGC_PWT_MASK;
}
/*! @} End of pwt_api_list                                                   */


/*****************************************************************************//*!
*
* @brief pwt module isr.
*        
* @param none
*
* @return none 
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void PWT_IRQHandler(void) 
{   

    if (PWT_Callback[0])
    {    
        PWT_Callback[0]();     
    }
}
#elif defined(MCU_SKEAZN642)
/* KEAZN64 does not support PWT. */
#else
/* If your device supports PWT, enable it. */
#endif
    


