
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
* @file pmc.c
*
* @author Freescale
*
* @brief providing APIs for configuring PMC. 
*
*******************************************************************************
*
* provide APIs for configuring PMC
******************************************************************************/
#include "pmc.h"

/******************************************************************************
* Constants
******************************************************************************/
/******************************************************************************
* Macros
******************************************************************************/
/******************************************************************************
* Types
******************************************************************************/
/******************************************************************************
* Global variables
******************************************************************************/
/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* PMC api list.
*
*//*! @addtogroup pmc_api_list
* @{
*******************************************************************************/
/*****************************************************************************//*!
*
* @brief  configure PMC with given parameters.
* 
* @param[in]   pPMC_Config       PMC configuration structure.
* @param[in]   pPMC             pointer to the PMC module.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see PMC_DeInit.
*
*****************************************************************************/
void PMC_Init(PMC_Type *pPMC, PMC_ConfigType *pPMC_Config)
{
    pPMC->SPMSC1 = pPMC_Config->sCtrlstatus.byte;
    pPMC->SPMSC2 = pPMC_Config->sDetectVoltSelect.byte;         
}


/*****************************************************************************//*!
*
* @brief  config the pmc register to the default mode.
* 
* @param[in]  pPMC              pointer to the PMC module.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see PMC_Init.
*
*****************************************************************************/
void PMC_DeInit(PMC_Type *pPMC)
{
    pPMC->SPMSC1 = 0x1C;
    pPMC->SPMSC2 = 0;         
}


/*****************************************************************************//*!
*
* @brief  config the pmc mode among run, wait and stop modes.
* 
* @param[in]  u8PmcMode         PMC mode select.
* @param[in]  pPMC              pointer to the PMC module.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void PMC_SetMode(PMC_Type *pPMC,uint8_t u8PmcMode)
{
    switch(u8PmcMode & 0x3) 
    {
        case PmcModeRun:    
            break;
        case PmcModeWait:
        	/* Clear the SLEEPDEEP bit to make sure we go into WAIT (sleep) mode instead
        	 * of deep sleep.
        	 */
        	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            /* Not using KEIL's uVision, so use the standard assembly command */
        	asm("WFI");
            break;
        case PmcModeStop4:
            /* enable LVD in stop mode */
            pPMC->SPMSC1 |= (PMC_SPMSC1_LVDE_MASK | PMC_SPMSC1_LVDSE_MASK);
        	/* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
        	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        	/* Not using KEIL's uVision, so use the standard assembly command */
        	asm("WFI");
        	break;
        case PmcModeStop3: 
            /* disable LVD in stop mode */
            pPMC->SPMSC1 &= ~(PMC_SPMSC1_LVDE_MASK | PMC_SPMSC1_LVDRE_MASK | PMC_SPMSC1_LVDSE_MASK);
        	/* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
        	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        	/* Not using KEIL's uVision, so use the standard assembly command */
        	asm("WFI");
        	break;
        default: 
            break;
    }
 
}

/*! @} End of pmc_api_list                                                    */

