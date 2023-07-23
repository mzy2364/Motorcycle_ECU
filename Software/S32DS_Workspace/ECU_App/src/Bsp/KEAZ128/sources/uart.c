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
* @file uart.c
*
* @author Freescale
*
* @brief providing common UART API. 
*
******************************************************************************/
#include "uart.h"
#include "ics.h"
#include "derivative.h"


uint16_t global_pass_count = 0;
uint16_t global_fail_count = 0;
UART_TxDoneCallbackType UART_TxDoneCallback[MAX_UART_NO] = {(0)};
UART_RxDoneCallbackType UART_RxDoneCallback[MAX_UART_NO] = {(0)};


//static uint8_t *pUART_TxBuff[MAX_UART_NO] = {(0)};           /* pointer to RxBuf */
//static uint8_t *pUART_RxBuff[MAX_UART_NO] = {(0)};           /* pointer to TxBuf */
//static uint16_t gu16UART_TxBuffPos[MAX_UART_NO] = {0};        /* write position to RxBuf */
//static uint16_t gu16UART_RxBuffPos[MAX_UART_NO] = {0};        /* read position to TxBuf */
//static uint32_t gu32UART_BuffSize[MAX_UART_NO] = {0};         /* buffer size*/

#define NULL	(0)
/******************************************************************************
* Local variables
******************************************************************************/
UART_CallbackType UART_Callback[3] = {(UART_CallbackType)(0)};
/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Local functions
*****************************************************************************/
//void UART_InitPrint(void);
/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* define UART APIs
*
*//*! @addtogroup uart_api_list
* @{
*******************************************************************************/

/*****************************************************************************//*!
*
* @brief initialize the UART, interrupts disabled, and no hardware flow-control.
*        
* @param[in] pUART       base of UART port
* @param[in] pConfig     pointer to UART configuration structure
*
* @return none
*
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void UART_Init(UART_MemMapPtr pUART, UART_ConfigType *pConfig)
{
    uint16_t u16Sbr;
    uint8_t u8Temp;
    uint32_t u32SysClk = pConfig->u32SysClkHz;
    uint32_t u32Baud = pConfig->u32Baudrate;
    
	/* Enable the clock to the selected UART */    
    if (pUART == UART0_BASE_PTR)
	{
    	SIM_BASE_PTR->SCGC |= SIM_SCGC_UART0_MASK;
	}
    if (pUART == UART1_BASE_PTR)
   	{
       	SIM_BASE_PTR->SCGC |= SIM_SCGC_UART1_MASK;
   	}
    if (pUART == UART2_BASE_PTR)
    {
          	SIM_BASE_PTR->SCGC |= SIM_SCGC_UART2_MASK;
     }

    /* Make sure that the transmitter and receiver are disabled while we 
     * change settings.
     */
    pUART->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
    
    /* Configure the UART for 8-bit mode, no parity */
    pUART->C1 = pConfig->sctrl1settings.byte;
    
    /* Calculate baud settings */
    u16Sbr = (((u32SysClk)>>4) + (u32Baud>>1))/u32Baud;
    
    /* Save off the current value of the UARTx_BDH except for the SBR field */
    u8Temp = pUART->BDH & ~(UART_BDH_SBR_MASK);
    
    pUART->BDH = u8Temp |  UART_BDH_SBR(u16Sbr >> 8);
    pUART->BDL = (uint8_t)(u16Sbr & UART_BDL_SBR_MASK);
    
    if(1==pConfig->bSbns)
    {
    	UART_Set2StopBit(pUART);
    }
    else
    {
    	UART_Set1StopBit(pUART);

    }
    pUART->C3=pConfig->sctrl3settings.byte;
  	pUART->C2 =pConfig->sctrl2settings.byte;
  	
    if(1==pConfig->sctrl2settings.bits.bIlie||1==pConfig->sctrl2settings.bits.bRie||1==pConfig->sctrl2settings.bits.bTcie||1==pConfig->sctrl2settings.bits.bTie
    		||1==pConfig->sctrl3settings.bits.bFeie||1==pConfig->sctrl3settings.bits.bNeie||1==pConfig->sctrl3settings.bits.bOrie||1==pConfig->sctrl3settings.bits.bPeie)
    {
    	if(pUART==UART0)
    	{
      	  Enable_Interrupt(UART0_IRQn);

    	}
    	if(pUART==UART1)
    	{
      	  Enable_Interrupt(UART1_IRQn);

    	}
    	if(pUART==UART2)
    	{
      	  Enable_Interrupt(UART2_IRQn);

    	}
    }
    

}

/*****************************************************************************//*!
*
* @brief receive a character.
*        
* @param[in] pUART       base of UART port
*
* @return unsigned char
*
*****************************************************************************/
uint8_t UART_GetChar(UART_MemMapPtr pUART)
{
    /* Wait until character has been received */
    while (!(pUART->S1 & UART_S1_RDRF_MASK));
    
    /* Return the 8-bit data from the receiver */
    return pUART->D;
}
/*****************************************************************************//*!
*
* @brief send a character.
*        
* @param[in] pUART       base of UART port
* @param[in] u8Char      char to send
*
* @return none
*
*****************************************************************************/
void UART_PutChar(UART_MemMapPtr pUART, uint8_t u8Char)
{    
    /* Wait until space is available in the FIFO */
    while (!(pUART->S1 & UART_S1_TDRE_MASK));
    
    /* Send the character */
    pUART->D = (uint8_t)u8Char;
}

/*****************************************************************************//*!
*
* @brief set baudrate.
*        
* @param[in] pUART       base of UART port
* @param[in] pConfig     baudrate config parameters
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
void UART_SetBaudrate(UART_MemMapPtr pUART, UART_ConfigBaudrateType *pConfig)
{
    uint8_t u8Temp;
    uint16_t u16Sbr;
    uint32_t u32SysClk    = pConfig->u32SysClkHz;
    uint32_t u32baud       = pConfig->u32Baudrate;
 
    /* Calculate baud settings */
    u16Sbr = (((u32SysClk)>>4) + (u32baud>>1))/u32baud;

    /* Save off the current value of the UARTx_BDH except for the SBR field */
    u8Temp = pUART->BDH & ~(UART_BDH_SBR_MASK);
    
    pUART->BDH = u8Temp |  UART_BDH_SBR(u16Sbr >> 8);
    pUART->BDL = (uint8_t)(u16Sbr & UART_BDL_SBR_MASK);

    /* Enable receiver and transmitter */
    pUART->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );

}


/*****************************************************************************//*!
*
* @brief disable interrupt.
*        
* @param[in] pUART base of UART port
* @param[in] InterruptType interrupt type
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
void UART_DisableInterrupt(UART_MemMapPtr pUART, UART_InterruptType InterruptType)
{

    if (InterruptType == UART_TxBuffEmptyInt)
    {
        pUART->C2 &= (~UART_C2_TIE_MASK);
    }
    else if (InterruptType == UART_TxCompleteInt)
    {
        pUART->C2 &= (~UART_C2_TCIE_MASK);
    }
    else if (InterruptType == UART_RxBuffFullInt)
    {
        pUART->C2 &= (~UART_C2_RIE_MASK);
    }
    else if (InterruptType == UART_IdleLineInt)
    {
        pUART->C2 &= (~UART_C2_ILIE_MASK);
    }
    else if (InterruptType == UART_RxOverrunInt)
    {
        pUART->C3 &= (~UART_C3_ORIE_MASK);
    }
    else if (InterruptType == UART_NoiseErrorInt)
    {
        pUART->C3 &= (~UART_C3_NEIE_MASK);
    }
    else if (InterruptType == UART_FramingErrorInt)
    {
        pUART->C3 &= (~UART_C3_FEIE_MASK);
    } 
    else if (InterruptType == UART_ParityErrorInt)
    {
        pUART->C3 &= (~UART_C3_FEIE_MASK);
    } 
    else
    {
        /* un-supported interrupt type */
    }  
}


/*****************************************************************************//*!
*
* @brief get flags from 2 UART status registers.
*        
* @param[in] pUART  base of UART port
*
* @return       16-bit flags
*
* @ Pass/ Fail criteria:
*****************************************************************************/
uint16_t UART_GetFlags(UART_MemMapPtr pUART)
{
    uint16_t u16StatusFlags = 0;

    u16StatusFlags = pUART->S2;
    u16StatusFlags = (u16StatusFlags<<8)| pUART->S1; 

    return u16StatusFlags;
}
/*****************************************************************************//*!
*
* @brief check whether the specified flag is set.
*        
* @param[in] pUART      base of UART port
* @param[in] FlagType   flag type
*
* @return       
*               1, flag is set
*               0, flag is clear
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
uint8_t UART_CheckFlag(UART_MemMapPtr *pUART, UART_FlagType FlagType)
{
    uint16_t u16StatusFlags = 0;

    u16StatusFlags = UART_GetFlags((UART_MemMapPtr)pUART);

    return (u16StatusFlags & (1<<FlagType));
}

/*****************************************************************************//*!
*
* @brief send a series of charecters using polling mode.
*        
* @param[in] pUART      base of UART port
* @param[in] pSendBuff  pointer of charecters to send
* @param[in] u32Length  number of charecters
*
* @return       none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
void UART_SendWait(UART_MemMapPtr pUART, uint8_t *pSendBuff, uint32_t u32Length)
{
    uint8_t u8TxChar;
    uint32_t  i;
    
    for (i = 0; i < u32Length; i++)
    {
        u8TxChar = pSendBuff[i];
        while (!UART_IsTxBuffEmpty(pUART))
        {
        }
        UART_WriteDataReg(pUART, u8TxChar);        
    }
}

/*****************************************************************************//*!
*
* @brief receive a series of charecters using polling mode.
*        
* @param[in] pUART          base of UART port
* @param[in] pReceiveBuff   pointer of charecters to receive
* @param[in] u32Length      number of charecters
*
* @return       none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
void UART_ReceiveWait(UART_MemMapPtr pUART, uint8_t *pReceiveBuff, uint32_t u32Length)
{
    uint8_t u8RxChar;
    uint32_t i;
    
    for (i = 0; i < u32Length; i++)
    {
        while (!UART_IsRxBuffFull(pUART))
        {  
        }    
        u8RxChar = UART_ReadDataReg(pUART);
        pReceiveBuff[i] = u8RxChar;
    }
}

/*****************************************************************************//*!
*
* @brief wait tx complete.
*        
* @param[in] pUART      base of UART port
*
* @return       none
*
* @ Pass/ Fail criteria: none*****************************************************************************/
void UART_WaitTxComplete(UART_MemMapPtr pUART)
{
    while (!UART_IsTxComplete(pUART));
}

/*****************************************************************************//*!
*
* @brief set up UART callback routines to be called by interrupt service routine.
*        
* @param[in]  pUART         pointer to an UART register base 
* @param[in]  pfnCallback   callback routine
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
void UART_SetCallback(UART_MemMapPtr pUART, UART_CallbackType pfnCallback)
{

    if(pUART==UART0){
    	UART_Callback[0]=pfnCallback;
    }else if(pUART==UART1){
    	UART_Callback[1]=pfnCallback;
    }else{
    	UART_Callback[2]=pfnCallback;
    }
}


/*! @} End of uart_api_list */


/*****************************************************************************//*!
*
* @brief UART0_BASE_PTR interrupt service routine.
*        
* @param        none
*
* @return       none
*
* @ Pass/ Fail criteria:
*****************************************************************************/

void UART0_IRQHandler(void)
{
	UART_Callback[0]();
}


/*****************************************************************************//*!
*
* @brief UART1_BASE_PTR interrupt service routine.
*        
* @param        none
*
* @return       none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
void UART1_IRQHandler(void)
{
    UART_Callback[1]();
}


/*****************************************************************************//*!
*
* @brief UART2_BASE_PTR interrupt service routine.
*        
* @param        none
*
* @return       none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
void UART2_IRQHandler(void)
{
    UART_Callback[2]();
}



