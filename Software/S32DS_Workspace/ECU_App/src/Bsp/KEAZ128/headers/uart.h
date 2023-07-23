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
* @file UART.h
*
* @author Freescale
*
*
* @brief provide commond UART utilities. 
*
*******************************************************************************/
#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* Includes
******************************************************************************/
#include "derivative.h"
#include "nvic.h"

/* callback types */
typedef void (*UART_TxDoneCallbackType)(void);
typedef void (*UART_RxDoneCallbackType)(void);

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros
******************************************************************************/
#define MAX_UART_NO             3

#define UART_0	UART0_BASE_PTR
#define UART_1	UART1_BASE_PTR
#define UART_2	UART2_BASE_PTR
/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
*define uart ctrl1 type
*
*//*! @addtogroup uart_ctrl1_type
* @{
*******************************************************************************/  

/*!
* @brief UART setting type.
*
*/ 

typedef union
{ 
	uint8_t byte;  /*!< byte field of union type */
	struct{
		uint8_t bPt			:1; 	/*!< Parity type */
		uint8_t bPe 		:1;		/*!< Parity enable */
		uint8_t bIlt 		:1;		/*!< Idle Line Type Select*/
		uint8_t bWake 		:1;		/*!< Receiver Wakeup Method Select*/
		uint8_t bM			:1;		/*!< 9-Bit or 8-Bit Mode Select*/
		uint8_t bRsrc   	:1;    	/*!< Receiver Source Select*/
		uint8_t bUartswai	:1;		/*!< UART Stops in Wait Mode*/
		uint8_t bLoops		:1;		/*!< Loop Mode Select*/
	}bits;   
} UART_Ctrl1Type, *UART_Ctrl1Ptr;
/*! @} End of uart_ctrl1_type */


/******************************************************************************
*define uart ctrl2 type
*
*//*! @addtogroup uart_ctrl2_type
* @{
*******************************************************************************/  

/*!
* @brief UART setting type.
*
*/ 

typedef union
{ 
	uint8_t byte;  /*!< byte field of union type */
	struct{
		uint8_t bSbk		:1; 	/*!< Send Break */
		uint8_t bRwu 		:1;		/*!< Receiver Wakeup Control*/
		uint8_t bRe 		:1;		/*!< Receiver Enable*/
		uint8_t bTe 		:1;		/*!< Transmitter Enable*/
		uint8_t bIlie		:1;		/*!< Idle Line Interrupt Enable for IDLE*/
		uint8_t bRie   		:1;    	/*!< Receiver Interrupt Enable for RDRF*/
		uint8_t bTcie		:1;		/*!< Transmission Complete Interrupt Enable for TC*/
		uint8_t bTie		:1;		/*!< Transmit Interrupt Enable for TDRE*/
	}bits;   
} UART_Ctrl2Type, *UART_Ctrl2Ptr;
/*! @} End of uart_ctrl2_type */


/******************************************************************************
*define uart ctrl3 type
*
*//*! @addtogroup uart_ctrl3_type
* @{
*******************************************************************************/  

/*!
* @brief UART setting type.
*
*/ 

typedef union
{ 
	uint8_t byte;  /*!< byte field of union type */
	struct{
		uint8_t bPeie		:1; 	/*!< Parity Error Interrupt Enable */
		uint8_t bFeie 		:1;		/*!< Framing Error Interrupt Enable*/
		uint8_t bNeie 		:1;		/*!< Noise Error Interrupt Enable*/
		uint8_t bOrie		:1;		/*!< Overrun Interrupt Enable*/
		uint8_t bTxinv		:1;		/*!< Transmit Data Inversion*/
		uint8_t bTxdir   	:1; 	/*!< TxD Pin Direction in Single-Wire Mode*/
	}bits;   
} UART_Ctrl3Type, *UART_Ctrl3Ptr;
/*! @} End of uart_ctrl3_type */


/******************************************************************************
*define uart config type
*
*//*! @addtogroup uart_config_type
* @{
******************************************************************************/
 /*!
 * @brief UART Configuration structure.
 *
 */   
typedef struct 
{
    UART_Ctrl1Type sctrl1settings;          /*!< UART Control 1 settings */
    UART_Ctrl2Type sctrl2settings;          /*!< UART Control 2 settings */
    UART_Ctrl3Type sctrl3settings;          /*!< UART Control 3 settings */
    uint8_t		   bSbns;					/*!< Stop Bit Number Select */
    uint32_t   	   u32SysClkHz;        		/*!< system clock */
    uint32_t       u32Baudrate;        		/*!< UART baudrate */
} UART_ConfigType,*UART_ConfigPtr;
/*! @} End of uart_config_type  */

/******************************************************************************
*define uart config baudrate type
*
*//*! @addtogroup uart_config_baudrate_type
* @{
******************************************************************************/
 /*!
 * @brief UART baudrate type structure.
 *
 */   
typedef struct
{
    uint32_t    u32SysClkHz;        /*!< system clock */
    uint32_t    u32Baudrate;        /*!< UART baudrate */
} UART_ConfigBaudrateType;
/*! @} End of uart_config_baudrate_type */

/******************************************************************************
*define uart config mode type list
*
*//*! @addtogroup uart_mode_type_list
* @{
******************************************************************************/
typedef enum
{
    UART_Mode8Bit,                  /*!< 8 bit mode */
    UART_Mode9Bit,                  /*!< 9 bit mode */
    UART_ModeEnableLoopback,        /*!< enable looback mode */
    UART_ModeDisableLoopback,       /*!< disable loopback mode*/
    UART_ModeEnableSingleWire,      /*!< enable single wire mode */
    UART_ModeDisableSingleWire,     /*!< disable single wire mode */
} UART_ModeType;
/*! @} End of uart_mode_type_list   */

/******************************************************************************
*define uart interrupt type list
*
*//*! @addtogroup uart_interrupt_type_list
* @{
******************************************************************************/

typedef enum
{
    UART_TxBuffEmptyInt,            /*!< transmit buffer empty interrupt */
    UART_TxCompleteInt,             /*!< transmit complete interrupt */
    UART_RxBuffFullInt,             /*!< receive buffer full interrupt */

    UART_IdleLineInt,               /*!< idle line interrupt */

    UART_RxOverrunInt,              /*!< receive overrun interrupt */
    UART_NoiseErrorInt,             /*!< noise error interrupt */
    UART_FramingErrorInt,           /*!< framing error interrupt */
    UART_ParityErrorInt,            /*!< parity error interrupt */
} UART_InterruptType;
/*! @} End of uart_interrupt_type_list  */

/******************************************************************************
*define uart flag type list
*
*//*! @addtogroup uart_flag_type_list
* @{
******************************************************************************/
typedef enum
{
    UART_FlagPF = 0,        /*!< Parity error flag */
    UART_FlagFE,            /*!< Framing error flag */
    UART_FlagNF,            /*!< Noise flag */
    UART_FlagOR,            /*!< Receive overrun */
    UART_FlagIDLE,          /*!< Idle line flag */
    UART_FlagRDRF,          /*!< Receive data register full flag */
    UART_FlagTC,            /*!< Transmission complete flag */
    UART_FlagTDRE,          /*!< Transmit data register flag */

    UART_FlagRAF,           /*!< Receiver active flag */
    UART_FlagLBKDE,         /*!< LIN break detection enable */
    UART_FlagBRK13,         /*!< Break character generation length */ 
    UART_FlagRWUID,         /*!< Receive wake up idle detect */
    UART_FlagRXINV,         /*!< Receive data inversion */
    UART_FlagRev1,          /*!< Reserved */
    UART_FlagRXEDGIF,       /*!< RxD pin active edge interrupt flag */
    UART_FlagLBKDIF,        /*!< LIN break detect interrupt flag */
} UART_FlagType;
/*! @} End of uart_flag_type_list   */

/* callback types */


typedef void (*UART_CallbackType) (void);

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

/******************************************************************************
* define UART APIs
*
*//*! @addtogroup uart_api_list
* @{
*******************************************************************************/

/*****************************************************************************//*!
*
* @brief read receive buffer
*        
* @param[in] pUART       base of UART port
*
* @return unsign char received char
*
*****************************************************************************/
static inline uint8_t UART_ReadDataReg(UART_MemMapPtr pUART)
{
    /* Return the 8-bit data from the receiver */
    return pUART->D;
}
/*****************************************************************************//*!
*
* @brief write transmit buffer
*        
* @param[in] pUART       base of UART port
* @param[in] u8Char      char to send
*
* @return none
*
*****************************************************************************/
static inline void UART_WriteDataReg(UART_MemMapPtr pUART, uint8_t u8Char)
{
    /* Send the character */
    pUART->D = (uint8_t)u8Char;
}

/*****************************************************************************//*!
*
* @brief check if a character has been received
*
* @param[in] pUART  base of UART port
*
* @return 0, No character received; no-zero, Character has been received
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline uint8_t UART_CharPresent(UART_MemMapPtr pUART)
{  
    return (pUART->S1 & UART_S1_RDRF_MASK);
}
/*****************************************************************************//*!
*
* @brief enable transmit
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_EnableTx(UART_MemMapPtr pUART)
{
    
    pUART->C2 |= UART_C2_TE_MASK;
}
/*****************************************************************************//*!
*
* @brief disable transmit
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_DisableTx(UART_MemMapPtr pUART)
{    
    pUART->C2 &= (~UART_C2_TE_MASK);
}
/*****************************************************************************//*!
*
* @brief enable receive
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_EnableRx(UART_MemMapPtr pUART)
{    
    pUART->C2 |= UART_C2_RE_MASK;
}
/*****************************************************************************//*!
*
* @brief disable receive
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_DisableRx(UART_MemMapPtr pUART)
{    
    pUART->C2 &= (~UART_C2_RE_MASK);
}
/*****************************************************************************//*!
*
* @brief Enable loopback mode
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_EnableLoopback(UART_MemMapPtr pUART)
{
    pUART->C1 |= UART_C1_LOOPS_MASK;
    pUART->C1 &= (~UART_C1_RSRC_MASK);
}
/*****************************************************************************//*!
*
* @brief enable single wire mode
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_EnableSingleWire(UART_MemMapPtr pUART)
{
    pUART->C1 |= UART_C1_LOOPS_MASK;
    pUART->C1 |= UART_C1_RSRC_MASK;
}
/*****************************************************************************//*!
*
* @brief set 8-bit mode
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_Set8BitMode(UART_MemMapPtr pUART)
{
    pUART->C1 &= (~UART_C1_M_MASK);
}
/*****************************************************************************//*!
*
* @brief set 9-bit mode
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_Set9BitMode(UART_MemMapPtr pUART)
{
    pUART->C1 |= UART_C1_M_MASK;
}

/*****************************************************************************//*!
*
* @brief set 1 stop bit 
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_Set1StopBit(UART_MemMapPtr pUART)
{
    pUART->BDH &= ~UART_BDH_SBNS_MASK;
}

/*****************************************************************************//*!
*
* @brief set 2 stop bit 
*        
* @param[in] pUART       base of UART port
*
* @return none
*
*****************************************************************************/
static inline void UART_Set2StopBit(UART_MemMapPtr pUART)
{
    pUART->BDH|= UART_BDH_SBNS_MASK;
}

/*****************************************************************************//*!
*
* @brief enable transmit buffer empty interrupt
*        
* @param[in] pUART       base of UART port
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline void UART_EnableTxBuffEmptyInt(UART_MemMapPtr pUART)
{
    pUART->C2 |= UART_C2_TIE_MASK;
}
/*****************************************************************************//*!
*
* @brief enable transmit complete interrupt
*        
* @param[in] pUART       base of UART port
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline void UART_EnableTxCompleteInt(UART_MemMapPtr pUART)
{
    pUART->C2 |= UART_C2_TCIE_MASK;
}
/*****************************************************************************//*!
*
* @brief enable receive buffer full interrupt
*        
* @param[in] pUART       base of UART port
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline void UART_EnableRxBuffFullInt(UART_MemMapPtr pUART)
{
    pUART->C2 |= UART_C2_RIE_MASK;
}
/*****************************************************************************//*!
*
* @brief disable transmit buffer empty interrupt
*        
* @param[in] pUART       base of UART port
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline void UART_DisableTxBuffEmptyInt(UART_MemMapPtr pUART)
{
        pUART->C2 &= (~UART_C2_TIE_MASK);    
}
/*****************************************************************************//*!
*
* @brief disable transmit complete interrupt
*        
* @param[in] pUART       base of UART port
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline void UART_DisableTxCompleteInt(UART_MemMapPtr pUART)
{
    pUART->C2 &= (~UART_C2_TCIE_MASK);   
}
/*****************************************************************************//*!
*
* @brief disable receive buffer full interrupt
*        
* @param[in] pUART       base of UART port
*
* @return none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline void UART_DisableRxBuffFullInt(UART_MemMapPtr pUART)
{
    pUART->C2 &= (~UART_C2_RIE_MASK);  
}
/*****************************************************************************//*!
*
* @brief print out break character
*        
* @param[in] pUART  base of UART port
*
* @return       none
*
* @ Pass/ Fail criteria:
*****************************************************************************/
static inline void UART_PutBreak(UART_MemMapPtr pUART)
{
    /* Write 1 then write 0 to UART_C2[SBK] bit, will put break character */
    pUART->C2 |= UART_C2_SBK_MASK; 
    pUART->C2 &= (~UART_C2_SBK_MASK);
}

/*****************************************************************************//*!
*
* @brief check whether tx is complete,i.e. data has been sent out.
*        
* @param[in] pUART  base of UART port
*
* @return       
*               1, Tx complete flag is set
*               0, Tx complete flag is clear
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline uint8_t UART_IsTxComplete(UART_MemMapPtr pUART)
{
    return (pUART->S1 & UART_S1_TC_MASK);
}
/*****************************************************************************//*!
*
* @brief check whether Tx buffer is empty
*        
* @param[in] pUART  base of UART port
*
* @return       
*               1, Tx buffer is empty
*               0, Tx buffer is not empty
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline uint8_t UART_IsTxBuffEmpty(UART_MemMapPtr pUART)
{
    return (pUART->S1 & UART_S1_TDRE_MASK);
}
/*****************************************************************************//*!
*
* @brief check whether Rx buffer is full, i.e. receive a character
*        
* @param[in] pUART  base of UART port
*
* @return       
*               1, Rx buffer is full
*               0, Rx buffer is not full
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline uint8_t UART_IsRxBuffFull(UART_MemMapPtr pUART)
{
    return (pUART->S1 & UART_S1_RDRF_MASK);
}



/*! @} End of uart_api_list */


/******************************************************************************
* Global functions declaration
******************************************************************************/
void UART_Init(UART_MemMapPtr pUART, UART_ConfigType *pConfig);
uint8_t UART_GetChar(UART_MemMapPtr pUART);
void UART_PutChar(UART_MemMapPtr pUART, uint8_t u8Char);
void UART_SetBaudrate(UART_MemMapPtr pUART, UART_ConfigBaudrateType *pConfig);
void UART_DisableInterrupt(UART_MemMapPtr pUART, UART_InterruptType InterruptType);
uint16_t UART_GetFlags(UART_MemMapPtr pUART);
uint8_t UART_CheckFlag(UART_MemMapPtr *pUART, UART_FlagType FlagType);
void UART_SendWait(UART_MemMapPtr pUART, uint8_t *pSendBuff, uint32_t u32Length);
void UART_ReceiveWait(UART_MemMapPtr pUART, uint8_t *pReceiveBuff, uint32_t u32Length);
void UART_WaitTxComplete(UART_MemMapPtr pUART);
void UART_SetCallback(UART_MemMapPtr pUART, UART_CallbackType pfnCallback);
void UART0_Isr(void);
void UART1_Isr(void);
void UART2_Isr(void);
void UART_SetTxDoneCallback(UART_MemMapPtr *pUART, UART_TxDoneCallbackType pfnCallback);
void UART_SetRxDoneCallback(UART_MemMapPtr *pUART, UART_RxDoneCallbackType pfnCallback);
void UART_HandleInt(UART_MemMapPtr *pUART);
void UART_SendInt(UART_MemMapPtr *pUART, uint8_t *pSendBuff, uint32_t u32Length);




#ifdef __cplusplus
}
#endif
#endif /* #ifndef _UART_H_ */
