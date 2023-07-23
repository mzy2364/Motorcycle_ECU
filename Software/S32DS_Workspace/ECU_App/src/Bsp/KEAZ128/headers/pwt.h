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
* @file     pwt.h
*
* @author   Freescale
*
*
* @brief    Periodic Interrupt Timer (PWT) driver head file. 
*
******************************************************************************/
#ifndef PWT_H_
#define PWT_H_
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* Includes
******************************************************************************/
#include "derivative.h"

#if defined(MCU_SKEAZ1284)
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros
******************************************************************************/

/******************************************************************************
* PWT control bit definition
*
*//*! @addtogroup pwt_controlbit
* @{
*******************************************************************************/

#define PWT_PCLKS_BUS               0 	/*!< select bus clock as PWT clock source  */
#define PWT_PCLKS_ALTCLK            1	/*!< select alternative clock as PWT clock source  */

#define PWT_PINSEL_PTD5             0	/*!< PWTIN[0]  */
#define PWT_PINSEL_PTB0             1	/*!< PWTIN[1]  */
#define PWT_PINSEL_ACMP0_OUTPUT     2	/*!< PWTIN[2]  */
#define PWT_PINSEL_ACMP1_OUTPUT     3	/*!< PWTIN[3]  */

#define FALLING_START_CAPTURE_FALLING           0	/*!< first falling-edge start to measure, all falling-edge capture */
#define RISING_START_CAPTURE_ALL                1	/*!< first rising-edge start to measure, all edge capture */
#define FALLING_START_CAPTURE_ALL               2	/*!< first falling-edge start to measure, all edge capture */
#define RISING_START_CAPTURE_RISING             3	/*!< first rising-edge start to measure, all rising-edge capture */

#define PWT_CLK_PRESCALER_1         0	/*!< presalcer is 1  */
#define PWT_CLK_PRESCALER_2         1	/*!< presalcer is 2  */
#define PWT_CLK_PRESCALER_4         2	/*!< presalcer is 4  */
#define PWT_CLK_PRESCALER_8         3	/*!< presalcer is 8  */
#define PWT_CLK_PRESCALER_16        4	/*!< presalcer is 16 */
#define PWT_CLK_PRESCALER_32        5	/*!< presalcer is 32 */
#define PWT_CLK_PRESCALER_64        6	/*!< presalcer is 64 */
#define PWT_CLK_PRESCALER_128       7	/*!< presalcer is 128 */

/*! @} End of pwt_controlbit                                                  */
        
/******************************************************************************
* Types
******************************************************************************/

/*
 * Callback type
 */

/******************************************************************************
* PWT callback function declaration
*
*//*! @addtogroup pwt_callback
* @{
*******************************************************************************/
typedef void (*PWT_CallbackType)(void);             /*!< PWT callback type */

/*! @} End of pwt_callback                                                    */

/* PWT configuration structure 
 */  
/******************************************************************************
* define PWT configuration structure
*
*//*! @addtogroup pwt_config_type
* @{
*******************************************************************************/

/*!
 * @brief PWT configuration type.
 *
 */
typedef struct
{   
    uint8_t bCounterOverflow         : 1;    /*!< 1: overflow bit is set, 0: rno overflow */
    uint8_t bRegisterReady           : 1;    /*!< 1: register has been updated, 0: no up-to-date */    
    uint8_t bSoftReset               : 1;    /*!< 1: reset PWT, 0: no action */ 
    uint8_t bOverflowIntEn           : 1;    /*!< 1: enable overflow interrupt, 0: disable overflow interrupt */ 
    uint8_t bReadyIntEn              : 1;    /*!< 1: enable data ready interrupt, 0: disable data ready interrupt */   
    uint8_t bPWTIntEn                : 1;    /*!< 1: enable PWT module interrupt, 0: disable PWT module interrupt */    
    uint8_t bPWTEn                   : 1;    /*!< 1: enable PWT module, 0: disable PWT module */      
    uint8_t bClockPrescaler          : 3;    /*!< prescaler value is from 1 to 128 */  
    uint8_t bEdgeSensitivity         : 2;    /*!< select edge triggers */
    uint8_t bPinSelect               : 2;    /*!< input pin select */    
    uint8_t bClockSource             : 1;    /*!< 1: PWT clock source is alternative clock, 0: PWT clock source is bus clock*/    
     
} PWT_ConfigType, *PWT_ConfigPtr; 
/*! @} End of pwt_config_type                                                */

/******************************************************************************
* Global variables
******************************************************************************/


/*!
 * inline functions
 */

/******************************************************************************
* PWT API list
*
*//*! @addtogroup pwt_api_list
* @{
*******************************************************************************/

/*****************************************************************************//*!
*
* @brief enable pwt module.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_Enable(void)
{

    PWT->R1 |= PWT_R1_PWTEN_MASK; 
    
}


/*****************************************************************************//*!
*
* @brief disable pwt module.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_Disable(void)
{

    PWT->R1 &= ~PWT_R1_PWTEN_MASK;
}

/*****************************************************************************//*!
*
* @brief enable pwt module interrupt.
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_EnableInt(void)

{
    PWT->R1 |= PWT_R1_PWTIE_MASK; 
}


/*****************************************************************************//*!
*
* @brief disable pwt module interrupt .
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_DisableInt(void)

{
    PWT->R1 &= ~PWT_R1_PWTIE_MASK;
}


/*****************************************************************************//*!
*
* @brief enable pwt module timer over flow interrupt.
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_EnableOverFlowInt(void)

{
    PWT->R1 |= PWT_R1_POVIE_MASK; 
}


/*****************************************************************************//*!
*
* @brief disable pwt module timer over flow interrupt.
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_DisableOverFlowInt(void)

{
    PWT->R1 &= ~PWT_R1_POVIE_MASK; 
}

/*****************************************************************************//*!
*
* @brief enable pwt module data ready interrupt.
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_EnableReadyInt(void)

{
    PWT->R1 |= PWT_R1_PRDYIE_MASK; 
}


/*****************************************************************************//*!
*
* @brief disable pwt module data ready interrupt.
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_DisableReadyInt(void)

{
    PWT->R1 &= ~PWT_R1_PRDYIE_MASK; 
}

/*****************************************************************************//*!
*
* @brief set pwt module soft reset.
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_SetSoftReset(void)
{
    PWT->R1 |= PWT_R1_PWTSR_MASK; 
}

/*****************************************************************************//*!
*
* @brief set pwt module input pin selct.
*        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_SetPinSelect(uint8_t u8PinSelect)
{
    PWT->R1 &= ~PWT_R1_PINSEL_MASK; 
    PWT->R1 |= PWT_R1_PINSEL(u8PinSelect);     
}


/*****************************************************************************//*!
*
* @brief set pwt module input edge sensitivity.
*        
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_SetEdgeSensitivity(uint8_t u8EdgeSensitivity)
{
    PWT->R1 &= ~PWT_R1_EDGE_MASK; 
    PWT->R1 |= PWT_R1_EDGE(u8EdgeSensitivity);   
}


/*****************************************************************************//*!
*
* @brief set pwt module input clock presacler.
*        
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
static inline void PWT_SetClockPresacler(uint8_t u8ClockPresacler)
{
    PWT->R1 &= ~PWT_R1_PRE_MASK; 
    PWT->R1 |= PWT_R1_PRE(u8ClockPresacler);     
}

/*****************************************************************************//*!
*
* @brief get pwt overflow flag.
*        
*
* @return bOverflowflag.
*
* @ Pass/ Fail criteria: none

*****************************************************************************/
static inline uint8_t PWT_GetOverflowFlag(void)

{
    uint8_t bOverflowflag;  
    
    bOverflowflag = (PWT->R1 & PWT_R1_PWTOV_MASK);    
    
    return bOverflowflag;
    
}


/*****************************************************************************//*!
*
* @brief clear pwt overflow flag.
*        
*
*
* @ Pass/ Fail criteria: none

*****************************************************************************/
static inline void PWT_ClrOverflowFlag(void)

{
    PWT->R1 &= ~PWT_R1_PWTOV_MASK; 
}


/*****************************************************************************//*!
*
* @brief get pwt ready flag.
*        
*
* @return bflag.
*
* @ Pass/ Fail criteria: none

*****************************************************************************/
static inline uint8_t PWT_GetReadyFlag(void)

{
    uint8_t bReadyflag;  
    
    bReadyflag = (PWT->R1 & PWT_R1_PWTRDY_MASK);    
    
    return bReadyflag;  
}


/*****************************************************************************//*!
*
* @brief clear pwt ready flag.
*        
*
*
* @ Pass/ Fail criteria: none

*****************************************************************************/
static inline void PWT_ClrReadyFlag(void)

{  
    PWT->R1 &= ~PWT_R1_PWTRDY_MASK;
}

/******************************************************************************
* Global functions
******************************************************************************/
void PWT_Init(PWT_ConfigType *pConfig);
void PWT_SetCallback(PWT_CallbackType pfnCallback);
void PWT_DeInit(void);
/*! @} End of pwt_api_list                                                   */

#ifdef __cplusplus
}
#endif
#elif defined(MCU_SKEAZN642)
/* PWT is not supported in KEAZN642. */
#else
/* If your device supports PWT, enable it and add any additional PWT features. */
#endif
#endif /* PWT_H_ */
