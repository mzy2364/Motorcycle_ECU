/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2014 Freescale Semiconductor, Inc.
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
* @file kbi.c
*
* @author Freescale
*
* @brief providing APIs for configuring KBI. 
*
*******************************************************************************
*
* provide APIs for configuring KBI
******************************************************************************/

#include "kbi.h"
/******************************************************************************
* External objects
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
KBI_CallbackType KBI_Callback[KBI_MAX_NO] = {(KBI_CallbackType)(0)};
uint32_t u8Port,sc;
/******************************************************************************
* Constants and macros
******************************************************************************/
#if defined(MCU_SKEAZ1284)
#define KBI_MAX_PINS_PER_PORT   32                  /*!< max number of pins */
#elif defined(MCU_SKEAZN642)
#define KBI_MAX_PINS_PER_PORT   8                  /*!< max number of pins */
#else
/* Define the number of pins per KBI port of your device. */
#endif
//#define KBI_Type KBI_MemMapPtr
/******************************************************************************
* Local types
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Local variables
******************************************************************************/
static KBI_Type* pKBI0 = (KBI_Type*)KBI0;
static KBI_Type* pKBI1 = (KBI_Type*)KBI1;
/******************************************************************************
* Local functions
******************************************************************************/
/******************************************************************************
* KBI api list
*
*//*! @addtogroup kbi_api_list
* @{
*******************************************************************************/


/******************************************************************************
* Global functions
******************************************************************************/

/*****************************************************************************//*!
*
* @brief initialize KBI module.
*        
* @param[in]  pKBI        pointer to KBI module.
* @param[in]  pConfig     pointer to KBI configuration structure.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see KBI_DeInit.
*
*****************************************************************************/
void KBI_Init(KBI_Type* pKBI, KBI_ConfigType *pConfig)
{	
     uint32_t    i;
     uint32_t     sc = 0;
     uint32_t     u8Port;
     uint32_t     u8PinPos;

     uint32_t    u16PinMapping[KBI_MAX_NO][KBI_MAX_PINS_PER_PORT] =
    {
#if defined(MCU_SKEAZ1284)
        {/* KBI0P0~KBI0P31 pins position in GPIOA register */
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31           
        },
        {/* KBI1P0~KBI1P31 pins position in GPIOB register */
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31		   
        }
#elif defined(MCU_SKEAZN642)
		{/* KBI0P0~KBI0P7 pins position in GPIOA register */
            0, 1, 2, 3, 8, 9, 10, 11
        },
        {/* KBI1P0~KBI1P7 pins position in GPIOA register */
			24, 25, 26, 27, 28, 29, 30, 31
        }
#else
/* Define the KBI pins of your device. */
#endif
    };
 
 
    
    if(pKBI0 == pKBI)
    {
        SIM->SCGC   |= SIM_SCGC_KBI0_MASK;             /* enable clock to KBI0 */\
         u8Port      =  0;
    }
    else if (pKBI1 == pKBI)
    {        
        SIM->SCGC   |= SIM_SCGC_KBI1_MASK;             /* enable clock to KBI1 */
        u8Port      =  1;
    }
    
    /* mask keyboard interrupts first */
     sc          = pConfig->sBits.bMode;
    pKBI->SC    = sc;

    /* configure KBI pin polarity and others */
    for (i = 0; i < KBI_MAX_PINS_PER_PORT; i++)
    {
        if(pConfig->sPin[i].bEn)
        {
            pKBI->PE    |= (1<<i);                      /* enable this KBI pin */
            pKBI->ES    = (pKBI->ES & ~(1<<i)) | (pConfig->sPin[i].bEdge << i);     
            u8PinPos = u16PinMapping[u8Port][i];
            
		   if (u8Port == 0)   /* KBI0 */
           {
		   	FGPIOA->PIDR  &= ~(1<<u8PinPos);              /* enable GPIO input */    
            FGPIOA->PDDR  &= ~(1<<u8PinPos);              /* configure pin as input */ 
           }
		   else if (u8Port == 1)   /* KBI1 */
           {
		   	FGPIOB->PIDR  &= ~(1<<u8PinPos);              /* enable GPIO input */     
            FGPIOB->PDDR  &= ~(1<<u8PinPos);              /* configure pin as input */ 
           }
		
		}
    }
    
#if defined(MCU_SKEAZ1284)
    /*Reset KBI_SP register*/
	sc = pConfig->sBits.bRstKbsp<<KBI_SC_RSTKBSP_SHIFT;
	pKBI->SC    |= sc;
	
    /*Real KBI_SP register enable*/
	sc = pConfig->sBits.bKbspEn<<KBI_SC_KBSPEN_SHIFT;
	pKBI->SC    |= sc;
#elif defined(MCU_SKEAZN642)
	/* KEAZN64 family has neither RSTKBSP or KBSPEN fields because those registers do not exist in the family's KBI peripheral. */
#else
	/* If your device has more KBI_SC fields add them here. */
#endif
	
  
	/* write to KBACK to clear any false interrupts */
    pKBI->SC    = sc;
    
    /* enable interrupt if needed */
    if(pConfig->sBits.bIntEn)
    {
        pKBI->SC    |=  KBI_SC_KBIE_MASK;
        
        if(pKBI0 == pKBI)
        {
            Enable_Interrupt(KBI0_IRQn);
        }
        if(pKBI1 == pKBI)
        {
          
            Enable_Interrupt(KBI1_IRQn);
        }
    }
}

/*****************************************************************************//*!
*
* @brief set up KBI callback routine.
*        
* @param[in] pKBI          pointer to KBI module.
* @param[in] pfnCallback   pointer to callback routine.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void KBI_SetCallback(KBI_Type* pKBI, KBI_CallbackType pfnCallback)
{
    if(pKBI0 == pKBI)
    {
        KBI_Callback[0] = pfnCallback;
    }
    else
    {
        KBI_Callback[1] = pfnCallback;        
    }
}

/*****************************************************************************//*!
*
* @brief deinit the kbi module.
*        
* @param[in]  pKBI       pointer to KBI module.
*         
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see KBI_Init.
*
*****************************************************************************/
void KBI_DeInit(KBI_Type* pKBI)
{
    if(pKBI0 == pKBI)
    {
        
        Disable_Interrupt(KBI0_IRQn);
    }
    else
    {
    	 Disable_Interrupt(KBI1_IRQn);
    }
    
    pKBI->PE = 0;
    pKBI->SC = 0;
    pKBI->ES = 0;
    
    if(pKBI0 == pKBI)
    {
        SIM->SCGC   &= ~SIM_SCGC_KBI0_MASK;             /* disable clock to KBI0 */
    }
    else 
    {
        SIM->SCGC   &= ~SIM_SCGC_KBI1_MASK;             /* disable clock to KBI1 */       
    }
}

/*! @} End of acmp_api_list                                                  */

/*****************************************************************************//*!
*
* @brief button group 0 (KBI0) interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/

void KBI0_IRQHandler(void)	
{
  KBI_MemMapPtr pKBI = (KBI_MemMapPtr)KBI0;
  pKBI->SC |= KBI_SC_KBACK_MASK;                        /* clear interrupt flag */

  if(KBI_Callback[0])
  {
      KBI_Callback[0]();
  }
}



/*****************************************************************************//*!
*
* @brief button group 0 (KBI0) interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/

void KBI1_IRQHandler(void)	
{
  KBI_MemMapPtr pKBI = (KBI_MemMapPtr)KBI1;
  pKBI->SC |= KBI_SC_KBACK_MASK;                        /* clear interrupt flag */
 
  if(KBI_Callback[1])
  {
      KBI_Callback[1]();
  }
}

