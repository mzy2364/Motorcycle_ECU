/*****************************************************************************
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
* @file ics.c
*
* @author Freescale
*
* @brief providing APIs for configuring internal clock sources (ICS). 
*
*******************************************************************************
*
* provide APIs for configuring internal clock sources (ICS)
******************************************************************************/
#include "ics.h"

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Constants and macros
******************************************************************************/

/* default value OSC register after reset */
#define OSC_CR_DEFAULT  0


/******************************************************************************
* Local types
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/


/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local functions
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/


/*****************************************************************************//*!
   *
   * @brief change clock from FEI with IRC trim from factory to FEI with IRC trim by using
   *  a custom trim value provided by a programming tool.
   *         
   * @param[in] pConfig pointer to the ICS configuration structure
   *
   * @return none
   *
   * @Pass/ Fail criteria: none
   *****************************************************************************/
void FEI_factory_to_FEI_custom(ICS_ConfigType *pConfig)
{
	/* Enable the internal reference clock*/ 
	ICS_C1|=ICS_C1_IRCLKEN_MASK; 		
	
	/* trim internal refernce clock*/
	ICS_C3 = *((uint8_t*) 0x03FF); 
	
	/*fine trim internal reference clock*/
	ICS_C4 = *((uint8_t*) 0x03FE); 
	
	/* Wait for FLL lock, now running at 1280 * IRC user trim */	
	while(!(ICS_S & ICS_S_LOCK_MASK));      	
	
#if defined(MCU_SKEAZ1284)	
	ICS_S |= ICS_S_LOCK_MASK ; 	       /* Clear Loss of lock sticky bit */
#elif defined(MCU_SKEAZN642)
/* This instruction causes errors in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
    
    /*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
    SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif


}

/*****************************************************************************//*!
   *
   * @brief change clock from FEI mode to FEE mode and divide clock by 2.
   *        
   * @param[in] pConfig pointer to the ICS configuration structure
   *
   * @return none
   *
   * @Pass/ Fail criteria: none
   *****************************************************************************/
void FEI_to_FEE(ICS_ConfigType *pConfig)
{
	/* enable OSC
	 * 
	 */
	OSC_Init(&pConfig->oscConfig); /* enable OSC */
	
	/* divide down external clock frequency to be within 31.25K to 39.0625K
	 * 
	 */
    ICS_SetOscDivider(pConfig->oscConfig.u32OscFreq);

	/* change FLL reference clock to external clock */
	  ICS_C1 =  ICS_C1 & ~ ICS_C1_IREFS_MASK ;     
	
	/* wait for the reference clock to be changed to external */     
	while(ICS_S & ICS_S_IREFST_MASK);
	
	/* wait for FLL to lock */
	while(!(ICS_S & ICS_S_LOCK_MASK));
		
	/* now FLL output clock is FLL reference clock* FLL multiplication factor. 
	 * See Reference Manual for multiplication factor.
	 */

	/* clear Loss of lock sticky bit */
#if defined(MCU_SKEAZ1284)	
	ICS_S |= ICS_S_LOLS_MASK;	
#elif defined(MCU_SKEAZN642)
/* This instruction causes errors in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	/* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
	
#if defined(MCU_SKEAZ1284)	
    SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  /*Core frequency divide by 2 for Bus freq*/
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;  /*Core frequency divide by 2 for Bus freq*/
#else
/* Write the appropriate instruction for your device. */
#endif
}

/*****************************************************************************//*!
   *
   * @brief change clock from FEI mode to FBI mode and divide clock by 2.
   *        
   * @param  pConfig    pointer to configuration strucure. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FEI_to_FBI(ICS_ConfigType *pConfig)
{
	/* change clock source to internal reference clock */
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK)) | ICS_C1_CLKS(1);   
	ICS_C2 = ICS_C2 & ~(ICS_C2_LP_MASK); 
	/* wait for the reference clock to be changed  */       
	while(((ICS_S & ICS_S_CLKST_MASK) >> ICS_S_CLKST_SHIFT) !=1);
			
	/* now internal reference clock is the system clock*/        
	/* clear Loss of lock sticky bit */
#if defined(MCU_SKEAZ1284)	
	ICS_S |= ICS_S_LOLS_MASK;	
#elif defined(MCU_SKEAZN642)
/* This line causes errors in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
	
	/*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}

/*****************************************************************************//*!
   *
   * @brief change clock from FEI mode to FBE mode and divide clock by 2.
   *        
   * @param  pConfig    pointer to configuration strucure. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FEI_to_FBE(ICS_ConfigType *pConfig)
{
	OSC_Init(&pConfig->oscConfig); /* enable OSC */

	/* change clock source to external reference clock */
    ICS_C1 =  ICS_C1 & ~(ICS_C1_IREFS_MASK);
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK)) | ICS_C1_CLKS(2);   
	ICS_C2 = ICS_C2 & ~(ICS_C2_LP_MASK); 
        
	/* wait for the reference clock to be changed  */       
	while(((ICS_S & ICS_S_CLKST_MASK) >> ICS_S_CLKST_SHIFT) !=2);
    while(ICS_S & ICS_S_IREFST_MASK);
			
	/* clear Loss of lock sticky bit */
#if defined(MCU_SKEAZ1284)
	ICS_S |= ICS_S_LOLS_MASK;	
#elif defined(MCU_SKEAZN642)
/* This line causes errors in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
	
	/*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}


/*****************************************************************************//*!
   *
   * @brief change clock from FEE mode to FEI mode.
   *        
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FEE_to_FEI(ICS_ConfigType *pConfig)
{
	/* select internal reference for FLL */ 
    ICS_C1 =  ICS_C1 | (ICS_C1_IREFS_MASK);
	
	/* wait for the reference clock to be changed */     
	while(!(ICS_S & ICS_S_IREFST_MASK));
	
	/* wait for FLL to lock */
	while(!(ICS_S & ICS_S_LOCK_MASK));

	/* clear Loss of lock sticky bit */
#if defined(MCU_SKEAZ1284)
	ICS_S |= ICS_S_LOLS_MASK;
#elif defined(MCU_SKEAZN642)
/* This line causes problems in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif

	/* now FLL output clock is target frequency*/
	/* disable OSC */
     OSC_Disable();            
      
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
	
     /*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
     SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK; 
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}

/*****************************************************************************//*!
   *
   * @brief change clock from FEE mode to FBI mode.
   *        
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FEE_to_FBI(ICS_ConfigType *pConfig)
{
#if defined(MCU_SKEAZ1284)
	/* clear Loss of lock sticky bit */	
	ICS_S |= ICS_S_LOLS_MASK;
#elif defined(MCU_SKEAZN642)
/* This line causes errors in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	/* select internal clock as clock source */
	/* select internal reference for FLL */ 
	/* LP = 0 */

    ICS_C1 =  ICS_C1 | (ICS_C1_IREFS_MASK);
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK)) | ICS_C1_CLKS(1);   
	
	/* wait for the reference clock to be changed */ 
	while(!(ICS_S & ICS_S_IREFST_MASK));
	while(((ICS_S & ICS_S_CLKST_MASK) >> ICS_S_CLKST_SHIFT) !=1);
	/* disable OSC */
    OSC_Disable(); 
    
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
    
    /*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
    SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}


/*****************************************************************************//*!
   *
   * @brief change clock from FEE mode to FBE mode.
   *        
   * @param  pConfig     pointer to configuration .
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FEE_to_FBE(ICS_ConfigType *pConfig)
{
#if defined(MCU_SKEAZ1284)
	/* clear Loss of lock sticky bit */
	ICS_S |= ICS_S_LOLS_MASK;
#elif defined(MCU_SKEAZN642)
/* This line causes errors for KEZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	/* select the external clock as clock source */
    /* LP = 0 */
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK)) | ICS_C1_CLKS(2);   
	ICS_C2 = ICS_C2 & ~(ICS_C2_LP_MASK); 
	
	/* wait for the clock source to be changed */
	while(((ICS_S & ICS_S_CLKST_MASK) >> ICS_S_CLKST_SHIFT) !=2);
	/* now system clock source is external clock*/
	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
   
#if defined(MCU_SKEAZ1284)
   /*Core frequency divide by 2 for Bus freq*/
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}


/*****************************************************************************//*!
   *
   * @brief change clock from FBI mode to FBE mode.
   *        
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBI_to_FBE(ICS_ConfigType *pConfig)
{
	OSC_Init(&pConfig->oscConfig); /* enable OSC */
    
	/* select external reference clock */
	/* select the external clock as clock source */

    ICS_C1 =  ICS_C1 & ~(ICS_C1_IREFS_MASK);
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK)) | ICS_C1_CLKS(2);   
	
	/* wait for the clock source to be changed */  
	while(((ICS_S & ICS_S_CLKST_MASK) >> ICS_S_CLKST_SHIFT) !=2);
	while((ICS_S & ICS_S_IREFST_MASK));
	
	/* now system clock source is external clock */
	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
	
	/*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif

}


/*****************************************************************************//**
   *
   * @brief change clock from FBI mode to FEE mode.
   *        
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBI_to_FEE(ICS_ConfigType *pConfig)
{
	OSC_Init(&pConfig->oscConfig); /* enable OSC */
	

	/* select external reference clock */
	/* select the FLL output as clock source */
    ICS_C1 =  ICS_C1 & ~(ICS_C1_IREFS_MASK);
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK));   
	
	/* change RDIV  reference divider to divide reference clock to be with FLL input spec
	 * 
	 */
    ICS_SetOscDivider(pConfig->oscConfig.u32OscFreq);
  
	
	/* wait for the clock source to be changed */
	while((ICS_S & ICS_S_CLKST_MASK));
	while((ICS_S & ICS_S_IREFST_MASK));
	
	/* wait for FLL to lock */
	while(!(ICS_S & ICS_S_LOCK_MASK));
	/* now system clock source is external clock*/
#if defined(MCU_SKEAZ1284)
	/* clear Loss of lock sticky bit */
	ICS_S |= ICS_S_LOLS_MASK;
#elif defined(MCU_SKEAZN642)
/* This line causes problems in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif	
	

	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
   
	/*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}

/*****************************************************************************//*!
   *
   * @brief change clock from FBI mode to FBILP mode. 
   * 
   * @param  pConfig     pointer to configuration 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @warning   must run without debug interface attached.
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBI_to_FBILP(ICS_ConfigType *pConfig)
{
	/* assume external crystal is 8Mhz or 4MHz
	 * 
	 */
	ICS_C2 |= ICS_C2_LP_MASK;	/* enter low power mode */
}


/*****************************************************************************//*!
   *
   * @brief change clock from FBI mode to FEI mode. 
   * 
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBI_to_FEI(ICS_ConfigType *pConfig)
{
	/* select internal reference clock */
	/* select the FLL output as clock source */
    ICS_C1 =  ICS_C1 | (ICS_C1_IREFS_MASK);
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK));   
	
	/* wait for the clock source to be changed */     
	while((ICS_S & ICS_S_CLKST_MASK));
	while(!(ICS_S & ICS_S_IREFST_MASK));

        
	/* now system clock source is FLL output*/
#if defined(MCU_SKEAZ1284)
	/* clear Loss of lock sticky bit */
	ICS_S |= ICS_S_LOLS_MASK;	
#elif defined(MCU_SKEAZN642)
/* This line causes problems in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
	
	/*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}



/*****************************************************************************//*!
   *
   * @brief change clock from FBE mode to FBI mode.
   *        
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBE_to_FBI(ICS_ConfigType *pConfig)
{
	/* select internal reference clock */
	/* select the internal clock as clock source */
    ICS_C1 =  ICS_C1 | (ICS_C1_IREFS_MASK);
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK)) | ICS_C1_CLKS(1);   
	
	/* wait for the clock source to be changed */ 
	while(((ICS_S & ICS_S_CLKST_MASK) >> ICS_S_CLKST_SHIFT) != 1);
	while(!(ICS_S & ICS_S_IREFST_MASK));
	
	/* now system clock source is internal clock*/
        
    /* Disable OSC to save power
     */
    OSC_Disable();
    
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
	
	/*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/*Write the appropriate instruction for your device. */
#endif
}

/*****************************************************************************//*!
   *
   * @brief change clock from FBE mode to FEE mode.
   *        
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBE_to_FEE(ICS_ConfigType *pConfig)
{
	/* select the FLL output as clock source */
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK));  
	ICS_C1= (ICS_C1 & ~(ICS_C1_IREFS_MASK) );     
	
	/* wait for the clock source to be changed */
	while(ICS_S & ICS_S_CLKST_MASK);
	/* now system clock source is FLL output*/

#if defined(MCU_SKEAZ1284)
	/* clear Loss of lock sticky bit */
	ICS_S |= ICS_S_LOLS_MASK;
#elif defined(MCU_SKEAZN642)
/* This line causes errors in KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
	
	 /* Set core frequency*/
	ICS_SetBusDivider(pConfig->bdiv);
   
	/*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
   SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}


/*****************************************************************************//*!
   *
   * @brief change clock from FBE mode to FEI mode.
   *        
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBE_to_FEI(ICS_ConfigType *pConfig)
{
	/* select internal reference clock */
	/* select the internal clock as clock source */

    ICS_C1 =  ICS_C1 | (ICS_C1_IREFS_MASK);
	ICS_C1 = (ICS_C1 & ~(ICS_C1_CLKS_MASK));   
	
	/* wait for the clock source to be changed */
	while((ICS_S & ICS_S_CLKST_MASK));
	while(!(ICS_S & ICS_S_IREFST_MASK));
	
	/* now system clock source is internal clock*/
#if defined(MCU_SKEAZ1284)
	/* clear Loss of lock sticky bit */
	ICS_S |= ICS_S_LOLS_MASK;	
#elif defined(MCU_SKEAZN642)
/* This line causes errors for KEAZN64. */
#else
/* Write the appropriate instruction for your device. */
#endif
        
    /* Disable OSC to save power
     */
    OSC_Disable();
    

	 /* Set core frequency, bus frequency = core freq /2*/
    ICS_SetBusDivider(pConfig->bdiv);
    
    /*Core frequency divide by 2 for Bus freq*/
#if defined(MCU_SKEAZ1284)
    SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK; 
#elif defined(MCU_SKEAZN642)
	SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
}



/*****************************************************************************//*!
   *
   * @brief change clock from FBE mode to FBELP mode. 
   *
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBE_to_FBELP(ICS_ConfigType *pConfig)
{
	/* enter low power mode */
 	ICS_C2 = ICS_C2 | (ICS_C2_LP_MASK); 
}


/*****************************************************************************//*!
   *
   * @brief change clock from FBELP mode to FBE mode. 
   * 
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBELP_to_FBE(ICS_ConfigType *pConfig)
{
	/* exit low power mode */
 	ICS_C2 = ICS_C2 & ~(ICS_C2_LP_MASK); 
}

/*****************************************************************************//*!
   *
   * @brief change clock from FBILP mode to FBI mode. 
   * 
   * @param  pConfig     pointer to configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   *****************************************************************************/
void FBILP_to_FBI(ICS_ConfigType *pConfig)
{
	/* exit low power mode */
	ICS_C2 = ICS_C2 & ~(ICS_C2_LP_MASK); 
}

/******************************************************************************
* define ICS API list
*
*//*! @addtogroup ics_api_list
* @{
*******************************************************************************/


/*****************************************************************************//*!
   *
   * @brief  trim internal clock (IRC). 
   * 
   * @param  u16TrimValue    trim value 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/

void ICS_Trim(uint16_t u16TrimValue)
{
   ICS_C3 =  (uint8_t) u16TrimValue;
   ICS_C4 = (ICS_C4 & ~(ICS_C4_SCFTRIM_MASK)) | ((u16TrimValue>>8) & 0x01);
   while(!(ICS_S & ICS_S_LOCK_MASK));    
}


/*****************************************************************************//*!
   *
   * @brief set oscillator divider so that the reference clock for FLL/PLL is within 
   *    spec.
   *
   * @param  u32OscFreqKHz  oscillator clock frequency in KHz.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/

void ICS_SetOscDivider(uint32_t u32OscFreqKHz)
{
    
    switch(u32OscFreqKHz)
    {
        case 8000L:
        case 10000L:
            /* 8MHz or 10MHz */
            ICS_C1 = (ICS_C1 & ~(ICS_C1_RDIV_MASK)) | ICS_C1_RDIV(3);	/* now the divided frequency is 8000/256 = 31.25K */
                                                                        /* now the divided frequency is 10000/256 = 39.0625K */
            break;
        case 4000L:
            /* 4MHz */
            ICS_C1 = (ICS_C1 & ~(ICS_C1_RDIV_MASK)) | ICS_C1_RDIV(2);	/* now the divided frequency is 4000/128 = 31.25K */
            break;
        case 16000L:
            /* 16MHz */
            ICS_C1 = (ICS_C1 & ~(ICS_C1_RDIV_MASK)) | ICS_C1_RDIV(4);	/* now the divided frequency is 16000/512 = 31.25K */
            break;
        case 20000L:
            /* 20MHz */
            ICS_C1 = (ICS_C1 & ~(ICS_C1_RDIV_MASK)) | ICS_C1_RDIV(4);     /* now the divided frequency is 20000/512 = 39.0625K */
            break;
        case 32L:
            /* 32KHz */
            ICS_C1  &= ~(ICS_C1_RDIV_MASK);
            break;
        default:
            break;
    }
}



/*****************************************************************************//*!
   *
   * @brief initialize ICS to the desired clock as defined in BUS_CLK_HZ.
   * 
   * @param  pConfig    pointer to ICS configuration.  
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_ConfigType
   **********************************************************************************************************************/

void ICS_Init(ICS_ConfigType *pConfig)
{
  if(pConfig->u8ClkMode == ICS_CLK_MODE_FEE)
  {    
      
        FEI_to_FEE(pConfig);                     /* switch to FEE mode with external crystal  */
     
  }
  
  else if (pConfig->u8ClkMode == ICS_CLK_MODE_FBE) 	   
  {    
	         
        FEI_to_FBE(pConfig);
       
  }
  else if (pConfig->u8ClkMode == ICS_CLK_MODE_FEI)
  {
	   ICS_C1 |=ICS_C1_IREFS_MASK;           /* Selection of intern clock to FLL */
	   ICS_C1|=ICS_C1_IRCLKEN_MASK; 		/* Enable the internal reference clock*/ 
	   ICS_C1 |=ICS_C1_CLKS(0b00);          /* Clock source select   Output FLL is selected */
	   ICS_SetBusDivider(pConfig->bdiv);	 /* Set core frequency, bus frequency = core freq /2*/
#if defined(MCU_SKEAZ1284)
       SIM_CLKDIV|=SIM_CLKDIV_OUTDIV2_MASK;  /*Core frequency divide by 2 for Bus freq*/  
#elif defined(MCU_SKEAZN642)
		SIM_BUSDIV |=SIM_BUSDIV_BUSDIV_MASK;
#else
/* Write the appropriate instruction for your device. */
#endif
	   
    } 
  
  else if (pConfig->u8ClkMode == ICS_CLK_MODE_FEI_CUSTOM)
  {
	  FEI_factory_to_FEI_custom(pConfig);
	  
  }
  
  else if (pConfig->u8ClkMode == ICS_CLK_MODE_FBI)
   { 
	   FEI_to_FBI(pConfig);		/* Switch to FBI mode*/
   
   }
  

}

/*! @} End of ics_api_list                                                    					*/


/******************************************************************************
* define OSC API list
*
*//*! @addtogroup osc_api_list
* @{
*******************************************************************************/

/*****************************************************************************//*!
   *
   * @brief initialize XOSC with given parameters: GAIN, RANGE in control structure.
   * 
   * @param  pConfig    pointer to OSC configuration. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void OSC_Init(OSC_ConfigType *pConfig)
{
	uint8_t  cr = 0;
	/* 
	 * 
	 */
	if(pConfig->bGain)
	{
		/* high gain */
		cr |= OSC_CR_HGO_MASK ;		/* Rs must be added and be large up to 200K */
	}
	
	if(pConfig->bRange)
	{
		cr |= OSC_CR_RANGE_MASK; /* high range */	
	}

	if(pConfig->bStopEnable)
	{
		cr |= OSC_CR_OSCSTEN_MASK; 
	}

	if(pConfig->bIsCryst)
	{
		cr |= OSC_CR_OSCOS_MASK;
	}

	if(pConfig->bEnable)
	{
		cr |= OSC_CR_OSCEN_MASK;
	}
    
    OSC_CR = cr;
    
	if(pConfig->bWaitInit)
	{

		/* wait for OSC to be initialized
		 * 
		 */   
		while(!(OSC_CR & OSC_CR_OSCINIT_MASK));
		
	}
}

/*****************************************************************************//*!
   *
   * @brief initialize OSC to the default state.
   * 
   * @param  none      
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   * @see   ICS_Init
   *****************************************************************************/

void OSC_DeInit(void)
{
    OSC_CR = OSC_CR_DEFAULT;
}


/*! @} End of osc_api_list                                                    					*/




