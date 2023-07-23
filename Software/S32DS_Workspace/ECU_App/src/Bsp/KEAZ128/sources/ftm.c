
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
* @file ftm.c
*
* @author Freescale
*
*
* @brief providing APIs for configuring FTM. 
*
*******************************************************************************
*
* provide APIs for configuring FTM
******************************************************************************/

#include "ftm.h"

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
static FTM_Type* pFTM0 = (FTM_Type*)FTM0;
static FTM_Type* pFTM1 = (FTM_Type*)FTM1;
static FTM_Type* pFTM2 = (FTM_Type*)FTM2;

/******************************************************************************
* Local functions
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
FTM_CallbackPtr FTM_Callback[3] = {(FTM_CallbackPtr)(0)};


/******************************************************************************
* FTM api lists
*
*//*! @addtogroup ftm_api_list
* @{
*******************************************************************************/
/*******************************************************************************//*!
*
* @brief set the ftm moule clock source and prescale.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    ClockSource           ftm clock source.
* @param[in]    ClockPrescale         prescale factor.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_ClockSet(FTM_Type* pFTM, uint8_t u8ClockSource, uint8_t u8ClockPrescale)
{
    uint8_t   u8Temp;
    u8Temp  = (pFTM->SC & 0xE0);
    u8Temp |= (FTM_SC_CLKS(u8ClockSource & 0x3) | FTM_SC_PS(u8ClockPrescale & 0x7));
    pFTM->SC = u8Temp;
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM_No to PWM mode.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    PWMModeSelect         select CPWM , EPWM  or combine pwm mode.
* @param[in]    PWMEdgeSelect         select high true or low true pulse.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_PWMInit(FTM_Type* pFTM, uint8_t u8PWMModeSelect, uint8_t u8PWMEdgeSelect)
{
    uint8_t   channels, i;
    
    /* open the clock gate */
	if (pFTM0 == pFTM)
    {
        channels = 2;
        SIM->SCGC |= SIM_SCGC_FTM0_MASK;
    }
    else if(pFTM1 == pFTM)
    {
        channels = 2;
    }        
    else
    {
        channels = 6;
        SIM->SCGC  |= SIM_SCGC_FTM2_MASK;
    }
    
    pFTM->SC  = 0x0;                                    /* disable counter */  
	pFTM->MOD = FTM_MOD_INIT; 
    
    if(FTM_PWMMODE_CENTERALLIGNED == u8PWMModeSelect)   /* enable CPWM */
    {
        pFTM->SC |= FTM_SC_CPWMS_MASK; 
    }
    else if(FTM_PWMMODE_COMBINE == u8PWMModeSelect)     /* enable combine pwm mode */
    {
   
        pFTM->MODE    |= FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK;
        pFTM->COMBINE = FTM_COMBINE_COMBINE0_MASK | FTM_COMBINE_COMP0_MASK | FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_DTEN0_MASK |
                        FTM_COMBINE_COMBINE1_MASK | FTM_COMBINE_COMP1_MASK | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_DTEN1_MASK |
                        FTM_COMBINE_COMBINE2_MASK | FTM_COMBINE_COMP2_MASK | FTM_COMBINE_SYNCEN2_MASK | FTM_COMBINE_DTEN2_MASK 
                        ;     
        pFTM->SC &= ~FTM_SC_CPWMS_MASK; 
    }
    if(FTM_PWM_HIGHTRUEPULSE == u8PWMEdgeSelect)
    {
        /* Configure timers PWM High True Pulses */
        for(i=0; i<channels; i++)
        {
            pFTM->CONTROLS[i].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  
            pFTM->CONTROLS[i].CnV  = FTM_C0V_INIT + i*100; 
        }
    }
    else if(FTM_PWM_LOWTRUEPULSE == u8PWMEdgeSelect)
    {
        /* Configure timers for PWM Low True Pulses */
        for(i=0; i<channels; i++) 
        {
            pFTM->CONTROLS[i].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK; 
            pFTM->CONTROLS[i].CnV  = FTM_C0V_INIT + i*100 ; 
        }
    }  
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM_No to input capture mode, enable interrupt.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    Channel               channel number to be configured.
* @param[in]    CaptureMode           select capture edge: rising, falling or both.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_InputCaptureInit(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8CaptureMode)
{ 

    /* open the clock gate */
	if ((pFTM0 == pFTM) && (u8FTM_Channel < 2))
    {
        SIM->SCGC |= SIM_SCGC_FTM0_MASK;
        Enable_Interrupt(FTM0_IRQn);
    }
    else if((pFTM1 == pFTM)  && (u8FTM_Channel < 2))
    {
        SIM->SCGC |= SIM_SCGC_FTM1_MASK;
        Enable_Interrupt(FTM1_IRQn);
    }        
    else 
    {
        SIM->SCGC |= SIM_SCGC_FTM2_MASK;
        Enable_Interrupt(FTM2_IRQn);
    }
    
    pFTM->SC  = 0x0;     /* disable counter */ 
    pFTM->MOD = 0xFFFF;  /* free running */
    
    if(FTM_INPUTCAPTURE_RISINGEDGE == u8CaptureMode)        /* enable interrupt, Capture on rising edge */
    {
        pFTM->CONTROLS[u8FTM_Channel].CnSC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK;
    }
    else if(FTM_INPUTCAPTURE_FALLINGEDGE == u8CaptureMode)  /* Capture on falling edge */
    {
        pFTM->CONTROLS[u8FTM_Channel].CnSC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSB_MASK;
    }
    else if(FTM_INPUTCAPTURE_BOTHEDGE == u8CaptureMode)     /* Capture on rising or falling edge */
    {
        pFTM->CONTROLS[u8FTM_Channel].CnSC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK;       
    }
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM_No to Dual Edge Capture mode to measure the
*        width or the period  of a pulse.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    ChannelPair           ChannelPair number to be configured: 0, 2, 4.
* @param[in]    CaptureMode           select capture edge: one shot and continuous mode.
* @param[in]    Channel_N_Edge        channel N detect edge. 
* @param[in]    Channel_Np1_Edge      channel N+1 detect edge.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_DualEdgeCaptureInit(FTM_Type* pFTM, uint8_t u8ChannelPair, uint8_t u8CaptureMode, 
                                 uint8_t u8Channel_N_Edge, uint8_t u8Channel_Np1_Edge)
{ 
    
    SIM->SCGC |= SIM_SCGC_FTM2_MASK;
    if((0 == u8ChannelPair) || (2== u8ChannelPair))
    {
                                        /* channel filter is active */
    }
       
    pFTM->SC    = 0x0;                  /* diable counter */ 
    pFTM->MOD   = 0xFFFF;
    pFTM->MODE |= FTM_MODE_FTMEN_MASK;  /* FTMEN = 1 */  
    /* DECAPEN = 1,  ChannelPair/2 * 8 */
    pFTM->COMBINE |=  ((FTM_COMBINE_DECAPEN0_MASK) << (u8ChannelPair * 4)); 
    
    pFTM->CONTROLS[u8ChannelPair].CnSC &= ~FTM_CnSC_CHF_MASK;       /* CH(n)F and CH(n+1)F bits must be cleared first */
    pFTM->CONTROLS[u8ChannelPair + 1].CnSC &= ~FTM_CnSC_CHF_MASK;
    
    if(FTM_INPUTCAPTURE_DUALEDGE_ONESHOT == u8CaptureMode)          /* oneshot mode */
    {
        pFTM->CONTROLS[u8ChannelPair].CnSC &= ~FTM_CnSC_MSA_MASK;
        pFTM->CONTROLS[u8ChannelPair+1].CnSC &= ~FTM_CnSC_MSA_MASK;
    }
    else if(FTM_INPUTCAPTURE_DUALEDGE_CONTINUOUS == u8CaptureMode)  /* continuouse mode */
    {
        pFTM->CONTROLS[u8ChannelPair].CnSC |= FTM_CnSC_MSA_MASK;
        pFTM->CONTROLS[u8ChannelPair+1].CnSC |= FTM_CnSC_MSA_MASK;
    }
    
    pFTM->CONTROLS[u8ChannelPair].CnSC |= (u8Channel_N_Edge << 2);   /* select detec edge */
    pFTM->CONTROLS[u8ChannelPair + 1].CnSC |= (u8Channel_Np1_Edge << 2);   
    
    pFTM->COMBINE |=  (FTM_COMBINE_DECAP0_MASK << (u8ChannelPair * 4)); 
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM_No to ouput compare mode.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    Channel               channel number to be configured.
* @param[in]    CompareMode           select compare edge: toggle, set and clear.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_OutputCompareInit(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8CompareMode)
{
   
    /* open the clock gate */
	if(pFTM0 == pFTM)
    {
        SIM->SCGC |= SIM_SCGC_FTM0_MASK;
    }
    else if(pFTM1 == pFTM)
    {
        SIM->SCGC |= SIM_SCGC_FTM1_MASK;
    }        
    else
    {
        SIM->SCGC |= SIM_SCGC_FTM2_MASK;
    }
    
    pFTM->SC  = 0x0;                                                                    /* diable counter */
    pFTM->MOD = FTM_MOD_INIT; 
    pFTM->CONTROLS[u8FTM_Channel].CnSC = (FTM_CnSC_MSA_MASK | (u8CompareMode << 2));    /* select detec edge */
    pFTM->CONTROLS[u8FTM_Channel].CnV  = FTM_C0V_INIT;
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM2 to start software synchronization.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_SoftwareSync(FTM_Type* pFTM)
{
  

    pFTM->SYNCONF   |= FTM_SYNCONF_SYNCMODE_MASK;   /* recommend enhanced sync mode */
    pFTM->SYNC      |= FTM_SYNC_SWSYNC_MASK;
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM to enable hardware synchronization.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    u8TriggerN            select the hardware trigger source.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_HardwareSync(FTM_Type* pFTM, uint8_t u8TriggerN)
{
    
    pFTM->SYNCONF   |= FTM_SYNCONF_SYNCMODE_MASK;   /* recommend enhanced sync mode */
    
    switch(u8TriggerN)
    {
        case FTM_SYNC_TRIGGER_TRIGGER2: 
                pFTM->SYNC |= FTM_SYNC_TRIG2_MASK;
                break;  
        case FTM_SYNC_TRIGGER_TRIGGER1: 
                pFTM->SYNC |= FTM_SYNC_TRIG1_MASK;
                break;     /* need configure FTM0CH0 first */
        case FTM_SYNC_TRIGGER_TRIGGER0:
                pFTM->SYNC |= FTM_SYNC_TRIG0_MASK; 
                break;     /* need configure CMP0 first */
        default: 
                break;
    }
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM to enable hardware synchronization, more then 1 trigger.
*        
* @param[in]    pFTM               pointer to one of three FTM base register address.
* @param[in]    u8TriggerMask      select the hardware trigger source. combine TRIG0~TREG2.(x000xxxx~x111xxxx)
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_HardwareSyncCombine(FTM_Type* pFTM, uint8_t u8TriggerMask)
{
    pFTM->SYNCONF   |= FTM_SYNCONF_SYNCMODE_MASK;   /* recommend enhanced sync mode */
    pFTM->SYNC      &= 0x8F;
    pFTM->SYNC      |= (u8TriggerMask & 0x70);
}

/*********************************************************************************//*!
*
* @brief generate FTM2 hardware trigger 2,Note: please call FTM_HardwareSyncCombine first.
*        
* @param[in]    pFTM               pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_GenerateTrig2(FTM_Type* pFTM)
{
   
    if(pFTM->SYNC & FTM_SYNC_TRIG2_MASK)
    {
#if defined(MCU_SKEAZ1284)
        SIM->SOPT0  |= SIM_SOPT0_FTMSYNC_MASK;
#elif defined(MCU_SKEAZN642)
		SIM->SOPT  |= SIM_SOPT_FTMSYNC_MASK;
#else
/* Write the implementation for your device. */
#endif

    }
}


/*********************************************************************************//*!
*
* @brief general configuration to FTM_No to start software synchronization.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    PrescalerValue        system clock divide value, 0 to 3.
* @param[in]    DeadtimeValue         n count clock is inserted, 0 to 63.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_PWMDeadtimeSet(FTM_Type* pFTM, uint8_t u8PrescalerValue, uint8_t u8DeadtimeValue)
{
    
    pFTM->COMBINE |= 0x101010;              /* enable dead time insertion */

    if(!(pFTM->MODE & FTM_MODE_WPDIS_MASK)) /* if write protection is enabled */
    {
        pFTM->MODE |= FTM_MODE_WPDIS_MASK;  /* disable the write protection */
        pFTM->DEADTIME = (FTM_DEADTIME_DTVAL(u8DeadtimeValue & 0x3F) | FTM_DEADTIME_DTPS(u8PrescalerValue & 0x3));
        pFTM->MODE &= ~FTM_MODE_WPDIS_MASK; /* enable the write protection */       
    }
    else 
    {
        /* if no protection */
        pFTM->DEADTIME = (FTM_DEADTIME_DTVAL(u8DeadtimeValue & 0x3F) | FTM_DEADTIME_DTPS(u8PrescalerValue & 0x3));
    }
    pFTM->SYNC |= FTM_SYNC_SWSYNC_MASK;     /* software sync */
}    

/*********************************************************************************//*!
*
* @brief set output mask.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    Channel               pwm channel needed to be masked.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_OutputMaskSet(FTM_Type* pFTM, uint8_t u8FTM_Channel)
{
   

    pFTM->OUTMASK |= (1 << u8FTM_Channel);
    
    if(pFTM->SYNC & FTM_SYNC_SYNCHOM_MASK)              /* if PWM sync is needed */
    {
        pFTM->SYNCONF |= FTM_SYNCONF_SYNCMODE_MASK;     /* recommend enhanced sync mode */
        if(pFTM->SYNCONF & FTM_SYNCONF_SWOM_MASK)       /* if software sync is needed*/
        {
            pFTM->SYNC |= FTM_SYNC_SWSYNC_MASK;
        }
        else if(pFTM->SYNCONF & FTM_SYNCONF_HWOM_MASK)  /* if hardware sync is needed*/
        {
            pFTM->SYNC |= FTM_SYNC_TRIG2_MASK;
#if defined(MCU_SKEAZ1284)
            SIM->SOPT0  |= SIM_SOPT0_FTMSYNC_MASK;         /* hardware sync */ 
#elif defined(MCU_SKEAZN642)
			SIM->SOPT   |= SIM_SOPT_FTMSYNC_MASK;		   /* hardware sync */
#else
/* Write the implementation for your device. */
#endif

        }
        else
        {
        }
    }
    else  /* no need to sync, update on the next rising edge of system clock  */
    {
    }
}

/*********************************************************************************//*!
*
* @brief general configuration to FTM_No to start software synchronization.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    Channel               pwm channel needed to be controlled by software.
* @param[in]    ChannelValue          the value to be set,  0 or 1.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_SWOutputControlSet(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8ChannelValue)
{
    
    if(FTM_SWOCTRL_HIGH == u8ChannelValue)
    {
        pFTM->SWOCTRL |= (0x0101 << u8FTM_Channel);
    }
    else if(FTM_SWOCTRL_LOW == u8ChannelValue)
    {
        pFTM->SWOCTRL |= (1 << u8FTM_Channel);
        pFTM->SWOCTRL &= ~(0x100 << u8FTM_Channel);
    }
    if(pFTM->SYNCONF & FTM_SYNCONF_SWOC_MASK)               /* if PWM sync is needed */
    {
        pFTM->SYNCONF |= FTM_SYNCONF_SYNCMODE_MASK;         /* recommend enhanced sync mode */
        if(pFTM->SYNCONF & FTM_SYNCONF_SWSOC_MASK)          /* if software sync is needed*/
        {
            pFTM->SYNC |= FTM_SYNC_SWSYNC_MASK;             /* software sync */ 
        }
        else if(pFTM->SYNCONF & FTM_SYNCONF_HWSOC_MASK)     /* if hardware sync is needed*/
        {
            pFTM->SYNC |= FTM_SYNC_TRIG2_MASK;
#if defined(MCU_SKEAZ1284)
            SIM->SOPT0  |= SIM_SOPT0_FTMSYNC_MASK;             /* hardware sync */ 
#elif defined(MCU_SKEAZN642)
			SIM->SOPT |= SIM_SOPT_FTMSYNC_MASK;					/* hardware sync */
#else
/* Write the implementation for your device. */
#endif

        }
    }
    else  /* no need to sync, update on the next rising edge of system clock  */
    {
    }
}

/*********************************************************************************//*!
*
* @brief set PWM polarity.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    Channel               pwm channel.
* @param[in]    ActiveValue           the value to be set,  0 or 1.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_PolaritySet(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8ActiveValue)
{
    
    if(FTM_POLARITY_HIGHACTIVE == u8ActiveValue)
    {
        pFTM->POL &=  ~(1 << u8FTM_Channel);
    }
    else if(FTM_POLARITY_LOWACTIVE == u8ActiveValue)
    {
        pFTM->POL |=  (1 << u8FTM_Channel);
    }
}

/*********************************************************************************//*!
*
* @brief set FTM behavior in debug mode.
*        
* @param[in]    pFTM             pointer to one of three FTM base register address.
* @param[in]    u8DebugMode      debug mode select from 00 to 11.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_SetDebugModeBehavior(FTM_Type* pFTM, uint8_t u8DebugMode)
{
    pFTM->CONF &= ~FTM_CONF_BDMMODE_MASK;
    pFTM->CONF |= FTM_CONF_BDMMODE(u8DebugMode);
}

/*********************************************************************************//*!
*
* @brief Selects the ratio between the number of counter overflows to the number of times the TOF bit is set.
*        
* @param[in]    pFTM             pointer to one of three FTM base register address.
* @param[in]    u8TOFNUM         TOF numbers before setting TOF bit, 0~31.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_SetTOFFrequency(FTM_Type* pFTM, uint8_t u8TOFNUM)
{
    pFTM->CONF &= ~FTM_CONF_NUMTOF_MASK;
    pFTM->CONF |= FTM_CONF_NUMTOF(u8TOFNUM);
}

/*********************************************************************************//*!
*
* @brief swap the output of CH(n) and CH(n+1).
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    ChannelPair           the pair to be swapped, 0,1,2.
*    
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
void FTM_InvertChannel(FTM_Type* pFTM, uint8_t u8ChannelPair)
{

    pFTM->INVCTRL |= 1<<u8ChannelPair;
    if(pFTM->SYNCONF & FTM_SYNCONF_INVC_MASK)       /* if PWM sync is needed */
    {
        pFTM->SYNCONF |= FTM_SYNCONF_SYNCMODE_MASK; /* recommend enhanced sync mode */
        if(pFTM->SYNCONF & FTM_SYNCONF_SWINVC_MASK) /* if software sync is needed*/
        {
            pFTM->SYNC |= FTM_SYNC_SWSYNC_MASK;     /* software sync */ 
        }
        else if(pFTM->SYNCONF & FTM_SYNCONF_HWINVC_MASK)    /* if hardware sync is needed*/
        {
            pFTM->SYNC |= FTM_SYNC_TRIG2_MASK;

#if defined(MCU_SKEAZ1284)
            SIM->SOPT0  |= SIM_SOPT0_FTMSYNC_MASK;             /* hardware sync */ 
#elif defined(MCU_SKEAZN642)
			SIM->SOPT 	|= SIM_SOPT_FTMSYNC_MASK;				/* hardware sync */
#else
/* Write the implementation for your device. */
#endif

        }
    }
    else  /* no need to sync, update on the next rising edge of system clock  */
    {
    }
}  

/*****************************************************************************//*!
*
* @brief configure the FTM as specified control parameters, CnSC and CnV not 
*        included.
*        
* @param[in]    pFTM          pointer to one of three FTM base register address.
* @param[in]    pConfig     pointer to FTM general parameters.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM_Init(FTM_Type* pFTM, FTM_ConfigType *pConfig)
{
    if(pFTM0 == pFTM)
    {
        SIM->SCGC |= SIM_SCGC_FTM0_MASK;
    }

    else if(pFTM1 == pFTM)
    {
        SIM->SCGC |= SIM_SCGC_FTM1_MASK;
    }
    else
    {
        SIM->SCGC |= SIM_SCGC_FTM2_MASK;
    }
        
    /* disable counter */
    pFTM->SC = 0; 
    
    if(pFTM2 == pFTM)
    {
     		pFTM->MODE |= pConfig->mode;
			pFTM->COMBINE   = pConfig->combine;      
			pFTM->CNTIN     = pConfig->cntin;      
			pFTM->SYNC      = pConfig->sync;      
			pFTM->OUTINIT   = pConfig->outinit;      
			pFTM->OUTMASK   = pConfig->outmask;      
			pFTM->DEADTIME  = pConfig->deadtime;      
			pFTM->EXTTRIG   = pConfig->exttrig;      
			pFTM->POL       = pConfig->pol;      
			pFTM->FMS       = pConfig->fms;      
			pFTM->FILTER    = pConfig->filter;      
			pFTM->FLTCTRL   = pConfig->fltctrl;    /* fault control */ 
			pFTM->FLTPOL    = pConfig->fltpol;      
			pFTM->CONF      = pConfig->conf;      
			pFTM->SYNCONF   = pConfig->synconf;      
			pFTM->SWOCTRL   = pConfig->swoctrl;      
			pFTM->PWMLOAD   = pConfig->pwmload;      

    }
	pFTM->MOD = pConfig->modulo;
	pFTM->CNT = pConfig->cnt;
    
    /* write SC to enable clock */
    pFTM->SC |= pConfig->clk_source<<3|pConfig->prescaler<<0|pConfig->cpwms<<5|pConfig->toie<<6;
    if( pConfig->toie)
    {
    	if(pFTM0 == pFTM)
		{
			Enable_Interrupt(FTM0_IRQn);

		}
		else if(pFTM1 == pFTM)
		{
			Enable_Interrupt(FTM1_IRQn);

		}
		else
		{
			Enable_Interrupt(FTM2_IRQn);

		}	
    }
}


/*****************************************************************************//*!
*
* @brief  close the FTM module.
*        
* @param[in]    pFTM          pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM_DeInit(FTM_Type* pFTM)
{
    pFTM->SC = 0;       
	pFTM->MOD = 0;
	pFTM->CNT = 0;
    if(pFTM2 == pFTM) 
    {
          pFTM->MODE = 0x4; 
          pFTM->COMBINE = 0;      
          pFTM->CNTIN = 0;      
          pFTM->SYNC = 0;      
          pFTM->OUTINIT = 0;      
          pFTM->OUTMASK = 0;      
          pFTM->DEADTIME = 0;      
          pFTM->EXTTRIG = 0;      
          pFTM->POL = 0;      
          pFTM->FMS = 0;      
          pFTM->FILTER = 0;      
          pFTM->FLTCTRL = 0;  
          pFTM->FLTPOL = 0;      
          pFTM->CONF = 0;      
          pFTM->SYNCONF = 0;      
          pFTM->SWOCTRL = 0;      
          pFTM->PWMLOAD = 0;      
    }
    /* close the clock gate */
	if (pFTM0 == pFTM)
    {
        SIM->SCGC &= ~SIM_SCGC_FTM0_MASK;
        Disable_Interrupt(FTM0_IRQn);
    }
#if !defined(CPU_KE04)
    else if(pFTM1 == pFTM)
    {
        SIM->SCGC &= ~SIM_SCGC_FTM1_MASK;
        Disable_Interrupt(FTM1_IRQn);
    } 
#endif
    else if (pFTM2 == pFTM)
    {
        SIM->SCGC &= ~SIM_SCGC_FTM2_MASK;
        Disable_Interrupt(FTM2_IRQn);
    }
}
    
/*****************************************************************************//*!
*
* @brief configure the FTM  channels, CnSC and CnV are included.
*        
* @param[in]    pFTM               pointer to one of three FTM base register address.
* @param[in]    FTM_Channel        FTM channel number.
* @param[in]    pTFTMCH_Params     pointer to FTM channel general parameters.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM_ChannelInit(FTM_Type* pFTM, uint8_t u8FTM_Channel, FTM_ChParamsType pTFTMCH_Params)
{ 
    
	if (pFTM0 == pFTM)
    {
        SIM->SCGC |= SIM_SCGC_FTM0_MASK;
    }

    else if(pFTM1 == pFTM)
    {
        SIM->SCGC |= SIM_SCGC_FTM1_MASK;
    }        

    else
    {
     
        SIM->SCGC |= SIM_SCGC_FTM2_MASK;
    }
    
    
    if( pTFTMCH_Params.ctrl.bits.bMode==FTM_INPUT_CAPTURE)
    {
        pFTM->CONTROLS[u8FTM_Channel].CnSC |= 0b00<<4;
    	 pFTM->CONTROLS[u8FTM_Channel].CnSC |= pTFTMCH_Params.ctrl.bits.bEdge<<2;
    }
    else if( pTFTMCH_Params.ctrl.bits.bMode==FTM_OUTPUT_COMPARE)
    {
        pFTM->CONTROLS[u8FTM_Channel].CnSC |= FTM_CnSC_MSA_MASK;
     	 pFTM->CONTROLS[u8FTM_Channel].CnSC |= pTFTMCH_Params.ctrl.bits.bOutCmp<<2;
    }
    else if( pTFTMCH_Params.ctrl.bits.bMode==FTM_PWMMODE_EDGEALLIGNED)
    {
    	 pFTM->CONTROLS[u8FTM_Channel].CnSC |= FTM_CnSC_MSB_MASK;
    	pFTM->CONTROLS[u8FTM_Channel].CnSC |= pTFTMCH_Params.ctrl.bits.bPWMPol<<2;
    }
    
    else if (pTFTMCH_Params.ctrl.bits.bMode==FTM_PWMMODE_CENTERALLIGNED)
    {
    	pFTM->SC|=FTM_SC_CPWMS_MASK;
    	pFTM->CONTROLS[u8FTM_Channel].CnSC |= pTFTMCH_Params.ctrl.bits.bPWMPol<<2;

    }
    else if (pTFTMCH_Params.ctrl.bits.bMode==FTM_PWMMODE_COMBINE)
    {
    	pFTM->CONTROLS[u8FTM_Channel].CnSC |= FTM_CnSC_MSA_MASK|FTM_CnSC_MSB_MASK|pTFTMCH_Params.ctrl.bits.bPWMPol<<2;

    }
    else if (pTFTMCH_Params.ctrl.bits.bMode==FTM_INPUTCAPTURE_DUALEDGE)
    {
    	pFTM->CONTROLS[u8FTM_Channel].CnSC |= pTFTMCH_Params.ctrl.bits.bEdge<<2;
    	if(pTFTMCH_Params.ctrl.bits.bDualCapMode==FTM_INPUTCAPTURE_DUALEDGE_ONESHOT)
    	{
    		pFTM->CONTROLS[u8FTM_Channel].CnSC &= ~FTM_CnSC_MSA_MASK;

    	}
    	else if(pTFTMCH_Params.ctrl.bits.bDualCapMode==FTM_INPUTCAPTURE_DUALEDGE_CONTINUOUS)
    	{
    		pFTM->CONTROLS[u8FTM_Channel].CnSC |= FTM_CnSC_MSA_MASK;

    	}
    
    }


    	
    if(1==pTFTMCH_Params.ctrl.bits.bCHIE)
    {
    	pFTM->CONTROLS[u8FTM_Channel].CnSC |=FTM_CnSC_CHIE_MASK;
    	if(pFTM0 == pFTM)
    	{
        	Enable_Interrupt(FTM0_IRQn);

    	}
    	else if(pFTM1 == pFTM)
    	{
        	Enable_Interrupt(FTM1_IRQn);

    	}
    	else
    	{
        	Enable_Interrupt(FTM2_IRQn);

    	}

    }
    
    pFTM->CONTROLS[u8FTM_Channel].CnV = pTFTMCH_Params.u16CnV; 
    
    return;
}


/*****************************************************************************//*!
*
* @brief configure the FTMx_SYNCONF register including SW and HW Sync selection.
*
* @param[in]    pFTM             pointer to one of three FTM base register address.
* @param[in]    u32ConfigValue   FTMx_SYNCONF register config value.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void  FTM_SyncConfigActivate(FTM_Type* pFTM, uint32_t u32ConfigValue)
{
    pFTM->SYNCONF |= u32ConfigValue;   
}

/*****************************************************************************//*!
*
* @brief configure the FTMx_SYNCONF register including SW and HW Sync selection.
*
* @param[in]    pFTM             pointer to one of three FTM base register address.
* @param[in]    u32ConfigValue   FTMx_SYNCONF register config value.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void  FTM_SyncConfigDeactivate(FTM_Type* pFTM, uint32_t u32ConfigValue)
{
    pFTM->SYNCONF &= ~u32ConfigValue;   
}

/*****************************************************************************//*!
*
* @brief This function sets the callback function.
*
* @param[in]    pFTM          pointer to one of three FTM base register address.
* @param[in]    pfnCallback     functon address.
*
* @return none.
*
* @ Pass/ Fail criteria: none
*
*****************************************************************************/
void  FTM_SetCallback(FTM_Type* pFTM, FTM_CallbackPtr pfnCallback)
{
   FTM_Callback[((uint32_t)pFTM - (uint32_t)FTM0_BASE)>>12] = pfnCallback;
}

/*! @} End of ftm_api_list                                                    */


/*****************************************************************************//*!
*
* @brief  FTM0_Isr interrupt service routine.
*        
* @param  none.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM0_IRQHandler(void)
{
    if(FTM_Callback[0])
    {
        FTM_Callback[0]();
    }
}

/*****************************************************************************//*!
*
* @brief  FTM1_Isr interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM1_IRQHandler(void)
{
    if(FTM_Callback[1])
    {
        FTM_Callback[1]();
    }
}

/*****************************************************************************//*!
*
* @brief  FTM2_Isr interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/

void FTM2_IRQHandler(void)
{
	if(FTM_Callback[2])
    {
        FTM_Callback[2]();
    }
}

