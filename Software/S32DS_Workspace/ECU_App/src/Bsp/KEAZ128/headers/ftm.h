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
* @file ftm.h
*
* @author Freescale
*
*
* @brief header file for FTM. 
*
*******************************************************************************
*
* provide APIs for accessing FTM
******************************************************************************/
#ifndef FTM_H_
#define FTM_H_
#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
* Includes
******************************************************************************/
#include "derivative.h"
#include "nvic.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros
******************************************************************************/
//#define FTM_Type FTM_MemMapPtr

/******************************************************************************
* FTM return status definition
*
*//*! @addtogroup ftm_returnstatus
* @{
*******************************************************************************/
#define FTM_ERR_SUCCESS         0               /*!< return ok */
#define FTM_ERR_INVALID_PARAM   1               /*!< return invalid parameter */
/*! @} End of ftm_returnstatus                                                */


/******************************************************************************
* FTM channel number definition
*
*//*! @addtogroup ftm_channelnumber
* @{
*******************************************************************************/
#define FTM_CHANNEL_CHANNEL0    0               /*!< channel 0 */
#define FTM_CHANNEL_CHANNEL1    1               /*!< channel 1 */
#define FTM_CHANNEL_CHANNEL2    2               /*!< channel 2 */
#define FTM_CHANNEL_CHANNEL3    3               /*!< channel 3 */
#define FTM_CHANNEL_CHANNEL4    4               /*!< channel 4 */
#define FTM_CHANNEL_CHANNEL5    5               /*!< channel 5 */

#define FTM_CHANNELPAIR0        0               /*!< channel pair 0:ch0 & ch1 */
#define FTM_CHANNELPAIR1        2               /*!< channel pair 1:ch2 & ch3 */
#define FTM_CHANNELPAIR2        4               /*!< channel pair 2:ch4 & ch5 */

/*! @} End of ftm_channelnumber                                               */



/******************************************************************************
* FTM channels modS definition
*
*//*!
* @{
*******************************************************************************/
#define FTM_PWMMODE_EDGEALLIGNED      		1          /*!< EPWM */
#define FTM_PWMMODE_CENTERALLIGNED    		2          /*!< CPWM */
#define FTM_PWMMODE_COMBINE					3		   /*!< Combine PWM */
#define FTM_INPUT_CAPTURE			  		4   	  /*!< Input capture */
#define FTM_OUTPUT_COMPARE  		  		5  	  	  /*!< Output compare */
#define FTM_INPUTCAPTURE_DUALEDGE		  	6		  /*!< Dual Edge Capture*/

/*! @} End of ftm_channels modes */


/*******************************************************************************
* FTM init value definition
*
*//*! @addtogroup ftm_initvalue
* @{
*******************************************************************************/
#define FTM_MOD_INIT	(20000-1)                /*!< MOD inite value */
#define FTM_C0V_INIT	1000                     /*!< C0V inite value */
#define FTM_C1V_INIT	1000                     /*!< C1V inite value */
#define FTM_C2V_INIT	1000                     /*!< C2V inite value */    
#define FTM_C3V_INIT	1000                     /*!< C3V inite value */
#define FTM_C4V_INIT	1000                     /*!< C4V inite value */
#define FTM_C5V_INIT	1000                     /*!< C5V inite value */
/*! @} End of ftm_initvalue                                                   */

/******************************************************************************
* FTM combine feature definition
*
*//*! @addtogroup ftm_combinefeature
* @{
*******************************************************************************/
#define FTM_COMBINE_FAULTEN_MASK  0x40       /*!< fault enable */
#define FTM_COMBINE_SYNCEN_MASK   0x20       /*!< sync enable */
#define FTM_COMBINE_DTEN_MASK     0x10       /*!< dead time insertion enable */
#define FTM_COMBINE_DECAP_MASK    0x08       /*!< dual capture status */
#define FTM_COMBINE_DECAPEN_MASK  0x04       /*!< dual capture enable */
#define FTM_COMBINE_COMP_MASK     0x02       /*!< complementary enable */
#define FTM_COMBINE_COMBINE_MASK  0x01       /*!< combine enable */
/*! @} End of ftm_combinefeature                                              */

/******************************************************************************
* FTM clock sources definition
*
*//*! @addtogroup ftm_clocksource
* @{
*******************************************************************************/
#define FTM_CLOCK_NOCLOCK             0                 /*!< No Clock */
#define FTM_CLOCK_SYSTEMCLOCK         1                 /*!< System clock/2 */
#define FTM_CLOCK_FIXEDFREQCLOCK      2                 /*!< Fixed Freq Clock */
#define FTM_CLOCK_EXTERNALCLOCK       3                 /*!< External Clock */

/* clock prescale */
#define FTM_CLOCK_PS_DIV1             0                 /*!< DIV 1 */
#define FTM_CLOCK_PS_DIV2             1                 /*!< DIV 2 */
#define FTM_CLOCK_PS_DIV4             2                 /*!< DIV 4 */
#define FTM_CLOCK_PS_DIV8             3                 /*!< DIV 8 */
#define FTM_CLOCK_PS_DIV16            4                 /*!< DIV 16 */
#define FTM_CLOCK_PS_DIV32            5                 /*!< DIV 32 */
#define FTM_CLOCK_PS_DIV64            6                 /*!< DIV 64 */
#define FTM_CLOCK_PS_DIV128           7                 /*!< DIV 128 */
/*! @} End of ftm_clocksource                                                 */

/******************************************************************************
* FTM dead time clock prescale definition
*
*//*! @addtogroup ftm_deadtimeprescale
* @{
*******************************************************************************/
/*  */
#define FTM_DEADTIME_DTPS_DIV1        0                 /*!< DIV 1 */
#define FTM_DEADTIME_DTPS_DIV4        2                 /*!< DIV 4 */
#define FTM_DEADTIME_DTPS_DIV16       3                 /*!< DIV 16 */
/*! @} End of ftm_deadtimeprescale                                            */




/******************************************************************************
* FTM output mode definition
*
*//*! @addtogroup ftm_outputmode
* @{
*******************************************************************************/
/* output mode */
#define FTM_OUTPUT_TOGGLE   1                    /*!< toggle output on match */
#define FTM_OUTPUT_CLEAR    2                    /*!< clear output on match */
#define FTM_OUTPUT_SET      3                    /*!< set output on match */
/*! @} End of ftm_outputmode                                                  */

/******************************************************************************
* FTM input capture edge definition
*
*//*! @addtogroup ftm_inputcaptureedge
* @{
*******************************************************************************/
/* mode edge select*/
#define FTM_INPUTCAPTURE_RISINGEDGE           1     /*!< rising edge */
#define FTM_INPUTCAPTURE_FALLINGEDGE          2     /*!< falling edge */
#define FTM_INPUTCAPTURE_BOTHEDGE             3     /*!< both edge */

#define FTM_INPUTCAPTURE_DUALEDGE_NOEDGE      0     /*!< none */
#define FTM_INPUTCAPTURE_DUALEDGE_RISINGEDGE  1     /*!< rising edge*/
#define FTM_INPUTCAPTURE_DUALEDGE_FALLINGEDGE 2     /*!< falling edge*/
#define FTM_INPUTCAPTURE_DUALEDGE_BOTHEDGE    3     /*!< both edge */
/*! @} End of ftm_inputcaptureedge                                            */

/******************************************************************************
* FTM dual edge capture mode definition
*
*//*! @addtogroup ftm_dualcapturemode
* @{
*******************************************************************************/
#define FTM_INPUTCAPTURE_DUALEDGE_ONESHOT     2  /*!< dual edge one shot mode*/ 
#define FTM_INPUTCAPTURE_DUALEDGE_CONTINUOUS  1  /*!< dual edge continuouse mode*/
/*! @} End of ftm_dualcapturemode                                            */

/******************************************************************************
* FTM PWM edge definition
*
*//*! @addtogroup ftm_pwmedge
* @{
*******************************************************************************/
#define FTM_PWM_HIGHTRUEPULSE        2            /*!< high true pulses */
#define FTM_PWM_LOWTRUEPULSE         1            /*!< low true pulses */
/*! @} End of ftm_pwmedge                                                     */

/******************************************************************************
* FTM sync trigger source definition
*
*//*! @addtogroup ftm_syncsource
* @{
*******************************************************************************/
#define FTM_SYNC_TRIGGER_SOFTWARE    1        /*!< Software synchronization */
#define FTM_SYNC_TRIGGER_TRIGGER2    2        /*!< Tigger2 synchronization, SIM_SOPT[FTMSYNC] */
#define FTM_SYNC_TRIGGER_TRIGGER1    3        /*!< Tigger1 synchronization, FTM0CH0 */
#define FTM_SYNC_TRIGGER_TRIGGER0    4        /*!< Tigger0 synchronization, ACMP0 */
/*! @} End of ftm_syncsource                                                  */

/******************************************************************************
* FTM SW output control definition
*
*//*! @addtogroup ftm_swoutputcontrol
* @{
*******************************************************************************/ 
#define FTM_SWOCTRL_HIGH    1                /*!< software output high */
#define FTM_SWOCTRL_LOW     0                /*!< software output low */
/*! @} End of ftm_swoutputcontrol                                             */

/******************************************************************************
* FTM  polarity definition
*
*//*! @addtogroup ftm_polarity
* @{
*******************************************************************************/
#define FTM_POLARITY_HIGHACTIVE     0        /*!< active high */
#define FTM_POLARITY_LOWACTIVE      1        /*!< active low */
/*! @} End of ftm_polarity                                                    */


/******************************************************************************
* Types
******************************************************************************/
/*! @brief FTM_CALLBACK function declaration                                  */
typedef void (*FTM_CallbackPtr)(void);
/*! @} End of ftm_callback                                                    */

/******************************************************************************
* FTM configure struct.
*
*//*! @addtogroup ftm_configsturct
* @{
*******************************************************************************/
/*!
* @brief FTM configure struct.
*
*/

typedef struct
{
  uint8_t   clk_source;       /*!< clock source */
  uint8_t   prescaler;        /*!< clock prescaler */
  uint8_t	cpwms;			  /*!< up-down counting mode */
  uint8_t	toie;		      /*!< enable overflow interrupt */
  uint16_t  modulo;           /*!< counter mod */
  uint16_t  cnt;              /*!< counter value */
  uint16_t  cntin;            /*!< counter inite */
  uint8_t   mode;             /*!< features mode selction */
  uint8_t   sync;             /*!< synchronization */
  uint8_t   outinit;          /*!< initial state for channels output  */
  uint8_t   outmask;          /*!< output mask */
  uint32_t  combine;          /*!< function for linked channels */
  uint16_t  deadtime;         /*!< dead time insertion control */
  uint8_t   exttrig;          /*!< external trigger */
  uint8_t   pol;              /*!< channels polarity */
  uint16_t  filter;           /*!< input filter control */
  uint8_t   fms;              /*!< fault mode status */
  uint16_t  fltctrl;          /*!< fault control */
  uint8_t   fltpol;           /*!< fault input polarity */
  uint16_t  conf;             /*!< FTM configuration */
  uint32_t  synconf;          /*!< synchronization configuration*/
  uint8_t   invctrl;          /*!< inverting control */
  uint16_t  swoctrl;          /*!< software output control */
  uint16_t  pwmload;          /*!< pwm load control */
} FTM_ConfigType, *FTM_ConfigPtr;
/*! @} End of ftm_configsturct                                                */

/******************************************************************************
* FTM channel configure struct.
*
*//*! @addtogroup ftm_chconfigsturct
* @{
*******************************************************************************/
/*!
* @brief FTM channel configure struct.
*
*/
typedef struct
{
  uint16_t        u16CnV;                  /*!< FTM channel value control */
  union
  {
    struct 
    {
      uint32_t    bMode         : 6;        /*!< flextimer mode:INPUT_CAPTURE, OUTPUT_COMPARE, EDGE_ALIGNED_PWM, CENTER_ALIGNED_PWM,
                                             * COMBINE_PWM, DUAL_EDGE_CAPTURE 
                                             */
      uint32_t    bEdge         : 2;        /*!< edge select */
      uint32_t    bOutCmp       : 2;        /*!< toggle, clear, set */
      uint32_t    bPWMPol       : 2;        /*!< high-true pulse, low-true pulses */
      uint32_t    bDualCapMode  : 2;        /*!< dual edge capture mode: one-shot, continuous mode */
      uint32_t    bCHIE         : 1;        /*!< enable channel interrupt */
    }bits;
  }ctrl;                                    /*!< FTM channel feature control */
} FTM_ChParamsType;

/*! @} End of ftm_chconfigsturct                                              */

/******************************************************************************
* Global variables
******************************************************************************/

/*!
 * inline functions
 */
/******************************************************************************
* FTM inline functions
*
*//*! @addtogroup ftm_api_list
* @{
*******************************************************************************/
/*****************************************************************************//*!
*
* @brief  enable the over flow interrupt.
*        
* @param[in]    pFTM          pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_DisableOverflowInt.
*
*****************************************************************************/
static inline void FTM_EnableOverflowInt(FTM_Type* pFTM)
{
    if(pFTM->SC & FTM_SC_TOF_MASK)
    {
        pFTM->SC &= ~FTM_SC_TOF_MASK;
    }
    pFTM->SC |= FTM_SC_TOIE_MASK;
}

/*****************************************************************************//*!
*
* @brief  disable the over flow interrupt.
*        
* @param[in]    pFTM          pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none
*
* @see FTM_EnableOverflowInt.
*
*****************************************************************************/
static inline void FTM_DisableOverflowInt(FTM_Type* pFTM)
{
    pFTM->SC &= ~FTM_SC_TOIE_MASK;
}

/*****************************************************************************//*!
*
* @brief  enable the channel interrupt.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
* @param[in]    u8FTM_Channel  channel number.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_DisableChannelInt.
*
*****************************************************************************/
static inline void FTM_EnableChannelInt(FTM_Type* pFTM, uint8_t u8FTM_Channel)
{
    pFTM->CONTROLS[u8FTM_Channel].CnSC |= FTM_CnSC_CHIE_MASK;
}

/*****************************************************************************//*!
*
* @brief  disable the channel interrupt.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
* @param[in]    u8FTM_Channel  channel number.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_EnableChannelInt.
*
*****************************************************************************/
static inline void FTM_DisableChannelInt(FTM_Type* pFTM, uint8_t u8FTM_Channel)
{
    pFTM->CONTROLS[u8FTM_Channel].CnSC &= ~FTM_CnSC_CHIE_MASK;
}

/*****************************************************************************//*!
*
* @brief  get the over flow flag.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
*    
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_ClrOverFlowFlag.
*
*****************************************************************************/
static inline uint8_t FTM_GetOverFlowFlag(FTM_Type* pFTM)
{
    return (pFTM->SC & FTM_SC_TOF_MASK);
}

/*****************************************************************************//*!
*
* @brief  clear the over flow flag.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
*    
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_GetOverFlowFlag.
*
*****************************************************************************/
static inline void FTM_ClrOverFlowFlag(FTM_Type* pFTM)
{
    if(pFTM->SC & FTM_SC_TOF_MASK)
    {
         pFTM->SC &= ~FTM_SC_TOF_MASK;
    }
}

/*****************************************************************************//*!
*
* @brief  get the channel flag.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
* @param[in]    u8FTM_Channel  channel number.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_ClrChannelFlag.
*
*****************************************************************************/
static inline uint8_t FTM_GetChannelFlag(FTM_Type* pFTM, uint8_t u8FTM_Channel)
{
    return (pFTM->CONTROLS[u8FTM_Channel].CnSC & FTM_CnSC_CHF_MASK);
}

/*****************************************************************************//*!
*
* @brief  clear the channel flag.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
*    
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_GetChannelFlag.
*
*****************************************************************************/
static inline void FTM_ClrChannelFlag(FTM_Type* pFTM, uint8_t u8FTM_Channel)
{
    (void) pFTM->CONTROLS[u8FTM_Channel].CnSC; //Dummy read while CHF is set
	pFTM->CONTROLS[u8FTM_Channel].CnSC &= ~FTM_CnSC_CHF_MASK;
}

/*********************************************************************************//*!
*
* @brief enable the write protection function.Write protected bits cannot be written. 
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_WriteProtectionDisable.
*
*********************************************************************************/
static inline void FTM_WriteProtectionEnable(FTM_Type* pFTM)
{
    pFTM->FMS |= FTM_FMS_WPEN_MASK;
}

/*********************************************************************************//*!
*
* @brief disable the write protection function.Write protected bits can be written. 
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_WriteProtectionDisable.
*
*********************************************************************************/
static inline void FTM_WriteProtectionDisable(FTM_Type* pFTM)
{
    if (pFTM->FMS & FTM_FMS_WPEN_MASK)
    {
        pFTM->MODE |= FTM_MODE_WPDIS_MASK;
    }
}

/*****************************************************************************//*!
*
* @brief  set FTMEN bit to enable FTM-specific register.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
*    
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetFTMBasic.
*
*****************************************************************************/
static inline void FTM_SetFTMEnhanced(FTM_Type* pFTM)
{
    if(pFTM->MODE & FTM_MODE_WPDIS_MASK)   /* if not write protected */
    {
        pFTM->MODE |= FTM_MODE_FTMEN_MASK;
    }
    else
    {
        FTM_WriteProtectionDisable(pFTM);
        pFTM->MODE |= FTM_MODE_FTMEN_MASK;
        FTM_WriteProtectionEnable(pFTM);
    }    
}

/*****************************************************************************//*!
*
* @brief  clear FTMEN bit to disable FTM-specific registers, only TPM-compatible
*         registers can be used.
*        
* @param[in]    pFTM           pointer to one of three FTM base register address.
*    
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetFTMEnhanced.
*
*****************************************************************************/
static inline void FTM_SetFTMBasic(FTM_Type* pFTM)
{
    if(pFTM->MODE & FTM_MODE_WPDIS_MASK)    /* if not write protected */
    {
        pFTM->MODE &= ~FTM_MODE_FTMEN_MASK;
    }
    else
    {
        FTM_WriteProtectionDisable(pFTM);
        pFTM->MODE &= ~FTM_MODE_FTMEN_MASK;
        FTM_WriteProtectionEnable(pFTM);
    }  
}

/*****************************************************************************//*!
*
* @brief  set the ftm mod value.
*              
* @param[in]    pFTM           pointer to one of three FTM base register address.
* @param[in]    u16ModValue    the mod value required to set.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetChannelValue.
*
*****************************************************************************/
static inline void FTM_SetModValue(FTM_Type* pFTM, uint16_t u16ModValue)
{
    pFTM->CNT = 0;
    pFTM->MOD = u16ModValue;
    if((FTM_Type*)FTM2 == pFTM)
    {
        if(pFTM->MODE & FTM_MODE_FTMEN_MASK)
        {
            pFTM->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;
        }
        else
        {
        }
    }
    else
    {
    }
}

/*****************************************************************************//*!
*
* @brief  set the ftm channel value.
*                
* @param[in]    pFTM             pointer to one of three FTM base register address.
* @param[in]    u16ChannelValue  the CnV value required to set.
* @param[in]    u8FTM_Channel    ftm channel number.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetModValue.
*
*****************************************************************************/
static inline void FTM_SetChannelValue(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint16_t u16ChannelValue)
{
    pFTM->CONTROLS[u8FTM_Channel].CnV = u16ChannelValue;
    if((FTM_Type*)FTM2 == pFTM)
    {
        if(pFTM->MODE & FTM_MODE_FTMEN_MASK)
        {
            if(u8FTM_Channel < 2)
            {
                pFTM->COMBINE |= FTM_COMBINE_SYNCEN0_MASK;  /* enable the sync function */
            }
            else if (u8FTM_Channel < 4)
            {
                pFTM->COMBINE |= FTM_COMBINE_SYNCEN1_MASK;
            }
            else
            {
                pFTM->COMBINE |= FTM_COMBINE_SYNCEN2_MASK;
            }
            pFTM->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;  
        }
        else
        {
        }
    }
    else
    {
    }
}

/*****************************************************************************//*!
*
* @brief  set the ftm channel value.
*               
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u16CounterValue   the CNTIN value required to set.
*
* @return none.
*
* @ Pass/ Fail criteria: none
*
* @see FTM_SetModValue.
*
*****************************************************************************/
static inline void FTM_SetCounterInitValue(FTM_Type* pFTM, uint16_t u16CounterValue)
{
    pFTM->CNTIN = u16CounterValue;
    if(pFTM->MODE & FTM_MODE_FTMEN_MASK)
    {
        pFTM->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;
    }
    else
    {
    }
}

/*****************************************************************************//*!
*
* @brief  set the channel output mask value, FTM2 used only.
*               
* @param[in]    pFTM            pointer to one of three FTM base register address.
* @param[in]    u16ChMask       the CNTIN value required to set.
*
* @return none.
*
* @ Pass/ Fail criteria: none
*
* @see FTM_UnMaskChannels.
*
*****************************************************************************/
static inline void FTM_MaskChannels(FTM_Type* pFTM, uint16_t u16ChMask)
{
     pFTM->OUTMASK |= u16ChMask;
}

/*****************************************************************************//*!
*
* @brief  clear the channel output mask value, FTM2 used only.
*               
* @param[in]    pFTM            pointer to one of three FTM base register address.
* @param[in]    u16ChMask       the CNTIN value required to set.
*
* @return none.
*
* @ Pass/ Fail criteria: none
*
* @see FTM_MaskChannels.
*
*****************************************************************************/
static inline void FTM_UnMaskChannels(FTM_Type* pFTM, uint16_t u16ChMask)
{
     pFTM->OUTMASK &= ~u16ChMask;
}

/*********************************************************************************//*!
*
* @brief set FTM channels polarity.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
* @param[in]    u8ChsPolValue         the channels value need to be set.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_GetChannelsPolarity.
*
*********************************************************************************/
static inline void FTM_SetChannelsPolarity(FTM_Type* pFTM, uint8_t u8ChsPolValue)
{
    pFTM->POL = u8ChsPolValue;
}

/*********************************************************************************//*!
*
* @brief get FTM channels polarity.
*        
* @param[in]    pFTM                  pointer to one of three FTM base register address.
*
* @return uint8_t the channels polarity.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetChannelsPolarity.
*
*********************************************************************************/
static inline uint8_t FTM_GetChannelsPolarity(FTM_Type* pFTM)
{
    return (pFTM->POL);
}

/*********************************************************************************//*!
*
* @brief select the enhanced SYNC mode.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_DisableEnhancedSYNCMode.
*
*********************************************************************************/
static inline void FTM_EnableEnhancedSYNCMode(FTM_Type* pFTM)
{
    pFTM->SYNCONF   |= FTM_SYNCONF_SYNCMODE_MASK;   /* recommend enhanced sync mode */
}

/*********************************************************************************//*!
*
* @brief select the legacy SYNC mode.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_EnableEnhancedSYNCMode.
*
*********************************************************************************/
static inline void FTM_DisableEnhancedSYNCMode(FTM_Type* pFTM)
{
    pFTM->SYNCONF   &= ~FTM_SYNCONF_SYNCMODE_MASK;   /* recommend enhanced sync mode */
}

/*********************************************************************************//*!
*
* @brief set the external trigger source.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8TirggerSource   initial trigger or CHn(0~5)trigger
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_GetExternalTriggerFlag.
*
*********************************************************************************/
static inline void FTM_SetExternalTrigger(FTM_Type* pFTM, uint8_t u8TirggerSource)
{
    pFTM->EXTTRIG   = u8TirggerSource;
}

/*********************************************************************************//*!
*
* @brief get the external trigger flag.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return ex trigger flag.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetExternalTrigger.
*
*********************************************************************************/
static inline uint8_t FTM_GetExternalTriggerFlag(FTM_Type* pFTM)
{
    return (pFTM->EXTTRIG & FTM_EXTTRIG_TRIGF_MASK);
}

/*********************************************************************************//*!
*
* @brief set LDOK bit.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetLoadMatchChannel.
*
*********************************************************************************/
static inline void FTM_SetLoadEnable(FTM_Type* pFTM)
{
    pFTM->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;
}

/*********************************************************************************//*!
*
* @brief set the channel in the matching process.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8Matchchannel    the channel in the matching process.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_SetLoadEnable.
*
*********************************************************************************/
static inline void FTM_SetLoadMatchChannel(FTM_Type* pFTM, uint8_t u8Matchchannel)
{
    pFTM->PWMLOAD |= u8Matchchannel;
}

/*********************************************************************************//*!
*
* @brief disable the channel input capture filter.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FTM_Channel     the channel number: 0~3.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_InputCaptureFilterSet.
*
*********************************************************************************/
static inline void FTM_InputCaptureFilterClr(FTM_Type* pFTM, uint8_t u8FTM_Channel)
{
    pFTM->FILTER &= ~(0x000F << (u8FTM_Channel << 2));
}

/*********************************************************************************//*!
*
* @brief set the channel input capture filter value.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FTM_Channel     the channel number: 0~3.
* @param[in]    u8FilterValue     fliter cycles:1~15, 0: disable channel filter.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_InputCaptureFilterClr.
*
*********************************************************************************/
static inline void FTM_InputCaptureFilterSet(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8FilterValue)
{
    if (u8FilterValue)
    {
        pFTM->FILTER |= (u8FilterValue << (u8FTM_Channel << 2));
    }
    else
    {
        FTM_InputCaptureFilterClr(pFTM, u8FTM_Channel);
    }
}


/*********************************************************************************//*!
*
* @brief enable the fault input pin.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FaultPin        the fault input channel number: 0~3.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_FaultPinDisable.
*
*********************************************************************************/
static inline void FTM_FaultPinEnable(FTM_Type* pFTM, uint8_t u8FaultPin)
{
    if (pFTM->MODE & FTM_MODE_WPDIS_MASK) /* if not protected */
    {
        pFTM->FLTCTRL |= (1 << u8FaultPin);
    }
    else                                  /* if protected */ 
    {
        FTM_WriteProtectionDisable(pFTM);
        pFTM->FLTCTRL |= (1 << u8FaultPin);
        FTM_WriteProtectionEnable(pFTM);
    }
}

/*********************************************************************************//*!
*
* @brief disable the fault input pin.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FaultPin        the fault input channel number: 0~3.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_FaultPinEnable.
*
*********************************************************************************/
static inline void FTM_FaultPinDisable(FTM_Type* pFTM, uint8_t u8FaultPin)
{
    if (pFTM->MODE & FTM_MODE_WPDIS_MASK) /* if not protected */
    {
        pFTM->FLTCTRL &= ~(1 << u8FaultPin);
    }
    else                                  /* if protected */ 
    {
        FTM_WriteProtectionDisable(pFTM);
        pFTM->FLTCTRL &= ~(1 << u8FaultPin);
        FTM_WriteProtectionEnable(pFTM);
    }
}

/*********************************************************************************//*!
*
* @brief enable the fault pin filter.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FaultPin        the fault input channel number: 0~3.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_FaultPinFilterDisable.
*
*********************************************************************************/
static inline void FTM_FaultPinFilterEnable(FTM_Type* pFTM, uint8_t u8FaultPin)
{
    if (pFTM->MODE & FTM_MODE_WPDIS_MASK) /* if not protected */
    {
        pFTM->FLTCTRL |= (0x10 << u8FaultPin);
    }
    else                                  /* if protected */ 
    {
        FTM_WriteProtectionDisable(pFTM);
        pFTM->FLTCTRL |= (0x10 << u8FaultPin);
        FTM_WriteProtectionEnable(pFTM);
    }
}

/*********************************************************************************//*!
*
* @brief disable the fault pin filter.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FaultPin        the fault input channel number: 0~3.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_FaultPinFilterDisable.
*
*********************************************************************************/
static inline void FTM_FaultPinFilterDisable(FTM_Type* pFTM, uint8_t u8FaultPin)
{
    if (pFTM->MODE & FTM_MODE_WPDIS_MASK) /* if not protected */
    {
        pFTM->FLTCTRL &= ~(0x10 << u8FaultPin);
    }
    else                                  /* if protected */ 
    {
        FTM_WriteProtectionDisable(pFTM);
        pFTM->FLTCTRL &= ~(0x10 << u8FaultPin);
        FTM_WriteProtectionEnable(pFTM);
    }
}

/*********************************************************************************//*!
*
* @brief disable all the fault pins filter together. 
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_FaultPinFilterSet.
*
*********************************************************************************/
static inline void FTM_FaultPinFilterCDisableAll(FTM_Type* pFTM)
{
    pFTM->FLTCTRL &= ~FTM_FLTCTRL_FFVAL_MASK;
}

/*********************************************************************************//*!
*
* @brief set the fault filter value. All channels share the same filter value.
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FilterValue     the fault input filter value: 1~15, 0 disable the filter.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_FaultPinFilterCDisableAll.
*
*********************************************************************************/
static inline void FTM_FaultPinFilterSet(FTM_Type* pFTM, uint8_t u8FilterValue)
{
    if (u8FilterValue)
    {
        pFTM->FLTCTRL |= FTM_FLTCTRL_FFVAL(u8FilterValue);
    }
    else
    {
        FTM_FaultPinFilterCDisableAll(pFTM);
    }
}

/*********************************************************************************//*!
*
* @brief get the logic OR of all the fault detection flags
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_GetFaultDetectionFlag.
*
*********************************************************************************/
static inline uint8_t FTM_GetFaultDetectionLogicORFlag(FTM_Type* pFTM)
{
    return (pFTM->FMS & FTM_FMS_FAULTF_MASK);
}

/*********************************************************************************//*!
*
* @brief get the fault detection flag
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
* @param[in]    u8FaultPin        fault input pin number: 0~3.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
* @see FTM_GetFaultDetectionLogicORFlag.
*
*********************************************************************************/
static inline uint8_t FTM_GetFaultDetectionFlag(FTM_Type* pFTM, uint8_t u8FaultPin)
{
    return (pFTM->FMS & (1 << u8FaultPin));
}

/*********************************************************************************//*!
*
* @brief get the logic OR value of the fault inputs
*        
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/
static inline uint8_t FTM_GetFaultInputsLogicORValue(FTM_Type* pFTM)
{
    return (pFTM->FMS & FTM_FMS_FAULTIN_MASK);
}


/*********************************************************************************//*!
*
* @brief Get the current count value of the FTM
*
* @param[in]    pFTM              pointer to one of three FTM base register address.
*
* @return uint16_t.
*
* @ Pass/ Fail criteria: none.
*
*********************************************************************************/

static inline uint16_t FTM_GetCountValue(FTM_Type* pFTM){
	return (uint16_t) pFTM->CNT;
}

/*! @} End of ftm_api_list                                                    */

/******************************************************************************
* Global functions
******************************************************************************/

void FTM_ClockSet(FTM_Type* pFTM, uint8_t u8ClockSource, uint8_t u8ClockPrescale);
void FTM_PWMInit(FTM_Type* pFTM, uint8_t u8PWMModeSelect, uint8_t u8PWMEdgeSelect);
void FTM_InputCaptureInit(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8CaptureMode);
void FTM_DualEdgeCaptureInit(FTM_Type* pFTM, uint8_t u8ChannelPair, uint8_t u8CaptureMode, 
                             uint8_t u8Channel_N_Edge, uint8_t u8Channel_Np1_Edge);
void FTM_OutputCompareInit(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8CompareMode);
void FTM_SoftwareSync(FTM_Type* pFTM);
void FTM_HardwareSync(FTM_Type* pFTM, uint8_t u8TriggerN);
void FTM_HardwareSyncCombine(FTM_Type* pFTM, uint8_t u8TriggerMask);
void FTM_GenerateTrig2(FTM_Type* pFTM);
void FTM_PWMDeadtimeSet(FTM_Type* pFTM, uint8_t u8PrescalerValue, uint8_t u8DeadtimeValue);
void FTM_OutputMaskSet(FTM_Type* pFTM, uint8_t u8FTM_Channel);
void FTM_SWOutputControlSet(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8ChannelValue);
void FTM_SetDebugModeBehavior(FTM_Type* pFTM, uint8_t u8DebugMode);
void FTM_SetTOFFrequency(FTM_Type* pFTM, uint8_t u8TOFNUM);
void FTM_PolaritySet(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8ActiveValue);
void FTM_InvertChannel(FTM_Type* pFTM, uint8_t u8ChannelPair);
void FTM_Init(FTM_Type* pFTM, FTM_ConfigType *pConfig);
void FTM_DeInit(FTM_Type* pFTM);
void FTM_ChannelInit(FTM_Type* pFTM, uint8_t u8FTM_Channel, FTM_ChParamsType pFTM_ChParams);
void FTM_SetCallback(FTM_Type* pFTM, FTM_CallbackPtr pfnCallback);
void  FTM_SyncConfigActivate(FTM_Type* pFTM, uint32_t u32ConfigValue);
void FTM_SyncConfigDeactivate(FTM_Type* pFTM, uint32_t u32ConfigValue);
uint8_t FTM_GetFaultDetectionLogicORFlag(FTM_Type* pFTM);
uint8_t FTM_GetFaultDetectionFlag(FTM_Type* pFTM, uint8_t u8FaultPin);
uint8_t FTM_GetFaultInputsLogicORValue(FTM_Type* pFTM);
void FTM_WriteProtectionEnable(FTM_Type* pFTM);
void FTM_WriteProtectionDisable(FTM_Type* pFTM);
void FTM_FaultPinFilterCDisableAll(FTM_Type* pFTM);
void FTM_FaultPinFilterSet(FTM_Type* pFTM, uint8_t u8FilterValue);
void FTM_FaultPinFilterDisable(FTM_Type* pFTM, uint8_t u8FaultPin);
void FTM_FaultPinFilterEnable(FTM_Type* pFTM, uint8_t u8FaultPin);
void FTM_FaultPinEnable(FTM_Type* pFTM, uint8_t u8FaultPin);
void FTM_FaultPinDisable(FTM_Type* pFTM, uint8_t u8FaultPin);
void FTM_InputCaptureFilterClr(FTM_Type* pFTM, uint8_t u8FTM_Channel);
void FTM_InputCaptureFilterSet(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint8_t u8FilterValue);
void FTM_SetLoadMatchChannel(FTM_Type* pFTM, uint8_t u8Matchchannel);
void FTM_SetLoadEnable(FTM_Type* pFTM);
uint8_t FTM_GetExternalTriggerFlag(FTM_Type* pFTM);
void FTM_DisableEnhancedSYNCMode(FTM_Type* pFTM);
void FTM_EnableEnhancedSYNCMode(FTM_Type* pFTM);
uint8_t FTM_GetChannelsPolarity(FTM_Type* pFTM);
void FTM_SetChannelsPolarity(FTM_Type* pFTM, uint8_t u8ChsPolValue);
void FTM_UnMaskChannels(FTM_Type* pFTM, uint16_t u16ChMask);
void FTM_MaskChannels(FTM_Type* pFTM, uint16_t u16ChMask);
void FTM_SetCounterInitValue(FTM_Type* pFTM, uint16_t u16CounterValue);
void FTM_SetChannelValue(FTM_Type* pFTM, uint8_t u8FTM_Channel, uint16_t u16ChannelValue);
void FTM_SetModValue(FTM_Type* pFTM, uint16_t u16ModValue);
void FTM_SetFTMBasic(FTM_Type* pFTM);
void FTM_SetFTMEnhanced(FTM_Type* pFTM);
void FTM_ClrChannelFlag(FTM_Type* pFTM, uint8_t u8FTM_Channel);
uint8_t FTM_GetChannelFlag(FTM_Type* pFTM, uint8_t u8FTM_Channel);
void FTM_ClrOverFlowFlag(FTM_Type* pFTM);
uint8_t FTM_GetOverFlowFlag(FTM_Type* pFTM);
void FTM_DisableChannelInt(FTM_Type* pFTM, uint8_t u8FTM_Channel);
void FTM_EnableChannelInt(FTM_Type* pFTM, uint8_t u8FTM_Channel);
void FTM_DisableOverflowInt(FTM_Type* pFTM);
void FTM_EnableOverflowInt(FTM_Type* pFTM);
uint16_t FTM_GetCountValue(FTM_Type* pFTM);

#ifdef __cplusplus
}
#endif
#endif /* FTM_H_ */
