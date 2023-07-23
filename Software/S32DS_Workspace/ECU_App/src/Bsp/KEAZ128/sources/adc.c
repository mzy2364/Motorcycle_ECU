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
* @file adc.c
*
* @author Freescale
*
*
* @brief providing APIs for configuring ADC module (ADC). 
*
*******************************************************************************
*
* provide APIs for configuring ADC module (ADC)
******************************************************************************/
#include "adc.h"
#include "nvic.h"
/******************************************************************************
* Local function
******************************************************************************/
ADC_CallbackType ADC_Callback[1] = {(0)};
/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* define ADC APIs
*
*//*! @addtogroup adc_api_list
* @{
*******************************************************************************/


/*****************************************************************************//**
   *
   * @brief initialize ADC module.
   *
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  pADC_Config point to ADC configuration structure. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void ADC_Init(ADC_MemMapPtr pADC, ADC_ConfigTypePtr pADC_Config)
{
    if( pADC == ADC)
    {
        SIM->SCGC |= SIM_SCGC_ADC_MASK;
    }

    /* set clock cource for ADC */
    ADC_SelectClock(pADC,pADC_Config->u8ClockSource);

    /* set clock divide */
    ADC_SelectClockDivide(pADC,pADC_Config->u8ClockDiv);

    /* set ADC mode */
    ADC_SetMode(pADC,pADC_Config->u8Mode);

    /* set FIFO level */
    ADC_SetFifoLevel(pADC,pADC_Config->u8FiFoLevel);

    /* set pin control */
    pADC->APCTL1 = pADC_Config->u16PinControl;

    if( pADC_Config->sSetting.bCompareEn )
    {
        ADC_CompareEnable(pADC);
    }
    
    if( pADC_Config->sSetting.bCompareGreaterEn )
    {
        ADC_CompareGreaterFunction(pADC);
    }
        
    if( pADC_Config->sSetting.bContinuousEn )
    {
        ADC_ContinuousConversion(pADC);
    }
        
    if( pADC_Config->sSetting.bCompareAndEn ) 
    {
        ADC_CompareFifoAnd(pADC);
    }
    
    if( pADC_Config->sSetting.bFiFoScanModeEn )
    {
        ADC_FifoScanModeEnable(pADC);
    }
    
    if( pADC_Config->sSetting.bHardwareTriggerEn )
    {
        ADC_SetHardwareTrigger(pADC);
    }

    if( pADC_Config->sSetting.bIntEn )
    {
        ADC_IntEnable(pADC);
#if defined(MCU_SKEAZ1284)
        Enable_Interrupt(ADC_IRQn);
#elif defined(MCU_SKEAZN642)
		Enable_Interrupt(ADC0_IRQn);
#else
/* Enable ADC interrupt for your device. */
#endif
    } 

    if( pADC_Config->sSetting.bLongSampleEn )
    {
        ADC_SetLongSample(pADC);
    } 

    if( pADC_Config->sSetting.bLowPowerEn )
    {
        ADC_SetLowPower(pADC);
    }
#if defined(MCU_SKEAZ1284)
    if( pADC_Config->sSetting.bHTRGMEn )
    {
        ADC_HardwareTriggerMultiple(pADC);
    }
    else
    {
		ADC_HardwareTriggerSingle(pADC);
    }
    
    if( pADC_Config->sSetting.bHTRGMASKEn )
    {
        ADC_HardwareTriggerMaskEnable(pADC);
    }
    else
    {
		ADC_HardwareTriggerMaskDisable(pADC);
    }
    if( pADC_Config->sSetting.bHTRGMASKSEL )
    {
        ADC_HardwareTriggerMaskAuto(pADC);
    }
    else
    {
		ADC_HardwareTriggerMaskNonAuto(pADC);
    }
#elif defined(MCU_SKEAZN642)
/* KEAZN64 does not support hardware trigger. */
#else
/* Add any additional features of your device's ADC. */
#endif
}

/*****************************************************************************//*!
   *
   * @brief disable ADC module.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none.
   *****************************************************************************/
void ADC_DeInit( ADC_MemMapPtr pADC )
{
    ADC_SetChannel(pADC,ADC_CHANNEL_DISABLE);

    SIM->SCGC &= ~SIM_SCGC_ADC_MASK;
}

/*****************************************************************************//*!
   *
   * @brief start a conversion and get conversion result
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u8Channel adc channel to conversion. 
   *
   * @return ADC conversion result.
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
unsigned int ADC_PollRead( ADC_MemMapPtr pADC, uint8_t u8Channel )
{
		ADC_SetChannel(pADC,u8Channel);
		while( !ADC_IsCOCOFlag(pADC) );
		return ADC_ReadResultReg(pADC);
}


/*****************************************************************************//*!
   *
   * @brief install ADC call back function.
   *        
   * @param[in]	 pADC_CallBack point to address of  adc call back function.
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none.
   *****************************************************************************/
void ADC_SetCallBack(ADC_CallbackType pADC_CallBack)
{
    ADC_Callback[0] = pADC_CallBack;
}

/*****************************************************************************//*!
   *
   * @brief set ADC channel.
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u8Channel adc channel to conversion. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void ADC_SetChannel( ADC_MemMapPtr pADC, uint8_t u8Channel )
{
    uint32_t u32temp;    
    u32temp = pADC->SC1; 
    u32temp &= ~ADC_SC1_ADCH_MASK;
    pADC->SC1 = u32temp|ADC_SC1_ADCH(u8Channel);   
}
/*****************************************************************************//*!
   *
   * @brief Voltage Reference Selection.
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u8Vref adc reference voltage selection. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void ADC_VrefSelect( ADC_MemMapPtr pADC, uint8_t u8Vref )
{
    uint32_t u32Temp;
    u32Temp = pADC->SC2;
    u32Temp &= ~ADC_SC2_REFSEL_MASK;
    pADC->SC2 = u32Temp|ADC_SC2_REFSEL(u8Vref);
}

/*****************************************************************************//*!
   *
   * @brief select clock divide
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u8Div Clock Divide Select. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void ADC_SelectClockDivide( ADC_MemMapPtr pADC, uint8_t u8Div )
{
    uint32_t u32Temp;
    u32Temp = pADC->SC3;
    u32Temp &= ~ADC_SC3_ADIV_MASK;
    pADC->SC3 = u32Temp|ADC_SC3_ADIV(u8Div);
}

/*****************************************************************************//*!
   *
   * @brief set ADC mode.
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u8Mode Conversion Mode Selection. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void ADC_SetMode( ADC_MemMapPtr pADC, uint8_t u8Mode )
{
    uint32_t u32Temp;
    u32Temp = pADC->SC3;
    u32Temp &= ~ADC_SC3_MODE_MASK;
    pADC->SC3 = u32Temp|ADC_SC3_MODE(u8Mode);
}
/*****************************************************************************//*!
   *
   * @brief Input Clock Select.
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u8Clock Input Clock Select. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void ADC_SelectClock( ADC_MemMapPtr pADC, uint8_t u8Clock )
{
    uint32_t u32Temp;
    u32Temp = pADC->SC3;
    u32Temp &= ~ADC_SC3_ADICLK_MASK;
    pADC->SC3 = u32Temp|ADC_SC3_ADICLK(u8Clock);
}

/*****************************************************************************//*!
   *
   * @brief FIFO Depth enables
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u8FifoLevel set FIFO level. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
void ADC_SetFifoLevel( ADC_MemMapPtr pADC, uint8_t u8FifoLevel )
{
    uint32_t u32Temp;
    u32Temp = pADC->SC4;
    u32Temp &= ~ADC_SC4_AFDEP_MASK;
    pADC->SC4 = u32Temp|ADC_SC4_AFDEP(u8FifoLevel);
}

/*! @} End of adc_api_list                                               						*/


/*****************************************************************************//*!
   *
   * @brief ADC interrupt service routine.
   *        
   * @param  none. 
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none.
   *****************************************************************************/
#if defined(MCU_SKEAZ1284)
void ADC_IRQHandler(void)
#elif defined(MCU_SKEAZN642)
void ADC0_IRQHandler(void)
#else
/* Declare this interrupt handler for your device. */
#endif
{
    if( ADC_Callback[0] )
    {
        ADC_Callback[0]();
    }
}















