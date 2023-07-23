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
* @file adc.h
*
* @author Freescale
*
*
* @brief header file for ADC module utilities (ADC). 
*
*******************************************************************************
*
* provide APIs for accessing ADC module (ADC)
******************************************************************************/

#ifndef ADC_H_
#define ADC_H_
#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
* Macros
******************************************************************************/
#include "derivative.h"
/******************************************************************************
*define ADC refernce voltage 
*
*//*! @addtogroup adc_ref_list
* @{
*******************************************************************************/

#define ADC_VREF_VREFH                 0x00			/*!< ADC reference voltage is VREFH*/
#define ADC_VREF_VDDA                  0x01			/*!< ADC reference voltage is VDDA*/

/*! @} End of adc_ref_list                                                    						*/

/******************************************************************************
* define ADC clock source 
*
*//*! @addtogroup adc_clock_source_list
* @{
*******************************************************************************/

#define CLOCK_SOURCE_BUS_CLOCK							0x00	/*!< ADC clock source is bus clock*/
#define CLOCK_SOURCE_BUS_CLOCK_DIVIDE_2			        0x01	/*!< ADC clock source is bus clock devided by 2*/
#define CLOCK_SOURCE_ALTCLK								0x02	/*!< ADC clock source is alternative clcok*/
#define CLOCK_SOURCE_ADACK								0x03	/*!< ADC clock source is asynchronous clock*/
/*! @} End of adc_clock_source_list                                          						*/


/******************************************************************************
* define ADC divider 
*
*//*! @addtogroup adc_clock_divider_list
* @{
*******************************************************************************/

#define ADC_ADIV_DIVIDE_1								0x00		/*!< ADC clock divide by 1*/
#define ADC_ADIV_DIVIDE_2								0x01		/*!< ADC clock divide by 2*/
#define ADC_ADIV_DIVIDE_4								0x02		/*!< ADC clock divide by 4*/
#define ADC_ADIV_DIVIDE_8								0x03		/*!< ADC clock divide by 8*/
/*! @} End of adc_clock_divider_list                                          						*/

/******************************************************************************
* define ADC mode 
*
*//*! @addtogroup adc_mode_list
* @{
*******************************************************************************/

#define ADC_MODE_8BIT									0x00		/*!< ADC 8bit mode*/
#define ADC_MODE_10BIT									0x01		/*!< ADC 10bit mode*/
#define ADC_MODE_12BIT									0x02		/*!< ADC 12bit mode */
/*! @} End of adc_mode_list                                               						*/

/******************************************************************************
* define ADC channel
*
*//*! @addtogroup adc_channel_list
* @{
*******************************************************************************/

#define ADC_CHANNEL_AD0                                 0x0 /*!< ADC input channel 0 */
#define ADC_CHANNEL_AD1                                 0x1 /*!< ADC input channel 1 */
#define ADC_CHANNEL_AD2                                 0x2 /*!< ADC input channel 2 */
#define ADC_CHANNEL_AD3                                 0x3 /*!< ADC input channel 3 */
#define ADC_CHANNEL_AD4                                 0x4 /*!< ADC input channel 4 */
#define ADC_CHANNEL_AD5                                 0x5 /*!< ADC input channel 5 */
#define ADC_CHANNEL_AD6                                 0x6 /*!< ADC input channel 6 */
#define ADC_CHANNEL_AD7                                 0x7 /*!< ADC input channel 7 */
#define ADC_CHANNEL_AD8                                 0x8 /*!< ADC input channel 8 */
#define ADC_CHANNEL_AD9                                 0x9 /*!< ADC input channel 9 */
#define ADC_CHANNEL_AD10                                 0xa /*!< ADC input channel 10 */
#define ADC_CHANNEL_AD11                                 0xb /*!< ADC input channel 11 */
#define ADC_CHANNEL_AD12                                 0xc /*!< ADC input channel 12 */
#define ADC_CHANNEL_AD13                                 0xd /*!< ADC input channel 13 */
#define ADC_CHANNEL_AD14                                 0xe /*!< ADC input channel 14 */
#define ADC_CHANNEL_AD15                                 0xf /*!< ADC input channel 15 */
#define ADC_CHANNEL_AD18_VSS                            0x12 /*!< ADC input channel VSS */ 
#define ADC_CHANNEL_AD22_TEMPSENSOR                     0x16 /*!< ADC input channel internal temperature sensor */
#define ADC_CHANNEL_AD23_BANDGAP                        0x17 /*!< ADC input channel bandgap */ 
#define ADC_CHANNEL_AD29_VREFH                          0x1D /*!< ADC input channel Vrefh */ 
#define ADC_CHANNEL_AD30_VREFL                          0x1E /*!< ADC input channel Vrefl */ 
#define ADC_CHANNEL_DISABLE                             0x1F /*!< ADC disable */
/*! @} End of adc_channel_list                                               						*/


/******************************************************************************
* define ADC FIFO_LEVEL
*
*//*! @addtogroup adc_fifo_level_list
* @{
*******************************************************************************/
#define ADC_FIFO_DISABLE                                0 /*!< FIFO Level 0 */
#define ADC_FIFO_LEVEL2                                 1 /*!< FIFO Level 1 */
#define ADC_FIFO_LEVEL3                                 2 /*!< FIFO Level 2 */
#define ADC_FIFO_LEVEL4                                 3 /*!< FIFO Level 3 */
#define ADC_FIFO_LEVEL5                                 4 /*!< FIFO Level 4 */
#define ADC_FIFO_LEVEL6                                 5 /*!< FIFO Level 5 */
#define ADC_FIFO_LEVEL7                                 6 /*!< FIFO Level 6 */
#define ADC_FIFO_LEVEL8                                 7	/*!< FIFO Level 7 */
/*! @} End of adc_fifo_level_list                                               						*/


/******************************************************************************
* define ADC trigger source
*
*//*! @addtogroup adc_trigger_list
* @{
*******************************************************************************/
#define ADC_HARDWARE_TRIGGER                            0x01  /*!< hardware trigger */
#define ADC_SOFTWARE_TRIGGER                            0x00	/*!< software trigger */
#define ADC_TRIGGER_RTC                                 0x00  /*!< RTC act as trigger source */          
#define ADC_TRIGGER_PIT                                 0x01  /*!< PIT act as trigger source */ 
#define ADC_TRIGGER_FTM2INIT                            0x10  /*!< FTM2 initialization act as trigger source */ 
#define ADC_TRIGGER_FTM2MATCH                           0x11  /*!< FTM2 match interrupt act as trigger source */
/*! @} End of adc_trigger_list                                               						*/


#define ADC_COMPARE_LESS                                0x00
#define ADC_COMPARE_GREATER                             0x01


/******************************************************************************
* define ADC call back
*
*//*! @addtogroup adc_callback
* @{
*******************************************************************************/
typedef void (*ADC_CallbackType)(void);							/*!< ADC call back function */
/*! @} End of adc_callback                                               						*/

/******************************************************************************
* 
*
*//*! @addtogroup adc_setting_type
* @{
*******************************************************************************/
/*!
 * @brief ADC setting type.
 *
 */
typedef struct
{
    uint16_t bIntEn                 :1;     /*!< 1: Interrupt Enable, 0: Interrupt disable */
    uint16_t bContinuousEn          :1;     /*!< 1: Continuous Conversion Enable, 0: Continuous Conversion disable */
    uint16_t bHardwareTriggerEn     :1;     /*!< 1: hardware trigger, 0: software trigger */
    uint16_t bCompareEn             :1;     /*!< 1: compare mode Enable, 0: compare mode disable */
    uint16_t bCompareGreaterEn      :1;     /*!< 1: Compare greater mode, 0: compare less than mode */
    uint16_t bLowPowerEn            :1;     /*!< 1: Low power mode, 0: high speed mode */
    uint16_t bLongSampleEn          :1;     /*!< 1: long sample mode, 0: short sample mode */
    uint16_t bFiFoScanModeEn        :1;     /*!< 1: FIFO scan mode enable, 0: FIFO scan mode disable */
    uint16_t bCompareAndEn          :1;     /*!< 1: Compare and logic, 0: Compare and logic */
#if defined(MCU_SKEAZ1284)    
    uint16_t bHTRGMEn               :1;     /*!< one hardware trigger pulse trigger multiple conversions in fifo mode */      
    uint16_t bHTRGMASKEn            :1;		/*!< Hardware trigger mask enable. */
    uint16_t bHTRGMASKSEL           :1;		/*!< This field selects hardware trigger mask mode. */
#elif defined(MCU_SKEAZN642)
	/* KEAZN64 does not feature hardware trigger */
	uint16_t bReserve				:7;		/*!< 7: Reserved fields */
#else
	/* Write any additional features for your device's ADC. */
#endif

}ADC_SettingType;
/*! @} End of adc_setting_type                                               						*/

/******************************************************************************
* 
*
*//*! @addtogroup adc_config_type
* @{
*******************************************************************************/
/*!
 * @brief ADC configure type.
 *
 */
typedef struct
{
    ADC_SettingType sSetting;               /*!< ADC setting structure*/
    uint16_t u16PinControl;                 /*!< pin control */
    uint8_t u8ClockSource;                  /*!< clock source selection */
    uint8_t u8ClockDiv;                     /*!< set clock divider */
    uint8_t u8Mode;                         /*!< set ADC mode(8/10/12 bit mode) */
    uint8_t u8FiFoLevel;                    /*!< set FIFO level */
}ADC_ConfigType,*ADC_ConfigTypePtr;
/*! @} End of adc_config_type                                               */

/******************************************************************************
* define ADC APIs
*
*//*! @addtogroup adc_api_list
* @{
*******************************************************************************/
/*****************************************************************************//*!
   *
   * @brief enable ADC interrupt.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_IntEnable( ADC_MemMapPtr pADC )
{
    pADC->SC1 |= ADC_SC1_AIEN_MASK;   
}
/*****************************************************************************//*!
   *
   * @brief disable ADC interrupt.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_IntDisable( ADC_MemMapPtr pADC )
{
    pADC->SC1 &= ~ADC_SC1_AIEN_MASK;   
}
/*****************************************************************************//*!
   *
   * @brief enable ADC continuous conversion.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_ContinuousConversion( ADC_MemMapPtr pADC )
{
    pADC->SC1 |= ADC_SC1_ADCO_MASK;   
}
/*****************************************************************************//*!
   *
   * @brief enable ADC single conversion
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_SingleConversion( ADC_MemMapPtr pADC )
{
    pADC->SC1 &= ~ADC_SC1_ADCO_MASK;   
}
/*****************************************************************************//*!
   *
   * @brief set the ADC to hardware trigger.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none.
   *****************************************************************************/
static inline void ADC_SetHardwareTrigger( ADC_MemMapPtr pADC )
{
    pADC->SC2 |= ADC_SC2_ADTRG_MASK;
}
/*****************************************************************************//*!
   *
   * @brief set the ADC to software trigger.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_SetSoftwareTrigger( ADC_MemMapPtr pADC )
{
    pADC->SC2 &= ~ADC_SC2_ADTRG_MASK;
}
/*****************************************************************************//*!
   *
   * @brief enable ADC compare function.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_CompareEnable( ADC_MemMapPtr pADC )
{
    pADC->SC2 |= ADC_SC2_ACFE_MASK;
}
/*****************************************************************************//*!
   *
   * @brief disable ADC compare function.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_CompareDisable( ADC_MemMapPtr pADC )
{
    pADC->SC2 &= ~ADC_SC2_ACFE_MASK;
}
/*****************************************************************************//*!
   *
   * @brief enable ADC compare greater function.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_CompareGreaterFunction( ADC_MemMapPtr pADC )
{
    pADC->SC2 |= ADC_SC2_ACFGT_MASK;
}
/*****************************************************************************//*!
   *
   * @brief enable ADC compare less function.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_CompareLessFunction( ADC_MemMapPtr pADC )
{
    pADC->SC2 &= ~ADC_SC2_ACFGT_MASK;
}
/*****************************************************************************//*!
   *
   * @brief set ADC to low power configuration.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_SetLowPower( ADC_MemMapPtr pADC )
{
    pADC->SC3 |= ADC_SC3_ADLPC_MASK;
}
/*****************************************************************************//*!
   *
   * @brief set ADC to high speed configuration.
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_SetHighSpeed( ADC_MemMapPtr pADC )
{
    pADC->SC3 &= ~ADC_SC3_ADLPC_MASK;
}
/*****************************************************************************//*!
   *
   * @brief Long Sample Time Configuration.
   *        
   * @param[in]  pADC point to ADC module type.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_SetLongSample( ADC_MemMapPtr pADC )
{
    pADC->SC3 |= ADC_SC3_ADLSMP_MASK;
}
/*****************************************************************************//*!
   *
   * @brief Short Sample Time Configuration.
   *        
   * @param[in]  pADC point to ADC module type.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_SetShortSample( ADC_MemMapPtr pADC )
{
    pADC->SC3 &= ~ADC_SC3_ADLSMP_MASK;
}
/*****************************************************************************//*!
   *
   * @brief FIFO scan mode enable.
   *        
   * @param[in]  pADC point to ADC module type.
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none.
   *****************************************************************************/
static inline void ADC_FifoScanModeEnable( ADC_MemMapPtr pADC )
{
    pADC->SC4 |= ADC_SC4_ASCANE_MASK;
}
/*****************************************************************************//*!
   *
   * @brief FIFO scan mode disable.
   *        
   * @param[in]  pADC point to ADC module type.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_FifoScanModeDisable( ADC_MemMapPtr pADC )
{
    pADC->SC4 &= ~ADC_SC4_ASCANE_MASK;
}
/*****************************************************************************//*!
   *
   * @brief OR all of compare trigger.
   *        
   * @param[in]  pADC point to ADC module type.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_CompareFifoOr( ADC_MemMapPtr pADC )
{
    pADC->SC4 &= ~ADC_SC4_ACFSEL_MASK;
}
/*****************************************************************************//*!
   *
   * @brief And all of compare trigger.
   *        
   * @param[in]  pADC point to ADC module type.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_CompareFifoAnd( ADC_MemMapPtr pADC )
{
    pADC->SC4 |= ADC_SC4_ACFSEL_MASK;
}
/*****************************************************************************//*!
   *
   * @brief read ADC result register.
   *        
   * @param[in]  pADC point to ADC module type.
   *
   * @return ADC result value.
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline uint16_t ADC_ReadResultReg( ADC_MemMapPtr pADC )
{
    return (uint16_t)pADC->R;
}
/*****************************************************************************//*!
   *
   * @brief set ADC compare value.
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u16Compare compare value. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_SetCompareValue( ADC_MemMapPtr pADC, uint16_t u16Compare )
{
    pADC->CV = u16Compare;
}
/*****************************************************************************//*!
   *
   * @brief ADC pin control enable.
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u16PinNumber enable ADC function to specified pin number. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_PinControlEnable( ADC_MemMapPtr pADC, uint16_t u16PinNumber)
{
    pADC->APCTL1 &= ~(0x01<<u16PinNumber);
}
/*****************************************************************************//*!
   *
   * @brief ADC pin control disable.
   *        
   * @param[in]  pADC point to ADC module type. 
   * @param[in]  u16PinNumber  disable ADC function to specified pin number. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_PinControlDisable( ADC_MemMapPtr pADC, uint16_t u16PinNumber)
{
    pADC->APCTL1 |= (0x01<<u16PinNumber);
}
/*****************************************************************************//*!
   *
   * @brief check conversion active status
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return TRUE or FALSE
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline uint8_t ADC_IsConversionActiveFlag( ADC_MemMapPtr pADC )
{
    return(pADC->SC2 & ADC_SC2_ADACT_MASK);
}
/*****************************************************************************//*!
   *
   * @brief check COCO flag
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return TRUE or FALSE
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline uint8_t ADC_IsCOCOFlag( ADC_MemMapPtr pADC )
{
    return(pADC->SC1 & ADC_SC1_COCO_MASK);
}
/*****************************************************************************//*!
   *
   * @brief check Result FIFO empty
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return TRUE or FALSE
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline uint8_t ADC_IsFIFOEmptyFlag( ADC_MemMapPtr pADC )
{
    return(pADC->SC2 & ADC_SC2_FEMPTY_MASK);
}
/*****************************************************************************//*!
   *
   * @brief check Result FIFO full
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return TRUE or FALSE
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline uint8_t ADC_IsFIFOFullFlag( ADC_MemMapPtr pADC )
{
    return(pADC->SC2 & ADC_SC2_FFULL_MASK);
}
#if defined(MCU_SKEAZ1284)
#ifndef CPU_KE02
/*****************************************************************************//*!
   *
   * @brief Hardware Trigger Multiple Conversion Enable
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_HardwareTriggerMultiple( ADC_MemMapPtr pADC )
{
    pADC->SC4 |= ADC_SC4_HTRGME_MASK;
}
/*****************************************************************************//*!
   *
   * @brief Hardware Trigger Single Conversion
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_HardwareTriggerSingle( ADC_MemMapPtr pADC )
{
    pADC->SC4 &= ~ADC_SC4_HTRGME_MASK;
}
/*****************************************************************************//*!
   *
   * @brief Hardware Trigger Mask Enable
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_HardwareTriggerMaskEnable( ADC_MemMapPtr pADC )
{
    pADC->SC5 |= ADC_SC5_HTRGMASKE_MASK;
}
/*****************************************************************************//*!
   *
   * @brief Hardware Trigger Mask Disable
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_HardwareTriggerMaskDisable( ADC_MemMapPtr pADC )
{
    pADC->SC5 &= ~ADC_SC5_HTRGMASKE_MASK;
}
/*****************************************************************************//*!
   *
   * @brief Hardware Trigger Mask Mode Select Automatic Mode
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_HardwareTriggerMaskAuto( ADC_MemMapPtr pADC )
{
    pADC->SC5 |= ADC_SC5_HTRGMASKSEL_MASK;
}
/*****************************************************************************//*!
   *
   * @brief Hardware Trigger Mask Mode Select to be with HTRGMASKE
   *        
   * @param[in]  pADC point to ADC module type. 
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
   *****************************************************************************/
static inline void ADC_HardwareTriggerMaskNonAuto( ADC_MemMapPtr pADC )
{
    pADC->SC5 &= ~ADC_SC5_HTRGMASKSEL_MASK;
}
#endif
#elif defined(MCU_SKEAZN642)
/* KEAZN64 has no hardware trigger. */
#else
/* Write prototypes for any additional ADC features your device has. */
#endif



/******************************************************************************
* Global function
******************************************************************************/

void ADC_SetChannel( ADC_MemMapPtr pADC, uint8_t u8Channel );
void ADC_IntEnable( ADC_MemMapPtr pADC );
void ADC_IntDisable( ADC_MemMapPtr pADC );
void ADC_ContinuousConversion( ADC_MemMapPtr pADC );
void ADC_SingleConversion( ADC_MemMapPtr pADC );
void ADC_SetSoftwareTrigger( ADC_MemMapPtr pADC );
void ADC_SetHardwareTrigger( ADC_MemMapPtr pADC );
void ADC_VrefSelect( ADC_MemMapPtr pADC, uint8_t u8Vref );
void ADC_CompareEnable( ADC_MemMapPtr pADC );
void ADC_CompareDisable( ADC_MemMapPtr pADC );
void ADC_CompareGreaterFunction( ADC_MemMapPtr pADC );
void ADC_CompareLessFunction( ADC_MemMapPtr pADC );
void ADC_SetLowPower( ADC_MemMapPtr pADC );
void ADC_SetHighSpeed( ADC_MemMapPtr pADC );
void ADC_SelectClockDivide( ADC_MemMapPtr pADC, uint8_t u8Div);
void ADC_SetLongSample(ADC_MemMapPtr pADC);
void ADC_SetShortSample(ADC_MemMapPtr pADC);
void ADC_SetMode(ADC_MemMapPtr pADC, uint8_t u8Mode);
void ADC_SelectClock(ADC_MemMapPtr pADC, uint8_t u8Clock);
void ADC_FifoScanModeEnable(ADC_MemMapPtr pADC);
void ADC_FifoScanModeDisable(ADC_MemMapPtr pADC);
void ADC_CompareFifoOr(ADC_MemMapPtr pADC);
void ADC_CompareFifoAnd(ADC_MemMapPtr pADC);
void ADC_SetFifoLevel(ADC_MemMapPtr pADC, uint8_t u8FifoLevel);
uint16_t ADC_ReadResultReg(ADC_MemMapPtr pADC );
void ADC_SetCompareValue(ADC_MemMapPtr pADC, uint16_t u16Compare );
void ADC_PinControlEnable(ADC_MemMapPtr pADC, uint16_t u16PinNumber);
void ADC_PinControlDisable(ADC_MemMapPtr pADC, uint16_t u16PinNumber);
uint8_t ADC_IsConversionActiveFlag(ADC_MemMapPtr pADC);
uint8_t ADC_IsCOCOFlag(ADC_MemMapPtr pADC);
uint8_t ADC_IsFIFOEmptyFlag(ADC_MemMapPtr pADC);
uint8_t ADC_IsFIFOFullFlag(ADC_MemMapPtr pADC);
#if defined(MCU_SKEAZ1284)
void ADC_HardwareTriggerMaskNonAuto(ADC_MemMapPtr pADC);
void ADC_HardwareTriggerMaskAuto(ADC_MemMapPtr pADC);
void ADC_HardwareTriggerMaskDisable( ADC_MemMapPtr pADC );
void ADC_HardwareTriggerMaskEnable( ADC_MemMapPtr pADC );
void ADC_HardwareTriggerSingle( ADC_MemMapPtr pADC );
void ADC_HardwareTriggerMultiple( ADC_MemMapPtr pADC );
#elif defined(MCU_SKEAZN642)
/* No hardware trigger for KEAZN64. */
#else
/* Prototypes for additional ADC features of your device. */
#endif
unsigned int ADC_PollRead( ADC_MemMapPtr pADC, uint8_t u8Channel);
void ADC_SetCallBack(ADC_CallbackType pADC_CallBack);
void ADC_DeInit(ADC_MemMapPtr pADC);
void ADC_Init(ADC_MemMapPtr pADC, ADC_ConfigTypePtr pADC_Config);

/*! @} End of adc_api_list                                               						*/


#endif /* ADC_H_ */
