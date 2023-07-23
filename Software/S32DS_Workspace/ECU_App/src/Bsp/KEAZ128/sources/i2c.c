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
* @file i2c.c
*
* @author Freescale
*
*
* @brief providing APIs for configuring I2C module (I2C). 
*
*******************************************************************************
*
* provide APIs for configuring I2C module (I2C).
******************************************************************************/

#include "i2c.h"

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
#if defined(MCU_SKEAZ1284)
static I2C_CallbackType I2C_Callback[2] = {(I2C_CallbackType)(0)};
#elif defined(MCU_SKEAZN642)
static I2C_CallbackType I2C_Callback[1] = {(I2C_CallbackType)(0)};
#else
/* Write the I2C callback table for your device. */
#endif

#if defined(MCU_SKEAZ1284)
static I2C_Type* pI2C0 = (I2C_Type*)I2C0;
static I2C_Type* pI2C1 = (I2C_Type*)I2C1;
#elif defined(MCU_SKEAZN642)
//static I2C_Type* pI2C0 = (I2C_Type*)I2C0;
#else
/* Define the I2C pointers for your device */
#endif
/******************************************************************************
* Local functions
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/


/******************************************************************************
* define I2C APIs
*
*//*! @addtogroup i2c_api_list
* @{
*******************************************************************************/
/*****************************************************************************//*!
   *
   * @brief Initialize I2C module.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   * @param[in] pI2CConfig point to I2C configure structure.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/

void I2C_Init(I2C_Type* pI2Cx,I2C_ConfigPtr pI2CConfig)
{
    uint8_t u8Temp;


#if defined(MCU_SKEAZ1284)    
    if(pI2Cx == pI2C0)
    {
        SIM->SCGC |= SIM_SCGC_I2C0_MASK;
    }
    else
    {
        SIM->SCGC |= SIM_SCGC_I2C1_MASK;
    }
#elif defined(MCU_SKEAZN642)
	SIM->SCGC |= SIM_SCGC_I2C_MASK;
#else
	/* Gate your device's clock to the I2C. */
#endif


	I2C_SetBaudRate(pI2Cx,pI2CConfig->u16F);
    I2C_SetSlaveAddress(pI2Cx,pI2CConfig->u16OwnA1);
    pI2Cx->FLT = (uint8_t)pI2CConfig->u16Filt;
    pI2Cx->RA = (uint8_t)pI2CConfig->u16RangeA & 0xfe;
    I2C_SetSCLLowTimeout(pI2Cx,pI2CConfig->u16Slt);

    /* configure C2 control register */
    u8Temp = 0;
    if( pI2CConfig->sSetting.bGCAEn )
    {
        u8Temp |= I2C_C2_GCAEN_MASK;
    }
    if( pI2CConfig->sSetting.bAddressExt )
    {
        u8Temp |= I2C_C2_ADEXT_MASK;
    }
    if( pI2CConfig->sSetting.bRangeAddEn )
    {
        u8Temp |= I2C_C2_RMEN_MASK;
    }
    pI2Cx->C2 |= u8Temp;

    /* configure SMB register */
    u8Temp = 0;
    if( pI2CConfig->sSetting.bFackEn )
    {
        u8Temp |= I2C_SMB_FACK_MASK;
    }
    if( pI2CConfig->sSetting.bSMB_AlertEn )
    {
        u8Temp |= I2C_SMB_ALERTEN_MASK;
    }
    if( pI2CConfig->sSetting.bSecondAddressEn )
    {
        u8Temp |= I2C_SMB_SIICAEN_MASK;
    }
    if( pI2CConfig->sSetting.bSHTF2IntEn )
    {
        u8Temp |= I2C_SMB_SHTF2IE_MASK;
    }
    pI2Cx->SMB = u8Temp;
    
    /* configure C1 register */
    u8Temp = 0;
    if( pI2CConfig->sSetting.bIntEn )
    {
        u8Temp |= I2C_C1_IICIE_MASK;
#if defined(MCU_SKEAZ1284)
        if(pI2Cx == pI2C0)
        {
            Enable_Interrupt(I2C0_IRQn);
        }
    
        else if(pI2Cx == pI2C1)
        {
        
            Enable_Interrupt(I2C1_IRQn);
        }
#elif defined(MCU_SKEAZN642)
		Enable_Interrupt(I2C0_IRQn);
#else
/* Enable I2C interrupts in the NVIC of your device. */
#endif
        
    }
    
    if( pI2CConfig->sSetting.bWakeUpEn )
	{
	  u8Temp |= I2C_C1_WUEN_MASK;
	}
	
	
	if( pI2CConfig->sSetting.bMSTEn )
	{
	  u8Temp |= I2C_C1_MST_MASK;
	}
	
	if( pI2CConfig->sSetting.bI2CEn )
	{
	 u8Temp |= I2C_C1_IICEN_MASK;
	}
    
    pI2Cx->C1 = u8Temp;
 




}
/*****************************************************************************//*!
   *
   * @brief send out start signals.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   *
   * @return error status
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/
uint8_t I2C_Start(I2C_Type* pI2Cx)
{
    uint32_t u32Timeout;
    uint8_t u8ErrorStatus;
    
    u32Timeout = 0; 
    u8ErrorStatus = 0x00;

    I2C_TxEnable(pI2Cx);
    pI2Cx->C1 |= I2C_C1_MST_MASK;

    while( (!I2C_IsBusy(pI2Cx)) && ( u32Timeout < I2C_WAIT_STATUS_TIMEOUT))
     {
        u32Timeout ++;
     }

     if( u32Timeout == I2C_WAIT_STATUS_TIMEOUT )
     {
        u8ErrorStatus |= I2C_ERROR_START_NO_BUSY_FLAG;
     }

     return u8ErrorStatus;
}

/*****************************************************************************//*!
   *
   * @brief send out stop signals.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   *
   * @return error status
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/
uint8_t I2C_Stop(I2C_Type* pI2Cx)
{
    uint32_t u32Timeout;
    uint8_t u8ErrorStatus;
    
    u32Timeout = 0; 
    u8ErrorStatus = 0x00;
    
    pI2Cx->C1 &= ~I2C_C1_MST_MASK;

    while( (I2C_IsBusy(pI2Cx) ) && ( u32Timeout < I2C_WAIT_STATUS_TIMEOUT))
     {
        u32Timeout ++;
     }

     if( u32Timeout == I2C_WAIT_STATUS_TIMEOUT )
     {
        u8ErrorStatus |= I2C_ERROR_STOP_BUSY_FLAG;
     }

     return u8ErrorStatus;
}


/*****************************************************************************//*!
   *
   * @brief send out repeat start signals.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   *
   * @return error status.
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/
uint8_t I2C_RepeatStart(I2C_Type* pI2Cx)
{
    uint32_t u32Timeout;
    uint8_t u8ErrorStatus;
    
    u32Timeout = 0; 
    u8ErrorStatus = 0x00;
    
    pI2Cx->C1 |= I2C_C1_RSTA_MASK;

    while( (!I2C_IsBusy(pI2Cx) ) && ( u32Timeout < I2C_WAIT_STATUS_TIMEOUT))
     {
        u32Timeout ++;
     }

     if( u32Timeout == I2C_WAIT_STATUS_TIMEOUT )
     {
        u8ErrorStatus |= I2C_ERROR_START_NO_BUSY_FLAG;
     }

     return u8ErrorStatus;
}

/*****************************************************************************//*!
   *
   * @brief set slave address.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/
void I2C_SetSlaveAddress(I2C_Type* pI2Cx,uint16_t u16SlaveAddress)
{
    /* write low 8bit address */
    pI2Cx->A1 = (uint8_t)u16SlaveAddress;

    /* write high 3bit address if it support 10bit slave address */
    pI2Cx->C2 &= ~I2C_C2_AD_MASK;
    pI2Cx->C2 |= (uint8_t)(u16SlaveAddress>>8)&0x03;
}

/*****************************************************************************//*!
   *
   * @brief disable IICIF interrupt.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/
void I2C_IntDisable(I2C_Type* pI2Cx)
{
    pI2Cx->C1 &= ~I2C_C1_IICIE_MASK;
#if defined(MCU_SKEAZ1284)    
    if(pI2Cx == pI2C0)
    {
        Disable_Interrupt(I2C0_IRQn);
    }
    else if(pI2Cx == pI2C1)
    {
        Disable_Interrupt(I2C1_IRQn);
    }
#elif defined(MCU_SKEAZN642)
	Disable_Interrupt(I2C0_IRQn);
#else
/* Diable I2C interrupts for your device. */
#endif
   
}
/*****************************************************************************//*!
   *
   * @brief enable IICIF interrupt.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none.
*****************************************************************************/
void I2C_IntEnable(I2C_Type* pI2Cx)
{
    pI2Cx->C1 |= I2C_C1_IICIE_MASK;
#if defined(MCU_SKEAZ1284)    
    if(pI2Cx == pI2C0)
    {
        Enable_Interrupt(I2C0_IRQn);
    }
   
    else if(pI2Cx == pI2C1)
    {
        Enable_Interrupt(I2C1_IRQn);
    }
#elif defined(MCU_SKEAZN642)
	Enable_Interrupt(I2C0_IRQn);
#else
/* Enable I2C interrupts for your device. */
#endif
   
}

/*****************************************************************************//*!
   *
   * @brief SCL low timeout value that determines the timeout period of SCL low.
   *        
   * @param[in] pI2Cx      point to I2C module type.
   * @param[in] u16Timeout 
   *
   * @return none.
   *
   * @ Pass/ Fail criteria: none.
*****************************************************************************/
void I2C_SetSCLLowTimeout(I2C_Type* pI2Cx, uint16_t u16Timeout)
{
    pI2Cx->SLTL = (uint8_t)u16Timeout;
    pI2Cx->SLTH = (uint8_t)(u16Timeout>>8);
}
/*****************************************************************************//*!
   *
   * @brief deinit I2C module.
   *        
   * @param[in] pI2Cx    point to I2C module type.
   *
   * @return none
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/
void I2C_Deinit(I2C_Type* pI2Cx)
{
     pI2Cx->C1 &= ~I2C_C1_IICEN_MASK;
#if defined(MCU_SKEAZ1284)    
    if(pI2Cx == pI2C0)
    {
        SIM->SCGC &= ~SIM_SCGC_I2C0_MASK;
    }
    else
    {
        SIM->SCGC &= ~SIM_SCGC_I2C1_MASK;
    }
#elif defined(MCU_SKEAZN642)
	SIM->SCGC &= ~SIM_SCGC_IIC_MASK;
#else
/* Disable the clock for your device. */
#endif

}

/*****************************************************************************//*!
   *
   * @brief write a byte to I2C module.
   *        
   * @param[in] pI2Cx    point to I2C module type.
   * @param[in] u8WrBuff  data buffer for writing. 
   *
   * @return error status
   *
   * @ Pass/ Fail criteria: none
*****************************************************************************/

uint8_t I2C_WriteOneByte(I2C_Type* pI2Cx, uint8_t u8WrBuff)
{
    uint32_t u32Timeout;
    uint8_t u8ErrorStatus;
    
    u32Timeout = 0; 
    u8ErrorStatus = 0x00;
    while (((I2C_GetStatus(pI2Cx)&I2C_S_TCF_MASK) !=  I2C_S_TCF_MASK)
            && (u32Timeout<I2C_WAIT_STATUS_TIMEOUT))
    {
        u32Timeout ++;
    }
    if (u32Timeout >= I2C_WAIT_STATUS_TIMEOUT)
    {
        u8ErrorStatus |= I2C_ERROR_NO_WAIT_TCF_FLAG; 
        return u8ErrorStatus;
    }
    
    I2C_TxEnable(pI2Cx);
    I2C_WriteDataReg(pI2Cx,u8WrBuff);

    u32Timeout = 0;
    while (((I2C_GetStatus(pI2Cx)&I2C_S_IICIF_MASK) !=  I2C_S_IICIF_MASK)
            && (u32Timeout<I2C_WAIT_STATUS_TIMEOUT))
    {
        u32Timeout ++;
    }
    if (u32Timeout >= I2C_WAIT_STATUS_TIMEOUT)
    {
        u8ErrorStatus |= I2C_ERROR_NO_WAIT_IICIF_FLAG; 
        return u8ErrorStatus;
    }

    /* clear IICIF flag */
    I2C_ClearStatus(pI2Cx,I2C_S_IICIF_MASK);
    if (I2C_GetStatus(pI2Cx) & I2C_S_RXAK_MASK) 
    {
        u8ErrorStatus |= I2C_ERROR_NO_GET_ACK;
    }
    return u8ErrorStatus;
}
/*****************************************************************************//*!
   *
   * @brief read a byte from slave I2C.
   *        
   * @param[in] pI2Cx    point to I2C module type.
   * @param[out] pRdBuff point to the data read from slave I2C.
   * @param[out] u8Ack   send out ack or nack.
   *
   * @return error status
   *
   * @ Pass/ Fail criteria:  none
*****************************************************************************/

uint8_t I2C_ReadOneByte(I2C_Type* pI2Cx, uint8_t *pRdBuff, uint8_t u8Ack)
{
    uint32_t u32Timeout;
    uint8_t u8ErrorStatus;
    
    u32Timeout = 0; 
    u8ErrorStatus = 0x00;
    while (((I2C_GetStatus(pI2Cx)&I2C_S_TCF_MASK) !=  I2C_S_TCF_MASK)
            && (u32Timeout<I2C_WAIT_STATUS_TIMEOUT))
    {
        u32Timeout ++;
    }
    if (u32Timeout >= I2C_WAIT_STATUS_TIMEOUT)
    {
        u8ErrorStatus |= I2C_ERROR_NO_WAIT_TCF_FLAG; 
        return u8ErrorStatus;
    }
    
    I2C_RxEnable(pI2Cx);

    if( u8Ack )
    {
        /* send out nack */
        I2C_SendNack(pI2Cx);
       
    }
    else
    {
         /* send out ack */
        I2C_SendAck(pI2Cx);
    }
    *pRdBuff = I2C_ReadDataReg(pI2Cx);

    u32Timeout = 0;
    while (((I2C_GetStatus(pI2Cx)&I2C_S_IICIF_MASK) !=  I2C_S_IICIF_MASK)
            && (u32Timeout<I2C_WAIT_STATUS_TIMEOUT))
    {
        u32Timeout ++;
    }
    if (u32Timeout >= I2C_WAIT_STATUS_TIMEOUT)
    {
        u8ErrorStatus |= I2C_ERROR_NO_WAIT_IICIF_FLAG; 
        return u8ErrorStatus;
    }

    /* clear IICIF flag */
    I2C_ClearStatus(pI2Cx,I2C_S_IICIF_MASK);

    return u8ErrorStatus;
}

/*****************************************************************************//*!
   *
   * @brief I2C0 master send data with interrupt. 
   * 
   * @param[in]  pI2Cx    point to I2C module type.       
   * @param[in] u16Address I2C slave address. 
   *
   * @return error status
   *
   * @ Pass/ Fail criteria:  read data
*****************************************************************************/
uint8_t I2C_MasterSend(I2C_Type* pI2Cx, uint16_t Address )
{
	 uint8_t u8Status;

	    if( I2C_IsBusy(pI2Cx) )
	    {
	        return I2C_ERROR_BUS_BUSY;
	    }

	    /* send out start on bus */
	    u8Status = I2C_Start(pI2Cx);
	    I2C_TxEnable(pI2Cx);
	    if( u8Status == I2C_ERROR_NULL )
	    {
	        u8Status = I2C_WriteOneByte(pI2Cx,Address|I2C_WRITE);
	
	    }
	    return u8Status;

}


/*****************************************************************************//*!
   *
   * @brief send data to I2C, and wait to complete transfering.
   *   
   * @param[in]  pI2Cx    point to I2C module type.
   * @param[in]  u16SlaveAddress slave address. 
   * @param[in]  pWrBuff point the first address of transferring data buffer. 
   * @param[in]  u32Length the length of transferring data.
   *
   * @return error status
   *
   * @ Pass/ Fail criteria:  none
*****************************************************************************/

uint8_t I2C_MasterSendWait(I2C_Type* pI2Cx,uint16_t u16SlaveAddress,uint8_t *pWrBuff,uint32_t u32Length)
{
	 uint32_t i;
	 uint8_t u8ErrorStatus;
	 

	/* send start signals to bus */
	u8ErrorStatus = I2C_Start(pI2Cx);

	/* send device address to slave */
	u8ErrorStatus = I2C_WriteOneByte(pI2Cx,u16SlaveAddress| I2C_WRITE);

	/* if no error occur, received the correct ack from slave
			continue to send data to slave
		*/
	if( u8ErrorStatus == I2C_ERROR_NULL )
	{
		for(i=0;i<u32Length;i++)
		{
			u8ErrorStatus = I2C_WriteOneByte(pI2Cx,pWrBuff[i]);
			if( u8ErrorStatus != I2C_ERROR_NULL )
			{
				return u8ErrorStatus;
			}
		}
	 }

	 /* send stop signals to bus */
	 u8ErrorStatus = I2C_Stop(pI2Cx);

	 return u8ErrorStatus;
            
}



/*****************************************************************************//*!
   *
   * @brief I2C master receive data with interrupt. 
   * 
   * @param[in]  pI2Cx    point to I2C module type.       
   * @param[in] u16Address I2C slave address. 
   *
   * @return error status.
   *
   * @ Pass/ Fail criteria:  none.
*****************************************************************************/
uint8_t I2C_MasterRead(I2C_Type* pI2Cx, uint16_t u16Address)
{
    
    uint8_t u8Status;
    
  I2C_IntDisable(pI2Cx);
   
   /* send out start on bus */
   u8Status = I2C_Start(pI2Cx);

   if( u8Status == I2C_ERROR_NULL )
   {
       u8Status = I2C_WriteOneByte(pI2Cx,u16Address|I2C_READ);
       if( u8Status == I2C_ERROR_NULL )
       {
           
            /* change to Rx mode */
            I2C_SendAck(pI2Cx);
            I2C_RxEnable(pI2Cx);
            I2C_ReadDataReg(pI2Cx);
       }
   }

  I2C_IntEnable(pI2Cx);
   
   return u8Status;

}
/*****************************************************************************//*!
   *
   * @brief read data from I2C,and wait to complete transferring.
   *        
   * @param[in] pI2Cx    point to I2C module type.
   * @param[in]  u16SlaveAddress slave address. 
   * @param[in]  pRdBuff point the first address of reading data buffer. 
   * @param[in]  the length of transferring data.
   *
   * @return error status
   *
   * @ Pass/ Fail criteria:  none
*****************************************************************************/

uint8_t I2C_MasterReadWait(I2C_Type* pI2Cx,uint16_t u16SlaveAddress,uint8_t *pRdBuff,uint32_t u32Length)
{
	 uint32_t i;
	    uint8_t u8ErrorStatus;

	    /* send start signals to bus */
	    u8ErrorStatus = I2C_Start(pI2Cx);

	    /* send device address to slave */
	    u8ErrorStatus = I2C_WriteOneByte(pI2Cx,u16SlaveAddress | I2C_READ);

	    /* if no error occur, received the correct ack from slave
	            continue to send data to slave
	        */
	    /* dummy read one byte to switch to Rx mode */
	    I2C_ReadOneByte(pI2Cx,&pRdBuff[0],I2C_SEND_ACK);
	    
	    if( u8ErrorStatus == I2C_ERROR_NULL )
	    {
	        for(i=0;i<u32Length-1;i++)
	        {
	            u8ErrorStatus = I2C_ReadOneByte(pI2Cx,&pRdBuff[i],I2C_SEND_ACK);
	            if( u8ErrorStatus != I2C_ERROR_NULL )
	            {
	                return u8ErrorStatus;
	            }
	        }
	        u8ErrorStatus = I2C_ReadOneByte(pI2Cx,&pRdBuff[i],I2C_SEND_NACK);
	     }
	     /* send stop signals to bus */
	     u8ErrorStatus = I2C_Stop(pI2Cx);
	     
	     return u8ErrorStatus;
}
#if defined(MCU_SKEAZ1284)
/*****************************************************************************//*!
   *
   * @brief set call back function for I2C1 module.
   *        
   * @param[in] pCallBack point to address of I2C1 call back function.
   *
   * @return none.
   *
   * @ Pass/ Fail criteria:  none.
*****************************************************************************/

void I2C1_SetCallBack( I2C_CallbackType pCallBack )
{
    I2C_Callback[1] = pCallBack;
}
#elif defined(MCU_SKEAZN642)
/* KEAZN642 supports only one instance of I2C. */
#else
/* If your device has two or more instances of I2C, be sure to add them to this file.*/
#endif

/*****************************************************************************//*!
   *
   * @brief set call back function for I2C0 module.
   *        
   * @param[in] pCallBack point to address of I2C0 call back function.
   *
   * @return none.
   *
   * @ Pass/ Fail criteria:  none.
*****************************************************************************/

void I2C0_SetCallBack( I2C_CallbackType pCallBack )
{
    I2C_Callback[0] = pCallBack;
}
/*! @} End of i2c_api_list                                               						*/


/*****************************************************************************//*!
   *
   * @brief I2C0 interrupt service routine.
   *        
   * @param  
   *
   * @return none
   *
   * @ Pass/ Fail criteria:  none
*****************************************************************************/
void I2C0_IRQHandler( void )
{
    if( I2C_Callback[0] )
    {
        I2C_Callback[0]();
    }
}
#if defined(MCU_SKEAZ1284)
/*****************************************************************************//*!
   *
   * @brief I2C1 interrupt service routine.
   *        
   * @param  
   *
   * @return none
   *
   * @ Pass/ Fail criteria:  none
*****************************************************************************/
void I2C1_IRQHandler( void )
{
    if( I2C_Callback[1] )
    {
        I2C_Callback[1]();
    }
}
#elif defined(MCU_SKEAZN642)
/* KEAZN64 only supports one instance of I2C. */
#else
/* Be sure to add a interrupt handler for each instance of I2C that your device supports. */
#endif


