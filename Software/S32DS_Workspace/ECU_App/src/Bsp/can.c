/*
 * can.c
 *
 *  Created on: 2023年3月2日
 *      Author: mzy2364
 */

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "can.h"
#include "systick.h"
/*==================================================================================================
*                              SOURCE FILE VERSION INFORMATION
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
#define CAN_TFLG_TME0  (0x01)
#define CAN_TFLG_TME1  (0x02)
#define CAN_TFLG_TME2  (0x04)


/* CAN moudle enable / disable */
#define CAN_Enable(pCANx)			(pCANx->CANCTL1 |= MSCAN_CANCTL1_CANE_MASK)
#define CAN_Disable(pCANx)			(pCANx->CANCTL1 &= ~MSCAN_CANCTL1_CANE_MASK)


/* WakeUp Interrupt */
#define CAN_WakeUpIntEn(pCANx)  	(pCANx->CANRIER |= MSCAN_CANRIER_WUPIE_MASK)
#define CAN_WakeUpIntDisable(pCANx) (pCANx->CANRIER &= ~MSCAN_CANRIER_WUPIE_MASK)

/* CAN Status Change Interrupt */
#define CAN_StatusChangeIntEn(pCANx) 		(pCANx->CANRIER |= MSCAN_CANRIER_CSCIE_MASK)
#define CAN_StatusChangeIntDisable(pCANx) 	(pCANx->CANRIER &= ~MSCAN_CANRIER_CSCIE_MASK)

/* Receiver Status Change */
#define CAN_ReceiverStatusChangeAllIntEn(pCANx) 	\
    							(pCANx->CANRIER |= MSCAN_CANRIER_RSTATE_MASK)
#define CAN_ReceiverStatusChangeIntDisable(pCANx) \
    							(pCANx->CANRIER &= ~MSCAN_CANRIER_RSTATE_MASK)

/* Transmitter Status Change Enable */
#define CAN_TransmitterStatusChangeAllIntEn(pCANx) \
   								(pCANx->CANRIER |= MSCAN_CANRIER_TSTATE_MASK)

#define CAN_TransmitterStatusChangeIntDisable(pCANx) \
    							(pCANx->CANRIER &= ~MSCAN_CANRIER_TSTATE_MASK)

/* Overrun Interrupt Enable */
#define CAN_OverrunIntEn(pCANx)		(pCANx->CANRIER |= MSCAN_CANRIER_OVRIE_MASK)
#define CAN_OverrunIntDisable(pCANx) \
    							(pCANx->CANRIER &= ~MSCAN_CANRIER_OVRIE_MASK)

/* Receiver Full Interrupt Enable */
#define CAN_ReceiverFullIntEn(pCANx) \
    							(pCANx->CANRIER |= MSCAN_CANRIER_RXFIE_MASK)
#define CAN_ReceiverFullIntDisable(pCANx) \
								(pCANx->CANRIER &= ~MSCAN_CANRIER_RXFIE_MASK)

/* Transmitter Empty Interrupt Enable */
#define CAN_TransmitterEmptyIntEn(pCANx) \
    							(pCANx->CANTIER |= MSCAN_CANTIER_TXEIE_MASK)

#define CAN_TransmitterEmptyIntDisable(pCANx)	\
    							(pCANx->CANTIER &= ~MSCAN_CANTIER_TXEIE_MASK)


#define CAN_TransBuff0EmptyIntEn(pCANx) \
    							(pCANx->CANTIER |= CAN_TFLG_TME0)

#define CAN_TransBuff0EmptyIntDisable(pCANx)	\
    							(pCANx->CANTIER &= ~CAN_TFLG_TME0)

#define CAN_TransBuff1EmptyIntEn(pCANx) \
    							(pCANx->CANTIER |= CAN_TFLG_TME1)

#define CAN_TransBuff1EmptyIntDisable(pCANx)	\
    							(pCANx->CANTIER &= ~CAN_TFLG_TME1)

#define CAN_TransBuff2EmptyIntEn(pCANx) \
    							(pCANx->CANTIER |= CAN_TFLG_TME2)

#define CAN_TransBuff2EmptyIntDisable(pCANx)	\
    							(pCANx->CANTIER &= ~CAN_TFLG_TME2)


/* Clear Receive Buffer Full Flag */
#define CAN_ClearRXF_Flag(pCANx) 	(pCANx->CANRFLG |= MSCAN_CANRFLG_RXF_MASK)

/* Clear Received Frame Flag */
#define CAN_ClearRXFRM_Flag(pCANx)	(pCANx->CANCTL0 |= MSCAN_CANCTL0_RXFRM_MASK)

/* Clear Wake-Up Interrupt Flag */
#define CAN_ClearWUPIF_Flag(pCANx) (pCANx->CANRFLG |= MSCAN_CANRFLG_WUPIF_MASK)

/* Clear CAN Status Change Interrupt Flag */
#define CAN_ClearCSCIF_Flag(pCANx) (pCANx->CANRFLG |= MSCAN_CANRFLG_CSCIF_MASK)

/* Clear Overrun Interrupt Flag */
#define CAN_ClearOVRIF_Flag(pCANx) (pCANx->CANRFLG |= MSCAN_CANRFLG_OVRIF_MASK)

/* Clear Sleep Mode Request */
#define CAN_ClearSLPRQ_Flag(pCANx) (pCANx->CANCTL0 &= ~MSCAN_CANCTL0_SLPRQ_MASK)

/* Send complete interrupt clear */
#define CAN_ClearTXF_Flag(pCANx) 	(pCANx->CANTFLG |= MSCAN_CANTFLG_TXE_MASK)



/* Sleep Mode Request. */
#define CAN_SleepModeReq(pCANx)		\
								(pCANx->CANCTL0 |= MSCAN_CANCTL0_SLPRQ_MASK)

/* Initialization Mode Request.*/
#define CAN_InitialModeReq(pCANx)	\
    							(pCANx->CANCTL0 |= MSCAN_CANCTL0_INITRQ_MASK)

/* Normal Mode Request.*/
#define CAN_NormalModeReq(pCANx)	\
								(pCANx->CANCTL0 &= ~MSCAN_CANCTL0_INITRQ_MASK)



/* Bus-Off Recovery Mode - user request. */
#define CAN_BusOffUserRecovery(pCANx)	\
    							(pCANx->CANCTL1 |= MSCAN_CANCTL1_BORM_MASK)

/* Bus-Off Recovery Mode - Auto. */
#define CAN_BusOffAutoRecovery(pCANx) 	\
								(pCANx->CANCTL1 &= ~MSCAN_CANCTL1_BORM_MASK)

/* Check Sleep Mode Acknowledge. */
#define CAN_IsSleepMode(pCANx) 	\
								(pCANx->CANCTL1 & MSCAN_CANCTL1_SLPAK_MASK)

/* Check Initialization Mode Acknowledge. */
#define CAN_IsInitialMode(pCANx) \
								(pCANx->CANCTL1 & MSCAN_CANCTL1_INITAK_MASK)

/* Check Wake-Up Interrupt Flag. */
#define CAN_IsWakeUpIntFlag(pCANx)	\
								(pCANx->CANRFLG& MSCAN_CANRFLG_WUPIF_MASK)

/* Check CAN Status Change Interrupt Flag. */
#define CAN_IsStatusChangeFlag(pCANx)\
								(pCANx->CANRFLG& MSCAN_CANRFLG_CSCIF_MASK)

/* Check Overrun Interrupt Flag */
#define CAN_IsOverRunFlag(pCANx)   \
								(pCANx->CANRFLG& MSCAN_CANRFLG_OVRIF_MASK)

/* Check Receive Buffer Full Flag */
#define CAN_IsRxBuffFull(pCANx) 	\
								(pCANx->CANRFLG & MSCAN_CANRFLG_RXF_MASK)

/* Get Receiver Status */
#define CAN_GetReceiverStatus(pCANx) \
								(pCANx->CANRFLG & MSCAN_CANRFLG_RSTAT_MASK)

/* Get Transmitter Empty Interrupt Enable buffer*/
#define CAN_GetTransIntEnBuff(pCANx)	\
								(pCANx->CANTIER & MSCAN_CANTIER_TXEIE_MASK)

/* Get Transmitter Status */
#define CAN_GetTransmitterStatus(pCANx)	\
								(pCANx->CANRFLG & MSCAN_CANRFLG_TSTAT_MASK)

/* Get Transmitter Buffer Empty Flag */
#define CAN_GetTransmitterBufferEmptyFlag(pCANx) 	\
								(pCANx->CANTFLG & MSCAN_CANTFLG_TXE_MASK)


/* CAN Stops in Wait Mode Enable */
#define CAN_StopInWaitModeEn(pCANx) \
    							(pCANx->CANCTL0 |= MSCAN_CANCTL0_CSWAI_MASK)

/* CAN Stops in Wait Mode Disable */
#define CAN_StopInWaitModeDisable(pCANx) \
								(pCANx->CANCTL0 &= ~MSCAN_CANCTL0_CSWAI_MASK)

/* WakeUp Enable */
#define CAN_WakeUpEn(pCANx)		(pCANx->CANCTL0 |= MSCAN_CANCTL0_WUPE_MASK)

/* WakeUp Disable */
#define CAN_WakeUpDisable(pCANx) \
								(pCANx->CANCTL0 &= ~MSCAN_CANCTL0_WUPE_MASK)

/* Read receiver stamps registers high bytes. */
#define CAN_ReadRTSRHReg(pCANx)		(pCANx->RTSRH)

/* Read receiver stamps registers low bytes.*/
#define CAN_ReadRTSRLReg(pCANx) 	(pCANx->RTSRL)

/* Read transmitter stamps registers high byte. */
#define CAN_ReadTTSRHReg(pCANx)		(pCANx->TTSRH)

/* Read transmitter stamps registers low byte. */
#define CAN_ReadTTSRLReg(pCANx)		(pCANx->TTSRL)


/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
/**
* @brief          
* @details        
*/

/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/

static can_time_segment_t default_bitrate[CAN_BAUDRATE_NUM]={
		{.BRP = 8, .SEG1 = 5, .SEG2 = 2, .SJW = 1},	//125K
		{.BRP = 4, .SEG1 = 5, .SEG2 = 2, .SJW = 1},	//250K
		{.BRP = 2, .SEG1 = 5, .SEG2 = 2, .SJW = 1},	//500K
		{.BRP = 1, .SEG1 = 5, .SEG2 = 2, .SJW = 1},		//1000K
		{.BRP = 2, .SEG1 = 5, .SEG2 = 2, .SJW = 1},	//custom
};
/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
static filter_group_t can0_filter_group[2] = {{0X00000000,0XFFFFFFFF},{0X00000000,0XFFFFFFFF}};

static volatile can_config_t can_cfg[CAN_COM_NUM] = {
		{.can_rx_cbk_func = NULL,
		.filter_group_num = 2,
		.is_init = 0,
		.filter_group_ptr = &can0_filter_group[0],
		}
};

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
static void can_gpio_init(Can_CntrlType controller);
static void can_gpio_deinit(Can_CntrlType controller);
static void _can_init(Can_CntrlType controller,can_baudrate_t baudrate);
static void _can_deinit(Can_CntrlType controller);
static bool can_is_transmit_empt(Can_CntrlType controller);
static void can_busoff_func(Can_CntrlType controller);

/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

/**
* @brief CAN初始化
* @param cancontrollerN端口
* @param baudrate-波特率
* @return
* @note
*/
void can_init(Can_CntrlType controller,can_baudrate_t baudrate)
{
	if(controller < CAN_COM_NUM)
	{
		if (false == ramf_init((struct _ramf_ctl*)&can_cfg[controller].ram_fifo_rx, \
				(uint8_t*)can_cfg[controller].ram_fifo_rx_buffer, CAN_MSG_SIZE, CAN_RX_MSG_COUNT))
		{
			while (1);
		}
		if (false == ramf_init((struct _ramf_ctl*)&can_cfg[controller].ram_fifo_tx, \
				(uint8_t*)can_cfg[controller].ram_fifo_tx_buffer, CAN_MSG_SIZE, CAN_TX_MSG_COUNT))
		{
			while (1);
		}

		can_gpio_init(controller);
		_can_init(controller,baudrate);
		can_cfg[controller].is_init = 1;
	}
}

/**
* @brief CAN反初始化
* @param controller-CAN端口
* @return
* @note
*/
void can_deinit(Can_CntrlType controller)
{
	if(controller < CAN_COM_NUM)
	{
		can_gpio_deinit(controller);
		_can_deinit(controller);
	}
}

/**
* @brief CAN滤波配置
* @param controller-CAN端口
* @param group_idx-滤波组ID号
* @param acr-验收码
* @param msk-屏蔽码
* @return
* @note 必须在CAN初始化前面配置滤波参数
*/
void can_filter_config(Can_CntrlType controller,uint8_t group_idx,uint32_t acr,uint32_t msk)
{
	if(controller < CAN_COM_NUM)
	{
		if(group_idx < can_cfg[controller].filter_group_num)
		{
			(can_cfg[controller].filter_group_ptr+group_idx)->Acceptance_Code = acr;
			(can_cfg[controller].filter_group_ptr+group_idx)->Mask_Code = msk;
		}
	}
}


/**
* @brief CAN异步发送函数
* @param controller-CAN端口
* @param PduInfo-发送数据信息
* @return
* @note
*/
void can_write(Can_CntrlType controller, const Can_PduType* PduInfo)
{
	CAN_MssageType can_message;

	if(controller < CAN_COM_NUM)
	{
		if(can_cfg[controller].is_init)
		{
			if(PduInfo)
			{
				can_message.Hrh = controller;
				can_message.CanId = PduInfo->id;
				can_message.CanDlc = PduInfo->length;
				if(can_message.CanDlc > 8)
					can_message.CanDlc = 8;
				memcpy(&can_message.CanSdu[0],PduInfo->sdu, can_message.CanDlc);

				ramf_write((struct _ramf_ctl*)&can_cfg[controller].ram_fifo_tx, (uint8_t *)&can_message);
			}
		}
	}
}

/**
* @brief CAN设置接收回调函数
* @param controller-CAN端口
* @param func-回调函数
* @return
* @note
*/
void can_set_rx_cbk_func(Can_CntrlType controller,can_cbk_func func)
{
	if(controller < CAN_COM_NUM)
	{
		if(func != NULL)
			can_cfg[controller].can_rx_cbk_func = func;
	}
}

/**
* @brief CAN周期调度函数
* @param None
* @return None
* @note 应该在程序的空闲任务中调用该函数
*/
void can_main_function_write(void)
{
	uint8_t i = 0;
	CAN_MssageType can_message;
	for(i = 0; i < CAN_COM_NUM; i++)
	{
		if(can_cfg[i].is_init)
		{
			if(can_is_transmit_empt(i) == true)
			{
				if(true == ramf_read((struct _ramf_ctl*)&can_cfg[i].ram_fifo_tx, (uint8_t *)&can_message))
				{
					can_send(can_message.Hrh, can_message.CanId, can_message.CanDlc, can_message.CanSdu);
				}
			}
		}
	}
}

/**
* @brief CAN周期调度函数
* @param None
* @return None
* @note 应该在程序的空闲任务中调用该函数
*/
void can_main_function_read(void)
{
	uint8_t controller = 0;
	Can_PduType PduInfo;
	CAN_MssageType can_message;
	for(controller = 0; controller < CAN_COM_NUM; controller ++)
	{
		if(can_cfg[controller].is_init)
		{
			if(can_cfg[controller].can_rx_cbk_func == NULL)
			{
				ramf_read((struct _ramf_ctl*)&can_cfg[controller].ram_fifo_rx, (uint8_t *)&can_message);
				continue;
			}
			if(true == ramf_read((struct _ramf_ctl*)&can_cfg[controller].ram_fifo_rx, (uint8_t *)&can_message))
			{
				PduInfo.id = can_message.CanId;
				PduInfo.length = can_message.CanDlc;
				PduInfo.sdu = (uint8_t*)&can_message.CanSdu;
				can_cfg[controller].can_rx_cbk_func(can_message.Hrh,&PduInfo);
			}
		}
	}
}

/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/

/**
* @brief CAN GPIO初始化
* @param[in] controller-CAN端口号
* @return None
* @note
*/
static void can_gpio_init(Can_CntrlType controller)
{
	/* CAN_TX 在 PTC7 上 CAN_RX 在 PTC6 上 */
	SIM->PINSEL1 &= ~SIM_PINSEL1_MSCANPS_MASK;
	SIM->PINSEL1 &= ~SIM_PINSEL1_MSCANPS_MASK;
}

/**
* @brief CAN GPIO反初始化
* @param[in] controller-CAN端口号
* @return None
* @note
*/
static void can_gpio_deinit(Can_CntrlType controller)
{
	if(controller < CAN_COM_NUM)
	{

	}
}

/**
* @brief CAN 硬件外设初始化
* @param[in] controller-CAN端口号
* @param[in] baudrate-CAN波特率
* @return None
* @note
*/
static void _can_init(Can_CntrlType controller,can_baudrate_t baudrate)
{
    /* Bus clock to the MSCAN module is enabled. */
    SIM->SCGC |= SIM_SCGC_MSCAN_MASK;

	/* Initialization Mode Request(INITRQ in MSCAN_CANCTL0) */
	/* When this bit is set by the CPU, the MSCAN in to initialization mode.
	   Any ongoing transmission or reception is aborted and synchronization to
	   the CAN bus is lost. The module indicates entry to initialization mode
	   by setting INITAK = 1.
	*/
	MSCAN->CANCTL0 |= MSCAN_CANCTL0_INITRQ_MASK;

    /* Initialization Mode Acknowledge
	   This flag indicates whether the MSCAN module is in initialization mode.
	   It is used as a handshake flag for the INITRQ initialization mode request.
	   Initialization mode is active when INITRQ = 1 and INITAK = 1.
	   The registers CANCTL1, CANBTR0, CANBTR1, CANIDAC, CANIDAR0-CANIDAR7, and
	   CANIDMR0 - CANIDMR7 can be written only by the CPU when the MSCAN is in
	   initialization mode.
			0:Running - The MSCAN operates normally.
			1:Initialization mode active - The MSCAN has entered initialization
			mode
	*/
    while (!(MSCAN->CANCTL1 & MSCAN_CANCTL1_INITAK_MASK));

    /* CAN Stops in Wait Mode. (CSWAI in MSCAN_CANCTL0) */
    /* 1: Enable The module ceases to be clocked during wait mode. */
	MSCAN->CANCTL0 &= ~MSCAN_CANCTL0_CSWAI_MASK;

	/* An internal 16-bit wide free running timer (TIME in MSCAN_CANCTL0) */
	/* 1: Enable internal MSCAN timer for timer stamp */
	MSCAN->CANCTL0 |= MSCAN_CANCTL0_TIME_MASK;

	/* WakeUp Enable (WUPE in MSCAN_CANCTL0).
	 This configuration bit allows the MSCAN to restart from sleep mode or
	 from power down mode (entered from sleep) when traffic on CAN is detected.
	 This bit must be configured before sleep mode entry for the selected
	 function to take effect.
	  -- 1: MWakeup enabled - The MSCAN is able to restart.
	*/
	MSCAN->CANCTL0 |= MSCAN_CANCTL0_WUPE_MASK;


    /* MSCAN Enable (CANE in MSCAN_CANCTL1) */
    MSCAN->CANCTL1 |= MSCAN_CANCTL1_CANE_MASK;

	/*  MSCAN Clock Source(CLKSRC in MSCAN_CANCTL1):
   	 		0:MSCAN clock source is the oscillator clock.
			1:MSCAN clock source is the bus clock.
	*/
	MSCAN->CANCTL1  &= ~MSCAN_CANCTL1_CLKSRC_MASK;

	/* Loopback Self Test Mode (LOOPB in MSCAN_CANCTL1)
		-- 0 : Loopback self test disabled.
		-- 1 : Loopback self test enabled.
	*/
	MSCAN->CANCTL1 &= ~MSCAN_CANCTL1_LOOPB_MASK;

	/* WakeUp Mode (WUPM in MSCAN_CANCTL1)
		If WUPE in CANCTL0 is enabled, this bit defines whether the integrated
		low-pass filter is applied to protect the MSCAN from spurious wakeup.
			-- 0:MSCAN wakes on any dominant level on the CAN bus.
			-- 1:MSCAN wakes only in case of a dominant pulse on the CAN bus
			that has a length of Twup.
	*/
	MSCAN->CANCTL1 |= MSCAN_CANCTL1_WUPM_MASK;

	/* Listen Only Mode.(LISTEN in MSCAN_CANCTL1)
		This bit configures the MSCAN as a CAN bus monitor. When LISTEN is set,
		all valid CAN messages with matching ID are received, but no
		acknowledgement or error frames are sent out. In addition, the error
		counters are frozen. Listen only mode supports applications which require
		 "hot plugging" or throughput analysis. The MSCAN is unable to transmit
		any messages when listen only mode is active.
		-- 0:Normal operation.
		-- 1:Listen only mode activated.
	 */
	MSCAN->CANCTL1 &= ~MSCAN_CANCTL1_LISTEN_MASK;


    /* Bus-Off Recovery Mode.(BORM in MSCAN_CANCTL1)
		This bit configures the bus-off state recovery mode of the MSCAN.
			-- 0:Automatic bus-off recovery .
			-- 1:Bus-off recovery upon user request.
	*/
	MSCAN->CANCTL1 &= ~MSCAN_CANCTL1_BORM_MASK;

    /* Baudrate setting
    Bit time = (1 + timesegment1 + timesegment2) * (Prescaler value)/ fCANCLK
	*/
    MSCAN->CANBTR0 = MSCAN_CANBTR0_SJW(default_bitrate[baudrate].SJW) | \
    				 MSCAN_CANBTR0_BRP(default_bitrate[baudrate].BRP-1);
    MSCAN->CANBTR1 = MSCAN_CANBTR1_TSEG1(default_bitrate[baudrate].SEG1-1)
    				|MSCAN_CANBTR1_TSEG2(default_bitrate[baudrate].SEG2-1);

   	if (1)
   	{
		MSCAN->CANBTR1 |= MSCAN_CANBTR1_SAMP_MASK;
   	}
   	else
   	{
		MSCAN->CANBTR1 &= ~MSCAN_CANBTR1_SAMP_MASK;
   	}

#if (1 == CAN_FILTER)
    /* CAN Identifier filter */

	/* MSCAN Identifier Acceptance Control Register (MSCAN_CANIDAC) */
	/* IDAM : Identifier Acceptance Mode
		The CPU sets these flags to define the identifier acceptance filter
		organization. In filter closed mode, no message is accepted such that
		the foreground buffer is never reloaded.
			-- 00:Two 32-bit acceptance filters.
			-- 01:Four 16-bit acceptance filters.
			-- 10:Eight 8-bit acceptance filters.
			-- 11:Filter closed
	*/
	MSCAN->CANIDAC = MSCAN_CANIDAC_IDAM(CAN_FILTER_MODE);

#if (CAN_FILTER_MODE == CAN_FILTER_TWO_32BIT)

	/*First group MSCAN_CANIDAR[1]*/
	MSCAN->CANIDAR_BANK_1[0] = (can_cfg[0].filter_group_ptr[0].Acceptance_Code >> 24) & 0XFF;
	MSCAN->CANIDAR_BANK_1[1] = (can_cfg[0].filter_group_ptr[0].Acceptance_Code >> 16) & 0XFF;
	MSCAN->CANIDAR_BANK_1[2] = (can_cfg[0].filter_group_ptr[0].Acceptance_Code >> 8) & 0XFF;
	MSCAN->CANIDAR_BANK_1[3] = (can_cfg[0].filter_group_ptr[0].Acceptance_Code >> 0) & 0XFF;
	/*First group MSCAN_CANIDMR[1]*/
	MSCAN->CANIDMR_BANK_1[0] = (can_cfg[0].filter_group_ptr[0].Mask_Code >> 24) & 0XFF;
	MSCAN->CANIDMR_BANK_1[1] = (can_cfg[0].filter_group_ptr[0].Mask_Code >> 16) & 0XFF;
	MSCAN->CANIDMR_BANK_1[2] = (can_cfg[0].filter_group_ptr[0].Mask_Code >> 8) & 0XFF;
	MSCAN->CANIDMR_BANK_1[3] = (can_cfg[0].filter_group_ptr[0].Mask_Code >> 0) & 0XFF;

	/*Second group MSCAN_CANIDAR[1]*/
	MSCAN->CANIDAR_BANK_2[0] = (can_cfg[0].filter_group_ptr[1].Acceptance_Code >> 24) & 0XFF;
	MSCAN->CANIDAR_BANK_2[1] = (can_cfg[0].filter_group_ptr[1].Acceptance_Code >> 16) & 0XFF;
	MSCAN->CANIDAR_BANK_2[2] = (can_cfg[0].filter_group_ptr[1].Acceptance_Code >> 8) & 0XFF;
	MSCAN->CANIDAR_BANK_2[3] = (can_cfg[0].filter_group_ptr[1].Acceptance_Code >> 0) & 0XFF;
	/*Second group MSCAN_CANIDMR[1]*/
	MSCAN->CANIDMR_BANK_2[0] = (can_cfg[0].filter_group_ptr[1].Mask_Code >> 24) & 0XFF;
	MSCAN->CANIDMR_BANK_2[1] = (can_cfg[0].filter_group_ptr[1].Mask_Code >> 16) & 0XFF;
	MSCAN->CANIDMR_BANK_2[2] = (can_cfg[0].filter_group_ptr[1].Mask_Code >> 8) & 0XFF;
	MSCAN->CANIDMR_BANK_2[3] = (can_cfg[0].filter_group_ptr[1].Mask_Code >> 0) & 0XFF;

#elif (CAN_FILTER_MODE == CAN_FILTER_FOUR_16BIT)
	uint32_t stdId = 0x7F0;
	uint8_t msgIDE = 0; // 0:Standard, 1:Externd;
	uint8_t msgRTR = 0; // 0:Data frame, 1:Remote frame;

	MSCAN->CANIDAR_BANK_1[0] = (uint8_t)(id>>3);
	MSCAN->CANIDAR_BANK_1[1] = (uint8_t)(((id<<5) & 0xE0) | \
									  (msgRTR<<4) | (msgIDE<<3));

    MSCAN->CANIDMR_BANK_1[0] = 0;
    MSCAN->CANIDMR_BANK_1[1] = 0xFF | (uint8_t)(~CAN_RIDR_RTR_MASK) | \
    								  (uint8_t)(~CAN_RIDR_IDE_MASK);

	stdId = 0x7F1;
	msgIDE = 0; // 0:Standard, 1:Externd;
	msgRTR = 0; // 0:Data frame, 1:Remote frame;

	MSCAN->CANIDAR_BANK_1[2] = (uint8_t)(id>>3);
	MSCAN->CANIDAR_BANK_1[3] = (uint8_t)(((id<<5) & 0xE0) | \
										  (msgRTR<<4) | (msgIDE<<3));

    MSCAN->CANIDMR_BANK_1[2] = 0;
    MSCAN->CANIDMR_BANK_1[3] = 0xFF | (~CAN_RIDR_RTR_MASK) | (~CAN_RIDR_IDE_MASK);


	stdId = 0x7F2;
	msgIDE = 0; // 0:Standard, 1:Externd;
	msgRTR = 0; // 0:Data frame, 1:Remote frame;

	MSCAN->CANIDAR_BANK_2[0] = (uint8_t)(id>>3);
	MSCAN->CANIDAR_BANK_2[1] = (uint8_t)(((id<<5) & 0xE0) | (msgRTR<<4) | (msgIDE<<3));

    MSCAN->CANIDMR_BANK_2[0] = 0;
    MSCAN->CANIDMR_BANK_2[1] = 0xFF | (~CAN_RIDR_RTR_MASK) | (~CAN_RIDR_IDE_MASK);


	stdId = 0x7F3;
	msgIDE = 0; // 0:Standard, 1:Externd;
	msgRTR = 0; // 0:Data frame, 1:Remote frame;

	MSCAN->CANIDAR_BANK_2[2] = (uint8_t)(id>>3);
	MSCAN->CANIDAR_BANK_2[3] = (uint8_t)(((id<<5) & 0xE0) | \
										  (msgRTR<<4) | (msgIDE<<3));

    MSCAN->CANIDMR_BANK_2[2] = 0;
    MSCAN->CANIDMR_BANK_2[3] = 0xFF | (~CAN_RIDR_RTR_MASK) | (~CAN_RIDR_IDE_MASK);
#else
	MSCAN->CANIDAR_BANK_1[0] = 0;
	MSCAN->CANIDAR_BANK_1[1] = 0;
	MSCAN->CANIDAR_BANK_1[2] = 0;
	MSCAN->CANIDAR_BANK_1[3] = 0;

	/* MSCAN Identifier Mask Register
		-- 0:Match corresponding acceptance code register and identifier bits.
		-- 1:Ignore corresponding acceptance code register bit*/
	MSCAN->CANIDMR_BANK_1[0] = 0xFF;
	MSCAN->CANIDMR_BANK_1[1] = 0xFF;
	MSCAN->CANIDMR_BANK_1[2] = 0xFF;
	MSCAN->CANIDMR_BANK_1[3] = 0xFF;

	MSCAN->CANIDAR_BANK_2[0] = 0;
	MSCAN->CANIDAR_BANK_2[1] = 0;
	MSCAN->CANIDAR_BANK_2[2] = 0;
	MSCAN->CANIDAR_BANK_2[3] = 0;

    MSCAN->CANIDMR_BANK_2[0] = 0xFF;
    MSCAN->CANIDMR_BANK_2[1] = 0xFF;
    MSCAN->CANIDMR_BANK_2[2] = 0xFF;
    MSCAN->CANIDMR_BANK_2[3] = 0xFF;

#endif
#endif

    /* Exit initialization mode. */
    MSCAN->CANCTL0 &= ~MSCAN_CANCTL0_INITRQ_MASK;

    /* wait to exit initialization mode. */
    while ((MSCAN->CANCTL1 & MSCAN_CANCTL1_INITAK_MASK));

	while(!(MSCAN->CANCTL0 & MSCAN_CANCTL0_SYNCH_MASK)); // Is Syn To Bus

	/* MSCAN Receiver Interrupt Enable Register */

	/* WakeUp Interrupt Enable (WUPIE in MSCAN_CANRIER)
		WUPIE and WUPE must both be enabled if the recovery mechanism from stop
		or wait is required.
	*/
	CAN_WakeUpIntDisable(MSCAN);

	/* CAN Status Change Interrupt Enable.(CSCIE in MSCAN_CANRIER) */
	CAN_StatusChangeIntDisable(MSCAN);

	/* Receiver Status Change Enable.(RSTATE in MSCAN_CANRIER)
		These RSTAT enable bits control the sensitivity level in which receiver
		state changes are causing CSCIF interrupts. Independent of the chosen
		sensitivity level the RSTAT flags continue to indicate the actual
		receiver state and are only updated if no CSCIF interrupt is pending.
			-- 00:Do not generate any CSCIF interrupt caused by receiver state
					changes.
			-- 01:Generate CSCIF interrupt only if the receiver enters or leaves
				 "bus-off" state. Discard other receiver state changes for
				 generating CSCIF interrupt.
			-- 10:Generate CSCIF interrupt only if the receiver enters or leaves
				 "RxErr" or "bus-off" state. Discard other receiver state
				 changes for generating CSCIF interrupt.
			-- 11:Generate CSCIF interrupt on all state changes.
  	*/
	CAN_ReceiverStatusChangeIntDisable(MSCAN);

	/* Transmitter Status Change Interrupt Enable.(TSTATE in MSCAN_CANRIER)
		These TSTAT enable bits control the sensitivity level in which
		transmitter state changes are causing CSCIF interrupts. Independent of
		the chosen sensitivity level, the TSTAT flags continue to indicate the
		actual transmitter state and are only updated if no CSCIF interrupt is
		pending.
			-- 00:Do not generate any CSCIF interrupt caused by transmitter
				state changes.
			-- 01:Generate CSCIF interrupt only if the transmitter enters or
				leaves "bus-off" state. Discard other transmitter state changes
				for generating CSCIF interrupt.
			-- 10:Generate CSCIF interrupt only if the transmitter enters or
				leaves "TxErr" or "bus-off" state. Discard other transmitter
				state changes for generating CSCIF interrupt.
			-- 11:Generate CSCIF interrupt on all state changes.
	*/
	CAN_TransmitterStatusChangeIntDisable(MSCAN);

	/* 	Overrun Interrupt Enable.(OVRIE in MSCAN_CANRIER)
		-- 0:No interrupt request is generated from this event.
		-- 1:An overrun event causes an error interrupt request.
	*/
    CAN_OverrunIntDisable(MSCAN);


    /* Receiver Full Interrupt Enable.(RXFIE in MSCAN_CANRIER)
		-- 0:No interrupt request is generated from this event.
		-- 1:A receive buffer full (successful message reception) event causes
		a receiver interrupt request.
	*/
    CAN_ReceiverFullIntEn(MSCAN);
    NVIC_EnableIRQ(MSCAN_RX_IRQn);// enable can rx interrupt
}

/**
* @brief CAN 硬件外设反初始化
* @param[in] controller-CAN端口号
* @return None
* @note
*/
static void _can_deinit(Can_CntrlType controller)
{
	if(controller < CAN_COM_NUM)
	{
		/* Initialization Mode Request(INITRQ in MSCAN_CANCTL0) */
		/* When this bit is set by the CPU, the MSCAN skips to initialization mode.
		   Any ongoing transmission or reception is aborted and synchronization to
		   the CAN bus is lost. The module indicates entry to initialization mode
		   by setting INITAK = 1.
		*/
		MSCAN->CANCTL0 |= MSCAN_CANCTL0_INITRQ_MASK;

	    /* Initialization Mode Acknowledge
		   This flag indicates whether the MSCAN module is in initialization mode.
		   It is used as a handshake flag for the INITRQ initialization mode request.
		   Initialization mode is active when INITRQ = 1 and INITAK = 1.
		   The registers CANCTL1, CANBTR0, CANBTR1, CANIDAC, CANIDAR0-CANIDAR7, and
		   CANIDMR0 - CANIDMR7 can be written only by the CPU when the MSCAN is in
		   initialization mode.
				0:Running - The MSCAN operates normally.
				1:Initialization mode active - The MSCAN has entered initialization
				mode
		*/
	    while (!(MSCAN->CANCTL1 & MSCAN_CANCTL1_INITAK_MASK));

		CAN_Disable(MSCAN);
		SIM->SCGC &= ~SIM_SCGC_MSCAN_MASK;	/* CLOCK */

		NVIC_DisableIRQ(MSCAN_RX_IRQn);// disable can rx interrupt
	}
}

/**
* @brief CAN 获取发送寄存器为空状态
* @param[in] controller-CAN端口号
* @return true-发送缓冲器为空,可以发送   false-发送缓冲器不为空
* @note
*/
static bool can_is_transmit_empt(Can_CntrlType controller)
{
	uint8_t TxEmpty = false;
	if(controller < CAN_COM_NUM)
	{
		TxEmpty = CAN_GetTransmitterBufferEmptyFlag(MSCAN);
	}
	return (TxEmpty);
}


/**
* @brief CAN 获取发送寄存器为空状态
* @param[in] controller-CAN端口号
* @param[in] id-CAN ID
* @param[in] length-数据长度
* @param[in] pData-数据
* @return 0-发送成功 other-发送失败
* @note
*/
uint8_t can_send(Can_CntrlType controller, Can_IdType id, uint8_t length, uint8_t* pData)
{
	uint8_t i, msgIDE, msgRTR, msgPrty;
	uint32_t transmitmailbox;

	/*Frame ID mode*/
	if (id & 0x18FFF000)msgIDE = CAN_ID_EXT;
	else msgIDE = CAN_ID_STD;

	msgRTR = CAN_RTR_DATA;

	/* detect bus Idle status.*/
	if(!(MSCAN->CANCTL0 & MSCAN_CANCTL0_SYNCH_MASK))
	{
		return 1;
	}

	/* Find free mailbox number */
    MSCAN->CANTBSEL = MSCAN->CANTFLG;
    transmitmailbox = MSCAN->CANTBSEL;

    /* ID fill */
	if (msgIDE == CAN_ID_EXT)
	{
		MSCAN->TEIDR0 = (uint8_t)(id>>21);
		MSCAN->TEIDR1 = (uint8_t)(((id<<3)>>16) & 0xE0) | (msgRTR<<4) | (msgIDE<<3);
		MSCAN->TEIDR1 |= (uint8_t)(((id<<1)>>16) & 0x07);
		MSCAN->TEIDR2 =  (uint8_t)((id<<1)>>8);
		MSCAN->TEIDR3 =  (uint8_t)(id<<1);
	}else
	{
		MSCAN->TSIDR0 = (uint8_t)(id>>3);
		MSCAN->TSIDR1 = (uint8_t)(((id<<5) & 0xE0) | (msgRTR<<4) | (msgIDE<<3));
	}

	/*data fill*/
	for (i = 0; i < 8; i++)
	{
		*((&(MSCAN->TEDSR[0]))+i) = pData[i];
	}

	MSCAN->TDLR = length;

	msgPrty = 0;
	MSCAN->TBPR = msgPrty;


    /* Start Transmit */

    /* The CPU must clear the Transmitter Flag flag(TXE in register
        MSCAN_CANTFLG) after a message is set up in the transmit buffer and is
        due for transmission.

        The MSCAN sets the flag after the message is sent successfully.
    */
    MSCAN->CANTFLG = transmitmailbox;

	return 0;
}


void MSCAN_RX_IRQHandler(void)
{
	CAN_MssageType can_recv_msg;
	struct _ramf_ctl *ramf_rx = (struct _ramf_ctl*)&can_cfg[CAN_COM0].ram_fifo_rx;
	/*Determine the source of MSCAN receive interrupt*/
	if(MSCAN->CANRFLG & MSCAN_CANRFLG_RXF_MASK)
	{
		can_recv_msg.Hrh = 0;
		if((MSCAN->REIDR1 & MSCAN_RSIDR1_RSRTR_MASK) == 0)
		{
			if((MSCAN->REIDR1 & MSCAN_RSIDR1_RSIDE_MASK))	//EXT ID
			{
				can_recv_msg.CanId = (uint32_t)(MSCAN->REIDR0 << 21)
		    	    					| (uint32_t)((MSCAN->REIDR1 & MSCAN_REIDR1_REID20_REID18_MASK) << (18-MSCAN_REIDR1_REID20_REID18_SHIFT))
		    							| (uint32_t)((MSCAN->REIDR1 & 0x07) << 15)
		    							| (uint32_t)(MSCAN->REIDR2 << 7)
		    							| (uint32_t)((MSCAN->REIDR3 & 0xFE) >> 1);
				can_recv_msg.CanDlc = MSCAN->RDLR & 0x0F;
				memcpy(can_recv_msg.CanSdu, (const uint8_t *)MSCAN->REDSR, can_recv_msg.CanDlc);

				ramf_write(ramf_rx, (uint8_t *)&can_recv_msg);
			}
			else	//STD ID
			{
				can_recv_msg.CanId = (uint32_t)(MSCAN->RSIDR0 << 3) | (uint32_t)(MSCAN->RSIDR1 >> 5);
				can_recv_msg.CanDlc = MSCAN->RDLR & 0x0F;
				memcpy(can_recv_msg.CanSdu, (const uint8_t *)MSCAN->REDSR, can_recv_msg.CanDlc);

				ramf_write(ramf_rx, (uint8_t *)&can_recv_msg);
			}
		}

		CAN_ClearRXF_Flag(MSCAN);	/*Clear Receive Buffer Full Flag*/
	}
}


/************************************************EOF************************************************/
