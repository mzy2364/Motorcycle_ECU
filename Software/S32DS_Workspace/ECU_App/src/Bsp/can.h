/*
 * can.h
 *
 *  Created on: 2023年3月2日
 *      Author: mzy2364
 */

#ifndef CAN_H_
#define CAN_H_

#ifdef __cplusplus
extern "C"{
#endif

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "derivative.h"
#include "ram_fifo.h"

/*==================================================================================================
*                              SOURCE FILE VERSION INFORMATION
==================================================================================================*/


/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

#define CAN_MSG_SIZE            (sizeof(CAN_MssageType))
#define CAN_RX_MSG_COUNT        (8)
#define CAN_TX_MSG_COUNT		(8)

/* CAN register configure */
#define CAN_ID_STD             		0x0U  /*!< Standard Id */
#define CAN_ID_EXT             		0x1U  /*!< Extended Id */

/* @defgroup CAN_remote_transmission_request CAN Remote Transmission Request */
#define CAN_RTR_DATA                0x0U  /*!< Data frame */
#define CAN_RTR_REMOTE              0x1U  /*!< Remote frame */

#define CAN_FILTER			1

#define CAN_FILTER_TWO_32BIT		0x00U
#define CAN_FILTER_FOUR_16BIT 		0x01U
#define CAN_FILTER_EIGHT_8BIT 		0x02U
#define CAN_FILTER_CLOSE 			0x03U

#define CAN_FILTER_MODE 			CAN_FILTER_TWO_32BIT

/*==================================================================================================
*                                             ENUMS
==================================================================================================*/
/**
* @brief          
* @details        
*/

typedef enum
{
	STD_Frame,
	EXT_Frame
}frame_type_t;

typedef struct
{
	uint32_t Acceptance_Code;
	uint32_t Mask_Code;
}filter_group_t;

typedef enum{
	CAN_COM0 = 0,
	CAN_COM_NUM,
}can_com_t;

/**
* @brief          CAN发送函数返回值
* @details
*/
typedef enum
{
    CAN_ERROR_NOERROR = 0,			 //!<return no error
    CAN_ERROR_BUFFERFULL = 1,
} can_error_t;


/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
/**
* @brief 波特率
*/
typedef enum
{
	CAN_BAUDRATE_125K = 0,
    CAN_BAUDRATE_250K,
    CAN_BAUDRATE_500K,
    CAN_BAUDRATE_1M,
    CAN_BAUDRATE_CUSTOM,
    CAN_BAUDRATE_NUM,
}can_baudrate_t;

enum
{
	CAN_BUS_125K = CAN_BAUDRATE_125K,
	CAN_BUS_250K = CAN_BAUDRATE_250K,
	CAN_BUS_500K = CAN_BAUDRATE_500K,
	CAN_BUS_1000K = CAN_BAUDRATE_1M,
	CAN_BUS_CUSTOM = CAN_BAUDRATE_CUSTOM,
};

typedef struct {
	uint8_t BRP;
	uint8_t SEG1;
	uint8_t SEG2;
	uint8_t SJW;
} can_time_segment_t;

typedef uint32_t Can_IdType;

typedef uint8_t Can_CntrlType;

typedef uint8_t Can_DlcType;

typedef uint8_t Can_HwHandleType;


typedef struct
{
	Can_DlcType length;
	Can_IdType   id;
	uint8_t*     sdu;
}Can_PduType;

typedef struct
{
	uint8_t 	Hrh;     				/*!< CAN message peripheral channel. */
	uint32_t 	CanId;					/*!< Message ID */
	uint8_t 	CanDlc;					/*!< Data length. */
	uint8_t 	CanSdu[8];				/*!< Data content. */
}CAN_MssageType;

typedef  uint8_t (*can_cbk_func)(Can_CntrlType controller, const Can_PduType* PduInfo);

typedef struct{
	can_cbk_func can_rx_cbk_func;
	struct _ramf_ctl ram_fifo_rx;
	struct _ramf_ctl ram_fifo_tx;
	uint8_t ram_fifo_rx_buffer[CAN_MSG_SIZE * CAN_RX_MSG_COUNT];
	uint8_t ram_fifo_tx_buffer[CAN_MSG_SIZE * CAN_TX_MSG_COUNT];
	uint8_t filter_group_num;
	filter_group_t *filter_group_ptr;
	uint8_t is_init;
}can_config_t;


/*==================================================================================================
*                                GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
extern void can_init(Can_CntrlType controller,can_baudrate_t baudrate);
extern void can_deinit(Can_CntrlType controller);
extern void can_filter_config(Can_CntrlType controller,uint8_t group_idx,uint32_t acr,uint32_t msk);
extern void can_set_rx_cbk_func(Can_CntrlType controller,can_cbk_func func);
extern void can_write(Can_CntrlType controller, const Can_PduType* PduInfo);
extern uint8_t can_send(Can_CntrlType controller, Can_IdType id, uint8_t length, uint8_t* pData);

extern void can_main_function_read(void);
extern void can_main_function_write(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H_ */

/************************************************EOF************************************************/
