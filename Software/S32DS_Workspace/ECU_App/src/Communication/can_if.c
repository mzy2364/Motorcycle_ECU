/*
 * can_if.c
 *
 *  Created on: 2023年3月29日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "can_if.h"
#include "Engine_Management.h"
#include "systick.h"
#include "bsp_adc.h"
#include "VBAT.h"
#include "TPS.h"
#include "User_Management.h"
#include "TLE8080EM.h"
#include "Fuel_Control.h"
#include "Spark_Control.h"
#include "ETEMP.h"
#include "ioa.h"
/*******************************************************************************
* Defines                                                                
*******************************************************************************/

/*******************************************************************************
* Macros                                                                
*******************************************************************************/

/*******************************************************************************
* Global Constant definition                         
*******************************************************************************/

/*******************************************************************************
* Local Constant definition                         
*******************************************************************************/

/*******************************************************************************
* Global Variables definition                         
*******************************************************************************/

/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
static uint32_t tick_100ms = 0;
static uint32_t tick_500ms = 0;
static uint32_t tick_1s = 0;
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void ecu1_msg_send(void);
static void ecu_dbg_01_msg_send(void);
static void ecu_dbg_02_msg_send(void);
static void ecu_dbg_03_msg_send(void);
static uint8_t ecu_msg_checksum(uint8_t data_byte_array[8]);
static uint8_t can_rx_callback(Can_CntrlType controller, const Can_PduType* PduInfo);
static uint8_t uds_diagnosis_func(uint8_t *data,uint16_t length);
static uint8_t ecu_ctrl_01_rx_func(const Can_PduType* PduInfo);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief CAN IF模块初始化
  * @param void
  * @retval	void
  * @note
  */
void can_if_init(void)
{
	can_init(CAN_COM0,CAN_BAUDRATE_500K);
	can_set_rx_cbk_func(CAN_COM0,can_rx_callback);
}

/**
  * @brief CAN IF模块周期调度任务
  * @param void
  * @retval	void
  * @note
  */
void can_if_task(void)
{
	/* 10ms */
	ecu1_msg_send();
	ecu_dbg_01_msg_send();
	ecu_dbg_02_msg_send();
	ecu_dbg_03_msg_send();

	/* 100ms */
	if(tick_100ms++ >= 10)
	{
		tick_100ms = 0;
	}

	/* 500ms */
	if(tick_500ms++ >= 50)
	{
		tick_500ms = 0;
	}

	/* 1s */
	if(tick_1s++ >= 100)
	{
		tick_1s = 0;
	}
}

/**
  * @brief ECU1报文发送
  * @param void
  * @retval	void
  * @note
  */
static void ecu1_msg_send(void)
{
	static uint8_t ecu1_rolling_counter = 0;
	uint8_t data[8] = {0};
	Can_PduType PduInfo;
	PduInfo.id = 0X201;
	PduInfo.length = 8;
	PduInfo.sdu = data;

	data[0] = engine_speed & 0XFF;
	data[1] = (engine_speed >> 8) & 0XFF;
	data[2] = load << 1;
	data[3] = vbat_get_mv() & 0XFF;
	data[4] = (vbat_get_mv() >> 8) & 0XFF;
	data[5] = etemp_get_temperature() + 40;
	data[6] |= (mil_status << 4) & 0X30;
	data[6] |= (load_valid << 6) & 0X40;
	data[6] |= ecu1_rolling_counter & 0X0F;
	data[7] = ecu_msg_checksum(data);
	can_write(CAN_COM0, &PduInfo);

	ecu1_rolling_counter++;
	if(ecu1_rolling_counter > 0X0F)
		ecu1_rolling_counter = 0;
}

/**
  * @brief ECU1报文发送
  * @param void
  * @retval	void
  * @note
  */
static void ecu_dbg_01_msg_send(void)
{
	uint8_t data[8] = {0};
	Can_PduType PduInfo;
	PduInfo.id = 0X300;
	PduInfo.length = 8;
	PduInfo.sdu = data;

	data[0] = tps_adc_get() & 0XFF;
	data[1] = (tps_adc_get() >> 8) & 0XFF;
	data[2] = adc_read_channel(MAP_ADC_CHANNEL) & 0XFF;
	data[3] = (adc_read_channel(MAP_ADC_CHANNEL) >> 8) & 0XFF;
	data[4] = adc_read_channel(ATEMP_ADC_CHANNEL) & 0XFF;
	data[5] = (adc_read_channel(ATEMP_ADC_CHANNEL) >> 8) & 0XFF;
	data[6] = adc_read_channel(ETEMP_ADC_CHANNEL) & 0XFF;
	data[7] = (adc_read_channel(ETEMP_ADC_CHANNEL) >> 8) & 0XFF;
	can_write(CAN_COM0, &PduInfo);
}

/**
  * @brief ECU2报文发送
  * @param void
  * @retval	void
  * @note
  */
static void ecu_dbg_02_msg_send(void)
{
	uint8_t data[8] = {0};
	Can_PduType PduInfo;
	PduInfo.id = 0X301;
	PduInfo.length = 8;
	PduInfo.sdu = data;

	data[0] = adc_read_channel(MAF_ADC_CHANNEL) & 0XFF;
	data[1] = (adc_read_channel(MAF_ADC_CHANNEL) >> 8) & 0XFF;
	data[2] = adc_read_channel(O2IN_ADC_CHANNEL) & 0XFF;
	data[3] = (adc_read_channel(O2IN_ADC_CHANNEL) >> 8) & 0XFF;
	data[4] |= (app_state & 0X07);
	data[4] |= di_ign_on << 3;
	data[4] |= di_hs_01 << 4;
	data[4] |= di_ls_01 << 5;
	data[4] |= di_ls_02 << 6;
	data[5] = current_spark_dwell_tooth;
	data[6] = tle8080em_get_diag_reg() & 0XFF;
	data[7] = (tle8080em_get_diag_reg() >> 8) & 0XFF;
	can_write(CAN_COM0, &PduInfo);
}

/**
  * @brief ECU3报文发送
  * @param void
  * @retval	void
  * @note
  */
static void ecu_dbg_03_msg_send(void)
{
	uint8_t data[8] = {0};
	Can_PduType PduInfo;
	PduInfo.id = 0X302;
	PduInfo.length = 8;
	PduInfo.sdu = data;

	data[0] = current_fuel_pulse & 0XFF;
	data[1] = (current_fuel_pulse >> 8) & 0XFF;
	can_write(CAN_COM0, &PduInfo);
}

/**
  * @brief CAN接收回调函数
  * @param controller:CAN通道
  * @param PduInfo:CAN报文信息
  * @retval	always 0
  * @note
  */
static uint8_t can_rx_callback(Can_CntrlType controller, const Can_PduType* PduInfo)
{
	if(controller == CAN_COM0)
	{
		switch(PduInfo->id)
		{
		case 0X781:
			uds_diagnosis_func(PduInfo->sdu,PduInfo->length);
			break;
		case 0X230:
			ecu_ctrl_01_rx_func(PduInfo);
			break;
		default:
			break;
		}
	}

	return 0;
}

/**
  * @brief UDS诊断报文处理
  * @param data:CAN报文数组
  * @param length:数据长度
  * @retval always 0
  * @note 暂时只处理升级报文
  */
static uint8_t uds_diagnosis_func(uint8_t *data,uint16_t length)
{
	if(length == 8)
	{
		if((data[0] == 0X02) && (data[1] == 0X10) && (data[2] == 0X02))
		{
			static uint8_t response_wait_times = 0;
			static uint32_t last_recv_time = 0;		//记录上一次收到这个报文的时间 用来向上位机申请复位时间
			if(last_recv_time > systick_get_current_tick())
			{
				//第一次收到10服务 需要等待下位机响应
				response_wait_times = 0;
			}
			else if((systick_get_current_tick() - last_recv_time) <= 100)
			{
				//两次帧发送间隔在100ms以内
				response_wait_times++;
			}
			else
			{
				//两次帧发送间隔超过100ms
				response_wait_times = 0;
			}
			last_recv_time = systick_get_current_tick();
			if(response_wait_times < 5)
			{
				//发送等待响应
				uint8_t data_buf[8] = {0};
				Can_PduType PduInfo;
				PduInfo.id = 0X7E1;
				PduInfo.length = 8;
				PduInfo.sdu = data_buf;

				data_buf[0] = 0X03;
				data_buf[1] = 0X7F;
				data_buf[2] = 0X10;
				data_buf[3] = 0X78;
				data_buf[4] = 0XAA;
				data_buf[5] = 0XAA;
				data_buf[6] = 0XAA;
				data_buf[7] = 0XAA;
				can_write(CAN_COM0, &PduInfo);
			}
			else
			{
				//复位
				NVIC_SystemReset();
			}
		}
	}
	return 0;
}

/**
  * @brief ecu_ctrl_01报文解析
  * @param PduInfo:接收报文信息
  * @retval
  * @note
  */
static uint8_t ecu_ctrl_01_rx_func(const Can_PduType* PduInfo)
{
	cal_fuel_pump_oe = PduInfo->sdu[0] & 0X01;
	cal_fuel_pump_ov = (PduInfo->sdu[0] >> 1) & 0X01;
	cal_mil_oe = (PduInfo->sdu[0] >> 2) & 0X01;
	cal_mil_ov = (PduInfo->sdu[0] >> 3) & 0X01;
	cal_inj_oe = (PduInfo->sdu[0] >> 4) & 0X01;
	cal_inj_ov = (PduInfo->sdu[0] >> 5) & 0X01;
	cal_ignition_oe = (PduInfo->sdu[0] >> 6) & 0X01;
	cal_ignition_ov = (PduInfo->sdu[0] >> 7) & 0X01;
	cal_fuel_correction = 0;
	cal_fuel_correction |= (PduInfo->sdu[2]);
	cal_fuel_correction <<= 8;
	cal_fuel_correction |= (PduInfo->sdu[1]);
	cal_spark_angle_correction = (PduInfo->sdu[3]);
	cal_spark_angle = (PduInfo->sdu[5] << 8) | PduInfo->sdu[4];
	return 0;
}

/**
  * @brief CAN报文CHECKSUM计算
  * @param data_byte_array:CAN报文数组
  * @retval
  * @note
  */
static uint8_t ecu_msg_checksum(uint8_t data_byte_array[8])
{
    #define cb_DATA_BYTE_SIZE 7
    #define cb_CRC_POLY 0x2F       //polynom = 0x2F

    uint8_t bit_index = 0;
    uint8_t byte_index = 0;
    uint8_t crc = 0xFF;
    for( byte_index=0; byte_index<cb_DATA_BYTE_SIZE;++byte_index )
    {
        crc ^= data_byte_array[byte_index];
        for( bit_index=0; bit_index<8; ++bit_index )
          {
              if( (crc & 0x80) != 0 )
                   crc = (crc << 1) ^ cb_CRC_POLY;
               else
                    crc = (crc << 1);
           }
    }
    return  ~crc;                           //return inverse value
}

