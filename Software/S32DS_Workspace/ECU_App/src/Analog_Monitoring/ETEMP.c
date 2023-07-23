/*
 * etemp.c
 *
 *	Engine Temperature Monitoring.
 *
 *  Created on: 2023年3月29日
 *      Author: mzy2364
 */



/*******************************************************************************
* include files                                                 
*******************************************************************************/
#include "ETEMP.h"
#include "bsp_adc.h"

/*******************************************************************************
* Defines                                                                
*******************************************************************************/
#define NTC_TABLE_LEN   191
#define VALID_TEMP		215
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
uint16_t engine_temp_adc = 0;
uint8_t engine_temp_valid = 0;
int16_t engine_temperature = VALID_TEMP;
/*******************************************************************************
* Local Variables definition                         
*******************************************************************************/
static uint8_t etemp_counter = 0;
static uint16_t etemp_data_buffer[ETEMP_BUFFER_SIZE];
static uint8_t etemp_buffer_counter = 0;
static uint8_t etemp_collection_rate_counter = 0;
static int16_t ntc_table[NTC_TABLE_LEN][2];
/*******************************************************************************
* Local Functions prototypes                         
*******************************************************************************/
static void etemp_adc_filter(void);
static float calculate_temperature_float(uint16_t ntc_adc, int16_t table[][2], int size);
static int16_t calculate_temperature_int16(uint16_t ntc_adc, int16_t table[][2], int size);
/*******************************************************************************
*  Global Functions Body                                   
*******************************************************************************/

/**
  * @brief 发动机缸体温度采集初始化
  * @param void
  * @retval	void
  * @note
  */
void etemp_init(void)
{
	for(etemp_counter = 0; etemp_counter < ETEMP_BUFFER_SIZE; etemp_counter++)
	{
		etemp_data_buffer[etemp_counter] = 0;
	}
}

/**
  * @brief 获取滤波后的发动机缸体温度传感器的ADC值
  * @param void
  * @retval	void
  * @note
  */
uint16_t etemp_adc_get(void)
{
	return engine_temp_adc;
}

/**
  * @brief 获取滤波后的发动机缸体温度值
  * @param void
  * @retval	温度值
  * @note 精确到1摄氏度
  */
int16_t etemp_get_temperature(void)
{
	return engine_temperature;
}

/**
  * @brief 获取发动机缸体温度的有效性
  * @param void
  * @retval	1-有效  0-无效
  * @note
  */
uint8_t etemp_get_temp_valid(void)
{
	return engine_temp_valid;
}

/**
  * @brief 发动机缸体温度监控
  * @param void
  * @retval	void
  * @note
  */
void etemp_monitoring(void)
{
	if(etemp_collection_rate_counter >= ETEMP_DATA_COLLECTION_RATE)
	{
		etemp_data_buffer[etemp_buffer_counter] = adc_read_channel(ETEMP_ADC_CHANNEL);

		etemp_collection_rate_counter = 0;
		etemp_buffer_counter++;
		if(etemp_buffer_counter >= ETEMP_BUFFER_SIZE)
		{
			etemp_adc_filter();
			etemp_buffer_counter = 0;
		}

	}
	else
	{
		etemp_collection_rate_counter++;
	}
}

/**
  * @brief 发动机缸体温度传感器滤波
  * @param void
  * @retval	void
  * @note
  */
static void etemp_adc_filter(void)
{
	uint32_t average = 0;
	for(etemp_counter = 0; etemp_counter < ETEMP_BUFFER_SIZE; etemp_counter++)
	{
		average += etemp_data_buffer[etemp_counter];
	}

	average = average >> 4;		//average/16

	engine_temp_adc = average;

	if((engine_temp_adc > ntc_table[0][1]) && (engine_temp_adc < ntc_table[NTC_TABLE_LEN-1][1]))
	{
		engine_temp_valid = 1;
		engine_temperature = calculate_temperature_int16(engine_temp_adc,ntc_table,NTC_TABLE_LEN);
	}
	else
	{
		engine_temp_valid = 0;
		engine_temperature = VALID_TEMP;
	}
}

/**
 * 使用NTC查表计算温度
 * @param ntc_adc NTC对应的ADC采集值
 * @param table NTC温度电阻表，是一个包含温度和电阻值的二维数组，按电阻值递增排序
 * @param size 表的长度，即包含的行数
 * @return 温度值，单位为摄氏度
 */
static float calculate_temperature_float(uint16_t ntc_adc, int16_t table[][2], int size)
{
	int low = 0;
	int high = size - 1;
	while (low <= high)
	{
		int mid = (low + high) / 2;
		if (ntc_adc < table[mid][1])
		{
			high = mid - 1;
		} 
		else if (ntc_adc > table[mid][1])
		{
			low = mid + 1;
		} 
		else
		{
			return table[mid][0];
		}
	}
	// 此时 low 和 high 分别指向最接近 ntc_adc 的两个电阻值
	if (low == 0)
	{
		return table[0][0];
	}
	else if (high == size - 1)
	{
		return table[size - 1][0];
	}
	else
	{
		float x0 = table[low - 1][0];
		float y0 = table[low - 1][1];
		float x1 = table[low][0];
		float y1 = table[low][1];
		return x0 + (x1 - x0) * (ntc_adc - y0) / (y1 - y0);
	}
}

/**
 * 使用NTC查表计算温度
 * @param ntc_adc NTC对应的ADC采集值
 * @param table NTC温度电阻表，是一个包含温度和电阻值的二维数组，按电阻值递增排序
 * @param size 表的长度，即包含的行数
 * @return 温度值，单位为摄氏度
 */
static int16_t calculate_temperature_int16(uint16_t ntc_adc, int16_t table[][2], int size)
{
	int low = 0;
	int high = size - 1;
	while (low <= high)
	{
		int mid = (low + high) / 2;
		if (ntc_adc < table[mid][1])
		{
			high = mid - 1;
		} 
		else if (ntc_adc > table[mid][1])
		{
			low = mid + 1;
		} 
		else
		{
			return table[mid][0];
		}
	}
	// 此时 low 和 high 分别指向最接近 ntc_adc 的两个电阻值
	if (low == 0)
	{
		return table[0][0];
	}
	else if (high == size - 1)
	{
		return table[size - 1][0];
	}
	else
	{
		return table[low][0];
	}
}

// NTC温度电阻表，按电阻值递增排序
static int16_t ntc_table[NTC_TABLE_LEN][2] = 
{
	{150,75},
	{149,76},
	{148,78},
	{147,80},
	{146,82},
	{145,84},
	{144,86},
	{143,88},
	{142,90},
	{141,92},
	{140,94},
	{139,96},
	{138,98},
	{137,101},
	{136,103},
	{135,106},
	{134,108},
	{133,111},
	{132,114},
	{131,117},
	{130,120},
	{129,122},
	{128,126},
	{127,129},
	{126,132},
	{125,135},
	{124,139},
	{123,142},
	{122,146},
	{121,150},
	{120,154},
	{119,157},
	{118,162},
	{117,166},
	{116,170},
	{115,175},
	{114,179},
	{113,184},
	{112,189},
	{111,194},
	{110,199},
	{109,204},
	{108,210},
	{107,216},
	{106,221},
	{105,227},
	{104,234},
	{103,240},
	{102,247},
	{101,253},
	{100,261},
	{99,268},
	{98,275},
	{97,283},
	{96,291},
	{95,299},
	{94,307},
	{93,316},
	{92,325},
	{91,334},
	{90,344},
	{89,354},
	{88,364},
	{87,374},
	{86,385},
	{85,396},
	{84,408},
	{83,419},
	{82,432},
	{81,444},
	{80,457},
	{79,471},
	{78,484},
	{77,498},
	{76,513},
	{75,528},
	{74,544},
	{73,559},
	{72,576},
	{71,593},
	{70,610},
	{69,629},
	{68,647},
	{67,666},
	{66,686},
	{65,706},
	{64,727},
	{63,748},
	{62,770},
	{61,793},
	{60,816},
	{59,840},
	{58,864},
	{57,889},
	{56,915},
	{55,942},
	{54,969},
	{53,997},
	{52,1026},
	{51,1055},
	{50,1085},
	{49,1115},
	{48,1147},
	{47,1179},
	{46,1212},
	{45,1245},
	{44,1280},
	{43,1315},
	{42,1350},
	{41,1387},
	{40,1423},
	{39,1461},
	{38,1500},
	{37,1539},
	{36,1578},
	{35,1618},
	{34,1659},
	{33,1700},
	{32,1743},
	{31,1785},
	{30,1828},
	{29,1871},
	{28,1915},
	{27,1959},
	{26,2003},
	{25,2048},
	{24,2093},
	{23,2138},
	{22,2184},
	{21,2229},
	{20,2275},
	{19,2321},
	{18,2366},
	{17,2412},
	{16,2458},
	{15,2503},
	{14,2548},
	{13,2593},
	{12,2638},
	{11,2682},
	{10,2726},
	{9,2770},
	{8,2813},
	{7,2856},
	{6,2898},
	{5,2939},
	{4,2979},
	{3,3020},
	{2,3059},
	{1,3098},
	{0,3136},
	{-1,3173},
	{-2,3209},
	{-3,3245},
	{-4,3279},
	{-5,3313},
	{-6,3346},
	{-7,3378},
	{-8,3409},
	{-9,3440},
	{-10,3469},
	{-11,3498},
	{-12,3525},
	{-13,3552},
	{-14,3578},
	{-15,3602},
	{-16,3626},
	{-17,3649},
	{-18,3672},
	{-19,3693},
	{-20,3714},
	{-21,3733},
	{-22,3752},
	{-23,3770},
	{-24,3788},
	{-25,3804},
	{-26,3820},
	{-27,3835},
	{-28,3850},
	{-29,3864},
	{-30,3877},
	{-31,3890},
	{-32,3902},
	{-33,3913},
	{-34,3924},
	{-35,3934},
	{-36,3944},
	{-37,3953},
	{-38,3962},
	{-39,3970},
	{-40,3978},
};
