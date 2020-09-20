/**
 * @file    microEEG_misc.c
 * @author  modified by gjmsilly
 * @brief   MicroEEG 杂项服务
 * @version 1.0
 * @date    2020-09-09
 * @copyright (c) 2020 gjmsilly
 *
 */

#include "microEEG_misc.h"
#include "ads1299.h"
#include "w5500_service.h"
#include "AttritubeTable.h"
#include "time.h"
#include "string.h"

/******************************************************************************
 * GLOBAL VARIABLES
 */
uint32_t CurTimeStamp = 0;	//!< 当前时间
uint8_t* pCurTimeStamp;			//!< 当前时间指针		
uint16_t CurEventTag = 0;
char* pcCurEventTag = (char*)&CurEventTag;

/*  ======================== 属性值变化处理服务 ============================
 */
/*! @fn	AttrChangeProcess 
 *	
 *	@param AttrChangeNum - 变化的属性值编号
 *
 *	@return SUCCESS 处理完成
 *					ERROR 处理异常
 */
uint8_t AttrChangeProcess (uint8_t AttrChangeNum)
{
	uint8_t ret = SUCCESS;
	uint8_t *pValue;

	switch(AttrChangeNum)
	{
		case SAMPLING: 
			App_GetAttr(SAMPLING,pValue); //获取属性值
			
			if(*pValue == SAMPLLE_START )
			{
			 /* ads1299 开始采集 */
			 ADS1299_SendCommand(ADS1299_CMD_RDATAC);				
			 ADS1299_SendCommand(ADS1299_CMD_START);
			}else
			/* ads1299 停止采集 */
			 ADS1299_SendCommand(ADS1299_CMD_STOP);
			break;
	}
	
}

	
/*  ============================ 时间戳服务 ==============================
 */ 
/*! @brief	样本时间戳服务 
 *					本服务生成样本的增量时间戳 uint32 精度10us 
 */

static void STSG_TIM5_Init(void);	

//样本时间戳服务		
void SampleTimestamp_Service_Init(void)
{
	pCurTimeStamp = (uint8_t*)&CurTimeStamp;	//!< 时间戳初始化
	
	STSG_TIM5_Init();	//!< 计时器初始化
}

//样本时间戳服务初始化
static void STSG_TIM5_Init(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct;
	
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
	
  TIM_InitStruct.Prescaler = 800;      //!< 80MHz APB1_TIM 10us/1Count
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFFFFFFF; //!< 2^32 
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);

  LL_TIM_DisableMasterSlaveMode(TIM5);
	
	TIM5->CNT = 0;	//!< 计数值初始化
	LL_TIM_EnableCounter(TIM5);
}

/*! @brief	首样时间戳服务 
 *					本服务将RTC时间转换生成标准UNIX格式 32位时间戳 uint64
 *
 *	@param	pout - 需要输出UNIX时间戳的地址
 */
void UNIXTimestamp_Service(uint8_t *pout)
{
	uint32_t RTCtime;	//!< RTC时间
	uint32_t RTCdate;	//!< RTC日期
	uint32_t _unix_;	//!< UNIX时间戳
	
	struct tm stm;		
	
	RTCtime = LL_RTC_TIME_Get(RTC);	//!< return(Format: 0x00HHMMSS)
	RTCdate = LL_RTC_DATE_Get(RTC); //!< return (Format: 0xWWDDMMYY)
	
	/*	用标准库 time.h 转换时间戳
			该库可以将rtc时间转换为unix时间戳	*/
	stm.tm_year =(int)((RTCdate)+100)&0x0000FF;	//!< rtc起始时间2000年 库起始时间1990年
	stm.tm_mon	=(int)((RTCdate >> 8)-1)&0x0000FF; //!< rtc一月为1 库一月为0  
	stm.tm_mday	=(int)(RTCdate >> 16)&0x0000FF;
	stm.tm_wday	=(int)(RTCdate >> 24)&0x0000FF;
  
	stm.tm_hour	=(int)((RTCtime>> 16)-8)&0x0000FF;	//!< 北京时间-8=UTC   
	stm.tm_min	=(int)(RTCtime>> 8)&0x0000FF;  
	stm.tm_sec	=(int)(RTCtime)&0x0000FF; 
	
	if((_unix_ = mktime(&stm))!= -1)	//!< 获取UNIX格式时间戳
	{
		memcpy(pout,&_unix_,4); //!< 32位
	}	
}

