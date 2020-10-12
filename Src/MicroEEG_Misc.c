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
uint32_t CurTimeStamp[10];			//!< 当前时间
uint32_t* pCurTimeStamp;				//!< 当前时间指针		
static uint32_t attrvalue=0;		//!< 属性值
uint32_t *pValue =&attrvalue;		//!< 属性值指针


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
	
	//!< ads1299 参数配置
	TADS1299CHnSET ChVal; 
	
	switch(AttrChangeNum)
	{
		case SAMPLING: 
			App_GetAttr(SAMPLING,pValue); //获取属性值

			if((*pValue&0x0000FF) == SAMPLLE_START )
			{
			 /* ads1299 开始采集 */
				TIM5->CNT=0; //!< 每次采样时增量计时器归零				
				Mod_DRDY_INT_Enable //使能nDReady中断
				LL_GPIO_SetOutputPin(Mod_START_GPIO_Port, Mod_START_Pin);
				ADS1299_SendCommand(ADS1299_CMD_RDATAC);			
				Mod_CS_Enable;	
				
			}else
			{
			/* ads1299 停止采集 */
				Mod_DRDY_INT_Disable //关闭nDReady中断		
				Mod_CS_Disable;
				LL_GPIO_ResetOutputPin(Mod_START_GPIO_Port, Mod_START_Pin);
				ADS1299_SendCommand(ADS1299_CMD_SDATAC);
				
				SYS_Event |= EEG_STOP_EVT; //!< 更新事件：ad数据暂停采集	
			}
		break;
			
		case CURSAMPLERATE:
			App_GetAttr(CURSAMPLERATE,pValue); //获取属性值
		
			switch(*pValue)
			{
				case 250:
						ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x96);		//250HZ采样
				break;
				
				case 500:
						ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x95);		//500HZ采样
				break;
				
				case 1000:
						ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x94);		//1kHZ采样
				break;				
				
				case 2000:
					ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x93);		//2kHZ采样
				break;
				
				default:
					ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x94);		//1kHZ采样
				break;			
			}				
					
		break;
		
		case CURGAIN:
			App_GetAttr(CURGAIN,pValue); //获取属性值
			
			switch(*(uint8_t*)pValue)
			{
				case 1:
					ChVal.control_bit.gain = 0;
				break;
				
				case 2:
					ChVal.control_bit.gain = 1;
				break;
				
				case 4:
					ChVal.control_bit.gain = 2;
				break;
				
				case 6:
					ChVal.control_bit.gain = 3;
				break;
				
				default:
					ChVal.control_bit.gain = 0;
				break;					
			}	
			ChVal.control_bit.pd = 0;
			ChVal.control_bit.mux = 0;			
			
			for(uint8_t i=0;i<8;i++)
			{
				ADS1299_Channel_Config(0,ADS1299_REG_CH1SET+i,ChVal);
				WaitUs(2);
			}
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
	pCurTimeStamp = CurTimeStamp;	//!< 时间戳初始化
	
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

