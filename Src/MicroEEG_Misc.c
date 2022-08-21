/**
 * @file    MicroEEG_Misc.c
 * @author  modified by gjmsilly
 * @brief   MicroEEG 杂项服务
 * @version 1.0
 * @date    2020-09-09
 * @copyright (c) 2020 gjmsilly
 *
 */

#include "MicroEEG_Misc.h"

#include "ads1299.h"
#include "w5500_service.h"
#include "socket.h"
#include "AttritubeTable.h"
#include "string.h"


#ifdef UNIXTimestamp
#include "time.h"
#elif xUNIXTimestamp
#include "stm32f4xx_ll_rtc.h"
#include "stm32f446xx.h"
#endif

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
	uint8_t val;
	
	//!< ads1299 参数配置
	TADS1299CHnSET ChVal; 
	
	switch(AttrChangeNum)
	{
		case SAMPLING: 
			App_GetAttr(SAMPLING,CHX_NONE,pValue); // 获取属性值

			if( *(uint8_t*)pValue == SAMPLLE_START )
			{
				
				/* 系统状态更新 */				
				SYS_Event |= EEG_DATA_START_EVT; //!< 更新事件：一包ad数据开始采集		
				
				/* 样本时间戳服务开启 */				
				LL_TIM_EnableCounter(TIM5); //!< 打开样本增量时间戳定时器
				
				/* ads1299 开始采集 */				
				ADS1299_Sampling_Control(SAMPLLE_START);
				
				/* 备份重要参数 */
				BKP_Write(SAMPLING_BKP,(uint32_t)SAMPLLE_START);
				
			}
			else
			{
				/* 系统状态更新 */
				SYS_Event |= EEG_STOP_EVT; //!< 更新事件：ad数据暂停采集
				
				/* ads1299 停止采集 */				
				ADS1299_Sampling_Control(SAMPLLE_STOP);
						
				/* 样本时间戳戳服务停止 */					
				LL_TIM_DisableCounter(TIM5); //!< 关闭定时器
				TIM5->CNT = 0;	//!< 样本增量时间戳定时器归零
			
				/* 备份重要参数 */
				BKP_Write(SAMPLING_BKP,(uint32_t)SAMPLLE_STOP);

			}
		break;
			
		case CURSAMPLERATE:
			
			/* 安全操作：修改采样率值时确保停止采样 */
			App_GetAttr(SAMPLING,CHX_NONE,pValue);	// 获取正在采样属性值	
		
			if((*pValue&0x0000FF) == SAMPLLE_START )		
				ADS1299_Sampling_Control(SAMPLLE_STOP); //!< 若正在采样，则先停止采样
			
			App_GetAttr(CURSAMPLERATE,CHX_NONE,pValue); //获取当前采样率属性值
			
			switch(*pValue)
			{
				case 250:
					ADS1299_SetSamplerate(0,250);
				break;
				
				case 500:
					ADS1299_SetSamplerate(0,500);
				break;
				
				case 1000:
					ADS1299_SetSamplerate(0,1000);
				break;				
				
				case 2000:
					ADS1299_SetSamplerate(0,2000);	
				break;
				
				default: //!< default samplerate 1K  针对上电后没有修改采样率的情况下异常断电设置
					ADS1299_SetSamplerate(0,1000);		//1kHZ采样			
				break;		
			}
			
			/* 备份重要参数 */
			BKP_Write(CURSAMPLERATE_BKP,*pValue);		
			
		break;
		
		case CURGAIN:
			
			/* 安全操作：修改采样率值时确保停止采样 */
			App_GetAttr(SAMPLING,CHX_NONE,pValue);	// 获取正在采样属性值	
		
			if((*pValue&0x0000FF) == SAMPLLE_START )		
				ADS1299_Sampling_Control(SAMPLLE_STOP); //!< 若正在采样，则先停止采样
			
			App_GetAttr(CURGAIN,CHX_NONE,pValue); //获取当前增益属性值
			
			switch(*pValue)
			{
				case 1:
					ADS1299_SetGain(0,1);
				break;
				
				case 2:
					ADS1299_SetGain(0,2);
				break;
				
				case 4:
					ADS1299_SetGain(0,4);
				break;
				
				case 6:
					ADS1299_SetGain(0,6);
				break;
				
				case 8:
					ADS1299_SetGain(0,8);
				break;
				
				case 12:
					ADS1299_SetGain(0,12);
				break;
				
				case 24:
					ADS1299_SetGain(0,24);			
				break;	
				
				default: //!< default gain x24 针对上电后没有修改增益的情况下异常断电设置
					ADS1299_SetGain(0,24);				
				break;						
			}
			
			/* 备份重要参数 */
			BKP_Write(CURGAIN_BKP,*pValue);	
			
		break;
	
		case IMP_MEAS_EN:
			App_GetAttr(IMP_MEAS_EN,CHX_NONE,pValue); // 获取属性值

			if( *(uint8_t*)pValue == IMP_MEAS_MODE )
			{
				SYS_Event |= EEG_IMP_MODE; //!< 更新事件：阻抗检测模式
			}
			else if( *(uint8_t*)pValue == SAMPLE_MODE ) 
			{
				SYS_Event &= ~EEG_IMP_MODE; //!< 清除前序事件
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

#ifdef UNIXTimestamp 
/*! @brief	首样时间戳服务 
 *					本服务将RTC时间转换生成标准UNIX格式 32位时间戳 uint64
 *
 *	@param	pout - 需要输出UNIX时间戳的地址
 */
void UNIXTimestamp_Service(uint8_t *pout)
{
	RTC_DateTypeDef sdatestructureget;	//!< RTC年月日
	RTC_TimeTypeDef stimestructureget;	//!< RTC时分秒
	uint32_t _unix_;	//!< UNIX时间戳

	struct tm stm;		
	
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);	

	/*	用标准库 time.h 转换时间戳
			该库可以将rtc时间转换为unix时间戳	*/

	stm.tm_year =((sdatestructureget.Year) +100);	//!< rtc起始时间2000年 库起始时间1900年
	stm.tm_mon	=((sdatestructureget.Month) -1); //!< rtc一月为1 库一月为0  
	stm.tm_mday	=(sdatestructureget.Date);
  
	stm.tm_hour	=((stimestructureget.Hours)-8);	//!< 北京时间-8=UTC   
	stm.tm_min	=(stimestructureget.Minutes);  
	stm.tm_sec	=(stimestructureget.Seconds); 

	if((_unix_ = mktime(&stm))!= -1)	//!< 获取UNIX格式时间戳
	{
		memcpy(pout,&_unix_,4); //!< 32位
	}
}
#endif

/*  ============================ LED服务 ==============================
 */ 
/*! @brief	LED服务	
 *					本服务提供设备LED状态指示
 */
void LED_Service_Init(void)
{
	//LED初始化
	PWR_LED1_ON;	//设备通电状态指示
	PWR_LED2_OFF;
	ACQ_LED1_OFF; //设备采样状态指示
	ACQ_LED2_OFF;
	ERR_LED1_OFF; 
	ERR_LED2_OFF; //设备运行状态指示
}

void LED_Service(uint16_t devstate)
{
	
	if( devstate & EEG_DATA_CPL_EVT ) //!< 设备采样中
	{		
		ACQ_LED1_TOGGLE;
		ERR_LED2_OFF;
	}
	else if( devstate & EEG_STOP_EVT ) //!< 设备停止采样
		ACQ_LED1_OFF;
	
	if( devstate & POWERDOWN_EVT ) //!< 设备发生异常断电
		ERR_LED2_ON;
}

/*  ============================ 备份服务 ==============================
 */ 
/*! @brief	备份服务	
 *					本服务提供设备重要运行状态的备份服务，
 *					用以设备异常断电后的上电恢复
 */

/*! 
 *	@fn			BKP_Service_Init 
 *
 *	@brief	备份服务初始化
 *					在设备上电后对掉电前设备状态进行检测，
 *					对异常掉电情况及时恢复掉电前状态,若设备正常开机则复位所有备份。
 *					!本函数必须在rtc初始化后且其他服务初始化前调用。
 */
void BKP_Service_Init()
{
	uint32_t val;
	
	/* Step 1 - 读取采样状态 */
	val = BKP_Read(SAMPLING_BKP);
	
	if( val == SAMPLLE_START ) //!< 若断电前设备正在采集
	{
		SYS_Event = NULL;						//!< 清除所有事件
		SYS_Event |= POWERDOWN_EVT; //!< 更新事件：异常断电
		
		/* Step 2 - 检查W5500状态 */
		if(getSn_SR(0)!= SOCK_ESTABLISHED) //!< W5500断开
			SYS_Event |= SOCKETDOWN_EVT; //!< 更新事件：网络断开
			
	}
	else	//!< 若正常开机
	{
		BKP_Write(SYS_EVENT_BKP,NULL);
		BKP_Write(TCP_DIPR_BKP,NULL);
		BKP_Write(SAMPLING_BKP,NULL);
		BKP_Write(CURGAIN_BKP,NULL);
		BKP_Write(CURSAMPLERATE_BKP,NULL);
	}

}

/*! 
 *	@fn			BKP_Service_Recovery 
 *
 *	@brief	备份恢复，本函数在备份服务检测异常后调用。
 */
void BKP_Service_Recovery()
{
	uint32_t val;
		
	/* Step 1.2 - 恢复采样相关属性 */
//	val = BKP_Read(CURGAIN_BKP);
//	if(val!=0)
//	{
//		App_WriteAttr((uint8_t)CURGAIN,val);					//!< 恢复全局增益属性值
//		AttrChangeProcess(CURGAIN);
//	}
	
	val = BKP_Read(CURSAMPLERATE_BKP);
	if(val!=0)
	{
		App_WriteAttr((uint8_t)CURSAMPLERATE,CHX_NONE,val);		//!< 恢复全局采样率属性值
		AttrChangeProcess(CURSAMPLERATE);	
	}
	
	val = BKP_Read(SAMPLING_BKP);
	if(val!=0)
	{
		App_WriteAttr((uint8_t)SAMPLING,CHX_NONE,val);					//!< 恢复正常采样属性值
		AttrChangeProcess(SAMPLING);
	}
}

/*! @fn			BKP_Write 
 *	
 *	@param 	bkpParam - 待备份参数
 *					bkpValue - 待备份参数值
 *
 *	@return NULL
 */
void BKP_Write(uint32_t bkpParam, uint32_t bkpValue)
{
	LL_RTC_BAK_SetRegister(RTC,bkpParam,bkpValue);
}

/*! @fn			BKP_Read 
 *	
 *	@param 	bkpParam - 待读取的备份参数
 *
 *	@return 备份参数值
 */
uint32_t BKP_Read(uint32_t bkpParam)
{
	uint32_t val;
	
	val = LL_RTC_BAK_GetRegister(RTC,bkpParam);
	
	return(val);
}