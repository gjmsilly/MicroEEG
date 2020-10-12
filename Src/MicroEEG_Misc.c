/**
 * @file    microEEG_misc.c
 * @author  modified by gjmsilly
 * @brief   MicroEEG �������
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
uint32_t CurTimeStamp[10];			//!< ��ǰʱ��
uint32_t* pCurTimeStamp;				//!< ��ǰʱ��ָ��		
static uint32_t attrvalue=0;		//!< ����ֵ
uint32_t *pValue =&attrvalue;		//!< ����ֵָ��


/*  ======================== ����ֵ�仯������� ============================
 */
/*! @fn	AttrChangeProcess 
 *	
 *	@param AttrChangeNum - �仯������ֵ���
 *
 *	@return SUCCESS �������
 *					ERROR �����쳣
 */
uint8_t AttrChangeProcess (uint8_t AttrChangeNum)
{
	uint8_t ret = SUCCESS;
	
	//!< ads1299 ��������
	TADS1299CHnSET ChVal; 
	
	switch(AttrChangeNum)
	{
		case SAMPLING: 
			App_GetAttr(SAMPLING,pValue); //��ȡ����ֵ

			if((*pValue&0x0000FF) == SAMPLLE_START )
			{
			 /* ads1299 ��ʼ�ɼ� */
				TIM5->CNT=0; //!< ÿ�β���ʱ������ʱ������				
				Mod_DRDY_INT_Enable //ʹ��nDReady�ж�
				LL_GPIO_SetOutputPin(Mod_START_GPIO_Port, Mod_START_Pin);
				ADS1299_SendCommand(ADS1299_CMD_RDATAC);			
				Mod_CS_Enable;	
				
			}else
			{
			/* ads1299 ֹͣ�ɼ� */
				Mod_DRDY_INT_Disable //�ر�nDReady�ж�		
				Mod_CS_Disable;
				LL_GPIO_ResetOutputPin(Mod_START_GPIO_Port, Mod_START_Pin);
				ADS1299_SendCommand(ADS1299_CMD_SDATAC);
				
				SYS_Event |= EEG_STOP_EVT; //!< �����¼���ad������ͣ�ɼ�	
			}
		break;
			
		case CURSAMPLERATE:
			App_GetAttr(CURSAMPLERATE,pValue); //��ȡ����ֵ
		
			switch(*pValue)
			{
				case 250:
						ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x96);		//250HZ����
				break;
				
				case 500:
						ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x95);		//500HZ����
				break;
				
				case 1000:
						ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x94);		//1kHZ����
				break;				
				
				case 2000:
					ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x93);		//2kHZ����
				break;
				
				default:
					ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x94);		//1kHZ����
				break;			
			}				
					
		break;
		
		case CURGAIN:
			App_GetAttr(CURGAIN,pValue); //��ȡ����ֵ
			
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

	
/*  ============================ ʱ������� ==============================
 */ 
/*! @brief	����ʱ������� 
 *					��������������������ʱ��� uint32 ����10us 
 */

static void STSG_TIM5_Init(void);	

//����ʱ�������		
void SampleTimestamp_Service_Init(void)
{
	pCurTimeStamp = CurTimeStamp;	//!< ʱ�����ʼ��
	
	STSG_TIM5_Init();	//!< ��ʱ����ʼ��
}

//����ʱ��������ʼ��
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
	
	TIM5->CNT = 0;	//!< ����ֵ��ʼ��
	LL_TIM_EnableCounter(TIM5);
}

/*! @brief	����ʱ������� 
 *					������RTCʱ��ת�����ɱ�׼UNIX��ʽ 32λʱ��� uint64
 *
 *	@param	pout - ��Ҫ���UNIXʱ����ĵ�ַ
 */
void UNIXTimestamp_Service(uint8_t *pout)
{
	uint32_t RTCtime;	//!< RTCʱ��
	uint32_t RTCdate;	//!< RTC����
	uint32_t _unix_;	//!< UNIXʱ���
	
	struct tm stm;		
	
	RTCtime = LL_RTC_TIME_Get(RTC);	//!< return(Format: 0x00HHMMSS)
	RTCdate = LL_RTC_DATE_Get(RTC); //!< return (Format: 0xWWDDMMYY)
	
	/*	�ñ�׼�� time.h ת��ʱ���
			�ÿ���Խ�rtcʱ��ת��Ϊunixʱ���	*/
	stm.tm_year =(int)((RTCdate)+100)&0x0000FF;	//!< rtc��ʼʱ��2000�� ����ʼʱ��1990��
	stm.tm_mon	=(int)((RTCdate >> 8)-1)&0x0000FF; //!< rtcһ��Ϊ1 ��һ��Ϊ0  
	stm.tm_mday	=(int)(RTCdate >> 16)&0x0000FF;
	stm.tm_wday	=(int)(RTCdate >> 24)&0x0000FF;
  
	stm.tm_hour	=(int)((RTCtime>> 16)-8)&0x0000FF;	//!< ����ʱ��-8=UTC   
	stm.tm_min	=(int)(RTCtime>> 8)&0x0000FF;  
	stm.tm_sec	=(int)(RTCtime)&0x0000FF; 
	
	if((_unix_ = mktime(&stm))!= -1)	//!< ��ȡUNIX��ʽʱ���
	{
		memcpy(pout,&_unix_,4); //!< 32λ
	}	
}

