/**
 * @file    MicroEEG_Misc.c
 * @author  modified by gjmsilly
 * @brief   MicroEEG �������
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
	uint8_t val;
	
	//!< ads1299 ��������
	TADS1299CHnSET ChVal; 
	
	switch(AttrChangeNum)
	{
		case SAMPLING: 
			App_GetAttr(SAMPLING,CHX_NONE,pValue); // ��ȡ����ֵ

			if( *(uint8_t*)pValue == SAMPLLE_START )
			{
				
				/* ϵͳ״̬���� */				
				SYS_Event |= EEG_DATA_START_EVT; //!< �����¼���һ��ad���ݿ�ʼ�ɼ�		
				
				/* ����ʱ��������� */				
				LL_TIM_EnableCounter(TIM5); //!< ����������ʱ�����ʱ��
				
				/* ads1299 ��ʼ�ɼ� */				
				ADS1299_Sampling_Control(SAMPLLE_START);
				
				/* ������Ҫ���� */
				BKP_Write(SAMPLING_BKP,(uint32_t)SAMPLLE_START);
				
			}
			else
			{
				/* ϵͳ״̬���� */
				SYS_Event |= EEG_STOP_EVT; //!< �����¼���ad������ͣ�ɼ�
				
				/* ads1299 ֹͣ�ɼ� */				
				ADS1299_Sampling_Control(SAMPLLE_STOP);
						
				/* ����ʱ���������ֹͣ */					
				LL_TIM_DisableCounter(TIM5); //!< �رն�ʱ��
				TIM5->CNT = 0;	//!< ��������ʱ�����ʱ������
			
				/* ������Ҫ���� */
				BKP_Write(SAMPLING_BKP,(uint32_t)SAMPLLE_STOP);

			}
		break;
			
		case CURSAMPLERATE:
			
			/* ��ȫ�������޸Ĳ�����ֵʱȷ��ֹͣ���� */
			App_GetAttr(SAMPLING,CHX_NONE,pValue);	// ��ȡ���ڲ�������ֵ	
		
			if((*pValue&0x0000FF) == SAMPLLE_START )		
				ADS1299_Sampling_Control(SAMPLLE_STOP); //!< �����ڲ���������ֹͣ����
			
			App_GetAttr(CURSAMPLERATE,CHX_NONE,pValue); //��ȡ��ǰ����������ֵ
			
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
				
				default: //!< default samplerate 1K  ����ϵ��û���޸Ĳ����ʵ�������쳣�ϵ�����
					ADS1299_SetSamplerate(0,1000);		//1kHZ����			
				break;		
			}
			
			/* ������Ҫ���� */
			BKP_Write(CURSAMPLERATE_BKP,*pValue);		
			
		break;
		
		case CURGAIN:
			
			/* ��ȫ�������޸Ĳ�����ֵʱȷ��ֹͣ���� */
			App_GetAttr(SAMPLING,CHX_NONE,pValue);	// ��ȡ���ڲ�������ֵ	
		
			if((*pValue&0x0000FF) == SAMPLLE_START )		
				ADS1299_Sampling_Control(SAMPLLE_STOP); //!< �����ڲ���������ֹͣ����
			
			App_GetAttr(CURGAIN,CHX_NONE,pValue); //��ȡ��ǰ��������ֵ
			
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
				
				default: //!< default gain x24 ����ϵ��û���޸������������쳣�ϵ�����
					ADS1299_SetGain(0,24);				
				break;						
			}
			
			/* ������Ҫ���� */
			BKP_Write(CURGAIN_BKP,*pValue);	
			
		break;
	
		case IMP_MEAS_EN:
			App_GetAttr(IMP_MEAS_EN,CHX_NONE,pValue); // ��ȡ����ֵ

			if( *(uint8_t*)pValue == IMP_MEAS_MODE )
			{
				SYS_Event |= EEG_IMP_MODE; //!< �����¼����迹���ģʽ
			}
			else if( *(uint8_t*)pValue == SAMPLE_MODE ) 
			{
				SYS_Event &= ~EEG_IMP_MODE; //!< ���ǰ���¼�
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

#ifdef UNIXTimestamp 
/*! @brief	����ʱ������� 
 *					������RTCʱ��ת�����ɱ�׼UNIX��ʽ 32λʱ��� uint64
 *
 *	@param	pout - ��Ҫ���UNIXʱ����ĵ�ַ
 */
void UNIXTimestamp_Service(uint8_t *pout)
{
	RTC_DateTypeDef sdatestructureget;	//!< RTC������
	RTC_TimeTypeDef stimestructureget;	//!< RTCʱ����
	uint32_t _unix_;	//!< UNIXʱ���

	struct tm stm;		
	
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);	

	/*	�ñ�׼�� time.h ת��ʱ���
			�ÿ���Խ�rtcʱ��ת��Ϊunixʱ���	*/

	stm.tm_year =((sdatestructureget.Year) +100);	//!< rtc��ʼʱ��2000�� ����ʼʱ��1900��
	stm.tm_mon	=((sdatestructureget.Month) -1); //!< rtcһ��Ϊ1 ��һ��Ϊ0  
	stm.tm_mday	=(sdatestructureget.Date);
  
	stm.tm_hour	=((stimestructureget.Hours)-8);	//!< ����ʱ��-8=UTC   
	stm.tm_min	=(stimestructureget.Minutes);  
	stm.tm_sec	=(stimestructureget.Seconds); 

	if((_unix_ = mktime(&stm))!= -1)	//!< ��ȡUNIX��ʽʱ���
	{
		memcpy(pout,&_unix_,4); //!< 32λ
	}
}
#endif

/*  ============================ LED���� ==============================
 */ 
/*! @brief	LED����	
 *					�������ṩ�豸LED״ָ̬ʾ
 */
void LED_Service_Init(void)
{
	//LED��ʼ��
	PWR_LED1_ON;	//�豸ͨ��״ָ̬ʾ
	PWR_LED2_OFF;
	ACQ_LED1_OFF; //�豸����״ָ̬ʾ
	ACQ_LED2_OFF;
	ERR_LED1_OFF; 
	ERR_LED2_OFF; //�豸����״ָ̬ʾ
}

void LED_Service(uint16_t devstate)
{
	
	if( devstate & EEG_DATA_CPL_EVT ) //!< �豸������
	{		
		ACQ_LED1_TOGGLE;
		ERR_LED2_OFF;
	}
	else if( devstate & EEG_STOP_EVT ) //!< �豸ֹͣ����
		ACQ_LED1_OFF;
	
	if( devstate & POWERDOWN_EVT ) //!< �豸�����쳣�ϵ�
		ERR_LED2_ON;
}

/*  ============================ ���ݷ��� ==============================
 */ 
/*! @brief	���ݷ���	
 *					�������ṩ�豸��Ҫ����״̬�ı��ݷ���
 *					�����豸�쳣�ϵ����ϵ�ָ�
 */

/*! 
 *	@fn			BKP_Service_Init 
 *
 *	@brief	���ݷ����ʼ��
 *					���豸�ϵ��Ե���ǰ�豸״̬���м�⣬
 *					���쳣���������ʱ�ָ�����ǰ״̬,���豸����������λ���б��ݡ�
 *					!������������rtc��ʼ���������������ʼ��ǰ���á�
 */
void BKP_Service_Init()
{
	uint32_t val;
	
	/* Step 1 - ��ȡ����״̬ */
	val = BKP_Read(SAMPLING_BKP);
	
	if( val == SAMPLLE_START ) //!< ���ϵ�ǰ�豸���ڲɼ�
	{
		SYS_Event = NULL;						//!< ��������¼�
		SYS_Event |= POWERDOWN_EVT; //!< �����¼����쳣�ϵ�
		
		/* Step 2 - ���W5500״̬ */
		if(getSn_SR(0)!= SOCK_ESTABLISHED) //!< W5500�Ͽ�
			SYS_Event |= SOCKETDOWN_EVT; //!< �����¼�������Ͽ�
			
	}
	else	//!< ����������
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
 *	@brief	���ݻָ����������ڱ��ݷ������쳣����á�
 */
void BKP_Service_Recovery()
{
	uint32_t val;
		
	/* Step 1.2 - �ָ������������ */
//	val = BKP_Read(CURGAIN_BKP);
//	if(val!=0)
//	{
//		App_WriteAttr((uint8_t)CURGAIN,val);					//!< �ָ�ȫ����������ֵ
//		AttrChangeProcess(CURGAIN);
//	}
	
	val = BKP_Read(CURSAMPLERATE_BKP);
	if(val!=0)
	{
		App_WriteAttr((uint8_t)CURSAMPLERATE,CHX_NONE,val);		//!< �ָ�ȫ�ֲ���������ֵ
		AttrChangeProcess(CURSAMPLERATE);	
	}
	
	val = BKP_Read(SAMPLING_BKP);
	if(val!=0)
	{
		App_WriteAttr((uint8_t)SAMPLING,CHX_NONE,val);					//!< �ָ�������������ֵ
		AttrChangeProcess(SAMPLING);
	}
}

/*! @fn			BKP_Write 
 *	
 *	@param 	bkpParam - �����ݲ���
 *					bkpValue - �����ݲ���ֵ
 *
 *	@return NULL
 */
void BKP_Write(uint32_t bkpParam, uint32_t bkpValue)
{
	LL_RTC_BAK_SetRegister(RTC,bkpParam,bkpValue);
}

/*! @fn			BKP_Read 
 *	
 *	@param 	bkpParam - ����ȡ�ı��ݲ���
 *
 *	@return ���ݲ���ֵ
 */
uint32_t BKP_Read(uint32_t bkpParam)
{
	uint32_t val;
	
	val = LL_RTC_BAK_GetRegister(RTC,bkpParam);
	
	return(val);
}