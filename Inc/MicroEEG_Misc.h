#ifndef __MICROEEG_MISC_H__
#define __MICROEEG_MISC_H__

/****************************************************************/
/* MicroEEG Misc Function                                       */
/*  1. AttrChangeProcess                                        */
/*  2. Timestamp_Service                                        */
/* 	3. LED_Service										                          */
/* All Rights Reserved                                          */
/* Modified by gjmsily 2021.1.15                                */
/****************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "main.h"
#include "stm32f4xx_ll_rtc.h"

/*********************************************************************
 * Macros
 */
/* ���ݷ�����ز��� */
#define SYS_EVENT_BKP				LL_RTC_BKP_DR0	//!< ϵͳ�¼�����
#define TCP_DIPR_BKP				LL_RTC_BKP_DR1	//!< Ŀ������IP���ݣ�TCP�������Ӻ��ȡ��UDP���ã�
#define SAMPLING_BKP				LL_RTC_BKP_DR2	//!< �������Ա���
#define CURGAIN_BKP					LL_RTC_BKP_DR3	//!< ��ǰȫ���������Ա���
#define CURSAMPLERATE_BKP		LL_RTC_BKP_DR4	//!< ��ǰȫ�ֲ��������Ա���

/***********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32_t* pCurTimeStamp;	//!< ��ǰʱ��ָ��	

/**********************************************************************
 * FUNCTIONS
 */
/* ����ֵ�仯������� */
uint8_t AttrChangeProcess (uint8_t AttrChangeNum);

/* ʱ������� */
void SampleTimestamp_Service_Init(void);
void UNIXTimestamp_Service(uint8_t *pout);

/* LED���� */
void LED_Service_Init(void);
void LED_Service(uint16_t devstate);

/* ���ݷ��� */
void BKP_Service_Init();
void BKP_Service_Recovery();
void BKP_Write(uint32_t bkpParam, uint32_t bkpValue);
uint32_t BKP_Read(uint32_t bkpParam);

#endif   /* __MICROEEG_MISC_H__ */
