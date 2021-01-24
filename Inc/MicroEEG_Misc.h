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
/* 备份服务相关参数 */
#define SYS_EVENT_BKP				LL_RTC_BKP_DR0	//!< 系统事件备份
#define TCP_DIPR_BKP				LL_RTC_BKP_DR1	//!< 目的主机IP备份（TCP建立连接后获取，UDP共用）
#define SAMPLING_BKP				LL_RTC_BKP_DR2	//!< 采样属性备份
#define CURGAIN_BKP					LL_RTC_BKP_DR3	//!< 当前全局增益属性备份
#define CURSAMPLERATE_BKP		LL_RTC_BKP_DR4	//!< 当前全局采样率属性备份

/***********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32_t* pCurTimeStamp;	//!< 当前时间指针	

/**********************************************************************
 * FUNCTIONS
 */
/* 属性值变化处理服务 */
uint8_t AttrChangeProcess (uint8_t AttrChangeNum);

/* 时间戳服务 */
void SampleTimestamp_Service_Init(void);
void UNIXTimestamp_Service(uint8_t *pout);

/* LED服务 */
void LED_Service_Init(void);
void LED_Service(uint16_t devstate);

/* 备份服务 */
void BKP_Service_Init();
void BKP_Service_Recovery();
void BKP_Write(uint32_t bkpParam, uint32_t bkpValue);
uint32_t BKP_Read(uint32_t bkpParam);

#endif   /* __MICROEEG_MISC_H__ */
