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

/*********************************************************************
 * Macros
 */

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
void LED_Service_Init(void);
void LED_Service(uint16_t devstate);

#endif   /* __MICROEEG_MISC_H__ */
