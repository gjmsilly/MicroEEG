#ifndef __MICROEEG_MISC_H__
#define __MICROEEG_MISC_H__

/****************************************************************/
/* MicroEEG Misc Function                                       */
/*  1. AttrChangeProcess                                        */
/*  2. Timestamp_Service                                        */
/* 											                                        */
/* All Rights Reserved                                          */
/* Modified by Liu Miao 2019.6                                  */
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

#endif   /* __MICROEEG_MISC_H__ */
