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
extern uint8_t* pCurTimeStamp;	//!< 当前时间指针	

/**********************************************************************
 * FUNCTIONS
 */
/* 属性值变化处理服务 */
uint8_t AttrChangeProcess (uint8_t AttrChangeNum);

/* 时间戳服务 */
void Timestamp_Service_Init(void);

#endif   /* __MICROEEG_MISC_H__ */
