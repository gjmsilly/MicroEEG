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


/* 
	 INIT -- LED Green OFF
	 STANDBY -- LED Green ON
	 ACQUIRING -- LED Green FLASH
	 BTCONNECTLOST -- LED Red ON/ Green OFF
*/

/* Private variables ----------------------------------------------------*/
//uint8_t LED1FlashCounter;
//uint8_t LED2FlashCounter;

//uint32_t CurTimeStamp = 0;
//char* pcCurTimeStamp;
//uint16_t CurEventTag = 0;
//char* pcCurEventTag = (char*)&CurEventTag;


//uint8_t SYS_Status = 0;

//uint8_t BT_USART_Mutex;
//uint8_t BT_USART_Ready;

/*  ======================== 属性值变化处理服务 ============================
 */
/*! @fn	AttrChangeProcess 
 *	
 *	@param AttrChangeNum - 变化的属性值编号
 *
 *	@return SUCCESS 处理完成
 *					FAILURE 处理异常
 */
uint8_t AttrChangeProcess (uint8_t AttrChangeNum)
{
	uint8_t ret = SUCCESS;
	uint8_t *pValue;

	switch(AttrChangeNum)
	{
		case SAMPLING: 
			App_GetAttr(SAMPLING,pValue); //获取属性值
			if(*pValue == 1)
			{
				ERR_LED1_ON;
			}else
			 ERR_LED2_ON;
	}
}

	
/*  ============================ 时间戳服务 ==============================
 */ 
/*! @brief	时间戳服务 
 *					本服务产生数据包时间增量时间戳 int64 精度1ns 
 */
//void TSG_TIM5_Init(void);	
//		
//void Timestamp_Service_Init(void)
//{
//	//pointer Init
//	pcCurTimeStamp = (char*)&CurTimeStamp;
//	
//	TSG_TIM5_Init();
//}
//	
//		
//uint32_t Timestamp_Service_GetTimestamp(void)
//{

//}

//// TimeStamp Generator 
//void TSG_TIM5_Init(void)
//{
//	LL_TIM_InitTypeDef TIM_InitStruct;

//	
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
//	
//  TIM_InitStruct.Prescaler = 800;      // 80MHz APB1_TIM 10us/1Count
//  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
//  TIM_InitStruct.Autoreload = 0xFFFFFFFF;
//  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//  LL_TIM_Init(TIM5, &TIM_InitStruct);

//  LL_TIM_DisableMasterSlaveMode(TIM5);
//	
//	TIM5->CNT = 0;
//	LL_TIM_EnableCounter(TIM5);
//}

