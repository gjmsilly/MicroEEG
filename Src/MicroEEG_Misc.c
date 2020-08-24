#include "MicroEEG_Misc.h"


/* 
	 INIT -- LED Green OFF
	 STANDBY -- LED Green ON
	 ACQUIRING -- LED Green FLASH
	 BTCONNECTLOST -- LED Red ON/ Green OFF
*/

/* Private variables ----------------------------------------------------*/
uint8_t LED1FlashCounter;
uint8_t LED2FlashCounter;

uint32_t CurTimeStamp = 0;
char* pcCurTimeStamp;
uint16_t CurEventTag = 0;
char* pcCurEventTag = (char*)&CurEventTag;


uint8_t SYS_Status = 0;

uint8_t BT_USART_Mutex;
uint8_t BT_USART_Ready;


/*    LED Service    */

void LED_Service_Init(void)
{
	LED1_OFF;
	LED2_OFF;
	LED_COP_OFF;
	
	LED1FlashCounter = 0;
	LED2FlashCounter = 0;
}


void LED_Service_Process(void)
{	
	
	
	switch (SYS_Status)
  {
  	
		case SYS_STATUS_INIT:
		{
  		LED1_OFF;
			LED2_OFF;
			break;
		}
  	
		case SYS_STATUS_STANDBY:
		{
  		LED1_OFF;
			LED2_ON;
			break;
		}
		
		case SYS_STATUS_ACQUIRING:
		{

			if(LED1FlashCounter > 250)
			{
				LED2_TOGGLE;
				LED1FlashCounter = 0;
			}
			
			LED1_OFF;
			break;
		}
		
		case SYS_STATUS_BTCONNECTIONLOST:
		{
  		LED1_ON;
			LED2_OFF;
			break;
		}
		
		case SYS_STATUS_SPECIALMODE:
		{
  		if(LED1FlashCounter > 100)
			{
				LED1_TOGGLE;
				LED1FlashCounter = 0;
			}
			
			LED2_OFF;
			break;
		}
		
  	default:
  		break;
  }
	
}


inline void LED_Service_Process_INT(void)
{
	LED1FlashCounter++;
}


/*    BTModule Service    */


void BTModule_Service_Init(void)
{
	BT_USART_Ready = 0;
	BT_USART_Mutex = 0;
}

void BTModule_Service_Process(void)
{
	if(LL_GPIO_IsInputPinSet(BT_LINK_GPIO_Port,BT_LINK_Pin))
	{
		//BT_USART_Ready = 0;
		SYS_Status = SYS_STATUS_BTCONNECTIONLOST;
	}
	else
	{
		//BT_USART_Ready = 1;
		if(SYS_Status == SYS_STATUS_BTCONNECTIONLOST)
		{
			SYS_Status = SYS_STATUS_STANDBY;
		}
	}
	
}

	
	
/*    Timestamp Service    */

void TSG_TIM5_Init(void);	
	
	
void Timestamp_Service_Init(void)
{
	//pointer Init
	pcCurTimeStamp = (char*)&CurTimeStamp;

	
	TSG_TIM5_Init();
}
	
	
	
uint32_t Timestamp_Service_GetTimestamp(void)
{

}


// TimeStamp Generator 
void TSG_TIM5_Init(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct;

	
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
	
  TIM_InitStruct.Prescaler = 800;      // 80MHz APB1_TIM 10us/1Count
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFFFFFFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);

  LL_TIM_DisableMasterSlaveMode(TIM5);
	
	TIM5->CNT = 0;
	LL_TIM_EnableCounter(TIM5);
}

