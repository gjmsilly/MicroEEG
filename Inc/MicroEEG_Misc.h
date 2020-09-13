#ifndef __MICROEEG_MISC_H__
#define __MICROEEG_MISC_H__

/****************************************************************/
/* MicroEEG Misc Function                                       */
/*  1. LED Service                                              */
/*  2. Trigger Service                                          */
/*  3. BT_HCI Wireless Connection Service                       */
/*  4. Timestamp Service                                        */
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
#define SYS_STATUS_INIT                0
#define SYS_STATUS_STANDBY             1
#define SYS_STATUS_ACQUIRING           2
#define SYS_STATUS_BTCONNECTIONLOST    3
#define SYS_STATUS_SPECIALMODE         4
 
#define LED1_ON        LL_GPIO_ResetOutputPin(LED1_GPIO_Port,LED1_Pin);
#define LED2_ON        LL_GPIO_ResetOutputPin(LED2_GPIO_Port,LED2_Pin);
#define LED_COP_ON     LL_GPIO_ResetOutputPin(BT_RESET_GPIO_Port, BT_RESET_Pin);
#define LED1_OFF       LL_GPIO_SetOutputPin(LED1_GPIO_Port,LED1_Pin);
#define LED2_OFF       LL_GPIO_SetOutputPin(LED2_GPIO_Port,LED2_Pin);
#define LED_COP_OFF    LL_GPIO_SetOutputPin(BT_RESET_GPIO_Port, BT_RESET_Pin);
#define LED1_TOGGLE    LL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
#define LED2_TOGGLE    LL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
 
 /*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t SYS_Status;
extern uint8_t BT_USART_Mutex;

/**********************************************************************
 * FUNCTIONS
 */
/* LED Service */
void LED_Service_Init (void);
void LED_Service_Process(void);
void LED_Service_Process_INT(void);

/* Trigger Service */
void Trigger_Service_Init(void);
void Trigger_Service_Process(void);

/* BT_HCI Wireless Connection Service */
void BTModule_Service_Init(void);
void BTModule_Service_Process(void);

/* Timestamp Service */
void Timestamp_Service_Init(void);
uint32_t Timestamp_Service_GetTimestamp(void);

#endif   /* __MICROEEG_MISC_H__ */
