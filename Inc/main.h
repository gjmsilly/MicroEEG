/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rtc.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t SYS_Event;									//!< 系统状态事件 - @ref System events

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// System events
#define ATTR_CHANGE_EVT					( 1 << 0 )	//!< 属性值变化
#define TCP_RECV_EVT						( 1 << 1 ) 	//!< TCP端口接收到一帧
#define TCP_SEND_EVT            ( 1 << 2 )	//!< TCP端口回复完成
#define TCP_PROCESSCLP_EVT			( 1 << 3 ) 	//!< TCP帧协议处理完毕
#define EEG_DATA_START_EVT			( 1 << 4 )	//!< 一包AD数据开始采集
#define EEG_DATA_CPL_EVT				( 1 << 5 )	//!< 一包AD数据采集完成
#define UDP_PROCESSCLP_EVT			( 1 << 6 )	//!< UDP帧协议处理完毕
#define EEG_STOP_EVT						( 1 << 7 )	//!< AD数据暂停采集

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERR_LED2_Pin LL_GPIO_PIN_2
#define ERR_LED2_GPIO_Port GPIOE
#define ERR_LED1_Pin LL_GPIO_PIN_3
#define ERR_LED1_GPIO_Port GPIOE
#define ACQ_LED2_Pin LL_GPIO_PIN_4
#define ACQ_LED2_GPIO_Port GPIOE
#define ACQ_LED1_Pin LL_GPIO_PIN_5
#define ACQ_LED1_GPIO_Port GPIOE
#define PWR_LED1_Pin LL_GPIO_PIN_6
#define PWR_LED1_GPIO_Port GPIOE
#define PWR_LED2_Pin LL_GPIO_PIN_13
#define PWR_LED2_GPIO_Port GPIOC
#define Label_TGR_Out_Pin LL_GPIO_PIN_3
#define Label_TGR_Out_GPIO_Port GPIOA
#define Label_TGR_OutA5_Pin LL_GPIO_PIN_5
#define Label_TGR_OutA5_GPIO_Port GPIOA
#define KeyPad_IO7_Pin LL_GPIO_PIN_7
#define KeyPad_IO7_GPIO_Port GPIOA
#define KeyPad_IO6_Pin LL_GPIO_PIN_4
#define KeyPad_IO6_GPIO_Port GPIOC
#define KeyPad_IO5_Pin LL_GPIO_PIN_5
#define KeyPad_IO5_GPIO_Port GPIOC
#define KeyPad_IO4_Pin LL_GPIO_PIN_0
#define KeyPad_IO4_GPIO_Port GPIOB
#define KeyPad_IO3_Pin LL_GPIO_PIN_1
#define KeyPad_IO3_GPIO_Port GPIOB
#define KeyPad_IO2_Pin LL_GPIO_PIN_2
#define KeyPad_IO2_GPIO_Port GPIOB
#define KeyPad_IO1_Pin LL_GPIO_PIN_7
#define KeyPad_IO1_GPIO_Port GPIOE
#define KeyPad_IO0_Pin LL_GPIO_PIN_8
#define KeyPad_IO0_GPIO_Port GPIOE
#define EXTM_IO3_Pin LL_GPIO_PIN_12
#define EXTM_IO3_GPIO_Port GPIOE
#define EXTM_IO2_Pin LL_GPIO_PIN_13
#define EXTM_IO2_GPIO_Port GPIOE
#define EXTM_IO1_Pin LL_GPIO_PIN_14
#define EXTM_IO1_GPIO_Port GPIOE
#define EXTM_IO0_Pin LL_GPIO_PIN_15
#define EXTM_IO0_GPIO_Port GPIOE
#define Mod5_SCK_Pin LL_GPIO_PIN_13
#define Mod5_SCK_GPIO_Port GPIOB
#define Mod5_MISO_Pin LL_GPIO_PIN_14
#define Mod5_MISO_GPIO_Port GPIOB
#define Mod5_MOSI_Pin LL_GPIO_PIN_15
#define Mod5_MOSI_GPIO_Port GPIOB
#define Mod5_nCS_Pin LL_GPIO_PIN_8
#define Mod5_nCS_GPIO_Port GPIOD
#define Mod5_START_Pin LL_GPIO_PIN_9
#define Mod5_START_GPIO_Port GPIOD
#define Mod5_nRESET_Pin LL_GPIO_PIN_10
#define Mod5_nRESET_GPIO_Port GPIOD
#define Mod5_nPWDN_Pin LL_GPIO_PIN_11
#define Mod5_nPWDN_GPIO_Port GPIOD
#define Mod5_nDRDY_Pin LL_GPIO_PIN_6
#define Mod5_nDRDY_GPIO_Port GPIOC
#define TRGPWR_EN_Pin LL_GPIO_PIN_7
#define TRGPWR_EN_GPIO_Port GPIOC
#define CAN_S_Pin LL_GPIO_PIN_8
#define CAN_S_GPIO_Port GPIOC
#define W5500_MOSI_Pin LL_GPIO_PIN_9
#define W5500_MOSI_GPIO_Port GPIOC
#define CAN_D_Pin LL_GPIO_PIN_8
#define CAN_D_GPIO_Port GPIOA
#define W5500_nRST_Pin LL_GPIO_PIN_15
#define W5500_nRST_GPIO_Port GPIOA
#define W5500_MISO_Pin LL_GPIO_PIN_10
#define W5500_MISO_GPIO_Port GPIOC
#define W5500_nINT_Pin LL_GPIO_PIN_0
#define W5500_nINT_GPIO_Port GPIOD
#define W5500_SCLK_Pin LL_GPIO_PIN_3
#define W5500_SCLK_GPIO_Port GPIOD
#define Modc_nDRDY_Pin LL_GPIO_PIN_4
#define Modc_nDRDY_GPIO_Port GPIOD
#define Mod1_nDRDY_Pin LL_GPIO_PIN_7
#define Mod1_nDRDY_GPIO_Port GPIOD
#define W5500_nCS_Pin LL_GPIO_PIN_6
#define W5500_nCS_GPIO_Port GPIOB
#define Mod_nCS_Pin LL_GPIO_PIN_7
#define Mod_nCS_GPIO_Port GPIOB
#define Mod_START_Pin LL_GPIO_PIN_8
#define Mod_START_GPIO_Port GPIOB
#define Mod_nRESET_Pin LL_GPIO_PIN_9
#define Mod_nRESET_GPIO_Port GPIOB
#define Mod_nPWDN_Pin LL_GPIO_PIN_0
#define Mod_nPWDN_GPIO_Port GPIOE
#define Mods_nDRDY_Pin LL_GPIO_PIN_1
#define Mods_nDRDY_GPIO_Port GPIOE
#define Mods_nDRDY_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

#define ERR_LED1_OFF  LL_GPIO_SetOutputPin  (GPIOE, ERR_LED1_Pin);
#define ERR_LED1_ON   LL_GPIO_ResetOutputPin(GPIOE, ERR_LED1_Pin);
#define ERR_LED2_OFF  LL_GPIO_SetOutputPin  (GPIOE, ERR_LED2_Pin);
#define ERR_LED2_ON   LL_GPIO_ResetOutputPin(GPIOE, ERR_LED2_Pin);
#define ACQ_LED1_OFF  LL_GPIO_SetOutputPin  (GPIOE, ACQ_LED1_Pin);
#define ACQ_LED1_ON   LL_GPIO_ResetOutputPin(GPIOE, ACQ_LED1_Pin);
#define ACQ_LED2_OFF  LL_GPIO_SetOutputPin  (GPIOE, ACQ_LED2_Pin);
#define ACQ_LED2_ON   LL_GPIO_ResetOutputPin(GPIOE, ACQ_LED2_Pin);
#define PWR_LED1_OFF  LL_GPIO_SetOutputPin  (GPIOE, PWR_LED1_Pin);
#define PWR_LED1_ON   LL_GPIO_ResetOutputPin(GPIOE, PWR_LED1_Pin);
#define PWR_LED2_OFF  LL_GPIO_SetOutputPin  (GPIOC, PWR_LED2_Pin);
#define PWR_LED2_ON   LL_GPIO_ResetOutputPin(GPIOC, PWR_LED2_Pin);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
