/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
  
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> 

#include "w5500_service.h"
#include "socket.h"
#include "w5500.h"
#include "protocol_ethernet.h"
#include "AttritubeTable.h"
#include "MicroEEG_Misc.h"
#include "imp_meas.h"
#include "ads1299.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ABS(x) ( (x)>0?(x):-(x) )	
#define MAX(x,y) ( (x) > (y) ? (x) : (y))
#define MIN(x,y) ( (x) <= (y) ? (x) : (y))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t SampleNum = 0 ;				//!< 采样样本数
int32_t max_sample_val[CHANNEL_NUM/8][8]; //int!
int32_t curr_sample_val[CHANNEL_NUM/8][8]; //int!

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern uint32_t CurTimeStamp[10];			//!< 当前时间
extern uint32_t TriggerTimeStamp;			//!< 标签事件发生时点
extern uint8_t chx_process;						//!< 当前处理的通道
extern uint8_t* pimp_sample;					//!< 阻抗检测原始采样值
extern uint32_t chx_imp_sample[CHANNEL_NUM/8][8];

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	uint16_t recvsize=0;
	
	TriggerTimeStamp = TIM5->CNT;//!< 获取当前时间
	SYS_Event |= TRIGGER_EVT; //!< 更新事件：标签事件
	
  /* USER CODE END EXTI0_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    /* USER CODE BEGIN LL_EXTI_LINE_0 */
    /* USER CODE END LL_EXTI_LINE_0 */
  }
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	
	/* ------------------ 阻抗测量模式 ------------------ */
	if( SYS_Event&EEG_IMP_MODE )
	{
		ADS1299_ReadResult(pimp_sample); //读取采样值
		
		//提取正在处理的通道采样值
		int chip = 0;
		for( chip=0; chip < CHANNEL_NUM/8; chip++) {
			
			//提取通道采样值
			curr_sample_val[chip][chx_process] = *(uint8_t*)(pimp_sample+3+3*chx_process+27*chip) << 16;
			curr_sample_val[chip][chx_process] |= *(uint8_t*)(pimp_sample+3+3*chx_process+27*chip +1) << 8;
			curr_sample_val[chip][chx_process] |= *(uint8_t*)(pimp_sample+3+3*chx_process+27*chip +2);
			
			//3字节补码转4字节
			if( (curr_sample_val[chip][chx_process] & 0x00800000) !=0 ) { //最高位为1
				curr_sample_val[chip][chx_process] |= 0xFF000000 ; 
			}
			
			//取通道采样值最大值
			max_sample_val[chip][chx_process] = MAX(max_sample_val[chip][chx_process],curr_sample_val[chip][chx_process]);
		}
		
		SampleNum++;
		
		if( SampleNum == 25 ) //250sps采样,7.8Hz测试信号 至少采半个周期 即采1/(7.8*2)*250 个点
		{	
			
			for( chip=0; chip < CHANNEL_NUM/8; chip++) {
				
				chx_imp_sample[chip][chx_process] = max_sample_val[chip][chx_process];// - min_sample_val[chip][chx_process];
				/* 复位 */
				max_sample_val[chip][chx_process] = 0; 
			}
			
			SampleNum = 0;
			
			SYS_Event |= CHX_IMP_DONE; //!< 更新事件：一通道阻抗值已读取完毕 -> 跳转阻抗值转换
		}		
	}
	/* ------------------ 脑电采样模式 ------------------ */
	else 
	{
		/* 样本时间戳获取 */
		if( SYS_Event&EEG_DATA_START_EVT ) //!< 每次开始采样时对样本序号清零
		{
			SampleNum=0;
		}	
		
		CurTimeStamp[SampleNum] = TIM5->CNT;  //!< 获取当前时间	
		
		/* 样本采集及封包 */	
		SYS_Event |= EEG_DATA_ACQ_EVT; //!< 更新事件：一包AD数据采集中
		SYS_Event &= ~EEG_DATA_START_EVT; //!< 清除前序事件 - 一包ad数据开始采集
		SYS_Event &= ~POWERDOWN_EVT; //!< 清除前序事件 - 设备发生异常断电

		if( UDP_DataProcess(SampleNum,SYS_Event)== UDP_DATA_CPL ) //!< 对单个样本封包
		{
			SampleNum++; //!< 样本序号+1
		}
		
		if( SampleNum == SAMPLENUM )
		{
			SampleNum=0; //!< 样本序号归零
			
			SYS_Event |= EEG_DATA_CPL_EVT; //!< 更新事件：一包ad数据采集完成 -> 跳转UDP帧协议服务			
			SYS_Event &= ~EEG_DATA_ACQ_EVT; //!< 清除前序事件 - 一包AD数据采集中	
		}		
	}
	
	LED_Service(SYS_Event); //!< LED指示
	
  /* USER CODE END EXTI1_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    /* USER CODE BEGIN LL_EXTI_LINE_1 */

    /* USER CODE END LL_EXTI_LINE_1 */
  }
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
