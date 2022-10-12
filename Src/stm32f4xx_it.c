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
uint8_t SampleNum = 0 ;				//!< ����������
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
extern uint32_t CurTimeStamp[10];			//!< ��ǰʱ��
extern uint32_t TriggerTimeStamp;			//!< ��ǩ�¼�����ʱ��
extern uint8_t chx_process;						//!< ��ǰ�����ͨ��
extern uint8_t* pimp_sample;					//!< �迹���ԭʼ����ֵ
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
	
	TriggerTimeStamp = TIM5->CNT;//!< ��ȡ��ǰʱ��
	SYS_Event |= TRIGGER_EVT; //!< �����¼�����ǩ�¼�
	
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
	
	/* ------------------ �迹����ģʽ ------------------ */
	if( SYS_Event&EEG_IMP_MODE )
	{
		ADS1299_ReadResult(pimp_sample); //��ȡ����ֵ
		
		//��ȡ���ڴ����ͨ������ֵ
		int chip = 0;
		for( chip=0; chip < CHANNEL_NUM/8; chip++) {
			
			//��ȡͨ������ֵ
			curr_sample_val[chip][chx_process] = *(uint8_t*)(pimp_sample+3+3*chx_process+27*chip) << 16;
			curr_sample_val[chip][chx_process] |= *(uint8_t*)(pimp_sample+3+3*chx_process+27*chip +1) << 8;
			curr_sample_val[chip][chx_process] |= *(uint8_t*)(pimp_sample+3+3*chx_process+27*chip +2);
			
			//3�ֽڲ���ת4�ֽ�
			if( (curr_sample_val[chip][chx_process] & 0x00800000) !=0 ) { //���λΪ1
				curr_sample_val[chip][chx_process] |= 0xFF000000 ; 
			}
			
			//ȡͨ������ֵ���ֵ
			max_sample_val[chip][chx_process] = MAX(max_sample_val[chip][chx_process],curr_sample_val[chip][chx_process]);
		}
		
		SampleNum++;
		
		if( SampleNum == 25 ) //250sps����,7.8Hz�����ź� ���ٲɰ������ ����1/(7.8*2)*250 ����
		{	
			
			for( chip=0; chip < CHANNEL_NUM/8; chip++) {
				
				chx_imp_sample[chip][chx_process] = max_sample_val[chip][chx_process];// - min_sample_val[chip][chx_process];
				/* ��λ */
				max_sample_val[chip][chx_process] = 0; 
			}
			
			SampleNum = 0;
			
			SYS_Event |= CHX_IMP_DONE; //!< �����¼���һͨ���迹ֵ�Ѷ�ȡ��� -> ��ת�迹ֵת��
		}		
	}
	/* ------------------ �Ե����ģʽ ------------------ */
	else 
	{
		/* ����ʱ�����ȡ */
		if( SYS_Event&EEG_DATA_START_EVT ) //!< ÿ�ο�ʼ����ʱ�������������
		{
			SampleNum=0;
		}	
		
		CurTimeStamp[SampleNum] = TIM5->CNT;  //!< ��ȡ��ǰʱ��	
		
		/* �����ɼ������ */	
		SYS_Event |= EEG_DATA_ACQ_EVT; //!< �����¼���һ��AD���ݲɼ���
		SYS_Event &= ~EEG_DATA_START_EVT; //!< ���ǰ���¼� - һ��ad���ݿ�ʼ�ɼ�
		SYS_Event &= ~POWERDOWN_EVT; //!< ���ǰ���¼� - �豸�����쳣�ϵ�

		if( UDP_DataProcess(SampleNum,SYS_Event)== UDP_DATA_CPL ) //!< �Ե����������
		{
			SampleNum++; //!< �������+1
		}
		
		if( SampleNum == SAMPLENUM )
		{
			SampleNum=0; //!< ������Ź���
			
			SYS_Event |= EEG_DATA_CPL_EVT; //!< �����¼���һ��ad���ݲɼ���� -> ��תUDP֡Э�����			
			SYS_Event &= ~EEG_DATA_ACQ_EVT; //!< ���ǰ���¼� - һ��AD���ݲɼ���	
		}		
	}
	
	LED_Service(SYS_Event); //!< LEDָʾ
	
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
