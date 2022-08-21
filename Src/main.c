/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "shell.h"
#include "ads1299.h"
#include "w5500_service.h"
#include "AttritubeTable.h"
#include "protocol_ethernet.h"
#include "SimpleInsQueue.h"
#include "MicroEEG_Misc.h"
#include "imp_meas.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Shell
#define Shell_UART USART1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CAN_HandleTypeDef hcan1;

QSPI_HandleTypeDef hqspi;
DMA_HandleTypeDef hdma_quadspi;

/* USER CODE BEGIN PV */
uint32_t SYS_Event;							//!< 系统状态事件
static InsQUEUE InsQueue;				//!< TCP指令队列 -  存放变化的属性编号
uint32_t TriggerTimeStamp;			//!< 标签事件发生时点

SHELL_TypeDef shell;						//!< shell句柄
extern uint8_t UDP_DTx_Buff[UDPD_Tx_Buff_Size];//!< ADS1299结果缓存区（shell调试用）
uint8_t ReadResult;             //!< shell调试用

uint8_t chx_process=0;   //!< 正在阻抗检测的通道编号,本版本为多片一起检测，即编号范围0~7
uint32_t cnt=0; //for debug
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */


static void AttrChangeCB(uint8_t AttrNum); //!< 属性值变化回调函数	
static void Sys_Control(); //!< 系统状态控制器

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

////////////////////////////////////////////////////////////////////////////////
/*!		@fn shell_XXX	  
 *		@brief	fuction for shell
 */
int fputc(int ch, FILE* stream)
{

	while(!LL_USART_IsActiveFlag_TXE(Shell_UART))
	{}
	LL_USART_TransmitData8(Shell_UART, ch);
  return ch;
}

void ShellPutchar(const char ch)
{
	while(!LL_USART_IsActiveFlag_TXE(Shell_UART))
	{}
	LL_USART_TransmitData8(Shell_UART, ch);
}

signed char ShellGetchar(char* ch)
{
	if(LL_USART_IsActiveFlag_RXNE(Shell_UART))
	{
		*ch = Shell_UART->DR;
		return 0;
	}
	else
	{
		return -1;
	}
}
////////////////////////////////////////////////////////////////////////////////
/*!		@fn ADC_XXX	  
 *		@brief	test functions for ADC
 */
void ADC_StartAcq(void)
{
	Mod_DRDY_INT_Enable(0);
	
	ADS1299_SendCommand(ADS1299_CMD_START);
	ADS1299_SendCommand(ADS1299_CMD_RDATAC);	
	
	printf("Start Acquision!\r\n");
}
SHELL_EXPORT_CMD(ADC_StartAcq, ADC_StartAcq,ADC start to sample);
	
void ADC_StopAcq(void)
{
	Mod_DRDY_INT_Disable(0);
	
	ADS1299_SendCommand(ADS1299_CMD_STOP);
	ADS1299_SendCommand(ADS1299_CMD_SDATAC);
	printf("Stop Acquision!\r\n");
}
SHELL_EXPORT_CMD(ADC_StopAcq, ADC_StopAcq,ADC stop to sample);

void ADC_SendCommand(unsigned char cmd)
{
	ADS1299_SendCommand(cmd);
	
	printf("Command %x SEND!\r\n",cmd);
}
SHELL_EXPORT_CMD(ADC_SendCommand, ADC_SendCommand, Send IMD Instruction);

void ADC_WriteReg(unsigned char chip,unsigned char offset,unsigned char data)
{
	int chip1val;
	
	if(chip!=1){
		// read chip1's reg value first and backup
		chip1val = ADS1299_ReadREG(1,offset);
		// change the specific chip's reg value
		ADS1299_WriteREG(chip,offset,data);
		// recovery the chip1 value
		ADS1299_WriteREG(1,offset,chip1val);
	}
	else
	ADS1299_WriteREG(chip,offset,data);
}
SHELL_EXPORT_CMD(ADC_WriteReg, ADC_WriteReg, Write ADS1299 Register);


void ADC_ChannelConfig(unsigned char chip,unsigned char channel,unsigned char sw)
{
	int chip1val;
	
	if(chip!=1){
		// read chip1's reg value first and backup
		chip1val = ADS1299_ReadREG(1,ADS1299_REG_CH1SET + channel);
		// change the specific chip's reg value
		ADS1299_Channel_Control(chip,channel,sw);
		// recovery the chip1 value
		ADS1299_WriteREG(1,ADS1299_REG_CH1SET + channel,chip1val);
	}
	else
	ADS1299_Channel_Control(chip,channel,sw);
}
SHELL_EXPORT_CMD(ADC_ChannelConfig, ADC_ChannelConfig, Channel Switch:0-close 1-open);


void ADC_SetMode(unsigned char mode)
{
	ADS1299_Mode_Config(mode);
	printf("ADC Mode %d is set!\r\n",mode);
}
SHELL_EXPORT_CMD(ADC_SetMode, ADC_SetMode, Set ADC mode:1-EEG_Acq 2-IMP_Meas 3-TEST);

void ladreg(unsigned char chip)
{
	uint8_t i;
	uint8_t ReadResult;
	
	printf("1299's Registers Values:\r\n");
	
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_DEVID);	
	printf("DEVID=%#x\r\n",ReadResult);
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_CONFIG1);	
	printf("CONF1=%#x\r\n",ReadResult);
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_CONFIG2);	
	printf("CONF2=%#x\r\n",ReadResult);
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_CONFIG3);	
	printf("CONF3=%#x\r\n",ReadResult);
	
	for(i=0;i<8;i++)
	{
		ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_CH1SET+i);	
		printf("CH%d=%#x\r\n",i,ReadResult);	
	}
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_BIASSENSP);	
	printf("BIASSENSP=%#x\r\n",ReadResult);
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_BIASSENSN);	
	printf("BIASSENSN=%#x\r\n",ReadResult);
	
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_LOFFSENSP);	
	printf("LOFFSENSP=%#x\r\n",ReadResult);
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_LOFFSENSN);	
	printf("LOFFSENSN=%#x\r\n",ReadResult);
	
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_LOFFFLIP);	
	printf("LOFFFLIP=%#x\r\n",ReadResult);
	
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_MISC1);	
	printf("MISC1=%#x\r\n",ReadResult);
	
	ReadResult = ADS1299_ReadREG(chip,ADS1299_REG_CONFIG4);	
	printf("CONF4=%#x\r\n",ReadResult);
}
SHELL_EXPORT_CMD(ladreg, ladreg, Read&List ADC Registers);

void checkbuffer(unsigned char offset)
{	
	int i,j;
	int chipNum = CHANNEL_NUM/8;
	for(i=0 ; i<chipNum; i++){
		for(j=0; j<27; j++){
			printf("%x ",UDP_DTx_Buff[HEAD_SIZE+7+j]);
		}
		printf("\r\n");
	}
	printf("\r\n");
}
SHELL_EXPORT_CMD(checkbuffer, checkbuffer, Print buffer of ADC Sample Value);
////////////////////////////////////////////////////////////////////////////////

/*!		@fn AttrChangeCB
 *
 *		@param	AttrNum 变化的属性编号
 *
 *		@brief	属性值变化回调
 */
static void AttrChangeCB(uint8_t AttrNum)
{
	EnQueue(&InsQueue,AttrNum);			//!< 变化的属性编号入队等待系统状态控制器处理

	SYS_Event |= ATTR_CHANGE_EVT;   //!< 更新事件：属性值变化
}

/*!		@fn Sys_Control	  
 *		@brief	系统状态控制器 - 事件驱动
 */
static void Sys_Control()
{
	uint8_t AttrChangeNum;
	uint8_t TCPstate,UDPstate;

	
	// TCP控制通道事件
	TCPstate = TCPServer_Service(0,SYS_Event);
	
	if ( TCPstate == TCP_RECV )
	{ 
		SYS_Event |= TCP_RECV_EVT;	//!< 更新事件：TCP端口接收一帧 -> 跳转TCP帧协议服务  
	}
	else if ( TCPstate == TCP_SEND )
	{ 
		SYS_Event |= TCP_SEND_EVT;	//!< 更新事件：TCP端口回复完成
		
		SYS_Event &= ~TCP_PROCESSCLP_EVT; //!< 清除前序事件 - TCP帧协议服务
	}
	else if (TCPstate == Sn_CLOSED )//!< 保护措施: TCP端口断开，则停止采样
	{
		App_WriteAttr(SAMPLING, CHX_NONE, SAMPLLE_STOP); //!< 修改开始采样属性值
		AttrChangeProcess(SAMPLING); //!< 执行停止采样操作	
	}
	
	if( SYS_Event & TCP_RECV_EVT ) 	 
	{
		if( TCP_ProcessFSM() == _FSM_CPL_)	//!< 更新事件：TCP帧协议服务处理完毕 -> 跳转TCP端口回复
		{			
			SYS_Event |= TCP_PROCESSCLP_EVT;

			SYS_Event &= ~TCP_RECV_EVT;	//!< 清除前序事件 - TCP端口接收一帧			
		}
	}
	
	// 属性变化事件
	if(( SYS_Event & TCP_SEND_EVT )&& ( SYS_Event & ATTR_CHANGE_EVT ))
	{
		AttrChangeNum=DeQueue(&InsQueue);	//!< 出队 - 变化的属性值
		AttrChangeProcess(AttrChangeNum); //!< 属性变化处理函数
		
		SYS_Event &= ~ATTR_CHANGE_EVT; //!< 清除前序事件 - 属性变化
		SYS_Event &= ~TCP_SEND_EVT; //!< 清除前序事件 - TCP回复完成			
	}
	else if(SYS_Event& TCP_SEND_EVT )
		SYS_Event &= ~TCP_SEND_EVT; //!< 清除前序事件 - TCP回复完成事件

	// UDP标签通道事件 
	if( SYS_Event & TRIGGER_EVT )
	{
		if(UDP_Service(2,SYS_Event) == UDP_RECV)		//!< UDP端口接收，清除接收中断标志位
		{
			SYS_Event |= UDP_RECV_EVT;	//!< 更新事件：UDP端口接收到一帧 -> 跳转UDP事件帧协议服务
			SYS_Event &= ~TRIGGER_EVT; //!< 清除前序事件 - 标签事件
		}
	}
	
	if( SYS_Event & UDP_RECV_EVT )
	{
		 if(UDP_TriggerProcess()==SUCCESS) //!< UDP事件帧协议服务处理完毕 -> 跳转UDP端口发送
		 {
			 SYS_Event |= UDP_TRGPROCESSCLP_EVT;	//!< 更新事件：UDP事件帧协议处理完毕
			 SYS_Event &= ~UDP_RECV_EVT; //!< 清除前序事件 - UDP端口接收到一帧
		 }			 
	}
		
	if( SYS_Event & UDP_TRGPROCESSCLP_EVT )
	{
			if(UDP_Service(2,SYS_Event) == UDP_SEND )
		{
			SYS_Event &= ~UDP_TRGPROCESSCLP_EVT; //!< 清除前序事件 - UDP事件帧协议处理完毕
		}
	}
	
	// UDP数据通道事件	
	if( SYS_Event & EEG_DATA_CPL_EVT )// 一包AD数据采集完成
	{
		if( UDP_DataProcess(0xFF,SYS_Event)== UDP_HEADER_CPL )
			{
				SYS_Event |=  UDP_DTPROCESSCLP_EVT; //!< 更新事件：UDP数据帧协议服务处理完毕
				SYS_Event &= ~ EEG_DATA_CPL_EVT; //!< 清除前序事件 - 一包AD数据采集完成
			}
	}
	
	if( SYS_Event & UDP_DTPROCESSCLP_EVT )// 一包AD数据打包完成
	{    
			if(	UDP_Service(1,SYS_Event)  == UDP_SEND ) 
		{
			SYS_Event &= ~ UDP_DTPROCESSCLP_EVT; //!< 清除前序事件 - UDP数据帧协议服务处理完毕	
			
			if( SYS_Event&EEG_STOP_EVT )	//!< 如果本UDP包发送之前发生过AD采集暂停事件
				SYS_Event &= ~EEG_STOP_EVT; //!< 清除前序事件 - AD数据暂停采集		
		}
	}
	
	// 阻抗检测事件
	if ( SYS_Event & EEG_IMP_MODE ) 
	{
		cnt++;
		if(cnt==8000){ //TODO 用定时器
		cnt=0;
		SYS_Event |= CHX_IMP_START; //!< 更新事件： 开始一通道阻抗值读取
		
		/* 发起一通道阻抗采集 */
		while( imp_control(chx_process) != CHX_IMP_CPL );
		chx_process++; 
		
		if( chx_process == 8 )
		{
			chx_process=0;
			
			//TODO 控制通道接收缓冲区写入 AC 03 01 CHX_IMP_VAL FF CC
			TCP_Rx_Buff[0] = 0xAC;
			TCP_Rx_Buff[1] = 0x03;
			TCP_Rx_Buff[2] = 0x01;
			TCP_Rx_Buff[3] = CHX_IMP_VAL;
			TCP_Rx_Buff[4] = 0xFF;
			TCP_Rx_Buff[5] = 0xCC;
			
			// 控制通道主动发送至上位机
			SYS_Event |= TCP_RECV_EVT;	//!< 更新事件：模拟接收一帧				

		}
		}
	}
	
	// 异常事件
	LED_Service(SYS_Event); //!< LED
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
		
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	
	//备份服务初始化
	BKP_Service_Init();		
	
	//LED 初始化	
	LED_Service_Init();
	
	//Shell 初始化
	shell.read = ShellGetchar;
	shell.write = ShellPutchar;
	shellInit(&shell);
		
	//ADS1299 初始化
	//	使能SPI	
	LL_SPI_Enable(SPI1);	
	//	配置DMA
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)&(SPI1->DR)); 	// SPI1_RX
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_3, (uint32_t)&(SPI1->DR)); 	// SPI1_TX
	//	复位ADS1299
	ADS1299_Init(0);
	ADS1299_Mode_Config(1);
	
	//样本时间戳服务初始化
	SampleTimestamp_Service_Init();
	
	//TCP帧协议服务初始化
	TCP_ProcessFSMInit();			
	
	//属性表服务初始化
	Attr_Tbl_Init();
		
	//主应用程序向属性表服务注册属性值变化回调函数
	Attr_Tbl_RegisterAppCBs(&AttrChangeCB);
	
	if( SYS_Event&POWERDOWN_EVT )
	{
		BKP_Service_Recovery();
		
		if(SYS_Event&SOCKETDOWN_EVT)
			W5500_Init();
	}
	else 
	{
		//W5500初始化
		W5500_Init(); //W5500初始化，配置Socket	

	}

	/* 开启独立看门狗 */
  //MX_IWDG_Init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// 喂狗
		//LL_IWDG_ReloadCounter(IWDG);
		
		// shell 服务
    shellTask(&shell);
		
		// 运行系统状态控制器
		Sys_Control();   
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_DisableOverDriveMode();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
  }
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_16, 320, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(160000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_3CYCLES);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PC12   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 3;
  hqspi.Init.FifoThreshold = 9;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 16;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC and set the Time and Date
  */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  LL_RTC_SetAsynchPrescaler(RTC, 127);
  LL_RTC_SetSynchPrescaler(RTC, 255);
  /* USER CODE BEGIN RTC_Init 2 */
	// !!!!! CubeMX LL生成RTC初始化时间没有RTC_DateStruct.Day
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_3, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_NOINCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  /* SPI1_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_3);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = Mod5_SCK_Pin|Mod5_MISO_Pin|Mod5_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 4294967295;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration
  PA3   ------> TIM2_CH4
  PA5   ------> TIM2_CH1
  */
  GPIO_InitStruct.Pin = Label_TGR_In1_Pin|Label_TGR_Out_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOE, ERR_LED2_Pin|ERR_LED1_Pin|ACQ_LED2_Pin|ACQ_LED1_Pin
                          |PWR_LED1_Pin|EXTM_IO0_Pin|Mod_nPWDN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, PWR_LED2_Pin|nChargeEN_Pin|TRGPWR_EN_Pin|CAN_S_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOD, Mod5_nCS_Pin|Mod5_START_Pin|Mod5_nRESET_Pin|Mod5_nPWDN_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CAN_D_GPIO_Port, CAN_D_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, Mod_START_Pin|Mod_nRESET_Pin);

  /**/
  LL_GPIO_SetOutputPin(W5500_nRST_GPIO_Port, W5500_nRST_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOD, Mod1_nCS_Pin|Mod2_nCS_Pin|Mod3_nCS_Pin|Mod4_nCS_Pin);

  /**/
  GPIO_InitStruct.Pin = ERR_LED2_Pin|ERR_LED1_Pin|ACQ_LED2_Pin|ACQ_LED1_Pin
                          |PWR_LED1_Pin|EXTM_IO0_Pin|Mod_nPWDN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PWR_LED2_Pin|nChargeEN_Pin|TRGPWR_EN_Pin|CAN_S_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PGau_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(PGau_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = KeyPad_IO1_Pin|KeyPad_IO0_Pin|EXTM_IO3_Pin|EXTM_IO2_Pin
                          |EXTM_IO1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Mod5_nCS_Pin|Mod5_START_Pin|Mod5_nRESET_Pin|Mod5_nPWDN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CAN_D_Pin|W5500_nRST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Mod1_nCS_Pin|Mod2_nCS_Pin|Mod3_nCS_Pin|Mod4_nCS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Mod_START_Pin|Mod_nRESET_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE6);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE4);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE7);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE1);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_6;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(Label_TGR_In2_GPIO_Port, Label_TGR_In2_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(W5500_nINT_GPIO_Port, W5500_nINT_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(Modc_nDRDY_GPIO_Port, Modc_nDRDY_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(Mod1_nDRDY_GPIO_Port, Mod1_nDRDY_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(Mods_nDRDY_GPIO_Port, Mods_nDRDY_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(Label_TGR_In2_GPIO_Port, Label_TGR_In2_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(W5500_nINT_GPIO_Port, W5500_nINT_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(Modc_nDRDY_GPIO_Port, Modc_nDRDY_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(Mod1_nDRDY_GPIO_Port, Mod1_nDRDY_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(Mods_nDRDY_GPIO_Port, Mods_nDRDY_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
