/**
 * @file    ads1299.c
 * @author  LiuMiao, gjmsilly
 * @brief   ads1299 driver for TUN-M1-CPUF446
 * @version 1.0.0
 * @date    2022-07-01
 *
 * @copyright (c) 2022 gjmsilly
 *
 */

/************************************************************************
 * INCLUDES
 */
#include "ads1299.h"
#include <stdlib.h>

/************************************************************************
 * GLOBAL VARIABLES
 */
uint8_t DummyByte=0x00;

/************************************************************************
 * LOCAL FUNCTIONS
 */
static void WaitUs(int iWaitUs);
static void ADS1299_Reset();
static void ADS1299_PowerOn();

/************************************************************************
 * FUNCTIONS
 */
 
/*!
 *  @fn     WaitUs
 *
 *  @brief  延时（非精准延时）
 *	
 *  @param  iWaitUs - 延时时间，单位us
 */
static void WaitUs(int iWaitUs)
{
    int iPreTickVal = SysTick -> VAL;  
	  int iCounterTargetValue;
	  int iCounterTargetInterval;
	   
	  iCounterTargetInterval = iWaitUs * (HAL_RCC_GetHCLKFreq()/1000000)-32;
	  if(iPreTickVal < iCounterTargetInterval)
	  {
			iCounterTargetValue =  iPreTickVal + SysTick->LOAD - iCounterTargetInterval;
		}
		else
		{
			iCounterTargetValue =  iPreTickVal - iCounterTargetInterval;
		}
		
		while(SysTick -> VAL >= iCounterTargetValue);
	
}
/*!
 *  @fn     ADS1299_Reset
 *
 *  @brief  ADS1299 复位
 */

static void ADS1299_Reset()
{
	Mod_RESET_L; // issue reset pulse
	WaitUs(6);  // wait at least 2tclk
	Mod_RESET_H;
	Mod_CS_Disable;
	WaitUs(40);  // wait for 18 tclk then start using device
}

/*!
 *  @fn     ADS1299_PowerOn
 *
 *  @brief  ADS1299 上电
 *
 *  ADS1299上电需满足Power-Up Sequencing，即上电之前所有输入引脚保持低电平。
 *  上电之后，等待tPOR(约128ms)后再进行复位时序。
 */
static void ADS1299_PowerOn()
{
	Mod_PDWN_H;  // set PDWN=1
	Mod_RESET_H;  // set RESET=1
	HAL_Delay(200);  // wait for at least tPOR = 128ms for POR
}

/*!
 *  @fn     ADS1299_Init
 *
 *  @brief  ADS1299 初始化
 *
 *  根据TUN-M1主板上搭载的ADS1299片数选择数据就绪的中断源DRDY，即当设备通道数
 *  为32时，认为主板上包含4片1299模块，这种情况下，为了保证数据就绪中断时，所
 *  有模块的数据均就绪，选择Mods_nDRDY（EXTI_1）作为中断源。否则，主板ADS1299
 *  模块片数不完整，选择Mod1_nDRDY（EXTI_7）作为中断源。
 */
void ADS1299_Init()
{
	/* 初始化与主控芯片的通信接口 */
	LL_SPI_Enable(SPI_Handle); //SPI
	LL_DMA_SetPeriphAddress(DMA_Handle, DMA_RX_STREAM, (uint32_t)&(SPI_Handle->DR)); //DMA
	LL_DMA_SetPeriphAddress(DMA_Handle, DMA_TX_STREAM, (uint32_t)&(SPI_Handle->DR)); 
	
	/* 初始化成功后DRDY会翻转，先禁止该中断源 */
  #ifdef Dev_Ch32
  Mod_DRDY_INT_Disable(0);
  #else
  Mod_DRDY_INT_Disable(1);
  #endif
	/* 初始化ADS1299 */
  ADS1299_PowerOn();
	ADS1299_Reset();	
  //Device wakes up in RDATAC Mode, send SDATAC cmd
  ADS1299_SendCommand(ADS1299_CMD_SDATAC);

}

/*!
 *  @fn     ADS1299_SendCommand
 *
 *  @brief  ADS1299 发送指令
 *
 *  本版本中，指令默认发送给所有ADS1299模块，暂不支持指定模块的指令发送。
 *
 *  @param  command - 指令
 */
void ADS1299_SendCommand(uint8_t command)
{
	__disable_irq();
	Mod_CS_Enable(0);
	WaitUs(4);
	
	while(!LL_SPI_IsActiveFlag_TXE(SPI_Handle));
	SPI_Handle->DR = command;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI_Handle));
	command = SPI_Handle->DR;
	while(LL_SPI_IsActiveFlag_BSY(SPI_Handle));
	WaitUs(4); // Delay time, final SCLK falling edge to CS high
	
	Mod_CS_Disable;
	WaitUs(4); // pulse duration before CS pull high
	__enable_irq();
}

/*!
 *  @fn     ADS1299_WriteREG
 *
 *  @brief  ADS1299 写寄存器
 *
 *  对指定的ADS1299模块指定寄存器写入新值。
 *
 *  @param  chip - ADS1299模块编号 1-4:模块1-4 0:所有模块
 *  @param  address - 寄存器地址
 *  @param  value - 寄存器待写入值
 */
 void ADS1299_WriteREG (uint8_t chip, uint8_t address, uint8_t value)
{
	
	address += 0x40;			
	
	__disable_irq();
	Mod_CS_Enable(chip);

	WaitUs(4);	
	while(!LL_SPI_IsActiveFlag_TXE(SPI_Handle));
	SPI_Handle->DR = address;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI_Handle));
	address = SPI_Handle->DR;
	WaitUs(4);  // 2us Command decode time 
	while(!LL_SPI_IsActiveFlag_TXE(SPI_Handle));
	SPI_Handle->DR = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI_Handle));
	address = SPI_Handle->DR;
	WaitUs(4);  // 2us Command decode time
	while(!LL_SPI_IsActiveFlag_TXE(SPI_Handle));
	SPI_Handle->DR = value;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI_Handle));
	address = SPI_Handle->DR;
	while(LL_SPI_IsActiveFlag_BSY(SPI_Handle));
	
	WaitUs(5);  // Delay time, final SCLK falling edge to CS high
	Mod_CS_Disable;																									
	__enable_irq();
}

/*!
 *  @fn     ADS1299_ReadREG
 *
 *  @brief  ADS1299 写寄存器
 *
 *  对指定的ADS1299模块指定寄存器值读取。
 *  [WARNING!]当硬件采取菊花链连接方式时，读取值均为模块1的值。
 *
 *  @param  chip - ADS1299模块编号
 *  @param  address - 寄存器地址
 *
 *  @return DRchar - 一字节寄存器值
 */
uint8_t ADS1299_ReadREG (uint8_t chip, uint8_t address)
{
	address += 0x20;
	
	uint8_t DRchar;	
		
	__disable_irq();
	Mod_CS_Enable(chip);
	
	WaitUs(5);	
	DRchar = SPI_Handle->DR;
	while(!LL_SPI_IsActiveFlag_TXE(SPI_Handle));
	SPI_Handle->DR = address;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI_Handle));
	DRchar = SPI_Handle->DR;
	WaitUs(4);  // 2us Command decode time
	while(!LL_SPI_IsActiveFlag_TXE(SPI_Handle));
	SPI_Handle->DR = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI_Handle));
	DRchar = SPI_Handle->DR;
	WaitUs(4);  // 2us Command decode time
	while(!LL_SPI_IsActiveFlag_TXE(SPI_Handle));
	SPI_Handle->DR = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI_Handle));
	while(LL_SPI_IsActiveFlag_BSY(SPI_Handle));
	DRchar = SPI_Handle->DR;
	
	WaitUs(5);  // Delay time, final SCLK falling edge to CS high
	Mod_CS_Disable;
	__enable_irq();

  return DRchar;
}

/*!
 *  @fn     ADS1299_Channel_Config
 *
 *  @brief  ADS1299 通道配置
 *
 *  @param  chip - ADS1299模块编号 1-4:模块1-4 0:所有模块
 *  @param  channel - 通道编号
 *  @param  Para - 通道参数
 */
void ADS1299_Channel_Config(uint8_t chip, uint8_t channel, TADS1299CHnSET Para)
{
	ADS1299_WriteREG (chip, (ADS1299_REG_CH1SET + channel), Para.value );
}


//TODO inline 报错
static void ADS1299_ReadResult_DMA(uint32_t DataHeadAddress, uint8_t DataLength)
{
	
	//	Configure the DMA channel for  SPI1_RX 
	LL_DMA_SetDataLength(DMA_Handle, DMA_RX_STREAM, DataLength);
	LL_DMA_SetMemoryAddress(DMA_Handle, DMA_RX_STREAM, DataHeadAddress);	
	LL_DMA_EnableStream(DMA_Handle, DMA_RX_STREAM);
	LL_DMA_DisableIT_TC(DMA_Handle, DMA_RX_STREAM);
	LL_SPI_EnableDMAReq_RX(SPI_Handle);
	
	//	Configure the DMA channel for  SPI1_TX 
	LL_DMA_SetDataLength(DMA_Handle, DMA_TX_STREAM, DataLength);
	LL_DMA_SetMemoryAddress(DMA_Handle, DMA_TX_STREAM, (uint32_t)&DummyByte);	
	LL_DMA_EnableStream(DMA_Handle, DMA_TX_STREAM);
	LL_DMA_DisableIT_TC(DMA_Handle, DMA_TX_STREAM);
	LL_SPI_EnableDMAReq_TX(SPI_Handle);
	
}

/*!
 *  @fn     ADS1299_ReadResult
 *
 *  @brief  ADS1299 采样数据读取
 *
 *  @param  pret - 数据读取内存首地址
 */
void ADS1299_ReadResult(uint8_t *pret)
{
  Mod_CS_Enable(0);
	WaitUs(4); 
	
	//DMA
	#ifdef Dev_Ch32 
	//!< DMA Read Bug: SPI 20MHz 
	ADS1299_ReadResult_DMA((uint32_t)pret, 108); 
	#endif
	#ifdef Dev_Ch24 
	ADS1299_ReadResult_DMA((uint32_t)pret, 81);
	#endif
	#ifdef Dev_Ch16 
	ADS1299_ReadResult_DMA((uint32_t)pret, 54);
	#endif
	#ifdef Dev_Ch8 
	ADS1299_ReadResult_DMA((uint32_t)pret, 27);
	#endif
	
	while(!LL_DMA_IsActiveFlag_TC0(DMA_Handle)); // Wait until all data trasferred from SPI1_RX
	
	LL_SPI_DisableDMAReq_TX(SPI_Handle);
	LL_SPI_DisableDMAReq_RX(SPI_Handle);
	
	LL_DMA_ClearFlag_TC3(DMA_Handle);
	LL_DMA_ClearFlag_TC0(DMA_Handle);
}


/*!
 *  @fn     ADS1299_Mode_Config
 *
 *  @brief  ADS1299 快速模式配置
 *
 *  @param  Mode - 模式 1:采样模式 2:阻抗检测模式 4:测试模式
 *
 */
void ADS1299_Mode_Config(uint8_t Mode)
{
	uint8_t i;
	uint8_t ReadResult;
	
	switch (Mode)
  {
		case ADS1299_ParaGroup_ACQ://EEG_Acq
		{
			ADS1299_WriteREG(1,ADS1299_REG_CONFIG1,0x96); //默认 250sps
			ADS1299_WriteREG(1,ADS1299_REG_CONFIG2,0xC0); //外部输入
			ADS1299_WriteREG(1,ADS1299_REG_CONFIG3,0xEC); 
			
			
			ADS1299_WriteREG(1,ADS1299_REG_LOFF,0x00);
			ADS1299_WriteREG(1,ADS1299_REG_LOFFSENSN,0x00);
			ADS1299_WriteREG(1,ADS1299_REG_LOFFSENSP,0x00);
			ADS1299_WriteREG(1,ADS1299_REG_BIASSENSN,0x00);
			ADS1299_WriteREG(1,ADS1299_REG_BIASSENSP,0xFF); //偏置全部接入
			ADS1299_WriteREG(1,ADS1299_REG_MISC1,0x20); // SRB1关闭
			
			for(i=0;i<8;i++)
			{
				// 默认所有模块增益一致 = 24
				do
					{
					ADS1299_WriteREG(1,ADS1299_REG_CH1SET+i,0x60);
					WaitUs(2);

					ReadResult = ADS1299_ReadREG(1,ADS1299_REG_CH1SET+i);
					WaitUs(10);
					}
						while(ReadResult!=0x60);
			}
			break;
		}
		
		case ADS1299_ParaGroup_IMP://IMP_Meas //TODO!
		{
			ADS1299_WriteREG(0,ADS1299_REG_LOFF,0x09);				//[3:2]=00(6nA),01(24nA),10(6uA),11(24uA); [1:0]=01(7.8Hz),10(31.2Hz)
			ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSN,0xFF);
			ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSP,0xFF);
			ADS1299_WriteREG(0,ADS1299_REG_BIASSENSN,0x00);
			ADS1299_WriteREG(0,ADS1299_REG_BIASSENSP,0x00);
			
			for(i=0;i<8;i++)
			{
				// Gain = 1
				ADS1299_WriteREG(0,ADS1299_REG_CH1SET+i,0x00);
				WaitUs(10);
			}			
			break;
		}
		
		case ADS1299_ParaGroup_TSIG://
		break;
   }
}

/*!
 *  @fn     ADS1299_Sampling_Control
 *
 *  @brief  ADS1299 采样开关设置
 *
 *  @param  Sampling - 0:停止采样 1：开始采样
 *
 *  @return EXIT_SUCCESS - 采样率设置成功
 *          EXIT_FAILURE - 采样率设置失败
 */
void ADS1299_Sampling_Control(uint8_t Sampling)
{
    switch(Sampling)
    {
        case 0:
            ADS1299_SendCommand(ADS1299_CMD_STOP);
            ADS1299_SendCommand(ADS1299_CMD_SDATAC);
				
						#ifdef Dev_Ch32
						Mod_DRDY_INT_Disable(0);
						#else
						Mod_DRDY_INT_Disable(1);
						#endif
        
						Mod_CS_Disable;
        break;

        case 1:
            ADS1299_SendCommand(ADS1299_CMD_START);
            ADS1299_SendCommand(ADS1299_CMD_RDATAC);
				
						#ifdef Dev_Ch32
						Mod_DRDY_INT_Enable(0);
						#else
						Mod_DRDY_INT_Enable(1);
						#endif
				
            Mod_CS_Enable(0);
        break;

    }
}

/*!
 *  @fn     ADS1299_SetSamplerate
 *
 *  @brief  ADS1299 采样率设置
 *
 *  本版本对采样率的设置为全局设置，即所有模块所有通道采样率一致
 *  [WARNING!]当硬件采取菊花链连接方式时，所有模块的采样率必须保
 *  持一致！
 *
 *  @param  chip - ADS1299模块编号，本版本不支持单模块采样率设置
 *  @param  Samplerate - 待设置的采样率
 *
 *  @return EXIT_SUCCESS - 采样率设置成功
 *          EXIT_FAILURE - 采样率设置失败
 */
bool ADS1299_SetSamplerate(uint8_t chip, uint16_t Samplerate)
{
    bool ret = EXIT_SUCCESS;
    uint8_t valset = 0;
    uint8_t valget = 0;

    switch(Samplerate)
    {
        case 250:
            valset = 0x96;
        break;

        case 500:
            valset = 0x95;
        break;

        case 1000:
            valset = 0x94;
        break;

        case 2000:
            valset = 0x93;
        break;

        default:
            valset = 0x94; //default 1kHz
        break;
    }

    /* 尝试配置 */
    ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,valset);
    /* 回读一次 */
    valget = ADS1299_ReadREG(0,ADS1299_REG_CONFIG1);

    if(valget!=valset)
        ret = EXIT_FAILURE;

    return ret;
}

/*!
 *  @fn     ADS1299_SetGain
 *
 *  @brief  ADS1299 增益设置
 *  本函数支持指定ADS1299模块所有通道的增益设置。
 *
 *  @param  chip - ADS1299模块编号 1-4:模块1-4 0:所有模块
 *  @param  gain - 待设置的增益
 *
 *  @return EXIT_SUCCESS - 增益设置成功
 *          EXIT_FAILURE - 增益设置失败
 */
bool ADS1299_SetGain(uint8_t chip, uint8_t gain)
{
    uint8_t valget,i;
    TADS1299CHnSET ChVal; 
		
    switch(gain)
     {
         case 1:
             ChVal.control_bit.gain = 0;
         break;

         case 2:
             ChVal.control_bit.gain = 1;
         break;

         case 4:
             ChVal.control_bit.gain = 2;
         break;

         case 6:
             ChVal.control_bit.gain = 3;
         break;

         case 8:
             ChVal.control_bit.gain = 4;
         break;

         case 12:
             ChVal.control_bit.gain = 5;
         break;

         case 24:
             ChVal.control_bit.gain = 6;
         break;

         default:
             ChVal.control_bit.gain = 6; //default x24
         break;
     }


     for(i=0;i<8;i++)
     {
         /* 尝试配置 */
         ADS1299_Channel_Config(chip,i,ChVal);
				
			   if(chip==0){
					 /* 如果为所有模块通道设置一致，则每配置一次回读一次 */
					 valget = ADS1299_ReadREG(chip,ADS1299_REG_CH1SET+i);
					 if(valget!=ChVal.value) 
						 return EXIT_FAILURE;
					}
     }
     return EXIT_SUCCESS;
}

/*!
 *  @fn     ADS1299_Channel_Control
 *
 *  @brief  ADS1299 通道开关控制
 *  通道关闭，则输入短接MUX=001，同时关闭对应通道的输入偏置接入。
 *
 *  @param  chip - ADS1299模块编号 1-4:模块1-4 0:所有模块
 *  @param  channel - 通道编号
 *  @param  PDn - 通道开关 0-关闭 1-打开
 */
void ADS1299_Channel_Control(uint8_t chip, uint8_t channel, uint8_t PDn)
{
	TADS1299CHnSET chVal;
	uint8_t bias = (1<<channel);
	
	if(PDn==0){
		chVal.control_bit.mux = 0x01;  // input shorted
		chVal.control_bit.pd = 0x01; // power down
		ADS1299_Channel_Config(chip,channel,chVal);
		
		ADS1299_WriteREG(chip,ADS1299_REG_BIASSENSP,~(0xFF&bias));
	}
	else
	{
		chVal.control_bit.mux = 0x00;  // input shorted
		chVal.control_bit.pd = 0x00; // power down
		ADS1299_Channel_Config(chip,channel,chVal);
	}
}