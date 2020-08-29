//*****************************************************************************
//
//! \file w5500.c
//! \brief W5500 HAL Interface.
//! \version 1.0.2
//! \date 2013/10/21
//! \par  Revision history
//!       <2015/02/05> Notice
//!        The version history is not updated after this point.
//!        Download the latest version directly from GitHub. Please visit the our GitHub repository for ioLibrary.
//!        >> https://github.com/Wiznet/ioLibrary_Driver
//!       <2014/05/01> V1.0.2
//!         1. Implicit type casting -> Explicit type casting. Refer to M20140501
//!            Fixed the problem on porting into under 32bit MCU
//!            Issued by Mathias ClauBen, wizwiki forum ID Think01 and bobh
//!            Thank for your interesting and serious advices.
//!       <2013/12/20> V1.0.1
//!         1. Remove warning
//!         2. WIZCHIP_READ_BUF WIZCHIP_WRITE_BUF in case _WIZCHIP_IO_MODE_SPI_FDM_
//!            for loop optimized(removed). refer to M20131220
//!       <2013/10/21> 1st Release
//! \author MidnightCow
//! \copyright
//!
//! Copyright (c)  2013, WIZnet Co., LTD.
//! All rights reserved.
//! 
//! Redistribution and use in source and binary forms, with or without 
//! modification, are permitted provided that the following conditions 
//! are met: 
//! 
//!     * Redistributions of source code must retain the above copyright 
//! notice, this list of conditions and the following disclaimer. 
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution. 
//!     * Neither the name of the <ORGANIZATION> nor the names of its 
//! contributors may be used to endorse or promote products derived 
//! from this software without specific prior written permission. 
//! 
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
//#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "w5500.h"
#include "W5500_App.h"
#include "QSPI.h"

extern QSPI_HandleTypeDef hqspi; 
extern DMA_HandleTypeDef hdma_quadspi;

#if   (_WIZCHIP_ == 5500)
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* 函数名  : WIZCHIP_READ
*
* 描述    : 读W5500寄存器区域指定寄存器的1个字节数据
*
* 输入    : @AddrSel: 32位 = 8空位+16位地址段+8位控制段
*						- 地址段: 寄存器偏移地址 - @ref 0x0002 即寄存器偏移量 = 2
*						- 控制段: 寄存器区域  通用寄存器区  		 - @ref  WIZCHIP_CREG_BLOCK
*																	Socket n寄存器区   - @ref  WIZCHIP_SREG_BLOCK(N)
*																	Socket n发送缓存区 - @ref  WIZCHIP_TXBUF_BLOCK(N)
*																	Socket n接收缓存区 - @ref  WIZCHIP_RXBUF_BLOCK(N)
*
*
* 返回值  : 读取到寄存器的1个字节数据
*
* 说明    : 无
*******************************************************************************/
uint8_t  WIZCHIP_READ(uint32_t AddrSel)
{
   uint8_t 	ret;
	 uint16_t RegAddr;
	 uint8_t 	ControlWord;
	
   RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500地址段:寄存器偏移地址
	 AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);	// W5500控制段:读访问、可变数据长度模式
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500控制段:寄存器区域 
	 
	 // QSPI命令配置 单线传输
   //	| 指令 |  地址   |  交替字节   | 空指令 | 数据 |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	ret  |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // 读1字节寄存器数据
	 QSPI_Receive(&ret,1);
	
   return ret;
}

/*******************************************************************************
* 函数名  : WIZCHIP_WRITE
*
* 描述    : 向W5500寄存器区域指定地址寄存器写1字节数据
*
* 输入    : @AddrSel: 32位 = 8空位+16位地址段+8位控制段
						@value: 1字节待写入数据
*
* 返回值  : 无
*
* 说明    : @AddrSel - @ref WIZCHIP_READ 
*******************************************************************************/
void WIZCHIP_WRITE(uint32_t AddrSel, uint8_t value )
{
	 uint16_t RegAddr;
	 uint8_t 	ControlWord;
   
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500地址段:寄存器偏移地址
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);	// W5500控制段:写访问、可变数据长度模式
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500控制段:寄存器区域 
	 
	 // QSPI命令配置 单线传输
   //	| 指令 |  地址   |  交替字节   | 空指令 | 数据  |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	value |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // 写入1字节数据
	 QSPI_Transmit(&value,1);
	
}
      
/*******************************************************************************
* 函数名  : WIZCHIP_WRITE_IT
*
* 描述    : 向W5500寄存器区域指定地址寄存器写1字节数据（用于中断）
*
* 输入    : @AddrSel: 32位 = 8空位+16位地址段+8位控制段
*						@value: 1字节待写入数据
*
* 返回值  : 无
*
* 说明    : @AddrSel - @ref WIZCHIP_READ
*******************************************************************************/
void WIZCHIP_WRITE_IT (uint32_t AddrSel, uint8_t value )
{
	 uint16_t RegAddr;
	 uint8_t 	ControlWord;
   
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500地址段:寄存器偏移地址
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);	// W5500控制段:写访问、可变数据长度模式
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500控制段:寄存器区域 
	 
	 // QSPI命令配置 单线传输
   //	| 指令 |  地址   |  交替字节   | 空指令 | 数据  |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	value |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // 写入1字节数据	 
	 QSPI_Transmit_IT(&value,1);
}

/*******************************************************************************
* 函数名  : WIZCHIP_READ_BUF
*
* 描述    : 读W5500寄存器区域指定地址寄存器的n字节数据
*
* 输入    : @AddrSel: 32位 = 8空位+16位地址段+8位控制段
*						@pBuf: 待读取数据缓冲区指针
*						@len: 待读取数据长度
*
* 返回值  : 无
*
* 说明    : @AddrSel - @ref WIZCHIP_READ
*******************************************************************************/
void WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
	 uint16_t RegAddr;
	 uint8_t ControlWord;
	
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500地址段:寄存器偏移地址
	 AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);	// W5500控制段:读访问、可变数据长度模式
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500控制段:寄存器区域
	
	 // QSPI命令配置 单线传输
   //	| 指令 |  地址   |  交替字节   | 空指令 | 数据 |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	pBuf |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // 读取n字节数据
	 QSPI_Receive(pBuf,len);
	
}

/*******************************************************************************
* 函数名  : WIZCHIP_WRITE_BUF
*
* 描述    : 向W5500寄存器区域指定地址寄存器写入n字节数据
*
* 输入    : @AddrSel: 32位=8空位+16位寄存器地址+8控制字
*						@pBuf: 待写入数据缓冲区指针
*						@len:待写入数据长度
*
* 返回值  : 无
*
* 说明    : @AddrSel - @ref WIZCHIP_READ
*******************************************************************************/
void WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
	 uint16_t RegAddr;
	 uint8_t ControlWord;
	
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500地址段:寄存器偏移地址
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);	// W5500控制段:写访问、可变数据长度模式
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500控制段:寄存器区域 

	 // QSPI命令配置 单线传输
   //	| 指令 |  地址   |  交替字节   | 空指令 | 数据 |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	pBuf |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 //写入n字节数据 
	 QSPI_Transmit(pBuf,len); 
	
}


/*******************************************************************************
* 函数名  : Write_SOCK_Data_Buffer
*
* 描述    : 将数据写入W5500的Socket n数据发送缓冲区（DMA方式）
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*						@dat_ptr: 待写入数据指针
*						@len: 待写入数据长度
*
* 返回值  : @ptr: 本次数据写入前发送缓冲区写指针（for debug） 
*
* 说明    : 无
*******************************************************************************/
uint16_t Write_SOCK_Data_Buffer(uint8_t sn, uint8_t *dat_ptr, uint16_t len)
{
	 uint16_t ptr = 0;
   uint32_t addrsel = 0;
	 uint16_t RegAddr;
	 uint8_t ControlWord;
			
	 //读取发送缓冲区写指针，为当前数据写入的首地址
   ptr = getSn_TX_WR(sn);
	
	 //将数据写入对应的端口的发送缓冲区
   addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3); 
	 RegAddr=((addrsel & 0x00FFFF00)>>8);           // W5500地址段:Socket n发送缓冲区地址
	 addrsel |= _W5500_SPI_WRITE_ ;	                // W5500控制段:写访问
	 ControlWord=((addrsel & 0x000000FF) >>  0);    // W5500控制段:寄存器区域 - Socket n发送缓存区
	 
	 // QSPI命令配置 单线传输
   //	| 指令 |  地址   |  交替字节   | 空指令 |  数据   |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	dat_ptr |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 hqspi.Instance->DLR=len-1;//len-1;				   //配置数据长度,17个字节不知所踪
	 HAL_QSPI_Transmit_DMA(&hqspi, dat_ptr);		   //DMA方式向发送缓冲区写n字节数据 
	 
	 while(1)
	 {
		 if(__HAL_DMA_GET_FLAG(&hdma_quadspi,DMA_FLAG_TCIF3_7))//等待 DMA2_Steam7 传输完成	
			{
				__HAL_DMA_CLEAR_FLAG(&hdma_quadspi,DMA_FLAG_TCIF3_7);//清除 DMA2_Steam7 传输完成标志			
				HAL_QSPI_Abort(&hqspi);//传输完成以后关闭DMA
				break;
	    }
	 }
	 
	//更新发送缓冲区写指针，为下一次数据写入的首地址
   ptr += len; 
   setSn_TX_WR(sn,ptr);
	 
	 return ptr;
}

/*******************************************************************************
* 函数名  : wiz_send_data
*
* 描述    : 将数据写入W5500的Socket n数据发送缓冲区
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*						@wizdata: 待写入数据指针
*						@len: 待写入数据长度
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
void wiz_send_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
   uint16_t ptr = 0;
   uint32_t AddrSel = 0;

   if(len == 0) //无数据待写入  
		 return; 
	 
	 //读取发送缓冲区写指针，为此次数据写入的首地址
   ptr = getSn_TX_WR(sn); 
	 
	 //将数据写入对应的端口的发送缓冲区
	 AddrSel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3); 
	 WIZCHIP_WRITE_BUF(AddrSel,wizdata, len); 
   
	 //更新发送缓冲区写指针，为下一次数据写入的首地址
   ptr += len; 
   setSn_TX_WR(sn,ptr);
}

/*******************************************************************************
* 函数名  : Read_SOCK_Data_Buffer
*
* 描述    : 从W5500接收数据缓冲区中读取数据（DMA方式）
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*           @dat_ptr: 保存待接收数据指针
*						@len: 待接收数据长度
*
* 返回值  : @ptr: 本次接收完成后接收缓冲区读指针 （for debug）
*
* 说明    : 无
*******************************************************************************/
uint16_t Read_SOCK_Data_Buffer(uint8_t sn, uint8_t *dat_ptr,uint32_t len)
{
	 uint16_t ptr = 0;
   uint32_t addrsel = 0;
   uint16_t RegAddr;
	 uint8_t ControlWord;
	 
	 //读取接收缓冲区读指针，为此次数据读取的首地址
	 ptr = getSn_RX_RD(sn);
	
   addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);
   RegAddr = ((addrsel & 0x00FFFF00)>>8);         // W5500地址段:Socket n接收缓冲区地址
	 addrsel |= _W5500_SPI_READ_ ;	                // W5500控制段:写访问
	 ControlWord=((addrsel & 0x000000FF) >>  0);    // W5500控制段:寄存器区域 - Socket n接收缓存区
	 
	 // QSPI命令配置 单线传输
   //	| 指令 |  地址   |  交替字节   | 空指令 |  数据   |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	dat_ptr |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);

	 HAL_QSPI_Receive_DMA(&hqspi,dat_ptr);
		
	 while(1)
	 {
		 if(__HAL_DMA_GET_FLAG(&hdma_quadspi,DMA_FLAG_TCIF3_7))//等待 DMA2_Steam7 接收完成	
			{
				__HAL_DMA_CLEAR_FLAG(&hdma_quadspi,DMA_FLAG_TCIF3_7);//清除 DMA2_Steam7 接收完成标志			
				HAL_QSPI_Abort(&hqspi);//接收完成以后关闭DMA
				break;
	    }
	  }
	 
	 //更新接收缓冲区读指针，为下一次数据读取的首地址 	 
   ptr += len; 
   setSn_RX_RD(sn,ptr);	

	 return ptr;
}

/*******************************************************************************
* 函数名  : wiz_recv_data
*
* 描述    : 从W5500接收数据缓冲区中读取数据
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*						@wizdata: 数据保存指针
*						@len: 读取数据长度
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
   uint16_t ptr = 0;
   uint32_t AddrSel = 0;
   
   if(len == 0)  
		 return;
	 
	 //读取接收缓冲区读指针，为此次数据读取的首地址   
	 ptr = getSn_RX_RD(sn);
	 
	 //从对应的端口的接收缓冲区读取数据
   AddrSel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);
	 WIZCHIP_READ_BUF(AddrSel, wizdata, len);
	 
	 //更新接收缓冲区读指针，为下一次数据读取的首地址   
	 ptr += len;
   setSn_RX_RD(sn,ptr);
	 setSn_CR(sn,Sn_CR_RECV); // RECV命令，将更新后的Sn_RX_RD告知W5500
   while(getSn_CR(sn));
}

/*******************************************************************************
* 函数名  : wiz_recv_ignore
*
* 描述    : 从W5500接收数据缓冲区中直接丢弃数据
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*						@len: 丢弃数据长度
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
void wiz_recv_ignore(uint8_t sn, uint16_t len)
{
   uint16_t ptr = 0;
	
	 //读取接收缓冲区读指针，为此次丢弃数据的首地址 
   ptr = getSn_RX_RD(sn);
	
	 //更新接收缓冲区读指针，为下一次数据读取/丢弃的首地址    
	 ptr += len;
   setSn_RX_RD(sn,ptr);
}

/*******************************************************************************
* 函数名  : getSn_TX_FSR
*
* 描述    : 读取Socket n 发送缓冲区空闲空间大小 
*						(Sn_TX_FSRn\Sn_TX_FSRn+1 寄存器值)
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*
* 返回值  : 发送缓冲区空闲空间大小
*
* 说明    : 无
*******************************************************************************/
uint16_t getSn_TX_FSR(uint8_t sn)
{
   uint16_t value1=0,value2=0;

   do
   { 
     // 读取Socket n寄存器Sn_TX_FSRn (offset=0x002n)的值 （value2的高八位）
		 value2 = WIZCHIP_READ(Sn_TX_FSR(sn)); 
     // 读取Socket n寄存器Sn_TX_FSRn+1 (offset=0x002n+1)的值 （value2的低八位）
		 value2 = (value2 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1)); 
      if (value2 != 0)
      {
				// 重复读取一次
        value1 = WIZCHIP_READ(Sn_TX_FSR(sn));
        value1 = (value1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
      }
   }while (value1 != value2);
   
	 return value1;
}

/*******************************************************************************
* 函数名  : getSn_RX_RSR
*
* 描述    : 读取Socket n 接收缓冲区空闲空间大小
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*
* 返回值  : 发送缓冲区空闲空间大小
*
* 说明    : 无
*******************************************************************************/
uint16_t getSn_RX_RSR(uint8_t sn)
{
   uint16_t value1=0,value2=0;

   do
   {
      value2 = WIZCHIP_READ(Sn_RX_RSR(sn));
      value2 = (value2 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
      if (value2 != 0)
      {
        value1 = WIZCHIP_READ(Sn_RX_RSR(sn));
        value1 = (value1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
      }
   }while (value1 != value2);
   return value1;
}

#endif
 ////////////////////////////////////////////////////////////////////////////////