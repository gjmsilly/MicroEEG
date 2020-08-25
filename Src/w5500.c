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


#if   (_WIZCHIP_ == 5500)
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* 函数名  : WIZCHIP_READ
*
* 描述    : 读W5500寄存器区域指定寄存器的1个字节数据
*
* 输入    : @AddrSel: 32位 = 8空位+16位地址段+8位控制段
*						- 地址段: 寄存器偏移地址
*						- 控制段: 寄存器区域  通用寄存器区
*																	Socket n寄存器区
*																	Socket n发送缓存区
*																	Socket n接收缓存区
*
*
* 返回值  : 读取到寄存器的1个字节数据
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
* 描述    : 将数据写入W5500的数据发送缓冲区
*
* 输入    : @sn: 端口号/Socket n
*						@dat_ptr: 数据保存缓冲区指针
*						@len: 待写入数据长度
*
* 返回值  : 无
*******************************************************************************/
void Write_SOCK_Data_Buffer(uint8_t sn, uint8_t *dat_ptr, uint16_t len)
{
	uint16_t offset,offset1;

	//如果不是UDP模式,可以在此设置目的主机的IP和端口号
	if((getSn_MR(sn)&0x0f) != SOCK_UDP)//如果Socket打开失败
	{		
		setSn_DIPR(sn, UDP_DIPR);//设置目的主机IP  		
		setSn_DPORT(sn, UDP_DPORT);//设置目的主机端口号				
	}

	offset=getSn_TX_WR(sn);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//计算实际的物理地址

  QSPI_Send_Control(offset,(_W5500_SPI_VDM_OP_|_W5500_SPI_WRITE_|(WIZCHIP_TXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	//写入地址，写控制字节（N个字节数据长度,写数据,选择端口s的寄存器），无空周期,单线传输数据
	if((offset+len)<S_TX_SIZE)//如果最大地址未超过W5500发送缓冲区寄存器的最大地址
	{
		QSPI_Transmit(dat_ptr,len);////写size个字节数据
	}
	else//如果最大地址超过W5500发送缓冲区寄存器的最大地址
	{
		offset=S_TX_SIZE-offset;
		QSPI_Transmit(dat_ptr,offset);////写size个字节数据

    QSPI_Send_Control(0x00,(_W5500_SPI_VDM_OP_|_W5500_SPI_WRITE_|(WIZCHIP_TXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	  //写入地址，写控制字节（N个字节数据长度,写数据,选择端口s的寄存器），无空周期,单线传输数据	
    QSPI_Transmit(dat_ptr,len);////写size-offset个字节数据
	}

	offset1+=len;//更新实际物理地址,即下次写待发送数据到发送数据缓冲区的起始地址
	setSn_TX_WR(sn, offset1);
	setSn_CR(sn, Sn_CR_SEND);//发送启动发送命令	

}

/*******************************************************************************
* 函数名  : wiz_send_data
*
* 描述    : 将数据写入W5500的数据发送缓冲区
*
* 输入    : @sn: 端口号/Socket n
*						@wizdata: 待写入数据指针
*						@len: 待写入数据长度
*
* 返回值  : 无
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
* 描述    : 从W5500接收数据缓冲区中读取数据
* 输入    : s:端口号,*dat_ptr:数据保存缓冲区指针
* 输出    : 无
* 返回值  : 读取到的数据长度,rx_size个字节
* 说明    : 无
*******************************************************************************/
uint16_t Read_SOCK_Data_Buffer(uint8_t sn, uint8_t *dat_ptr)
{
	uint16_t rx_size;
	uint16_t offset, offset1;

	rx_size=getSn_RX_RSR(sn);
	if(rx_size==0) return 0;//没接收到数据则返回
	if(rx_size>1460) rx_size=1460;

	offset=getSn_RX_RD(sn);
	offset1=offset;
	offset&=(S_RX_SIZE-1);//计算实际的物理地址

	QSPI_Send_Control(offset,(_W5500_SPI_VDM_OP_|_W5500_SPI_READ_|(WIZCHIP_RXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	//写入地址，写控制字节（N个字节数据长度,读数据,选择端口s的寄存器），无空周期,单线传输数据
	
	if((offset+rx_size)<S_RX_SIZE)//如果最大地址未超过W5500接收缓冲区寄存器的最大地址
	{
		QSPI_Receive(dat_ptr,rx_size);//读取rx_size个字节数据，并将读取到的数据保存到数据保存缓冲区
	}
	else//如果最大地址超过W5500接收缓冲区寄存器的最大地址
	{
		offset=S_RX_SIZE-offset;
		QSPI_Receive(dat_ptr,offset);//读取rx_size个字节数据，并将读取到的数据保存到数据保存缓冲区
		
	  QSPI_Send_Control(0x00,(_W5500_SPI_VDM_OP_|_W5500_SPI_READ_|(WIZCHIP_RXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	  //写入地址，写控制字节（N个字节数据长度,读数据,选择端口s的寄存器），无空周期,单线传输数据
		
		QSPI_Receive(dat_ptr,rx_size);//读取rx_size-offset个字节数据，并将读取到的数据保存到数据保存缓冲区
	}
	offset1+=rx_size;//更新实际物理地址,即下次读取接收到的数据的起始地址
	setSn_RX_RD(sn, offset1);
	setSn_CR(sn, Sn_CR_RECV);//发送启动接收命令
	return rx_size;//返回接收到数据的长度
}

/*******************************************************************************
* 函数名  : wiz_recv_data
*
* 描述    : 从W5500接收数据缓冲区中读取数据
*
* 输入    : @sn:端口号
*						@wizdata: 数据保存指针
*						@len: 读取数据长度
*
* 返回值  : 无
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
}

/*******************************************************************************
* 函数名  : wiz_recv_ignore
*
* 描述    : 从W5500接收数据缓冲区中直接丢弃数据
*
* 输入    : @sn:端口号
*						@len: 丢弃数据长度
*
* 返回值  : 无
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
*******************************************************************************/
uint16_t getSn_TX_FSR(uint8_t sn)
{
   uint16_t value1=0,value2=0;

   do
   { 
     // 读取Socket n寄存器Sn_TX_FSRn (offset=0x002n)的值 （value1 高八位）
		 value2 = WIZCHIP_READ(Sn_TX_FSR(sn)); 
     // 读取Socket n寄存器Sn_TX_FSRn+1 (offset=0x002n+1)的值 （value1 低八位）
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