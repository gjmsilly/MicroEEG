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
* ������  : WIZCHIP_READ
*
* ����    : ��W5500�Ĵ�������ָ���Ĵ�����1���ֽ�����
*
* ����    : @AddrSel: 32λ = 8��λ+16λ��ַ��+8λ���ƶ�
*						- ��ַ��: �Ĵ���ƫ�Ƶ�ַ
*						- ���ƶ�: �Ĵ�������  ͨ�üĴ�����
*																	Socket n�Ĵ�����
*																	Socket n���ͻ�����
*																	Socket n���ջ�����
*
*
* ����ֵ  : ��ȡ���Ĵ�����1���ֽ�����
*******************************************************************************/
uint8_t  WIZCHIP_READ(uint32_t AddrSel)
{
   uint8_t 	ret;
	 uint16_t RegAddr;
	 uint8_t 	ControlWord;
	
   RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500��ַ��:�Ĵ���ƫ�Ƶ�ַ
	 AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);	// W5500���ƶ�:�����ʡ��ɱ����ݳ���ģʽ
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500���ƶ�:�Ĵ������� 
	 
	 // QSPI�������� ���ߴ���
   //	| ָ�� |  ��ַ   |  �����ֽ�   | ��ָ�� | ���� |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	ret  |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // ��1�ֽڼĴ�������
	 QSPI_Receive(&ret,1);
	
   return ret;
}

/*******************************************************************************
* ������  : WIZCHIP_WRITE
*
* ����    : ��W5500�Ĵ�������ָ����ַ�Ĵ���д1�ֽ�����
*
* ����    : @AddrSel: 32λ = 8��λ+16λ��ַ��+8λ���ƶ�
						@value: 1�ֽڴ�д������
*
* ����ֵ  : ��
*******************************************************************************/
void WIZCHIP_WRITE(uint32_t AddrSel, uint8_t value )
{
	 uint16_t RegAddr;
	 uint8_t 	ControlWord;
   
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500��ַ��:�Ĵ���ƫ�Ƶ�ַ
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);	// W5500���ƶ�:д���ʡ��ɱ����ݳ���ģʽ
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500���ƶ�:�Ĵ������� 
	 
	 // QSPI�������� ���ߴ���
   //	| ָ�� |  ��ַ   |  �����ֽ�   | ��ָ�� | ����  |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	value |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // д��1�ֽ�����
	 QSPI_Transmit(&value,1);
	
}
      
/*******************************************************************************
* ������  : WIZCHIP_WRITE_IT
*
* ����    : ��W5500�Ĵ�������ָ����ַ�Ĵ���д1�ֽ����ݣ������жϣ�
*
* ����    : @AddrSel: 32λ = 8��λ+16λ��ַ��+8λ���ƶ�
*						@value: 1�ֽڴ�д������
*
* ����ֵ  : ��
*******************************************************************************/
void WIZCHIP_WRITE_IT (uint32_t AddrSel, uint8_t value )
{
	 uint16_t RegAddr;
	 uint8_t 	ControlWord;
   
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500��ַ��:�Ĵ���ƫ�Ƶ�ַ
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);	// W5500���ƶ�:д���ʡ��ɱ����ݳ���ģʽ
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500���ƶ�:�Ĵ������� 
	 
	 // QSPI�������� ���ߴ���
   //	| ָ�� |  ��ַ   |  �����ֽ�   | ��ָ�� | ����  |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	value |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // д��1�ֽ�����	 
	 QSPI_Transmit_IT(&value,1);
}

/*******************************************************************************
* ������  : WIZCHIP_READ_BUF
*
* ����    : ��W5500�Ĵ�������ָ����ַ�Ĵ�����n�ֽ�����
*
* ����    : @AddrSel: 32λ = 8��λ+16λ��ַ��+8λ���ƶ�
*						@pBuf: ����ȡ���ݻ�����ָ��
*						@len: ����ȡ���ݳ���
*
* ����ֵ  : ��
*******************************************************************************/
void WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
	 uint16_t RegAddr;
	 uint8_t ControlWord;
	
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500��ַ��:�Ĵ���ƫ�Ƶ�ַ
	 AddrSel |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);	// W5500���ƶ�:�����ʡ��ɱ����ݳ���ģʽ
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500���ƶ�:�Ĵ�������
	
	 // QSPI�������� ���ߴ���
   //	| ָ�� |  ��ַ   |  �����ֽ�   | ��ָ�� | ���� |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	pBuf |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 // ��ȡn�ֽ�����
	 QSPI_Receive(pBuf,len);
	
}

/*******************************************************************************
* ������  : WIZCHIP_WRITE_BUF
*
* ����    : ��W5500�Ĵ�������ָ����ַ�Ĵ���д��n�ֽ�����
*
* ����    : @AddrSel: 32λ=8��λ+16λ�Ĵ�����ַ+8������
*						@pBuf: ��д�����ݻ�����ָ��
*						@len:��д�����ݳ���
*
* ����ֵ  : ��
*******************************************************************************/
void WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
	 uint16_t RegAddr;
	 uint8_t ControlWord;
	
	 RegAddr 	= ((AddrSel & 0x00FFFF00)>>8); 							// W5500��ַ��:�Ĵ���ƫ�Ƶ�ַ
	 AddrSel |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);	// W5500���ƶ�:д���ʡ��ɱ����ݳ���ģʽ
	 ControlWord = ((AddrSel & 0x000000FF) >> 0);					// W5500���ƶ�:�Ĵ������� 

	 // QSPI�������� ���ߴ���
   //	| ָ�� |  ��ַ   |  �����ֽ�   | ��ָ�� | ���� |
	 // | NULL | RegAddr | ControlWord |	NULL 	|	pBuf |
	 QSPI_Send_Control(RegAddr,ControlWord,0,QSPI_DATA_1_LINE);
	 //д��n�ֽ����� 
	 QSPI_Transmit(pBuf,len); 
	
}


/*******************************************************************************
* ������  : Write_SOCK_Data_Buffer
*
* ����    : ������д��W5500�����ݷ��ͻ�����
*
* ����    : @sn: �˿ں�/Socket n
*						@dat_ptr: ���ݱ��滺����ָ��
*						@len: ��д�����ݳ���
*
* ����ֵ  : ��
*******************************************************************************/
void Write_SOCK_Data_Buffer(uint8_t sn, uint8_t *dat_ptr, uint16_t len)
{
	uint16_t offset,offset1;

	//�������UDPģʽ,�����ڴ�����Ŀ��������IP�Ͷ˿ں�
	if((getSn_MR(sn)&0x0f) != SOCK_UDP)//���Socket��ʧ��
	{		
		setSn_DIPR(sn, UDP_DIPR);//����Ŀ������IP  		
		setSn_DPORT(sn, UDP_DPORT);//����Ŀ�������˿ں�				
	}

	offset=getSn_TX_WR(sn);
	offset1=offset;
	offset&=(S_TX_SIZE-1);//����ʵ�ʵ������ַ

  QSPI_Send_Control(offset,(_W5500_SPI_VDM_OP_|_W5500_SPI_WRITE_|(WIZCHIP_TXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	//д���ַ��д�����ֽڣ�N���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ��������޿�����,���ߴ�������
	if((offset+len)<S_TX_SIZE)//�������ַδ����W5500���ͻ������Ĵ���������ַ
	{
		QSPI_Transmit(dat_ptr,len);////дsize���ֽ�����
	}
	else//�������ַ����W5500���ͻ������Ĵ���������ַ
	{
		offset=S_TX_SIZE-offset;
		QSPI_Transmit(dat_ptr,offset);////дsize���ֽ�����

    QSPI_Send_Control(0x00,(_W5500_SPI_VDM_OP_|_W5500_SPI_WRITE_|(WIZCHIP_TXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	  //д���ַ��д�����ֽڣ�N���ֽ����ݳ���,д����,ѡ��˿�s�ļĴ��������޿�����,���ߴ�������	
    QSPI_Transmit(dat_ptr,len);////дsize-offset���ֽ�����
	}

	offset1+=len;//����ʵ�������ַ,���´�д���������ݵ��������ݻ���������ʼ��ַ
	setSn_TX_WR(sn, offset1);
	setSn_CR(sn, Sn_CR_SEND);//����������������	

}

/*******************************************************************************
* ������  : wiz_send_data
*
* ����    : ������д��W5500�����ݷ��ͻ�����
*
* ����    : @sn: �˿ں�/Socket n
*						@wizdata: ��д������ָ��
*						@len: ��д�����ݳ���
*
* ����ֵ  : ��
*******************************************************************************/
void wiz_send_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
   uint16_t ptr = 0;
   uint32_t AddrSel = 0;

   if(len == 0) //�����ݴ�д��  
		 return; 
	 
	 //��ȡ���ͻ�����дָ�룬Ϊ�˴�����д����׵�ַ
   ptr = getSn_TX_WR(sn); 
	 
	 //������д���Ӧ�Ķ˿ڵķ��ͻ�����
	 AddrSel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3); 
	 WIZCHIP_WRITE_BUF(AddrSel,wizdata, len); 
   
	 //���·��ͻ�����дָ�룬Ϊ��һ������д����׵�ַ
   ptr += len; 
   setSn_TX_WR(sn,ptr);
}

/*******************************************************************************
* ������  : Read_SOCK_Data_Buffer
* ����    : ��W5500�������ݻ������ж�ȡ����
* ����    : s:�˿ں�,*dat_ptr:���ݱ��滺����ָ��
* ���    : ��
* ����ֵ  : ��ȡ�������ݳ���,rx_size���ֽ�
* ˵��    : ��
*******************************************************************************/
uint16_t Read_SOCK_Data_Buffer(uint8_t sn, uint8_t *dat_ptr)
{
	uint16_t rx_size;
	uint16_t offset, offset1;

	rx_size=getSn_RX_RSR(sn);
	if(rx_size==0) return 0;//û���յ������򷵻�
	if(rx_size>1460) rx_size=1460;

	offset=getSn_RX_RD(sn);
	offset1=offset;
	offset&=(S_RX_SIZE-1);//����ʵ�ʵ������ַ

	QSPI_Send_Control(offset,(_W5500_SPI_VDM_OP_|_W5500_SPI_READ_|(WIZCHIP_RXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	//д���ַ��д�����ֽڣ�N���ֽ����ݳ���,������,ѡ��˿�s�ļĴ��������޿�����,���ߴ�������
	
	if((offset+rx_size)<S_RX_SIZE)//�������ַδ����W5500���ջ������Ĵ���������ַ
	{
		QSPI_Receive(dat_ptr,rx_size);//��ȡrx_size���ֽ����ݣ�������ȡ�������ݱ��浽���ݱ��滺����
	}
	else//�������ַ����W5500���ջ������Ĵ���������ַ
	{
		offset=S_RX_SIZE-offset;
		QSPI_Receive(dat_ptr,offset);//��ȡrx_size���ֽ����ݣ�������ȡ�������ݱ��浽���ݱ��滺����
		
	  QSPI_Send_Control(0x00,(_W5500_SPI_VDM_OP_|_W5500_SPI_READ_|(WIZCHIP_RXBUF_BLOCK(sn)<<3)),0,QSPI_DATA_1_LINE);
	  //д���ַ��д�����ֽڣ�N���ֽ����ݳ���,������,ѡ��˿�s�ļĴ��������޿�����,���ߴ�������
		
		QSPI_Receive(dat_ptr,rx_size);//��ȡrx_size-offset���ֽ����ݣ�������ȡ�������ݱ��浽���ݱ��滺����
	}
	offset1+=rx_size;//����ʵ�������ַ,���´ζ�ȡ���յ������ݵ���ʼ��ַ
	setSn_RX_RD(sn, offset1);
	setSn_CR(sn, Sn_CR_RECV);//����������������
	return rx_size;//���ؽ��յ����ݵĳ���
}

/*******************************************************************************
* ������  : wiz_recv_data
*
* ����    : ��W5500�������ݻ������ж�ȡ����
*
* ����    : @sn:�˿ں�
*						@wizdata: ���ݱ���ָ��
*						@len: ��ȡ���ݳ���
*
* ����ֵ  : ��
*******************************************************************************/
void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
   uint16_t ptr = 0;
   uint32_t AddrSel = 0;
   
   if(len == 0)  
		 return;
	 
	 //��ȡ���ջ�������ָ�룬Ϊ�˴����ݶ�ȡ���׵�ַ   
	 ptr = getSn_RX_RD(sn);
	 
	 //�Ӷ�Ӧ�Ķ˿ڵĽ��ջ�������ȡ����
   AddrSel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);
	 WIZCHIP_READ_BUF(AddrSel, wizdata, len);
	 
	 //���½��ջ�������ָ�룬Ϊ��һ�����ݶ�ȡ���׵�ַ   
	 ptr += len;
   setSn_RX_RD(sn,ptr);
}

/*******************************************************************************
* ������  : wiz_recv_ignore
*
* ����    : ��W5500�������ݻ�������ֱ�Ӷ�������
*
* ����    : @sn:�˿ں�
*						@len: �������ݳ���
*
* ����ֵ  : ��
*******************************************************************************/
void wiz_recv_ignore(uint8_t sn, uint16_t len)
{
   uint16_t ptr = 0;
	
	 //��ȡ���ջ�������ָ�룬Ϊ�˴ζ������ݵ��׵�ַ 
   ptr = getSn_RX_RD(sn);
	
	 //���½��ջ�������ָ�룬Ϊ��һ�����ݶ�ȡ/�������׵�ַ    
	 ptr += len;
   setSn_RX_RD(sn,ptr);
}

/*******************************************************************************
* ������  : getSn_TX_FSR
*
* ����    : ��ȡSocket n ���ͻ��������пռ��С 
*						(Sn_TX_FSRn\Sn_TX_FSRn+1 �Ĵ���ֵ)
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*
* ����ֵ  : ���ͻ��������пռ��С
*******************************************************************************/
uint16_t getSn_TX_FSR(uint8_t sn)
{
   uint16_t value1=0,value2=0;

   do
   { 
     // ��ȡSocket n�Ĵ���Sn_TX_FSRn (offset=0x002n)��ֵ ��value1 �߰�λ��
		 value2 = WIZCHIP_READ(Sn_TX_FSR(sn)); 
     // ��ȡSocket n�Ĵ���Sn_TX_FSRn+1 (offset=0x002n+1)��ֵ ��value1 �Ͱ�λ��
		 value2 = (value2 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1)); 
      if (value2 != 0)
      {
				// �ظ���ȡһ��
        value1 = WIZCHIP_READ(Sn_TX_FSR(sn));
        value1 = (value1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
      }
   }while (value1 != value2);
   
	 return value1;
}

/*******************************************************************************
* ������  : getSn_RX_RSR
*
* ����    : ��ȡSocket n ���ջ��������пռ��С
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*
* ����ֵ  : ���ͻ��������пռ��С
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