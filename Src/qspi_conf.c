/**
 * @file    qspi_conf.c
 * @author  Yeyangyang
 * @brief   qspi�弶���� - ��������оƬ��W5500оƬspiͨ��
 * @version 1.0.0
 * @date    2019-07-04
 *
 * @copyright (c) 2019 yeyangyang
 *
 */

#include "stm32f4xx_hal.h"
#include "qspi_conf.h"

extern QSPI_HandleTypeDef hqspi; 

/*******************************************************************************
* ������  : QSPI_Send_CMD
*
* ����    : QSPI������������
*
* ����    : @address:��ַ����
*						@controlword:�����ֽ�����
*					  @dummyCycles:��ָ������������
*						@dataMode:���������շ�ģʽ
*						
* ����ֵ  : ��
*******************************************************************************/
void QSPI_Send_Control( uint16_t address,uint8_t controlword,uint32_t dummyCycles,uint32_t dataMode)
{
		QSPI_CommandTypeDef Cmdhandler;
		
		Cmdhandler.InstructionMode=QSPI_INSTRUCTION_NONE; 					// ָ������ - NULL	
		Cmdhandler.Address=address; 																// ��ַ���� - address
		Cmdhandler.AddressSize=QSPI_ADDRESS_16_BITS; 								// ��ַ����� - 16λ  
		Cmdhandler.AddressMode=QSPI_ADDRESS_1_LINE; 								// ��ַ�����ģʽ - ����	
		Cmdhandler.AlternateBytes=controlword;											// �����ֽ����� - controlword
	  Cmdhandler.AlternateBytesSize=QSPI_ALTERNATE_BYTES_8_BITS;	// �����ֽڳ��� - 8λ
		Cmdhandler.AlternateByteMode=QSPI_ALTERNATE_BYTES_1_LINE; 	// �����ֽڷ���ģʽ - ����		
		Cmdhandler.DummyCycles=dummyCycles; 												// ��ָ������������ - dummyCycles
		Cmdhandler.DataMode=dataMode; 															// ���������շ�ģʽ - dataMode
		Cmdhandler.DdrMode=QSPI_DDR_MODE_DISABLE; 									// �ر� DDR ģʽ
		Cmdhandler.DdrHoldHalfCycle=QSPI_DDR_HHC_ANALOG_DELAY;      // Ĭ������
		Cmdhandler.SIOOMode= QSPI_SIOO_INST_EVERY_CMD;              // Ĭ������

		HAL_QSPI_Command(&hqspi,&Cmdhandler,HAL_QPSI_TIMEOUT_DEFAULT_VALUE);

}

/*******************************************************************************
* ������  : QSPI_Send_Control_IT
*
* ����    : QSPI�����������ã��жϷ�ʽ��
*
* ����    : @address:��ַ����
*						@controlword:�����ֽ�����
*					  @dummyCycles:��ָ������������
*						@dataMode:���������շ�ģʽ
*						
* ����ֵ  : ��
*******************************************************************************/
void QSPI_Send_Control_IT(uint16_t address,uint8_t controlword,uint32_t dummyCycles,uint32_t dataMode)
{
		QSPI_CommandTypeDef Cmdhandler;
	
		Cmdhandler.InstructionMode=QSPI_INSTRUCTION_NONE; 					// ָ������ - NULL	
		Cmdhandler.Address=address; 																// ��ַ���� - address
		Cmdhandler.AddressSize=QSPI_ADDRESS_16_BITS; 								// ��ַ����� - 16λ  
		Cmdhandler.AddressMode=QSPI_ADDRESS_1_LINE; 								// ��ַ�����ģʽ - ����	
		Cmdhandler.AlternateBytes=controlword;											// �����ֽ����� - controlword
	  Cmdhandler.AlternateBytesSize=QSPI_ALTERNATE_BYTES_8_BITS;	// �����ֽڳ��� - 8λ
		Cmdhandler.AlternateByteMode=QSPI_ALTERNATE_BYTES_1_LINE; 	// �����ֽڷ���ģʽ - ����		
		Cmdhandler.DummyCycles=dummyCycles; 												// ��ָ������������ - dummyCycles
		Cmdhandler.DataMode=dataMode; 															// ���������շ�ģʽ - dataMode
		Cmdhandler.DdrMode=QSPI_DDR_MODE_DISABLE; 									// �ر� DDR ģʽ
		Cmdhandler.DdrHoldHalfCycle=QSPI_DDR_HHC_ANALOG_DELAY;      // Ĭ������
		Cmdhandler.SIOOMode= QSPI_SIOO_INST_EVERY_CMD;              // Ĭ������
	
		HAL_QSPI_Command_IT(&hqspi,&Cmdhandler);
}

/*******************************************************************************
* ������  : QSPI_Receive
*
* ����    : QSPI����ָ�����ȵ�����
*
* ����    : @buf: �������ݻ������׵�ַ
						@datalen: �������ݳ���
*
* ����ֵ  : 0 - ����
						1 - ����
*******************************************************************************/
uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen)
{
	hqspi.Instance->DLR=datalen-1; //�������ݳ���
	
	if( HAL_QSPI_Receive(&hqspi,buf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE)== HAL_OK ) 
		return 0; //��������
	else 
		return 1;
}

/*******************************************************************************
* ������  : QSPI_Transmit
*
* ����    : QSPI����ָ�����ȵ�����
*
* ����    : @buf: �������ݻ������׵�ַ
						@datalen: �������ݳ���
*
* ����ֵ  : 0 - ����
						1 - ����
*******************************************************************************/
uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen)
{
	hqspi.Instance->DLR=datalen-1; //�������ݳ���
	
	if( HAL_QSPI_Transmit(&hqspi,buf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE)==HAL_OK ) 
		return 0; //��������
	else 
		return 1;
}

/*******************************************************************************
* ������  : QSPI_Transmit_IT
*
* ����    : QSPI����ָ�����ȵ����ݣ��жϷ�ʽ��
*
* ����    : @buf: �������ݻ������׵�ַ
						@datalen: �������ݳ���
*
* ����ֵ  : 0 - ����
						1 - ����
*******************************************************************************/
uint8_t QSPI_Transmit_IT(uint8_t* buf,uint32_t datalen)
{
	hqspi.Instance->DLR=datalen-1; //�������ݳ���
	
	if(HAL_QSPI_Transmit_IT(&hqspi,buf)==HAL_OK) 
		return 0; //��������
	else 
		return 1;
}
