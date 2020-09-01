#include "stm32f4xx_hal.h"
#include "qspi_conf.h"

extern QSPI_HandleTypeDef hqspi; 

/*******************************************************************************
* 函数名  : QSPI_Send_CMD
*
* 描述    : QSPI发送命令设置
*
* 输入    : @address:地址命令
*						@controlword:交替字节命令
*					  @dummyCycles:空指令命令周期数
*						@dataMode:数据命令收发模式
*						
* 返回值  : 无
*******************************************************************************/
void QSPI_Send_Control( uint16_t address,uint8_t controlword,uint32_t dummyCycles,uint32_t dataMode)
{
		QSPI_CommandTypeDef Cmdhandler;
		
		Cmdhandler.InstructionMode=QSPI_INSTRUCTION_NONE; 					// 指令命令 - NULL	
		Cmdhandler.Address=address; 																// 地址命令 - address
		Cmdhandler.AddressSize=QSPI_ADDRESS_16_BITS; 								// 地址命令长度 - 16位  
		Cmdhandler.AddressMode=QSPI_ADDRESS_1_LINE; 								// 地址命令发送模式 - 单线	
		Cmdhandler.AlternateBytes=controlword;											// 交替字节命令 - controlword
	  Cmdhandler.AlternateBytesSize=QSPI_ALTERNATE_BYTES_8_BITS;	// 交替字节长度 - 8位
		Cmdhandler.AlternateByteMode=QSPI_ALTERNATE_BYTES_1_LINE; 	// 交替字节发送模式 - 单线		
		Cmdhandler.DummyCycles=dummyCycles; 												// 空指令命令周期数 - dummyCycles
		Cmdhandler.DataMode=dataMode; 															// 数据命令收发模式 - dataMode
		Cmdhandler.DdrMode=QSPI_DDR_MODE_DISABLE; 									// 关闭 DDR 模式
		Cmdhandler.DdrHoldHalfCycle=QSPI_DDR_HHC_ANALOG_DELAY;      // 默认配置
		Cmdhandler.SIOOMode= QSPI_SIOO_INST_EVERY_CMD;              // 默认配置

		HAL_QSPI_Command(&hqspi,&Cmdhandler,HAL_QPSI_TIMEOUT_DEFAULT_VALUE);

}

/*******************************************************************************
* 函数名  : QSPI_Send_Control_IT
*
* 描述    : QSPI发送命令设置（中断方式）
*
* 输入    : @address:地址命令
*						@controlword:交替字节命令
*					  @dummyCycles:空指令命令周期数
*						@dataMode:数据命令收发模式
*						
* 返回值  : 无
*******************************************************************************/
void QSPI_Send_Control_IT(uint16_t address,uint8_t controlword,uint32_t dummyCycles,uint32_t dataMode)
{
		QSPI_CommandTypeDef Cmdhandler;
	
		Cmdhandler.InstructionMode=QSPI_INSTRUCTION_NONE; 					// 指令命令 - NULL	
		Cmdhandler.Address=address; 																// 地址命令 - address
		Cmdhandler.AddressSize=QSPI_ADDRESS_16_BITS; 								// 地址命令长度 - 16位  
		Cmdhandler.AddressMode=QSPI_ADDRESS_1_LINE; 								// 地址命令发送模式 - 单线	
		Cmdhandler.AlternateBytes=controlword;											// 交替字节命令 - controlword
	  Cmdhandler.AlternateBytesSize=QSPI_ALTERNATE_BYTES_8_BITS;	// 交替字节长度 - 8位
		Cmdhandler.AlternateByteMode=QSPI_ALTERNATE_BYTES_1_LINE; 	// 交替字节发送模式 - 单线		
		Cmdhandler.DummyCycles=dummyCycles; 												// 空指令命令周期数 - dummyCycles
		Cmdhandler.DataMode=dataMode; 															// 数据命令收发模式 - dataMode
		Cmdhandler.DdrMode=QSPI_DDR_MODE_DISABLE; 									// 关闭 DDR 模式
		Cmdhandler.DdrHoldHalfCycle=QSPI_DDR_HHC_ANALOG_DELAY;      // 默认配置
		Cmdhandler.SIOOMode= QSPI_SIOO_INST_EVERY_CMD;              // 默认配置
	
		HAL_QSPI_Command_IT(&hqspi,&Cmdhandler);
}

/*******************************************************************************
* 函数名  : QSPI_Receive
*
* 描述    : QSPI接收指定长度的数据
*
* 输入    : @buf: 接收数据缓冲区首地址
						@datalen: 传输数据长度
*
* 返回值  : 0 - 正常
						1 - 错误
*******************************************************************************/
uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen)
{
	hqspi.Instance->DLR=datalen-1; //配置数据长度
	
	if( HAL_QSPI_Receive(&hqspi,buf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE)== HAL_OK ) 
		return 0; //接收数据
	else 
		return 1;
}

/*******************************************************************************
* 函数名  : QSPI_Transmit
*
* 描述    : QSPI发送指定长度的数据
*
* 输入    : @buf: 发送数据缓冲区首地址
						@datalen: 传输数据长度
*
* 返回值  : 0 - 正常
						1 - 错误
*******************************************************************************/
uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen)
{
	hqspi.Instance->DLR=datalen-1; //配置数据长度
	
	if( HAL_QSPI_Transmit(&hqspi,buf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE)==HAL_OK ) 
		return 0; //发送数据
	else 
		return 1;
}

/*******************************************************************************
* 函数名  : QSPI_Transmit_IT
*
* 描述    : QSPI发送指定长度的数据（中断方式）
*
* 输入    : @buf: 发送数据缓冲区首地址
						@datalen: 传输数据长度
*
* 返回值  : 0 - 正常
						1 - 错误
*******************************************************************************/
uint8_t QSPI_Transmit_IT(uint8_t* buf,uint32_t datalen)
{
	hqspi.Instance->DLR=datalen-1; //配置数据长度
	
	if(HAL_QSPI_Transmit_IT(&hqspi,buf)==HAL_OK) 
		return 0; //发送数据
	else 
		return 1;
}
