#ifndef __PROTOCOL_ETHERNET_H__
#define __PROTOCOL_ETHERNET_H__

/*********************************************************************
 * INCLUDES
 */
#include <stddef.h>
#include <stdint.h>

/*********************************************************************
 * Macros
 */
#define TCP_Rx_Buff_Size		16
#define TCP_Tx_Buff_Size		1024
#define UDP_Tx_Buff_Size		1024

/* 指令码 */
#define	DummyIns						0x00	//!< 空指令
#define	CAttr_Read					0x01	//!< 读一个普通属性
#define	CAttr_Write					0x10	//!< 写一个普通属性
#define	ChxAttr_Read				0x02	//!< 读一个通道属性
#define	ChxAttr_Write				0x20	//!< 写一个通道属性

/* 状态机运行状态 */
#define _FSM_CPL_						0x00	//!< 状态机运行完成
#define _FSM_ON_GOING_			0x01	//!< 状态机正在运行
#define _FSM_ERR_						0x02	//!< 状态机错误

 /*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];			//TCP接收数据缓冲区 
extern uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];			//TCP发送数据缓冲区
extern uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];			//UDP发送数据缓冲区

/**********************************************************************
 * FUNCTIONS
 */
void ProtocolProcessFSMInit(void);
uint8_t ProtocolProcessFSM(void);

#endif  // __PROTOCOL_ETHERNET_H__
