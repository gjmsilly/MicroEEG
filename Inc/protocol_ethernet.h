#ifndef __PROTOCOL_ETHERNET_H__
#define __PROTOCOL_ETHERNET_H__

/*********************************************************************
 * INCLUDES
 */
#include <stddef.h>
#include <stdint.h>

/*********************************************************************
 * CONSTANTS
 */

// 控制通道（TCP端口）

// 上位机->设备 指令解析
const uint8_t TCP_Recv_FH = 0xAC;				//<! TCP接收帧头
const uint8_t TCP_Recv_FT = 0xCC;				//<! TCP接收帧尾

const uint8_t	DummyIns = 0x00;					//<! 空指令
const uint8_t	CAttr_Read = 0x01;				//<! 读一个普通属性
const uint8_t	CAttr_Write = 0x10;				//<! 写一个普通属性
const uint8_t	ChxAttr_Read = 0x02;			//<! 读一个通道属性
const uint8_t	ChxAttr_Write = 0x20;			//<! 写一个通道属性

// 设备->上位机 回复
const uint8_t TCP_Send_FH = 0xA2;				//<! TCP发送帧头
const uint8_t TCP_Send_FT = 0xC2;				//<! TCP发送帧尾

const uint8_t	TCP_SUCCESS = 0x00;				//<! 指令解析正常
const uint8_t	TCPERR_Attr_RO = 0x01;		//<! 指令错误：对只读属性写操作
const uint8_t	TCPERR_Attr_LEN = 0x02;		//<! 指令错误：写指令数据长度错误

/*********************************************************************
 * FUNCTIONS
 */
void ProtocolProcessFSMInit(void);
void ProtocolProcessFSM(void);

#endif  // __PROTOCOL_ETHERNET_H__
