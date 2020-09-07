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
#define TCP_Rx_Buff_Size		40
#define TCP_Tx_Buff_Size		1024
#define UDP_Tx_Buff_Size		1024

 /*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];			//TCP接收数据缓冲区 
extern uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];			//TCP发送数据缓冲区
extern uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];			//UDP发送数据缓冲区
extern uint8_t TCP_RPY_Size;
/**********************************************************************
 * FUNCTIONS
 */
void ProtocolProcessFSMInit(void);
void ProtocolProcessFSM(void);

#endif  // __PROTOCOL_ETHERNET_H__
