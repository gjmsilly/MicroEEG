#ifndef  _W5500_SERVICE_H_
#define  _W5500_SERVICE_H_

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"

/*********************************************************************
 * TYPEDEFS
 */
/* 网络配置参数 */    		//8.26 地址对齐问题 debug
typedef struct
{
	uint8_t Gateway_IP[4];	//网关IP地址 
	uint8_t Sub_Mask[4];		//子网掩码 
	uint8_t IP_Addr[4];			//源IP地址，即本机IP地址
	uint8_t Phy_Addr[6];		//源MAC地址，即本机IP地址 	

}NETWORKParam_t;

/* Socket n配置参数 */
typedef struct
{
	uint16_t Sn_Port;				//Socket n端口号/源端口号
	
	//本机为TCP客户端模式时配置	
	uint16_t Sn_DPort;			//Socket n目的端口号 
	uint8_t  Sn_DIP[4];			//Socket n目的IP地址 
	
	//本机为UDP模式时配置
	uint16_t UDP_DPORT;			//Socket n目的端口号
	uint8_t  UDP_DIPR[4];		//Socket n目的IP地址
	uint8_t  _UDP_DIPR_[2]; //字节对齐

}SOCKETnParam_t;

/*********************************************************************
 * Macros
 */
#define TRUE								0x00
#define FALSE								0xff

// 端口状态
#define Sn_OPEN							0x00	//!< 端口已打开
#define Sn_LISTEN						0x01	//!< 端口正在监听
#define Sn_CLOSE						0x02	//!< 端口已关闭
#define TCP_RECV						0x03	//!< TCP端口接收一帧
#define TCP_SEND						0x04	//!< TCP端口发送完毕

// 通讯收发缓冲区参数
#define TCP_Rx_Buff_Size		16
#define TCP_Tx_Buff_Size		16
#define UDP_Tx_Buff_Size		1193		//!< 23+样本数 x（数据域头部9+通道数x3）字节

/***********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];			//TCP接收数据缓冲区 
extern uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];			//TCP发送数据缓冲区
extern uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];			//UDP发送数据缓冲区
extern uint8_t *pUDP_Tx_Buff;		
extern NETWORKParam_t net_param;
extern SOCKETnParam_t  sn_param[2];

/***********************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);

uint8_t TCPServer_Service(uint8_t sn , uint8_t Procesflag);
uint8_t UDP_Service(uint8_t sn);

#endif   // _W5500_SERVICE_H_
