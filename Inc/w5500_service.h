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
#define UDP_RECV						0x05	//!< UDP端口接收一帧
#define UDP_SEND						0x06	//!< UDP端口发送完毕

// 通讯收发缓冲区参数
#define TCP_Rx_Buff_Size			16
#define TCP_Tx_Buff_Size			16
#define	UDP_TrgRx_Buff_Size		16
#define	UDP_TrgTx_Buff_Size		16

#ifdef Dev_Ch32 
#define UDPD_Tx_Buff_Size		1173		//!< 数据帧头部23 + 样本数10 x（数据域头部7 + (本组通道状态3 + 八通道8 x 每通道量化字节数3）x 通道组数4)字节
#endif
#ifdef Dev_Ch24 
#define UDPD_Tx_Buff_Size		903			//!< 数据帧头部23 + 样本数10 x（数据域头部7 + (本组通道状态3 + 八通道8 x 每通道量化字节数3）x 通道组数3)字节
#endif
#ifdef Dev_Ch16 
#define UDPD_Tx_Buff_Size		633			//!< 数据帧头部23 + 样本数10 x（数据域头部7 + (本组通道状态3 + 八通道8 x 每通道量化字节数3）x 通道组数2)字节
#endif
#ifdef Dev_Ch8 
#define UDPD_Tx_Buff_Size		363			//!< 数据帧头部23 + 样本数10 x（数据域头部7 + (本组通道状态3 + 八通道8 x 每通道量化字节数3）x 通道组数1)字节
#endif

/***********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];				//TCP接收缓冲区 
extern uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];				//TCP发送缓冲区
extern uint8_t UDP_DTx_Buff[UDPD_Tx_Buff_Size];			//UDP数据发送缓冲区
extern uint8_t UDP_TrgRx_Buff[UDP_TrgRx_Buff_Size];	//UDP事件发送缓冲区
extern uint8_t UDP_TrgTx_Buff[UDP_TrgTx_Buff_Size];	//UDP事件接收缓冲区
extern uint8_t *pUDP_DTx_Buff;		
extern NETWORKParam_t net_param,*Pnet_param;
extern SOCKETnParam_t  sn_param[3],*Psn_param;

/***********************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);

uint8_t TCPServer_Service(uint8_t sn , uint16_t Procesflag);
uint8_t UDP_Service(uint8_t sn, uint16_t Procesflag);

#endif   // _W5500_SERVICE_H_
